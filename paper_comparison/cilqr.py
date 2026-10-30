"""Independent augmented-Lagrangian constrained iLQR, not onboard C++ timing."""

from time import perf_counter

import casadi as ca
import numpy as np


class CilqrPlanner:
    """Projected controls, GN cost/penalty Hessian, Riccati recursion, Armijo.

    RK4 replaces the reference branch's Euler integration for the common test.
    PHR inequality multipliers replace its exponential barriers. All such
    differences are intentional and recorded in the experiment documentation.
    """

    def __init__(self, model, max_inner=40, max_outer=7):
        self.model, self.max_inner, self.max_outer = model, max_inner, max_outer
        m = model
        lam, rho = ca.MX.sym("lambda", m.g.numel()), ca.MX.sym("rho")
        z = ca.vertcat(m.x, m.u)
        jr = ca.jacobian(m.residual, z)
        jg = ca.jacobian(m.g, z)
        active = ca.fmax(0, lam + rho * m.g)
        objective = m.cost + (ca.sumsqr(active) - ca.sumsqr(lam)) / (2 * rho)
        gradient = ca.gradient(objective, z)
        hessian = (
            2 * m.dt * jr.T @ ca.diag(ca.DM(m.settings.weights)) @ jr
            + jg.T @ ca.diag(rho * (lam + rho * m.g > 0)) @ jg
        )
        # Expand symbolic operations into compiled C; linear algebra/Riccati stays Python.
        jit = {"jit": True, "compiler": "shell", "jit_options": {"flags": ["-O2"]}}
        self.derivatives = ca.Function(
            "derivs", [m.x, m.u, m.p, lam, rho], [gradient, hessian], jit
        )
        self.value = ca.Function("value", [m.x, m.u, m.p, lam, rho], [objective], jit)
        te = m.terminal_cost
        terminal_g = ca.substitute(m.g, m.u, ca.DM.zeros(2))
        terminal_active = ca.fmax(0, lam + rho * terminal_g)
        te += (ca.sumsqr(terminal_active) - ca.sumsqr(lam)) / (2 * rho)
        terminal_jg = ca.jacobian(terminal_g, m.x)
        self.terminal = ca.Function(
            "terminal",
            [m.x, m.p, lam, rho],
            [
                te,
                ca.gradient(te, m.x),
                2
                * ca.jacobian(m.terminal_residual, m.x).T
                @ ca.diag(ca.DM(m.settings.terminal_weights))
                @ ca.jacobian(m.terminal_residual, m.x)
                + terminal_jg.T
                @ ca.diag(rho * (lam + rho * terminal_g > 0))
                @ terminal_jg,
            ],
            jit,
        )
        self.U, self.X = np.zeros((m.N, 2)), None
        self.build_seconds = 0.0

    def _rollout(self, initial, controls):
        states = [initial.copy()]
        for control in controls:
            state = np.asarray(self.model.step(states[-1], control)).ravel()
            if (
                not np.isfinite(state).all()
                or abs(state[2]) > 1.4
                or abs(state[1]) > 30
            ):
                return None
            states.append(state)
        return np.array(states)

    def _value(self, X, U, P, multipliers, rho):
        value = sum(
            float(self.value(x, u, p, l, rho))
            for x, u, p, l in zip(X[:-1], U, P, multipliers)
        )
        return value + float(self.terminal(X[-1], P[-1], multipliers[-1], rho)[0])

    def solve(self, initial, parameters):
        start = perf_counter()
        m = self.model
        U = np.clip(self.U.copy(), m.ul, m.uu)
        X = self._rollout(initial, U)
        if X is None:
            U = np.zeros_like(U)
            X = self._rollout(initial, U)
        multipliers = np.zeros((m.N + 1, m.g.numel()))
        rho, regularization, iterations = 100.0, 1e-4, 0
        line_evals = 0
        for _ in range(self.max_outer):
            current = self._value(X, U, parameters, multipliers, rho)
            for _ in range(self.max_inner):
                iterations += 1
                _, vx, vxx = self.terminal(X[-1], parameters[-1], multipliers[-1], rho)
                vx, vxx = np.asarray(vx).ravel(), np.asarray(vxx)
                feedforward, feedback = np.zeros_like(U), np.zeros((m.N, 2, m.nx))
                descent, valid = 0.0, True
                for k in reversed(range(m.N)):
                    grad, hess = self.derivatives(
                        X[k], U[k], parameters[k], multipliers[k], rho
                    )
                    grad, hess = np.asarray(grad).ravel(), np.asarray(hess)
                    A, B = (np.asarray(z) for z in m.linearize(X[k], U[k]))
                    qx, qu = grad[: m.nx] + A.T @ vx, grad[m.nx :] + B.T @ vx
                    qxx = hess[: m.nx, : m.nx] + A.T @ vxx @ A
                    quu = (
                        hess[m.nx :, m.nx :]
                        + B.T @ vxx @ B
                        + regularization * np.eye(2)
                    )
                    qux = hess[m.nx :, : m.nx] + B.T @ vxx @ A
                    try:
                        np.linalg.cholesky(quu)
                        d, K = -np.linalg.solve(quu, qu), -np.linalg.solve(quu, qux)
                    except np.linalg.LinAlgError:
                        valid = False
                        break
                    feedforward[k], feedback[k] = d, K
                    descent += float(qu @ d)
                    vx = qx + K.T @ quu @ d + K.T @ qu + qux.T @ d
                    vxx = qxx + K.T @ quu @ K + K.T @ qux + qux.T @ K
                    vxx = (vxx + vxx.T) / 2
                if not valid:
                    regularization *= 10
                    continue
                accepted, improvement = False, 0.0
                for alpha in [
                    1,
                    0.5,
                    0.25,
                    0.125,
                    0.0625,
                    0.03125,
                    0.015625,
                    0.0078125,
                ]:
                    line_evals += 1
                    trial_x, trial_u = [initial.copy()], []
                    for k in range(m.N):
                        uk = np.clip(
                            U[k]
                            + alpha * feedforward[k]
                            + feedback[k] @ (trial_x[-1] - X[k]),
                            m.ul,
                            m.uu,
                        )
                        xk = np.asarray(m.step(trial_x[-1], uk)).ravel()
                        if (
                            not np.isfinite(xk).all()
                            or abs(xk[2]) > 1.4
                            or abs(xk[1]) > 30
                        ):
                            break
                        trial_u.append(uk)
                        trial_x.append(xk)
                    if len(trial_u) != m.N:
                        continue
                    trial_x, trial_u = np.array(trial_x), np.array(trial_u)
                    value = self._value(trial_x, trial_u, parameters, multipliers, rho)
                    if np.isfinite(value) and value <= current + 1e-4 * alpha * descent:
                        improvement = current - value
                        X, U, current, accepted = trial_x, trial_u, value, True
                        regularization = max(1e-6, regularization / 3)
                        break
                if not accepted:
                    regularization *= 10
                    if regularization > 1e8:
                        break
                elif improvement < 1e-9 * max(1, abs(current)):
                    break
            violations = np.array(
                [
                    np.asarray(
                        m.gfun(X[k], U[k] if k < m.N else np.zeros(2), parameters[k])
                    ).ravel()
                    for k in range(m.N + 1)
                ]
            )
            if np.max(violations) <= 1e-4:
                break
            multipliers = np.maximum(0, multipliers + rho * violations)
            rho *= 10
        self.X, self.U = X, U
        diagnostics = m.violation(X, U, parameters, initial)
        # Includes Python recursion + generated-C callbacks, not native-only time.
        return dict(
            status=0 if diagnostics["max_violation"] <= 1e-4 else 2,
            native_solver_ms=None,
            solve_wall_ms=(perf_counter() - start) * 1000,
            iterations=iterations,
            line_search_evaluations=line_evals,
            **diagnostics,
        )

    def shift(self):
        self.U = np.vstack([self.U[1:], self.U[-1]])
