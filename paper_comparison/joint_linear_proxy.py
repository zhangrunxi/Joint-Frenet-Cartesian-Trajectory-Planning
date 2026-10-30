"""Xing-inspired sequential-QP proxy, NOT a verified reproduction of Xing [26].

Only the published abstract and Reiter's discussion were accessible. The
Cartesian dynamics / algebraic frame connection / iterated QP architecture is
retained; state selection, geometry, costs and globalization below are ours.
"""

from time import perf_counter

import casadi as ca
import numpy as np
import osqp
from scipy import sparse

from paper_comparison.model import approximate_road_offsets, bus_vehicle, gym_pose
from reiter2023.geometry import covering_circles, ellipse_constraint


class JointLinearProxy:
    nx, nu = 8, 2

    def __init__(self, road, settings, obstacle="CIRCLES", max_iter=150):
        self.road, self.settings, self.obstacle = road, settings, obstacle
        self.N, self.dt, self.max_iter = settings.intervals, settings.dt, max_iter
        self.vehicle = v = bus_vehicle()
        x, u, p = ca.MX.sym("x", 8), ca.MX.sym("u", 2), ca.MX.sym("p", 6)
        # Frenet pose is algebraic: never integrated as an extra ODE here.
        cart = ca.vertcat(x[3], x[4], x[5:8])
        z = ca.MX.sym("z", 5)
        rhs = ca.vertcat(
            u[0],
            u[1],
            z[0] * ca.cos(z[4]),
            z[0] * ca.sin(z[4]),
            z[0] * ca.tan(z[1]) / v.length,
        )
        ode = ca.Function("proxy_ode", [z, u], [rhs])
        h = ca.MX.sym("dt")
        k1 = ode(z, u)
        k2 = ode(z + h * k1 / 2, u)
        k3 = ode(z + h * k2 / 2, u)
        k4 = ode(z + h * k3, u)
        step = z + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.cart_step = ca.Function("cart_step", [z, u, h], [step])
        self.frame = ca.Function("frame", [x], [x[5:8] - road.pose(x[0], x[1], x[2])])
        y = ca.MX.sym("next", 8)
        dyn = y[3:8] - self.cart_step(cart, u, self.dt)
        self.dynamics = ca.Function(
            "dyn", [x, u, y], [dyn, ca.jacobian(dyn, x), ca.jacobian(dyn, u)]
        )
        self.connection = ca.Function(
            "connection", [x], [self.frame(x), ca.jacobian(self.frame(x), x)]
        )
        if obstacle == "CIRCLES":
            aa, ra = covering_circles(gym_pose(x[5:8]), v, 3)
            bb, rb = covering_circles(gym_pose(p[:3]), v, 3)
            obs = ca.vertcat(
                *[
                    ca.sqrt(ca.sumsqr(a - b) + 1e-16)
                    - ra
                    - rb
                    - settings.obstacle_margin
                    for a in aa
                    for b in bb
                ]
            )
        elif obstacle == "ELIPSE":
            obs = ellipse_constraint(
                gym_pose(x[5:8]), gym_pose(p[:3]), v, v, settings.obstacle_margin
            )
        else:
            raise ValueError(obstacle)
        self.h = ca.vertcat(
            x[3] ** 2 * ca.tan(x[4]) / v.length,
            1 - x[1] * road.kappa,
            approximate_road_offsets(x, road),
            obs,
        )
        width = road.half_width - settings.road_margin
        self.hl = np.r_[-settings.max_lat_acc, 0.1, [-width] * 4, np.zeros(obs.numel())]
        self.hu = np.r_[
            settings.max_lat_acc, 1e8, [width] * 4, np.full(obs.numel(), 1e8)
        ]
        self.constraints = ca.Function(
            "constraints", [x, p], [self.h, ca.jacobian(self.h, x)]
        )
        self.xl = np.r_[
            -np.inf, -np.inf, -0.65, 0, -v.maximum_steering_angle, [-np.inf] * 3
        ]
        self.xu = np.r_[
            np.inf,
            np.inf,
            0.65,
            settings.max_speed,
            v.maximum_steering_angle,
            [np.inf] * 3,
        ]
        self.ul = np.array([settings.min_acc, -v.maximum_steering_rate])
        self.uu = np.array([settings.max_acc, v.maximum_steering_rate])
        self.wx = np.r_[1, 500, 1000, 1000, 10000, 0, 0, 0] * self.dt
        self.wt = np.r_[10, 500, 1000, 1000, 10000, 0, 0, 0]
        # Same physical force cost as the matched Reiter bus (F=m*a).
        self.wu = np.array([0.001 * v.mass**2, 2e6]) * self.dt
        self.X = self.U = None

    def initial(self):
        return self.augment(np.array([0, 0, 0, 12, np.arctan(4.32 * self.road.kappa)]))

    def augment(self, state):
        return np.r_[state[:5], self.road.numeric_pose(state)]

    def parameters(self, time, initial_s):
        return np.array(
            [
                np.r_[
                    self.road.numeric_pose([35 + 8 * t, 0, 0]),
                    initial_s + k * self.dt * self.settings.speed,
                    self.settings.lateral,
                    self.settings.speed,
                ]
                for k, t in enumerate(time + np.arange(self.N + 1) * self.dt)
            ]
        )

    def integrate(self, x, u, dt):
        cart = np.asarray(self.cart_step(x[3:8], u, dt)).ravel()
        px, py, heading = cart[2:5]
        k = self.road.kappa
        if abs(k) < 1e-12:
            s, n = px, py
        else:
            angle = np.arctan2(k * px, 1 - k * py)
            angle += 2 * np.pi * np.round((k * x[0] - angle) / (2 * np.pi))
            s = angle / k
            n = (1 - np.hypot(k * px, 1 - k * py)) / k
        return np.r_[s, n, heading - k * s, cart]

    def shift(self):
        self.X = np.vstack(
            [self.X[1:], self.integrate(self.X[-1], self.U[-1], self.dt)]
        )
        self.U = np.vstack([self.U[1:], self.U[-1]])

    def _values(self, X, U, P, initial):
        errors = [np.max(np.abs(X[0] - initial))]
        equality, violation = 0.0, 0.0
        objective = 0.0
        for k in range(self.N + 1):
            eq = np.asarray(self.frame(X[k])).ravel()
            equality += np.sum(np.abs(eq))
            errors.append(np.max(np.abs(eq)))
            hh = np.asarray(self.constraints(X[k], P[k])[0]).ravel()
            vv = np.r_[self.hl - hh, hh - self.hu, self.xl - X[k], X[k] - self.xu]
            violation += np.maximum(vv, 0).sum()
            errors.append(max(0.0, np.max(vv)))
            ref = np.r_[P[k, 3], P[k, 4], 0, P[k, 5], 0, 0, 0, 0]
            objective += np.dot(self.wt if k == self.N else self.wx, (X[k] - ref) ** 2)
            if k < self.N:
                e = np.asarray(self.dynamics(X[k], U[k], X[k + 1])[0]).ravel()
                equality += np.abs(e).sum()
                errors.append(np.max(np.abs(e)))
                vv = np.r_[self.ul - U[k], U[k] - self.uu]
                violation += np.maximum(vv, 0).sum()
                errors.append(max(0.0, np.max(vv)))
                objective += np.dot(self.wu, U[k] ** 2)
        return max(errors), objective, equality + violation

    def solve(self, initial, P):
        start = perf_counter()
        if self.X is None:
            self.U = np.zeros((self.N, 2))
            self.X = np.array([initial] * (self.N + 1))
            for k in range(self.N):
                self.X[k + 1] = self.integrate(self.X[k], self.U[k], self.dt)
        self.X[0] = initial
        nz = 8 * (self.N + 1) + 2 * self.N
        sx = lambda k: slice(8 * k, 8 * (k + 1))
        su = lambda k: slice(8 * (self.N + 1) + 2 * k, 8 * (self.N + 1) + 2 * (k + 1))
        weights = np.r_[np.tile(self.wx, self.N), self.wt, np.tile(self.wu, self.N)]
        objective_scale = max(weights)
        H = sparse.diags((2 * weights + 1e-8) / objective_scale, format="csc")
        qp_seconds, qp_status, penalty = 0.0, "not_run", 1e4
        converged = False
        for iteration in range(self.max_iter):
            q = np.zeros(nz)
            # Allocate sparse stage rows once per nonlinear iteration.
            nr = 8 + 3 * (self.N + 1) + 5 * self.N + self.h.numel() * (self.N + 1) + nz
            A = sparse.lil_matrix((nr, nz))
            lo, hi = np.full(nr, -np.inf), np.full(nr, np.inf)
            row = 0
            A[row : row + 8, sx(0)] = np.eye(8)
            lo[row : row + 8] = hi[row : row + 8] = initial - self.X[0]
            row += 8
            for k in range(self.N + 1):
                ref = np.r_[P[k, 3], P[k, 4], 0, P[k, 5], 0, 0, 0, 0]
                q[sx(k)] = 2 * (self.wt if k == self.N else self.wx) * (self.X[k] - ref)
                f, J = self.connection(self.X[k])
                A[row : row + 3, sx(k)] = np.asarray(J)
                lo[row : row + 3] = hi[row : row + 3] = -np.asarray(f).ravel()
                row += 3
                if k < self.N:
                    f, J, B = self.dynamics(self.X[k], self.U[k], self.X[k + 1])
                    A[row : row + 5, sx(k)] = np.asarray(J)
                    A[row : row + 5, su(k)] = np.asarray(B)
                    A[row : row + 5, 8 * (k + 1) + 3 : 8 * (k + 2)] = np.eye(5)
                    lo[row : row + 5] = hi[row : row + 5] = -np.asarray(f).ravel()
                    row += 5
                    q[su(k)] = 2 * self.wu * self.U[k]
                f, J = self.constraints(self.X[k], P[k])
                nh = self.h.numel()
                A[row : row + nh, sx(k)] = np.asarray(J)
                lo[row : row + nh], hi[row : row + nh] = (
                    self.hl - np.asarray(f).ravel(),
                    self.hu - np.asarray(f).ravel(),
                )
                row += nh
            current = np.r_[self.X.ravel(), self.U.ravel()]
            A[row:, :] = sparse.eye(nz)
            lo[row:] = (
                np.r_[np.tile(self.xl, self.N + 1), np.tile(self.ul, self.N)] - current
            )
            hi[row:] = (
                np.r_[np.tile(self.xu, self.N + 1), np.tile(self.uu, self.N)] - current
            )
            solver = osqp.OSQP()
            solver.setup(
                P=H,
                q=q / objective_scale,
                A=A.tocsc(),
                l=lo,
                u=hi,
                verbose=False,
                eps_abs=1e-6,
                eps_rel=1e-6,
                max_iter=100000,
                polishing=True,
            )
            result = solver.solve(raise_error=False)
            qp_seconds += result.info.run_time
            qp_status = result.info.status
            if result.info.status_val not in (1, 2):
                break
            penalty = max(penalty, 2 * objective_scale * np.max(np.abs(result.y)))
            du = result.x[8 * (self.N + 1) :].reshape(-1, 2)
            violation, cost, infeas = self._values(self.X, self.U, P, initial)
            merit = cost + penalty * infeas
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125):
                UU = self.U + scale * du
                # Feasible-rollout globalization is our adaptation, not Xing's.
                XX = np.empty_like(self.X)
                XX[0] = initial
                for k in range(self.N):
                    XX[k + 1] = self.integrate(XX[k], UU[k], self.dt)
                v, c, e = self._values(XX, UU, P, initial)
                if c + penalty * e <= merit + 1e-8:
                    self.X, self.U = XX, UU
                    accepted = True
                    if v < 1e-5 and (
                        np.max(np.abs(scale * du)) < 1e-4
                        or abs(c - cost) < 1e-9 * max(1.0, abs(cost))
                    ):
                        converged = True
                    break
            if converged or not accepted:
                break
        violation, _, _ = self._values(self.X, self.U, P, initial)
        return {
            "status": 0 if converged and violation <= 1e-4 else 2,
            "max_violation": float(violation),
            "iterations": iteration + 1,
            "coordinate_error": float(
                max(np.max(np.abs(self.frame(x))) for x in self.X)
            ),
            "dynamics_defect": float(
                max(
                    np.max(
                        np.abs(self.dynamics(self.X[k], self.U[k], self.X[k + 1])[0])
                    )
                    for k in range(self.N)
                )
            ),
            "solve_wall_ms": (perf_counter() - start) * 1000,
            "native_solver_ms": None,
            "qp_kernel_sum_ms": qp_seconds * 1000,
            "qp_status": qp_status,
            "proxy_not_verified_xing": True,
        }
