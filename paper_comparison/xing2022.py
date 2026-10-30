"""Equation-based Xing et al., RA-L 2022, DOI 10.1109/LRA.2022.3194330.

Original-order states [V,X,Y,psi,S,D,phi], controls [delta,a].
Equations 6/7, 17/19, 14 and 20 are implemented as iterated linear QPs.
Bus shape inflation, upstream keep-left/yield decisions, weights, numerical
tolerances and capability limits are documented experimental adaptations.
"""

from dataclasses import asdict, dataclass
from time import perf_counter

import casadi as ca
import numpy as np
import osqp
from scipy import sparse

from paper_comparison.evaluation import rectangle_vertices
from paper_comparison.model import bus_vehicle


@dataclass(frozen=True)
class XingOptions:
    max_iterations: int = 50
    apriori_tolerance_m: float = 0.2
    feasibility_tolerance: float = 1e-4
    # Eq. 20 weights are not specified numerically in the paper.
    speed_weight: float = 1000.0
    progress_weight: float = 1.0
    lateral_weight: float = 500.0
    heading_weight: float = 1000.0
    steering_rate_weight: float = 2e6
    jerk_weight: float = 1.0
    decision: str = "left"
    enforce_lateral_acceleration: bool = True
    qp_tolerance: float = 1e-7
    obstacle_start: float = 35.0
    obstacle_speed: float = 8.0
    initial_speed: float = 12.0
    # "point": Eq. 12 static bounds on the reference point, shrunk by half width.
    road_bound: str = "contour"


class _Rows:
    def __init__(self, columns):
        self.columns, self.row = columns, 0
        self.rr, self.cc, self.vv, self.lo, self.hi = [], [], [], [], []

    def add(self, blocks, lower, upper):
        lower, upper = np.atleast_1d(lower), np.atleast_1d(upper)
        for start, block in blocks:
            block = np.atleast_2d(block)
            r, c = np.nonzero(block)
            self.rr.extend(self.row + r)
            self.cc.extend(start + c)
            self.vv.extend(block[r, c])
        self.lo.extend(lower)
        self.hi.extend(upper)
        self.row += len(lower)

    def matrix(self):
        return sparse.csc_matrix(
            (self.vv, (self.rr, self.cc)), shape=(self.row, self.columns)
        )


class XingPlanner:
    nx, nu = 7, 2
    state_order = ("V", "X", "Y", "psi", "S", "D", "phi")

    def __init__(self, road, settings, options=None):
        if road.heading0 != 0:
            raise ValueError("This experimental adapter requires heading0=0")
        self.road, self.settings = road, settings
        self.options = options or XingOptions()
        if self.options.decision not in ("left", "yield"):
            raise ValueError("Upstream decision must be left or yield")
        self.N, self.dt = settings.intervals, settings.dt
        self.vehicle = v = bus_vehicle()
        self.X = self.U = None
        self.previous_input = np.array([np.arctan(v.length * road.kappa), 0.0])
        self.ul = np.array([-v.maximum_steering_angle, settings.min_acc])
        self.uu = np.array([v.maximum_steering_angle, settings.max_acc])
        self.du = self.dt * np.array([v.maximum_steering_rate, settings.max_jerk])
        self._functions()
        self._objective()

    def _functions(self):
        z, u, h = ca.SX.sym("z", 4), ca.SX.sym("u", 2), ca.SX.sym("h")
        distance = h * z[0] + 0.5 * h**2 * u[1]
        turn = ca.tan(u[0]) * distance / self.vehicle.length
        half = turn / 2
        # Analytic continuation of Eq. 6 at delta=0; avoids 0/0 and bad Jacobians.
        sinc = ca.if_else(
            ca.fabs(half) < 1e-4,
            1 - half**2 / 6 + half**4 / 120 - half**6 / 5040,
            ca.sin(half) / ca.if_else(ca.fabs(half) < 1e-4, 1, half),
        )
        step = ca.vertcat(
            z[0] + h * u[1],
            z[1] + distance * sinc * ca.cos(z[3] + half),
            z[2] + distance * sinc * ca.sin(z[3] + half),
            z[3] + turn,
        )
        self.transition = ca.Function(
            "xing_exact_transition",
            [z, u, h],
            [step, ca.jacobian(step, z), ca.jacobian(step, u)],
        )
        alat = z[0] ** 2 * ca.tan(u[0]) / self.vehicle.length
        self.lateral_acceleration = ca.Function(
            "xing_alat", [z, u], [alat, ca.jacobian(alat, z), ca.jacobian(alat, u)]
        )
        alat_end = (z[0] + self.dt * u[1]) ** 2 * ca.tan(u[0]) / self.vehicle.length
        self.lateral_acceleration_end = ca.Function(
            "xing_alat_end",
            [z, u],
            [alat_end, ca.jacobian(alat_end, z), ca.jacobian(alat_end, u)],
        )

    def project(self, z, near_s=0.0):
        """Exact g(pc) for the shared straight/circular road, with branch unwrap."""
        _, x, y, psi = z
        k = self.road.kappa
        if abs(k) < 1e-12:
            s, d = x, y
        else:
            theta = np.arctan2(k * x, 1 - k * y)
            theta += 2 * np.pi * np.round((k * near_s - theta) / (2 * np.pi))
            s = theta / k
            d = (1 - np.hypot(k * x, 1 - k * y)) / k
        return np.array([s, d, psi - k * s])

    def connection(self, z, near_s=0.0):
        """Equations 17/19; phi=psi-k*S supplies the omitted heading row."""
        pf = self.project(z, near_s)
        k, s, d = self.road.kappa, pf[0], pf[1]
        heading = k * s
        G = np.zeros((3, 4))
        G[0, 1:3] = np.array([np.cos(heading), np.sin(heading)]) / (1 - k * d)
        G[1, 1:3] = [-np.sin(heading), np.cos(heading)]
        G[2] = -k * G[0]
        G[2, 3] = 1
        return pf, G

    def initial(self):
        return np.r_[
            self.options.initial_speed,
            self.road.numeric_pose([0, 0, 0]),
            0.0,
            0.0,
            0.0,
        ]

    def integrate(self, state, control, dt):
        pc = np.asarray(self.transition(state[:4], control, dt)[0]).ravel()
        return np.r_[pc, self.project(pc, state[4])]

    @staticmethod
    def canonical(state):
        return np.asarray(state)[[4, 5, 6, 0, 1, 2, 3]]

    def _extents(self, state):
        """Frozen-apriori full bus footprint in Frenet (bus adaptation to Fig. 5).

        Include nearest edge points on circles so the lateral extrema cover the
        whole rectangle, rather than assuming all extrema occur at corners.
        """
        vertices = rectangle_vertices(state[1:4])
        points = list(vertices)
        k = self.road.kappa
        if k:
            center = np.array([0.0, 1 / k])
            for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
                t = np.clip(np.dot(center - a, b - a) / np.dot(b - a, b - a), 0, 1)
                points.append(a + t * (b - a))
        pf = np.array([self.project([0, *p, state[3]], state[4])[:2] for p in points])
        relative = pf - state[4:6]
        return relative.min(axis=0), relative.max(axis=0)

    def corridor(self, X, time):
        """Equations 12--15, with explicit keep-left / yield upstream decision.

        Rectangular obstacle subtraction is inflated by the frozen ego footprint.
        Blocked keep-left corridors use the yielding wall from Eq. 14b; no hidden
        policy rescue or alternative homotopy search is performed.
        """
        bounds = []
        for j, x in enumerate(X):
            ego_lo, ego_hi = self._extents(x)
            o = self.options
            obs_s = o.obstacle_start + o.obstacle_speed * (time + j * self.dt)
            obstacle = np.r_[
                o.obstacle_speed, self.road.numeric_pose([obs_s, 0, 0]), obs_s, 0.0, 0.0
            ]
            obs_lo, obs_hi = self._extents(obstacle)
            road_lo, road_hi = (
                (-1.15, 1.15) if o.road_bound == "point" else (ego_lo[1], ego_hi[1])
            )
            dlo = -self.road.half_width + self.settings.road_margin - road_lo
            dhi = self.road.half_width - self.settings.road_margin - road_hi
            lateral_wall = obs_hi[1] - ego_lo[1] + self.settings.obstacle_margin
            s_lower = obs_s + obs_lo[0] - ego_hi[0] - self.settings.obstacle_margin
            s_upper = obs_s + obs_hi[0] - ego_lo[0] + self.settings.obstacle_margin
            blocked = lateral_wall > dhi
            yield_wall = self.options.decision == "yield" or blocked
            slo, shi = -np.inf, s_lower if yield_wall else np.inf
            if not yield_wall and s_lower <= x[4] <= s_upper:
                dlo = max(dlo, lateral_wall)
            bounds.append([slo, shi, dlo, dhi])
        return np.asarray(bounds)

    def _objective(self):
        o, N, dt = self.options, self.N, self.dt
        self.w = dt * np.array(
            [
                o.speed_weight,
                0,
                0,
                0,
                o.progress_weight,
                o.lateral_weight,
                o.heading_weight,
            ]
        )
        # Sum ||delta_u||_R: choose R so identical increments penalize rates over dt.
        self.r = np.array([o.steering_rate_weight, o.jerk_weight]) / dt
        self.nz = 7 * (N + 1) + 2 * N
        self.uoffset = 7 * (N + 1)
        diagonal = np.r_[np.zeros(7), np.tile(self.w, N), np.zeros(2 * N)]
        D = sparse.diags([np.ones(N), -np.ones(N - 1)], [0, -1], format="csc")
        self.D = sparse.kron(D, sparse.eye(2), format="csc")
        H = sparse.diags(2 * diagonal + 1e-10, format="csc")
        hu = 2 * self.D.T @ sparse.diags(np.tile(self.r, N)) @ self.D
        H += sparse.block_diag([sparse.csc_matrix((self.uoffset, self.uoffset)), hu])
        self.objective_scale = max(self.r)
        self.H = sparse.triu(H / self.objective_scale, format="csc")

    def _qp(self, initial, time):
        rows = _Rows(self.nz)
        q = np.zeros(self.nz)
        rows.add([(0, np.eye(7))], initial, initial)
        corridor = self.corridor(self.X, time)
        for j in range(self.N + 1):
            x = self.X[j]
            start = 7 * j
            if j:
                ref = np.array(
                    [
                        self.settings.speed,
                        0,
                        0,
                        0,
                        initial[4] + j * self.dt * self.settings.speed,
                        self.settings.lateral,
                        0,
                    ]
                )
                q[start : start + 7] = -2 * self.w * ref
            pf, G = self.connection(x[:4], x[4])
            rows.add([(start, np.c_[-G, np.eye(3)])], pf - G @ x[:4], pf - G @ x[:4])
            sel = np.eye(7)[[0, 4, 5, 6]]
            slo, shi, dlo, dhi = corridor[j]
            rows.add(
                [(start, sel)],
                [0, slo, dlo, -0.65],
                [self.settings.max_speed, shi, dhi, 0.65],
            )
            if j == self.N:
                continue
            u, us = self.U[j], self.uoffset + 2 * j
            h, A, B = [np.asarray(v) for v in self.transition(x[:4], u, self.dt)]
            rhs = h.ravel() - A @ x[:4] - B @ u
            rows.add(
                [
                    (start, np.c_[-A, np.zeros((4, 3))]),
                    (start + 7, np.eye(7)[:4]),
                    (us, -B),
                ],
                rhs,
                rhs,
            )
            rows.add([(us, np.eye(2))], self.ul, self.uu)
            blocks = [(us, np.eye(2))]
            offset = self.previous_input if j == 0 else np.zeros(2)
            if j:
                blocks.append((us - 2, -np.eye(2)))
            rows.add(blocks, offset - self.du, offset + self.du)
            if self.options.enforce_lateral_acceleration:
                # Constant a and delta make |a_lat| maximal at an endpoint.
                for function in (
                    self.lateral_acceleration,
                    self.lateral_acceleration_end,
                ):
                    value, J, K = [np.asarray(v) for v in function(x[:4], u)]
                    offset = float(value.item() - J.ravel() @ x[:4] - K.ravel() @ u)
                    rows.add(
                        [(start, np.c_[J, np.zeros((1, 3))]), (us, K)],
                        [-self.settings.max_lat_acc - offset],
                        [self.settings.max_lat_acc - offset],
                    )
        q[self.uoffset : self.uoffset + 2] -= 2 * self.r * self.previous_input
        return rows, q / self.objective_scale

    def residuals(self, X, U, initial, time):
        if not all(np.isfinite(v).all() for v in (X, U, initial, self.previous_input)):
            return float("inf"), float("inf"), float("inf")
        defects, connections = [], []
        maximum = float(np.max(np.abs(X[0] - initial)))
        bounds = self.corridor(X, time)
        for j, x in enumerate(X):
            connections.append(np.max(np.abs(x[4:] - self.project(x[:4], x[4]))))
            slo, shi, dlo, dhi = bounds[j]
            maximum = max(
                maximum,
                -x[0],
                x[0] - self.settings.max_speed,
                slo - x[4],
                x[4] - shi,
                dlo - x[5],
                x[5] - dhi,
                abs(x[6]) - 0.65,
            )
            if j == self.N:
                continue
            u = U[j]
            predicted = np.asarray(self.transition(x[:4], u, self.dt)[0]).ravel()
            defects.append(np.max(np.abs(X[j + 1, :4] - predicted)))
            previous = self.previous_input if j == 0 else U[j - 1]
            maximum = max(
                maximum,
                np.max(self.ul - u),
                np.max(u - self.uu),
                np.max(np.abs(u - previous) - self.du),
            )
            if self.options.enforce_lateral_acceleration:
                for function in (
                    self.lateral_acceleration,
                    self.lateral_acceleration_end,
                ):
                    alat = float(function(x[:4], u)[0])
                    maximum = max(maximum, abs(alat) - self.settings.max_lat_acc)
        coordinate, defect = float(max(connections)), float(max(defects))
        return max(0.0, maximum, coordinate, defect), coordinate, defect

    def solve(self, initial, time):
        if not np.isfinite(initial).all() or not np.isfinite(time):
            raise ValueError("Initial state and time must be finite")
        start = perf_counter()
        if self.X is None:
            self.U = np.tile(self.previous_input, (self.N, 1))
            self.X = np.empty((self.N + 1, 7))
            self.X[0] = initial
            for j in range(self.N):
                self.X[j + 1] = self.integrate(self.X[j], self.U[j], self.dt)
        self.X[0] = initial
        qp_ms, history, status = 0.0, [], 2
        qp_status = "not_run"
        for iteration in range(self.options.max_iterations):
            rows, q = self._qp(initial, time)
            solver = osqp.OSQP()
            solver.setup(
                P=self.H,
                q=q,
                A=rows.matrix(),
                l=np.array(rows.lo),
                u=np.array(rows.hi),
                verbose=False,
                eps_abs=self.options.qp_tolerance,
                eps_rel=self.options.qp_tolerance,
                max_iter=30000,
                polishing=True,
                scaled_termination=False,
            )
            solver.warm_start(x=np.r_[self.X.ravel(), self.U.ravel()])
            result = solver.solve(raise_error=False)
            qp_ms += result.info.run_time * 1000
            qp_status = result.info.status
            if result.info.status_val != 1:
                history.append({"iteration": iteration + 1, "qp_status": qp_status})
                status = 4
                break
            if result.x is None or not np.isfinite(result.x).all():
                qp_status = "nonfinite_qp_solution"
                history.append({"iteration": iteration + 1, "qp_status": qp_status})
                status = 4
                break
            X = result.x[: self.uoffset].reshape(self.N + 1, 7)
            U = result.x[self.uoffset :].reshape(self.N, 2)
            position_changes = np.linalg.norm(X[1:, 1:3] - self.X[1:, 1:3], axis=1)
            apriori = float(position_changes.mean())
            self.X, self.U = X, U
            violation, coordinate, defect = self.residuals(X, U, initial, time)
            history.append(
                {
                    "iteration": iteration + 1,
                    "qp_status": qp_status,
                    "apriori_error_m": apriori,
                    "maximum_position_change_m": float(position_changes.max()),
                    "max_violation": violation,
                    "qp_primal_residual": result.info.prim_res,
                    "qp_dual_residual": result.info.dual_res,
                    "qp_dual_residual_original_objective": result.info.dual_res
                    * self.objective_scale,
                }
            )
            if (
                apriori <= self.options.apriori_tolerance_m
                and violation <= self.options.feasibility_tolerance
            ):
                status = 0
                break
        violation, coordinate, defect = self.residuals(self.X, self.U, initial, time)
        return {
            "status": status,
            "iterations": iteration + 1,
            "max_violation": violation,
            "coordinate_error": coordinate,
            "dynamics_defect": defect,
            "qp_status": qp_status,
            "native_solver_ms": None,
            "qp_kernel_sum_ms": qp_ms,
            "solve_wall_ms": 1000 * (perf_counter() - start),
            "iteration_history": history,
            "paper_equation_reconstruction": True,
        }

    def shift(self):
        self.previous_input = self.U[0].copy()
        self.X = np.vstack(
            [self.X[1:], self.integrate(self.X[-1], self.U[-1], self.dt)]
        )
        self.U = np.vstack([self.U[1:], self.U[-1]])

    def configuration(self):
        return {
            "state_order": self.state_order,
            "control_order": ("delta", "a"),
            "options": asdict(self.options),
            "input_lower": self.ul.tolist(),
            "input_upper": self.uu.tolist(),
            "input_increment_bounds": self.du.tolist(),
            "stage_weights": self.w.tolist(),
            "input_increment_weights": self.r.tolist(),
            "slack": "hard-constraint option of Eq. 15; epsilon fixed zero",
            "corridor_geometry": "frozen-apriori full bus Frenet bounding envelope",
            "acceptance": "paper apriori criterion AND common nonlinear residual <=1e-4",
            "lateral_acceleration_validation": "both endpoints per constant-input interval",
            "objective_normalization": self.objective_scale,
            "qp_dual_residual_units": "normalized objective; original units additionally recorded",
        }
