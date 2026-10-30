"""Six-state paper reconstruction and matched nine-state lifting ablation.

The engineering reference is planning/2.1 at bee02d33. This is a new,
standalone implementation, not the onboard planner or its original tuning.
"""

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from uuid import uuid4

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from reiter2023.geometry import (
    covering_circles,
    covering_circles_paper,
    ellipse_constraint,
)
from paper_comparison.vehicle import GEOMETRY
from reiter2023.trajectory_planner_acados_20222 import paper_vehicle


@dataclass(frozen=True)
class Settings:
    intervals: int = 40
    dt: float = 0.1
    lateral: float = 5.5
    speed: float = 15.0
    road_margin: float = 0.2
    obstacle_margin: float = 0.2
    # Lateral offset of the obstacle from the reference line. Zero places it on
    # the centre line, which is what every run before the gap study used.
    obstacle_lateral: float = 0.0
    max_speed: float = 20.0
    min_acc: float = -3.0
    max_acc: float = 2.5
    max_jerk: float = 4.0
    max_lat_acc: float = 5.0
    max_lat_jerk: float = 5.0
    # New experimental weights, not claimed to be deployment settings.
    weights: tuple = (
        1.0,
        500.0,
        1000.0,
        1000.0,
        1.0,
        10000.0 * GEOMETRY.wheelbase**2,
        2e6 * GEOMETRY.wheelbase**2,
        1.0,
        1.0,
        1.0,
    )
    terminal_weights: tuple = (10.0, 500.0, 1000.0, 1000.0, 1.0, 10000.0 * GEOMETRY.wheelbase**2)


def bus_vehicle():
    v = paper_vehicle()
    # Table 1 totals 6.50 m, but wb+overhangs totals 6.50 exactly as well.
    # vehicle_gym's center=wb/2 needs a +0.09 m rear-axle correction below.
    v.length_front = v.length_rear = GEOMETRY.wheelbase / 2
    v.chassis_length, v.chassis_width = GEOMETRY.length, GEOMETRY.width
    v.maximum_velocity = 20.0
    v.maximum_steering_angle = np.deg2rad(GEOMETRY.max_steering_deg)
    v.maximum_steering_rate = np.deg2rad(GEOMETRY.max_steering_rate_degps)
    v.maximum_lateral_acc = 5.0
    v.maximum_acceleration_force = v.mass * 2.5
    v.maximum_deceleration_force = v.mass * 3.0
    return v


def transform_local(pose, x, y=0.0):
    return pose[:2] + ca.vertcat(
        x * ca.cos(pose[2]) - y * ca.sin(pose[2]),
        x * ca.sin(pose[2]) + y * ca.cos(pose[2]),
    )


def gym_pose(pose):
    """Correct vehicle_gym's symmetric-chassis origin for the real bus overhangs."""
    shift = (GEOMETRY.length / 2 - GEOMETRY.rear_overhang) - GEOMETRY.wheelbase / 2
    return ca.vertcat(transform_local(pose, shift), pose[2])


def point_segment_squared(p, a, b):
    ab = b - a
    t = ca.fmin(1, ca.fmax(0, ca.dot(p - a, ab) / (ca.sumsqr(ab) + 1e-16)))
    return ca.sumsqr(p - a - t * ab)


def segment_squared(a, b, c, d):
    """Exact piecewise planar segment distance, including interior crossings."""
    ab, cd, ac = b - a, d - c, c - a
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]
    den = cross(ab, cd)
    safe_den = ca.if_else(ca.fabs(den) > 1e-12, den, 1.0)
    t, u = cross(ac, cd) / safe_den, cross(ac, ab) / safe_den
    crossing = ca.logic_and(
        ca.fabs(den) > 1e-12,
        ca.logic_and(ca.logic_and(t >= 0, t <= 1), ca.logic_and(u >= 0, u <= 1)),
    )
    ends = ca.fmin(
        ca.fmin(point_segment_squared(a, c, d), point_segment_squared(b, c, d)),
        ca.fmin(point_segment_squared(c, a, b), point_segment_squared(d, a, b)),
    )
    return ca.if_else(crossing, 0.0, ends)


def hybrid_parts(pose):
    """Equations (15)--(21): central capsule plus four end-strip circles."""
    length, width = GEOMETRY.length, GEOMETRY.width
    center = length / 2 - GEOMETRY.rear_overhang
    half_segment = (length - width) / 2
    a = transform_local(pose, center - half_segment)
    b = transform_local(pose, center + half_segment)
    circles = [
        transform_local(pose, center + sx * (length / 2 - width / 4), sy * width / 4)
        for sx in [-1, 1]
        for sy in [-1, 1]
    ]
    return a, b, width / 2, circles, np.sqrt(2) * width / 4


def capsule_distance(pose, other_pose, margin):
    """Equation (23): the sufficient safety condition between two capsules.

    The safety condition of the manuscript is the minimum distance between the
    two continuous capsule sets, d_min = min_{c1 in Omega_1, c2 in Omega_2}
    ||c1 - c2||, with safety guaranteed when d_min > r1 + r2. Because the
    capsules are continuous, this single condition separates the two bodies
    regardless of where the closest pair lies, which is what makes the
    pruning of Section 5.2 safe.
    """
    a, b, r, _, _ = hybrid_parts(pose)
    c, d, q, _, _ = hybrid_parts(other_pose)
    return ca.sqrt(segment_squared(a, b, c, d) + 1e-16) - r - q - margin


def hybrid_component_distances(pose, other_pose, margin):
    """The 25 capsule/circle component-pair distances, in a fixed order.

    Order: 1 segment-segment pair, 4 ego corner circles against the opponent
    segment, 4 opponent corner circles against the ego segment, and 16
    ego-corner against opponent-corner pairs.
    """
    a, b, r, circles, rc = hybrid_parts(pose)
    c, d, q, others, qc = hybrid_parts(other_pose)
    distance = lambda squared, radius: ca.sqrt(squared + 1e-16) - radius - margin
    values = [distance(segment_squared(a, b, c, d), r + q)]
    values += [distance(point_segment_squared(p, c, d), rc + q) for p in circles]
    values += [distance(point_segment_squared(p, a, b), qc + r) for p in others]
    values += [distance(ca.sumsqr(p - q), rc + qc) for p in circles for q in others]
    return values


def hybrid_constraints(pose, other_pose, margin):
    """All 25 component pairs, not the manuscript's underspecified pair pruning.

    Keeping every pair prevents a frozen nearest-pair selection from overlooking
    a different colliding component during an optimizer step.
    """
    return ca.vertcat(*hybrid_component_distances(pose, other_pose, margin))


def select_component(values, indices):
    """Constraint value of the components named by ``indices``.

    ``indices`` is a list of component indices. For every index the matching
    component distance is selected with a comparison chain, which keeps the
    expression smooth in the state and makes it non-smooth only in the
    discrete index, exactly as the manuscript's pruning description implies.
    The constraint is the minimum over the selected components, so the
    strictest of the retained pairs is enforced.
    """
    indices = list(indices)
    if len(indices) == 1:
        index = indices[0]
        selected = values[0]
        for position in range(1, len(values)):
            selected = ca.if_else(
                ca.eq(index, position), values[position], selected
            )
        return selected

    count = len(values)
    selected = values[0]
    for index in indices:
        candidate = values[0]
        for position in range(1, count):
            candidate = ca.if_else(
                ca.eq(index, position), values[position], candidate
            )
        selected = ca.fmin(selected, candidate)
    return selected


def approximate_road_offsets(x, road, segments=5, frozen_heading=False):
    """Rear-axle contour offsets along the deformed vehicle body.

    ``frozen_heading=True`` reproduces the frozen-heading-deviation variant of
    Lima et al. [16]: the heading deviation is held at its value at the rear
    axle along the whole contour instead of being integrated along it.
    """
    s, n, alpha = x[0], x[1], x[2]
    result = []
    for longitudinal in [-GEOMETRY.rear_overhang, GEOMETRY.front_extent]:
        for lateral in [-GEOMETRY.half_width, GEOMETRY.half_width]:
            ds = longitudinal * ca.cos(alpha) - lateral * ca.sin(alpha)
            # Signed ds handles the backward rear-overhang integral explicitly.
            offset = n + lateral / ca.cos(alpha)
            for i in range(segments):
                angle = (
                    alpha
                    if frozen_heading
                    else alpha - road.curvature(s) * ds * i / segments
                )
                offset += ds / segments * ca.tan(angle)
            result.append(offset)
    return ca.vertcat(*result)


class PaperModel:
    def __init__(
        self,
        road,
        settings=None,
        lifted=False,
        obstacle="HYBRID",
        frozen_heading=False,
        circle_count=3,
        circle_covering="gym",
        pruned=False,
        n_disc=5,
        kappa_as_state=True,
        prune_keep=1,
        obstacle_constraint="components",
    ):
        self.road, self.settings = road, settings or Settings()
        self.lifted, self.obstacle = lifted, obstacle
        self.frozen_heading = frozen_heading
        self.circle_count = circle_count
        # "gym" uses the fixed-radius covering of vehicle_gym; "paper" uses
        # the N-dependent radius of the manuscript's Eq. (11)-(14).
        self.circle_covering = circle_covering
        self.pruned = pruned
        self.n_disc = n_disc
        # When False the sixth state holds the steering angle instead of the
        # curvature, and the curvature follows from the kinematic relation
        # kappa = delta / l_wb. This is the ablation reviewer 2.7 asks for.
        self.kappa_as_state = kappa_as_state
        # Number of component pairs kept by the hierarchical pruning.
        self.prune_keep = prune_keep
        # "components" checks every capsule/circle pair; "capsule" uses the
        # sufficient condition of Eq. (23), which is the manuscript's own
        # formulation and is the weaker of the two.
        self.obstacle_constraint = obstacle_constraint
        self.nx, self.nu = (9 if lifted else 6), 2
        self.N, self.dt = self.settings.intervals, self.settings.dt
        self.vehicle = bus_vehicle()
        self._symbolic()

    def _symbolic(self):
        opts, vehicle = self.settings, self.vehicle
        x, u = ca.MX.sym("x", self.nx), ca.MX.sym("u", 2)
        # Three obstacle pose parameters plus moving s/d/v targets, plus the
        # pruned component index when the manuscript's pruning strategy is used.
        p = ca.MX.sym("p", 6 + self.prune_keep if self.pruned else 6)
        s, n, alpha, v, a, sixth = (x[i] for i in range(6))
        if self.kappa_as_state:
            curvature = sixth
        else:
            # Steering angle as the state; curvature follows from kappa = d/l_wb.
            curvature = sixth / self.vehicle.length
        pose_from_frenet = self.road.pose(s, n, alpha)
        pose = x[6:9] if self.lifted else pose_from_frenet
        den = 1 - n * self.road.curvature(s)
        sd = v * ca.cos(alpha) / den
        if self.kappa_as_state:
            sixth_rate = u[1]
        else:
            # The control is now the steering rate, so the steering angle
            # integrates directly.
            sixth_rate = u[1]
        rhs = [
            sd,
            v * ca.sin(alpha),
            v * curvature - self.road.curvature(s) * sd,
            a,
            u[0],
            sixth_rate,
        ]
        if self.lifted:
            rhs += [v * ca.cos(pose[2]), v * ca.sin(pose[2]), v * curvature]
        rhs = ca.vertcat(*rhs)
        self.x, self.u, self.p, self.rhs = x, u, p, rhs
        self.ode = ca.Function("ode", [x, u], [rhs])
        dt = ca.MX.sym("dt")
        k1 = self.ode(x, u)
        k2 = self.ode(x + dt * k1 / 2, u)
        k3 = self.ode(x + dt * k2 / 2, u)
        k4 = self.ode(x + dt * k3, u)
        step = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.integrate = ca.Function("integrate", [x, u, dt], [step])
        self.step = ca.Function("step", [x, u], [self.integrate(x, u, self.dt)])
        self.linearize = ca.Function(
            "linearize",
            [x, u],
            [ca.jacobian(self.step(x, u), x), ca.jacobian(self.step(x, u), u)],
        )
        if self.obstacle == "HYBRID" and self.obstacle_constraint == "capsule":
            obs = ca.vertcat(
                capsule_distance(pose, p[:3], opts.obstacle_margin)
            )
        elif self.obstacle == "HYBRID":
            if self.pruned:
                # Manuscript Section 5.2: keep only the corrected nearest pair.
                obs = ca.vertcat(
                    select_component(
                        hybrid_component_distances(
                            pose, p[:3], opts.obstacle_margin
                        ),
                        [p[6 + offset] for offset in range(self.prune_keep)],
                    )
                )
            else:
                obs = hybrid_constraints(pose, p[:3], opts.obstacle_margin)
        elif self.obstacle == "ELIPSE":
            obs = ellipse_constraint(
                gym_pose(pose), gym_pose(p[:3]), vehicle, vehicle, opts.obstacle_margin
            )
        elif self.obstacle == "CIRCLES":
            cover = (
                covering_circles_paper
                if self.circle_covering == "paper"
                else covering_circles
            )
            ego, er = cover(gym_pose(pose), vehicle, self.circle_count)
            other, rr = cover(gym_pose(p[:3]), vehicle, self.circle_count)
            obs = ca.vertcat(
                *[
                    ca.sqrt(ca.sumsqr(a - b) + 1e-16) - er - rr - opts.obstacle_margin
                    for a in ego
                    for b in other
                ]
            )
        else:
            raise ValueError(self.obstacle)
        offsets = approximate_road_offsets(
            x,
            self.road,
            segments=self.n_disc,
            frozen_heading=self.frozen_heading,
        )
        alat = v**2 * curvature
        if self.kappa_as_state:
            curvature_rate = u[1]
        else:
            curvature_rate = u[1] / vehicle.length
        jlat = 2 * v * a * curvature + v**2 * curvature_rate
        steering_rate = vehicle.length * curvature_rate / (
            1 + (vehicle.length * curvature) ** 2
        )
        self.h = ca.vertcat(alat, jlat, steering_rate, den, offsets, obs)
        width = self.road.half_width - opts.road_margin
        self.hl = np.r_[
            -opts.max_lat_acc,
            -opts.max_lat_jerk,
            -vehicle.maximum_steering_rate,
            0.1,
            np.full(4, -width),
            np.zeros(obs.numel()),
        ]
        self.hu = np.r_[
            opts.max_lat_acc,
            opts.max_lat_jerk,
            vehicle.maximum_steering_rate,
            1e8,
            np.full(4, width),
            np.full(obs.numel(), 1e8),
        ]
        self.hfun = ca.Function("hfun", [x, u, p], [self.h])
        # Shared physical residuals for the six/nine-state dimension ablation.
        residual = ca.vertcat(
            s - p[3],
            n - p[4],
            alpha,
            v - p[5],
            a,
            curvature,
            curvature_rate,
            u[0],
            alat,
            jlat,
        )
        self.residual = residual
        self.terminal_residual = ca.substitute(residual[:6], u, ca.DM.zeros(2))
        weights = np.asarray(opts.weights, dtype=float)
        if not self.kappa_as_state:
            # u[1] is the steering rate here; the curvature rate is u[1]/l_wb.
            # Rescale so the residual penalises the same physical quantity.
            weights[6] *= vehicle.length**2
        self.cost = ca.dot(ca.DM(weights) * residual, residual) * self.dt
        self.terminal_cost = ca.dot(
            ca.DM(opts.terminal_weights) * self.terminal_residual,
            self.terminal_residual,
        )
        self.state_indices = np.array([2, 3, 4, 5])
        sixth_low = (
            -np.tan(vehicle.maximum_steering_angle) / vehicle.length
            if self.kappa_as_state
            else -vehicle.maximum_steering_angle
        )
        sixth_high = (
            np.tan(vehicle.maximum_steering_angle) / vehicle.length
            if self.kappa_as_state
            else vehicle.maximum_steering_angle
        )
        self.xl = np.array([-0.65, 0, opts.min_acc, sixth_low])
        self.xu = np.array([0.65, opts.max_speed, opts.max_acc, sixth_high])
        # Physical steering-rate constraint above is state dependent; this box is redundant.
        max_kr = (
            vehicle.maximum_steering_rate
            / vehicle.length
            * (1 + np.tan(vehicle.maximum_steering_angle) ** 2)
        )
        if not self.kappa_as_state:
            # The control is the steering rate directly; the curvature rate
            # follows from kappa = delta / l_wb.
            max_kr = vehicle.maximum_steering_rate
        self.ul, self.uu = (
            np.array([-opts.max_jerk, -max_kr]),
            np.array([opts.max_jerk, max_kr]),
        )
        g = ca.vertcat(
            self.hl - self.h,
            self.h - self.hu,
            self.xl - x[self.state_indices],
            x[self.state_indices] - self.xu,
        )
        self.g = g
        self.gfun = ca.Function("gfun", [x, u, p], [g])

    def initial(self, speed=12.0):
        curvature = self.road.kappa
        sixth = curvature if self.kappa_as_state else curvature * self.vehicle.length
        state = np.array([0, 0, 0, speed, 0, sixth])
        return self.augment(state)

    def augment(self, state):
        return (
            np.r_[state[:6], self.road.numeric_pose(state)]
            if self.lifted
            else np.array(state[:6], dtype=float)
        )

    def parameters(self, time, initial_s, obstacle_start=35.0, obstacle_speed=8.0):
        times = time + np.arange(self.N + 1) * self.dt
        rows = [
            np.r_[
                self.road.numeric_pose(
                    [obstacle_start + obstacle_speed * t, self.settings.obstacle_lateral, 0]
                ),
                initial_s + k * self.dt * self.settings.speed,
                self.settings.lateral,
                self.settings.speed,
            ]
            for k, t in enumerate(times)
        ]
        if self.pruned:
            # The index columns are filled in by the planner before each solve.
            rows = [
                np.r_[row, np.zeros(self.prune_keep)] for row in rows
            ]
        return np.array(rows)

    def full_obstacle_violation(self, X, P):
        """Largest overlap over all component pairs, ignoring the pruning.

        The pruned variant only constrains the retained pairs, so a trajectory
        can satisfy the constraint it was given and still overlap on a pair
        that was dropped. This evaluation is the independent check that the
        pruning did not lose a collision.
        """
        worst = 0.0
        for k in range(self.N + 1):
            if self.lifted:
                pose = np.array(X[k][6:9], dtype=float)
            else:
                pose = np.asarray(self.road.pose(X[k][0], X[k][1], X[k][2])).ravel()
            for value in hybrid_component_distances(
                pose, P[k][:3], self.settings.obstacle_margin
            ):
                worst = max(worst, -float(ca.evalf(value)))
        return worst

    def violation(self, X, U, P, initial):
        if not np.isfinite(X).all() or not np.isfinite(U).all():
            return {
                "max_violation": 1e30,
                "dynamics_defect": 1e30,
                "coordinate_error": 1e30,
                "full_obstacle_violation": 1e30,
            }
        violation = max(0.0, float(np.max(self.ul - U)), float(np.max(U - self.uu)))
        for k in range(self.N + 1):
            violation = max(
                violation,
                float(
                    np.max(self.gfun(X[k], U[k] if k < self.N else np.zeros(2), P[k]))
                ),
            )
        defect = max(
            float(np.max(np.abs(np.asarray(self.step(X[k], U[k])).ravel() - X[k + 1])))
            for k in range(self.N)
        )
        coordinate_error = (
            max(float(np.max(np.abs(x[6:9] - self.road.numeric_pose(x)))) for x in X)
            if self.lifted
            else 0.0
        )
        full_obstacle = (
            self.full_obstacle_violation(X, P)
            if self.pruned
            else 0.0
        )
        return {
            "max_violation": max(
                violation,
                defect,
                float(np.max(np.abs(X[0] - initial))),
                full_obstacle,
            ),
            "dynamics_defect": defect,
            "coordinate_error": coordinate_error,
            "full_obstacle_violation": full_obstacle,
        }


class AcadosPaperPlanner:
    """Common-solver adaptation, explicitly NOT the paper's original CiLQR."""

    def __init__(self, model, directory):
        self.model = m = model
        ocp, am = AcadosOcp(), AcadosModel()
        am.name = "bus_paper_" + uuid4().hex[:10]
        am.x, am.u, am.p = m.x, m.u, m.p
        am.xdot = ca.MX.sym("xdot", m.nx)
        am.f_expl_expr, am.f_impl_expr = m.rhs, am.xdot - m.rhs
        am.con_h_expr = am.con_h_expr_e = m.h
        # Terminal constraints cannot depend on an absent terminal control.
        am.con_h_expr_e = ca.substitute(m.h, m.u, ca.DM.zeros(2))
        am.cost_y_expr, am.cost_y_expr_e = m.residual, m.terminal_residual
        ocp.model = am
        ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W, ocp.cost.W_e = (
            2 * np.diag(m.settings.weights),
            2 * np.diag(m.settings.terminal_weights),
        )
        ocp.cost.yref, ocp.cost.yref_e = np.zeros(10), np.zeros(6)
        ocp.constraints.idxbx = ocp.constraints.idxbx_e = m.state_indices
        ocp.constraints.lbx = ocp.constraints.lbx_e = m.xl
        ocp.constraints.ubx = ocp.constraints.ubx_e = m.xu
        ocp.constraints.idxbu = np.arange(2)
        ocp.constraints.lbu, ocp.constraints.ubu = m.ul, m.uu
        ocp.constraints.lh = ocp.constraints.lh_e = m.hl
        ocp.constraints.uh = ocp.constraints.uh_e = m.hu
        ocp.constraints.x0 = m.initial()
        ocp.parameter_values = np.r_[
            1000, 0, 0, 0, m.settings.lateral, m.settings.speed,
            np.zeros(max(0, int(m.p.numel()) - 6)),
        ]
        so = ocp.solver_options
        so.N_horizon, so.tf = m.N, m.N * m.dt
        so.nlp_solver_type, so.nlp_solver_max_iter = "SQP", 150
        so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        so.qp_solver_cond_N, so.qp_solver_iter_max = max(1, m.N // 2), 100
        so.hessian_approx, so.integrator_type = "GAUSS_NEWTON", "ERK"
        so.sim_method_num_stages, so.sim_method_num_steps = 4, 1
        so.tol, so.print_level = 1e-6, 0
        directory = Path(directory).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        ocp.code_export_directory = str(directory)
        start = perf_counter()
        self.solver = AcadosOcpSolver(ocp, json_file=str(directory / "ocp.json"))
        self.build_seconds = perf_counter() - start
        self.X, self.U = None, None

    def pruning_indices(self, parameters):
        """Component index selected at each predicted step.

        The manuscript selects, per step, the pair of components with the
        smallest distance and replaces redundant sub-capsule members by corner
        circles. Here the selection uses the previous solution's predicted ego
        poses, which is the information available at planning time; the selected
        index is held constant over one solve, so the constraint is non-smooth in
        the index exactly as in the manuscript's description.
        """
        m = self.model
        if self.X is not None:
            # The previous solution holds the Cartesian pose of the lifted
            # state, or the Frenet state itself for the six-state model.
            poses = self.X[:, :3]
        else:
            # Before the first solve, extrapolate the Frenet state along the
            # planned speed; the obstacle poses come from the parameters.
            initial_s = float(parameters[0][3])
            poses = np.array(
                [
                    [
                        initial_s + k * m.dt * m.settings.speed,
                        m.settings.lateral,
                        0.0,
                    ]
                    for k in range(m.N + 1)
                ]
            )
        indices = []
        for k in range(m.N + 1):
            if m.lifted:
                # The lifted state carries the Cartesian pose directly.
                ego = np.array(poses[k][:3], dtype=float)
            else:
                ego = m.road.pose(poses[k, 0], poses[k, 1], poses[k, 2])
            values = []
            for value in hybrid_component_distances(
                ego, parameters[k][:3], m.settings.obstacle_margin
            ):
                values.append(float(ca.evalf(value)))
            order = np.argsort(values)
            indices.append(
                [int(index) for index in order[: max(1, m.prune_keep)]]
            )
        return indices

    def solve(self, initial, parameters):
        m = self.model
        start = perf_counter()
        self.solver.constraints_set(0, "lbx", initial)
        self.solver.constraints_set(0, "ubx", initial)
        if self.X is None:
            state = initial.copy()
            for k in range(m.N + 1):
                self.solver.set(k, "x", state)
                if k < m.N:
                    self.solver.set(k, "u", np.zeros(2))
                    state = np.asarray(m.step(state, [0, 0])).ravel()
        if m.pruned:
            indices = self.pruning_indices(parameters)
            parameters = np.array(
                [
                    np.r_[row[:6], np.asarray(index, dtype=float)]
                    for row, index in zip(parameters, indices)
                ]
            )
        for k in range(m.N + 1):
            self.solver.set(k, "p", parameters[k])
        status = int(self.solver.solve())
        native_ms = float(self.solver.get_stats("time_tot")) * 1000
        wall_ms = (perf_counter() - start) * 1000
        self.X = np.array([self.solver.get(k, "x") for k in range(m.N + 1)])
        self.U = np.array([self.solver.get(k, "u") for k in range(m.N)])

        if m.pruned and m.prune_keep <= 1:
            # The manuscript switches the retained component pair with the
            # location of the closest pair, so the index has to follow the
            # trajectory. acados cannot change parameters inside a solve, so
            # the selection is redone on the trajectory just computed and the
            # problem is solved again with the updated indices.
            updated = self.pruning_indices(parameters)
            current = [list(map(int, row[6:])) for row in parameters]
            if updated != current:
                parameters = np.array(
                    [
                        np.r_[row[:6], np.asarray(index, dtype=float)]
                        for row, index in zip(parameters, updated)
                    ]
                )
                for k in range(m.N + 1):
                    self.solver.set(k, "p", parameters[k])
                status = int(self.solver.solve())
                native_ms += float(self.solver.get_stats("time_tot")) * 1000
                wall_ms = (perf_counter() - start) * 1000
                self.X = np.array([self.solver.get(k, "x") for k in range(m.N + 1)])
                self.U = np.array([self.solver.get(k, "u") for k in range(m.N)])

        diagnostics = m.violation(self.X, self.U, parameters, initial)
        self.solver_iterations = int(self.solver.get_stats("sqp_iter"))
        return dict(
            status=status,
            native_solver_ms=native_ms,
            solve_wall_ms=wall_ms,
            iterations=self.solver_iterations,
            **diagnostics,
        )

    def shift(self):
        m = self.model
        for k in range(m.N):
            self.solver.set(k, "x", self.X[k + 1])
            self.solver.set(k, "u", self.U[min(k + 1, m.N - 1)])
        self.solver.set(m.N, "x", np.asarray(m.step(self.X[-1], self.U[-1])).ravel())
