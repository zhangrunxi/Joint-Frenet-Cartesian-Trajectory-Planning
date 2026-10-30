"""Reconstruction of EJC 2023 equations (9), (13)--(15), (17)--(19).

Uses current vehicle_gym parameters and circle geometry. This is not the missing
author class and does not monkey-patch the upstream package or original scripts.
"""

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from uuid import uuid4

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from vehicle_models.model_kinematic import KinematicModelParameters
from vehiclegym.planner.trajectory_planner_base import Planner, VehicleObstacleModel

from .geometry import SampledRoad, covering_circles, ellipse_constraint

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class PlannerOptions:
    n_nodes: int = 41
    time_disc: float = 0.1
    use_lifting: bool = True
    obstacle_model: VehicleObstacleModel = VehicleObstacleModel.ELIPSE
    n_circles_ego: int = 3
    n_circles_opp: int = 3
    v_max_terminal_set: float = 15.0
    velocity_set: float = 40.0
    lateral_reference: float = 0.0
    road_margin: float = 0.0
    obstacle_margin: float = 0.0
    denominator_min: float = 0.1
    wind_speed: float = 20.0
    wind_direction: float = 0.0
    nlp_solver: str = "SQP_RTI"
    nlp_solver_max_iter: int = 100
    qp_tolerance: float = 1e-7
    q0: tuple = (1.0, 500.0, 1000.0, 1000.0, 10000.0)
    qn: tuple = (10.0, 90.0, 100.0, 10.0, 10.0)
    r0: tuple = (0.001, 2e6)
    deactivate_obstacle_avoidance: bool = False
    build_directory: str | None = None


def paper_vehicle():
    vehicle = KinematicModelParameters()
    vehicle.length_front = vehicle.length_rear = 1.7
    vehicle.maximum_acceleration_force = vehicle.maximum_deceleration_force = 10000.0
    vehicle.maximum_velocity = 40.0
    vehicle.maximum_lateral_acc = 5.0
    vehicle.maximum_steering_angle = 0.3
    vehicle.maximum_steering_rate = 0.39
    return vehicle


class VehiclePlannerAcados20222(Planner):
    """Five/eight-state Cartesian-obstacle NMPC, with a legacy-shaped API."""

    def __init__(
        self,
        ego_model_params=None,
        road=None,
        planner_options=None,
        opp_model_params=None,
    ):
        super().__init__()
        if road is None:
            raise ValueError("An explicit reference road is required")
        self.road = road if hasattr(road, "pose") else SampledRoad(road)
        self.ego = ego_model_params or paper_vehicle()
        self.opp_model_params = list(opp_model_params or [])
        self.planner_opts = options = planner_options or PlannerOptions()
        if options.obstacle_model not in (
            VehicleObstacleModel.ELIPSE,
            VehicleObstacleModel.CIRCLES,
        ):
            raise NotImplementedError(
                "This reconstruction supports ellipse and covering circles"
            )
        if options.n_nodes < 3 or options.time_disc <= 0:
            raise ValueError("Need >= 3 nodes and a positive time step")
        if options.nlp_solver not in ("SQP_RTI", "SQP"):
            raise ValueError("Use SQP_RTI or SQP")
        self.nx, self.nu = (8 if options.use_lifting else 5), 2
        self.N = options.n_nodes - 1
        self.n_nodes = self.nodes = options.n_nodes
        self.time_disc = options.time_disc
        self.time_grid = np.arange(self.n_nodes) * self.time_disc
        self.time_pred = self.time_grid[-1]
        self.initialized = False
        self.states_ego = None
        self.last_status = []
        self._create_model()
        self._create_solver()

    def _create_model(self):
        options, vehicle = self.planner_opts, self.ego
        self.x = x = ca.MX.sym("x", self.nx)
        self.u = u = ca.MX.sym("u", 2)
        self.p = p = ca.MX.sym("p", 3 * len(self.opp_model_params))
        s, n, alpha, v, delta = (x[i] for i in range(5))
        transformed = self.road.pose(s, n, alpha)
        pose = x[5:8] if options.use_lifting else transformed
        kappa = self.road.curvature(s)
        denominator = 1 - n * kappa
        s_dot = v * ca.cos(alpha) / denominator
        yaw_rate = v * ca.tan(delta) / vehicle.length
        # Equation (9) uses the inverse-transform heading even for the lifted ODE.
        relative_speed = v - options.wind_speed * ca.cos(
            transformed[2] - options.wind_direction
        )
        drag = (
            0.5
            * 1.29
            * vehicle.drag_force_coeff
            * vehicle.drag_area_head
            * relative_speed**2
        )
        rolling = vehicle.mass * vehicle.roll_force_coeff * 9.81
        rhs = [
            s_dot,
            v * ca.sin(alpha),
            yaw_rate - kappa * s_dot,
            (u[0] - drag - rolling) / vehicle.mass,
            u[1],
        ]
        if options.use_lifting:
            rhs.extend([v * ca.cos(pose[2]), v * ca.sin(pose[2]), yaw_rate])
        self.rhs = ca.vertcat(*rhs)
        self.f_ode = ca.Function("paper_ode", [x, u], [self.rhs])
        dt = ca.MX.sym("dt")
        k1 = self.f_ode(x, u)
        k2 = self.f_ode(x + dt * k1 / 2, u)
        k3 = self.f_ode(x + dt * k2 / 2, u)
        k4 = self.f_ode(x + dt * k3, u)
        self.integrate = ca.Function(
            "paper_rk4", [x, u, dt], [x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6]
        )
        self.pose_function = ca.Function("cartesian_pose", [x], [pose])
        self.transform_function = ca.Function("inverse_frenet", [x], [transformed])
        self.a_lat = v**2 * ca.tan(delta) / vehicle.length
        obstacles = []
        if not options.deactivate_obstacle_avoidance:
            for j, other in enumerate(self.opp_model_params):
                other_pose = p[3 * j : 3 * j + 3]
                if options.obstacle_model == VehicleObstacleModel.ELIPSE:
                    obstacles.append(
                        ellipse_constraint(
                            pose, other_pose, vehicle, other, options.obstacle_margin
                        )
                    )
                else:
                    ego_centers, ego_radius = covering_circles(
                        pose, vehicle, options.n_circles_ego
                    )
                    opp_centers, opp_radius = covering_circles(
                        other_pose, other, options.n_circles_opp
                    )
                    for a in ego_centers:
                        for b in opp_centers:
                            obstacles.append(
                                ca.sqrt(ca.sumsqr(a - b) + 1e-12)
                                - ego_radius
                                - opp_radius
                                - options.obstacle_margin
                            )
        self.obstacle_expr = ca.vertcat(*obstacles) if obstacles else ca.MX.zeros(0, 1)
        self.h = ca.vertcat(self.a_lat, denominator, self.obstacle_expr)
        self.h_terminal = ca.vertcat(self.a_lat, denominator)
        self.h_function = ca.Function("constraint_values", [x, u, p], [self.h])
        self.h_terminal_function = ca.Function(
            "terminal_values", [x], [self.h_terminal]
        )

    def _create_solver(self):
        options, vehicle = self.planner_opts, self.ego
        ocp = AcadosOcp()
        model = AcadosModel()
        model.name = (
            "reiter2023_"
            + ("lifted_" if options.use_lifting else "direct_")
            + uuid4().hex[:8]
        )
        model.x, model.u, model.p = self.x, self.u, self.p
        model.xdot = ca.MX.sym("xdot", self.nx)
        model.f_expl_expr = self.rhs
        model.f_impl_expr = model.xdot - self.rhs
        model.con_h_expr = self.h
        model.con_h_expr_e = self.h_terminal
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.time_pred
        # acados integrates stage costs with dt and uses 1/2*y'Wy.
        ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        model.cost_y_expr = ca.vertcat(self.x[:5], self.u)
        model.cost_y_expr_e = self.x[:5]
        ocp.cost.W = 2 * np.diag([*options.q0, *options.r0])
        ocp.cost.W_e = 2 * np.diag(options.qn)
        ocp.cost.yref = np.zeros(7)
        ocp.cost.yref_e = np.zeros(5)
        self.state_indices = np.array([1, 3, 4])
        self.state_lower = np.array(
            [
                -self.road.half_width + options.road_margin,
                0.0,
                -vehicle.maximum_steering_angle,
            ]
        )
        self.state_upper = np.array(
            [
                self.road.half_width - options.road_margin,
                vehicle.maximum_velocity,
                vehicle.maximum_steering_angle,
            ]
        )
        ocp.constraints.idxbx = self.state_indices
        ocp.constraints.lbx, ocp.constraints.ubx = self.state_lower, self.state_upper
        ocp.constraints.idxbx_e = self.state_indices
        ocp.constraints.lbx_e = self.state_lower
        self.terminal_upper = self.state_upper.copy()
        self.terminal_upper[1] = options.v_max_terminal_set
        ocp.constraints.ubx_e = self.terminal_upper
        self.input_lower = np.array(
            [-vehicle.maximum_deceleration_force, -vehicle.maximum_steering_rate]
        )
        self.input_upper = np.array(
            [vehicle.maximum_acceleration_force, vehicle.maximum_steering_rate]
        )
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.lbu, ocp.constraints.ubu = self.input_lower, self.input_upper
        self.h_lower = np.r_[
            -vehicle.maximum_lateral_acc,
            options.denominator_min,
            np.zeros(self.obstacle_expr.numel()),
        ]
        self.h_upper = np.r_[
            vehicle.maximum_lateral_acc, 1e10, np.full(self.obstacle_expr.numel(), 1e10)
        ]
        ocp.constraints.lh, ocp.constraints.uh = self.h_lower, self.h_upper
        ocp.constraints.lh_e, ocp.constraints.uh_e = self.h_lower[:2], self.h_upper[:2]
        ocp.constraints.x0 = np.zeros(self.nx)
        ocp.parameter_values = np.zeros(self.p.numel())
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = max(1, self.N // 2)
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.nlp_solver_type = options.nlp_solver
        ocp.solver_options.nlp_solver_max_iter = options.nlp_solver_max_iter
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.tol = options.qp_tolerance
        ocp.solver_options.print_level = 0
        self.build_directory = Path(
            options.build_directory or ROOT / "results/reconstructed/build" / model.name
        )
        self.build_directory.mkdir(parents=True, exist_ok=True)
        ocp.code_export_directory = str(self.build_directory.resolve())
        self.ocp = ocp
        start = perf_counter()
        self.solver = AcadosOcpSolver(
            ocp, json_file=str(self.build_directory / "ocp.json")
        )
        self.build_seconds = perf_counter() - start
        self.obstacle_predictions = np.zeros(
            (len(self.opp_model_params), self.n_nodes, 3)
        )
        if self.opp_model_params:
            self.obstacle_predictions[:, :, 0] = 10000
        self.set_obstacle_predictions(self.obstacle_predictions)

    def augment_state(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (5,) or not np.isfinite(state).all():
            raise ValueError("Expected five finite Frenet states")
        if self.planner_opts.use_lifting:
            return np.r_[state, self.road.numeric_pose(state)]
        return state.copy()

    def set_obstacle_predictions(self, cartesian_predictions):
        predictions = np.asarray(cartesian_predictions, dtype=float)
        expected = (len(self.opp_model_params), self.n_nodes, 3)
        if predictions.shape != expected or not np.isfinite(predictions).all():
            raise ValueError(f"Cartesian predictions must have shape {expected}")
        self.obstacle_predictions = predictions.copy()
        for k in range(self.n_nodes):
            self.solver.set(k, "p", predictions[:, k, :].ravel())

    def set_states(self, states_ego, actions=None, states_opp=None, t_current=None):
        self.states_ego = np.asarray(states_ego, dtype=float).copy()
        initial = self.augment_state(self.states_ego)
        if actions is not None:
            actions = np.asarray(actions, dtype=float)
            if actions.shape != (3,):
                raise ValueError(
                    "Legacy actions are [lateral reference, speed reference, speed weight]"
                )
            lateral, speed, speed_weight = actions
        else:
            lateral, speed = (
                self.planner_opts.lateral_reference,
                self.planner_opts.velocity_set,
            )
            speed_weight = self.planner_opts.q0[3]
        if not np.isfinite([lateral, speed, speed_weight]).all() or speed_weight < 0:
            raise ValueError(
                "References must be finite with a nonnegative speed weight"
            )
        if states_opp is not None:
            self.set_obstacles(states_opp)
        for k in range(self.n_nodes):
            reference = np.array(
                [initial[0] + self.time_grid[k] * speed, lateral, 0, speed, 0]
            )
            if k < self.N:
                self.solver.cost_set(k, "yref", np.r_[reference, 0, 0])
                weights = np.array([*self.planner_opts.q0, *self.planner_opts.r0])
                weights[3] = speed_weight
                self.solver.cost_set(k, "W", 2 * np.diag(weights))
            else:
                self.solver.cost_set(k, "yref", reference)
        self.solver.constraints_set(0, "lbx", initial)
        self.solver.constraints_set(0, "ubx", initial)
        if not self.initialized:
            self._initial_guess(initial)

    def _initial_guess(self, initial):
        state = initial.copy()
        for k in range(self.n_nodes):
            self.solver.set(k, "x", state)
            if k < self.N:
                hold = self._holding_control(state)
                self.solver.set(k, "u", hold)
                state = np.asarray(self.integrate(state, hold, self.time_disc)).ravel()
        self.initialized = True

    def _holding_control(self, state):
        vehicle, options = self.ego, self.planner_opts
        phi = self.road.numeric_pose(state)[2]
        relative = state[3] - options.wind_speed * np.cos(phi - options.wind_direction)
        drag = (
            0.5 * 1.29 * vehicle.drag_force_coeff * vehicle.drag_area_head * relative**2
        )
        force = drag + vehicle.mass * vehicle.roll_force_coeff * 9.81
        return np.array([force, 0.0])

    def set_obstacles(self, obstacles):
        """Compatibility convenience; constant-n/s-speed prediction, not an interaction model."""
        if len(obstacles) != len(self.opp_model_params):
            raise ValueError(
                "Provide one state or full Frenet prediction per configured obstacle"
            )
        predictions = []
        for state in obstacles:
            state = np.asarray(state, dtype=float)
            if state.shape == (5,):
                samples = np.tile(state, (self.n_nodes, 1))
                denominator = 1 - state[1] * float(self.road.curvature(state[0]))
                samples[:, 0] += (
                    self.time_grid * state[3] * np.cos(state[2]) / denominator
                )
            elif state.shape == (5, self.n_nodes):
                samples = state.T
            else:
                raise ValueError("Obstacle input must be (5,) or (5, n_nodes)")
            predictions.append([self.road.numeric_pose(row) for row in samples])
        self.set_obstacle_predictions(
            np.asarray(predictions).reshape(-1, self.n_nodes, 3)
        )

    def solve(self):
        if self.states_ego is None:
            raise RuntimeError("Call set_states before solve")
        start = perf_counter()
        status = self.solver.solve()
        self.wall_time_ms = (perf_counter() - start) * 1000
        self.computation_time = float(self.solver.get_stats("time_tot"))
        self.last_status = [int(status)]
        self.x_full = np.column_stack(
            [self.solver.get(k, "x") for k in range(self.n_nodes)]
        )
        self.u_full = np.column_stack([self.solver.get(k, "u") for k in range(self.N)])
        self.current_slacks = np.zeros(self.h.numel())
        self.diagnostics = self.evaluate_solution()
        return int(status)

    def warm_start(self, states_ego, actions=None, max_iterations=50, tolerance=1e-5):
        """Explicit repeated RTI initialization; every iteration is counted and reported."""
        self.set_states(states_ego, actions=actions)
        statuses, times, previous_controls = [], [], None
        change = float("inf")
        for _ in range(max_iterations):
            statuses.append(self.solve())
            times.append(self.computation_time)
            if previous_controls is not None:
                change = float(
                    np.max(
                        np.abs(
                            (self.u_full - previous_controls)
                            / self.input_upper[:, None]
                        )
                    )
                )
            if statuses[-1] != 0:
                break
            if (
                len(statuses) >= 5
                and change < 1e-4
                and self.diagnostics["max_violation"] <= tolerance
            ):
                break
            previous_controls = self.u_full.copy()
        return {
            "statuses": statuses,
            "iterations": len(statuses),
            "solver_time_s": float(sum(times)),
            "normalized_control_change": change,
            "diagnostics": dict(self.diagnostics),
        }

    def shift_guess(self):
        if self.x_full is None:
            raise RuntimeError("No solution to shift")
        for k in range(self.N):
            self.solver.set(k, "x", self.x_full[:, k + 1])
            self.solver.set(k, "u", self.u_full[:, min(k + 1, self.N - 1)])
        last = np.asarray(
            self.integrate(self.x_full[:, -1], self.u_full[:, -1], self.time_disc)
        ).ravel()
        self.solver.set(self.N, "x", last)

    def evaluate_solution(self):
        X, U = self.x_full, self.u_full
        if not np.isfinite(X).all() or not np.isfinite(U).all():
            return {"finite": False, "max_violation": float("inf")}
        initial = self.augment_state(self.states_ego)
        dynamic_error = max(
            float(
                np.max(
                    np.abs(
                        X[:, k + 1]
                        - np.asarray(
                            self.integrate(X[:, k], U[:, k], self.time_disc)
                        ).ravel()
                    )
                )
            )
            for k in range(self.N)
        )
        box_violation = 0.0
        h_violation = 0.0
        min_obstacle = float("inf")
        for k in range(self.n_nodes):
            upper = self.terminal_upper if k == self.N else self.state_upper
            value = X[self.state_indices, k]
            box_violation = max(
                box_violation,
                float(np.max(self.state_lower - value)),
                float(np.max(value - upper)),
            )
            if k < self.N:
                values = np.asarray(
                    self.h_function(
                        X[:, k], U[:, k], self.obstacle_predictions[:, k, :].ravel()
                    )
                ).ravel()
                if len(values) > 2:
                    min_obstacle = min(min_obstacle, float(np.min(values[2:])))
                h_violation = max(
                    h_violation,
                    float(np.max(self.h_lower - values)),
                    float(np.max(values - self.h_upper)),
                )
            else:
                values = np.asarray(self.h_terminal_function(X[:, k])).ravel()
                h_violation = max(
                    h_violation,
                    float(np.max(self.h_lower[:2] - values)),
                    float(np.max(values - self.h_upper[:2])),
                )
        input_violation = max(
            0.0,
            float(np.max(self.input_lower[:, None] - U)),
            float(np.max(U - self.input_upper[:, None])),
        )
        initial_error = float(np.max(np.abs(X[:, 0] - initial)))
        consistency = 0.0
        if self.nx == 8:
            consistency = max(
                float(np.max(np.abs(X[5:8, k] - self.road.numeric_pose(X[:5, k]))))
                for k in range(self.n_nodes)
            )
        return {
            "finite": True,
            "initial_error": initial_error,
            "rk4_defect": dynamic_error,
            "state_bound_violation": max(0.0, box_violation),
            "input_bound_violation": input_violation,
            "nonlinear_violation": max(0.0, h_violation),
            "min_obstacle_constraint": min_obstacle
            if np.isfinite(min_obstacle)
            else None,
            "coordinate_consistency_error": consistency,
            "max_violation": max(
                initial_error,
                dynamic_error,
                box_violation,
                input_violation,
                h_violation,
                0.0,
            ),
        }

    def reset_solver(self):
        self.solver.reset()
        self.initialized = False

    def get_statistics(self, fields):
        return [self.solver.get_stats(field) for field in fields]
