"""Matched bus MPC experiment; saves every rejected solution and all timings."""

import argparse
import csv
import hashlib
import json
import os
import platform
import sys
from dataclasses import asdict, replace
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import casadi as ca
import numpy as np
from vehiclegym.planner.trajectory_planner_base import VehicleObstacleModel

from paper_comparison.cilqr import CilqrPlanner
from paper_comparison.evaluation import evaluate_sample
from paper_comparison.joint_linear_proxy import JointLinearProxy
from paper_comparison.model import (
    AcadosPaperPlanner,
    PaperModel,
    Settings,
    approximate_road_offsets,
    bus_vehicle,
    gym_pose,
)
from paper_comparison.harness_options import (
    INITIAL_GUESS,
    N_DISC,
    accepted_status,
    lateral_ramp,
)
from paper_comparison.vehicle import GEOMETRY
from reiter2023 import CircularRoad, PlannerOptions, VehiclePlannerAcados20222


class BusReiter(VehiclePlannerAcados20222):
    """Reiter equations with common bus geometry/limits and contour road bounds.

    road_bound="point" restores the published reference-point bound
    |n| <= half_width - margin, shrunk only by the constant half bus width.
    """

    road_bound = "contour"

    def _create_model(self):
        super()._create_model()
        # Correct the obstacle/body origins without modifying preserved reconstruction.
        pose = (
            self.x[5:8]
            if self.planner_opts.use_lifting
            else self.transform_function(self.x)
        )
        from reiter2023.geometry import covering_circles, ellipse_constraint

        if self.planner_opts.obstacle_model == VehicleObstacleModel.ELIPSE:
            obs = ellipse_constraint(
                gym_pose(pose), gym_pose(self.p[:3]), self.ego, self.ego, 0.2
            )
        else:
            aa, ra = covering_circles(gym_pose(pose), self.ego, 3)
            bb, rb = covering_circles(gym_pose(self.p[:3]), self.ego, 3)
            obs = ca.vertcat(
                *[
                    ca.sqrt(ca.sumsqr(a - b) + 1e-16) - ra - rb - 0.2
                    for a in aa
                    for b in bb
                ]
            )
        # Existing builder assigns >=0 to all entries after the first two.
        if self.road_bound in ("point", "inflated"):
            offsets, width = self.x[1], self.road.half_width - 0.2 - GEOMETRY.half_width
            if self.road_bound == "inflated" and abs(self.road.kappa) > 1e-12:
                # Static curvature-aware inflation of the outer bound: the
                # outward swing of the outer front corner of a body tangent
                # to the arc of the rear axle, the usual engineering fix of
                # the reference-point bound.
                radius = 1 / abs(self.road.kappa) + GEOMETRY.half_width
                swing = np.hypot(radius, GEOMETRY.front_extent) - radius
                outer = -np.sign(self.road.kappa)
                offsets = offsets + outer * swing / 2
                width = width - swing / 2
        else:
            offsets = approximate_road_offsets(self.x, self.road, segments=N_DISC)
            width = self.road.half_width - 0.2
        self.obstacle_expr = ca.vertcat(obs, offsets + width, width - offsets)
        self.h = ca.vertcat(
            self.a_lat,
            1 - self.x[1] * self.road.curvature(self.x[0]),
            self.obstacle_expr,
        )
        self.h_function = ca.Function("bus_h", [self.x, self.u, self.p], [self.h])


class BusReiterPoint(BusReiter):
    road_bound = "point"


class BusReiterInflated(BusReiter):
    road_bound = "inflated"


METHODS = [
    "reiter_direct_ellipse",
    "reiter_lifted_ellipse",
    "reiter_direct_circles",
    "reiter_lifted_circles",
    "paper6_hybrid_sqp",
    "paper9_hybrid_sqp",
    "paper6_circles_sqp",
    "paper6_circlesN3_sqp",
    "paper6_circlesN5_sqp",
    "paper6_hybrid_cilqr",
    "xing_inspired_proxy_circles",
    "xing_inspired_proxy_ellipse",
]


def finite_json(obj):
    if isinstance(obj, dict):
        return {k: finite_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [finite_json(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return finite_json(obj.tolist())
    if isinstance(obj, (float, np.floating)):
        return float(obj) if np.isfinite(obj) else None
    if isinstance(obj, np.integer):
        return int(obj)
    return obj


def run_case(
    output,
    method,
    scenario,
    steps,
    settings,
    half_width=8.5,
    obstacle_start=35.0,
    obstacle_speed=8.0,
    kappa=None,
    initial_speed=12.0,
):
    if kappa is None:
        kappa = 0 if scenario == "straight" else 1 / 60
    road = CircularRoad(kappa=kappa, half_width=half_width)

    def obstacle_s(t):
        return obstacle_start + obstacle_speed * t

    name = method + "_" + scenario
    start = perf_counter()
    is_reiter = method.startswith("reiter")
    is_proxy = method.startswith("xing_inspired_proxy")
    if is_reiter:
        bus = bus_vehicle()
        # Remove wind/drag/rolling for exact shared acceleration limits a=F/m.
        bus.drag_force_coeff = bus.roll_force_coeff = 0
        options = PlannerOptions(
            n_nodes=settings.intervals + 1,
            time_disc=settings.dt,
            use_lifting="lifted" in method,
            obstacle_model=VehicleObstacleModel.ELIPSE
            if "ellipse" in method
            else VehicleObstacleModel.CIRCLES,
            velocity_set=settings.speed,
            lateral_reference=settings.lateral,
            v_max_terminal_set=settings.max_speed,
            wind_speed=0,
            nlp_solver="SQP",
            nlp_solver_max_iter=150,
            qp_tolerance=1e-6,
            qn=(10.0, 500.0, 1000.0, 1000.0, 10000.0),
            road_margin=0.2,
            obstacle_margin=0.2,
            build_directory=str(output / "build" / name),
        )
        cls = (
            BusReiterPoint
            if method.endswith("_point")
            else BusReiterInflated
            if method.endswith("_inflated")
            else BusReiter
        )
        planner = cls(bus, road, options, [bus])
        state = np.array(
            [0, 0, 0, initial_speed, np.arctan(GEOMETRY.wheelbase * road.kappa)]
        )
        nx = planner.nx
    elif is_proxy:
        model = planner = JointLinearProxy(
            road, settings, obstacle="ELIPSE" if "ellipse" in method else "CIRCLES"
        )
        state, nx = model.initial(), model.nx
    else:
        model = PaperModel(
            road,
            settings,
            lifted=method.startswith("paper9"),
            obstacle="CIRCLES" if "circles" in method else "HYBRID",
            frozen_heading="frozen" in method,
            circle_count=5 if "circles5" in method or "circlesN5" in method else 3,
            circle_covering="paper" if "circlesN" in method else "gym",
            pruned="pruned" in method,
            kappa_as_state="nokappa" not in method,
            prune_keep=1,
            obstacle_constraint="capsule" if "capsule" in method else "components",
            n_disc=N_DISC,
        )
        # "paper6_frozen_sqp" keeps the proposed geometry and only freezes the
        # heading deviation along the contour.
        assert not ("frozen" in method and "circles" in method)
        planner = (
            CilqrPlanner(model, max_inner=40, max_outer=7)
            if method.endswith("cilqr")
            else AcadosPaperPlanner(model, output / "build" / name)
        )
        state, nx = model.initial(initial_speed), model.nx
    setup_seconds = perf_counter() - start
    records, states, controls = [], [state.copy()], []
    min_road, min_obs, collision = float("inf"), float("inf"), False
    alat_values, jlat_values, acc_values, jerk_values, steer_rate_values = (
        [],
        [],
        [],
        [],
        [],
    )
    startup = None
    for k in range(steps):
        time = k * settings.dt
        if is_reiter:
            times = time + np.arange(planner.n_nodes) * settings.dt
            predictions = np.array(
                [[road.numeric_pose([obstacle_s(t), settings.obstacle_lateral, 0]) for t in times]]
            )
            whole_start = perf_counter()
            planner.set_obstacle_predictions(predictions)
            planner.set_states(state)
            if k == 0 and INITIAL_GUESS == "ramp":
                last = planner.n_nodes - 1
                for stage in range(planner.n_nodes):
                    guess = np.array(planner.solver.get(stage, "x"), dtype=float)
                    n = lateral_ramp(stage, last, state[1], settings.lateral)
                    s = state[0] + initial_speed * stage * settings.dt
                    guess[:5] = [s, n, 0.0, initial_speed, state[4]]
                    if planner.planner_opts.use_lifting:
                        guess[5:8] = np.asarray(road.numeric_pose([s, n, 0.0])).ravel()
                    planner.solver.set(stage, "x", guess)
            status = planner.solve()
            d = {
                "status": status,
                "native_solver_ms": planner.computation_time * 1000,
                "solve_wall_ms": (perf_counter() - whole_start) * 1000,
                "iterations": int(planner.solver.get_stats("sqp_iter")),
                "max_violation": planner.diagnostics["max_violation"],
                "dynamics_defect": planner.diagnostics["rk4_defect"],
                "coordinate_error": planner.diagnostics["coordinate_consistency_error"],
            }
            X, U = planner.x_full.T, planner.u_full.T
        else:
            state = model.augment(state)
            parameters = (
                model.parameters(time, state[0])
                if is_proxy
                else model.parameters(time, state[0], obstacle_start, obstacle_speed)
            )
            d = planner.solve(state, parameters)
            X, U = planner.X, planner.U
        if startup is None:
            startup = dict(d)
        d["accepted"] = accepted_status(d["status"]) and d["max_violation"] <= 1e-4
        d["cycle"] = k
        records.append(d)
        if not d["accepted"]:
            (output / (name + "_rejected.json")).write_text(
                json.dumps(
                    finite_json({"states": X, "controls": U, "diagnostics": d}),
                    indent=2,
                    allow_nan=False,
                )
            )
            break
        control = U[0]
        if is_reiter or is_proxy:
            acc, curvature = (
                control[0] / bus.mass if is_reiter else control[0],
                np.tan(state[4]) / GEOMETRY.wheelbase,
            )
            kr = (1 + np.tan(state[4]) ** 2) / GEOMETRY.wheelbase * control[1]
            steer_rate = control[1]
            jerk = (acc - acc_values[-1]) / settings.dt if acc_values else 0.0
            augmented = (
                planner.augment_state(state) if is_reiter else model.augment(state)
            )
        else:
            acc, curvature = state[4], state[5]
            kr, jerk = control[1], control[0]
            steer_rate = GEOMETRY.wheelbase * kr / (1 + (GEOMETRY.wheelbase * curvature) ** 2)
            augmented = state.copy()
        alat_values.append(state[3] ** 2 * curvature)
        jlat_values.append(2 * state[3] * acc * curvature + state[3] ** 2 * kr)
        acc_values.append(acc)
        jerk_values.append(jerk)
        steer_rate_values.append(steer_rate)
        for j in range(5):
            integrator = planner.integrate if is_reiter else model.integrate
            augmented = np.asarray(
                integrator(augmented, control, settings.dt / 5)
            ).ravel()
            t = time + (j + 1) * settings.dt / 5
            sample = evaluate_sample(
                augmented,
                road.numeric_pose([obstacle_s(t), settings.obstacle_lateral, 0]),
                road.kappa,
                road.half_width,
            )
            min_obs = min(min_obs, sample["obstacle_clearance_m"])
            min_road = min(min_road, sample["road_clearance_m"])
            collision |= (
                sample["obstacle_clearance_m"] < -1e-7
                or sample["road_clearance_m"] < -1e-7
            )
        state = augmented[:5] if is_reiter else augmented
        states.append(state.copy())
        controls.append(control.copy())
        planner.shift_guess() if is_reiter else planner.shift()
        if collision:
            break

    def stat(key, percentile=None):
        # Exclude first cold solve and report it separately.
        values = [r[key] for r in records[1:] if r[key] is not None]
        return (
            (
                float(np.mean(values))
                if percentile is None
                else float(np.percentile(values, percentile))
            )
            if values
            else None
        )

    def rms(values):
        return float(np.sqrt(np.mean(np.square(values)))) if values else None

    result = {
        "name": name,
        "method": method,
        "scenario": scenario,
        "curvature": road.kappa,
        "half_width": road.half_width,
        "settings": asdict(settings),
        "nx": nx,
        "nu": 2,
        "N": settings.intervals,
        "decision_variables": nx * (settings.intervals + 1) + 2 * settings.intervals,
        "path_nonlinear_constraints": int(
            planner.h.numel() if is_reiter else model.h.numel()
        ),
        "setup_seconds": setup_seconds,
        "startup": startup,
        "steps_requested": steps,
        "completed_steps": len(controls),
        "success": len(controls) == steps and not collision,
        "collision_or_road_exit": collision,
        "min_obstacle_clearance_m": min_obs,
        "min_road_clearance_m": min_road,
        "progress_m": float(state[0]),
        "final_relative_s_m": float(state[0] - obstacle_s(len(controls) * settings.dt)),
        "passed_obstacle": bool(
            state[0] > obstacle_s(len(controls) * settings.dt) + GEOMETRY.length
        ),
        "rms_lateral_acc_mps2": rms(alat_values),
        "rms_lateral_jerk_mps3": rms(jlat_values),
        "rms_longitudinal_jerk_mps3": rms(jerk_values),
        "max_abs_acc_mps2": max(map(abs, acc_values), default=0),
        "max_abs_longitudinal_jerk_mps3": max(map(abs, jerk_values), default=0),
        "max_abs_steering_rate_radps": max(map(abs, steer_rate_values), default=0),
        "mean_native_solver_ms": stat("native_solver_ms"),
        "p95_native_solver_ms": stat("native_solver_ms", 95),
        "mean_solve_wall_ms": stat("solve_wall_ms"),
        "p95_solve_wall_ms": stat("solve_wall_ms", 95),
        "warm_wall_deadline_miss_fraction": float(
            np.mean([r["solve_wall_ms"] > 100 for r in records[1:]])
        )
        if len(records) > 1
        else None,
        "max_constraint_violation": max(
            (r["max_violation"] for r in records), default=0
        ),
        "max_coordinate_error": max(
            (r["coordinate_error"] for r in records), default=0
        ),
        "states": states,
        "controls": controls,
        "cycles": records,
    }
    result = finite_json(result)
    (output / (name + ".json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps(
            {
                k: result[k]
                for k in [
                    "name",
                    "success",
                    "completed_steps",
                    "min_obstacle_clearance_m",
                    "mean_solve_wall_ms",
                ]
            }
        ),
        flush=True,
    )
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--steps", type=int, default=100)
    p.add_argument("--lateral", type=float, default=5.5)
    p.add_argument("--half-width", type=float, default=8.5)
    p.add_argument("--methods", nargs="+", choices=METHODS, default=METHODS[:8])
    p.add_argument(
        "--scenarios",
        nargs="+",
        choices=["straight", "curve"],
        default=["straight", "curve"],
    )
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.steps < 1 or args.half_width <= GEOMETRY.half_width + 0.2:
        p.error("positive steps and sufficient road width required")
    settings = replace(Settings(), lateral=args.lateral)
    results = []
    for method in args.methods:
        for scenario in args.scenarios:
            target = args.output / f"{method}_{scenario}.json"
            if target.exists():
                raise FileExistsError(f"Refuse to overwrite {target}")
            results.append(
                run_case(
                    args.output, method, scenario, args.steps, settings, args.half_width
                )
            )
    rows = [
        {
            k: v
            for k, v in r.items()
            if k not in ["states", "controls", "cycles", "startup", "settings"]
        }
        for r in results
    ]
    (args.output / "summary.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False) + "\n"
    )
    with (args.output / "summary.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    source_files = [
        *ROOT.glob("paper_comparison/*.py"),
        Path(__file__),
        ROOT / "scripts/run_paper_comparison.sh",
        ROOT / "reiter2023/trajectory_planner_acados_20222.py",
        ROOT / "reiter2023/geometry.py",
    ]
    provenance = {
        "command": sys.argv,
        "python": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "casadi": ca.__version__,
        "numpy": np.__version__,
        "reference_branch": "2.1",
        "reference_commit": "bee02d33c0cb50a5605db98b3cac44b816cdfecf",
        "thread_environment": {
            k: os.environ.get(k)
            for k in [
                "OPENBLAS_NUM_THREADS",
                "OMP_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS",
            ]
        },
        "sha256": {
            str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in source_files
        },
        # The manuscript PDF is not redistributed with the experiment code.
        "paper_sha256": None,
    }
    (args.output / "provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
