"""Compile and validate independently reconstructed Cartesian-obstacle NMPC."""

import argparse
import csv
import json
import os
import platform
import subprocess
import sys
import traceback
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ACADOS_SOURCE_DIR", str(ROOT / "vendor/acados"))
for key in ["OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"]:
    os.environ[key] = "1"
os.environ.setdefault("MPLBACKEND", "Agg")

import casadi as ca
import numpy as np
from vehiclegym.planner.trajectory_planner_base import VehicleObstacleModel
from vehiclegym.utils.evaluation_tools import check_collision_cartesian

from reiter2023 import CircularRoad, PlannerOptions, VehiclePlannerAcados20222
from reiter2023.geometry import body_center
from reiter2023.trajectory_planner_acados_20222 import paper_vehicle


def opponent_predictions(road, time, speed, start_s, nodes, dt):
    times = time + np.arange(nodes) * dt
    return np.array([[road.numeric_pose([start_s + speed * t, 0, 0]) for t in times]])


def collision_and_clearance(road, state, opponent_pose, ego, other):
    pose = road.numeric_pose(state)
    a = np.asarray(body_center(ca.DM(pose), ego)).ravel()
    b = np.asarray(body_center(ca.DM(opponent_pose), other)).ravel()
    collision = bool(
        check_collision_cartesian(
            tuple(a),
            (ego.chassis_length, ego.chassis_width, pose[2]),
            tuple(b),
            (other.chassis_length, other.chassis_width, opponent_pose[2]),
        )
    )
    # Center distance is a diagnostic, not polygon clearance.
    return collision, float(np.linalg.norm(a - b))


def run_case(
    output,
    lifted,
    model,
    scenario,
    steps,
    solver_type="SQP_RTI",
    lateral=0.0,
    rti_iterations=1,
):
    kappa = 0 if scenario == "straight" else 1 / 60
    road = CircularRoad(kappa=kappa)
    vehicle, other = paper_vehicle(), paper_vehicle()
    name = f"{'lifted' if lifted else 'direct'}_{model}_{scenario}"
    options = PlannerOptions(
        use_lifting=lifted,
        n_nodes=41,
        velocity_set=25,
        nlp_solver=solver_type,
        lateral_reference=lateral,
        obstacle_model=getattr(VehicleObstacleModel, model),
        build_directory=str(output / "build" / name),
    )
    planner = VehiclePlannerAcados20222(vehicle, road, options, [other])
    state = np.array([0, 0, 0, 12, np.arctan(vehicle.length * kappa)])
    opponent_speed = 8.0
    opponent_s = 35.0
    planner.set_obstacle_predictions(
        opponent_predictions(
            road, 0, opponent_speed, opponent_s, planner.n_nodes, planner.time_disc
        )
    )
    start = perf_counter()
    warmup = planner.warm_start(state)
    warmup_wall_s = perf_counter() - start
    records, actual, applied = [], [state.tolist()], []
    failures, collision, min_distance = 0, False, float("inf")
    for k in range(steps):
        planner.set_obstacle_predictions(
            opponent_predictions(
                road,
                k * planner.time_disc,
                opponent_speed,
                opponent_s,
                planner.n_nodes,
                planner.time_disc,
            )
        )
        planner.set_states(state)
        iteration_records = []
        for iteration in range(rti_iterations if solver_type == "SQP_RTI" else 1):
            status = planner.solve()
            iteration_records.append(
                {
                    "status": status,
                    "solver_ms": planner.computation_time * 1000,
                    "wall_ms": planner.wall_time_ms,
                    "max_violation": planner.diagnostics["max_violation"],
                }
            )
            if status != 0 or planner.diagnostics["max_violation"] <= 1e-4:
                break
        diagnostic = planner.diagnostics.copy()
        accepted = (
            status == 0 and diagnostic["finite"] and diagnostic["max_violation"] <= 1e-4
        )
        record = {
            "cycle": k,
            "status": status,
            "accepted": accepted,
            "solver_ms": sum(row["solver_ms"] for row in iteration_records),
            "wall_ms": sum(row["wall_ms"] for row in iteration_records),
            "iterations": iteration_records,
            **diagnostic,
        }
        records.append(record)
        if not accepted:
            failures += 1
            break
        control = planner.u_full[:, 0]
        augmented = planner.augment_state(state)
        for substep in range(5):
            augmented = np.asarray(
                planner.integrate(augmented, control, planner.time_disc / 5)
            ).ravel()
            time = (k + (substep + 1) / 5) * planner.time_disc
            other_pose = road.numeric_pose([opponent_s + opponent_speed * time, 0, 0])
            contact, distance = collision_and_clearance(
                road, augmented[:5], other_pose, vehicle, other
            )
            collision |= contact
            min_distance = min(min_distance, distance)
        state = augmented[:5].copy()
        actual.append(state.tolist())
        applied.append(control.tolist())
        planner.shift_guess()
        if collision:
            break
    valid_warmup = (
        warmup["statuses"][-1] == 0 and warmup["diagnostics"]["max_violation"] <= 1e-4
    )
    timings = [row["solver_ms"] for row in records]
    result = {
        "name": name,
        "implementation": "independent_paper_reconstruction",
        "original_author_class": False,
        "full_paper_benchmark_reproduced": False,
        "scenario": {
            "curvature": kappa,
            "steps_requested": steps,
            "dt": planner.time_disc,
            "obstacle_start_s": opponent_s,
            "obstacle_s_speed": opponent_speed,
        },
        "options": {
            "formulation": "lifted_ode" if lifted else "direct_elimination",
            "obstacle_model": model,
            "solver": options.nlp_solver,
            "wind_speed": options.wind_speed,
            "lateral_reference": lateral,
            "max_rti_iterations_per_cycle": rti_iterations,
        },
        "dimensions": {
            "nx": planner.nx,
            "nu": planner.nu,
            "intervals": planner.N,
            "state_and_control_variables": planner.nx * planner.n_nodes
            + planner.nu * planner.N,
            "path_nonlinear_constraints": planner.h.numel(),
            "terminal_nonlinear_constraints": planner.h_terminal.numel(),
            "slacks": 0,
        },
        "build_seconds": planner.build_seconds,
        "warmup_wall_s": warmup_wall_s,
        "warmup": warmup,
        "cycles": records,
        "actual_frenet_states": actual,
        "applied_controls": applied,
        "success": bool(
            valid_warmup and failures == 0 and not collision and len(applied) == steps
        ),
        "solver_failures": failures,
        "collision": collision,
        "min_center_distance_m": min_distance if np.isfinite(min_distance) else None,
        "mean_solver_ms": float(np.mean(timings)) if timings else None,
        "p95_solver_ms": float(np.percentile(timings, 95)) if timings else None,
    }
    (output / f"{name}.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps(
            {
                key: result[key]
                for key in [
                    "name",
                    "success",
                    "solver_failures",
                    "collision",
                    "mean_solver_ms",
                ]
            }
        ),
        flush=True,
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--solver", choices=["SQP_RTI", "SQP"], default="SQP_RTI")
    parser.add_argument("--lateral-reference", type=float, default=0.0)
    parser.add_argument(
        "--rti-iterations",
        type=int,
        default=1,
        help="Explicit feasibility refinements; >1 is not paper single-step RTI",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "results/reconstructed")
    parser.add_argument(
        "--models",
        nargs="+",
        choices=["ELIPSE", "CIRCLES"],
        default=["ELIPSE", "CIRCLES"],
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        choices=["straight", "curve"],
        default=["straight", "curve"],
    )
    parser.add_argument(
        "--formulations",
        nargs="+",
        choices=["direct", "lifted"],
        default=["direct", "lifted"],
    )
    args = parser.parse_args()
    if args.steps < 1 or args.rti_iterations < 1:
        parser.error("--steps and --rti-iterations must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    results, exceptions = [], []
    for formulation in args.formulations:
        for model in args.models:
            for scenario in args.scenarios:
                try:
                    results.append(
                        run_case(
                            args.output,
                            formulation == "lifted",
                            model,
                            scenario,
                            args.steps,
                            args.solver,
                            args.lateral_reference,
                            args.rti_iterations,
                        )
                    )
                except (
                    RuntimeError,
                    ValueError,
                    OSError,
                    TypeError,
                    AttributeError,
                ) as error:
                    traceback.print_exc()
                    exceptions.append(
                        {
                            "formulation": formulation,
                            "model": model,
                            "scenario": scenario,
                            "error": repr(error),
                        }
                    )
    report = {
        "kind": "reconstructed_acados_smoke_test",
        "platform": platform.platform(),
        "python": sys.version,
        "casadi": ca.__version__,
        "acados_commit": subprocess.check_output(
            ["git", "-C", str(ROOT / "vendor/acados"), "rev-parse", "HEAD"], text=True
        ).strip(),
        "complete_paper_reproduction": False,
        "results": [
            {
                k: r[k]
                for k in [
                    "name",
                    "success",
                    "solver_failures",
                    "collision",
                    "mean_solver_ms",
                    "p95_solver_ms",
                    "dimensions",
                ]
            }
            for r in results
        ],
        "exceptions": exceptions,
        "all_smoke_cases_passed": not exceptions
        and bool(results)
        and all(r["success"] for r in results),
    }
    (args.output / "summary.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    if results:
        with (args.output / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "name",
                    "success",
                    "solver_failures",
                    "collision",
                    "mean_solver_ms",
                    "p95_solver_ms",
                ],
            )
            writer.writeheader()
            writer.writerows(
                {key: row[key] for key in writer.fieldnames} for row in results
            )
    return 0 if report["all_smoke_cases_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
