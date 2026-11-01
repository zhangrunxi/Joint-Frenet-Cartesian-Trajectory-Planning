"""Run source-verified Xing reconstruction and all preserved formal baselines."""

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
sys.path.insert(0, str(ROOT / "scripts"))

import casadi as ca
import numpy as np
import osqp
from run_paper_comparison import METHODS, finite_json, run_case

from paper_comparison.evaluation import evaluate_sample
from paper_comparison.model import Settings
from paper_comparison.vehicle import GEOMETRY
from paper_comparison.xing2022 import XingOptions, XingPlanner
from reiter2023 import CircularRoad
from paper_comparison.harness_options import accepted_status

FINAL_METHODS = METHODS[:8] + ["xing_joint_corridor", "xing_joint_yield"]
# Published reference-point road bounds; only for road-contour ablations.
POINT_METHODS = [
    "reiter_direct_ellipse_point",
    "reiter_lifted_ellipse_point",
    "xing_joint_corridor_point",
]


def run_xing(
    output,
    method,
    scenario,
    steps,
    settings,
    half_width,
    max_iterations=50,
    obstacle_start=35.0,
    obstacle_speed=8.0,
    kappa=None,
    initial_speed=12.0,
):
    if kappa is None:
        kappa = 0 if scenario == "straight" else 1 / 60
    road = CircularRoad(kappa, half_width)
    options = XingOptions(
        decision="yield" if method.endswith("yield") else "left",
        max_iterations=max_iterations,
        obstacle_start=obstacle_start,
        obstacle_speed=obstacle_speed,
        road_bound="point" if method.endswith("_point") else "contour",
        initial_speed=initial_speed,
    )

    def obstacle_s(t):
        return obstacle_start + obstacle_speed * t

    started = perf_counter()
    planner = XingPlanner(road, settings, options)
    setup = perf_counter() - started
    state = planner.initial()
    states, controls, cycles = [planner.canonical(state)], [], []
    original_states = [state.copy()]
    predictions = []
    min_obs, min_road, collision = np.inf, np.inf, False
    accelerations, alats, jerks, jlates, rates, interval_alats = [], [], [], [], [], []
    name = method + "_" + scenario
    for j in range(steps):
        previous = planner.previous_input.copy()
        d = planner.solve(state, j * settings.dt)
        d["accepted"] = bool(
            accepted_status(d["status"])
            and d["max_violation"] <= 1e-4
            and np.isfinite(planner.X).all()
            and np.isfinite(planner.U).all()
            and all(
                np.isfinite(d[key])
                for key in ("max_violation", "coordinate_error", "dynamics_defect")
            )
        )
        d["cycle"] = j
        cycles.append(d)
        if not d["accepted"]:
            (output / (name + "_rejected.json")).write_text(
                json.dumps(
                    finite_json(
                        {
                            "states": np.array(
                                [planner.canonical(x) for x in planner.X]
                            ),
                            "original_order_states": planner.X,
                            "controls": planner.U,
                            "diagnostics": d,
                        }
                    ),
                    indent=2,
                    allow_nan=False,
                )
            )
            break
        predictions.append(
            {
                "cycle": j,
                "time_s": j * settings.dt,
                "previous_applied_input": previous,
                "original_order_states": planner.X.copy(),
                "controls": planner.U.copy(),
                "final_recomputed_corridor": planner.corridor(
                    planner.X, j * settings.dt, recompute=True
                ),
            }
        )
        u = planner.U[0].copy()
        accelerations.append(float(u[1]))
        alat = state[0] ** 2 * np.tan(u[0]) / planner.vehicle.length
        alats.append(float(alat))
        interval_alats.extend(
            [
                abs(float(alat)),
                abs(
                    float(
                        (state[0] + settings.dt * u[1]) ** 2
                        * np.tan(u[0])
                        / planner.vehicle.length
                    )
                ),
            ]
        )
        jerks.append(float((u[1] - previous[1]) / settings.dt))
        rates.append(float((u[0] - previous[0]) / settings.dt))
        jlates.append(float((alat - alats[-2]) / settings.dt) if len(alats) > 1 else 0)
        for sub in range(5):
            state = planner.integrate(state, u, settings.dt / 5)
            t = (j + (sub + 1) / 5) * settings.dt
            evaluation = evaluate_sample(
                planner.canonical(state),
                road.numeric_pose([obstacle_s(t), settings.obstacle_lateral, 0]),
                road.kappa,
                half_width,
            )
            min_obs = min(min_obs, evaluation["obstacle_clearance_m"])
            min_road = min(min_road, evaluation["road_clearance_m"])
            collision |= min_obs < -1e-7 or min_road < -1e-7
        controls.append(u)
        states.append(planner.canonical(state))
        original_states.append(state.copy())
        planner.shift()
        if collision:
            break

    def stat(key, percentile=None):
        values = [c[key] for c in cycles[1:] if c[key] is not None]
        if not values:
            return None
        return float(
            np.mean(values) if percentile is None else np.percentile(values, percentile)
        )

    def rms(values):
        return float(np.sqrt(np.mean(np.square(values)))) if values else None

    result = {
        "name": name,
        "method": method,
        "scenario": scenario,
        "curvature": road.kappa,
        "half_width": half_width,
        "settings": asdict(settings),
        "effective_configuration": planner.configuration(),
        "nx": 7,
        "nu": 2,
        "N": settings.intervals,
        "decision_variables": planner.nz,
        "path_nonlinear_constraints": 0,
        "constraint_note": "Linear QP constraints are rebuilt from nonlinear models each iteration",
        "stored_state_order": ["S", "D", "phi", "V", "X", "Y", "psi"],
        "control_order": ["delta", "a"],
        "setup_seconds": setup,
        "startup": cycles[0],
        "steps_requested": steps,
        "completed_steps": len(controls),
        "success": len(controls) == steps and not collision,
        "collision_or_road_exit": bool(collision),
        "min_obstacle_clearance_m": min_obs,
        "min_road_clearance_m": min_road,
        "progress_m": float(state[4]),
        "final_relative_s_m": float(state[4] - obstacle_s(len(controls) * settings.dt)),
        "passed_obstacle": bool(
            state[4] > obstacle_s(len(controls) * settings.dt) + GEOMETRY.length
        ),
        "rms_lateral_acc_mps2": rms(alats),
        "max_abs_lateral_acceleration_mps2": max(interval_alats, default=0),
        "rms_lateral_jerk_mps3": rms(jlates),
        "rms_longitudinal_jerk_mps3": rms(jerks),
        "comfort_metric_note": "Input increments and lateral acceleration finite differences; not continuous jerk of instantaneous ZOH commands",
        "max_abs_acc_mps2": max(map(abs, accelerations), default=0),
        "max_abs_longitudinal_jerk_mps3": max(map(abs, jerks), default=0),
        "max_abs_steering_rate_radps": max(map(abs, rates), default=0),
        "mean_native_solver_ms": None,
        "p95_native_solver_ms": None,
        "mean_solve_wall_ms": stat("solve_wall_ms"),
        "p95_solve_wall_ms": stat("solve_wall_ms", 95),
        "mean_qp_kernel_sum_ms": stat("qp_kernel_sum_ms"),
        "warm_wall_deadline_miss_fraction": float(
            np.mean([c["solve_wall_ms"] > 100 for c in cycles[1:]])
        )
        if len(cycles) > 1
        else None,
        "max_constraint_violation": max(c["max_violation"] for c in cycles),
        "max_coordinate_error": max(c["coordinate_error"] for c in cycles),
        "states": states,
        "original_order_states": original_states,
        "controls": controls,
        "cycles": cycles,
    }
    result = finite_json(result)
    (output / (name + ".json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    (output / (name + "_predictions.json")).write_text(
        json.dumps(finite_json({"accepted_predictions": predictions}), allow_nan=False)
        + "\n"
    )
    print(
        json.dumps(
            {
                k: result[k]
                for k in (
                    "name",
                    "completed_steps",
                    "success",
                    "progress_m",
                    "mean_solve_wall_ms",
                )
            }
        ),
        flush=True,
    )
    return result


def source_hashes():
    paths = [
        *ROOT.glob("paper_comparison/*.py"),
        *ROOT.glob("reiter2023/*.py"),
        ROOT / "scripts/run_paper_comparison.py",
        Path(__file__),
    ]
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in paths
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--steps", type=int, default=120)
    p.add_argument("--half-width", type=float, default=8.5)
    p.add_argument("--lateral", type=float, default=5.5)
    p.add_argument("--intervals", type=int, default=40)
    p.add_argument("--max-xing-iterations", type=int, default=50)
    p.add_argument("--methods", nargs="+", choices=FINAL_METHODS, default=FINAL_METHODS)
    p.add_argument(
        "--scenarios",
        nargs="+",
        choices=["straight", "curve"],
        default=["straight", "curve"],
    )
    args = p.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("Use an empty new output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    before = source_hashes()
    (args.output / "source_hashes_at_start.json").write_text(
        json.dumps(before, indent=2) + "\n"
    )
    settings = replace(Settings(), lateral=args.lateral, intervals=args.intervals)
    results = []
    for method in args.methods:
        for scenario in args.scenarios:
            if method.startswith("xing_joint"):
                result = run_xing(
                    args.output,
                    method,
                    scenario,
                    args.steps,
                    settings,
                    args.half_width,
                    args.max_xing_iterations,
                )
            else:
                result = run_case(
                    args.output, method, scenario, args.steps, settings, args.half_width
                )
            results.append(result)
    rows = [
        {
            k: v
            for k, v in case.items()
            if k
            not in (
                "states",
                "original_order_states",
                "controls",
                "cycles",
                "startup",
                "settings",
                "effective_configuration",
            )
        }
        for case in results
    ]
    (args.output / "summary.json").write_text(
        json.dumps(rows, indent=2, allow_nan=False) + "\n"
    )
    columns = sorted(set().union(*(r.keys() for r in rows)))
    with (args.output / "summary.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    assert source_hashes() == before, "Sources changed during benchmark"
    # The Xing paper PDF is copyrighted and is not redistributed. When it is
    # absent the recorded digest is null and the corresponding check is skipped.
    paper = ROOT / "sources" / "Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf"
    if not paper.exists():
        paper = (
            ROOT.parents[1] / "Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf"
        )
    provenance = {
        "command": sys.argv,
        "python": sys.version,
        "platform": platform.platform(),
        "casadi": ca.__version__,
        "numpy": np.__version__,
        "osqp": osqp.__version__,
        "sha256": before,
        "xing_paper_sha256": (
            hashlib.sha256(paper.read_bytes()).hexdigest() if paper.exists() else None
        ),
        "xing_paper_available": paper.exists(),
        "thread_environment": {
            k: os.environ.get(k)
            for k in (
                "OPENBLAS_NUM_THREADS",
                "OMP_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS",
            )
        },
        "all_calls_sequential": True,
        "source_verified_equation_reconstruction": True,
        "author_implementation": False,
    }
    (args.output / "provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
