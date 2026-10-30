"""Frozen scenario matrix with uniform planning-call timing for local revision.

This adds a five-circle geometry ablation without changing historical models.
Every method keeps its declared objective, dynamics and solver. Uniform timing
does not turn this into a controlled cross-method algorithm-speed experiment.
"""

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
from run_paper_comparison import BusReiter, finite_json
from vehiclegym.planner.trajectory_planner_base import VehicleObstacleModel

from paper_comparison.cilqr import CilqrPlanner
from paper_comparison.evaluation import evaluate_sample
from paper_comparison.model import (
    AcadosPaperPlanner,
    PaperModel,
    Settings,
    bus_vehicle,
    gym_pose,
)
from paper_comparison.xing2022 import XingOptions, XingPlanner
from reiter2023 import CircularRoad, PlannerOptions
from reiter2023.geometry import covering_circles

METHODS = (
    "reiter_direct_ellipse",
    "reiter_lifted_ellipse",
    "paper6_hybrid_sqp",
    "paper6_circles_sqp",
    "paper6_five_circles_sqp",
    "paper6_hybrid_cilqr",
    "xing_joint_corridor",
)


class FiveCircleModel(PaperModel):
    """Only the circle count differs from the preserved three-circle model."""

    def _symbolic(self):
        super()._symbolic()
        pose = self.road.pose(self.x[0], self.x[1], self.x[2])
        aa, ra = covering_circles(gym_pose(pose), self.vehicle, 5)
        bb, rb = covering_circles(gym_pose(self.p[:3]), self.vehicle, 5)
        obs = ca.vertcat(
            *[
                ca.sqrt(ca.sumsqr(a - b) + 1e-16)
                - ra
                - rb
                - self.settings.obstacle_margin
                for a in aa
                for b in bb
            ]
        )
        self.h = ca.vertcat(self.h[:8], obs)
        self.hl = np.r_[self.hl[:8], np.zeros(obs.numel())]
        self.hu = np.r_[self.hu[:8], np.full(obs.numel(), 1e8)]
        self.hfun = ca.Function("five_circle_h", [self.x, self.u, self.p], [self.h])
        self.g = ca.vertcat(
            self.hl - self.h,
            self.h - self.hu,
            self.xl - self.x[self.state_indices],
            self.x[self.state_indices] - self.xu,
        )
        self.gfun = ca.Function("five_circle_g", [self.x, self.u, self.p], [self.g])


def specification():
    scenes = []
    for kappa in (0.0, 1 / 80, 1 / 40):
        for half_width in (4.3, 4.6, 5.2, 8.5):
            scenes.append(
                {
                    "id": f"r{'inf' if kappa == 0 else int(1 / kappa)}_w{half_width}",
                    "curvature": kappa,
                    "half_width": half_width,
                    "lateral_target": 5.5 if half_width == 8.5 else half_width - 2,
                }
            )
    return {
        "version": 1,
        "purpose": "Broaden deterministic coverage; uniform implementation timing; three/five-circle ablation",
        "methods": METHODS,
        "scenarios": scenes,
        "settings": asdict(Settings()),
        "steps": 120,
        "initial_speed_mps": 12,
        "obstacle_s": "35 + 8*t",
        "obstacle_lateral_m": 0,
        "acceptance_residual": 1e-4,
        "safety_sampling_seconds": 0.02,
        "retuning_after_inspecting_results": False,
        "method_order": "cyclic rotation by scenario index; cases run sequentially",
        "timer": "parameters/references and initial-state updates, solve, output retrieval, built-in feasibility diagnostics, accepted-output copy, warm-start shift",
        "excluded_from_timer": "model/code generation, executed plant propagation, independent geometry evaluation, full forecast serialization and disk I/O",
        "limits": [
            "Deterministic stress matrix, not extracted field scenarios or independent random trials",
            "Width and lateral target vary jointly",
            "Objectives, inputs, terminal conditions, backend and optimality criteria differ across methods",
            "AL-CiLQR is an independent Python adaptation, not the deployed C++ implementation",
            "Hybrid uses all 25 component pairs, not the manuscript pruning algorithm",
            "No fallback or retuning after failure",
        ],
    }


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


def make_planner(method, road, settings, build):
    if method.startswith("reiter"):
        bus = bus_vehicle()
        bus.drag_force_coeff = bus.roll_force_coeff = 0
        opts = PlannerOptions(
            n_nodes=settings.intervals + 1,
            time_disc=settings.dt,
            use_lifting="lifted" in method,
            obstacle_model=VehicleObstacleModel.ELIPSE,
            velocity_set=settings.speed,
            lateral_reference=settings.lateral,
            v_max_terminal_set=settings.max_speed,
            wind_speed=0,
            nlp_solver="SQP",
            nlp_solver_max_iter=150,
            qp_tolerance=1e-6,
            qn=(10, 500, 1000, 1000, 10000),
            road_margin=settings.road_margin,
            obstacle_margin=settings.obstacle_margin,
            build_directory=str(build),
        )
        planner = BusReiter(bus, road, opts, [bus])
        return planner, np.array([0, 0, 0, 12, np.arctan(4.32 * road.kappa)])
    if method.startswith("xing"):
        planner = XingPlanner(road, settings, XingOptions())
        return planner, planner.initial()
    cls = FiveCircleModel if "five_circles" in method else PaperModel
    model = cls(
        road, settings, obstacle="CIRCLES" if "circles" in method else "HYBRID"
    )
    planner = (
        CilqrPlanner(model, max_inner=40, max_outer=7)
        if method.endswith("cilqr")
        else AcadosPaperPlanner(model, build)
    )
    return planner, model.initial()


def canonical(method, state, road):
    if method.startswith("xing"):
        return XingPlanner.canonical(state)
    if method.startswith("reiter"):
        s, n, alpha, speed, delta = state[:5]
        return np.array([s, n, alpha, speed, np.tan(delta) / 4.32])
    return np.asarray(state)[:6]


def planning_call(method, planner, state, time, settings, road):
    """Common timer excludes the independent evaluator for every implementation."""
    start = perf_counter()
    if method.startswith("xing"):
        diagnostics = planner.solve(state, time)
        X, U = planner.X, planner.U
    elif method.startswith("reiter"):
        times = time + np.arange(planner.n_nodes) * settings.dt
        predictions = np.array(
            [[road.numeric_pose([35 + 8 * t, 0, 0]) for t in times]]
        )
        planner.set_obstacle_predictions(predictions)
        planner.set_states(state)
        status = planner.solve()
        X, U = planner.x_full.T, planner.u_full.T
        diagnostics = {
            "status": int(status),
            "native_solver_ms": planner.computation_time * 1000,
            "iterations": int(planner.solver.get_stats("sqp_iter")),
            "max_violation": planner.diagnostics["max_violation"],
            "coordinate_error": planner.diagnostics["coordinate_consistency_error"],
            "dynamics_defect": planner.diagnostics["rk4_defect"],
        }
    else:
        initial = planner.model.augment(state)
        parameters = planner.model.parameters(time, state[0])
        diagnostics = planner.solve(initial, parameters)
        X, U = planner.X, planner.U
    finite = np.isfinite(X).all() and np.isfinite(U).all()
    finite = finite and all(
        np.isfinite(diagnostics[key])
        for key in ("max_violation", "coordinate_error", "dynamics_defect")
    )
    accepted = bool(
        diagnostics["status"] == 0
        and finite
        and diagnostics["max_violation"] <= 1e-4
        and diagnostics["coordinate_error"] <= 1e-4
    )
    control = U[0].copy() if accepted else None
    if accepted:
        planner.shift_guess() if method.startswith("reiter") else planner.shift()
    diagnostics["planning_call_ms"] = 1000 * (perf_counter() - start)
    diagnostics["accepted"] = accepted
    return diagnostics, control, X.copy(), U.copy()


def write_json(path, value):
    path.write_text(json.dumps(finite_json(value), allow_nan=False, indent=2) + "\n")


def summarize_times(cycles):
    output = {}
    for phase, values in (("all", cycles), ("warm", cycles[1:])):
        times = np.array([c["planning_call_ms"] for c in values], dtype=float)
        for label, operation in (
            ("mean", np.mean),
            ("p95", lambda t: np.percentile(t, 95)),
            ("p99", lambda t: np.percentile(t, 99)),
            ("max", np.max),
        ):
            output[f"{phase}_{label}_planning_call_ms"] = (
                float(operation(times)) if len(times) else None
            )
        output[f"{phase}_deadline_miss_fraction"] = (
            float(np.mean(times > 100)) if len(times) else None
        )
        output[f"{phase}_calls"] = len(times)
    return output


def run_case(output, method, scene, steps):
    settings = replace(Settings(), lateral=scene["lateral_target"])
    road = CircularRoad(scene["curvature"], scene["half_width"])
    name = method + "_" + scene["id"]
    started = perf_counter()
    planner, state = make_planner(method, road, settings, output / "build" / name)
    setup = perf_counter() - started
    states, controls, cycles, predictions = [state.copy()], [], [], []
    obs_min, road_min = np.inf, np.inf
    stop_reason = "completed"
    for cycle in range(steps):
        d, control, X, U = planning_call(
            method, planner, state, cycle * settings.dt, settings, road
        )
        d["cycle"] = cycle
        cycles.append(d)
        predictions.append({"cycle": cycle, "accepted": d["accepted"], "X": X, "U": U})
        if not d["accepted"]:
            stop_reason = "planner_rejected"
            break
        if method.startswith("reiter"):
            state = planner.augment_state(state)
        for sub in range(1, 6):
            if method.startswith("xing"):
                state = planner.integrate(state, control, settings.dt / 5)
            else:
                integrator = (
                    planner.integrate
                    if method.startswith("reiter")
                    else planner.model.integrate
                )
                state = np.asarray(
                    integrator(state, control, settings.dt / 5)
                ).ravel()
            check = evaluate_sample(
                canonical(method, state, road),
                road.numeric_pose([35 + 8 * (cycle + sub / 5) * settings.dt, 0, 0]),
                road.kappa,
                road.half_width,
            )
            obs_min = min(obs_min, check["obstacle_clearance_m"])
            road_min = min(road_min, check["road_clearance_m"])
        if method.startswith("reiter"):
            state = state[:5]
        states.append(state.copy())
        controls.append(control.copy())
        if min(obs_min, road_min) < -1e-7:
            stop_reason = "sampled_collision_or_road_exit"
            break
    progress = float(canonical(method, state, road)[0])
    relative_s = progress - (35 + 8 * len(controls) * settings.dt)
    summary = {
        "name": name,
        "method": method,
        "scenario": scene["id"],
        **scene,
        "settings": asdict(settings),
        "setup_seconds": setup,
        "completed_steps": len(controls),
        "steps_requested": steps,
        "success": len(controls) == steps and stop_reason == "completed",
        "stop_reason": stop_reason,
        "progress_m": progress,
        "final_relative_s_m": relative_s,
        "passed_obstacle": relative_s > 6.5,
        "min_obstacle_clearance_m": obs_min,
        "min_road_clearance_m": road_min,
        "sampled_margin_0p2_satisfied": bool(min(obs_min, road_min) >= 0.2 - 1e-4),
        "warm_rejected_calls": sum(not c["accepted"] for c in cycles[1:]),
        **summarize_times(cycles),
    }
    write_json(
        output / (name + ".json"),
        {**summary, "states": states, "controls": controls, "cycles": cycles},
    )
    write_json(output / (name + "_forecasts.json"), predictions)
    print(
        json.dumps(
            {
                k: summary[k]
                for k in (
                    "name",
                    "success",
                    "completed_steps",
                    "progress_m",
                    "warm_mean_planning_call_ms",
                )
            }
        ),
        flush=True,
    )
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=METHODS)
    parser.add_argument("--scene-ids", nargs="+")
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--spec-only", action="store_true")
    args = parser.parse_args()
    if args.steps < 1 or args.steps > 120:
        raise ValueError("Use 1..120 steps")
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("Use an empty new output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    spec = specification()
    if args.scene_ids:
        unknown = set(args.scene_ids) - {s["id"] for s in spec["scenarios"]}
        if unknown:
            raise ValueError(f"Unknown scenarios: {unknown}")
    write_json(args.output / "frozen_protocol.json", spec)
    before = source_hashes()
    write_json(
        args.output / "provenance.json",
        {
            "source_sha256": before,
            "protocol_sha256": hashlib.sha256(
                (args.output / "frozen_protocol.json").read_bytes()
            ).hexdigest(),
            "platform": platform.platform(),
            "python": sys.version,
            "casadi": ca.__version__,
            "numpy": np.__version__,
            "threads": {
                k: os.getenv(k)
                for k in (
                    "OPENBLAS_NUM_THREADS",
                    "OMP_NUM_THREADS",
                    "VECLIB_MAXIMUM_THREADS",
                )
            },
            "actual_steps": args.steps,
            "actual_methods": args.methods,
            "actual_scene_ids": args.scene_ids,
        },
    )
    if args.spec_only:
        return
    summaries = []
    for index, scene in enumerate(spec["scenarios"]):
        if args.scene_ids and scene["id"] not in args.scene_ids:
            continue
        order = list(args.methods)
        rotation = index % len(order)
        for method in order[rotation:] + order[:rotation]:
            summaries.append(run_case(args.output, method, scene, args.steps))
            write_json(args.output / "summary.json", summaries)
    with (args.output / "summary.csv").open("w", newline="") as stream:
        rows = [{k: v for k, v in r.items() if k != "settings"} for r in summaries]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(finite_json(rows))
    if source_hashes() != before:
        raise RuntimeError("Experiment source changed during execution")
    print(f"Completed {len(summaries)} cases; source hashes unchanged.", flush=True)


if __name__ == "__main__":
    main()
