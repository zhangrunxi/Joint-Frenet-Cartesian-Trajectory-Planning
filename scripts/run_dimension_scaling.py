"""Sequential fixed-state HYBRID SQP replay: six versus nine states, N=20/40/80."""

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import sys
from dataclasses import asdict, replace
from pathlib import Path
from time import perf_counter, sleep, time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import casadi as ca
import numpy as np

from paper_comparison.model import AcadosPaperPlanner, PaperModel, Settings
from reiter2023.geometry import CircularRoad


def wait_for_idle():
    """Wait through chained comparison jobs, including their parent shell."""
    quiet_since = None
    announced = False
    while True:
        snapshot = subprocess.run(
            ["ps", "-axo", "pid=,command="], check=True, capture_output=True, text=True
        ).stdout
        active = [
            line.strip()
            for line in snapshot.splitlines()
            if "run_paper_comparison" in line
        ]
        if active:
            quiet_since = None
            if not announced:
                print(
                    "Waiting for run_paper_comparison jobs to finish:\n"
                    + "\n".join(active),
                    flush=True,
                )
                announced = True
        elif quiet_since is None:
            quiet_since = perf_counter()
        elif perf_counter() - quiet_since >= 5:
            return {
                "checked_unix_s": time(),
                "quiet_seconds": 5,
                "matching_comparison_processes": [],
            }
        sleep(1)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean(value), indent=2, allow_nan=False) + "\n")


def write_csv(path, records):
    if not records:
        return
    columns = list(dict.fromkeys(key for record in records for key in record))
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(clean(records))


def load_replays(directories, following):
    replays = []
    for directory in directories:
        for scenario in ("straight", "curve"):
            path = directory.resolve() / f"paper6_hybrid_sqp_{scenario}.json"
            record = json.loads(path.read_text())
            if (
                record.get("method") != "paper6_hybrid_sqp"
                or record.get("scenario") != scenario
            ):
                raise ValueError(f"Not a six-state HYBRID reference: {path}")
            states = np.asarray(record["states"], dtype=float)
            if states.ndim != 2 or states.shape[1] != 6 or len(states) < following + 1:
                raise ValueError(
                    f"Need initial plus {following} six-state samples: {path}"
                )
            states = states[: following + 1].copy()
            if not np.isfinite(states).all():
                raise ValueError(f"Nonfinite replay states: {path}")
            settings = Settings(
                **{
                    **record["settings"],
                    "weights": tuple(record["settings"]["weights"]),
                    "terminal_weights": tuple(record["settings"]["terminal_weights"]),
                }
            )
            if settings.dt != 0.1:
                raise ValueError(f"Reference dt must be 0.1: {path}")
            replays.append(
                {
                    "group": directory.name,
                    "scenario": scenario,
                    "path": path,
                    "sha256": digest(path),
                    "states": states,
                    "settings": settings,
                    "kappa": record["curvature"],
                    "half_width": record["half_width"],
                }
            )
    if len({(r["group"], r["scenario"]) for r in replays}) != len(replays):
        raise ValueError("Input dataset basenames must be distinct")
    return replays


def reset(planner):
    planner.solver.reset(reset_qp_solver_mem=True)
    planner.X, planner.U = None, None


def accepted(diagnostics, tolerance):
    return bool(
        diagnostics["status"] == 0
        and np.isfinite(diagnostics["max_violation"])
        and diagnostics["max_violation"] <= tolerance
        and np.isfinite(diagnostics["coordinate_error"])
        and diagnostics["coordinate_error"] <= tolerance
    )


def statistics(records, field, prefix):
    values = [
        record[field]
        for record in records
        if record.get(field) is not None and np.isfinite(record[field])
    ]
    return {
        prefix + "_mean": float(np.mean(values)) if values else None,
        prefix + "_p95": float(np.percentile(values, 95)) if values else None,
        prefix + "_max": float(np.max(values)) if values else None,
    }


def run_case(replay, horizon, repeats, output, tolerance, journal):
    name = f"{replay['group']}_{replay['scenario']}_N{horizon}"
    settings = replace(replay["settings"], intervals=horizon, dt=0.1)
    road = CircularRoad(kappa=replay["kappa"], half_width=replay["half_width"])
    planners, build = {}, []
    for nx in (6, 9):
        start = perf_counter()
        model = PaperModel(road, settings, lifted=nx == 9, obstacle="HYBRID")
        planner = AcadosPaperPlanner(model, output / "build" / name / f"own{nx}")
        planners[nx] = planner
        build.append(
            {
                "nx": nx,
                "setup_seconds": perf_counter() - start,
                "solver_build_seconds": planner.build_seconds,
            }
        )
    base = {
        "group": replay["group"],
        "scenario": replay["scenario"],
        "N": horizon,
        "dt": settings.dt,
        "horizon_seconds": horizon * settings.dt,
        "half_width": road.half_width,
        "lateral": settings.lateral,
    }
    calls, pairs = [], []
    total_calls = {6: 0, 9: 0}
    for repeat in range(repeats):
        for planner in planners.values():
            reset(planner)
        for sample, state in enumerate(replay["states"]):
            time = sample * settings.dt
            parameters = planners[6].model.parameters(time, state[0])
            np.testing.assert_array_equal(
                parameters, planners[9].model.parameters(time, state[0])
            )
            results, controls, trajectories = {}, {}, {}
            # Alternating dimension order reduces systematic first/second timing bias.
            order = (6, 9) if (repeat + sample) % 2 == 0 else (9, 6)
            for position, nx in enumerate(order):
                planner = planners[nx]
                rollout_guess = planner.X is None
                initial = planner.model.augment(state)
                # Fail closed if a comparison starts after the initial quiet check.
                snapshot = subprocess.run(
                    ["ps", "-axo", "pid=,command="],
                    check=True,
                    capture_output=True,
                    text=True,
                ).stdout
                if "run_paper_comparison" in snapshot:
                    wait_for_idle()
                diagnostics = planner.solve(initial, parameters)
                valid = accepted(diagnostics, tolerance)
                entry = dict(
                    **base,
                    repeat=repeat,
                    sample=sample,
                    replay_time_s=time,
                    nx=nx,
                    nu=2,
                    decision_variables=nx * (horizon + 1) + 2 * horizon,
                    path_nonlinear_constraints=int(planner.model.h.numel()),
                    phase="cold" if sample == 0 else "warm",
                    solver_first_call=total_calls[nx] == 0,
                    zero_control_rollout_guess=rollout_guess,
                    pair_order=position,
                    accepted=valid,
                    **diagnostics,
                )
                entry.update(
                    {
                        f"initial_{key}": float(value)
                        for key, value in zip(
                            ("s", "n", "alpha", "v", "a", "curvature"), state
                        )
                    }
                )
                entry.update(
                    first_jerk=float(planner.U[0, 0]),
                    first_curvature_rate=float(planner.U[0, 1]),
                )
                calls.append(clean(entry))
                journal.write(json.dumps(clean(entry), allow_nan=False) + "\n")
                journal.flush()
                total_calls[nx] += 1
                results[nx] = entry
                controls[nx], trajectories[nx] = (
                    planner.U.copy(),
                    planner.X[:, :6].copy(),
                )
                if valid:
                    planner.shift()
                else:
                    # Recover deterministically rather than reuse a failed iterate.
                    reset(planner)
            delta = controls[9] - controls[6]
            pair = dict(
                **base,
                repeat=repeat,
                sample=sample,
                phase="cold" if sample == 0 else "warm",
                both_accepted=results[6]["accepted"] and results[9]["accepted"],
                own6_accepted=results[6]["accepted"],
                own9_accepted=results[9]["accepted"],
                own6_native_ms=results[6]["native_solver_ms"],
                own9_native_ms=results[9]["native_solver_ms"],
                first_jerk_delta=float(delta[0, 0]),
                first_curvature_rate_delta=float(delta[0, 1]),
                horizon_jerk_delta_max=float(np.max(np.abs(delta[:, 0]))),
                horizon_curvature_rate_delta_max=float(np.max(np.abs(delta[:, 1]))),
                physical_state_delta_max=float(
                    np.max(np.abs(trajectories[9] - trajectories[6]))
                ),
            )
            pairs.append(clean(pair))
        print(f"{name}: repeat {repeat + 1}/{repeats} complete", flush=True)
    summaries = []
    for nx in (6, 9):
        for phase in ("cold", "warm"):
            selected = [r for r in calls if r["nx"] == nx and r["phase"] == phase]
            valid = [r for r in selected if r["accepted"]]
            row = dict(
                **base,
                nx=nx,
                phase=phase,
                decision_variables=nx * (horizon + 1) + 2 * horizon,
                calls=len(selected),
                accepted_calls=len(valid),
                acceptance_rate=len(valid) / len(selected) if selected else None,
                rollout_guess_calls=sum(
                    r["zero_control_rollout_guess"] for r in selected
                ),
            )
            row.update(statistics(selected, "native_solver_ms", "native_ms_all"))
            row.update(statistics(valid, "native_solver_ms", "native_ms_accepted"))
            row.update(statistics(selected, "solve_wall_ms", "solve_wall_ms"))
            row.update(statistics(selected, "iterations", "sqp_iterations"))
            row.update(statistics(selected, "max_violation", "violation"))
            summaries.append(row)
    write_json(
        output / f"{name}.json",
        {
            "config": base,
            "settings": asdict(settings),
            "replay_source": str(replay["path"]),
            "replay_sha256": replay["sha256"],
            "build": build,
            "measurements": calls,
            "pairs": pairs,
            "summary": summaries,
        },
    )
    return calls, pairs, summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--inputs",
        nargs="+",
        type=Path,
        default=[
            ROOT / "results/paper_comparison_wide_final",
            ROOT / "results/paper_comparison_narrow_final",
        ],
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/dimension_scaling"
    )
    parser.add_argument("--horizons", nargs="+", type=int, default=[20, 40, 80])
    parser.add_argument("--following-states", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--acceptance-tolerance", type=float, default=1e-4)
    args = parser.parse_args()
    if (
        args.repeats < 1
        or args.following_states < 1
        or any(n < 1 for n in args.horizons)
        or len(set(args.horizons)) != len(args.horizons)
        or not 0 < args.acceptance_tolerance < 1
    ):
        parser.error(
            "Require positive repeats/states/unique horizons and tolerance in (0,1)"
        )
    output = args.output.resolve()
    if (output / "provenance.json").exists() or (
        output / "measurements.jsonl"
    ).exists():
        parser.error(f"Refusing to overwrite benchmark output: {output}")
    try:
        replays = load_replays(args.inputs, args.following_states)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    output.mkdir(parents=True, exist_ok=True)
    source_paths = [
        Path(__file__).resolve(),
        ROOT / "scripts/run_dimension_scaling.sh",
        ROOT / "paper_comparison/model.py",
        ROOT / "reiter2023/geometry.py",
    ]
    idle_check = wait_for_idle()
    provenance = {
        "command": sys.argv,
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "casadi": ca.__version__,
        "horizons": args.horizons,
        "repeats": args.repeats,
        "following_states": args.following_states,
        "acceptance_tolerance": args.acceptance_tolerance,
        "timing": "Native acados time_tot in ms; sequential calls, no overlapping solves.",
        "cold": "Sample 0 after QP/NLP reset each repeat; excludes build time. Not process startup.",
        "warm": "Samples 1..20 follow shifted own previous solution; failed solves reset to zero-control rollout.",
        "pairing": "Identical saved physical initial states and parameters; alternating 6/9 call order.",
        "dimensions": "Nominal multiple-shooting variables nx*(N+1)+nu*N; not allocated memory.",
        "memory_measured": False,
        "comparison_idle_check": idle_check,
        "comparison_overlap_guard": "Check process inventory before each solve; wait for five quiet seconds if needed.",
        "thread_environment": {
            key: os.environ.get(key)
            for key in (
                "OPENBLAS_NUM_THREADS",
                "OMP_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS",
            )
        },
        "sha256": {str(path.relative_to(ROOT)): digest(path) for path in source_paths},
        "replay_sources": [
            {
                "path": str(r["path"]),
                "sha256": r["sha256"],
                "initial_states": r["states"].tolist(),
            }
            for r in replays
        ],
        "complete": False,
    }
    write_json(output / "provenance.json", provenance)
    measurements, pairs, summaries = [], [], []
    with (output / "measurements.jsonl").open("x") as journal:
        for replay in replays:
            for horizon in args.horizons:
                calls, matched, summary = run_case(
                    replay,
                    horizon,
                    args.repeats,
                    output,
                    args.acceptance_tolerance,
                    journal,
                )
                measurements.extend(calls)
                pairs.extend(matched)
                summaries.extend(summary)
                write_csv(output / "measurements.csv", measurements)
                write_csv(output / "paired_controls.csv", pairs)
                write_csv(output / "summary.csv", summaries)
                write_json(output / "summary.json", summaries)
    provenance.update(
        complete=True, solve_calls=len(measurements), paired_calls=len(pairs)
    )
    write_json(output / "provenance.json", provenance)
    print(
        f"Completed {len(measurements)} sequential solves; results: {output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
