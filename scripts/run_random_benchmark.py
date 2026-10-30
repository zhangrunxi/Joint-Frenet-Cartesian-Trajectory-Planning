"""Randomized scenario benchmark over the three reconstructed methods.

Scenario distributions are fixed in ``RANDOM_PROTOCOL`` and generated from a
seeded ``numpy`` generator, so a rerun with the same seed and protocol
reproduces the same case list. Nothing is selected on the basis of results.

Each case runs every method on the same environment, obstacle prediction and
road. Failures, aborts and clearance violations are recorded as failures; they
are not retried or dropped.
"""

import argparse
import csv
import hashlib
import json
import os
import platform
import sys
from dataclasses import replace
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import numpy as np

from paper_comparison.model import Settings
from run_paper_comparison import run_case
from run_final_comparison import run_xing

# The case list is fully determined by this table and the seed.
RANDOM_PROTOCOL = {
    "seed": 20261030,
    "cases": 40,
    # "general": lateral target drawn from the offsets below, so the scenario is
    # ordinary following or crossing traffic. "overtake": the lateral target is
    # placed on the far side of the obstacle, so every case asks the planner to
    # pass the obstacle.
    "task": "general",
    "half_width_m": [4.6, 8.5],
    # Lateral targets are drawn relative to the road half width so that the
    # target always stays inside the drivable area. The bus half width is
    # 1.15 m and the road margin is 0.2 m, so |lateral| <= half_width - 1.35.
    "lateral_offset_m": [-2.0, -1.0, 0.0, 1.0, 2.0],
    "kappa_per_m": [-1 / 40, 0.0, 1 / 60, 1 / 35],
    "obstacle_start_m": [25.0, 35.0, 50.0],
    "obstacle_speed_mps": [6.0, 8.0, 11.0],
    "initial_speed_mps": [8.0, 12.0, 15.0],
    "steps": 120,
}

METHODS = [
    "reiter_direct_ellipse",
    "reiter_lifted_ellipse",
    "reiter_direct_circles",
    "reiter_lifted_circles",
    "paper6_hybrid_sqp",
    "paper9_hybrid_sqp",
    "paper6_circles_sqp",
    "paper6_hybrid_cilqr",
    "xing_joint_corridor",
    "xing_joint_yield",
]

POINT_METHODS = [
    "reiter_direct_ellipse_point",
    "xing_joint_corridor_point",
]

# Representation ablations inside the own model, all sharing one solver:
# frozen heading deviation ([16]-style contour), five covering circles and
# the proposed capsule-plus-corner-circle geometry.
ABLATION_METHODS = [
    "paper6_frozen_sqp",
    "paper6_circles5_sqp",
    "paper6_pruned_sqp",
    "paper6_capsule_sqp",
    "paper6_nokappa_sqp",
]


def build_cases(protocol):
    rng = np.random.default_rng(protocol["seed"])
    cases = []
    for index in range(protocol["cases"]):
        half_width = float(rng.choice(protocol["half_width_m"]))
        if protocol.get("task") == "overtake":
            side = float(rng.choice([-1.0, 1.0]))
            lateral = side * (half_width - 1.5)
        else:
            lateral = float(rng.choice(protocol["lateral_offset_m"]))
        # Keep the reference point inside the drivable corridor.
        lateral = float(
            np.clip(lateral, -half_width + 1.35, half_width - 1.35)
        )
        kappa = float(rng.choice(protocol["kappa_per_m"]))
        obstacle_start = float(rng.choice(protocol["obstacle_start_m"]))
        obstacle_speed = float(rng.choice(protocol["obstacle_speed_mps"]))
        initial_speed = float(rng.choice(protocol["initial_speed_mps"]))
        cases.append(
            {
                "case_id": index,
                "half_width": half_width,
                "lateral": lateral,
                "kappa": kappa,
                "obstacle_start": obstacle_start,
                "obstacle_speed": obstacle_speed,
                "initial_speed": initial_speed,
            }
        )
    return cases


def run_method(output, method, case, steps, intervals):
    settings = replace(
        Settings(),
        lateral=case["lateral"],
        intervals=intervals,
        speed=case["initial_speed"],
    )
    scenario = "curve" if abs(case["kappa"]) > 1e-9 else "straight"
    name = f"case{case['case_id']:03d}_{method}_{scenario}"
    if method.startswith("xing"):
        return run_xing(
            output,
            method,
            scenario,
            steps,
            settings,
            case["half_width"],
            50,
            case["obstacle_start"],
            case["obstacle_speed"],
            case["kappa"],
            case["initial_speed"],
        )
    return run_case(
        output,
        method,
        scenario,
        steps,
        settings,
        case["half_width"],
        case["obstacle_start"],
        case["obstacle_speed"],
        case["kappa"],
        case["initial_speed"],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=RANDOM_PROTOCOL["steps"])
    parser.add_argument("--intervals", type=int, default=40)
    parser.add_argument("--cases", type=int, default=RANDOM_PROTOCOL["cases"])
    parser.add_argument(
        "--methods",
        nargs="+",
        default=METHODS + POINT_METHODS + ABLATION_METHODS,
    )
    parser.add_argument(
        "--task", choices=["general", "overtake"], default=RANDOM_PROTOCOL["task"]
    )
    parser.add_argument("--case-start", type=int, default=0)
    parser.add_argument("--case-stop", type=int, default=None)
    args = parser.parse_args()

    protocol = dict(RANDOM_PROTOCOL)
    protocol["cases"] = args.cases
    protocol["steps"] = args.steps
    protocol["task"] = args.task
    cases = build_cases(protocol)
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("Use an empty new output directory")
    args.output.mkdir(parents=True, exist_ok=True)

    (args.output / "protocol.json").write_text(
        json.dumps(protocol, indent=2) + "\n"
    )
    (args.output / "cases.json").write_text(json.dumps(cases, indent=2) + "\n")

    rows = []
    stop = args.case_stop if args.case_stop is not None else len(cases)
    for case in cases[args.case_start : stop]:
        for method in args.methods:
            started = perf_counter()
            try:
                result = run_method(
                    args.output, method, case, args.steps, args.intervals
                )
                row = {
                    key: value
                    for key, value in result.items()
                    if key
                    not in ("states", "controls", "cycles", "startup", "settings")
                }
                row["case_id"] = case["case_id"]
                row["wall_seconds"] = perf_counter() - started
                row["error"] = None
            except Exception as exc:  # keep the failure, never hide it
                row = {
                    "case_id": case["case_id"],
                    "method": method,
                    "scenario": "curve"
                    if abs(case["kappa"]) > 1e-9
                    else "straight",
                    "success": False,
                    "error": f"{type(exc).__name__}: {exc}",
                    "wall_seconds": perf_counter() - started,
                }
            rows.append(row)
            (args.output / "partial_results.jsonl").write_text(
                "\n".join(json.dumps(r, default=str) for r in rows) + "\n"
            )
            print(
                f"case {case['case_id']:03d} {method:34s} "
                f"success={row.get('success')} "
                f"progress={row.get('progress_m')} "
                f"clear_obs={row.get('min_obstacle_clearance_m')} "
                f"clear_road={row.get('min_road_clearance_m')} "
                f"{row.get('error') or ''}",
                flush=True,
            )

    (args.output / "results.json").write_text(
        json.dumps(rows, indent=2, default=str) + "\n"
    )
    columns = sorted({key for row in rows for key in row})
    with (args.output / "results.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    (args.output / "provenance.json").write_text(
        json.dumps(
            {
                "command": sys.argv,
                "python": sys.version,
                "platform": platform.platform(),
                "numpy": np.__version__,
                "sha256": {
                    str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sorted(ROOT.glob("paper_comparison/*.py"))
                },
                "thread_environment": {
                    key: os.environ.get(key)
                    for key in (
                        "OPENBLAS_NUM_THREADS",
                        "OMP_NUM_THREADS",
                        "VECLIB_MAXIMUM_THREADS",
                    )
                },
                "all_calls_sequential": True,
                "author_implementation": False,
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
