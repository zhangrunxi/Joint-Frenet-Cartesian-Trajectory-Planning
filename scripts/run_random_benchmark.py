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

from paper_comparison.evaluation import initial_state_diagnostics
from paper_comparison.model import Settings
from paper_comparison.harness_options import (
    ACCEPT_MAXITER,
    INITIAL_GUESS,
    N_DISC,
    XING_ACCEPT_INACCURATE,
    XING_FREEZE_CORRIDOR,
)
from paper_comparison.vehicle import GEOMETRY
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
    "paper6_circlesN3_sqp",
    "paper6_circlesN5_sqp",
    "paper6_pruned_sqp",
    "paper6_capsule_sqp",
    "paper6_nokappa_sqp",
]


# The rear-axle reference point may sit at most half_width - (body half width
# + road margin) from the centre line; for the bus that is half_width - 1.35 m.
BODY_HALF_WIDTH = GEOMETRY.half_width
ROAD_MARGIN = 0.20
# The lateral acceleration limit of Eq. (34f) is 5 m/s^2. A case is only
# generated when the initial state satisfies it with a margin, so that an
# infeasible first frame cannot be mistaken for a planner failure.
MAX_LATERAL_ACCELERATION = 5.0
LATERAL_ACCELERATION_MARGIN = 0.5


def build_cases(protocol):
    """Generate the frozen case list.

    Two properties are enforced for every case, because a case that violates
    them measures the scenario generator rather than the planner:

    * the initial state satisfies the lateral acceleration bound of Eq. (34f),
      v0^2 * |kappa| <= 5 - 0.5 m/s^2, so the first frame is feasible;
    * on the overtaking task the ego vehicle is faster than the obstacle by at
      least 1.5 m/s, so the overtake is dynamically possible at all.
    """
    rng = np.random.default_rng(protocol["seed"])
    cases = []
    index = 0
    attempts = 0
    while index < protocol["cases"]:
        attempts += 1
        if attempts > 1000 * protocol["cases"]:
            raise RuntimeError("could not generate feasible cases")
        half_width = float(rng.choice(protocol["half_width_m"]))
        if protocol.get("task") == "overtake":
            side = float(rng.choice([-1.0, 1.0]))
            lateral = side * (half_width - 1.5)
        else:
            lateral = float(rng.choice(protocol["lateral_offset_m"]))
        # Keep the reference point inside the drivable corridor.
        lateral = float(
            np.clip(
                lateral,
                -half_width + BODY_HALF_WIDTH + ROAD_MARGIN,
                half_width - BODY_HALF_WIDTH - ROAD_MARGIN,
            )
        )
        kappa = float(rng.choice(protocol["kappa_per_m"]))
        obstacle_start = float(rng.choice(protocol["obstacle_start_m"]))
        obstacle_speed = float(rng.choice(protocol["obstacle_speed_mps"]))
        initial_speed = float(rng.choice(protocol["initial_speed_mps"]))

        lateral_acceleration = initial_speed**2 * abs(kappa)
        if lateral_acceleration > MAX_LATERAL_ACCELERATION - LATERAL_ACCELERATION_MARGIN:
            continue
        if protocol.get("task") == "overtake" and initial_speed < obstacle_speed + 1.5:
            continue

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
        index += 1
    return cases


# Narrow-road grids of NARROW_BENCHMARK_PROTOCOL.md. They are full grids, not
# random draws, so that every gap and every curvature is covered. The grids
# are scaled to the vehicle: an SUV drives narrower lanes and tighter turns.
GAP_PROTOCOLS = {
    "bus": {
        "gap_m": [2.8, 3.0, 3.3, 3.6, 4.0, 4.5, 5.0, 5.5, 6.0],
        "kappa_per_m": [0.0, 1 / 60, 1 / 35, -1 / 40],
        "initial_speed_mps": [8.0, 11.0],
    },
    "suv": {
        "gap_m": [2.4, 2.6, 2.8, 3.0, 3.3, 3.6, 4.0, 4.4, 4.8],
        "kappa_per_m": [0.0, 1 / 30, 1 / 20, -1 / 25],
        "initial_speed_mps": [6.0, 9.0],
    },
}
GAP_PROTOCOL = {
    "task": "gap",
    "vehicle": GEOMETRY.name,
    **GAP_PROTOCOLS[GEOMETRY.name],
    # A stopped vehicle of the same type 0.3 m from the right curb leaves the
    # gap on the left; the lateral target is the middle of that gap.
    "curb_clearance_m": 0.3,
    "obstacle_start_m": 30.0,
    "steps": 120,
}

# Version 2 of the gap task (protocol section 5): 0.1 m steps through the
# range where the minimum passable gaps of the geometries lie (exact
# rectangle 2.70 / hybrid 3.18 / circles 3.65 m for the bus, 2.30 / 2.69 /
# 3.09 m for the SUV), plus the wide gaps the ellipse needs.
GAP2_GAPS = {
    "bus": [round(2.7 + 0.1 * i, 1) for i in range(16)] + [4.5, 5.0, 5.5, 6.0],
    "suv": [round(2.3 + 0.1 * i, 1) for i in range(14)] + [4.0, 4.4, 4.8],
}
GAP2_PROTOCOL = {**GAP_PROTOCOL, "task": "gap2", "gap_m": GAP2_GAPS[GEOMETRY.name]}

GAP_METHODS = [
    "paper6_hybrid_sqp",
    "paper6_capsule_sqp",
    "paper6_circles_sqp",
    "paper6_circles5_sqp",
    "reiter_direct_ellipse",
    "reiter_lifted_ellipse",
    "reiter_direct_circles",
    "reiter_lifted_circles",
    "xing_joint_corridor",
]

CURB_PROTOCOLS = {
    "bus": {
        "half_width_m": [2.5, 3.5],
        "kappa_per_m": [1 / 15, 1 / 20, 1 / 30, 1 / 40, -1 / 15, -1 / 20, -1 / 30, -1 / 40],
        "initial_speed_mps": [5.0, 8.0],
    },
    "suv": {
        "half_width_m": [1.75, 2.5],
        "kappa_per_m": [1 / 8, 1 / 12, 1 / 20, 1 / 30, -1 / 8, -1 / 12, -1 / 20, -1 / 30],
        "initial_speed_mps": [4.0, 6.0],
    },
}
CURB_PROTOCOL = {
    "task": "curb",
    "vehicle": GEOMETRY.name,
    **CURB_PROTOCOLS[GEOMETRY.name],
    # The obstacle is parked 30 m outside the road and never interacts.
    "obstacle_lateral_m": 30.0,
    "obstacle_start_m": 30.0,
    "steps": 120,
}

CURB_METHODS = [
    "paper6_hybrid_sqp",
    "paper6_frozen_sqp",
    "reiter_direct_ellipse",
    "reiter_direct_ellipse_point",
    "reiter_direct_ellipse_inflated",
    "xing_joint_corridor",
    "xing_joint_corridor_point",
]


def lateral_feasible(speed, kappa):
    return speed**2 * abs(kappa) <= MAX_LATERAL_ACCELERATION - LATERAL_ACCELERATION_MARGIN


def build_gap_cases(protocol):
    cases = []
    for gap in protocol["gap_m"]:
        half_width = (gap + 2 * BODY_HALF_WIDTH + protocol["curb_clearance_m"]) / 2
        obstacle_lateral = -half_width + protocol["curb_clearance_m"] + BODY_HALF_WIDTH
        lateral = round(
            min(
                protocol["curb_clearance_m"] / 2 + BODY_HALF_WIDTH,
                half_width - BODY_HALF_WIDTH - ROAD_MARGIN,
            ),
            9,
        )
        for kappa in protocol["kappa_per_m"]:
            for speed in protocol["initial_speed_mps"]:
                if not lateral_feasible(speed, kappa):
                    continue
                cases.append(
                    {
                        "case_id": len(cases),
                        "gap": gap,
                        "half_width": half_width,
                        "lateral": lateral,
                        "kappa": kappa,
                        "obstacle_start": protocol["obstacle_start_m"],
                        "obstacle_speed": 0.0,
                        "obstacle_lateral": obstacle_lateral,
                        "initial_speed": speed,
                    }
                )
    return cases


def build_curb_cases(protocol):
    cases = []
    for half_width in protocol["half_width_m"]:
        for kappa in protocol["kappa_per_m"]:
            outer = float(-np.sign(kappa))
            for speed in protocol["initial_speed_mps"]:
                if not lateral_feasible(speed, kappa):
                    continue
                cases.append(
                    {
                        "case_id": len(cases),
                        "half_width": half_width,
                        "lateral": outer * (half_width - BODY_HALF_WIDTH - ROAD_MARGIN),
                        "kappa": kappa,
                        "obstacle_start": protocol["obstacle_start_m"],
                        "obstacle_speed": 0.0,
                        "obstacle_lateral": outer * protocol["obstacle_lateral_m"],
                        "initial_speed": speed,
                    }
                )
    return cases


# Version 2 of the overtaking task (protocol section 5.4): a two-lane narrow
# road, the slow vehicle in the centre of the right lane, the target in the
# centre of the left lane. The widest lateral separation the road allows is
# 1.5 * half_width - (body half width + road margin), so the half widths
# straddle the same-s separation each obstacle geometry needs. Only cases that
# can be completed at the cruise speed within the run are kept.
OVERTAKE2_PROTOCOLS = {
    "bus": {
        "half_width_m": [3.0, 3.25, 3.5, 4.0],
        "speed_pairs_mps": [[11.0, 5.0], [14.0, 8.0]],
    },
    "suv": {
        "half_width_m": [2.5, 2.75, 3.0, 3.5],
        "speed_pairs_mps": [[11.0, 5.0], [14.0, 8.0]],
    },
}
OVERTAKE2_PROTOCOL = {
    "task": "overtake2",
    "vehicle": GEOMETRY.name,
    **OVERTAKE2_PROTOCOLS[GEOMETRY.name],
    "kappa_per_m": [0.0, 1 / 60, 1 / 35, -1 / 40],
    "obstacle_start_m": [20.0, 30.0],
    "steps": 120,
    "dt_s": 0.1,
    # The ego front must end one body length ahead of the obstacle front.
    "pass_distance_margin_m": 2.0,
}


def build_overtake2_cases(protocol):
    cases = []
    horizon = protocol["steps"] * protocol["dt_s"]
    for half_width in protocol["half_width_m"]:
        for kappa in protocol["kappa_per_m"]:
            for speed, obstacle_speed in protocol["speed_pairs_mps"]:
                if not lateral_feasible(speed, kappa):
                    continue
                for start in protocol["obstacle_start_m"]:
                    needed = start + GEOMETRY.length + protocol["pass_distance_margin_m"]
                    if (speed - obstacle_speed) * horizon < needed:
                        continue
                    cases.append(
                        {
                            "case_id": len(cases),
                            "half_width": half_width,
                            "lateral": half_width / 2,
                            "kappa": kappa,
                            "obstacle_start": start,
                            "obstacle_speed": obstacle_speed,
                            "obstacle_lateral": -half_width / 2,
                            "initial_speed": speed,
                        }
                    )
    return cases


def run_method(output, method, case, steps, intervals, xing_iterations=50):
    settings = replace(
        Settings(),
        lateral=case["lateral"],
        intervals=intervals,
        speed=case["initial_speed"],
        obstacle_lateral=case.get("obstacle_lateral", 0.0),
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
            xing_iterations,
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
    parser.add_argument("--methods", nargs="+", default=None)
    parser.add_argument(
        "--task",
        choices=["general", "overtake", "overtake2", "gap", "gap2", "curb"],
        default=RANDOM_PROTOCOL["task"],
    )
    parser.add_argument("--case-start", type=int, default=0)
    # The SQP iteration cap of the Xing reconstruction. acados runs up to 150.
    parser.add_argument("--xing-max-iterations", type=int, default=50)
    parser.add_argument("--case-stop", type=int, default=None)
    args = parser.parse_args()

    if args.task == "overtake2":
        protocol, methods = dict(OVERTAKE2_PROTOCOL), GAP_METHODS
        protocol["steps"] = args.steps
        protocol["dt_s"] = Settings().dt
        cases = build_overtake2_cases(protocol)
    elif args.task in ("gap", "gap2"):
        protocol = dict(GAP_PROTOCOL if args.task == "gap" else GAP2_PROTOCOL)
        methods = GAP_METHODS
        cases = build_gap_cases(protocol)
    elif args.task == "curb":
        protocol, methods = dict(CURB_PROTOCOL), CURB_METHODS
        cases = build_curb_cases(protocol)
    else:
        protocol = dict(RANDOM_PROTOCOL)
        protocol["cases"] = args.cases
        protocol["task"] = args.task
        methods = METHODS + POINT_METHODS + ABLATION_METHODS
        cases = build_cases(protocol)
    protocol["steps"] = args.steps
    protocol["xing_max_iterations"] = args.xing_max_iterations
    protocol["initial_guess"] = INITIAL_GUESS
    protocol["n_disc"] = N_DISC
    protocol["accept_maxiter"] = ACCEPT_MAXITER
    protocol["xing_accept_inaccurate_qp"] = XING_ACCEPT_INACCURATE
    protocol["xing_freeze_corridor"] = XING_FREEZE_CORRIDOR
    protocol["stage0_nonlinear_constraints"] = True
    protocol["initial_state_validation"] = "exact common body margins; invalid cases retained and scored separately"
    cases = [{**case, **initial_state_diagnostics(case)} for case in cases]
    args.methods = args.methods or methods
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
                    args.output,
                    method,
                    case,
                    args.steps,
                    args.intervals,
                    args.xing_max_iterations,
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
            row.update({key: value for key, value in case.items()
                        if key.startswith("initial_")})
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
