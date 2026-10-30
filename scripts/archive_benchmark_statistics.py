"""Reduce a randomized-benchmark run to the statistics a reader has to check.

The per-method JSON files of a benchmark run carry the full trajectory
(``states``, ``controls`` and ``cycles``), which makes them far too large to
archive next to the code. The statistics that the manuscript and the reviewer
response quote are all in the top level of each file, so the archive keeps
those and drops the trajectory.

Usage:
    python scripts/archive_benchmark_statistics.py \\
        --input results/random_benchmark_v1 \\
        --output results/archived/random_benchmark_pre_correction
"""

import argparse
import json
from pathlib import Path

# The trajectory fields, which are dropped from the archive.
TRAJECTORY_KEYS = ("states", "controls", "cycles")

# The fields kept for each method, in the order they are written out.
KEPT_KEYS = (
    "name",
    "method",
    "scenario",
    "curvature",
    "half_width",
    "settings",
    "nx",
    "nu",
    "N",
    "decision_variables",
    "path_nonlinear_constraints",
    "setup_seconds",
    "startup",
    "steps_requested",
    "completed_steps",
    "success",
    "collision_or_road_exit",
    "min_obstacle_clearance_m",
    "min_road_clearance_m",
    "progress_m",
    "final_relative_s_m",
    "passed_obstacle",
    "rms_lateral_acc_mps2",
    "rms_lateral_jerk_mps3",
    "rms_longitudinal_jerk_mps3",
    "max_abs_acc_mps2",
    "max_abs_longitudinal_jerk_mps3",
    "max_abs_steering_rate_radps",
    "mean_native_solver_ms",
    "p95_native_solver_ms",
    "mean_solve_wall_ms",
    "p95_solve_wall_ms",
    "warm_wall_deadline_miss_fraction",
    "max_constraint_violation",
    "max_coordinate_error",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    archived = 0
    for source in sorted(args.input.glob("*.json")):
        # The scenario definition, the protocol and the provenance record are
        # copied unchanged below; they are not per-method statistics.
        if source.name in ("cases.json", "protocol.json", "provenance.json"):
            continue
        payload = json.loads(source.read_text())
        if not isinstance(payload, dict):
            continue
        reduced = {key: payload[key] for key in KEPT_KEYS if key in payload}
        for key in TRAJECTORY_KEYS:
            assert key not in reduced, key
        (args.output / source.name).write_text(json.dumps(reduced, indent=2) + "\n")
        archived += 1

    # The scenario definition and the protocol are copied unchanged: they are
    # what a reader needs in order to judge the numbers.
    for name in (
        "cases.json",
        "protocol.json",
        "provenance.json",
        "summary.json",
        "summary.csv",
        "results.csv",
    ):
        source = args.input / name
        if source.exists():
            (args.output / name).write_bytes(source.read_bytes())

    # The figures are copied as well, so that the archive is self-contained.
    figures = args.input / "figures"
    if figures.is_dir():
        (args.output / "figures").mkdir(exist_ok=True)
        for figure in sorted(figures.glob("*.png")):
            (args.output / "figures" / figure.name).write_bytes(figure.read_bytes())

    print(f"archived {archived} method files into {args.output}")


if __name__ == "__main__":
    main()
