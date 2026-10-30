"""Aggregate the randomized benchmark into per-method statistics.

Reports, per method: completion rate, overtake rate, clearance quantiles,
constraint-violation rate and solve-time quantiles. Solve times are grouped by
backend because acados native time and Python wall time are not comparable.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

BACKENDS = {
    "reiter_direct_ellipse": "acados",
    "reiter_lifted_ellipse": "acados",
    "reiter_direct_circles": "acados",
    "reiter_lifted_circles": "acados",
    "reiter_direct_ellipse_point": "acados",
    "paper6_hybrid_sqp": "acados",
    "paper9_hybrid_sqp": "acados",
    "paper6_circles_sqp": "acados",
    "paper6_frozen_sqp": "acados",
    "paper6_circles5_sqp": "acados",
    "paper6_circlesN3_sqp": "acados",
    "paper6_circlesN5_sqp": "acados",
    "paper6_pruned_sqp": "acados",
    "paper6_capsule_sqp": "acados",
    "paper6_nokappa_sqp": "acados",
    "paper6_hybrid_cilqr": "python",
    "xing_joint_corridor": "python",
    "xing_joint_yield": "python",
    "xing_joint_corridor_point": "python",
}

TIMING_KEY = {
    "acados": "mean_native_solver_ms",
    "python": "mean_solve_wall_ms",
}


def quantiles(values):
    """Quantiles together with the sample count they were computed from.

    The count is carried because it is not the case count: a failed case
    contributes no clearance and no solve time, so the clearance and timing
    distributions are computed over fewer cases than the completion rate.
    A reader who compares the two without this number reads the wrong
    denominator.
    """
    values = [v for v in values if v is not None and np.isfinite(v)]
    if not values:
        return {"n": 0}
    array = np.asarray(values, dtype=float)
    return {
        "n": int(array.size),
        "min": float(array.min()),
        "p05": float(np.percentile(array, 5)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(array.max()),
        "mean": float(array.mean()),
    }


MAX_LATERAL_ACCELERATION = 5.0


def infeasible(row):
    """True when the scene itself admits no constraint-satisfying trajectory.

    The lateral acceleration of a constant-curvature reference at the initial
    speed already exceeds the bus limit, so every planner must abort. The
    criterion depends only on the scene, not on the method.
    """
    speed = row.get("initial_speed")
    kappa = row.get("curvature")
    if speed is None or kappa is None:
        return False
    return speed**2 * abs(kappa) > MAX_LATERAL_ACCELERATION


def summarize(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["method"]].append(row)
    summary = []
    for method, group in sorted(grouped.items()):
        backend = BACKENDS.get(method, "unknown")
        feasible = [r for r in group if not infeasible(r)]
        completed = [r for r in feasible if r.get("success")]
        obstacles = [r.get("min_obstacle_clearance_m") for r in feasible]
        roads = [r.get("min_road_clearance_m") for r in feasible]
        violations = [
            r.get("max_constraint_violation")
            for r in feasible
            if r.get("max_constraint_violation") is not None
        ]
        summary.append(
            {
                "method": method,
                "backend": backend,
                "cases": len(group),
                "infeasible_scenes": len(group) - len(feasible),
                "feasible_cases": len(feasible),
                "completed": len(completed),
                "completion_rate": len(completed) / len(feasible) if feasible else None,
                "overtakes": sum(1 for r in feasible if r.get("passed_obstacle")),
                "overtake_rate": (
                    sum(1 for r in feasible if r.get("passed_obstacle")) / len(feasible)
                    if feasible
                    else None
                ),
                "errors": sum(1 for r in group if r.get("error")),
                "obstacle_clearance": quantiles(obstacles),
                "road_clearance": quantiles(roads),
                "min_clearance": quantiles(
                    [
                        min(o, d)
                        for o, d in zip(obstacles, roads)
                        if o is not None and d is not None
                    ]
                ),
                "max_constraint_violation": quantiles(violations),
                "solve_ms": quantiles(
                    [
                        r.get(TIMING_KEY.get(backend, "mean_solve_wall_ms"))
                        for r in feasible
                    ]
                ),
                "progress_m": quantiles([r.get("progress_m") for r in feasible]),
            }
        )
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    rows = json.loads((args.input / "results.json").read_text())
    summary = summarize(rows)
    text = json.dumps(summary, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + "\n")
        columns = [
            "method",
            "backend",
            "cases",
            "infeasible_scenes",
            "feasible_cases",
            "completed",
            "completion_rate",
            "overtakes",
            "overtake_rate",
        ]
        with args.output.with_suffix(".csv").open("w") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(
                {key: row.get(key) for key in columns} for row in summary
            )


if __name__ == "__main__":
    main()
