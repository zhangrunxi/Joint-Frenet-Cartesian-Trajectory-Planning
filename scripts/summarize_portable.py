"""Summarize portable smoke runs without treating them as paper-20 results."""

import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    data = json.loads((ROOT / "results/portable/experiment.json").read_text())
    rows = []
    for scenario in data["scenarios"]:
        for run in scenario["runs"]:
            times = [plan["solve_ms"] for plan in run["plans"]]
            metrics = run["metrics"]
            rows.append(
                {
                    "scenario": scenario["id"],
                    "reference": run["id"],
                    "seed": run["seed"],
                    "solves": len(times),
                    "completed": metrics["completed"],
                    "collision": metrics["collision"],
                    "solver_failures": metrics["solver_failures"],
                    "mean_solve_ms": float(np.mean(times)),
                    "median_solve_ms": float(np.median(times)),
                    "p95_solve_ms": float(np.percentile(times, 95)),
                    "min_body_separation_m": metrics["min_body_separation_m"],
                    "max_violation": metrics["max_violation"],
                    "max_obstacle_slack": metrics["max_obstacle_slack"],
                    "paper20_baseline": False,
                }
            )
    if not rows:
        raise ValueError("No portable runs were exported")
    with (ROOT / "results/portable/summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "scope": "Current portable backend only; NOT paper 20 reproduction",
        "runs": len(rows),
        "solves": sum(r["solves"] for r in rows),
        "completed_runs": sum(r["completed"] for r in rows),
        "collision_runs": sum(r["collision"] for r in rows),
        "solver_failures": sum(r["solver_failures"] for r in rows),
        "note": "Single seed per task/reference; timings are local diagnostics, not a benchmark.",
    }
    (ROOT / "results/portable/summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
