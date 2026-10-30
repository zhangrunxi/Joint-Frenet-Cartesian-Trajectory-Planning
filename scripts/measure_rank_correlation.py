"""Rank correlation between solve time and completion rate.

The reviewer asks whether the compared planners can be separated by their
solve time. This script computes the rank correlation between the two
quantities, and it pins down the population, the timing field and the way
ties are broken, because a rank correlation is only meaningful together with
the choices that produced it.

The population and the conventions are the following, and they are the same
ones the tables in the response letter use:

- Only the curved-road scenarios enter the correlation. The straight-road
  scenarios are excluded so that the two tasks stay comparable.
- The completion rate is the fraction of successful scenarios over the
  curved-road scenarios of the method.
- The solve time of a method is the **median over its successful scenarios**
  of the mean wall-clock time per planning cycle. Failed scenarios are
  excluded, because a solver that iterates until it fails does not have a
  solve time comparable to one that converged.
- Ties are broken with the average rank, which is the default of
  ``scipy.stats.spearmanr`` and ``kendalltau``.

Usage:
    python scripts/measure_rank_correlation.py \\
        --input results/random_benchmark_v2 --output results/rank_correlation.csv
"""

import argparse
import csv
import json
import statistics
from pathlib import Path

from scipy.stats import kendalltau, spearmanr

SCENARIO = "curve"
TIMING_FIELD = "mean_solve_wall_ms"


def load_rows(results_dir: Path):
    """The per-case records of a benchmark run.

    A run keeps them in ``partial_results.jsonl``, one JSON object per line.
    The per-method ``*.json`` files of the same directory are the aggregated
    statistics of one method and are not sufficient here, because the
    correlation needs the per-case solve times of the successful cases.
    """
    source = results_dir / "partial_results.jsonl"
    rows = []
    for line in source.read_text().splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        rows.append(json.loads(line))
    return rows


def per_method(rows):
    """Completion rate and median solve time per method, on one scenario kind."""
    grouped = {}
    for row in rows:
        if row.get("scenario") != SCENARIO:
            continue
        method = row["method"]
        entry = grouped.setdefault(method, {"cases": 0, "completed": 0, "times": []})
        entry["cases"] += 1
        if row.get("success"):
            entry["completed"] += 1
            timing = row.get(TIMING_FIELD)
            if timing is not None:
                entry["times"].append(float(timing))
    return grouped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    grouped = per_method(load_rows(args.input))
    points = []
    for method, entry in sorted(grouped.items()):
        if not entry["times"]:
            continue
        points.append(
            (
                method,
                entry["completed"] / entry["cases"],
                statistics.median(entry["times"]),
            )
        )

    rates = [p[1] for p in points]
    times = [p[2] for p in points]
    spearman = spearmanr(rates, times).statistic
    kendall = kendalltau(rates, times).statistic

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["method", "completion_rate", "median_solve_ms_of_successful_cases"]
        )
        for method, rate, timing in points:
            writer.writerow([method, f"{rate:.6f}", f"{timing:.6f}"])
        writer.writerow([])
        writer.writerow(["scenario", SCENARIO])
        writer.writerow(["timing_field", TIMING_FIELD])
        writer.writerow(["timing_population", "successful cases only"])
        writer.writerow(["tie_breaking", "average rank (scipy default)"])
        writer.writerow(["methods", len(points)])
        writer.writerow(["spearman", f"{spearman:.6f}"])
        writer.writerow(["kendall", f"{kendall:.6f}"])

    print(f"methods={len(points)} spearman={spearman:.4f} kendall={kendall:.4f}")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
