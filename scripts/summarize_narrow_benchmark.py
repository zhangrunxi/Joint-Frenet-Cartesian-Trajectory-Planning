"""Tables of the narrow-road benchmark (NARROW_BENCHMARK_PROTOCOL.md).

The gap task is scored by passing: a run counts only when it is safe
(``success``) *and* the ego bus is past the obstacle (``passed_obstacle``).
Following the obstacle without collision is not a pass.

The curb task is scored by the exact road clearance of the body. A clearance
below zero is a curb strike, below 0.19 m a margin violation, and the distance
of the clearance from the 0.2 m margin says how conservative the constraint is.

Usage:
    python scripts/summarize_narrow_benchmark.py --task gap \\
        --input results/narrow_gap --output results/narrow_gap/summary.csv
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def load(root: Path):
    """Per-case records and cases of every part directory under ``root``."""
    rows, cases = [], {}
    for part in sorted({p.parent for p in root.rglob("partial_results.jsonl")}):
        for case in json.loads((part / "cases.json").read_text()):
            cases[case["case_id"]] = case
        for line in (part / "partial_results.jsonl").read_text().splitlines():
            if line.strip().startswith("{"):
                rows.append(json.loads(line))
    return rows, cases


MARGIN_TOLERANCE_M = 0.19


def gap_table(rows, cases):
    """Passes per method and gap, over all curvatures and speeds.

    ``passed`` is a collision-free pass. ``passed_with_margin`` additionally
    keeps the 0.2 m obstacle margin every method is asked for, so that a
    geometry that under-covers the body cannot score by grazing the obstacle.
    """
    table = defaultdict(lambda: [0, 0, 0])
    for row in rows:
        case = cases[row["case_id"]]
        key = (row["method"], case["gap"], case["kappa"] == 0.0)
        passed = bool(row.get("success") and row.get("passed_obstacle"))
        clearance = row.get("min_obstacle_clearance_m")
        table[key][0] += 1
        table[key][1] += passed
        table[key][2] += passed and clearance is not None and clearance >= MARGIN_TOLERANCE_M
    out = []
    for (method, gap, straight), (n, passed, kept) in sorted(table.items()):
        out.append(
            {
                "method": method,
                "gap_m": gap,
                "road": "straight" if straight else "curve",
                "cases": n,
                "passed": passed,
                "passed_with_margin": kept,
            }
        )
    return out


def curb_table(rows, cases):
    table = defaultdict(list)
    for row in rows:
        table[row["method"]].append(row)
    out = []
    for method, group in sorted(table.items()):
        clearances = [
            r["min_road_clearance_m"]
            for r in group
            if r.get("min_road_clearance_m") is not None
        ]
        out.append(
            {
                "method": method,
                "cases": len(group),
                "completed": sum(1 for r in group if r.get("success")),
                "evaluated": len(clearances),
                "curb_strikes": sum(1 for c in clearances if c < 0.0),
                "margin_violations": sum(1 for c in clearances if c < MARGIN_TOLERANCE_M),
                "min_clearance_m": min(clearances, default=None),
                "median_clearance_m": sorted(clearances)[len(clearances) // 2]
                if clearances
                else None,
            }
        )
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=["gap", "curb"], required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rows, cases = load(args.input)
    table = gap_table(rows, cases) if args.task == "gap" else curb_table(rows, cases)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    print(f"{len(rows)} runs, wrote {args.output}")


if __name__ == "__main__":
    main()
