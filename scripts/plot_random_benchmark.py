"""Plot the randomized benchmark summary.

Produces a completion/overtake panel and a clearance panel from
``results.json`` written by ``run_random_benchmark.py``.
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from summarize_random_benchmark import BACKENDS, TIMING_KEY, infeasible

LABELS = {
    "reiter_direct_ellipse": "Reiter Direct / ellipse",
    "reiter_direct_circles": "Reiter Direct / circles",
    "reiter_lifted_ellipse": "Reiter Lifted / ellipse",
    "reiter_lifted_circles": "Reiter Lifted / circles",
    "reiter_direct_ellipse_point": "Reiter Direct / ellipse (point bound)",
    "paper6_hybrid_sqp": "Ours 6-state hybrid",
    "paper9_hybrid_sqp": "Ours 9-state hybrid",
    "paper6_circles_sqp": "Ours 6-state circles",
    "paper6_circles5_sqp": "Ours 6-state five circles",
    "paper6_circlesN3_sqp": "Ours 6-state three circles (Eq. 11-14)",
    "paper6_circlesN5_sqp": "Ours 6-state five circles (Eq. 11-14)",
    "paper6_frozen_sqp": "Ours 6-state frozen heading",
    "paper6_hybrid_cilqr": "Ours 6-state AL-CiLQR",
    "xing_joint_corridor": "Xing joint keep-left",
    "xing_joint_yield": "Xing joint yield",
    "xing_joint_corridor_point": "Xing joint (point bound)",
}

ORDER = [
    "reiter_direct_ellipse",
    "reiter_direct_circles",
    "reiter_lifted_ellipse",
    "reiter_lifted_circles",
    "reiter_direct_ellipse_point",
    "paper6_hybrid_sqp",
    "paper9_hybrid_sqp",
    "paper6_circles_sqp",
    "paper6_circles5_sqp",
    "paper6_frozen_sqp",
    "paper6_hybrid_cilqr",
    "xing_joint_corridor",
    "xing_joint_corridor_point",
    "xing_joint_yield",
]


def collect(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row["method"], []).append(row)
    return grouped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rows = json.loads((args.input / "results.json").read_text())
    grouped = collect(rows)
    methods = [m for m in ORDER if m in grouped]
    labels = [LABELS.get(m, m) for m in methods]

    figure, axes = plt.subplots(1, 3, figsize=(15, 5.2))

    positions = np.arange(len(methods))
    completions, overtakes, feasible = [], [], []
    for method in methods:
        group = grouped[method]
        good = [r for r in group if not infeasible(r)]
        feasible.append(len(good))
        completions.append(
            sum(1 for r in good if r.get("success")) / len(good) if good else 0
        )
        overtakes.append(
            sum(1 for r in good if r.get("passed_obstacle")) / len(good) if good else 0
        )

    width = 0.38
    axes[0].bar(positions - width / 2, completions, width, label="completed")
    axes[0].bar(positions + width / 2, overtakes, width, label="overtaken")
    axes[0].set_ylabel("rate over feasible scenarios")
    axes[0].set_title("(a) Completion and overtake rate")
    axes[0].legend(frameon=False)
    axes[0].set_ylim(0, 1.05)

    for axis, key, title in (
        (axes[1], "min_obstacle_clearance_m", "(b) Minimum obstacle clearance"),
        (axes[2], "min_road_clearance_m", "(c) Minimum road clearance"),
    ):
        for index, method in enumerate(methods):
            group = [r for r in grouped[method] if not infeasible(r)]
            values = [r.get(key) for r in group if r.get(key) is not None]
            if not values:
                continue
            axis.scatter(
                np.full(len(values), index) + np.random.default_rng(index).normal(0, 0.06, len(values)),
                values,
                s=12,
                alpha=0.55,
            )
            axis.scatter(
                [index], [np.median(values)], marker="_", s=220, color="black", zorder=3
            )
        axis.axhline(0.2, color="tab:gray", linestyle="--", linewidth=1, label="0.2 m margin")
        axis.set_title(title)
        axis.set_ylabel("clearance (m)")
        axis.legend(frameon=False)

    for axis in axes:
        axis.set_xticks(positions)
        axis.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
        axis.grid(axis="y", alpha=0.25)

    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
