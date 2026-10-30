"""Finite-grid geometry rejection audit, independent of optimizer convergence."""

import argparse
import csv
import json
import sys
from pathlib import Path

import casadi as ca
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from paper_comparison.model import bus_vehicle, gym_pose, hybrid_constraints
from reiter2023.geometry import covering_circles, ellipse_constraint


def functions():
    pose = ca.MX.sym("pose", 3)
    other = ca.DM.zeros(3)
    v = bus_vehicle()
    aa, ra = covering_circles(gym_pose(pose), v, 3)
    bb, rb = covering_circles(gym_pose(other), v, 3)
    circle = ca.vertcat(
        *[ca.sqrt(ca.sumsqr(a - b) + 1e-16) - ra - rb - 0.2 for a in aa for b in bb]
    )
    aa5, ra5 = covering_circles(gym_pose(pose), v, 5)
    bb5, rb5 = covering_circles(gym_pose(other), v, 5)
    circle5 = ca.vertcat(
        *[
            ca.sqrt(ca.sumsqr(a - b) + 1e-16) - ra5 - rb5 - 0.2
            for a in aa5
            for b in bb5
        ]
    )
    return {
        "circles5": ca.Function(
            "circles5_grid", [pose], [ca.mmin(circle5)]
        ),
        "hybrid": ca.Function(
            "hybrid_grid", [pose], [ca.mmin(hybrid_constraints(pose, other, 0.2))]
        ),
        "circles": ca.Function("circles_grid", [pose], [ca.mmin(circle)]),
        "ellipse": ca.Function(
            "ellipse_grid",
            [pose],
            [ellipse_constraint(gym_pose(pose), gym_pose(other), v, v, 0.2)],
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/geometry_conservatism"
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refusing to overwrite existing geometry audit")
    args.output.mkdir(parents=True)
    # Same-heading rectangles admit an exact analytic Euclidean separation oracle.
    xx, yy = np.meshgrid(np.linspace(-12, 12, 481), np.linspace(-7, 7, 281))
    exact = np.hypot(np.maximum(np.abs(xx) - 6.5, 0), np.maximum(np.abs(yy) - 2.3, 0))
    safe = exact >= 0.2 - 1e-10
    poses = np.vstack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])
    masks, rows = {}, []
    for name, fn in functions().items():
        mapped = fn.map(xx.size, "serial")
        values = np.asarray(mapped(poses)).ravel().reshape(xx.shape)
        accepted = values >= -1e-10
        masks[name] = accepted
        row = {
            "model": name,
            "grid_points": xx.size,
            "oracle_safe_points": int(safe.sum()),
            "accepted_points": int(accepted.sum()),
            "unsafe_accepted_points": int((accepted & ~safe).sum()),
            "safe_rejected_points": int((safe & ~accepted).sum()),
            "safe_rejected_fraction": float((safe & ~accepted).sum() / safe.sum()),
            "same_s_lateral_threshold_m": None,
        }
        # Numerical bisection on n>0, same longitudinal rear-axle coordinate.
        lo, hi = 0.0, 10.0
        for _ in range(50):
            middle = (lo + hi) / 2
            if float(fn([0, middle, 0])) >= 0:
                hi = middle
            else:
                lo = middle
        row["same_s_lateral_threshold_m"] = hi
        rows.append(row)
    columns = len(masks)
    fig, axes = plt.subplots(
        1, columns, figsize=(5 * columns, 4.6), sharex=True, sharey=True
    )
    axes = np.atleast_1d(axes)
    for ax, (name, accepted) in zip(axes, masks.items()):
        # 0=unsafe oracle,1=safe but representation rejects,2=accepted safe.
        image = np.where(safe, np.where(accepted, 2, 1), 0)
        ax.imshow(
            image,
            origin="lower",
            extent=(-12, 12, -7, 7),
            aspect="equal",
            cmap=matplotlib.colors.ListedColormap(["#34393d", "#efb86d", "#d9eee7"]),
            vmin=0,
            vmax=2,
        )
        ax.set_title(name)
        ax.set_xlabel("Rear-axle longitudinal offset [m]")
        ax.grid(alpha=0.15)
    axes[0].set_ylabel("Rear-axle lateral offset [m]")
    from matplotlib.patches import Patch

    fig.legend(
        handles=[
            Patch(color=c, label=l)
            for c, l in [
                ("#34393d", "True rectangles fail 0.2 m clearance"),
                ("#efb86d", "Safe rectangles rejected by surrogate"),
                ("#d9eee7", "Accepted safe rectangles"),
            ]
        ],
        loc="lower center",
        ncol=3,
    )
    fig.suptitle(
        "Aligned buses only: deterministic finite-grid conservatism, not a driving success rate"
    )
    fig.tight_layout(rect=(0, 0.1, 1, 0.93))
    fig.savefig(args.output / "geometry_conservatism.png", dpi=180)
    plt.close(fig)
    report = {
        "geometry": "6.5 x 2.3 m aligned identical rectangles, rear axle +2.25 m center",
        "oracle": "hypot(max(abs(ds)-6.5,0),max(abs(dn)-2.3,0)) >= 0.2",
        "grid": {"ds": [-12, 12], "dn": [-7, 7], "spacing_m": 0.05, "points": xx.size},
        "limitations": "One relative heading only; finite-domain grid fractions, not probabilities or continuous guarantees.",
        "results": rows,
    }
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    with (args.output / "summary.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(
        args.output / "grid.npz", longitudinal=xx, lateral=yy, oracle_safe=safe, **masks
    )
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
