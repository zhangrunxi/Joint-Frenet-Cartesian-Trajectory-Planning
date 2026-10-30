"""Illustration of the hierarchical constraint pruning of Section 5.2.

Reviewer 3 asks for a figure showing the pruning procedure: extraction of the
critical pair, its correction, and the resulting constraint. The figure is
drawn from the model's own component distances, so it shows the pair the
implementation would actually select rather than a hand-drawn sketch.
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import casadi as ca

from paper_comparison.model import (
    hybrid_component_distances,
    hybrid_parts,
    transform_local,
)
from reiter2023.geometry import CircularRoad

# Component order as returned by hybrid_component_distances.
LABELS = ["ego segment / other segment"]


def component_labels():
    labels = ["ego segment / other segment"]
    labels += [f"ego corner {i + 1} / other segment" for i in range(4)]
    labels += [f"ego segment / other corner {i + 1}" for i in range(4)]
    labels += [
        f"ego corner {i + 1} / other corner {j + 1}" for i in range(4) for j in range(4)
    ]
    return labels


def draw_body(ax, pose, color, label, alpha=1.0):
    """Draw the bus footprint: central capsule plus four corner circles.

    The true rectangle is drawn faintly so the reader can see that the
    capsule-plus-corner-circles model covers the body exactly.
    """
    a, b, r, circles, rc = hybrid_parts(pose)
    a = np.asarray(a).ravel()
    b = np.asarray(b).ravel()
    corners = []
    for ds, dn in ((0.0, -1.15), (5.5, -1.15), (5.5, 1.15), (0.0, 1.15)):
        corners.append(
            np.asarray(transform_local(pose, ds, dn)).ravel()[:2]
        )
    ax.add_patch(
        plt.Polygon(
            corners,
            closed=True,
            facecolor=color,
            alpha=0.08 * alpha,
            edgecolor=color,
            linewidth=0.8,
            linestyle="--",
            zorder=1,
        )
    )
    for centre in circles:
        centre = np.asarray(centre).ravel()
        ax.add_patch(
            Circle(
                (centre[0], centre[1]),
                rc,
                fill=False,
                edgecolor=color,
                linewidth=1.4,
                alpha=alpha,
                zorder=3,
            )
        )
    ax.plot(
        [a[0], b[0]],
        [a[1], b[1]],
        color=color,
        linewidth=2.0,
        alpha=alpha,
        zorder=2,
        label=label,
    )
    return a, b, r, circles, rc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    road = CircularRoad(1 / 60, 4.6)
    ego = np.asarray(road.pose(10.0, 1.0, 0.12)).ravel()
    other = np.asarray(road.pose(14.0, 0.0, 0.0)).ravel()
    values = [
        float(ca.evalf(v)) for v in hybrid_component_distances(ego, other, 0.2)
    ]
    labels = component_labels()
    order = int(np.argmin(values))
    critical = labels[order]

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.6))

    # Left: the geometry with the selected pair highlighted.
    ax = axes[0]
    ego_parts = draw_body(ax, ego, "#2f6f9f", "ego bus")
    other_parts = draw_body(ax, other, "#c1533a", "obstacle bus")

    # Highlight the two members of the critical pair. The pair order follows
    # hybrid_component_distances: index 0 is the two segments, 1-4 the ego
    # corners against the other segment, 5-8 the ego segment against the
    # other corners, 9-24 the corner-against-corner pairs.
    ego_a, ego_b = ego_parts[0], ego_parts[1]
    other_circles = other_parts[3]
    if order == 0:
        members = [(ego_a, ego_b), (other_parts[0], other_parts[1])]
    elif 1 <= order <= 4:
        centre = np.asarray(ego_parts[3][order - 1]).ravel()
        members = [(centre, centre), (other_parts[0], other_parts[1])]
    elif 5 <= order <= 8:
        centre = np.asarray(other_circles[order - 5]).ravel()
        members = [(ego_a, ego_b), (centre, centre)]
    else:
        index = order - 9
        first = np.asarray(ego_parts[3][index // 4]).ravel()
        second = np.asarray(other_circles[index % 4]).ravel()
        members = [(first, first), (second, second)]

    for first, second in members:
        ax.plot(
            first[0],
            first[1],
            "o",
            color="#1b1b1b",
            markersize=7,
            markerfacecolor="none",
            markeredgewidth=1.6,
            zorder=6,
        )
        ax.plot(
            second[0],
            second[1],
            "o",
            color="#1b1b1b",
            markersize=7,
            markerfacecolor="none",
            markeredgewidth=1.6,
            zorder=6,
        )
    ax.set_title("Step 1-2: geometry and the selected critical pair")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper left", fontsize=8)

    # Right: the component distances in their original order, with the
    # selected one marked. The original order is kept so that the pair names
    # and the plotted positions refer to the same component.
    ax = axes[1]
    ranks = np.arange(len(values))
    ax.plot(ranks, values, "-o", color="#555555", ms=3)
    ax.axhline(0.0, color="#999999", lw=0.8, ls="--")
    ax.scatter([order], [values[order]], s=70, color="#c1533a", zorder=5)
    rank = int(np.sum(np.asarray(values) < values[order])) + 1
    ax.annotate(
        f"critical pair (rank {rank} of {len(values)})\n{critical}\n"
        f"d = {values[order]:+.3f} m",
        xy=(order, values[order]),
        xytext=(0.42, 0.06),
        textcoords="axes fraction",
        arrowprops=dict(arrowstyle="->", color="#c1533a"),
        fontsize=8,
    )
    ax.set_title("Step 3: keep the smallest distance only")
    ax.set_xlabel("component pair index (fixed order)")
    ax.set_ylabel("signed distance [m]")
    ax.grid(alpha=0.25)

    fig.suptitle(
        "Hierarchical constraint pruning: critical-pair extraction, correction "
        "and constraint generation",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.output, dpi=200)
    print(f"critical pair: {critical} ({values[order]:+.4f} m)")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
