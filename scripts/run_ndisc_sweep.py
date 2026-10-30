"""Contour-model accuracy as a function of the discretization count N_disc.

Reproduces the quantity behind Table 3 of the manuscript: the RMSE between the
corner positions predicted by the road-boundary contour model and the exact
Cartesian corner positions, for a straight and a curved reference line.

The contour model integrates the heading deviation along the vehicle body in
``n_disc`` piecewise-linear segments. The reference is the exact corner
position on the circular reference line, computed independently of the
optimization. No solver is involved, so the result isolates the
representation error that the reviewer asked about.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from paper_comparison.evaluation import rectangle_vertices
from paper_comparison.model import transform_local
from reiter2023.geometry import CircularRoad

# Rear axle at the origin, body 6.50 m long, 2.30 m wide, rear overhang 1.00 m.
CORNERS = [(-1.0, -1.15), (5.5, -1.15), (5.5, 1.15), (-1.0, 1.15)]
NAMES = ["RL", "RR", "FR", "FL"]


def approximate_offsets(s, n, alpha, kappa, segments, frozen_heading=False):
    """The four contour offsets used by the road-boundary constraint."""
    offsets = []
    for longitudinal, lateral in CORNERS:
        ds = longitudinal * np.cos(alpha) - lateral * np.sin(alpha)
        offset = n + lateral / np.cos(alpha)
        for i in range(segments):
            angle = (
                alpha if frozen_heading else alpha - kappa * ds * i / segments
            )
            offset += ds / segments * np.tan(angle)
        offsets.append(offset)
    return np.array(offsets)


def exact_offsets(s, n, alpha, road):
    """Exact Frenet lateral offset of each corner.

    The corner is placed on the body at its longitudinal station, then its
    lateral coordinate is measured against the reference line at that station.
    """
    pose = np.asarray(road.pose(s, n, alpha)).ravel()
    offsets = []
    for ds, dn in CORNERS:
        corner = np.asarray(transform_local(pose, ds, dn)).ravel()
        # Reference point at the corner's own longitudinal station.
        station = np.asarray(road.pose(corner[0], 0.0, 0.0)).ravel()
        direction = np.array([-np.sin(station[2]), np.cos(station[2])])
        offsets.append(float(np.dot(corner - station[:2], direction)))
    return np.array(offsets)


def predicted_offsets(s, n, alpha, road, segments, frozen_heading=False):
    """Lateral offsets the model constrains, one per corner."""
    return approximate_offsets(
        s, n, alpha, road.kappa, segments, frozen_heading
    )


def sweep(kappa, alphas, segments_list, samples, frozen_heading=False):
    road = CircularRoad(kappa, 8.5)
    rows = []
    for segments in segments_list:
        errors = {name: [] for name in NAMES}
        for alpha in alphas:
            for s in np.linspace(0.0, 40.0, samples):
                n = 1.0 + 0.02 * s
                exact = exact_offsets(s, n, alpha, road)
                predicted = predicted_offsets(
                    s, n, alpha, road, segments, frozen_heading
                )
                for index, name in enumerate(NAMES):
                    errors[name].append(float(abs(exact[index] - predicted[index])))
        rows.append(
            {
                "n_disc": segments,
                "kappa": kappa,
                "frozen_heading": frozen_heading,
                **{
                    f"rmse_{name.lower()}": float(np.sqrt(np.mean(np.square(errors[name]))))
                    for name in NAMES
                },
                "rmse_max": float(
                    np.sqrt(
                        np.mean(
                            np.square(
                                [
                                    value
                                    for name in NAMES
                                    for value in errors[name]
                                ]
                            )
                        )
                    )
                ),
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--segments", nargs="+", type=int, default=[1, 2, 5, 10, 20])
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--alphas", nargs="+", type=float, default=[0.0, 0.1, 0.2])
    parser.add_argument("--curvature", type=float, default=1 / 60)
    args = parser.parse_args()

    rows = []
    rows += sweep(0.0, args.alphas, args.segments, args.samples)
    rows += sweep(
        args.curvature, args.alphas, args.segments, args.samples
    )
    rows += sweep(
        args.curvature, args.alphas, args.segments, args.samples, frozen_heading=True
    )

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "ndisc_sweep.json").write_text(json.dumps(rows, indent=2) + "\n")
    with (args.output / "ndisc_sweep.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    for row in rows:
        label = "frozen" if row["frozen_heading"] else "integrated"
        print(
            f"kappa={row['kappa']:+.4f} {label:10s} N_disc={row['n_disc']:2d} "
            + "  ".join(
                f"{name}={row['rmse_' + name.lower()]:.4f}" for name in NAMES
            )
        )


if __name__ == "__main__":
    main()
