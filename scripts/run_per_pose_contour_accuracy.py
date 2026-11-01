"""Per-pose accuracy of the contour road-boundary model.

Evaluate the implemented corner lateral offsets against an independent exact
projection onto a straight line or circle. The older sweep incorrectly uses
Cartesian x as Frenet station when constructing its reference values. This
script measures lateral projection error, not the manuscript's full Cartesian
position RMSE; the latter requires a defined longitudinal reconstruction.
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
from paper_comparison.model import approximate_road_offsets
from paper_comparison.vehicle import GEOMETRY
from reiter2023.geometry import CircularRoad

CORNERS = [
    (-GEOMETRY.rear_overhang, -GEOMETRY.half_width),
    (GEOMETRY.front_extent, -GEOMETRY.half_width),
    (GEOMETRY.front_extent, GEOMETRY.half_width),
    (-GEOMETRY.rear_overhang, GEOMETRY.half_width),
]
NAMES = ["RR", "FR", "FL", "RL"]


def exact_offsets(s, n, alpha, road):
    corners = rectangle_vertices(road.numeric_pose([s, n, alpha]))
    if abs(road.kappa) < 1e-12:
        return corners[:, 1]
    k = road.kappa
    return (1 - np.hypot(k * corners[:, 0], 1 - k * corners[:, 1])) / k


def predicted_offsets(s, n, alpha, road, segments, frozen_heading=False):
    state = np.array([s, n, alpha, 0.0, 0.0, 0.0])
    values = np.asarray(approximate_road_offsets(
        state, road, segments, frozen_heading)).ravel()
    return values[[0, 2, 3, 1]]


def sweep(kappa, alphas, offsets, segments_list, frozen_heading=False):
    road = CircularRoad(kappa, 8.5)
    rows = []
    for segments in segments_list:
        errors = {name: [] for name in NAMES}
        lateral = {name: [] for name in NAMES}
        for alpha in alphas:
            for n in offsets:
                exact = exact_offsets(0.0, n, alpha, road)
                predicted = predicted_offsets(
                    0.0, n, alpha, road, segments, frozen_heading
                )
                distances = np.abs(predicted - exact)
                for index, name in enumerate(NAMES):
                    errors[name].append(float(distances[index]))
                    lateral[name].append(float(abs(exact[index] - predicted[index])))
        row = {
            "n_disc": segments,
            "kappa": kappa,
            "frozen_heading": frozen_heading,
        }
        for index, name in enumerate(NAMES):
            row[f"rmse_{name}"] = float(np.sqrt(np.mean(np.square(errors[name]))))
            row[f"max_{name}"] = float(np.max(errors[name]))
            row[f"lateral_rmse_{name}"] = float(
                np.sqrt(np.mean(np.square(lateral[name])))
            )
        worst = np.max(np.asarray([errors[name] for name in NAMES]), axis=0)
        row["rmse_max"] = float(np.sqrt(np.mean(np.square(worst))))
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--kappa", nargs="+", type=float, default=[0.0, 1 / 60, 0.05, 0.1])
    parser.add_argument("--alphas", nargs="+", type=float, default=[0.0, 0.05, 0.1, 0.2])
    parser.add_argument("--offsets", nargs="+", type=float, default=[0.0, 1.0, 2.0])
    parser.add_argument("--segments", nargs="+", type=int, default=[1, 2, 5, 10, 20, 40])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    for frozen in (False, True):
        for kappa in args.kappa:
            rows = sweep(kappa, args.alphas, args.offsets, args.segments, frozen)
            tag = f"kappa{round(kappa, 4)}{'_frozen' if frozen else ''}"
            keys = list(rows[0])
            with open(args.output / f"per_pose_{tag}.csv", "w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=keys)
                writer.writeheader()
                writer.writerows(rows)
            summary.extend(rows)
    (args.output / "per_pose_summary.json").write_text(json.dumps(summary, indent=2))
    (args.output / "config.json").write_text(
        json.dumps(
            {
                "vehicle": GEOMETRY.name,
                "corners": CORNERS,
                "alphas": args.alphas,
                "offsets": args.offsets,
                "kappa": args.kappa,
                "segments": args.segments,
                "metric": "corner lateral projection error in metres, RMSE over the pose grid; not Cartesian position RMSE",
            },
            indent=2,
        )
    )
    for row in summary:
        print(
            f"kappa={row['kappa']:.4f} frozen={row['frozen_heading']} "
            f"N={row['n_disc']:2d} FR_rmse={row['rmse_FR']:.5f} "
            f"FR_max={row['max_FR']:.5f} FL_rmse={row['rmse_FL']:.5f}"
        )


if __name__ == "__main__":
    main()
