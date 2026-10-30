"""Exact circular-road body-clearance sensitivity, not the original paper model."""

import argparse
import csv
import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path

import casadi as ca

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import run_paper_comparison as comparison

from paper_comparison.model import (
    PaperModel,
    Settings,
    point_segment_squared,
    transform_local,
)


class ExactRoadPaperModel(PaperModel):
    """Replace only four path road entries; hold every other setting unchanged."""

    def _symbolic(self):
        super()._symbolic()
        k = self.road.kappa
        pose = self.road.pose(self.x[0], self.x[1], self.x[2])
        corners = [
            transform_local(pose, ds, dn)
            for ds, dn in [(-1.0, -1.15), (5.5, -1.15), (5.5, 1.15), (-1.0, 1.15)]
        ]
        if abs(k) < 1e-12:
            clearances = [
                self.road.half_width - ca.mmax(ca.vertcat(*[c[1] for c in corners])),
                self.road.half_width + ca.mmin(ca.vertcat(*[c[1] for c in corners])),
            ]
        else:
            center = ca.DM([0, 1 / k])
            radius = abs(1 / k)
            min_distance = ca.sqrt(
                ca.mmin(
                    ca.vertcat(
                        *[
                            point_segment_squared(center, a, b)
                            for a, b in zip(corners, corners[1:] + corners[:1])
                        ]
                    )
                )
                + 1e-16
            )
            # If the circle center lies inside the rectangular body, minimum radius is zero.
            relative = center - pose[:2]
            local_x = ca.cos(pose[2]) * relative[0] + ca.sin(pose[2]) * relative[1]
            local_y = -ca.sin(pose[2]) * relative[0] + ca.cos(pose[2]) * relative[1]
            inside = ca.logic_and(
                ca.logic_and(local_x >= -1, local_x <= 5.5), ca.fabs(local_y) <= 1.15
            )
            min_distance = ca.if_else(inside, 0, min_distance)
            max_distance = ca.sqrt(
                ca.mmax(ca.vertcat(*[ca.sumsqr(c - center) for c in corners])) + 1e-16
            )
            clearances = [
                min_distance - (radius - self.road.half_width),
                radius + self.road.half_width - max_distance,
            ]
        # Keep h dimension identical for the existing solver/interface; duplicate exact bounds.
        self.h = ca.vertcat(self.h[:4], *clearances, *clearances, self.h[8:])
        self.hl[4:8] = self.settings.road_margin
        self.hu[4:8] = 1e8
        self.hfun = ca.Function("exact_road_h", [self.x, self.u, self.p], [self.h])
        self.g = ca.vertcat(
            self.hl - self.h,
            self.h - self.hu,
            self.xl - self.x[self.state_indices],
            self.x[self.state_indices] - self.xu,
        )
        self.gfun = ca.Function("exact_road_g", [self.x, self.u, self.p], [self.g])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/exact_road_sensitivity"
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refusing to overwrite existing sensitivity")
    args.output.mkdir(parents=True)
    comparison.PaperModel = ExactRoadPaperModel
    rows = []
    for width in [4.3, 4.6, 4.9]:
        output = args.output / f"half_width_{width}"
        output.mkdir()
        for method in ["paper6_hybrid_sqp", "paper6_circles_sqp"]:
            for scenario in ["straight", "curve"]:
                result = comparison.run_case(
                    output,
                    method,
                    scenario,
                    120,
                    replace(Settings(), lateral=width - 2),
                    width,
                )
                result["road_variant"] = (
                    "Exact straight/circular full-body road bound, NOT original approximate paper constraint"
                )
                (output / (result["name"] + ".json")).write_text(
                    json.dumps(result, indent=2) + "\n"
                )
                rows.append(
                    {
                        k: v
                        for k, v in result.items()
                        if k
                        not in ["states", "controls", "cycles", "startup", "settings"]
                    }
                )
    (args.output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
    with (args.output / "summary.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    sources = [
        Path(__file__),
        ROOT / "scripts/run_paper_comparison.py",
        ROOT / "paper_comparison/model.py",
    ]
    (args.output / "provenance.json").write_text(
        json.dumps(
            {
                "command": sys.argv,
                "variant": "exact annular body road constraint; not original paper model",
                "sha256": {
                    str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sources
                },
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
