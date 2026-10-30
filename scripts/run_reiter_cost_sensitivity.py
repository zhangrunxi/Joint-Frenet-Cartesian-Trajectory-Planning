"""Reiter acceleration-cost sensitivity; original results remain immutable."""

import argparse
import csv
import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import run_paper_comparison as comparison

from paper_comparison.model import Settings

OriginalBusReiter = comparison.BusReiter


class AccelerationMatchedReiter(OriginalBusReiter):
    def __init__(self, vehicle, road, options, obstacles):
        # F=m*a, so rF=1/m^2 makes rF*F^2 = a^2.
        options = replace(options, r0=(1 / vehicle.mass**2, options.r0[1]))
        super().__init__(vehicle, road, options, obstacles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/reiter_acceleration_cost_sensitivity",
    )
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Refusing to overwrite sensitivity output")
    args.output.mkdir(parents=True)
    comparison.BusReiter = AccelerationMatchedReiter
    summaries = []
    for width, lateral in [(8.5, 5.5), (4.6, 2.6)]:
        output = args.output / f"half_width_{width}"
        output.mkdir()
        rows = []
        for method in comparison.METHODS[:4]:
            for scenario in ["straight", "curve"]:
                result = comparison.run_case(
                    output,
                    method,
                    scenario,
                    120,
                    replace(Settings(), lateral=lateral),
                    width,
                )
                result["cost_sensitivity"] = {
                    "acceleration_square_weight": 1.0,
                    "original_acceleration_square_weight": 1345.6,
                    "other_costs_and_constraints_unchanged": True,
                    "not_fully_matched_model_objective": True,
                }
                (output / (result["name"] + ".json")).write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n"
                )
                row = {
                    k: v
                    for k, v in result.items()
                    if k
                    not in [
                        "states",
                        "controls",
                        "cycles",
                        "startup",
                        "settings",
                        "cost_sensitivity",
                    ]
                }
                rows.append(row)
                summaries.append({**row, "acceleration_square_weight": 1.0})
        (output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
        with (output / "summary.csv").open("w") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
    (args.output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    with (args.output / "summary.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    sources = [
        Path(__file__),
        ROOT / "scripts/run_paper_comparison.py",
        ROOT / "paper_comparison/model.py",
        ROOT / "reiter2023/trajectory_planner_acados_20222.py",
    ]
    (args.output / "provenance.json").write_text(
        json.dumps(
            {
                "command": sys.argv,
                "adaptation": "only Reiter force-cost weight changed from0.001 to1/mass^2",
                "mass_scaling_kg": 1160.0,
                "acceleration_square_weight": 1.0,
                "limitation": "Not a fully cost- or dynamics-matched experiment: jerk states/constraints remain different.",
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
