"""Fig. 7 extended with the proposed model and the five-circle covering.

The manuscript's Fig. 7 plots the three-circle covering only. This figure adds
the proposed capsule-plus-corner-circle model and the five-circle covering
over the same five vehicle types.

All three curves reproduce the numbers printed in the manuscript for the
minibus: 0.77 m and 1.16 m for the three-circle covering, 1.08 m and 0.48 m
for the five-circle covering, and 0.43 m for the proposed model.
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rows = json.loads((args.input / "geometry_redundancy.json").read_text())
    vehicles = []
    for row in rows:
        if row["vehicle"] not in vehicles:
            vehicles.append(row["vehicle"])

    series = {}
    for row in rows:
        series.setdefault(row["model"], []).append(row)

    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.6))
    styles = {
        "circles3": ("#3f6ea8", "o", "three-circle covering (Eq. 11-12)"),
        "circles5": ("#c07b3a", "s", "five-circle covering (Eq. 11-14, N = 5)"),
        "hybrid_corners": (
            "#2e7d5b",
            "^",
            "proposed capsule + corner circles (Eq. 22)",
        ),
    }

    for ax, key, label in zip(axes, ("delta_l_m", "delta_w_m"), ("Longitudinal", "Lateral")):
        for model, (color, marker, name) in styles.items():
            entries = series.get(model, [])
            if not entries:
                continue
            ax.plot(
                [entry["aspect_ratio"] for entry in entries],
                [entry[key] for entry in entries],
                "-",
                marker=marker,
                color=color,
                linewidth=1.8,
                markersize=6,
                label=name,
            )
        ax.set_xlabel(r"Aspect Ratio  $\lambda = L_{veh}/W_{veh}$")
        ax.set_ylabel(f"{label} Redundancy [m]")
        ax.grid(alpha=0.25)
        # The legend sits below the axes so that it cannot cover a curve, and
        # the x limits leave room for the rightmost vehicle label.
        ax.set_xlim(right=max(
            entry["aspect_ratio"]
            for entries in series.values()
            for entry in entries
        ) + 0.45)

    for entry in series.get("circles3", []):
        axes[0].annotate(
            entry["vehicle"],
            xy=(entry["aspect_ratio"], entry["delta_l_m"]),
            xytext=(5, 7),
            clip_on=False,
            textcoords="offset points",
            fontsize=6.5,
            color="#3f6ea8",
        )

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        fontsize=8,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.905),
        ncol=3,
        frameon=False,
    )
    fig.suptitle(
        "Redundancy across vehicle types: the three-circle and five-circle "
        "coverings and the proposed capsule-plus-corner-circle model",
        fontsize=11,
        y=0.99,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
