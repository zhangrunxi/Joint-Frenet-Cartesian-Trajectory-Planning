"""Fig. 7 extended with the proposed model and the five-circle covering.

The manuscript's Fig. 7 plots the three-circle covering only. This figure adds
the proposed capsule-plus-corner-circle model and the five-circle covering
over the same five vehicle types.

The three-circle and hybrid curves reproduce the numbers printed in the
manuscript (0.77 m and 1.16 m for the three-circle minibus, 0.43 m for the
hybrid minibus). The five-circle reconstruction from Eq. 18-20 does not
reproduce the 1.08 m and 0.48 m quoted in the text, so the manuscript's
minibus values are marked separately and the definition has to be checked
against the LaTeX source before the curve is published.
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# The values the manuscript quotes for the minibus, for reference.
PAPER_FIVE_CIRCLE = {"delta_l_m": 1.08, "delta_w_m": 0.48}


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
    # Only the two curves that reproduce the manuscript's printed values are
    # drawn. The five-circle reconstruction does not reproduce the manuscript's
    # 1.08 m and 0.48 m, so its curve is withheld and the manuscript's own
    # minibus value is shown as a marked point instead.
    styles = {
        "circles3": ("#3f6ea8", "o", "three-circle covering (Eq. 11-12)"),
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
        ax.legend(fontsize=7.5, loc="best")

    # Mark the manuscript's five-circle minibus values, which the
    # reconstruction does not reproduce.
    minibus = [
        row
        for row in series.get("circles5", [])
        if row["vehicle"] == "Minibus"
    ]
    if minibus:
        entry = minibus[0]
        for ax, key, offset in zip(
            axes,
            ("delta_l_m", "delta_w_m"),
            ((0.30, 0.90), (0.30, 0.78)),
        ):
            ax.scatter(
                [entry["aspect_ratio"]],
                [PAPER_FIVE_CIRCLE[key]],
                marker="x",
                s=80,
                linewidths=2.2,
                color="#c07b3a",
                zorder=6,
            )
            ax.annotate(
                "five-circle, manuscript value",
                xy=(entry["aspect_ratio"], PAPER_FIVE_CIRCLE[key]),
                xytext=offset,
                textcoords="axes fraction",
                fontsize=7,
                color="#8a5524",
                arrowprops=dict(arrowstyle="->", color="#c07b3a", lw=0.8),
            )

    for entry in series.get("circles3", []):
        axes[0].annotate(
            entry["vehicle"],
            xy=(entry["aspect_ratio"], entry["delta_l_m"]),
            xytext=(5, 7),
            textcoords="offset points",
            fontsize=6.5,
            color="#3f6ea8",
        )

    fig.suptitle(
        "Redundancy across vehicle types: three-circle covering and the "
        "proposed capsule-plus-corner-circle model",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
