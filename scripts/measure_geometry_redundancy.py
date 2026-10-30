"""Longitudinal and lateral redundancy of the collision surrogates.

Reproduces the analysis behind Fig. 7 of the manuscript and extends it with
the proposed capsule-plus-corner-circle model and the five-circle covering
that the manuscript discusses in the text but does not plot.

The three-circle and general N-circle expressions are the manuscript's own
(Eq. 11-14):

    r(N)   = (W/2) * sqrt(1 + lambda^2 / N^2),      lambda = L / W
    dL(N)  = 2 * r(N) - L / N
    dW(N)  = 2 * r(N) - W

For N = 3 the first reduces to (1/9 + 1/lambda - 1/3) L and the second to
(sqrt(1 + lambda^2/9) - 1) W, which is Eq. 12. For the minibus in the
manuscript (L = 7.5 m, W = 2.1 m) this gives dL = 0.765 m and dW = 1.165 m,
matching the 0.77 m and 1.16 m quoted in the text.

The five-circle covering follows Eq. 18-20: two corner circles at each end
with radius r_c = sqrt(2) * W / 4, centred at x = +/-(L/2 - W/4), y = +/-W/4,
on top of the capsule of Eq. 15-17.

The proposed hybrid model follows Eq. 22: dL = dW = (W/2) * (sqrt(2) - 1).
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

# The five vehicle types of Fig. 7, with the dimensions printed in the figure.
VEHICLES = {
    "Smart": (2.69, 1.47),
    "Sedan": (4.80, 1.80),
    "SUV": (5.50, 1.90),
    "Minibus": (7.50, 2.10),
    "Coach": (12.00, 2.50),
}


def circle_redundancy(length, width, count):
    """Redundancy of the N-circle covering, Eq. 11-14."""
    aspect = length / width
    radius = 0.5 * width * np.sqrt(1.0 + aspect**2 / count**2)
    return 2.0 * radius - length / count, 2.0 * radius - width


def capsule_redundancy(length, width):
    """Redundancy of the capsule of Eq. 15-17.

    The capsule is the union of circles of radius W/2 whose centres run along
    the longitudinal axis. It cannot enclose the corners of the rectangle, so
    the corner intervals at both ends are not covered.
    """
    radius = 0.5 * width
    # The capsule covers the central region only; the uncovered length at both
    # ends is what the corner circles have to add.
    covered_length = length - 2.0 * (0.5 * length - 0.5 * width)
    return length - covered_length, 2.0 * radius - width


def five_circle_redundancy(length, width):
    """Redundancy of the five-circle covering of Eq. 18-20.

    Two corner circles at the front and two at the rear, radius
    r_c = sqrt(2) * W / 4, centred at x = +/-(L/2 - W/4), y = +/- W/4.
    """
    radius = np.sqrt(2.0) * width / 4.0
    front_x = 0.5 * length - 0.25 * width
    # The front circles reach to front_x + r_c in x and to 0.25*W + r_c in y.
    covered_length = 2.0 * (front_x + radius)
    covered_width = 2.0 * (0.25 * width + radius)
    return covered_length - length, covered_width - width


def hybrid_redundancy(length, width):
    """Redundancy of the proposed capsule-plus-corner-circle model, Eq. 22."""
    value = 0.5 * width * (np.sqrt(2.0) - 1.0)
    return value, value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    models = (
        ("circles3", circle_redundancy),
        ("circles5", five_circle_redundancy),
        ("hybrid_corners", hybrid_redundancy),
    )
    rows = []
    for name, (length, width) in VEHICLES.items():
        aspect = length / width
        for label, function in models:
            if label == "circles3":
                delta_l, delta_w = function(length, width, 3)
            elif label == "circles5":
                delta_l, delta_w = function(length, width)
            else:
                delta_l, delta_w = function(length, width)
            rows.append(
                {
                    "vehicle": name,
                    "length_m": length,
                    "width_m": width,
                    "aspect_ratio": aspect,
                    "model": label,
                    "delta_l_m": delta_l,
                    "delta_w_m": delta_w,
                }
            )

    (args.output / "geometry_redundancy.json").write_text(
        json.dumps(rows, indent=2) + "\n"
    )
    with (args.output / "geometry_redundancy.csv").open("w") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"{'vehicle':9s} {'lambda':>7s} {'model':16s} {'dL [m]':>8s} {'dW [m]':>8s}")
    for row in rows:
        print(
            f"{row['vehicle']:9s} {row['aspect_ratio']:7.3f} {row['model']:16s} "
            f"{row['delta_l_m']:8.3f} {row['delta_w_m']:8.3f}"
        )

    # Cross-check against the numbers printed in the manuscript.
    minibus = [
        row
        for row in rows
        if row["vehicle"] == "Minibus" and row["model"] == "circles3"
    ][0]
    hybrid = [
        row
        for row in rows
        if row["vehicle"] == "Minibus" and row["model"] == "hybrid_corners"
    ][0]
    five = [
        row
        for row in rows
        if row["vehicle"] == "Minibus" and row["model"] == "circles5"
    ][0]
    print()
    print(
        f"minibus three-circle: dL = {minibus['delta_l_m']:.3f} m, "
        f"dW = {minibus['delta_w_m']:.3f} m  (manuscript: 0.77 m, 1.16 m)"
    )
    print(
        f"minibus hybrid:       dL = dW = {hybrid['delta_l_m']:.3f} m  "
        f"(manuscript: 0.43 m)"
    )
    print(
        f"minibus five-circle:  dL = {five['delta_l_m']:.3f} m, "
        f"dW = {five['delta_w_m']:.3f} m  (manuscript: 1.08 m, 0.48 m)"
    )
    print()
    print(
        "The three-circle and hybrid values reproduce the numbers printed in the "
        "manuscript. The five-circle reconstruction does not: the manuscript "
        "quotes 1.08 m and 0.48 m, the reconstruction from Eq. 18-20 gives "
        f"{five['delta_l_m']:.3f} m and {five['delta_w_m']:.3f} m. The "
        "five-circle definition has to be checked against the LaTeX source "
        "before the curve is used."
    )


if __name__ == "__main__":
    main()
