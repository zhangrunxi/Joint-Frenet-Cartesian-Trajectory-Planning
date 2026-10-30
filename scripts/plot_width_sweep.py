"""Plot stored width-sweep scalars; completion is not obstacle passing."""

import argparse
import csv
import json
import sys
from pathlib import Path
from time import sleep

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
WIDTHS = (4.0, 4.3, 4.6, 4.9, 5.2, 5.5)
SCENARIOS = ("straight", "curve")
STYLES = {
    "reiter_lifted_circles": ("Reiter lifted / circles", "#16805d", "--"),
    "paper6_circles_sqp": ("Paper 6 / circles", "#2166ac", "-."),
    "paper6_hybrid_sqp": ("Paper 6 / hybrid", "#c73036", "-"),
}
METRICS = (
    ("progress_m", "Final rear-axle s [m]"),
    ("min_obstacle_clearance_m", "Minimum obstacle clearance [m]"),
    ("min_road_clearance_m", "Minimum road clearance [m]"),
)


def finite(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and np.isfinite(value)
    )


def read_summaries(directory, widths):
    """Require the full matrix, including explicitly unsuccessful case records."""
    records = []
    for width in widths:
        path = directory / f"half_width_{width:.1f}" / "summary.json"
        data = json.loads(path.read_text())
        if not isinstance(data, list) or len(data) != len(STYLES) * len(SCENARIOS):
            raise ValueError(f"Expected six case summaries: {path}")
        seen = set()
        for record in data:
            key = (record.get("method"), record.get("scenario"))
            if key in seen or key[0] not in STYLES or key[1] not in SCENARIOS:
                raise ValueError(
                    f"Duplicate or unexpected method/scenario: {path}: {key}"
                )
            seen.add(key)
            if not finite(record.get("half_width")) or not np.isclose(
                record["half_width"], width, atol=1e-10, rtol=0
            ):
                raise ValueError(f"Width mismatch in {path}")
            completed, requested = (
                record.get("completed_steps"),
                record.get("steps_requested"),
            )
            if (
                not isinstance(completed, int)
                or not isinstance(requested, int)
                or not 0 <= completed <= requested
                or requested != 120
                or not isinstance(record.get("success"), bool)
            ):
                raise ValueError(
                    f"Missing/invalid run completion metadata: {path}: {key}"
                )
            if record["success"] and completed != requested:
                raise ValueError(f"Successful case has incomplete steps: {path}: {key}")
            for field, _ in METRICS:
                if record.get(field) is not None and not finite(record[field]):
                    raise ValueError(f"Invalid scalar {field}: {path}: {key}")
            records.append({**record, "source_summary": str(path.resolve())})
    return records


def complete(record):
    return record["success"] and record["completed_steps"] == record["steps_requested"]


def export_csv(records, output):
    rows = [
        {
            key: value
            for key, value in record.items()
            if value is None or isinstance(value, (str, bool, int, float))
        }
        for record in records
    ]
    leading = [
        "half_width",
        "method",
        "scenario",
        "success",
        "completed_steps",
        "steps_requested",
    ]
    columns = leading + sorted(
        set().union(*(row.keys() for row in rows)) - set(leading)
    )
    with (output / "width_sweep.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def draw(records, widths, output):
    figure, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    for column, scenario in enumerate(SCENARIOS):
        axes[0, column].set_title(scenario.capitalize())
        for method, (_, color, linestyle) in STYLES.items():
            selected = sorted(
                [
                    record
                    for record in records
                    if record["method"] == method and record["scenario"] == scenario
                ],
                key=lambda record: record["half_width"],
            )
            for row, (field, ylabel) in enumerate(METRICS):
                axis = axes[row, column]
                # Break the full-run curve at failed points; never interpolate across them.
                axis.plot(
                    [record["half_width"] for record in selected],
                    [
                        record[field]
                        if complete(record) and finite(record.get(field))
                        else np.nan
                        for record in selected
                    ],
                    color=color,
                    linestyle=linestyle,
                    linewidth=2,
                    zorder=2,
                )
                for index, record in enumerate(selected):
                    width, value = record["half_width"], record.get(field)
                    if not finite(value):
                        axis.annotate(
                            "Missing scalar",
                            (width, 0.03 + 0.045 * index),
                            xycoords=("data", "axes fraction"),
                            fontsize=7,
                            rotation=90,
                            color=color,
                        )
                        continue
                    done = complete(record)
                    axis.scatter(
                        width,
                        value,
                        color=color,
                        marker="o" if done else "x",
                        s=33 if done else 70,
                        linewidths=1.8,
                        zorder=4,
                    )
                    if not done:
                        method_index = list(STYLES).index(method)
                        axis.annotate(
                            f"{record['completed_steps']}/{record['steps_requested']}",
                            (width, value),
                            xytext=(5, 6 + 5 * method_index),
                            textcoords="offset points",
                            color=color,
                            fontsize=7,
                            bbox={
                                "facecolor": "white",
                                "edgecolor": "none",
                                "alpha": 0.7,
                            },
                        )
                axis.set_ylabel(ylabel)
                axis.margins(y=0.18)
                axis.grid(alpha=0.18)
                if row > 0:
                    axis.axhline(
                        0, color="#777777", linewidth=0.8, linestyle=":", zorder=1
                    )
        axes[2, column].set_xlabel(
            "Road half-width [m]; lateral reference = half-width - 2 m"
        )
        axes[2, column].set_xticks(widths)
        axes[2, column].set_xlim(min(widths) - 0.1, max(widths) + 0.15)
    handles = [
        Line2D([], [], color=color, linestyle=linestyle, linewidth=2, label=label)
        for label, color, linestyle in STYLES.values()
    ]
    handles += [
        Line2D(
            [],
            [],
            color="#333333",
            marker="o",
            linestyle="none",
            label="Completed 120 steps",
        ),
        Line2D(
            [],
            [],
            color="#333333",
            marker="x",
            linestyle="none",
            label="Failed/incomplete (steps labeled)",
        ),
    ]
    figure.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.035),
        ncol=3,
        frameon=False,
        fontsize=9,
    )
    figure.text(
        0.5,
        0.015,
        "Completion is not obstacle passing. Incomplete clearances/progress cover shorter runs.",
        ha="center",
        fontsize=9,
    )
    figure.tight_layout(rect=(0, 0.11, 1, 1))
    figure.savefig(output / "width_sweep.png", dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results/width_sweep")
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/width_sweep/plots"
    )
    parser.add_argument("--widths", type=float, nargs="+", default=list(WIDTHS))
    parser.add_argument(
        "--wait",
        action="store_true",
        help="Wait until every expected summary is present and valid",
    )
    parser.add_argument("--poll-seconds", type=float, default=5)
    args = parser.parse_args()
    if (
        not all(
            finite(width) and width > 0 and round(width, 1) == width
            for width in args.widths
        )
        or len(set(args.widths)) != len(args.widths)
        or not finite(args.poll_seconds)
        or args.poll_seconds <= 0
    ):
        parser.error(
            "Require unique positive one-decimal widths and a positive finite poll interval"
        )
    widths = sorted(args.widths)
    previous_error = None
    while True:
        try:
            records = read_summaries(args.input, widths)
            break
        except (OSError, ValueError, KeyError, TypeError) as error:
            if not args.wait:
                parser.error(str(error))
            if str(error) != previous_error:
                print(
                    f"Waiting for complete width summaries: {error}",
                    file=sys.stderr,
                    flush=True,
                )
                previous_error = str(error)
            sleep(args.poll_seconds)
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    export_csv(records, args.output)
    draw(records, widths, args.output)
    print(
        f"Plotted {len(records)} stored cases across {len(widths)} widths; "
        f"{sum(not complete(record) for record in records)} failed/incomplete; output: {args.output}"
    )


if __name__ == "__main__":
    main()
