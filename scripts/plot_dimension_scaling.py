"""Render saved all-call horizon scaling timings, including failed calls."""

import argparse
import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
HORIZONS = (20, 40, 80)
STYLES = {
    6: ("Own 6 / hybrid SQP", "#c73036", "-"),
    9: ("Own 9 / hybrid SQP", "#ed8b23", "--"),
}


def load(directory):
    provenance = json.loads((directory / "provenance.json").read_text())
    if not provenance.get("complete"):
        raise ValueError("Scaling run is not marked complete")
    summary = json.loads((directory / "summary.json").read_text())
    calls = [
        json.loads(line)
        for line in (directory / "measurements.jsonl").read_text().splitlines()
    ]
    if len(calls) != provenance["solve_calls"]:
        raise ValueError("Measurement count differs from provenance")
    lookup = {}
    for row in summary:
        key = (row["group"], row["scenario"], row["N"], row["nx"], row["phase"])
        if key in lookup:
            raise ValueError(f"Duplicate summary: {key}")
        selected = [
            call
            for call in calls
            if (call["group"], call["scenario"], call["N"], call["nx"], call["phase"])
            == key
        ]
        if (
            len(selected) != row["calls"]
            or sum(call["accepted"] for call in selected) != row["accepted_calls"]
        ):
            raise ValueError(f"Acceptance count differs from measurements: {key}")
        values = [call["native_solver_ms"] for call in selected]
        if not np.isfinite(values).all() or not np.isclose(
            np.mean(values), row["native_ms_all_mean"], atol=1e-10, rtol=1e-10
        ):
            raise ValueError(f"All-call mean differs from measurements: {key}")
        lookup[key] = row
    groups = sorted(
        {row["group"] for row in summary},
        key=lambda group: (
            -next(row["half_width"] for row in summary if row["group"] == group)
        ),
    )
    if len(groups) != 2:
        raise ValueError("Expected wide and narrow dataset groups")
    expected = {
        (group, scenario, n, nx, phase)
        for group in groups
        for scenario in ("straight", "curve")
        for n in HORIZONS
        for nx in STYLES
        for phase in ("warm", "cold")
    }
    if set(lookup) != expected:
        raise ValueError(
            "Expected full two-width, two-road, N20/40/80, six/nine-state matrix"
        )
    return provenance, lookup, calls, groups


def failure_note(calls, group, scenario):
    notes = []
    for phase in ("warm", "cold"):
        entries = []
        for n in HORIZONS:
            for nx in STYLES:
                selected = [
                    call
                    for call in calls
                    if call["group"] == group
                    and call["scenario"] == scenario
                    and call["phase"] == phase
                    and call["N"] == n
                    and call["nx"] == nx
                ]
                failed = [call for call in selected if not call["accepted"]]
                if failed:
                    statuses = ", ".join(
                        f"status {status}: {count}"
                        for status, count in sorted(
                            Counter(call["status"] for call in failed).items()
                        )
                    )
                    entries.append(
                        f"N{n} / own{nx}: {len(failed)}/{len(selected)} ({statuses})"
                    )
        notes.append(
            f"{phase.capitalize()} failures: "
            + ("; ".join(entries) if entries else "none")
        )
    return "\n".join(notes)


def draw(directory, output):
    provenance, lookup, calls, groups = load(directory)
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), squeeze=False)
    for row, group in enumerate(groups):
        width = next(
            value["half_width"] for key, value in lookup.items() if key[0] == group
        )
        label = "Wide" if row == 0 else "Narrow"
        for column, scenario in enumerate(("straight", "curve")):
            axis = axes[row, column]
            axis.set_title(
                f"{label} / {scenario} — half-width {width:g} m", fontsize=11
            )
            for nx, (_, color, linestyle) in STYLES.items():
                selected = [lookup[(group, scenario, n, nx, "warm")] for n in HORIZONS]
                means = [value["native_ms_all_mean"] for value in selected]
                axis.plot(
                    HORIZONS,
                    means,
                    color=color,
                    linestyle=linestyle,
                    marker="o",
                    linewidth=2.3,
                    markersize=6,
                )
                for n, mean, value in zip(HORIZONS, means, selected):
                    failures = value["calls"] - value["accepted_calls"]
                    low_point = mean < max(means) * 0.1
                    axis.annotate(
                        f"{mean:.2f}",
                        (n, mean),
                        xytext=(20, 25)
                        if nx == 6 and low_point
                        else (0, -15 if nx == 6 else 8),
                        textcoords="offset points",
                        ha="left" if nx == 6 and low_point else "center",
                        color=color,
                        fontsize=9,
                        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8},
                    )
                    if failures:
                        axis.scatter(
                            n, mean, marker="x", s=95, color="#222222", zorder=5
                        )
            axis.text(
                0.02,
                0.96,
                failure_note(calls, group, scenario),
                transform=axis.transAxes,
                va="top",
                fontsize=8,
                linespacing=1.5,
                bbox={"facecolor": "white", "edgecolor": "#dddddd", "alpha": 0.95},
            )
            axis.set_xticks(HORIZONS)
            axis.set_xlim(15, 85)
            axis.set_ylim(
                0,
                max(
                    lookup[(group, scenario, n, nx, "warm")]["native_ms_all_mean"]
                    for n in HORIZONS
                    for nx in STYLES
                )
                * 1.65,
            )
            axis.set_xlabel("Horizon N (dt = 0.1 s)")
            axis.set_ylabel("Warm all-call native mean [ms]")
            axis.grid(alpha=0.18)
    accepted = sum(call["accepted"] for call in calls)
    figure.suptitle(
        "Matched physical-state replay: six versus nine states", fontsize=14
    )
    handles = [
        Line2D(
            [],
            [],
            label=label,
            color=color,
            linestyle=linestyle,
            marker="o",
            linewidth=2.3,
        )
        for label, color, linestyle in STYLES.values()
    ]
    handles.append(
        Line2D(
            [],
            [],
            color="#222222",
            marker="x",
            linestyle="none",
            label="Warm group includes failed calls",
        )
    )
    figure.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.075),
        ncol=3,
        frameon=False,
    )
    figure.text(
        0.5,
        0.045,
        f"{provenance['repeats']} repeats; initial state + {provenance['following_states']} replay states; "
        f"{accepted}/{len(calls)} accepted. Failed calls remain in warm means.",
        ha="center",
        fontsize=9,
    )
    figure.text(
        0.5,
        0.022,
        "Cold = reset sample 0 (not process startup); cold timings excluded from curves, failures listed. No memory measurement.",
        ha="center",
        fontsize=9,
    )
    figure.tight_layout(rect=(0, 0.13, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(f"Rendered saved scaling means and audited acceptance counts: {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, default=ROOT / "results/dimension_scaling"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/dimension_scaling/dimension_scaling.png",
    )
    args = parser.parse_args()
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    try:
        draw(args.input, args.output)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
