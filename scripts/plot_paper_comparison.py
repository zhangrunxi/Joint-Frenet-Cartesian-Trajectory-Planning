"""Plot stored paper-comparison cases and export scalars; never run a solver."""

import argparse
import csv
import json
import sys
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from paper_comparison.evaluation import pose_from_state

STYLES = {
    "paper6_hybrid_sqp": ("Paper 6 / hybrid", "#c73036", "-"),
    "paper9_hybrid_sqp": ("Paper 9 / hybrid", "#ed8b23", "--"),
    "paper6_circles_sqp": ("Paper 6 / circles", "#925b35", "-."),
    "paper6_hybrid_cilqr": ("Paper 6 / CiLQR", "#aa4d86", ":"),
    "reiter_direct_ellipse": ("Reiter direct / ellipse", "#2166ac", "-"),
    "reiter_lifted_ellipse": ("Reiter lifted / ellipse", "#48a1cf", "--"),
    "reiter_direct_circles": ("Reiter direct / circles", "#16805d", "-."),
    "reiter_lifted_circles": ("Reiter lifted / circles", "#72a443", ":"),
    "xing_joint_corridor": ("Xing 7 / corridor (left)", "#111111", "--"),
    "xing_joint_yield": ("Xing 7 / yield policy ablation", "#0b8e9b", ":"),
    "xing_inspired_proxy_circles": (
        "Xing-inspired proxy (unverified) / circles",
        "#6b4c9a",
        "-.",
    ),
    "xing_inspired_proxy_ellipse": (
        "Xing-inspired proxy (unverified) / ellipse",
        "#b78a00",
        "--",
    ),
}
SCENARIOS = ("straight", "curve")


def style(method):
    return STYLES.get(method, (method.replace("_", " "), "#555555", "-"))


def group_label(group):
    name = group.removeprefix("paper_comparison_").replace("_", " ")
    return textwrap.fill(name, width=18)


def finite(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and np.isfinite(value)
    )


def read_cases(directories):
    """Read immediate case files only, tolerating a runner's partial JSON write."""
    cases, seen = [], set()
    for directory in directories:
        directory = directory.expanduser().resolve()
        if not directory.is_dir():
            raise ValueError(f"Input directory does not exist: {directory}")
        for path in sorted(directory.glob("*.json")):
            if any(
                word in path.stem.lower()
                for word in ("summary", "provenance", "rejected")
            ):
                continue
            if path in seen:
                continue
            seen.add(path)
            try:
                record = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError) as error:
                print(f"Skipping {path}: {error}", file=sys.stderr)
                continue
            if not isinstance(record, dict) or not isinstance(
                record.get("method"), str
            ):
                continue
            if record.get("scenario") not in SCENARIOS:
                continue
            cases.append(
                {"group": directory.name, "source_file": str(path), "record": record}
            )
    if not cases:
        raise ValueError(
            "No straight/curve case records found in the input directories"
        )
    parents = {}
    for case in cases:
        parents.setdefault(case["group"], set()).add(
            str(Path(case["source_file"]).parent)
        )
    # Keep distinct datasets distinct even when folder basenames happen to match.
    for case in cases:
        if len(parents[case["group"]]) > 1:
            case["group"] = str(Path(case["source_file"]).parent)
    return cases


def export_csv(cases, output):
    rows = []
    for case in cases:
        row = {"group": case["group"], "source_file": case["source_file"]}
        if "source_group" in case:
            row["source_group"] = case["source_group"]
        for key, value in case["record"].items():
            if value is None or isinstance(value, (str, bool, int, float)):
                row[key] = value
        # Flatten scalar metadata without rounding, converting units, or recomputing statistics.
        for section in ("settings", "startup"):
            for key, value in (case["record"].get(section) or {}).items():
                if value is None or isinstance(value, (str, bool, int, float)):
                    row[f"{section}.{key}"] = value
        rows.append(row)
    leading = ["group", "source_file", "name", "method", "scenario"]
    columns = leading + sorted(
        set().union(*(row.keys() for row in rows)) - set(leading)
    )
    with (output / "comparison.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def legend_handles(methods):
    return [
        Line2D(
            [],
            [],
            label=style(method)[0],
            color=style(method)[1],
            linestyle=style(method)[2],
            linewidth=2.3,
        )
        for method in methods
    ]


def states_for(record):
    try:
        states = np.asarray(record.get("states", []), dtype=float)
    except (TypeError, ValueError):
        return np.empty((0, 3))
    if states.ndim != 2 or states.shape[1] < 3:
        return np.empty((0, 3))
    return states[:, :3]


def coordinates(states, kappa, cartesian):
    if cartesian:
        return np.array([pose_from_state(state, kappa)[:2] for state in states])
    return states[:, :2]


def merge_by_width(cases):
    """Merge plotting groups only; leave original record values untouched."""
    widths = {case["record"].get("half_width") for case in cases}
    if not all(finite(width) and width > 0 for width in widths):
        raise ValueError(
            "Width merging requires a finite positive half_width in every case"
        )
    labels = {width: f"Half-width {width:g} m" for width in widths}
    if len(widths) == 2:
        labels[min(widths)], labels[max(widths)] = "Narrow", "Wide"
    seen = set()
    merged = []
    for case in cases:
        record = case["record"]
        group = labels[record["half_width"]]
        key = (group, record["scenario"], record["method"])
        if key in seen:
            raise ValueError(f"Duplicate method/scenario after width merging: {key}")
        seen.add(key)
        merged.append({**case, "source_group": case["group"], "group": group})
    return merged


def native_methods(cases, methods):
    return [
        method
        for method in methods
        if "cilqr" not in method.lower()
        and any(
            case["record"]["method"] == method
            and finite(case["record"].get("mean_native_solver_ms"))
            for case in cases
        )
    ]


def draw_trajectories(cases, groups, methods, output, cartesian=False):
    figure, axes = plt.subplots(
        len(groups), 2, figsize=(13, 3.5 * len(groups)), squeeze=False
    )
    for row, group in enumerate(groups):
        for column, scenario in enumerate(SCENARIOS):
            axis = axes[row, column]
            selected = [
                case["record"]
                for case in cases
                if case["group"] == group and case["record"]["scenario"] == scenario
            ]
            axis.set_title(f"{group} / {scenario}", fontsize=10)
            axis.set_xlabel("x [m]" if cartesian else "Rear-axle s [m]")
            axis.set_ylabel("y [m]" if cartesian else "Rear-axle n [m]")
            if not selected:
                axis.text(
                    0.5, 0.5, "No stored cases", transform=axis.transAxes, ha="center"
                )
                continue
            road_lines, obstacle_lines = set(), set()
            maximum_s, maximum_time = 40.0, 0.0
            for record in selected:
                states = states_for(record)
                if len(states):
                    finite_s = states[np.isfinite(states[:, 0]), 0]
                    if len(finite_s):
                        maximum_s = max(maximum_s, float(finite_s.max()))
                dt = record.get("settings", {}).get("dt", 0.1)
                duration = max(0, len(states) - 1) * dt
                maximum_time = max(maximum_time, duration)
            maximum_s = max(maximum_s, 35 + 8 * maximum_time) + 5
            s_grid = np.linspace(0, maximum_s, 450)
            for record in selected:
                kappa = record.get("curvature", 0 if scenario == "straight" else 1 / 60)
                settings = record.get("settings", {})
                width, reference = record.get("half_width"), settings.get("lateral")
                for offset, color, linestyle in [
                    (-width if finite(width) else None, "#b1b1ad", "-"),
                    (width, "#b1b1ad", "-"),
                    (reference, "#777777", "--"),
                ]:
                    key = (kappa, offset, linestyle)
                    if not finite(offset) or key in road_lines:
                        continue
                    road_lines.add(key)
                    points = coordinates(
                        np.c_[
                            s_grid, np.full_like(s_grid, offset), np.zeros_like(s_grid)
                        ],
                        kappa,
                        cartesian,
                    )
                    axis.plot(
                        *points.T,
                        color=color,
                        linestyle=linestyle,
                        linewidth=0.9,
                        zorder=1,
                    )
                if kappa not in obstacle_lines:
                    obstacle_lines.add(kappa)
                    obstacle_s = np.linspace(35, 35 + 8 * maximum_time, 100)
                    points = coordinates(
                        np.c_[obstacle_s, np.zeros((100, 2))], kappa, cartesian
                    )
                    axis.plot(
                        *points.T, color="#353535", linestyle=":", linewidth=2, zorder=2
                    )
                    axis.scatter(
                        *points[[0, -1]].T, color="#353535", marker="s", s=20, zorder=3
                    )
                states = states_for(record)
                if not len(states):
                    continue
                points = coordinates(states, kappa, cartesian)
                _, color, linestyle = style(record["method"])
                emphasized = record["method"] in (
                    "paper6_hybrid_sqp",
                    "paper9_hybrid_sqp",
                )
                axis.plot(
                    *points.T,
                    color=color,
                    linestyle=linestyle,
                    linewidth=2.5 if emphasized else 1.6,
                    zorder=5 if emphasized else 4,
                )
                axis.scatter(
                    *points[-1],
                    color=color,
                    marker="o" if record.get("success") else "x",
                    s=28 if record.get("success") else 65,
                    zorder=8,
                )
            if cartesian:
                axis.set_aspect("equal", adjustable="datalim")
            else:
                axis.set_xlim(-2, maximum_s)
            axis.grid(alpha=0.18)
    handles = legend_handles(methods) + [
        Line2D(
            [],
            [],
            color="#353535",
            linestyle=":",
            marker="s",
            label="Obstacle: s=35+8t, n=0",
        ),
        Line2D([], [], color="#777777", linestyle="--", label="Lateral reference"),
        Line2D([], [], color="#b1b1ad", label="Road limits"),
        Line2D(
            [],
            [],
            color="#333333",
            marker="x",
            linestyle="none",
            label="Aborted case endpoint",
        ),
    ]
    figure.legend(
        handles=handles, loc="lower center", ncol=3, fontsize=8, frameon=False
    )
    figure.tight_layout(rect=(0, 0.27 / len(groups), 1, 1))
    figure.savefig(
        output / ("trajectories_cartesian.png" if cartesian else "trajectories.png"),
        dpi=180,
    )
    plt.close(figure)


def grouped_bars(axis, cases, groups, methods, field, ylabel, native_only=False):
    selected_methods = native_methods(cases, methods) if native_only else methods
    categories = [(group, scenario) for group in groups for scenario in SCENARIOS]
    positions = np.arange(len(categories))
    width = 0.8 / max(1, len(selected_methods))
    for index, method in enumerate(selected_methods):
        values = []
        for group, scenario in categories:
            records = [
                case["record"]
                for case in cases
                if case["group"] == group
                and case["record"]["scenario"] == scenario
                and case["record"]["method"] == method
            ]
            value = records[0].get(field) if records else None
            values.append(value if finite(value) else np.nan)
        axis.bar(
            positions + (index - (len(selected_methods) - 1) / 2) * width,
            values,
            width=width * 0.92,
            color=style(method)[1],
        )
    axis.set_xticks(
        positions,
        [f"{group_label(group)}\n{scenario}" for group, scenario in categories],
        fontsize=8,
    )
    axis.set_ylabel(ylabel)
    axis.set_ylim(bottom=0)
    axis.grid(axis="y", alpha=0.18)
    if not any(
        finite(case["record"].get(field))
        for case in cases
        if not (native_only and "cilqr" in case["record"]["method"].lower())
    ):
        axis.text(0.5, 0.5, "No stored values", transform=axis.transAxes, ha="center")


def draw_performance(cases, groups, methods, output):
    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    grouped_bars(
        axes[0, 0],
        cases,
        groups,
        methods,
        "mean_native_solver_ms",
        "Mean native SQP time [ms]",
        native_only=True,
    )
    axes[0, 0].set_title(
        "Stored warm-solve mean\nCompiled SQP only; Xing QP and CiLQR omitted", fontsize=10
    )
    grouped_bars(
        axes[0, 1],
        cases,
        groups,
        methods,
        "decision_variables",
        "Nominal uncondensed state/input count",
    )
    axes[0, 1].set_title("nx(N+1)+nuN; not solver-specific optimization dimensions")
    markers = ["o", "s", "^", "D", "v", "P", "X", "<", ">"]
    dataset_handles = []
    for group_index, group in enumerate(groups):
        marker = markers[group_index % len(markers)]
        dataset_handles.append(
            Line2D(
                [], [], color="#555555", marker=marker, linestyle="none", label=group
            )
        )
        for case in cases:
            if case["group"] != group:
                continue
            record = case["record"]
            for axis, field in [
                (axes[1, 0], "min_obstacle_clearance_m"),
                (axes[1, 1], "min_road_clearance_m"),
            ]:
                progress, clearance = record.get("progress_m"), record.get(field)
                if not finite(progress) or not finite(clearance):
                    continue
                color = style(record["method"])[1]
                axis.scatter(
                    progress,
                    clearance,
                    marker=marker,
                    s=65,
                    facecolors=color if record["scenario"] == "straight" else "none",
                    edgecolors=color,
                    linewidths=1.7,
                )
                if record.get("success") is False:
                    axis.scatter(progress, clearance, marker="x", color="#222222", s=22)
    for axis, title in zip(
        axes[1], ["Minimum obstacle clearance [m]", "Minimum road clearance [m]"]
    ):
        axis.axhline(0, color="#777777", linewidth=0.8, linestyle="--")
        axis.set_xlabel("Stored progress s [m]")
        axis.set_ylabel(title)
        axis.grid(alpha=0.18)
    axes[1, 0].legend(handles=dataset_handles, fontsize=8, frameon=False)
    axes[1, 1].set_title(
        "Filled: straight; open: curve; x: incomplete case", fontsize=10
    )
    figure.legend(
        handles=legend_handles(methods),
        loc="lower center",
        ncol=3,
        fontsize=8,
        frameon=False,
    )
    figure.tight_layout(rect=(0, 0.12, 1, 1))
    figure.savefig(output / "performance.png", dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--merge-by-width",
        action="store_true",
        help="Merge equal half_width datasets; label two widths Wide/Narrow",
    )
    parser.add_argument(
        "--cartesian", action="store_true", help="Also save a Cartesian trajectory grid"
    )
    args = parser.parse_args()
    try:
        cases = read_cases(args.inputs)
        if args.merge_by_width:
            cases = merge_by_width(cases)
    except ValueError as error:
        parser.error(str(error))
    groups = list(dict.fromkeys(case["group"] for case in cases))
    present = {case["record"]["method"] for case in cases}
    methods = [method for method in STYLES if method in present] + sorted(
        present - STYLES.keys()
    )
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    export_csv(cases, args.output)
    draw_trajectories(cases, groups, methods, args.output)
    draw_performance(cases, groups, methods, args.output)
    if args.cartesian:
        draw_trajectories(cases, groups, methods, args.output, cartesian=True)
    print(
        f"Plotted {len(cases)} stored cases from {len(groups)} datasets into {args.output}"
    )


if __name__ == "__main__":
    main()
