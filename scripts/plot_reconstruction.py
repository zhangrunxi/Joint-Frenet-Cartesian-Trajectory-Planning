"""Plot existing reconstruction logs; never launches or changes a solver."""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from reiter2023.geometry import CircularRoad


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    colors = {"direct": "#c15327", "lifted": "#007a79"}
    for column, scenario in enumerate(["straight", "curve"]):
        for formulation in ["direct", "lifted"]:
            path = args.directory / f"{formulation}_ELIPSE_{scenario}.json"
            data = json.loads(path.read_text())
            state = np.array(data["actual_frenet_states"])
            road = CircularRoad(data["scenario"]["curvature"])
            poses = np.array([road.numeric_pose(x) for x in state])
            times = np.arange(len(state)) * data["scenario"]["dt"]
            axes[0, column].plot(
                poses[:, 0],
                poses[:, 1],
                color=colors[formulation],
                linestyle="-" if formulation == "direct" else "--",
                label=formulation,
            )
            axes[1, column].plot(
                times,
                state[:, 1],
                color=colors[formulation],
                linestyle="-" if formulation == "direct" else "--",
                label=formulation,
            )
        end = state[-1, 0] + 10
        s_grid = np.linspace(0, end, 300)
        for n in [-road.half_width, 0, road.half_width]:
            points = np.array([road.numeric_pose([s, n, 0]) for s in s_grid])
            axes[0, column].plot(
                points[:, 0],
                points[:, 1],
                color="#aaa9a4",
                linewidth=0.8,
                linestyle=":" if n == 0 else "-",
            )
        opp_s = (
            data["scenario"]["obstacle_start_s"]
            + data["scenario"]["obstacle_s_speed"] * times
        )
        opp = np.array([road.numeric_pose([s, 0, 0]) for s in opp_s])
        axes[0, column].plot(
            opp[:, 0],
            opp[:, 1],
            color="#524c42",
            linestyle=":",
            label="opponent rear axle",
        )
        axes[0, column].set(
            title=scenario.title(), xlabel="Cartesian x (m)", ylabel="Cartesian y (m)"
        )
        axes[0, column].set_aspect("equal", adjustable="datalim")
        axes[1, column].set(xlabel="Time (s)", ylabel="Frenet lateral offset (m)")
        for row in range(2):
            axes[row, column].grid(alpha=0.2)
            axes[row, column].legend(fontsize=8)
    fig.suptitle(
        "Independent Reiter 2023 reconstruction | ellipse | "
        + data["options"]["solver"]
        + "\nLocal smoke test, not the original paper benchmark"
    )
    fig.tight_layout()
    fig.savefig(args.directory / "trajectories.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
