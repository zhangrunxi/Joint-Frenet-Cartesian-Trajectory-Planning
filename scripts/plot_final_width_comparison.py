"""Overlay original three-configuration width sweep with source-verified Xing."""

import argparse
import json
from pathlib import Path

import plot_width_sweep as plotting


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paper-reiter", required=True, type=Path)
    parser.add_argument("--xing", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    records = plotting.read_summaries(args.paper_reiter, plotting.WIDTHS)
    for width in plotting.WIDTHS:
        path = args.xing / f"half_width_{width:.1f}" / "summary.json"
        extra = json.loads(path.read_text())
        assert len(extra) == 2
        assert {r["scenario"] for r in extra} == {"straight", "curve"}
        for row in extra:
            assert row["method"] == "xing_joint_corridor"
            assert row["half_width"] == width
            assert row["steps_requested"] == 120
            records.append({**row, "source_summary": str(path.resolve())})
    plotting.STYLES["xing_joint_corridor"] = (
        "Xing 7 / corridor (left)",
        "#111111",
        ":",
    )
    args.output.mkdir(parents=True, exist_ok=True)
    plotting.export_csv(records, args.output)
    plotting.draw(records, plotting.WIDTHS, args.output)
    print(f"Plotted {len(records)} width cases including incomplete runs.")


if __name__ == "__main__":
    main()
