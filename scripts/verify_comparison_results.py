"""Validate the stored experiment archive without launching any solver."""

import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATASETS = {
    "paper_comparison_wide_final": 16,
    "paper_comparison_narrow_final": 16,
    "xing_inspired_wide_v2": 4,
    "xing_inspired_narrow_v2": 4,
    "reiter_acceleration_cost_sensitivity/half_width_8.5": 8,
    "reiter_acceleration_cost_sensitivity/half_width_4.6": 8,
    **{
        f"width_sweep/half_width_{width}": 6
        for width in ("4.0", "4.3", "4.6", "4.9", "5.2", "5.5")
    },
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_case(case, summary):
    assert case["completed_steps"] == len(case["controls"])
    assert len(case["states"]) == case["completed_steps"] + 1
    assert len(case["cycles"]) in (case["completed_steps"], case["completed_steps"] + 1)
    for field, expected in summary.items():
        assert case[field] == expected, (case["name"], field)
    if case["success"]:
        assert case["completed_steps"] == case["steps_requested"]
        assert not case["collision_or_road_exit"]
    for cycle in case["cycles"]:
        accepted = cycle["status"] == 0 and cycle["max_violation"] <= 1e-4
        assert cycle["accepted"] == accepted
        assert cycle["solve_wall_ms"] >= 0
    for field in ("min_obstacle_clearance_m", "min_road_clearance_m", "progress_m"):
        assert math.isfinite(case[field])
    assert all(c["accepted"] for c in case["cycles"][: case["completed_steps"]])


def main():
    files, total_cases, completed_steps = set(), 0, 0
    for name, expected in DATASETS.items():
        directory = ROOT / "results" / name
        summaries = json.loads((directory / "summary.json").read_text())
        assert len(summaries) == expected
        files.add(directory / "summary.json")
        files.add(directory / "summary.csv")
        provenance_path = directory / "provenance.json"
        if name.startswith("reiter_acceleration_cost_sensitivity/"):
            provenance_path = directory.parent / "provenance.json"
        provenance = json.loads(provenance_path.read_text())
        files.add(provenance_path)
        for source, sha in provenance["sha256"].items():
            if source == "scripts/run_paper_comparison.py":
                if name.startswith("paper_comparison_"):
                    source = "sources/run_paper_comparison_pre_proxy.py"
                elif name.startswith("xing_inspired_"):
                    source = "sources/run_paper_comparison_with_proxy_v2.py"
            if source == "scripts/run_reiter_cost_sensitivity.py":
                source = "sources/run_reiter_cost_sensitivity_executed.py"
            path = ROOT / source
            assert digest(path) == sha, (name, source)
            files.add(path)
        for summary in summaries:
            path = directory / (summary["name"] + ".json")
            case = json.loads(path.read_text())
            validate_case(case, summary)
            if case["cycles"] and not case["cycles"][-1]["accepted"]:
                rejected = directory / (summary["name"] + "_rejected.json")
                assert rejected.exists()
                files.add(rejected)
            files.add(path)
            total_cases += 1
            completed_steps += case["completed_steps"]
    exact_directory = ROOT / "results/exact_road_sensitivity"
    exact_summaries = json.loads((exact_directory / "summary.json").read_text())
    assert len(exact_summaries) == 12
    exact_provenance = json.loads((exact_directory / "provenance.json").read_text())
    for source, sha in exact_provenance["sha256"].items():
        path = ROOT / source
        assert digest(path) == sha, ("exact_road_sensitivity", source)
        files.add(path)
    files.update(exact_directory.glob("*.json"))
    files.update(exact_directory.glob("*.csv"))
    for summary in exact_summaries:
        path = (
            exact_directory
            / f"half_width_{summary['half_width']}"
            / (summary["name"] + ".json")
        )
        case = json.loads(path.read_text())
        validate_case(case, summary)
        files.add(path)
        files.update(path.parent.glob("*_rejected.json"))
        total_cases += 1
        completed_steps += case["completed_steps"]
    scaling = ROOT / "results/dimension_scaling"
    records = [
        json.loads(line)
        for line in (scaling / "measurements.jsonl").read_text().splitlines()
    ]
    provenance = json.loads((scaling / "provenance.json").read_text())
    assert provenance["complete"] and len(records) == provenance["solve_calls"] == 1512
    files.update(path for path in scaling.iterdir() if path.is_file())
    files.update((ROOT / "paper_comparison").glob("*.py"))
    files.update((ROOT / "tests").glob("test_*comparison*.py"))
    files.update((ROOT / "tests").glob("test_paper*.py"))
    files.update((ROOT / "tests").glob("test_joint*.py"))
    files.add(ROOT / "ACCEPTANCE_REPORT.md")
    files.add(ROOT / "REVIEWER_RESPONSE_DRAFT.md")
    files.add(ROOT / "AUDIT_FINDINGS.md")
    files.add(ROOT / "tests/test_recorded_limitations.py")
    files.add(ROOT / "tests/test_exact_road_sensitivity.py")
    files.add(ROOT / "results/effective_method_configurations.json")
    files.update((ROOT / "results/geometry_conservatism").glob("*"))
    for directory in (
        "paper_comparison_plots",
        "three_family_comparison_plots",
        "width_sweep/plots",
        "exact_road_sensitivity/plots",
    ):
        files.update((ROOT / "results" / directory).glob("*.png"))
        files.update((ROOT / "results" / directory).glob("*.csv"))
    files.update((ROOT / "results/dimension_scaling").glob("*.png"))
    manifest = {
        "case_count": total_cases,
        "completed_closed_loop_cycles": completed_steps,
        "scaling_calls": len(records),
        "xing_verified_reproduction": False,
        "sha256": {str(p.relative_to(ROOT)): digest(p) for p in sorted(files)},
    }
    target = ROOT / "results/comparison_archive_manifest.json"
    target.write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        f"Verified {total_cases} cases, {completed_steps} executed cycles, {len(records)} scaling calls."
    )
    print(
        "This historical archive contains Xing-inspired proxies only; "
        "use verify_final_comparison.py for the new paper reconstruction. Manifest: "
        + str(target)
    )


if __name__ == "__main__":
    main()
