"""The scenario generator must only produce cases with a feasible initial state.

A case whose initial lateral acceleration exceeds the bound of Eq. (34f) is
infeasible in its first frame, and every optimizer reports infeasibility on it.
A case in which the ego vehicle cannot catch up with the obstacle cannot be
overtaken at all. Both properties would be measured by the scenario generator
rather than by the planners, so the generator rejects them.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_random_benchmark import (  # noqa: E402
    LATERAL_ACCELERATION_MARGIN,
    MAX_LATERAL_ACCELERATION,
    RANDOM_PROTOCOL,
    build_cases,
)

# The bus half width is 1.15 m and the road margin is 0.2 m.
BODY_HALF_WIDTH = 1.15
ROAD_MARGIN = 0.20


def _protocol(task):
    protocol = dict(RANDOM_PROTOCOL)
    protocol["task"] = task
    return protocol


@pytest.mark.parametrize("task", ["general", "overtake"])
def test_every_case_has_a_feasible_initial_lateral_acceleration(task):
    for case in build_cases(_protocol(task)):
        lateral_acceleration = case["initial_speed"] ** 2 * abs(case["kappa"])
        assert lateral_acceleration <= (
            MAX_LATERAL_ACCELERATION - LATERAL_ACCELERATION_MARGIN
        ), case


@pytest.mark.parametrize("task", ["general", "overtake"])
def test_every_case_starts_inside_the_road(task):
    for case in build_cases(_protocol(task)):
        extent = abs(case["lateral"]) + BODY_HALF_WIDTH
        assert extent <= case["half_width"] - ROAD_MARGIN, case


def test_the_overtaking_cases_can_catch_up_with_the_obstacle():
    for case in build_cases(_protocol("overtake")):
        assert case["initial_speed"] >= case["obstacle_speed"] + 1.5, case


def test_the_case_list_is_reproducible():
    assert build_cases(_protocol("general")) == build_cases(_protocol("general"))


def test_the_case_count_is_preserved():
    for task in ("general", "overtake"):
        assert len(build_cases(_protocol(task))) == RANDOM_PROTOCOL["cases"]


def test_the_overtaking_task_places_the_vehicle_beside_the_obstacle():
    """The ego vehicle is on the far side of the obstacle, not on top of it."""
    for case in build_cases(_protocol("overtake")):
        assert abs(case["lateral"]) > 2.0, case


def test_overtake2_main_applies_steps_before_generation(monkeypatch, tmp_path):
    import json
    import run_random_benchmark as benchmark

    def unexpected_run(*args, **kwargs):
        pytest.fail("No overtake can finish in one step")

    monkeypatch.setattr(benchmark, "run_method", unexpected_run)
    monkeypatch.setattr(sys, "argv", ["benchmark", "--task", "overtake2",
                        "--steps", "1", "--output", str(tmp_path / "short")])
    benchmark.main()
    assert json.loads((tmp_path / "short/cases.json").read_text()) == []
    assert json.loads((tmp_path / "short/protocol.json").read_text())["steps"] == 1


def test_suv_invalid_common_initial_states_remain_in_separate_summary_group():
    from paper_comparison.evaluation import initial_state_diagnostics
    from paper_comparison.vehicle import SUV
    from summarize_narrow_benchmark import curb_table

    case = dict(case_id=0, half_width=1.75, lateral=-.6, kappa=1/8,
                obstacle_start=30, obstacle_speed=0, obstacle_lateral=-30,
                initial_speed=4)
    diagnostics = initial_state_diagnostics(case, geometry=SUV)
    assert not diagnostics["initial_state_valid"]
    assert diagnostics["initial_road_clearance_m"] == pytest.approx(.026703233984886)
    assert "road_margin" in diagnostics["initial_state_invalid_reasons"]
    valid = {**case, "case_id": 1, "half_width": 2.5}
    valid.update(initial_state_diagnostics(valid, geometry=SUV))
    cases = {0: {**case, **diagnostics}, 1: valid}
    rows = [dict(case_id=i, method="baseline", success=False) for i in cases]
    table = curb_table(rows, cases)
    assert sum(r["cases"] for r in table) == 2
    assert {r["initial_state_valid"]: r["cases"] for r in table} == {True: 1, False: 1}
    assert sum(r["eligible_cases"] for r in table) == 1


def test_legacy_suv_results_are_classified_without_rewriting_cases(tmp_path):
    import json
    from summarize_narrow_benchmark import load

    case = dict(case_id=0, half_width=1.75, lateral=-.6, kappa=1/8,
                obstacle_start=30, obstacle_speed=0, obstacle_lateral=-30,
                initial_speed=4)
    original = json.dumps([case])
    (tmp_path / "cases.json").write_text(original)
    (tmp_path / "protocol.json").write_text(json.dumps({"vehicle": "suv"}))
    (tmp_path / "partial_results.jsonl").write_text(json.dumps(dict(case_id=0, method="baseline", success=False)))
    rows, cases = load(tmp_path)
    assert not cases[0]["initial_state_valid"]
    assert not rows[0]["initial_state_valid"]
    assert (tmp_path / "cases.json").read_text() == original


def test_invalid_case_is_recorded_not_dropped(monkeypatch, tmp_path):
    import json
    import run_random_benchmark as benchmark

    case = dict(case_id=7, half_width=1.0, lateral=0, kappa=0,
                obstacle_start=30, obstacle_speed=0, obstacle_lateral=0,
                initial_speed=4)
    calls = []
    monkeypatch.setattr(benchmark, "build_cases", lambda protocol: [case])
    def fake_run(output, method, recorded, *args):
        calls.append(recorded)
        return dict(method=method, success=False)
    monkeypatch.setattr(benchmark, "run_method", fake_run)
    monkeypatch.setattr(sys, "argv", ["benchmark", "--methods", "paper6_hybrid_sqp",
                        "--output", str(tmp_path / "invalid")])
    benchmark.main()
    recorded = json.loads((tmp_path / "invalid/cases.json").read_text())
    rows = json.loads((tmp_path / "invalid/results.json").read_text())
    assert len(calls) == len(recorded) == len(rows) == 1
    assert recorded[0]["case_id"] == rows[0]["case_id"] == 7
    assert rows[0]["initial_state_valid"] is False
    assert "road_margin" in rows[0]["initial_state_invalid_reasons"]


def test_general_summary_does_not_score_invalid_starts_as_planner_failures():
    from summarize_random_benchmark import summarize

    rows = [dict(method="paper6_hybrid_sqp", success=True, initial_state_valid=True),
            dict(method="paper6_hybrid_sqp", success=False, initial_state_valid=False)]
    summary = summarize(rows)[0]
    assert summary["cases"] == 2
    assert summary["infeasible_scenes"] == 1
    assert summary["feasible_cases"] == 1
    assert summary["completion_rate"] == 1
