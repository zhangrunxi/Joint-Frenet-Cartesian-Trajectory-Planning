"""Check that the new geometry ablation changes only collision geometry."""

import importlib.util
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location(
    "revision_supplement", ROOT / "scripts/run_revision_supplement.py"
)
supplement = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supplement)

from paper_comparison.model import PaperModel, Settings
from reiter2023 import CircularRoad


def test_five_circle_preserves_dynamics_cost_and_noncollision_constraints():
    road, settings = CircularRoad(1 / 60, 4.6), Settings(lateral=2.6)
    three = PaperModel(road, settings, obstacle="CIRCLES")
    five = supplement.FiveCircleModel(road, settings, obstacle="CIRCLES")
    state = np.array([8.0, 0.8, 0.08, 11.0, 0.4, 0.012])
    control = np.array([0.2, 0.003])
    parameter = three.parameters(0.3, state[0])[0]
    np.testing.assert_array_equal(three.step(state, control), five.step(state, control))
    np.testing.assert_array_equal(
        three.hfun(state, control, parameter)[:8],
        five.hfun(state, control, parameter)[:8],
    )
    assert three.h.numel() == 17 and five.h.numel() == 33
    assert str(three.cost) == str(five.cost)
    assert str(three.terminal_cost) == str(five.terminal_cost)
    np.testing.assert_array_equal(three.xl, five.xl)
    np.testing.assert_array_equal(three.xu, five.xu)
    np.testing.assert_array_equal(three.ul, five.ul)
    np.testing.assert_array_equal(three.uu, five.uu)


def test_frozen_matrix_includes_unseen_curvatures_and_all_methods():
    protocol = supplement.specification()
    assert len(protocol["scenarios"]) == 12
    assert {s["curvature"] for s in protocol["scenarios"]} == {0, 1 / 80, 1 / 40}
    assert len(protocol["methods"]) == 7
    assert protocol["retuning_after_inspecting_results"] is False


def test_timing_statistics_include_rejected_warm_calls():
    cycles = [
        {"planning_call_ms": 200, "accepted": True},
        {"planning_call_ms": 20, "accepted": True},
        {"planning_call_ms": 120, "accepted": False},
    ]
    summary = supplement.summarize_times(cycles)
    assert summary["warm_mean_planning_call_ms"] == 70
    assert summary["warm_deadline_miss_fraction"] == 0.5
    assert summary["warm_calls"] == 2
