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
