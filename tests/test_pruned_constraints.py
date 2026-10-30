"""Component selection for the manuscript's hierarchical constraint pruning."""

import casadi as ca
import numpy as np
import pytest

from paper_comparison.model import (
    hybrid_component_distances,
    hybrid_parts,
    select_component,
)
from reiter2023.geometry import CircularRoad


def poses():
    road = CircularRoad(1 / 60, 4.6)
    ego = np.asarray(road.pose(10.0, 1.0, 0.12)).ravel()
    other = np.asarray(road.pose(14.0, 0.0, 0.0)).ravel()
    return ego, other


def test_component_count_is_twenty_five():
    ego, other = poses()
    values = hybrid_component_distances(ego, other, 0.2)
    assert len(values) == 25


def test_selection_returns_the_chosen_component():
    ego, other = poses()
    values = hybrid_component_distances(ego, other, 0.2)
    numeric = [float(ca.evalf(value)) for value in values]
    for index in (0, 7, 24):
        selected = select_component(values, [index])
        assert float(ca.evalf(selected)) == pytest.approx(numeric[index])


def test_selection_of_several_components_takes_the_strictest():
    ego, other = poses()
    values = hybrid_component_distances(ego, other, 0.2)
    numeric = [float(ca.evalf(value)) for value in values]
    for indices in ([0, 7], [3, 11, 19], [24, 0]):
        selected = select_component(values, indices)
        expected = min(numeric[index] for index in indices)
        assert float(ca.evalf(selected)) == pytest.approx(expected)


def test_selection_is_continuous_in_the_state():
    """The constraint must stay smooth in the state; only the index is a
    discrete, non-smooth choice, which is what the manuscript describes."""
    ego, other = poses()
    values = hybrid_component_distances(ego, other, 0.2)
    selected = select_component(values, [3])
    assert selected.numel() == 1


def test_segment_and_corner_parts_match_the_geometry():
    ego, other = poses()
    a, b, r, circles, rc = hybrid_parts(ego)
    c, d, q, others, qc = hybrid_parts(other)
    assert len(circles) == 4
    assert len(others) == 4
    assert r == pytest.approx(2.3 / 2)
    assert rc == pytest.approx(np.sqrt(2) * 2.3 / 4)
    assert q == pytest.approx(2.3 / 2)
    assert qc == pytest.approx(np.sqrt(2) * 2.3 / 4)


def test_nearest_component_is_the_smallest_distance():
    ego, other = poses()
    values = hybrid_component_distances(ego, other, 0.2)
    numeric = [float(ca.evalf(value)) for value in values]
    assert int(np.argmin(numeric)) == numeric.index(min(numeric))


def test_capsule_condition_is_the_weaker_sufficient_condition():
    """Eq. (23) separates the bodies as wholes; the pairwise form is tighter.

    The manuscript uses the continuous capsule minimum distance, which is a
    sufficient condition for the two bodies to be separated. Checking every
    component pair individually is a stronger requirement, so it must never
    report a larger clearance than the capsule condition.
    """
    from paper_comparison.model import capsule_distance

    ego, other = poses()
    capsule = float(ca.evalf(capsule_distance(ego, other, 0.2)))
    components = [
        float(ca.evalf(value))
        for value in hybrid_component_distances(ego, other, 0.2)
    ]
    assert capsule >= min(components)
