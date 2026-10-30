"""Analytic geometry regressions; no planner or symbolic dependencies required."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

# Load directly so an evolving package __init__ cannot pull in solver modules.
MODULE = Path(__file__).resolve().parents[1] / "paper_comparison/evaluation.py"
SPEC = importlib.util.spec_from_file_location("comparison_evaluation", MODULE)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)
rectangle_vertices = evaluation.rectangle_vertices
polygon_clearance = evaluation.polygon_clearance
road_clearance = evaluation.road_clearance
pose_from_state = evaluation.pose_from_state
evaluate_sample = evaluation.evaluate_sample


def square(center=(0, 0), side=2):
    return rectangle_vertices([*center, 0], side, side, side / 2)


def rotate(vertices, angle):
    cosine, sine = np.cos(angle), np.sin(angle)
    return vertices @ np.array([[cosine, sine], [-sine, cosine]])


def test_default_rectangle_uses_rear_axle_and_correct_overhang():
    expected = [[-1, -1.15], [5.5, -1.15], [5.5, 1.15], [-1, 1.15]]
    np.testing.assert_allclose(rectangle_vertices([0, 0, 0]), expected)
    np.testing.assert_allclose(
        rectangle_vertices([7, -3, np.pi / 2]),
        rotate(np.asarray(expected), np.pi / 2) + [7, -3],
    )


def test_custom_rectangle_dimensions():
    np.testing.assert_allclose(
        rectangle_vertices([1, 2, 0], length=4, width=2, rear_overhang=0.5),
        [[0.5, 1], [4.5, 1], [4.5, 3], [0.5, 3]],
    )


@pytest.mark.parametrize("angle", [0, 0.37, -1.2])
@pytest.mark.parametrize(
    "offset, expected",
    [
        ((5, 5), np.sqrt(18)),
        ((3, 0.5), 1),
        ((1.5, 0), -0.5),
        ((0, 0), -2),
        ((2, 0), 0),
        ((2, 2), 0),
    ],
)
def test_polygon_clearance_is_exact_and_rigid_motion_invariant(angle, offset, expected):
    a = rotate(square(), angle) + [13, -7]
    b = rotate(square(offset), angle) + [13, -7]
    assert polygon_clearance(a, b) == pytest.approx(expected, abs=1e-12)
    assert polygon_clearance(b[::-1], a) == pytest.approx(expected, abs=1e-12)


def test_sat_penetration_handles_containment_not_interval_overlap():
    assert polygon_clearance(square(side=10), square(side=2)) == pytest.approx(-6)
    assert polygon_clearance(square(side=10), square((3, 0))) == pytest.approx(-3)


def test_differently_rotated_square_separation_and_overlap():
    diamond = rotate(square(), np.pi / 4)
    assert polygon_clearance(square(), diamond + [4, 0]) == pytest.approx(
        3 - np.sqrt(2)
    )
    assert polygon_clearance(square(), diamond + [2, 0]) == pytest.approx(
        1 - np.sqrt(2)
    )


@pytest.mark.parametrize("yaw", [0, 0.4, np.pi / 2])
@pytest.mark.parametrize("y", [0, 3])
def test_straight_road_clearance_includes_rotated_chassis(yaw, y):
    vertices = rectangle_vertices([0, y, yaw], length=4, width=2, rear_overhang=2)
    extent = 2 * abs(np.sin(yaw)) + abs(np.cos(yaw))
    assert road_clearance(vertices, 0, 3) == pytest.approx(3 - abs(y) - extent)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("reverse", [False, True])
def test_circle_inner_edge_violates_road_while_all_vertices_are_safe(sign, reverse):
    vertices = rectangle_vertices([-2, 1, 0], length=6, width=1, rear_overhang=1)
    vertices[:, 1] *= sign
    if reverse:
        vertices = vertices[::-1]
    radii = np.linalg.norm(vertices - [0, sign * 10], axis=1)
    assert np.all(np.abs(radii - 10) < 1)
    # The closest point is (0, +/-1.5), strictly inside the long edge.
    assert road_clearance(vertices, sign * 0.1, 1) == pytest.approx(-0.5)


@pytest.mark.parametrize("sign", [-1, 1])
def test_circle_outer_clearance_comes_from_farthest_vertex(sign):
    vertices = square((0, -1.5), side=2)
    vertices[:, 1] *= sign
    expected = 11 - np.hypot(1, 12.5)
    assert road_clearance(vertices, sign * 0.1, 1) == pytest.approx(expected)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("angle", [0, 0.6])
def test_circle_center_inside_polygon_has_zero_minimum_radius(sign, angle):
    vertices = rotate(square(), angle) + [0, sign * 10]
    assert road_clearance(vertices, sign * 0.1, 1) == pytest.approx(-9)


def test_circle_center_on_edge_and_inner_boundary_contact():
    assert road_clearance(square((1, 10)), 0.1, 1) == pytest.approx(-9)
    vertices = rectangle_vertices([-2, 0.5, 0], 6, 1, 1)
    assert road_clearance(vertices, 0.1, 1) == pytest.approx(0)


@pytest.mark.parametrize("kappa", [0, 0.1, -0.1, 1e-14, -1e-14])
def test_pose_from_state_straight_and_quarter_circle(kappa):
    if abs(kappa) < 1e-10:
        state, expected = [7, 2, 0.3], [7, 2, 0.3]
    else:
        sign = np.sign(kappa)
        state = [np.pi / (2 * abs(kappa)), 2, 0.3]
        expected = [10 - 2 * sign, 10 * sign, sign * np.pi / 2 + 0.3]
    np.testing.assert_allclose(pose_from_state(state, kappa), expected, atol=1e-12)


def test_pose_uses_only_frenet_components_not_lifted_cartesian_state():
    expected = pose_from_state([5, 1, 0.2], -0.03)
    state = np.array([5, 1, 0.2, 12, 0, 999, -999, np.nan])
    np.testing.assert_allclose(pose_from_state(state, -0.03), expected)


def test_evaluate_sample_uses_full_rectangles_and_returns_scalar_metres():
    result = evaluate_sample([0, 0, 0, 12, 0], [7, 0, 0], 0, 3)
    assert result == pytest.approx(
        {"obstacle_clearance_m": 0.5, "road_clearance_m": 1.85}
    )
    assert all(isinstance(value, float) for value in result.values())
    assert evaluate_sample([0, 0, 0], [6, 0, 0], 0, 1)[
        "obstacle_clearance_m"
    ] == pytest.approx(-0.5)


@pytest.mark.parametrize("sign", [-1, 1])
def test_evaluate_sample_transforms_frenet_pose_on_curved_road(sign):
    kappa = sign * 0.1
    state = [5 * np.pi, 0, 0]
    # Analytic quarter-circle rear-axle pose, with the other chassis coincident.
    other = [10, sign * 10, sign * np.pi / 2]
    result = evaluate_sample(state, other, kappa, 4)
    assert result["obstacle_clearance_m"] == pytest.approx(-2.3)
    assert result["road_clearance_m"] == pytest.approx(14 - np.hypot(11.15, 5.5))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"length": 0},
        {"width": -1},
        {"rear_overhang": -1},
        {"rear_overhang": 7},
        {"length": np.nan},
    ],
)
def test_invalid_rectangle_dimensions_are_rejected(kwargs):
    with pytest.raises(ValueError):
        rectangle_vertices([0, 0, 0], **kwargs)


@pytest.mark.parametrize(
    "vertices",
    [
        [],
        [[0, 0], [1, 1]],
        [[0, 0]] * 4,
        [[0, 0], [1, 0], [2, 0]],
        [[0, 0], [1, np.nan], [0, 1]],
    ],
)
def test_invalid_polygon_inputs_are_rejected(vertices):
    with pytest.raises(ValueError):
        polygon_clearance(vertices, square())
    with pytest.raises(ValueError):
        road_clearance(vertices, 0.1, 1)


@pytest.mark.parametrize("kappa, half_width", [(np.nan, 1), (0, -1), (0, np.inf)])
def test_invalid_road_parameters_are_rejected(kappa, half_width):
    with pytest.raises(ValueError):
        road_clearance(square(), kappa, half_width)


@pytest.mark.parametrize(
    "state, kappa", [([1, 2], 0), ([1, np.nan, 0], 0), ([1, 2, 0], np.inf)]
)
def test_invalid_pose_inputs_are_rejected(state, kappa):
    with pytest.raises(ValueError):
        pose_from_state(state, kappa)
