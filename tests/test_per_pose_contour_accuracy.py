"""Analytic checks for the independent circular-road corner oracle."""

import importlib.util
from pathlib import Path

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_per_pose_contour_accuracy.py"
spec = importlib.util.spec_from_file_location("per_pose_accuracy", SCRIPT)
accuracy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(accuracy)


def test_straight_projection_is_exact_for_rotated_body():
    for alpha in (-0.2, 0.0, 0.2):
        exact = accuracy.exact_offsets(7.0, 0.8, alpha, accuracy.CircularRoad(0, 8.5))
        expected = [0.8 + x * np.sin(alpha) + y * np.cos(alpha)
                    for x, y in accuracy.CORNERS]
        np.testing.assert_allclose(exact, expected, atol=1e-12)


def test_circle_projection_matches_radius_and_is_station_invariant():
    for kappa in (-0.05, 0.05):
        road = accuracy.CircularRoad(kappa, 8.5)
        expected = [(1 - np.hypot(kappa*x, 1-kappa*y))/kappa
                    for x, y in accuracy.CORNERS]
        for station in (0.0, 7.0, 40.0):
            np.testing.assert_allclose(
                accuracy.exact_offsets(station, 0, 0, road), expected, atol=1e-12)


def test_predicted_offsets_match_both_vehicle_geometries():
    from paper_comparison.model import approximate_road_offsets
    road = accuracy.CircularRoad(0.05, 8.5)
    state = np.array([3.0, 0.8, 0.1, 5.0, 0.0, 0.05])
    # Production order is rear-right, rear-left, front-right, front-left.
    production = np.asarray(approximate_road_offsets(state, road, 10)).ravel()
    predicted = accuracy.predicted_offsets(3, .8, .1, road, 10)
    np.testing.assert_allclose(predicted, production[[0, 2, 3, 1]], atol=1e-12)


def test_corner_names_identify_physical_body_corners():
    expected = {
        "RR": (-accuracy.GEOMETRY.rear_overhang, -accuracy.GEOMETRY.half_width),
        "FR": (accuracy.GEOMETRY.front_extent, -accuracy.GEOMETRY.half_width),
        "FL": (accuracy.GEOMETRY.front_extent, accuracy.GEOMETRY.half_width),
        "RL": (-accuracy.GEOMETRY.rear_overhang, accuracy.GEOMETRY.half_width),
    }
    assert dict(zip(accuracy.NAMES, accuracy.CORNERS)) == expected
    road = accuracy.CircularRoad(0, 8.5)
    for name, value in zip(accuracy.NAMES, accuracy.predicted_offsets(0, .8, .2, road, 10)):
        x, y = expected[name]
        np.testing.assert_allclose(value, .8 + x * np.sin(.2) + y * np.cos(.2), atol=1e-12)
