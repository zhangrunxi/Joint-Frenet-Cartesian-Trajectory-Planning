"""Independent checks for the explicitly non closed loop geometry experiment."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

PATH = Path(__file__).resolve().parents[1] / "scripts/run_variable_curvature_geometry.py"
spec = importlib.util.spec_from_file_location("variable_geometry", PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_heading_integrates_variable_curvature():
    road = m.Road("s_bend")
    for s in [0, 20, 30, 45, 60]:
        h = 1e-4
        assert (road.heading(s+h)-road.heading(s-h))/(2*h) == pytest.approx(road.curvature(s), abs=1e-9)
    assert road.curvature(0) < 0 < road.curvature(60)


def test_projection_round_trip():
    road = m.Road("enter")
    s = np.array([0., 20., 30., 60.])
    points = road.xy(s) + np.column_stack([-np.sin(road.heading(s)), np.cos(road.heading(s))]) * 1.8
    projected, n = road.project(points, s + 5)
    np.testing.assert_allclose(projected, s, atol=1e-8)
    np.testing.assert_allclose(n, 1.8, atol=1e-8)


def test_straight_rectangle_and_contour():
    road = m.Road("straight")
    vehicle = m.VEHICLES["bus"]
    expected = [-1.15, 1.15]
    np.testing.assert_allclose(m.truth_extent(road, vehicle, 20, 0, 0), expected, atol=1e-9)
    np.testing.assert_allclose(m.rectangle_extent(road, vehicle, 20, 0, 0), expected, atol=1e-8)
    np.testing.assert_allclose(m.paper_extent(road, vehicle, 20, 0, 0), expected, atol=1e-9)


@pytest.mark.parametrize("profile", ["enter", "exit", "s_bend"])
def test_rectangle_matches_dense_independent_truth(profile):
    road = m.Road(profile)
    for vehicle in m.VEHICLES.values():
        for s, n, alpha in [(0, 0, 0), (30, 1.5, .08), (50, -1.5, -.08)]:
            exact = m.rectangle_extent(road, vehicle, s, n, alpha)
            dense = m.truth_extent(road, vehicle, s, n, alpha)
            np.testing.assert_allclose(exact, dense, atol=2e-6)
        checks = m.initial_checks(road, vehicle)
        assert checks["passed"]


def test_heading_difference_is_a_real_method_change():
    road = m.Road("s_bend")
    vehicle = m.VEHICLES["bus"]
    changed = m.paper_extent(road, vehicle, 30, 0, .08, segments=100)
    original = m.paper_extent(road, vehicle, 30, 0, .08, segments=100, original=True)
    assert np.max(np.abs(changed-original)) > .005


def test_dense_boundary_convergence():
    road = m.Road("s_bend")
    v = m.VEHICLES["bus"]
    coarse = m.truth_extent(road, v, 30, 1.5, .08, count=257)
    dense = m.truth_extent(road, v, 30, 1.5, .08, count=1025)
    np.testing.assert_allclose(coarse, dense, atol=3e-6)


def test_runner_records_static_results_and_refuses_overwrite(tmp_path, monkeypatch):
    original_arange = np.arange
    monkeypatch.setattr(m.np, "arange", lambda *args: np.array([0.]) if len(args) == 3 else original_arange(*args))
    monkeypatch.setattr(m, "VEHICLES", {"bus": m.VEHICLES["bus"]})
    output = tmp_path / "geometry"
    m.evaluate(output)
    import json
    result = json.loads((output / "summary.json").read_text())
    assert result["experiment_type"] == "static_geometry_not_closed_loop"
    assert len(result["summary"]) == 21
    assert all(row["count"] == 3 for row in result["summary"])
    with pytest.raises(FileExistsError):
        m.evaluate(output)


def test_projection_against_independent_nearest_distance():
    from scipy.optimize import minimize_scalar
    road = m.Road("enter")
    point = m.corners(road, m.VEHICLES["bus"], 60, 1.5, 0)[1]
    projected, n = road.project(point, 55)
    optimum = minimize_scalar(lambda s: np.sum((point-road.xy(s))**2), bounds=(50, 75), method="bounded", options={"xatol": 1e-11})
    assert projected == pytest.approx(optimum.x, abs=2e-6)
    assert abs(n) == pytest.approx(np.sqrt(optimum.fun), abs=1e-9)


def test_boundary_interior_extremum_error_below_uncertainty_budget():
    road = m.Road("enter")
    v = m.VEHICLES["bus"]
    coarse = m.truth_extent(road, v, 60, 1.5, 0, count=257)
    fine = m.truth_extent(road, v, 60, 1.5, 0, count=4097)
    assert np.max(np.abs(coarse-fine)) < m.TOL


def test_unknown_road_is_rejected():
    with pytest.raises(ValueError):
        m.Road("unknown")
