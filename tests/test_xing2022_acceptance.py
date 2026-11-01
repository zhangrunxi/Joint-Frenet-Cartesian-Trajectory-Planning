"""Regressions for independent audit's endpoint and nonfinite findings."""

from types import SimpleNamespace

import numpy as np
import pytest

from paper_comparison import xing2022
from paper_comparison.model import Settings
from paper_comparison.xing2022 import XingPlanner
from reiter2023 import CircularRoad


def seed(model, initial):
    model.X = np.empty((model.N + 1, 7))
    model.U = np.tile(model.previous_input, (model.N, 1))
    model.X[0] = initial
    for j in range(model.N):
        model.X[j + 1] = model.integrate(model.X[j], model.U[j], model.dt)


def test_audited_forecast_endpoint_excess_is_rejected():
    m = XingPlanner(CircularRoad(0, 8.5), Settings(intervals=1))
    x = m.initial()
    x[0] = 14.79826483982653
    u = np.array([0.09628269772854797, 1.6913241991319308])
    m.previous_input = u.copy()
    y = m.integrate(x, u, m.dt)
    assert float(m.lateral_acceleration(x[:4], u)[0]) < 5
    expected = (x[0] + m.dt * u[1]) ** 2 * np.tan(u[0]) / 4.32
    assert expected == pytest.approx(5.008434599053182)
    violation, _, _ = m.residuals(np.array([x, y]), u[None], x, 0)
    assert violation == pytest.approx(expected - 5)


@pytest.mark.parametrize("component", ["state_speed", "control", "initial", "previous"])
@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_nonfinite_arrays_fail_closed(component, invalid):
    m = XingPlanner(CircularRoad(0, 8.5), Settings(intervals=2))
    x = m.initial()
    seed(m, x)
    if component == "state_speed":
        m.X[1, 0] = invalid
    elif component == "control":
        m.U[0, 1] = invalid
    elif component == "initial":
        x[0] = invalid
    else:
        m.previous_input[1] = invalid
    assert all(np.isinf(v) for v in m.residuals(m.X, m.U, x, 0))


def test_solved_status_with_nan_solution_is_not_executed(monkeypatch):
    m = XingPlanner(CircularRoad(0, 8.5), Settings(intervals=2))
    seed(m, m.initial())
    old_x, old_u = m.X.copy(), m.U.copy()

    class InvalidQp:
        def setup(self, **kwargs):
            pass

        def warm_start(self, **kwargs):
            pass

        def solve(self, **kwargs):
            return SimpleNamespace(
                x=np.full(m.nz, np.nan),
                info=SimpleNamespace(status_val=1, status="solved", run_time=0.001),
            )

    monkeypatch.setattr(xing2022.osqp, "OSQP", InvalidQp)
    d = m.solve(m.initial(), 0)
    assert d["status"] == 4 and d["qp_status"] == "nonfinite_qp_solution"
    np.testing.assert_array_equal(m.X, old_x)
    np.testing.assert_array_equal(m.U, old_u)


def test_endpoint_acceleration_jacobian_includes_acceleration_input():
    m = XingPlanner(CircularRoad(0, 8.5), Settings(intervals=2))
    z, u = np.array([13, 0, 0, 0.1]), np.array([0.08, 1.2])
    _, J, K = m.lateral_acceleration_end(z, u)
    vend = z[0] + m.dt * u[1]
    expected_v = 2 * vend * np.tan(u[0]) / 4.32
    assert np.asarray(J)[0, 0] == pytest.approx(expected_v)
    assert np.asarray(K)[0, 1] == pytest.approx(m.dt * expected_v)
    assert np.asarray(K)[0, 0] == pytest.approx(vend**2 / (4.32 * np.cos(u[0]) ** 2))


def test_frozen_corridor_rejects_acceleration_into_obstacle():
    from paper_comparison.xing2022 import XingOptions
    from paper_comparison.evaluation import evaluate_sample

    m = XingPlanner(CircularRoad(0, 8.5), Settings(), XingOptions(
        initial_speed=5, obstacle_start=35, obstacle_speed=0, freeze_corridor=True))
    initial = m.initial()
    seed(m, initial)
    cached = m.corridor(m.X, 0)
    m._frozen_corridor = cached
    trajectory, controls = [initial], []
    for j in range(m.N):
        control = np.array([0, min(.4 * (j + 1), 2.5)])
        controls.append(control)
        trajectory.append(m.integrate(trajectory[-1], control, m.dt))
    trajectory, controls = np.array(trajectory), np.array(controls)
    assert min(evaluate_sample(m.canonical(x), [35, 0, 0], 0, 8.5)[
        "obstacle_clearance_m"] for x in trajectory) < -2
    violation, coordinate, defect = m.residuals(trajectory, controls, initial, 0)
    assert violation >= 2.5 - 1e-10
    assert max(coordinate, defect) < 1e-10
    assert m._frozen_corridor is cached
    assert m.corridor(trajectory, 0) is cached
    assert np.max(m.corridor(trajectory, 0, recompute=True)[:, 2]) == pytest.approx(2.5)


def test_recomputed_corridor_updates_body_envelope_without_changing_cache():
    m = XingPlanner(CircularRoad(0, 4.6), Settings(intervals=2))
    seed(m, m.initial())
    m._frozen_corridor = cached = m.corridor(m.X, 0)
    candidate = m.X.copy()
    candidate[:, 3] = candidate[:, 6] = .3
    fresh = m.corridor(candidate, 0, recompute=True)
    assert np.all(fresh[:, 3] < cached[:, 3] - .5)
    assert m.corridor(candidate, 0) is cached


def test_prediction_recording_recomputes_frozen_corridor(monkeypatch, tmp_path):
    import importlib.util
    import json
    from pathlib import Path

    script = Path(__file__).resolve().parents[1] / "scripts/run_final_comparison.py"
    spec = importlib.util.spec_from_file_location("final_comparison_recording", script)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class SeededPlanner(XingPlanner):
        def solve(self, initial, time):
            seed(self, initial)
            self._frozen_corridor = self.corridor(self.X, time).copy()
            self._frozen_corridor[:, 3] = 999
            return dict(status=0, max_violation=0, coordinate_error=0,
                        dynamics_defect=0)

    monkeypatch.setattr(runner, "XingPlanner", SeededPlanner)
    runner.run_xing(tmp_path, "xing_joint_corridor", "straight", 1,
                    Settings(intervals=2), 8.5, obstacle_start=500)
    prediction = json.loads((tmp_path / "xing_joint_corridor_straight_predictions.json").read_text())
    bounds = np.array([row[3] for row in prediction["accepted_predictions"][0]["final_recomputed_corridor"]], dtype=float)
    np.testing.assert_allclose(bounds, 8.5 - .2 - runner.GEOMETRY.half_width)
