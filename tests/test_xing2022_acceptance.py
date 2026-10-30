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
