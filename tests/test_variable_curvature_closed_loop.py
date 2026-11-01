"""Non solver smoke checks: safe while a serial timing experiment is active."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import run_variable_curvature_closed_loop as m


def test_rk4_straight_constant_speed():
    road = m.Road("straight")
    initial = np.array([0., 0., 0., 5., 0., 0.])
    states = m.rollout(initial, np.zeros((8, 2)), road, .2)
    np.testing.assert_allclose(states[-1], [8., 0, 0, 5, 0, 0], atol=1e-12)


@pytest.mark.parametrize("method", m.METHODS)
def test_common_initial_state_physical_and_method_constraints(method):
    road, vehicle = m.Road("enter"), m.VEHICLES["bus"]
    initial = np.array([0., 0., 0., 5., 0., road.curvature(0)])
    controls = np.zeros((2, 2))
    states = m.rollout(initial, controls, road, .1)
    assert m.residuals(states, controls, method, road, vehicle, m.fixed_padding(vehicle)).min() > -1e-5


def test_unsafe_reference_point_is_rejected_by_independent_rectangle():
    road, vehicle = m.Road("exit"), m.VEHICLES["bus"]
    state = np.array([0., 2., 0., 5., 0., road.curvature(0)])
    point = m.envelope("rear_axle_point", road, vehicle, state, 0)
    body = m.envelope("xing_rectangle_generalized", road, vehicle, state, 0)
    assert np.max(abs(point)) < m.HALF_WIDTH-m.MARGIN
    assert np.max(abs(body)) > m.HALF_WIDTH-m.MARGIN


def test_symbolic_shared_road_and_model():
    import casadi as ca
    from variable_curvature_closed_loop import SharedRoad, VariableCurvaturePaperModel

    from paper_comparison.model import Settings
    road = SharedRoad("enter")
    for s in [0., 30., 60.]:
        np.testing.assert_allclose(road.numeric_pose([s, 0, 0])[:2], road.xy(s), atol=1e-8)
    for contour in ["original_local", "heading_difference", "reference_point"]:
        model = VariableCurvaturePaperModel(road, Settings(intervals=2), contour=contour)
        state = model.initial()
        assert state.shape == (6,)
        assert np.isfinite(np.asarray(model.ode(state, [0, 0]))).all()
        p = np.r_[1000., 1000., 0., 0., 0., 5.]
        assert np.asarray(model.hfun(state, [0, 0], p)).size > 8
    s = ca.MX.sym("s")
    heading = ca.Function("heading", [s], [road.symbolic_heading(s)])
    assert float(heading(30.)) == pytest.approx(road.heading(30.))
