"""The kappa-as-state ablation must not change the physics.

With the curvature as a state the control is the curvature rate; with the
steering angle as a state the control is the steering rate and the curvature
follows from kappa = delta / l_wb. Both variants must describe the same
vehicle, so on a straight road at zero curvature they have to agree.
"""

import numpy as np
import pytest

from paper_comparison.model import PaperModel, Settings
from reiter2023.geometry import CircularRoad


def models():
    settings = Settings()
    road = CircularRoad(0.0, 8.5)
    return (
        PaperModel(road, settings, kappa_as_state=True),
        PaperModel(road, settings, kappa_as_state=False),
    )


def test_state_count_is_unchanged():
    with_kappa, without_kappa = models()
    assert with_kappa.nx == without_kappa.nx == 6


def test_initial_states_agree():
    with_kappa, without_kappa = models()
    a = with_kappa.initial(speed=12.0)
    b = without_kappa.initial(speed=12.0)
    assert a[5] == pytest.approx(b[5] / with_kappa.vehicle.length)
    assert a[:5] == pytest.approx(b[:5])


def test_dynamics_agree_on_a_straight_road():
    """Same physical input must give the same physical next state.

    The sixth state holds the curvature in one variant and the steering angle
    in the other, so the comparison is made on the curvature, not on the raw
    state entry.
    """
    with_kappa, without_kappa = models()
    state = np.array([0.0, 0.5, 0.0, 12.0, 0.0, 0.0])
    curvature_rate = 0.02
    u_kappa = [0.0, curvature_rate]
    u_steering = [0.0, curvature_rate * with_kappa.vehicle.length]
    step_a = np.asarray(with_kappa.step(state, u_kappa)).ravel()
    step_b = np.asarray(without_kappa.step(state, u_steering)).ravel()
    assert step_a[:5] == pytest.approx(step_b[:5], abs=1e-12)
    assert step_a[5] == pytest.approx(
        step_b[5] / without_kappa.vehicle.length, abs=1e-12
    )


def test_curvature_derivatives_agree():
    """The lateral acceleration residual uses the curvature rate in both."""
    import casadi as ca

    with_kappa, without_kappa = models()
    state = np.array([0.0, 0.5, 0.0, 12.0, 0.0, 0.0])
    curvature_rate = 0.02
    parameters = np.zeros(6)
    residual_a = np.asarray(
        ca.Function(
            "res_a",
            [with_kappa.x, with_kappa.u, with_kappa.p],
            [with_kappa.residual],
        )(state, [0.0, curvature_rate], parameters)
    ).ravel()
    residual_b = np.asarray(
        ca.Function(
            "res_b",
            [without_kappa.x, without_kappa.u, without_kappa.p],
            [without_kappa.residual],
        )(
            state,
            [0.0, curvature_rate * with_kappa.vehicle.length],
            parameters,
        )
    ).ravel()
    assert residual_a == pytest.approx(residual_b, abs=1e-12)


def test_steering_angle_bounds_are_respected():
    with_kappa, without_kappa = models()
    angle = with_kappa.vehicle.maximum_steering_angle
    assert without_kappa.xl[3] == pytest.approx(-angle)
    assert without_kappa.xu[3] == pytest.approx(angle)
    # With the curvature as a state the bound is the matching curvature.
    assert with_kappa.xu[3] == pytest.approx(
        np.tan(angle) / with_kappa.vehicle.length
    )
