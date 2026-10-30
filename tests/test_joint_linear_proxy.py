"""Independent checks for the explicitly unverified Xing-inspired proxy."""

import numpy as np
import pytest

from paper_comparison.joint_linear_proxy import JointLinearProxy
from paper_comparison.model import Settings
from reiter2023 import CircularRoad


@pytest.fixture(params=[0.0, 1 / 60, -1 / 60])
def model(request):
    return JointLinearProxy(CircularRoad(request.param, 8.5), Settings(intervals=4))


def test_frame_connection_and_cartesian_rollout(model):
    x = model.augment(np.array([47.0, 1.2, 0.12, 13.0, 0.06]))
    u = np.array([0.8, 0.03])
    y = model.integrate(x, u, 0.1)
    np.testing.assert_allclose(np.asarray(model.frame(y)).ravel(), 0, atol=1e-12)
    np.testing.assert_allclose(
        np.asarray(model.dynamics(x, u, y)[0]).ravel(), 0, atol=1e-12
    )
    assert y[3] == pytest.approx(13.08)
    assert y[4] == pytest.approx(0.063)


def test_connection_jacobian(model):
    x = model.augment(np.array([12.0, -0.6, -0.1, 10.0, 0.05]))
    _, jac = model.connection(x)
    numerical = np.column_stack(
        [
            (
                np.asarray(model.frame(x + np.eye(8)[i] * 1e-5)).ravel()
                - np.asarray(model.frame(x - np.eye(8)[i] * 1e-5)).ravel()
            )
            / 2e-5
            for i in range(8)
        ]
    )
    np.testing.assert_allclose(np.asarray(jac), numerical, rtol=1e-7, atol=1e-8)


def test_cartesian_dynamics_linearization(model):
    x = model.augment(np.array([12.0, -0.6, -0.1, 10.0, 0.05]))
    u = np.array([0.7, 0.04])
    y = model.integrate(x, u, 0.1)
    _, A, B = model.dynamics(x, u, y)
    for i in range(8):
        d = np.eye(8)[i] * 1e-5
        num = (
            np.asarray(model.dynamics(x + d, u, y)[0]).ravel()
            - np.asarray(model.dynamics(x - d, u, y)[0]).ravel()
        ) / 2e-5
        np.testing.assert_allclose(np.asarray(A)[:, i], num, atol=1e-8)
    for i in range(2):
        d = np.eye(2)[i] * 1e-5
        num = (
            np.asarray(model.dynamics(x, u + d, y)[0]).ravel()
            - np.asarray(model.dynamics(x, u - d, y)[0]).ravel()
        ) / 2e-5
        np.testing.assert_allclose(np.asarray(B)[:, i], num, atol=1e-8)


def test_straight_tracking_qp_converges_and_reports_proxy():
    m = JointLinearProxy(
        CircularRoad(0, 8.5), Settings(intervals=4, lateral=0, speed=12)
    )
    x = m.initial()
    d = m.solve(x, m.parameters(0, 0))
    assert d["status"] == 0
    assert d["max_violation"] < 1e-6
    assert d["proxy_not_verified_xing"]
    assert d["native_solver_ms"] is None
    assert d["qp_kernel_sum_ms"] > 0


def test_iteration_limit_is_not_silently_accepted():
    m = JointLinearProxy(CircularRoad(0, 8.5), Settings(intervals=10), max_iter=1)
    d = m.solve(m.initial(), m.parameters(0, 0))
    assert d["status"] == 2
    assert d["iterations"] == 1
