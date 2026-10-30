import casadi as ca
import numpy as np
import pytest
from vehiclegym.planner.trajectory_planner_base import VehicleObstacleModel

from reiter2023 import CircularRoad, PlannerOptions, VehiclePlannerAcados20222
from reiter2023.geometry import covering_circles, ellipse_constraint
from reiter2023.trajectory_planner_acados_20222 import paper_vehicle


def symbolic_planner(lifted, road=None):
    planner = VehiclePlannerAcados20222.__new__(VehiclePlannerAcados20222)
    planner.road = road or CircularRoad(0.02)
    planner.ego = paper_vehicle()
    planner.opp_model_params = [paper_vehicle()]
    planner.planner_opts = PlannerOptions(use_lifting=lifted)
    planner.nx = 8 if lifted else 5
    planner._create_model()
    return planner


def test_lifted_rhs_equals_transformed_frenet_derivative():
    planner = symbolic_planner(False)
    jacobian = ca.Function(
        "J",
        [planner.x],
        [ca.jacobian(planner.transform_function(planner.x), planner.x)],
    )
    rng = np.random.default_rng(173)
    for _ in range(20):
        x = np.array(
            [
                rng.uniform(0, 100),
                rng.uniform(-4, 4),
                rng.uniform(-0.2, 0.2),
                rng.uniform(5, 20),
                rng.uniform(-0.1, 0.1),
            ]
        )
        rhs = np.asarray(planner.f_ode(x, [500, 0.02])).ravel()
        pose = planner.road.numeric_pose(x)
        expected = np.array(
            [
                x[3] * np.cos(pose[2]),
                x[3] * np.sin(pose[2]),
                x[3] * np.tan(x[4]) / planner.ego.length,
            ]
        )
        np.testing.assert_allclose(np.asarray(jacobian(x)) @ rhs, expected, atol=1e-12)


def test_lifted_initial_state_and_rk4_drift():
    direct, lifted = symbolic_planner(False), symbolic_planner(True)
    state = np.array([10, 1, 0.05, 12, 0.08])
    augmented = lifted.augment_state(state)
    np.testing.assert_allclose(augmented[5:], direct.road.numeric_pose(state))
    a = np.asarray(direct.integrate(state, [1000, 0.01], 0.1)).ravel()
    b = np.asarray(lifted.integrate(augmented, [1000, 0.01], 0.1)).ravel()
    np.testing.assert_allclose(a, b[:5], atol=1e-12)
    np.testing.assert_allclose(b[5:], direct.road.numeric_pose(a), atol=1e-6)


def test_ellipse_rotates_and_translates_with_cartesian_scene():
    vehicle = paper_vehicle()
    a, b = np.array([2, 1, 0.3]), np.array([12, 4, -0.5])
    reference = float(ellipse_constraint(a, b, vehicle, vehicle))
    angle = 0.7
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )

    def transform(pose):
        return np.r_[rotation @ pose[:2] + [20, -12], pose[2] + angle]

    assert float(
        ellipse_constraint(transform(a), transform(b), vehicle, vehicle)
    ) == pytest.approx(reference)
    assert float(ellipse_constraint(a, a, vehicle, vehicle)) < 0


@pytest.mark.parametrize("count", [1, 3, 5])
def test_circle_union_covers_full_chassis(count):
    vehicle = paper_vehicle()
    centers, radius = covering_circles(ca.DM([0, 0, 0]), vehicle, count)
    centers = np.array([np.asarray(c).ravel() for c in centers])
    xs = np.linspace(
        (vehicle.length - vehicle.chassis_length) / 2,
        (vehicle.length + vehicle.chassis_length) / 2,
        101,
    )
    for side in [-1, 1]:
        points = np.c_[xs, np.full_like(xs, side * vehicle.chassis_width / 2)]
        distances = np.linalg.norm(points[:, None, :] - centers[None, :, :], axis=2)
        assert np.max(np.min(distances, axis=1)) <= radius + 1e-10


def test_insufficient_circles_and_unsupported_model_are_not_silently_accepted():
    with pytest.raises(ValueError, match="do not cover"):
        covering_circles(ca.DM([0, 0, 0]), paper_vehicle(), 2)
    with pytest.raises(NotImplementedError):
        VehiclePlannerAcados20222(
            road=CircularRoad(),
            planner_options=PlannerOptions(
                obstacle_model=VehicleObstacleModel.HYPERPLANE
            ),
        )
