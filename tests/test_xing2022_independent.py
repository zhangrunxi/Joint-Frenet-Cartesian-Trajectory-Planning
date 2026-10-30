"""Independent numerical checks of Xing Eq. 6/7, 17/19 and corridor QPs.

Only tiny OSQP problems are solved; no acados code generation or native builds.
"""

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from paper_comparison import xing2022
from paper_comparison.model import Settings
from paper_comparison.xing2022 import XingOptions, XingPlanner
from reiter2023.geometry import CircularRoad

WHEELBASE = 4.32


def vector(value):
    return np.asarray(value, dtype=float).ravel()


def reference_pose(s, d, phi, kappa):
    """Independent Cartesian road chart with initial heading zero."""
    heading = kappa * s
    if kappa == 0:
        reference = np.array([s, 0.0])
    else:
        reference = np.array([np.sin(heading), 1 - np.cos(heading)]) / kappa
    return np.r_[
        reference + d * np.array([-np.sin(heading), np.cos(heading)]), heading + phi
    ]


def reference_project(pc, kappa, near_s):
    """Recover the foot point from circle radii and signed normal distance."""
    _, x, y, psi = pc
    if kappa == 0:
        return np.array([x, y, psi])
    center = np.array([0.0, 1 / kappa])
    radial, initial_radial = np.array([x, y]) - center, -center
    cross = initial_radial[0] * radial[1] - initial_radial[1] * radial[0]
    angle = np.arctan2(cross, initial_radial @ radial)
    angle += 2 * np.pi * np.round((kappa * near_s - angle) / (2 * np.pi))
    s = angle / kappa
    d = np.sign(kappa) * (abs(1 / kappa) - np.linalg.norm(radial))
    return np.array([s, d, psi - angle])


def bicycle_integral(pc, control, dt):
    """Independent integration of the continuous rear-axle bicycle equations."""
    delta, acceleration = control

    def rhs(_time, state):
        speed, _, _, heading = state
        return [
            acceleration,
            speed * np.cos(heading),
            speed * np.sin(heading),
            speed * np.tan(delta) / WHEELBASE,
        ]

    solution = solve_ivp(rhs, (0, dt), pc, method="DOP853", rtol=2e-12, atol=2e-13)
    assert solution.success
    return solution.y[:, -1]


def centered_jacobian(function, argument, step=1e-5):
    argument = np.asarray(argument, dtype=float)
    perturbations = np.eye(len(argument)) * step
    return np.column_stack(
        [
            (function(argument + change) - function(argument - change)) / (2 * step)
            for change in perturbations
        ]
    )


def seed_rollout(planner, initial):
    planner.X = np.empty((planner.N + 1, 7))
    planner.U = np.tile(planner.previous_input, (planner.N, 1))
    planner.X[0] = initial
    for stage in range(planner.N):
        planner.X[stage + 1] = planner.integrate(
            planner.X[stage], planner.U[stage], planner.dt
        )


@pytest.fixture(scope="module")
def numeric_planner():
    return XingPlanner(CircularRoad(0, 8.5), Settings(intervals=4))


@pytest.mark.parametrize("delta", [0.0, -1e-8, 1e-8, -0.22, 0.22])
@pytest.mark.parametrize("acceleration", [-1.3, 1.7])
@pytest.mark.parametrize("dt", [0.1, 0.7])
def test_exact_transition_matches_continuous_bicycle_integration(
    numeric_planner, delta, acceleration, dt
):
    pc, control = np.array([13.0, 4.0, -2.0, 0.6]), [delta, acceleration]
    actual = vector(numeric_planner.transition(pc, control, dt)[0])
    np.testing.assert_allclose(
        actual, bicycle_integral(pc, control, dt), rtol=2e-11, atol=2e-11
    )
    assert actual[0] == pytest.approx(pc[0] + acceleration * dt)


@pytest.mark.parametrize("delta", [0.0, 0.17])
def test_transition_when_speed_changes_sign(numeric_planner, delta):
    pc, control, dt = np.array([0.2, -1, 2, -0.4]), [delta, -1.0], 0.8
    np.testing.assert_allclose(
        vector(numeric_planner.transition(pc, control, dt)[0]),
        bicycle_integral(pc, control, dt),
        atol=2e-11,
    )


@pytest.mark.parametrize("delta", [-0.2, -1e-9, 0.0, 1e-9, 0.2])
def test_transition_jacobians_against_finite_difference_of_solve_ivp(
    numeric_planner, delta
):
    pc, control, dt = np.array([11.0, -3.0, 2.0, -0.8]), np.array([delta, 0.9]), 0.3
    _, A, B = numeric_planner.transition(pc, control, dt)
    expected_a = centered_jacobian(
        lambda state: bicycle_integral(state, control, dt), pc
    )
    expected_b = centered_jacobian(
        lambda action: bicycle_integral(pc, action, dt), control
    )
    assert np.isfinite(np.asarray(A)).all() and np.isfinite(np.asarray(B)).all()
    np.testing.assert_allclose(np.asarray(A), expected_a, rtol=2e-7, atol=2e-8)
    np.testing.assert_allclose(np.asarray(B), expected_b, rtol=2e-7, atol=2e-8)


def test_zero_steering_jacobian_has_correct_analytic_continuation(numeric_planner):
    pc, acceleration, dt = np.array([12.0, 3.0, -5.0, 0.7]), -0.8, 0.4
    distance = pc[0] * dt + 0.5 * acceleration * dt**2
    sine, cosine = np.sin(pc[3]), np.cos(pc[3])
    expected_a = np.eye(4)
    expected_a[1, 0], expected_a[2, 0] = dt * cosine, dt * sine
    expected_a[1, 3], expected_a[2, 3] = -distance * sine, distance * cosine
    expected_b = np.array(
        [
            [0, dt],
            [-(distance**2) * sine / (2 * WHEELBASE), 0.5 * dt**2 * cosine],
            [distance**2 * cosine / (2 * WHEELBASE), 0.5 * dt**2 * sine],
            [distance / WHEELBASE, 0],
        ]
    )
    _, A, B = numeric_planner.transition(pc, [0, acceleration], dt)
    np.testing.assert_allclose(np.asarray(A), expected_a, atol=1e-12)
    np.testing.assert_allclose(np.asarray(B), expected_b, atol=1e-12)


@pytest.mark.parametrize("kappa", [0.0, 1 / 60, -1 / 60])
@pytest.mark.parametrize("s,d,phi", [(0, 0, 0), (37, 1.2, 0.23), (-15, -1.8, -0.31)])
def test_eq19_connection_finite_difference_including_phi_row(kappa, s, d, phi):
    planner = XingPlanner(CircularRoad(kappa, 8.5), Settings(intervals=4))
    pc = np.r_[11.0, reference_pose(s, d, phi, kappa)]
    pf, G = planner.connection(pc, s)
    np.testing.assert_allclose(pf, [s, d, phi], atol=1e-12)
    np.testing.assert_allclose(
        planner.project(pc, s), reference_project(pc, kappa, s), atol=1e-12
    )
    expected = centered_jacobian(lambda state: reference_project(state, kappa, s), pc)
    np.testing.assert_allclose(G, expected, rtol=2e-7, atol=2e-8)
    np.testing.assert_allclose(G[:, 0], 0, atol=0)
    np.testing.assert_allclose(G[2, :3], -kappa * G[0, :3], atol=1e-14)
    assert G[2, 3] == pytest.approx(1)
    # Eq. 17 must be exact at the selected expansion point.
    affine_offset = pf - G @ pc
    np.testing.assert_allclose(G @ pc + affine_offset, [s, d, phi], atol=1e-12)


@pytest.mark.parametrize("kappa", [1 / 60, -1 / 60])
def test_connection_preserves_circle_branch_near_multiple_turns(kappa):
    planner = XingPlanner(CircularRoad(kappa, 8.5), Settings(intervals=4))
    s, d, phi = (2 * np.pi + 0.4) / kappa, 0.8, -0.2
    pc = np.r_[10, reference_pose(s, d, phi, kappa)]
    pf, G = planner.connection(pc, s)
    np.testing.assert_allclose(pf, [s, d, phi], atol=2e-12)
    expected = centered_jacobian(lambda state: reference_project(state, kappa, s), pc)
    np.testing.assert_allclose(G, expected, atol=2e-8, rtol=2e-7)


@pytest.mark.parametrize("delta", [0, -0.2, 0.2])
def test_lateral_acceleration_jacobians(numeric_planner, delta):
    pc, control = np.array([11, 2, -1, 0.3]), np.array([delta, 0.4])
    value, J, K = numeric_planner.lateral_acceleration(pc, control)
    expected = pc[0] ** 2 * np.tan(delta) / WHEELBASE
    assert float(value) == pytest.approx(expected)
    np.testing.assert_allclose(
        vector(J), [2 * pc[0] * np.tan(delta) / WHEELBASE, 0, 0, 0]
    )
    np.testing.assert_allclose(
        vector(K), [pc[0] ** 2 / (WHEELBASE * np.cos(delta) ** 2), 0]
    )


def body_points(pose):
    """Dense chassis edges/interior with rear axle x interval [-1, 5.5]."""
    xx, yy = np.meshgrid(np.linspace(-1, 5.5, 151), np.linspace(-1.15, 1.15, 81))
    local = np.c_[xx.ravel(), yy.ravel()]
    cosine, sine = np.cos(pose[2]), np.sin(pose[2])
    return local @ np.array([[cosine, sine], [-sine, cosine]]) + pose[:2]


def exact_lateral_interval(pose, kappa):
    local_corners = np.array([[-1, -1.15], [5.5, -1.15], [5.5, 1.15], [-1, 1.15]])
    cosine, sine = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    corners = local_corners @ rotation.T + pose[:2]
    if kappa == 0:
        return np.min(corners[:, 1]), np.max(corners[:, 1])
    center = np.array([0.0, 1 / kappa])
    center_local = rotation.T @ (center - pose[:2])
    closest_local = np.clip(center_local, [-1, -1.15], [5.5, 1.15])
    minimum_radius = np.linalg.norm(center_local - closest_local)
    maximum_radius = np.linalg.norm(corners - center, axis=1).max()
    offsets = np.sign(kappa) * (
        abs(1 / kappa) - np.array([minimum_radius, maximum_radius])
    )
    return offsets.min(), offsets.max()


@pytest.mark.parametrize("kappa", [0.0, 1 / 60, -1 / 60])
@pytest.mark.parametrize("s,d,phi", [(0, 0, 0), (8, 0.8, 0.3), (12, -1.1, -0.35)])
def test_corridor_road_margins_equal_true_frozen_rectangle_geometry(kappa, s, d, phi):
    settings = Settings(intervals=4, lateral=0, speed=12)
    planner = XingPlanner(CircularRoad(kappa, 8.5), settings)
    pose = reference_pose(s, d, phi, kappa)
    state = np.r_[12, pose, s, d, phi]
    # Obstacle is well ahead; lower lateral bound is solely the road boundary.
    _, _, lower_d, upper_d = planner.corridor(state[None], time=0)[0]
    dmin, dmax = exact_lateral_interval(pose, kappa)
    usable_width = 8.5 - settings.road_margin
    assert lower_d == pytest.approx(-usable_width - (dmin - d), abs=2e-12)
    assert upper_d == pytest.approx(usable_width - (dmax - d), abs=2e-12)
    expected_margin = 8.5 - max(abs(dmin), abs(dmax))
    assert min(d - lower_d, upper_d - d) + settings.road_margin == pytest.approx(
        expected_margin
    )
    points = body_points(pose)
    if kappa == 0:
        offsets = points[:, 1]
    else:
        radii = np.linalg.norm(points - [0, 1 / kappa], axis=1)
        offsets = np.sign(kappa) * (abs(1 / kappa) - radii)
    assert offsets.min() >= dmin - 1e-12
    assert offsets.max() <= dmax + 1e-12
    np.testing.assert_allclose([offsets.min(), offsets.max()], [dmin, dmax], atol=2e-4)


@pytest.mark.parametrize("sign", [-1, 1])
def test_corridor_accounts_for_radial_edge_interior_extremum(sign):
    kappa = sign * 0.1
    planner = XingPlanner(CircularRoad(kappa, 8.5), Settings(intervals=4))
    pose = np.array([0, sign, 0.0])
    state = np.r_[12, pose, 0, sign, 0]
    low, high = planner._extents(state)
    expected = exact_lateral_interval(pose, kappa)
    np.testing.assert_allclose(
        [low[1] + state[5], high[1] + state[5]], expected, atol=1e-12
    )
    # x=0 is inside the long edge; none of the four corners reaches this offset.
    closest_radius = 10 - 1 - 1.15
    assert max(abs(expected[0]), abs(expected[1])) >= 10 - closest_radius


def test_frozen_straight_corridor_inflates_obstacle_by_actual_rear_overhang():
    planner = XingPlanner(CircularRoad(0, 8.5), Settings(intervals=4))
    state = np.array([12, 35, 0, 0, 35, 0, 0])
    _, smax, dmin, dmax = planner.corridor(state[None], time=0)[0]
    assert np.isposinf(smax)
    assert dmin == pytest.approx(2.3 + planner.settings.obstacle_margin)
    assert dmax == pytest.approx(8.5 - planner.settings.road_margin - 1.15)
    yielding = XingPlanner(
        CircularRoad(0, 8.5), Settings(intervals=4), XingOptions(decision="yield")
    )
    _, smax, dmin, _ = yielding.corridor(state[None], time=0)[0]
    assert smax == pytest.approx(35 - 1 - 5.5 - yielding.settings.obstacle_margin)
    assert dmin == pytest.approx(-8.5 + yielding.settings.road_margin + 1.15)


def test_first_input_increment_uses_previous_executed_input_and_later_differences():
    planner = XingPlanner(
        CircularRoad(0, 8.5), Settings(intervals=4, lateral=0, speed=12)
    )
    planner.previous_input = np.array([0.06, 0.7])
    seed_rollout(planner, planner.initial())
    planner.U[0] = [0.07, 0.8]  # A new candidate must not replace executed history.
    rows, q = planner._qp(planner.initial(), time=0)
    matrix = rows.matrix().toarray()
    lower, upper = np.asarray(rows.lo), np.asarray(rows.hi)
    for stage in range(planner.N):
        for component in range(2):
            coefficients = np.zeros(planner.nz)
            coefficients[planner.uoffset + 2 * stage + component] = 1
            if stage:
                coefficients[planner.uoffset + 2 * (stage - 1) + component] = -1
            candidates = np.flatnonzero(np.all(matrix == coefficients, axis=1))
            offset = planner.previous_input[component] if stage == 0 else 0
            matches = [
                index
                for index in candidates
                if np.isclose(lower[index], offset - planner.du[component])
                and np.isclose(upper[index], offset + planner.du[component])
            ]
            assert len(matches) == 1
    expected_q = -2 * planner.r * planner.previous_input / planner.objective_scale
    np.testing.assert_allclose(q[planner.uoffset : planner.uoffset + 2], expected_q)
    first = planner.U[0].copy()
    planner.shift()
    np.testing.assert_array_equal(planner.previous_input, first)


@pytest.mark.parametrize("kappa", [0.0, 1 / 60, -1 / 60])
@pytest.mark.parametrize("speed,lateral", [(12, 0), (13, 0.5)])
def test_short_tracking_qp_is_physically_feasible(kappa, speed, lateral):
    planner = XingPlanner(
        CircularRoad(kappa, 8.5),
        Settings(intervals=4, speed=speed, lateral=lateral),
        XingOptions(feasibility_tolerance=1e-7),
    )
    initial = planner.initial()
    result = planner.solve(initial, time=0)
    assert result["status"] == 0, result
    assert result["qp_status"] == "solved"
    assert result["max_violation"] <= planner.options.feasibility_tolerance
    assert result["coordinate_error"] < 1e-7
    assert result["native_solver_ms"] is None
    assert result["qp_kernel_sum_ms"] > 0
    assert 1 <= result["iterations"] <= planner.options.max_iterations
    np.testing.assert_allclose(planner.X[0], initial, atol=1e-7)
    previous = planner.previous_input.copy()
    for stage, action in enumerate(planner.U):
        assert np.all(action >= planner.ul - 1e-7)
        assert np.all(action <= planner.uu + 1e-7)
        assert np.all(np.abs(action - previous) <= planner.du + 1e-7)
        expected = bicycle_integral(planner.X[stage, :4], action, planner.dt)
        np.testing.assert_allclose(planner.X[stage + 1, :4], expected, atol=1e-7)
        previous = action
    for state in planner.X:
        np.testing.assert_allclose(
            state[4:], reference_project(state[:4], kappa, state[4]), atol=1e-7
        )
        dmin, dmax = exact_lateral_interval(state[1:4], kappa)
        assert min(8.5 + dmin, 8.5 - dmax) >= planner.settings.road_margin - 1e-7
    if speed > initial[0]:
        assert planner.U[0, 1] > 0
        assert planner.X[-1, 0] > initial[0]


def test_iteration_limit_retained_even_when_qp_solution_is_feasible():
    planner = XingPlanner(
        CircularRoad(0, 8.5),
        Settings(intervals=4, speed=15, lateral=1),
        XingOptions(max_iterations=1, apriori_tolerance_m=0),
    )
    result = planner.solve(planner.initial(), time=0)
    assert result["qp_status"] == "solved"
    assert result["max_violation"] <= planner.options.feasibility_tolerance
    assert result["status"] == 2
    assert result["iterations"] == len(result["iteration_history"]) == 1
    assert result["iteration_history"][0]["apriori_error_m"] > 0
    assert np.isfinite(planner.X).all() and np.isfinite(planner.U).all()


def test_real_infeasible_initial_state_keeps_failure_and_original_rollout():
    planner = XingPlanner(
        CircularRoad(0, 8.5), Settings(intervals=4, speed=12, lateral=0)
    )
    initial = planner.initial()
    initial[0] = planner.settings.max_speed + 5
    seed_rollout(planner, initial)
    before_x, before_u = planner.X.copy(), planner.U.copy()
    result = planner.solve(initial, time=0)
    assert result["status"] == 4
    assert "infeasible" in result["qp_status"]
    assert result["iterations"] == len(result["iteration_history"]) == 1
    assert result["max_violation"] >= 5
    np.testing.assert_array_equal(planner.X, before_x)
    np.testing.assert_array_equal(planner.U, before_u)


def test_later_qp_failure_retains_last_successful_iterate(monkeypatch):
    planner = XingPlanner(
        CircularRoad(1 / 60, 8.5),
        Settings(intervals=4, speed=13, lateral=0.5),
        XingOptions(apriori_tolerance_m=0),
    )
    real_qp = xing2022.osqp.OSQP
    previous_input = planner.previous_input.copy()
    results = []

    class FailSecondQp:
        def __init__(self):
            self.solver = real_qp()

        def setup(self, **kwargs):
            self.solver.setup(**kwargs)

        def warm_start(self, **kwargs):
            self.solver.warm_start(**kwargs)

        def solve(self, **kwargs):
            if not results:
                result = self.solver.solve(**kwargs)
                assert result.info.status_val == 1
            else:
                result = SimpleNamespace(
                    x=np.full(planner.nz, np.nan),
                    info=SimpleNamespace(
                        status_val=7,
                        status="maximum iterations reached",
                        run_time=0.001,
                    ),
                )
            results.append(result)
            return result

    monkeypatch.setattr(xing2022.osqp, "OSQP", FailSecondQp)
    result = planner.solve(planner.initial(), time=0)
    assert result["status"] == 4
    assert result["iterations"] == len(results) == 2
    assert result["iteration_history"][0]["qp_status"] == "solved"
    assert result["iteration_history"][1]["qp_status"] == "maximum iterations reached"
    np.testing.assert_array_equal(
        planner.X, results[0].x[: planner.uoffset].reshape(planner.N + 1, 7)
    )
    np.testing.assert_array_equal(
        planner.U, results[0].x[planner.uoffset :].reshape(planner.N, 2)
    )
    # Solving a prediction does not execute an input; only shift() advances it.
    np.testing.assert_array_equal(planner.previous_input, previous_input)


@pytest.mark.parametrize(
    "status_value,label",
    [
        (2, "solved inaccurate"),
        (3, "primal infeasible"),
        (7, "maximum iterations reached"),
    ],
)
def test_unsuccessful_qp_iterate_is_not_accepted_or_retried(
    monkeypatch, status_value, label
):
    planner = XingPlanner(
        CircularRoad(0, 8.5), Settings(intervals=4, speed=12, lateral=0)
    )
    seed_rollout(planner, planner.initial())
    before_x, before_u = planner.X.copy(), planner.U.copy()
    calls = []

    class FailedQp:
        def setup(self, **_kwargs):
            pass

        def warm_start(self, **_kwargs):
            pass

        def solve(self, **_kwargs):
            calls.append(1)
            return SimpleNamespace(
                x=np.full(planner.nz, np.nan),
                info=SimpleNamespace(
                    status_val=status_value, status=label, run_time=0.001
                ),
            )

    monkeypatch.setattr(xing2022.osqp, "OSQP", FailedQp)
    result = planner.solve(planner.initial(), time=0)
    assert result["status"] == 4
    assert result["qp_status"] == label
    assert len(calls) == result["iterations"] == 1
    assert result["iteration_history"] == [{"iteration": 1, "qp_status": label}]
    np.testing.assert_array_equal(planner.X, before_x)
    np.testing.assert_array_equal(planner.U, before_u)
