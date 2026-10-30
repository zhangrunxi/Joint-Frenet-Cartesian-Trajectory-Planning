"""Symbolic paper-model regressions; never build an acados solver or JIT code."""

import casadi as ca
import numpy as np
import pytest

from paper_comparison import cilqr
from paper_comparison.evaluation import polygon_clearance, rectangle_vertices
from paper_comparison.model import (
    PaperModel,
    Settings,
    approximate_road_offsets,
    hybrid_constraints,
    hybrid_parts,
    segment_squared,
)
from reiter2023.geometry import CircularRoad


def numeric(value):
    return np.asarray(value, dtype=float).ravel()


@pytest.fixture(scope="module", params=[0.0, 0.025, -0.025])
def models(request):
    road = CircularRoad(request.param)
    settings = Settings(intervals=3)
    return (
        PaperModel(road, settings, lifted=False),
        PaperModel(road, settings, lifted=True),
    )


@pytest.fixture(scope="module")
def hybrid_functions():
    pose, other = ca.MX.sym("pose", 3), ca.MX.sym("other", 3)
    a, b, radius, circles, circle_radius = hybrid_parts(pose)
    parts = ca.Function(
        "parts", [pose], [a, b, radius, ca.horzcat(*circles), circle_radius]
    )
    constraints = ca.Function(
        "pairs", [pose, other], [hybrid_constraints(pose, other, 0.2)]
    )
    return parts, constraints


def test_six_state_rhs_matches_physical_derivatives(models):
    direct, _ = models
    rng = np.random.default_rng(81)
    kappa = direct.road.kappa
    for _ in range(16):
        _s, n, alpha, v, acc, curvature = state = np.array(
            [
                rng.uniform(-30, 50),
                rng.uniform(-3, 3),
                rng.uniform(-0.4, 0.4),
                rng.uniform(2, 18),
                rng.uniform(-2, 2),
                rng.uniform(-0.08, 0.08),
            ]
        )
        jerk, curvature_rate = control = rng.uniform([-3, -0.03], [3, 0.03])
        sd = v * np.cos(alpha) / (1 - n * kappa)
        expected = [
            sd,
            v * np.sin(alpha),
            v * curvature - kappa * sd,
            acc,
            jerk,
            curvature_rate,
        ]
        np.testing.assert_allclose(
            numeric(direct.ode(state, control)), expected, atol=1e-12
        )


def test_lifted_rhs_equals_frenet_transform_jacobian(models):
    direct, lifted = models
    transform = direct.road.pose(*[direct.x[i] for i in range(3)])
    jacobian = ca.Function(
        "pose_jacobian", [direct.x], [ca.jacobian(transform, direct.x)]
    )
    rng = np.random.default_rng(17)
    for _ in range(12):
        state = rng.uniform([-20, -3, -0.3, 3, -2, -0.05], [60, 3, 0.3, 18, 2, 0.05])
        control = rng.uniform([-3, -0.02], [3, 0.02])
        augmented = lifted.augment(state)
        np.testing.assert_allclose(augmented[6:], direct.road.numeric_pose(state))
        rhs = numeric(lifted.ode(augmented, control))
        np.testing.assert_allclose(
            rhs[:6], numeric(direct.ode(state, control)), atol=1e-12
        )
        np.testing.assert_allclose(
            rhs[6:],
            np.asarray(jacobian(state)) @ numeric(direct.ode(state, control)),
            atol=1e-12,
        )


def test_rk4_common_state_and_coordinate_error_convergence(models):
    direct, lifted = models
    state, control = np.array([12, 1.2, 0.25, 13, 0.7, 0.06]), [0.3, 0.015]
    errors = []
    for dt in [0.2, 0.1, 0.05]:
        six = numeric(direct.integrate(state, control, dt))
        nine = numeric(lifted.integrate(lifted.augment(state), control, dt))
        np.testing.assert_allclose(six, nine[:6], atol=1e-13, rtol=0)
        errors.append(np.max(np.abs(nine[6:] - direct.road.numeric_pose(six))))
    assert errors[0] < 1e-4
    if direct.road.kappa == 0:
        # With heading0=0 this transform is affine, so RK4 commutes exactly.
        np.testing.assert_allclose(errors, 0, atol=1e-13)
    else:
        # RK4 is not exactly invariant under nonlinear coordinate transforms.
        assert errors[1] < errors[0] / 12
        assert errors[2] < errors[1] / 12
    np.testing.assert_allclose(
        numeric(direct.step(state, control)),
        numeric(direct.integrate(state, control, direct.dt)),
    )


@pytest.mark.parametrize("curvature", [-0.12, -0.04, 0, 0.04, 0.12])
@pytest.mark.parametrize("curvature_rate", [-0.07, 0.07])
def test_steering_rate_is_exact_atan_derivative(models, curvature, curvature_rate):
    direct, _ = models
    state = np.array([5, 1, 0.1, 11, 0.5, curvature])
    control, parameters = [0.4, curvature_rate], [40, 20, 0, 10, 0, 15]
    constraints = numeric(direct.hfun(state, control, parameters))
    expected = 4.32 * curvature_rate / (1 + (4.32 * curvature) ** 2)
    assert direct.vehicle.length == pytest.approx(4.32)
    assert constraints[2] == pytest.approx(expected)
    delta = ca.atan(4.32 * direct.x[5])
    derivative = ca.Function(
        "delta_dot", [direct.x, direct.u], [ca.jacobian(delta, direct.x) @ direct.rhs]
    )
    assert float(derivative(state, control)) == pytest.approx(expected)
    residual = ca.Function(
        "residual", [direct.x, direct.u, direct.p], [direct.residual]
    )
    np.testing.assert_allclose(
        numeric(residual(state, control, parameters))[5:7],
        [curvature, curvature_rate],
    )
    assert constraints[0] == pytest.approx(state[3] ** 2 * curvature)
    assert constraints[1] == pytest.approx(
        2 * state[3] * state[4] * curvature + state[3] ** 2 * curvature_rate
    )


def point_segment_distances(points, a, b):
    edge = b - a
    t = np.clip((points - a) @ edge / (edge @ edge), 0, 1)
    return np.linalg.norm(points - a - t[:, None] * edge, axis=1)


@pytest.mark.parametrize("pose", [[0, 0, 0], [14, -8, 0.73], [-5, 3, -1.2]])
def test_hybrid_union_covers_dense_chassis_edges_and_interior(hybrid_functions, pose):
    parts, _ = hybrid_functions
    a, b, radius, centers, circle_radius = parts(pose)
    a, b, centers = numeric(a), numeric(b), np.asarray(centers).T
    corners = rectangle_vertices(pose)
    longitudinal, lateral = np.meshgrid(np.linspace(0, 1, 181), np.linspace(0, 1, 81))
    points = (
        corners[0]
        + longitudinal.ravel()[:, None] * (corners[1] - corners[0])
        + lateral.ravel()[:, None] * (corners[3] - corners[0])
    )
    capsule_margin = point_segment_distances(points, a, b) - float(radius)
    circle_margin = np.linalg.norm(points[:, None] - centers[None], axis=2).min(axis=1)
    circle_margin -= float(circle_radius)
    # The capsule misses corners; the end circles miss the central strip.
    assert capsule_margin.max() > 0
    assert circle_margin.max() > 0
    assert np.minimum(capsule_margin, circle_margin).max() <= 1e-12


def test_hybrid_parts_use_rear_overhang_not_wheelbase_center(hybrid_functions):
    parts, _ = hybrid_functions
    a, b, radius, centers, circle_radius = parts([0, 0, 0])
    np.testing.assert_allclose(numeric(a), [0.15, 0], atol=1e-14)
    np.testing.assert_allclose(numeric(b), [4.35, 0], atol=1e-14)
    np.testing.assert_allclose(
        np.asarray(centers).T,
        [[-0.425, -0.575], [-0.425, 0.575], [4.925, -0.575], [4.925, 0.575]],
    )
    assert float(radius) == pytest.approx(1.15)
    assert float(circle_radius) == pytest.approx(np.sqrt(2) * 0.575)


@pytest.mark.parametrize(
    "points, expected",
    [
        ([[-2, 0], [2, 0], [0, -2], [0, 2]], 0),
        ([[0, 0], [4, 0], [1, 3], [3, 3]], 9),
        ([[0, 0], [2, 0], [1, 0], [3, 0]], 0),
        ([[0, 0], [1, 0], [3, 0], [4, 0]], 4),
        ([[0, 0], [1, 0], [1, 0], [1, 2]], 0),
        ([[0, 0], [1, 0], [3, 2], [3, 4]], 8),
        ([[0, 0], [0, 0], [1, -1], [1, 1]], 1),
        ([[0, 0], [0, 0], [3, 4], [3, 4]], 25),
    ],
)
def test_segment_distance_crossing_parallel_and_degenerate(points, expected):
    symbols = [ca.MX.sym(name, 2) for name in ["a", "b", "c", "d"]]
    distance = ca.Function("segment_distance", symbols, [segment_squared(*symbols)])
    a, b, c, d = np.asarray(points, dtype=float)
    for ordered in [(a, b, c, d), (b, a, c, d), (c, d, a, b), (a, b, d, c)]:
        assert float(distance(*ordered)) == pytest.approx(expected, abs=1e-12)


def transformed_pose(pose, angle, translation):
    cosine, sine = np.cos(angle), np.sin(angle)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return np.r_[rotation @ np.asarray(pose[:2]) + translation, pose[2] + angle]


@pytest.mark.parametrize("other", [[13, 2, -0.4], [2, 0.2, 0.9], [4, 8, 1.1]])
def test_every_hybrid_pair_is_rigid_motion_invariant(hybrid_functions, other):
    _, constraints = hybrid_functions
    pose = [1, -2, 0.31]
    reference = numeric(constraints(pose, other))
    assert reference.shape == (25,)
    for angle in [-1.2, 0.7, np.pi]:
        actual = numeric(
            constraints(
                transformed_pose(pose, angle, [50, -30]),
                transformed_pose(other, angle, [50, -30]),
            )
        )
        np.testing.assert_allclose(actual, reference, atol=3e-13, rtol=1e-12)
    swapped = numeric(constraints(other, pose))
    np.testing.assert_allclose(swapped[:1], reference[:1])
    np.testing.assert_allclose(swapped[1:5], reference[5:9])
    np.testing.assert_allclose(swapped[5:9], reference[1:5])
    np.testing.assert_allclose(swapped[9:].reshape(4, 4), reference[9:].reshape(4, 4).T)


def test_all_25_hybrid_constraints_have_expected_pair_distances(hybrid_functions):
    parts, constraints = hybrid_functions
    a, b, r, circles, rc = parts([0, 0, 0])
    c, d, q, others, qc = parts([1, 8, 0])
    a, b, c, d = map(numeric, [a, b, c, d])
    circles, others = np.asarray(circles).T, np.asarray(others).T
    expected = [8 - float(r) - float(q) - 0.2]
    expected.extend(point_segment_distances(circles, c, d) - float(rc) - float(q) - 0.2)
    expected.extend(point_segment_distances(others, a, b) - float(qc) - float(r) - 0.2)
    expected.extend(
        (
            np.linalg.norm(circles[:, None] - others[None], axis=2)
            - float(rc)
            - float(qc)
            - 0.2
        ).ravel()
    )
    np.testing.assert_allclose(numeric(constraints([0, 0, 0], [1, 8, 0])), expected)


def test_feasible_hybrid_constraints_imply_actual_rectangles_do_not_overlap(
    hybrid_functions,
):
    _, constraints = hybrid_functions
    rng = np.random.default_rng(105)
    safe, overlapping = 0, 0
    for _ in range(240):
        pose = np.r_[rng.uniform(-3, 3, 2), rng.uniform(-np.pi, np.pi)]
        other = np.r_[rng.uniform(-12, 12, 2), rng.uniform(-np.pi, np.pi)]
        values = numeric(constraints(pose, other))
        actual = polygon_clearance(rectangle_vertices(pose), rectangle_vertices(other))
        if values.min() >= 0:
            safe += 1
            assert actual >= 0.2 - 1e-12
        if actual < -1e-8:
            overlapping += 1
            assert values.min() < 0
    assert safe > 50
    assert overlapping > 10


@pytest.mark.parametrize("alpha", [-0.6, -0.2, 0, 0.3, 0.6])
def test_straight_road_offsets_equal_actual_corner_y(alpha):
    state = ca.MX.sym("state", 6)
    offsets = ca.Function(
        "road_offsets", [state], [approximate_road_offsets(state, CircularRoad(0))]
    )
    actual = numeric(offsets([4, 1.3, alpha, 12, 0, 0]))
    # Offset loop: rear-right, rear-left, front-right, front-left.
    expected = rectangle_vertices([4, 1.3, alpha])[[0, 3, 1, 2], 1]
    np.testing.assert_allclose(actual, expected, atol=1e-12)


@pytest.mark.parametrize("kappa", [-0.04, 0.04])
def test_signed_rear_overhang_integral_bends_in_correct_direction(kappa):
    state = ca.MX.sym("state", 6)
    longitudinal = np.repeat([-1.0, 5.5], 2)
    lateral = np.tile([-1.15, 1.15], 2)
    # Analytic integral of the approximation, NOT exact Cartesian corner projection.
    integral_limit = lateral + np.log(np.cos(kappa * longitudinal)) / kappa
    errors = []
    for segments in [5, 40, 320]:
        offsets = ca.Function(
            "curved_offsets",
            [state],
            [approximate_road_offsets(state, CircularRoad(kappa), segments)],
        )
        actual = numeric(offsets([0, 0, 0, 12, 0, kappa]))
        bending = actual - lateral
        assert np.all(np.sign(bending) == -np.sign(kappa))
        assert abs(bending[2]) > abs(bending[0])
        errors.append(np.max(np.abs(actual - integral_limit)))
    assert errors[1] < errors[0] / 7
    assert errors[2] < errors[1] / 7
    assert errors[2] < 0.003


def test_terminal_residual_and_cost_have_no_control_dependency(models):
    for model in models:
        # Construction without u fails if even an indirect free symbol survives.
        terminal = ca.Function(
            "terminal_x_p",
            [model.x, model.p],
            [model.terminal_residual, model.terminal_cost],
        )
        assert not ca.depends_on(model.terminal_residual, model.u)
        assert not ca.depends_on(model.terminal_cost, model.u)
        state = model.augment([2, 1, 0.2, 10, 0.5, 0.04])
        residual, cost = terminal(state, [40, 5, 0, 3, 2, 15])
        expected = np.array([-1, -1, 0.2, -5, 0.5, 0.04])
        np.testing.assert_allclose(numeric(residual), expected)
        assert float(cost) == pytest.approx(
            np.dot(model.settings.terminal_weights, expected**2)
        )


def test_cilqr_terminal_constructs_and_evaluates_without_jit(models, monkeypatch):
    original_function = ca.Function

    def without_jit(name, arguments, expressions, options):
        assert options["jit"] is True
        return original_function(name, arguments, expressions)

    # Exercise the real CiLQR constructor but suppress only its C compilation.
    monkeypatch.setattr(cilqr.ca, "Function", without_jit)
    for model in models:
        planner = cilqr.CilqrPlanner(model, max_inner=1, max_outer=1)
        state = model.augment([2, 1, 0.2, 10, 0.5, 0.04])
        outputs = planner.terminal(
            state, [40, 15, 0, 3, 2, 15], np.zeros(model.g.numel()), 100
        )
        assert [output.shape for output in outputs] == [
            (1, 1),
            (model.nx, 1),
            (model.nx, model.nx),
        ]
        assert all(np.isfinite(np.asarray(output)).all() for output in outputs)
        np.testing.assert_allclose(
            np.asarray(outputs[2]), np.asarray(outputs[2]).T, atol=1e-10
        )


@pytest.mark.parametrize(
    "obstacle, count", [("HYBRID", 25), ("ELIPSE", 1), ("CIRCLES", 9)]
)
def test_six_nine_costs_and_constraints_identical_at_consistent_pose(
    models, obstacle, count
):
    direct, lifted = models
    if obstacle != "HYBRID":
        direct = PaperModel(direct.road, direct.settings, obstacle=obstacle)
        lifted = PaperModel(
            lifted.road, lifted.settings, lifted=True, obstacle=obstacle
        )
    functions = [
        ca.Function(
            "costs",
            [m.x, m.u, m.p],
            [m.residual, m.cost, m.terminal_residual, m.terminal_cost],
        )
        for m in [direct, lifted]
    ]
    np.testing.assert_array_equal(direct.hl, lifted.hl)
    np.testing.assert_array_equal(direct.hu, lifted.hu)
    assert direct.h.numel() == lifted.h.numel() == 8 + count
    rng = np.random.default_rng(199)
    for _ in range(8):
        state = rng.uniform([-10, -2, -0.3, 5, -1, -0.04], [30, 2, 0.3, 15, 1, 0.04])
        augmented = lifted.augment(state)
        control = rng.uniform([-2, -0.02], [2, 0.02])
        parameters = np.r_[direct.road.numeric_pose([45, 0, 0]), 10, 3, 15]
        for six, nine in zip(
            functions[0](state, control, parameters),
            functions[1](augmented, control, parameters),
        ):
            np.testing.assert_allclose(np.asarray(six), np.asarray(nine), atol=1e-10)
        np.testing.assert_allclose(
            numeric(direct.hfun(state, control, parameters)),
            numeric(lifted.hfun(augmented, control, parameters)),
            atol=1e-10,
        )
        np.testing.assert_allclose(
            numeric(direct.gfun(state, control, parameters)),
            numeric(lifted.gfun(augmented, control, parameters)),
            atol=1e-10,
        )
