"""The manuscript's N-circle covering and its difference from vehicle_gym's.

The manuscript's Eq. 11-14 defines the covering radius as a function of the
number of circles, r(N) = (W/2) * sqrt(1 + lambda^2 / N^2). The covering used
by the comparison planners, ``vehicle_gym``'s ``get_circle_rect_cover``, uses a
fixed radius W/sqrt(2) and only redistributes the centres. The two agree only
when N equals the aspect ratio of the vehicle.

The two are therefore not interchangeable, and the redundancy analysis of
Fig. 7 (which uses the manuscript's expression) and the conservatism audit
(which uses the implementation's geometry) answer different questions.
"""

import sys

import casadi as ca
import numpy as np
import pytest

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

from reiter2023.geometry import covering_circles, covering_circles_paper  # noqa: E402
from reiter2023.trajectory_planner_acados_20222 import paper_vehicle  # noqa: E402


def test_paper_radius_follows_the_manuscript_expression():
    vehicle = paper_vehicle()
    length = vehicle.chassis_length
    width = vehicle.chassis_width
    aspect = length / width
    for count in (3, 5):
        _, radius = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, count)
        expected = 0.5 * width * np.sqrt(1.0 + aspect**2 / count**2)
        assert radius == pytest.approx(expected)


def test_the_two_coverings_agree_only_at_the_aspect_ratio():
    vehicle = paper_vehicle()
    aspect = vehicle.chassis_length / vehicle.chassis_width
    _, paper_radius = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, 3)
    _, gym_radius = covering_circles(ca.DM([0, 0, 0]), vehicle, 3)
    assert paper_radius != pytest.approx(gym_radius)

    # The two radii coincide when N = lambda.
    _, at_aspect = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, 3)
    expected = 0.5 * vehicle.chassis_width * np.sqrt(1.0 + aspect**2 / 3.0**2)
    assert at_aspect == pytest.approx(expected)
    assert gym_radius == pytest.approx(
        0.5 * vehicle.chassis_width * np.sqrt(2.0)
    )


def test_the_paper_covering_is_tighter_than_the_gym_covering_for_five_circles():
    """Five circles with the manuscript's radius waste less than five with the
    fixed radius, which is the point of the N-dependent radius."""
    vehicle = paper_vehicle()
    width = vehicle.chassis_width
    length = vehicle.chassis_length

    _, paper_radius = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, 5)
    _, gym_radius = covering_circles(ca.DM([0, 0, 0]), vehicle, 5)

    # Lateral redundancy, 2r - W.
    assert 2 * paper_radius - width < 2 * gym_radius - width


def test_the_paper_covering_places_one_circle_per_count():
    vehicle = paper_vehicle()
    for count in (3, 5):
        centers, _ = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, count)
        assert len(centers) == count


def test_the_paper_covering_centres_span_the_rectangle():
    """The centres run from the rear to the front of the chassis.

    The pose is the rear-axle position and the chassis centre sits half a
    wheelbase ahead of it, so the outermost centres are at the two ends of the
    body.
    """
    vehicle = paper_vehicle()
    centers, _ = covering_circles_paper(ca.DM([0, 0, 0]), vehicle, 5)
    xs = sorted(float(center[0]) for center in centers)
    # The chassis centre sits half a wheelbase ahead of the rear-axle pose,
    # offset by the rear overhang, which is the layout get_circle_rect_cover
    # uses for its outermost circles.
    d_rear = (vehicle.chassis_length - vehicle.length) / 2.0
    body_center = d_rear + vehicle.length / 2.0
    half = vehicle.chassis_length / 2.0
    assert xs[0] == pytest.approx(body_center - half)
    assert xs[-1] == pytest.approx(body_center + half)
