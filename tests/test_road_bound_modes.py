"""Published reference-point road bounds versus bus-contour road bounds."""

from dataclasses import replace

import numpy as np

from paper_comparison.model import Settings
from paper_comparison.xing2022 import XingOptions, XingPlanner
from reiter2023.geometry import CircularRoad

HALF_WIDTH = 4.6


def prior(planner, road, s0=0.0, heading=0.0):
    """Centerline prior in original order [V,X,Y,psi,S,D,phi]."""
    rows = []
    for j in range(planner.N + 1):
        s = s0 + 1.2 * j
        pose = np.asarray(road.numeric_pose([s, 0.0, heading])).ravel()
        rows.append(np.r_[12.0, pose, s, 0.0, heading])
    return np.array(rows)


def corridor(road_bound, kappa, heading=0.0):
    road = CircularRoad(kappa, HALF_WIDTH)
    # Obstacle far ahead so only the road bounds are active.
    options = XingOptions(road_bound=road_bound, obstacle_start=500.0)
    planner = XingPlanner(road, replace(Settings(), lateral=2.6), options)
    return planner, planner.corridor(prior(planner, road, heading=heading), 0.0)


def test_point_bounds_are_constant_half_width_shrink():
    for kappa in (0.0, 1 / 60, -1 / 40):
        _, bounds = corridor("point", kappa, heading=0.2)
        limit = HALF_WIDTH - 0.2 - 1.15
        assert np.allclose(bounds[:, 2], -limit)
        assert np.allclose(bounds[:, 3], limit)


def test_contour_bounds_tighten_with_heading_on_curve():
    _, point = corridor("point", 1 / 60, heading=0.2)
    _, contour = corridor("contour", 1 / 60, heading=0.2)
    # A yawed 6.5 m bus occupies more lateral width than its 2.3 m body width.
    assert np.all(contour[:, 3] < point[:, 3] - 0.1)
    assert np.all(contour[:, 2] > point[:, 2] + 0.1)


def test_obstacle_parameters_move_the_wall():
    road = CircularRoad(0.0, HALF_WIDTH)
    settings = replace(Settings(), lateral=2.6)
    near = XingPlanner(road, settings, XingOptions(obstacle_start=20.0))
    far = XingPlanner(road, settings, XingOptions(obstacle_start=60.0))
    # Lateral obstacle wall applies only near the obstacle's s-interval.
    X = prior(near, road, s0=20.0)
    assert near.corridor(X, 0.0)[0, 2] > far.corridor(X, 0.0)[0, 2]
