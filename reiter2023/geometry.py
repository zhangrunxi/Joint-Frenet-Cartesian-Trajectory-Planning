"""Rear-axle Frenet/Cartesian transforms with an exact circular test geometry."""

from dataclasses import dataclass

import casadi as ca
import numpy as np
from vehicle_models.utils import get_circle_rect_cover


@dataclass(frozen=True)
class CircularRoad:
    kappa: float = 0.0
    half_width: float = 8.5
    heading0: float = 0.0

    def curvature(self, s):
        return self.kappa

    def pose(self, s, n, alpha):
        heading = self.heading0 + self.kappa * s
        if abs(self.kappa) < 1e-12:
            x, y = s * ca.cos(self.heading0), s * ca.sin(self.heading0)
        else:
            x = (ca.sin(heading) - ca.sin(self.heading0)) / self.kappa
            y = (ca.cos(self.heading0) - ca.cos(heading)) / self.kappa
        return ca.vertcat(
            x - n * ca.sin(heading), y + n * ca.cos(heading), heading + alpha
        )

    def numeric_pose(self, state):
        return np.asarray(self.pose(*np.asarray(state)[:3]), dtype=float).ravel()


class SampledRoad:
    """Adapter for current vehicle_gym Track/Road; spline geometry is approximate."""

    def __init__(self, road):
        if hasattr(road, "s_grid_"):
            s, xy = road.s_grid_, np.c_[road.x_grid_, road.y_grid_]
            heading, kappa = road.phi_grid_, road.kappa_grid_
            self.half_width = float(min(np.min(road.nl_grid_), np.min(road.nr_grid_)))
        else:
            s, xy = road.s, road.xy
            heading, kappa = road.heading, road.curvature
            self.half_width = float(road.half_width)
        s = np.asarray(s, dtype=float)
        xy = np.asarray(xy, dtype=float)
        heading, kappa = np.unwrap(heading), np.asarray(kappa, dtype=float)
        if s.ndim != 1 or len(s) < 4 or np.any(np.diff(s) <= 0):
            raise ValueError("Road samples must have an increasing distance grid")
        if (
            xy.shape != (len(s), 2)
            or heading.shape != s.shape
            or kappa.shape != s.shape
        ):
            raise ValueError("Road geometry arrays have incompatible shapes")
        if not all(np.isfinite(a).all() for a in [s, xy, heading, kappa]):
            raise ValueError("Road geometry must be finite")
        self.domain = (float(s[0]), float(s[-1]))
        self._x = ca.interpolant("road_x", "bspline", [s], xy[:, 0])
        self._y = ca.interpolant("road_y", "bspline", [s], xy[:, 1])
        self._heading = ca.interpolant("road_heading", "bspline", [s], heading)
        self._kappa = ca.interpolant("road_kappa", "bspline", [s], kappa)

    def curvature(self, s):
        return self._kappa(s)

    def pose(self, s, n, alpha):
        heading = self._heading(s)
        return ca.vertcat(
            self._x(s) - n * ca.sin(heading),
            self._y(s) + n * ca.cos(heading),
            heading + alpha,
        )

    def numeric_pose(self, state):
        return np.asarray(self.pose(*np.asarray(state)[:3]), dtype=float).ravel()


def body_center(pose, vehicle):
    # vehicle_gym assumes the symmetric chassis center is half a wheelbase ahead.
    return pose[:2] + vehicle.length / 2 * ca.vertcat(ca.cos(pose[2]), ca.sin(pose[2]))


def covering_circles(pose, vehicle, count):
    if not isinstance(count, int) or count < 1:
        raise ValueError("Circle count must be a positive integer")
    if count >= 2:
        minimum = int(np.ceil(vehicle.chassis_length / vehicle.chassis_width))
        if count < minimum:
            raise ValueError(
                f"{count} circles do not cover this rectangle; need >= {minimum}"
            )
    xs, ys, radius = get_circle_rect_cover(
        count,
        vehicle.length,
        vehicle.chassis_length,
        vehicle.chassis_width,
        pose[0],
        pose[1],
        pose[2],
    )
    return [ca.vertcat(x, y) for x, y in zip(xs, ys)], float(radius)


def covering_circles_paper(pose, vehicle, count):
    """The N-circle covering of the manuscript's Eq. (11)-(14).

    In the manuscript the covering radius depends on the number of circles,

        r(N) = (W/2) * sqrt(1 + lambda^2 / N^2),   lambda = L / W

    so that N circles of that radius distributed along the longitudinal axis
    cover the rectangle with the smallest possible redundancy. This differs
    from ``covering_circles``, which uses the fixed-radius covering of
    ``vehicle_gym`` (radius W/sqrt(2), independent of N); the two agree only
    when N equals the aspect ratio.

    The circle centres are spread evenly along the axis between the two ends
    of the rectangle, which is the layout the redundancy expression assumes.
    """
    if not isinstance(count, int) or count < 1:
        raise ValueError("Circle count must be a positive integer")

    length = vehicle.chassis_length
    width = vehicle.chassis_width
    aspect = length / width
    radius = 0.5 * width * float(np.sqrt(1.0 + aspect**2 / count**2))

    # The rectangle spans [-L/2, L/2] about the body centre, which sits
    # wheelbase/2 ahead of the rear axle.
    d_rear = (length - vehicle.length) / 2.0
    body_center = d_rear + vehicle.length / 2.0

    if count == 1:
        offsets = np.zeros(1)
    else:
        offsets = np.linspace(-length / 2.0, length / 2.0, count)

    heading = pose[2]
    cosine, sine = ca.cos(heading), ca.sin(heading)
    centers = []
    for offset in offsets:
        x = body_center + offset
        centers.append(ca.vertcat(pose[0] + x * cosine, pose[1] + x * sine))
    return centers, radius


def ellipse_constraint(ego_pose, obstacle_pose, ego, obstacle, margin=0.0):
    """Equation (14), with a circumscribing ego circle and inflated obstacle axes."""
    radius = np.hypot(ego.chassis_length, ego.chassis_width) / 2
    axes = np.array([obstacle.chassis_length, obstacle.chassis_width]) / np.sqrt(2)
    axes += radius + margin
    delta = body_center(ego_pose, ego) - body_center(obstacle_pose, obstacle)
    angle = obstacle_pose[2]
    local = ca.vertcat(
        ca.cos(angle) * delta[0] + ca.sin(angle) * delta[1],
        -ca.sin(angle) * delta[0] + ca.cos(angle) * delta[1],
    )
    return ca.sumsqr(local / axes) - 1
