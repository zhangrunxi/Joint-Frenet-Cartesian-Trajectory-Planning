"""Independent NumPy geometry checks, without planner collision approximations.

Distances are in metres; positive clearance is safe, zero is contact, and
negative clearance is penetration. Polygon inputs are cyclic convex vertices.
"""

import numpy as np

from paper_comparison.vehicle import GEOMETRY


def _first_three(value, name):
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size < 3 or not np.isfinite(values[:3]).all():
        raise ValueError(f"{name} must have at least three finite components")
    return values[:3]


def _polygon(value):
    vertices = np.asarray(value, dtype=float)
    if (
        vertices.ndim != 2
        or vertices.shape[1] != 2
        or len(vertices) < 3
        or not np.isfinite(vertices).all()
    ):
        raise ValueError("polygon must contain at least three finite 2D vertices")
    edges = np.roll(vertices, -1, axis=0) - vertices
    relative = vertices - vertices[0]
    following = np.roll(relative, -1, axis=0)
    twice_area = np.sum(
        relative[:, 0] * following[:, 1] - relative[:, 1] * following[:, 0]
    )
    if np.any(np.linalg.norm(edges, axis=1) == 0) or twice_area == 0:
        raise ValueError("polygon must have nonzero edges and area")
    return vertices


def rectangle_vertices(
    pose,
    length=GEOMETRY.length,
    width=GEOMETRY.width,
    rear_overhang=GEOMETRY.rear_overhang,
):
    """Return four CCW chassis corners for a rear-axle ``[x, y, yaw]`` pose.

    The longitudinal body interval is [-rear_overhang, length-rear_overhang],
    not a rectangle centered at the rear axle or at half the wheelbase.
    """
    x, y, yaw = _first_three(pose, "pose")
    dimensions = np.asarray([length, width, rear_overhang], dtype=float)
    if (
        not np.isfinite(dimensions).all()
        or length <= 0
        or width <= 0
        or not 0 <= rear_overhang <= length
    ):
        raise ValueError("length/width must be positive and overhang within length")
    rear, front = -rear_overhang, length - rear_overhang
    local = np.array(
        [[rear, -width / 2], [front, -width / 2], [front, width / 2], [rear, width / 2]]
    )
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + [x, y]


def _point_edge_distances(points, vertices):
    """Pairwise distances from points to closed polygon edge segments."""
    edges = np.roll(vertices, -1, axis=0) - vertices
    offsets = points[:, None, :] - vertices[None, :, :]
    parameters = np.sum(offsets * edges[None, :, :], axis=2)
    parameters /= np.sum(edges * edges, axis=1)[None, :]
    parameters = np.clip(parameters, 0.0, 1.0)
    residuals = offsets - parameters[:, :, None] * edges[None, :, :]
    return np.linalg.norm(residuals, axis=2)


def polygon_clearance(a, b):
    """Return signed clearance between two convex polygons in either winding.

    For intersection, the negative SAT minimum translation depth is returned,
    including containment (not merely the length of interval intersection).
    Otherwise the result is the exact minimum Euclidean segment distance.
    """
    a, b = _polygon(a), _polygon(b)
    # A shared origin avoids projection cancellation under scene translation.
    origin = a[0].copy()
    a, b = a - origin, b - origin
    edges = np.concatenate([np.roll(a, -1, axis=0) - a, np.roll(b, -1, axis=0) - b])
    axes = np.column_stack([-edges[:, 1], edges[:, 0]])
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    projection_a, projection_b = a @ axes.T, b @ axes.T
    gaps = np.maximum(
        projection_a.min(axis=0) - projection_b.max(axis=0),
        projection_b.min(axis=0) - projection_a.max(axis=0),
    )
    signed_depth = float(gaps.max())
    if signed_depth <= 0:
        return signed_depth
    return float(
        min(_point_edge_distances(a, b).min(), _point_edge_distances(b, a).min())
    )


def road_clearance(vertices, kappa, half_width):
    """Minimum signed road margin over the *whole* convex chassis polygon.

    The road has initial heading zero: either |y| <= half_width, or a
    circular band centered at (0, 1/kappa) with nominal radius |1/kappa|.
    On the circle the margin is half_width - abs(radial_distance - radius).
    Its minimum needs both radial extrema, not just corner clearances.
    """
    vertices = _polygon(vertices)
    if not np.isfinite(kappa) or not np.isfinite(half_width) or half_width < 0:
        raise ValueError("kappa must be finite and half_width finite/nonnegative")
    if kappa == 0:
        return float(half_width - np.abs(vertices[:, 1]).max())

    center = np.array([0.0, 1.0 / kappa])
    radius = abs(1.0 / kappa)
    relative = vertices - center
    edges = np.roll(vertices, -1, axis=0) - vertices
    to_center = -relative
    crosses = edges[:, 0] * to_center[:, 1] - edges[:, 1] * to_center[:, 0]
    center_inside = np.all(crosses >= 0) or np.all(crosses <= 0)
    minimum_radius = (
        0.0
        if center_inside
        else float(_point_edge_distances(center[None], vertices).min())
    )
    maximum_radius = float(np.linalg.norm(relative, axis=1).max())
    return float(
        min(
            minimum_radius - (radius - half_width), radius + half_width - maximum_radius
        )
    )


def pose_from_state(state, kappa):
    """Convert the first Frenet components [s, n, alpha] to rear-axle pose.

    Extra state components (including lifted Cartesian variables) are ignored
    deliberately, so evaluation does not trust a planner's lifting constraints.
    """
    s, n, alpha = _first_three(state, "state")
    if not np.isfinite(kappa):
        raise ValueError("kappa must be finite")
    heading = kappa * s
    # sinc avoids subtracting nearly equal cosines near the straight-road limit.
    center_x = s * np.sinc(heading / np.pi)
    center_y = 0.5 * kappa * s * s * np.sinc(heading / (2 * np.pi)) ** 2
    return np.array(
        [
            center_x - n * np.sin(heading),
            center_y + n * np.cos(heading),
            heading + alpha,
        ]
    )


def evaluate_sample(state, other_pose, kappa, half_width):
    """Evaluate two full default chassis; ``other_pose`` is Cartesian."""
    ego = rectangle_vertices(pose_from_state(state, kappa))
    other = rectangle_vertices(other_pose)
    return {
        "obstacle_clearance_m": polygon_clearance(ego, other),
        "road_clearance_m": road_clearance(ego, kappa, half_width),
    }


def initial_state_diagnostics(case, geometry=GEOMETRY, road_margin=0.2,
                              obstacle_margin=0.2, max_lat_acc=5.0):
    """Method-independent validity of the shared [s,n,alpha]=[0,0,0] start.

    Keep invalid cases for audit, but separate them from feasible-scene rates.
    Geometry is explicit so archived SUV cases can be audited in a bus process.
    """
    kappa = case["kappa"]
    ego = rectangle_vertices([0, 0, 0], geometry.length, geometry.width,
                             geometry.rear_overhang)
    other_pose = pose_from_state(
        [case["obstacle_start"], case.get("obstacle_lateral", 0.0), 0], kappa)
    other = rectangle_vertices(other_pose, geometry.length, geometry.width,
                               geometry.rear_overhang)
    road = road_clearance(ego, kappa, case["half_width"])
    obstacle = polygon_clearance(ego, other)
    alat = case["initial_speed"] ** 2 * abs(kappa)
    checks = (("road_margin", road < road_margin - 1e-7),
              ("obstacle_margin", obstacle < obstacle_margin - 1e-7),
              ("lateral_acceleration", alat > max_lat_acc + 1e-7))
    reasons = [reason for reason, invalid in checks if invalid]
    return {
        "initial_state_valid": not reasons,
        "initial_state_invalid_reasons": reasons,
        "initial_road_clearance_m": road,
        "initial_obstacle_clearance_m": obstacle,
        "initial_lateral_acceleration_mps2": alat,
    }
