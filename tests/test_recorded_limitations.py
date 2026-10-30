"""Document known limitations without changing the immutable experiments."""

import casadi as ca
import numpy as np

from paper_comparison.evaluation import (
    pose_from_state,
    rectangle_vertices,
    road_clearance,
)
from paper_comparison.model import approximate_road_offsets
from reiter2023 import CircularRoad


def test_approximate_road_margin_is_not_a_physical_margin_guarantee():
    # Observed width-sweep case, physical state49 (not a synthetic failure).
    state = np.array(
        [
            72.49861341797929,
            3.555727592739441,
            -0.0009255483729755409,
            14.952857846149424,
            -0.15586872467097404,
            0.013741367312772769,
        ]
    )
    road = CircularRoad(1 / 60, 4.9)
    x = ca.MX.sym("x", 6)
    offsets = ca.Function(
        "known_approximation", [x], [approximate_road_offsets(x, road)]
    )
    optimizer_margin = road.half_width - np.max(np.abs(np.asarray(offsets(state))))
    physical_margin = road_clearance(
        rectangle_vertices(pose_from_state(state, road.kappa)),
        road.kappa,
        road.half_width,
    )
    assert optimizer_margin >= 0.2 - 1e-6
    assert 0.194 < physical_margin < 0.195
    assert optimizer_margin - physical_margin > 0.005
