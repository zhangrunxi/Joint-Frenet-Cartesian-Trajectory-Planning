"""Exact-road variant must match the independent full-body evaluator."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_exact_road_sensitivity import ExactRoadPaperModel

from paper_comparison.evaluation import (
    pose_from_state,
    rectangle_vertices,
    road_clearance,
)
from paper_comparison.model import Settings
from reiter2023 import CircularRoad


@pytest.mark.parametrize("kappa", [0, 1 / 60, -1 / 60])
def test_exact_road_model_matches_independent_evaluator(kappa):
    m = ExactRoadPaperModel(CircularRoad(kappa, 4.6), Settings(intervals=3))
    rng = np.random.default_rng(237)
    for _ in range(100):
        x = np.r_[
            rng.uniform(-20, 180),
            rng.uniform(-3, 3),
            rng.uniform(-0.6, 0.6),
            12,
            0,
            kappa,
        ]
        value = np.asarray(m.hfun(x, [0, 0], [500, 0, 0, 0, 0, 15])).ravel()[4:8]
        physical = road_clearance(
            rectangle_vertices(pose_from_state(x, kappa)), kappa, 4.6
        )
        assert value.min() == pytest.approx(physical, abs=1e-10)
        np.testing.assert_allclose(m.hl[4:8], 0.2)
