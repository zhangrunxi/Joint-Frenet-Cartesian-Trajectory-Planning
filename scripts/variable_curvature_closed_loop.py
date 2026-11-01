"""Minimal shared road/model seam for a future variable-curvature closed loop.

No solver is compiled or run on import. Original and extended contours stay separate.
The exact Xing rectangular envelope remains a numeric evaluator, not a codegen-ready
constraint; this module does not claim a completed fair closed-loop comparison.
"""
import sys
from pathlib import Path
from uuid import uuid4

import casadi as ca
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from run_variable_curvature_geometry import (
    HALF_WIDTH,
    MARGIN,
    Road,
    rectangle_extent,
    truth_extent,
)

from paper_comparison.model import PaperModel, approximate_road_offsets
from paper_comparison.vehicle import GEOMETRY


class SharedRoad(Road):
    """Analytic heading/curvature and matched numeric/symbolic centerline geometry."""
    half_width = HALF_WIDTH

    def __init__(self, profile):
        super().__init__(profile)
        grid = np.linspace(-40, 120, 8001)
        # CasADi B-spline interpolation provides differentiable codegen geometry.
        xy = self.xy(grid)
        self._cx = ca.interpolant("road_x_" + uuid4().hex, "bspline", [grid], xy[:, 0])
        self._cy = ca.interpolant("road_y_" + uuid4().hex, "bspline", [grid], xy[:, 1])

    def curvature(self, s):
        if isinstance(s, (ca.MX, ca.SX, ca.DM)):
            return self.a + self.b*ca.tanh((s-30)/10)
        return super().curvature(s)

    def symbolic_heading(self, s):
        # The entire supported domain has |(s-30)/10| <= 9, so cosh is safe.
        return self.a*s+10*self.b*(ca.log(ca.cosh((s-30)/10))-np.log(np.cosh(3.)))

    def pose(self, s, n, alpha):
        theta = self.symbolic_heading(s)
        return ca.vertcat(self._cx(s)-n*ca.sin(theta), self._cy(s)+n*ca.cos(theta), theta+alpha)

    def numeric_pose(self, state):
        return np.asarray(self.pose(*np.asarray(state)[:3]), dtype=float).ravel()


def heading_difference_offsets(x, road, segments=5):
    """Explicit method change; original approximate_road_offsets remains untouched."""
    s, n, alpha = x[0], x[1], x[2]
    values = []
    for longitudinal in [-GEOMETRY.rear_overhang, GEOMETRY.front_extent]:
        for lateral in [-GEOMETRY.half_width, GEOMETRY.half_width]:
            ds = longitudinal*ca.cos(alpha)-lateral*ca.sin(alpha)
            offset = n+lateral/ca.cos(alpha)
            for i in range(segments):
                u = ds*i/segments
                angle = alpha-(road.symbolic_heading(s+u)-road.symbolic_heading(s))
                offset += ds/segments*ca.tan(angle)
            values.append(offset)
    return ca.vertcat(*values)


class VariableCurvaturePaperModel(PaperModel):
    """Keep shared dynamics/cost/physical limits; select original or extended road bound."""
    def __init__(self, road, settings, *, contour="original_local", **kwargs):
        if contour not in {"original_local", "heading_difference", "reference_point"}:
            raise ValueError(contour)
        self.contour = contour
        super().__init__(road, settings, **kwargs)

    def initial(self, speed=5.0):
        k = float(self.road.curvature(0.))
        sixth = k if self.kappa_as_state else k*self.vehicle.length
        return self.augment(np.array([0., 0., 0., speed, 0., sixth]))

    def _symbolic(self):
        super()._symbolic()
        if self.contour == "original_local":
            offsets = approximate_road_offsets(self.x, self.road, segments=self.n_disc)
        elif self.contour == "heading_difference":
            offsets = heading_difference_offsets(self.x, self.road, self.n_disc)
        else:
            offsets = ca.repmat(self.x[1], 4, 1)
        self.h = ca.vertcat(self.h[:4], offsets, self.h[8:])
        self.hfun = ca.Function("variable_h", [self.x, self.u, self.p], [self.h])
        self.g = ca.vertcat(self.hl-self.h, self.h-self.hu,
                           self.xl-self.x[self.state_indices], self.x[self.state_indices]-self.xu)
        self.gfun = ca.Function("variable_g", [self.x, self.u, self.p], [self.g])


def independent_road_evaluation(road, vehicle, state):
    """Rigid-body dense boundary and optimized edge diagnostics, independent of contour."""
    s, n, alpha = np.asarray(state)[:3]
    sampled = truth_extent(road, vehicle, s, n, alpha, count=1025)
    optimized = rectangle_extent(road, vehicle, s, n, alpha)
    return {"clearance_m": float(HALF_WIDTH-np.max(np.abs(sampled))),
            "optimized_clearance_m": float(HALF_WIDTH-np.max(np.abs(optimized))),
            "sampling_discrepancy_m": float(np.max(np.abs(sampled-optimized))),
            "margin_satisfied": bool(HALF_WIDTH-np.max(np.abs(optimized)) >= MARGIN)}
