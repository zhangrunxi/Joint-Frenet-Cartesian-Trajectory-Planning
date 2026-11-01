"""Variable curvature static feasible sets; deliberately NOT a closed loop benchmark."""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.integrate import cumulative_simpson
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize_scalar

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from paper_comparison.vehicle import VEHICLES

HALF_WIDTH, MARGIN, TOL = 3.0, 0.2, 1e-5


class Road:
    def __init__(self, profile):
        if profile not in {"enter", "exit", "s_bend", "straight"}:
            raise ValueError(profile)
        self.a, self.b = {"enter": (.015, .015), "exit": (.015, -.015),
                          "s_bend": (0., .03), "straight": (0., 0.)}[profile]
        grid = np.linspace(-40, 120, 8001)
        theta = self.heading(grid)
        xy = cumulative_simpson(np.column_stack([np.cos(theta), np.sin(theta)]), x=grid, axis=0, initial=0)
        self._xy = CubicSpline(grid, xy)
        self._origin = self._xy(0)

    def heading(self, s):
        s = np.asarray(s)
        return self.a*s + 10*self.b*(np.logaddexp((s-30)/10, -(s-30)/10)-np.logaddexp(-3., 3.))

    def curvature(self, s):
        return self.a + self.b*np.tanh((np.asarray(s)-30)/10)

    def derivative(self, s):
        return self.b/10 / np.cosh((np.asarray(s)-30)/10)**2

    def xy(self, s):
        return self._xy(s)-self._origin

    def project(self, points, near_s):
        points = np.asarray(points)
        s = np.broadcast_to(near_s, points.shape[:-1]).astype(float).copy()
        for _ in range(15):
            theta = self.heading(s)
            tangent = np.stack([np.cos(theta), np.sin(theta)], axis=-1)
            normal = np.stack([-np.sin(theta), np.cos(theta)], axis=-1)
            delta = points-self.xy(s)
            n = np.sum(delta*normal, axis=-1)
            den = 1-self.curvature(s)*n
            if np.any(den < .5):
                raise ValueError("Nonregular projection")
            step = np.sum(delta*tangent, axis=-1)/den
            s += step
            if np.max(np.abs(step)) < 1e-10:
                break
        if np.max(np.abs(step)) > 1e-8 or np.any((s < -40) | (s > 120)):
            raise ValueError("Projection failed or left road domain")
        normal = np.stack([-np.sin(self.heading(s)), np.cos(self.heading(s))], axis=-1)
        residual = points-self.xy(s)
        tangent = np.stack([np.cos(self.heading(s)), np.sin(self.heading(s))], axis=-1)
        if np.max(np.abs(np.sum(residual*tangent, axis=-1))) > 1e-8:
            raise ValueError("Projection orthogonality residual too large")
        return s, np.sum(residual*normal, axis=-1)


def corners(road, v, s, n, alpha):
    theta = float(road.heading(s))
    center = road.xy(s)+n*np.array([-np.sin(theta), np.cos(theta)])
    psi = theta+alpha
    rotation = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
    local = np.array([[-v.rear_overhang, -v.half_width], [v.front_extent, -v.half_width],
                      [v.front_extent, v.half_width], [-v.rear_overhang, v.half_width]])
    return center+local@rotation.T


def truth_extent(road, v, s, n, alpha, count=257):
    c = corners(road, v, s, n, alpha)
    t = np.linspace(0, 1, count)
    points = c[:, None, :]+t[None, :, None]*(np.roll(c, -1, axis=0)-c)[:, None, :]
    _, lateral = road.project(points.reshape(-1, 2), s)
    return np.array([lateral.min(), lateral.max()])


def rectangle_extent(road, v, s, n, alpha):
    """Generalized Xing footprint: true-road projection, optimize every edge extremum."""
    c = corners(road, v, s, n, alpha)
    values = []
    grid = np.linspace(0, 1, 17)
    for a, b in zip(c, np.roll(c, -1, axis=0)):
        _, offsets = road.project(a+grid[:, None]*(b-a), s)
        values.extend(offsets)
        for i in range(1, len(grid)-1):
            for sign in [1, -1]:
                if sign*offsets[i] <= min(sign*offsets[i-1], sign*offsets[i+1]) and np.ptp(offsets) > 1e-12:
                    def objective(t, sign=sign, a=a, b=b):
                        return sign*float(road.project(a+t*(b-a), s)[1])
                    optimum = minimize_scalar(objective, bounds=(grid[i-1], grid[i+1]), method="bounded", options={"xatol": 1e-10})
                    if not optimum.success:
                        raise ValueError("Edge extremum minimization failed")
                    values.append(sign*optimum.fun)
    return np.array([min(values), max(values)])


def circle_extent(v, k, n, alpha):
    """Exact circular whole rectangle extent, including edge interior minimum radius."""
    local = np.array([[-v.rear_overhang, -v.half_width], [v.front_extent, -v.half_width],
                      [v.front_extent, v.half_width], [-v.rear_overhang, v.half_width]])
    rotation = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])
    c = local@rotation.T+np.array([0, n])
    if abs(k) < 1e-12:
        return np.array([c[:, 1].min(), c[:, 1].max()])
    center = np.array([0, 1/k])
    b = np.roll(c, -1, axis=0)
    t = np.clip(np.sum((center-c)*(b-c), axis=1)/np.sum((b-c)**2, axis=1), 0, 1)
    minimum = np.min(np.linalg.norm(c+t[:, None]*(b-c)-center, axis=1))
    maximum = np.max(np.linalg.norm(c-center, axis=1))
    offsets = (1-abs(k)*np.array([minimum, maximum]))/k
    return np.sort(offsets)


def fixed_padding(v):
    return max(np.max(np.abs(circle_extent(v, k, n, alpha)-n))
               for k in [-.03, 0, .03] for n in [-2.5, 0, 2.5] for alpha in [-.08, 0, .08])


def paper_extent(road, v, s, n, alpha, segments=5, original=False):
    offsets = []
    for longitudinal in [-v.rear_overhang, v.front_extent]:
        for lateral in [-v.half_width, v.half_width]:
            ds = longitudinal*np.cos(alpha)-lateral*np.sin(alpha)
            u = ds*np.arange(segments)/segments
            difference = road.curvature(s)*u if original else road.heading(s+u)-road.heading(s)
            offsets.append(n+lateral/np.cos(alpha)+ds*np.mean(np.tan(alpha-difference)))
    return np.array([min(offsets), max(offsets)])


def initial_checks(road, v):
    s = np.linspace(0, 60, 601)
    speed = 5.
    k, kp = road.curvature(s), road.derivative(s)
    initial = truth_extent(road, v, 0, 0, 0)
    result = {"initial_full_body_clearance_m": float(HALF_WIDTH-np.max(np.abs(initial))),
              "max_reference_lateral_acceleration": float(np.max(speed**2*np.abs(k))),
              "max_reference_lateral_jerk": float(np.max(speed**3*np.abs(kp))),
              "max_reference_steering_deg": float(np.rad2deg(np.max(np.abs(np.arctan(v.wheelbase*k))))),
              "max_reference_steering_rate_degps": float(np.rad2deg(np.max(np.abs(v.wheelbase*speed*kp/(1+(v.wheelbase*k)**2)))))}
    result["passed"] = bool(result["initial_full_body_clearance_m"] >= MARGIN
        and result["max_reference_lateral_acceleration"] <= 5
        and result["max_reference_lateral_jerk"] <= 5
        and result["max_reference_steering_deg"] <= v.max_steering_deg
        and result["max_reference_steering_rate_degps"] <= v.max_steering_rate_degps)
    return result


def evaluate(output):
    output.mkdir(parents=True, exist_ok=False)
    summaries, checks = [], []
    fields = ["vehicle", "profile", "s", "n", "alpha", "method", "truth_min", "truth_max", "pred_min", "pred_max", "truth_safe", "pred_safe", "uncertain"]
    with (output/"poses.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for name, v in VEHICLES.items():
            padding = fixed_padding(v)
            for profile in ["enter", "exit", "s_bend"]:
                road = Road(profile)
                check = initial_checks(road, v)
                checks.append({"vehicle": name, "profile": profile, **check})
                if not check["passed"]:
                    raise ValueError("Common initial state or reference witness infeasible")
                totals = {}
                for s in np.arange(0, 61, 5):
                    for alpha in [-.08, 0., .08]:
                        for n in np.arange(-2.5, 2.51, .25):
                            truth = truth_extent(road, v, s, n, alpha)
                            methods = {"rear_axle_point": np.array([n, n]),
                                "fixed_worst_curvature": np.array([n-padding, n+padding]),
                                "local_curvature_adaptive": circle_extent(v, road.curvature(s), n, alpha),
                                "xing_rectangle_generalized": rectangle_extent(road, v, s, n, alpha),
                                "paper_heading_difference": paper_extent(road, v, s, n, alpha),
                                "paper_heading_difference_100": paper_extent(road, v, s, n, alpha, 100),
                                "paper_original_local": paper_extent(road, v, s, n, alpha, original=True)}
                            true_clear = HALF_WIDTH-np.max(np.abs(truth))-MARGIN
                            for method, bounds in methods.items():
                                pred_clear = HALF_WIDTH-np.max(np.abs(bounds))-MARGIN
                                uncertain = (abs(true_clear) <= TOL or abs(pred_clear) <= TOL
                                             or np.max(np.abs(methods["xing_rectangle_generalized"]-truth)) > TOL)
                                ts, ps = bool(true_clear >= 0), bool(pred_clear >= 0)
                                row = dict(zip(fields, [name, profile, s, n, alpha, method, *truth, *bounds, ts, ps, uncertain]))
                                writer.writerow(row)
                                stats = totals.setdefault(method, {"count": 0, "truth_feasible": 0, "predicted_feasible": 0, "false_safe": 0, "false_reject": 0, "uncertain": 0, "max_extent_error_m": 0.})
                                for key, value in {"count": 1, "truth_feasible": ts, "predicted_feasible": ps, "false_safe": ps and not ts and not uncertain, "false_reject": ts and not ps and not uncertain, "uncertain": uncertain}.items():
                                    stats[key] += int(value)
                                stats["max_extent_error_m"] = max(stats["max_extent_error_m"], float(np.max(np.abs(bounds-truth))))
                summaries.extend({"vehicle": name, "profile": profile, "method": method, **stats} for method, stats in totals.items())
                print(f"{name} {profile}: 819 static poses evaluated", flush=True)
    payload = {"experiment_type": "static_geometry_not_closed_loop", "initial_and_reference_checks": checks, "summary": summaries,
               "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__), next(q for q in (ROOT/"docs"/"VARIABLE_CURVATURE_PROTOCOL.md", ROOT/"VARIABLE_CURVATURE_PROTOCOL.md") if q.exists())]}}
    (output/"summary.json").write_text(json.dumps(payload, indent=2)+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"results/variable_curvature_geometry")
    evaluate(parser.parse_args().output)
