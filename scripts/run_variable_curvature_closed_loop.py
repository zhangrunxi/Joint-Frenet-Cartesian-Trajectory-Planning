"""Common SLSQP direct-shooting closed-loop GEOMETRY ablation, not native solvers.

Run only after competing serial timing tasks finish. Defaults span the full road.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from run_variable_curvature_geometry import (
    HALF_WIDTH,
    MARGIN,
    VEHICLES,
    Road,
    circle_extent,
    fixed_padding,
    initial_checks,
    paper_extent,
    rectangle_extent,
    truth_extent,
)
from scipy.optimize import minimize

METHODS = ["rear_axle_point", "fixed_worst_curvature", "local_curvature_adaptive",
           "xing_rectangle_generalized", "paper_original_local", "paper_heading_difference"]


def rhs(x, u, road):
    s, n, alpha, v, a, k = x
    den = 1-n*road.curvature(s)
    sd = v*np.cos(alpha)/den
    return np.array([sd, v*np.sin(alpha), v*k-road.curvature(s)*sd, a, u[0], u[1]])


def integrate(x, u, road, dt):
    k1 = rhs(x, u, road)
    k2 = rhs(x+dt*k1/2, u, road)
    k3 = rhs(x+dt*k2/2, u, road)
    k4 = rhs(x+dt*k3, u, road)
    return x+dt*(k1+2*k2+2*k3+k4)/6


def rollout(initial, controls, road, dt):
    states = [np.asarray(initial)]
    for u in controls:
        states.append(integrate(states[-1], u, road, dt))
    return np.array(states)


def envelope(method, road, vehicle, x, padding):
    s, n, alpha = x[:3]
    if method == "rear_axle_point":
        return np.array([n, n])
    if method == "fixed_worst_curvature":
        return np.array([n-padding, n+padding])
    if method == "local_curvature_adaptive":
        return circle_extent(vehicle, road.curvature(s), n, alpha)
    if method == "xing_rectangle_generalized":
        return rectangle_extent(road, vehicle, s, n, alpha)
    return paper_extent(road, vehicle, s, n, alpha, original=method == "paper_original_local")


def residuals(states, controls, method, road, vehicle, padding):
    """All common physical constraints at every predicted node, including x0."""
    values = []
    for i, x in enumerate(states):
        s, n, alpha, v, a, k = x
        _jerk, kr = controls[min(i, len(controls)-1)]
        bounds = envelope(method, road, vehicle, x, padding)
        values.extend(HALF_WIDTH-MARGIN-np.abs(bounds))
        values.extend([1-n*road.curvature(s)-.1, .65-abs(alpha), v, 20-v,
                       a+3, 2.5-a, 5-abs(v*v*k),
                       5-abs(2*v*a*k+v*v*kr),
                       np.deg2rad(vehicle.max_steering_deg)-abs(np.arctan(vehicle.wheelbase*k)),
                       np.deg2rad(vehicle.max_steering_rate_degps)-abs(vehicle.wheelbase*kr/(1+(vehicle.wheelbase*k)**2)),
                       s+20, 100-s])
    return np.asarray(values)


def objective(states, controls, target, speed, dt):
    x = states[1:]
    return float(dt*np.sum(500*(x[:, 1]-target)**2+1000*x[:, 2]**2
                           +100*(x[:, 3]-speed)**2+x[:, 4]**2
                           +1000*x[:, 5]**2+controls[:, 0]**2+1e5*controls[:, 1]**2))


def run_case(vehicle_name, profile, method, horizon, steps, dt, maxiter, side):
    vehicle, road = VEHICLES[vehicle_name], Road(profile)
    check = initial_checks(road, vehicle)
    if not check["passed"]:
        raise ValueError("Shared initial state failed")
    initial = np.array([0., 0., 0., 5., 0., float(road.curvature(0))])
    state = initial.copy()
    padding = fixed_padding(vehicle)
    target = side*(HALF_WIDTH-MARGIN-vehicle.half_width-.05)
    warm = np.zeros((horizon, 2))
    all_states, all_controls, records = [state.copy()], [], []
    minimum_clearance = float("inf")
    stop_reason = "step_budget"
    for cycle in range(steps):
        cycle_state = state.copy()
        def trajectory(flat, cycle_state=cycle_state):
            return rollout(cycle_state, flat.reshape(horizon, 2), road, dt)
        def cost(flat):
            return objective(trajectory(flat), flat.reshape(horizon, 2), target, 5., dt)
        def constraints(flat):
            return residuals(trajectory(flat), flat.reshape(horizon, 2), method, road, vehicle, padding)
        result = minimize(cost, warm.ravel(), method="SLSQP", bounds=[(-4., 4.), (-.1, .1)]*horizon,
                          constraints={"type": "ineq", "fun": constraints},
                          options={"maxiter": maxiter, "ftol": 1e-6, "disp": False})
        worst = float(max(0., -constraints(result.x).min()))
        accepted = bool(result.success and np.isfinite(result.x).all() and worst <= 1e-5)
        records.append({"cycle": cycle, "status": int(result.status), "success": bool(result.success),
                        "accepted": accepted, "iterations": int(result.nit), "max_violation": worst,
                        "message": str(result.message)})
        if not accepted:
            stop_reason = "solver_rejected"
            break
        control = result.x.reshape(horizon, 2)[0]
        safe = True
        for _ in range(5):
            state = integrate(state, control, road, dt/5)
            dense = truth_extent(road, vehicle, *state[:3], count=1025)
            optimized = rectangle_extent(road, vehicle, *state[:3])
            clearance = HALF_WIDTH-np.max(np.abs(optimized))
            minimum_clearance = min(minimum_clearance, float(clearance))
            if clearance < MARGIN-1e-5 or np.max(np.abs(dense-optimized)) > 1e-5:
                safe = False
        all_states.append(state.copy())
        all_controls.append(control.copy())
        warm = np.vstack([result.x.reshape(horizon, 2)[1:], result.x.reshape(horizon, 2)[-1]])
        if not safe:
            stop_reason = "independent_safety_rejected"
            break
        if state[0] >= 60:
            stop_reason = "road_end"
            break
    completed = stop_reason == "road_end"
    return {"vehicle": vehicle_name, "profile": profile, "method": method, "side": side,
            "solver": "common_SLSQP_direct_shooting_geometry_ablation_not_native_Xing",
            "horizon": horizon, "dt": dt, "maxiter": maxiter, "step_budget": steps,
            "completed_full_60m": completed, "stop_reason": stop_reason,
            "minimum_full_body_clearance_m": minimum_clearance if np.isfinite(minimum_clearance) else None,
            "initial_checks": check, "target_n": target,
            "states": np.asarray(all_states).tolist(), "controls": np.asarray(all_controls).tolist(), "records": records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle", choices=VEHICLES, default="bus")
    parser.add_argument("--profile", choices=["enter", "exit", "s_bend"], default="enter")
    parser.add_argument("--methods", choices=METHODS, nargs="+", default=METHODS)
    parser.add_argument("--side", choices=[-1, 1], type=int, default=1)
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--dt", type=float, default=.2)
    parser.add_argument("--maxiter", type=int, default=150)
    args = parser.parse_args()
    if args.horizon < 2 or args.steps < 1 or args.dt <= 0 or args.maxiter < 1:
        parser.error("positive step/iteration budgets, dt, horizon>=2 required")
    args.output.mkdir(parents=True, exist_ok=False)
    for method in args.methods:
        result = run_case(args.vehicle, args.profile, method, args.horizon, args.steps, args.dt, args.maxiter, args.side)
        result["source_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__), Path(__file__).with_name("run_variable_curvature_geometry.py")]}
        (args.output/(method+".json")).write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        print(method, result["stop_reason"], flush=True)


if __name__ == "__main__":
    main()
