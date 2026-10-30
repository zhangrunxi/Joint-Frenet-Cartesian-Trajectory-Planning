"""Independently validate final archives and replay Xing with scipy DOP853."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from run_paper_comparison import finite_json

from paper_comparison.evaluation import evaluate_sample


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project(pc, kappa, near_s):
    speed, x, y, psi = pc
    if not kappa:
        return np.array([x, y, psi, speed, x, y, psi])
    center = np.array([0, 1 / kappa])
    radial = np.array([x, y]) - center
    initial_radial = -center
    angle = np.arctan2(
        initial_radial[0] * radial[1] - initial_radial[1] * radial[0],
        initial_radial @ radial,
    )
    angle += 2 * np.pi * np.round((kappa * near_s - angle) / (2 * np.pi))
    s = angle / kappa
    n = np.sign(kappa) * (abs(1 / kappa) - np.linalg.norm(radial))
    return np.array([s, n, psi - angle, speed, x, y, psi])


def obstacle_pose(time, kappa):
    s = 35 + 8 * time
    if not kappa:
        return [s, 0, 0]
    return [np.sin(kappa * s) / kappa, (1 - np.cos(kappa * s)) / kappa, kappa * s]


def replay_xing(case, prediction_file):
    states = np.asarray(case["original_order_states"])
    stored = np.asarray(case["states"])
    U = np.asarray(case["controls"])
    np.testing.assert_allclose(stored, states[:, [4, 5, 6, 0, 1, 2, 3]], atol=1e-12)
    dt, kappa = case["settings"]["dt"], case["curvature"]
    minimum_road, minimum_obs, max_replay_error = np.inf, np.inf, 0.0
    maximum_predicted_alat = 0.0
    previous = np.array([np.arctan(4.32 * kappa), 0.0])
    predictions = json.loads(prediction_file.read_text())["accepted_predictions"]
    assert len(predictions) == len(U)
    for j, u in enumerate(U):
        assert (
            np.max(np.abs(u - previous) - np.array([np.deg2rad(24.55), 4]) * dt) <= 1e-4
        )
        assert -3 - 1e-4 <= u[1] <= 2.5 + 1e-4
        assert abs(u[0]) <= np.deg2rad(29.32) + 1e-4

        def rhs(_t, pc, control=u):
            v, _, _, psi = pc
            return [
                control[1],
                v * np.cos(psi),
                v * np.sin(psi),
                v * np.tan(control[0]) / 4.32,
            ]

        trajectory = solve_ivp(
            rhs,
            (0, dt),
            states[j, :4],
            method="DOP853",
            rtol=2e-12,
            atol=2e-13,
            dense_output=True,
        )
        assert trajectory.success
        expected = project(trajectory.y[:, -1], kappa, stored[j, 0])
        max_replay_error = max(
            max_replay_error, float(np.max(np.abs(expected - stored[j + 1])))
        )
        np.testing.assert_allclose(expected, stored[j + 1], atol=1e-8, rtol=1e-11)
        for sub in range(1, 6):
            state = project(trajectory.sol(sub * dt / 5), kappa, stored[j, 0])
            check = evaluate_sample(
                state,
                obstacle_pose((j + sub / 5) * dt, kappa),
                kappa,
                case["half_width"],
            )
            minimum_road = min(minimum_road, check["road_clearance_m"])
            minimum_obs = min(minimum_obs, check["obstacle_clearance_m"])
        pred = predictions[j]
        np.testing.assert_allclose(pred["previous_applied_input"], previous, atol=1e-12)
        np.testing.assert_allclose(pred["controls"][0], u, atol=1e-12)
        assert len(pred["original_order_states"]) == case["N"] + 1
        assert len(pred["controls"]) == case["N"]
        predicted_states = np.array(pred["original_order_states"])
        predicted_inputs = np.array(pred["controls"])
        assert (
            np.isfinite(predicted_states).all() and np.isfinite(predicted_inputs).all()
        )
        start_v = predicted_states[:-1, 0]
        end_v = start_v + dt * predicted_inputs[:, 1]
        predicted_alat = (
            np.maximum(start_v**2, end_v**2)
            * np.abs(np.tan(predicted_inputs[:, 0]))
            / 4.32
        )
        maximum_predicted_alat = max(
            maximum_predicted_alat, float(predicted_alat.max())
        )
        assert predicted_alat.max() <= case["settings"]["max_lat_acc"] + 1e-4
        previous = u
    if len(U):
        assert abs(minimum_road - case["min_road_clearance_m"]) < 1e-8
        assert abs(minimum_obs - case["min_obstacle_clearance_m"]) < 1e-8
    return {
        "case": case["name"],
        "replayed_cycles": len(U),
        "max_state_replay_error": max_replay_error,
        "max_predicted_interval_abs_lateral_acceleration_mps2": maximum_predicted_alat,
        "replayed_min_road_clearance_m": minimum_road,
        "replayed_min_obstacle_clearance_m": minimum_obs,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", nargs="+", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    files, cases, cycles, replays = {}, 0, 0, []
    for directory in args.inputs:
        directory = directory.resolve()
        provenance = json.loads((directory / "provenance.json").read_text())
        for relative, expected in provenance["sha256"].items():
            assert sha(ROOT / relative) == expected, relative
        paper = ROOT / "sources" / "Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf"
        if not paper.exists():
            paper = (
                ROOT.parents[1]
                / "Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf"
            )
        if provenance.get("xing_paper_sha256"):
            assert paper.exists(), "Recorded paper digest but no local PDF"
            if provenance.get("xing_paper_sha256"):
            assert paper.exists(), "Recorded paper digest but no local PDF"
            assert sha(paper) == provenance["xing_paper_sha256"]
        summaries = json.loads((directory / "summary.json").read_text())
        for summary in summaries:
            path = directory / (summary["name"] + ".json")
            case = json.loads(path.read_text())
            for k, value in summary.items():
                assert case[k] == value, (path, k)
            n = case["completed_steps"]
            assert len(case["states"]) == n + 1 and len(case["controls"]) == n
            assert len(case["cycles"]) in (n, n + 1)
            assert all(c["accepted"] for c in case["cycles"][:n])
            assert all(
                c["accepted"] == (c["status"] == 0 and c["max_violation"] <= 1e-4)
                for c in case["cycles"]
            )
            if case["success"]:
                assert (
                    n == case["steps_requested"] and not case["collision_or_road_exit"]
                )
            elif case["cycles"] and not case["cycles"][-1]["accepted"]:
                assert (directory / (case["name"] + "_rejected.json")).exists()
            if case["method"].startswith("xing_joint"):
                replays.append(
                    replay_xing(case, directory / (case["name"] + "_predictions.json"))
                )
            cases += 1
            cycles += n
        for path in directory.glob("*.json"):
            files[str(path.relative_to(ROOT))] = sha(path)
        for path in directory.glob("*.csv"):
            files[str(path.relative_to(ROOT))] = sha(path)
    report = {
        "verified_cases": cases,
        "verified_executed_cycles": cycles,
        "xing_independent_ode_replays": replays,
        "sha256": files,
        "scope": "Archive consistency, source identity, exact Xing execution replay and 0.02s physical safety sample recomputation. Not a continuous-time certificate or author-code identity.",
    }
    args.output.write_text(
        json.dumps(finite_json(report), indent=2, allow_nan=False) + "\n"
    )
    print(
        f"Verified {cases} cases / {cycles} cycles; independently replayed {sum(r['replayed_cycles'] for r in replays)} Xing cycles."
    )


if __name__ == "__main__":
    main()
