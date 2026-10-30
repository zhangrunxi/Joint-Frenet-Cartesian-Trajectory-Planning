"""Opt-in generated-code tests: set REITER_RUN_INTEGRATION=1."""

import os

import numpy as np
import pytest
from vehiclegym.planner.trajectory_planner_base import VehicleObstacleModel

from reiter2023 import (
    CircularRoad,
    PlannerOptions,
    SampledRoad,
    VehiclePlannerAcados20222,
)
from reiter2023.trajectory_planner_acados_20222 import paper_vehicle

pytestmark = pytest.mark.skipif(
    os.environ.get("REITER_RUN_INTEGRATION") != "1",
    reason="Opt-in native acados compilation",
)


@pytest.mark.parametrize("lifted", [False, True])
@pytest.mark.parametrize(
    "model", [VehicleObstacleModel.ELIPSE, VehicleObstacleModel.CIRCLES]
)
def test_compiled_plan_satisfies_constraints_and_rejects_impossible_initial_state(
    tmp_path, lifted, model
):
    road = CircularRoad(1 / 60)
    vehicle = paper_vehicle()
    planner = VehiclePlannerAcados20222(
        vehicle,
        road,
        PlannerOptions(
            n_nodes=21,
            use_lifting=lifted,
            obstacle_model=model,
            nlp_solver="SQP",
            velocity_set=12,
            build_directory=str(tmp_path / "solver"),
        ),
        [vehicle],
    )
    predictions = np.array(
        [[road.numeric_pose([40 + 8 * t, 0, 0]) for t in planner.time_grid]]
    )
    planner.set_obstacle_predictions(predictions)
    state = [0, 0, 0, 12, np.arctan(vehicle.length / 60)]
    planner.set_states(state)
    assert planner.solve() == 0
    assert planner.diagnostics["max_violation"] < 1e-5
    assert planner.diagnostics["coordinate_consistency_error"] < 1e-5
    planner.reset_solver()
    # Stage-zero nonlinear acceleration cannot satisfy the physical bound here.
    planner.set_states([0, 0, 0, 35, 0.3])
    status = planner.solve()
    assert status != 0 or planner.diagnostics["max_violation"] > 1e-4


def test_current_vehicle_gym_track_adapter_compiles(tmp_path):
    from vehiclegym.road.track import Track

    road = SampledRoad(Track.synthetic(bend=0.002, length=200))
    planner = VehiclePlannerAcados20222(
        road=road,
        planner_options=PlannerOptions(
            n_nodes=11,
            use_lifting=False,
            nlp_solver="SQP",
            velocity_set=12,
            build_directory=str(tmp_path / "sampled"),
        ),
    )
    planner.set_states([0, 0, 0, 12, np.arctan(3.4 * 0.002)])
    assert planner.solve() == 0
    assert planner.diagnostics["max_violation"] < 1e-5
