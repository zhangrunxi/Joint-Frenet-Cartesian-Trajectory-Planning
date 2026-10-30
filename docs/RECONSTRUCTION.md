# Reconstruction Record for VehiclePlannerAcados20222

## Conclusion and Scope

Combining paper [20] with the current `vehicle_gym` code, `reiter2023.VehiclePlannerAcados20222` has been implemented so that it can be compiled and run natively on this Apple Silicon machine. It uses the actual acados / HPIPM backend, not an IPOPT substitute. This implementation is an independent reconstruction based on the paper; it is not the original `20222` class provided by the authors, and it does not make the original `comparisons_short.py` run again directly.

Rolling planning tests have been completed for both representations:

- Direct Elimination: state `[s, n, alpha, v, delta]`, with the Cartesian pose computed by the inverse Frenet transformation.
- Lifted ODE: the above states plus `[x, y, phi]`, integrating the Cartesian ODE with a consistent initial value at the same time.
- Obstacles are constrained in Cartesian coordinates, supporting ellipse and covering circles; both representations use the same vehicle, cost, boundaries, and prediction input.
- Separating hyperplane, Conventional Frenet, the complete random scenario set of the paper, and the original opponent vehicles' NMPC are not yet implemented.

The current results are sufficient to support continued development of the comparison platform, but not to claim a complete reproduction of the statistical results, run times, or safety guarantees of the original paper.

## Mapping Between Code and Formulas

| Paper content | Implementation location | Current handling |
| --- | --- | --- |
| Eqs. (1), (2): force, steering rate, lateral acceleration | `_create_model` in `reiter2023/trajectory_planner_acados_20222.py` | Includes wind speed, wind direction, aerodynamic drag, and rolling resistance; the physical control units are N and rad/s |
| Eq. (9): Frenet ODE | Same as above | `s_dot=v*cos(alpha)/(1-n*kappa)`; uses the rear-axle lateral acceleration |
| Eqs. (12c), (13): augmented initial value and Cartesian ODE | `augment_state`, `_create_model` | At each replan, a consistent initial value is constructed from the measured five states; the prediction coordinate-consistency error is recorded separately |
| Eq. (14): ellipse constraint | `ellipse_constraint` in `reiter2023/geometry.py` | The covering ellipse of the obstacle rectangle plus the covering-circle radius of the ego vehicle; uses the same rear-axle-to-body-centre offset |
| Eq. (15): covering-circle constraint | `covering_circles`, `_create_model` | Reuses the circle-centre construction of the current `vehicle_gym`; checks that the number of circles is sufficient to cover the body; imposes a distance constraint on each pair of circles |
| Eq. (17): state and control costs | `_create_solver`, `set_states` | The stage weights use the values of Table 4, multiplied by `dt` once by acados; because of the `1/2` convention of acados, the matrices are multiplied by 2; the terminal cost is not multiplied by `dt` |
| Eqs. (18), (19): multiple shooting | `_create_solver` | RK4, Gauss–Newton, PARTIAL_CONDENSING_HPIPM, with the condensing horizon at half the number of intervals |
| Constraint check after the solve | `evaluate_solution` | The initial value, the RK4 defect, the state/control bounds, the lateral acceleration, the obstacles, and the coordinate consistency are recorded separately |

`set_states`, `set_obstacles`, `warm_start`, `solve`, `reset_solver`, `get_statistics`, and some of the output interfaces of the current `Planner` are retained, but this is not a fully compatible replacement for the old project. `set_obstacle_predictions` takes an explicit array of Cartesian rear-axle predictions `(n_obstacles, n_nodes, 3)`, and that interface is preferred to avoid coordinate ambiguity.

The current `SampledRoad` adapts `vehicle_gym.Track` / `Road`; the geometric spline, heading, and curvature of a sampled road are not strictly analytically consistent, and extrapolation outside the domain is not yet supported as a formal scenario. The valid reference-line domain should be fixed and its derivative consistency checked separately before the formal comparison. The main experiment of this work uses analytical straight lines/arcs, which excludes that factor.

## Actual Test Results

### Single RTI: The Original Failure Record Is Retained

Directory: `results/reconstructed_full/`; log: `logs/reconstruction_full.log`.

By default one RTI per cycle, a centerline reference, and 80 requested cycles: the 4 straight-road configurations passed, while the 4 curve-road configurations were rejected by the checker in the initial stage. The solver status was 0, but the RK4 dynamics defect of the predicted trajectories was about `2.3e-4` to `3.4e-4`, above the current `1e-4` threshold.

This does not prove that the original method is infeasible and is not equivalent to a physical collision; it shows that a solver status of 0 cannot replace the nonlinear feasibility check. The threshold was not relaxed to mask the failure this time.

### Full SQP: All 8 Configurations Pass

Directory: `results/reconstructed_sqp/`; log: `logs/reconstruction_sqp.log`.

Each configuration was rolled for 120 cycles, 12 seconds in total, for 960 cycles across the 8 configurations. There were no solve failures and no detected rectangle collisions. The times below are only the solver diagnostics of this machine; they exclude code generation, compilation, initialization, and the Python evaluation, and are not intended for direct comparison with the deployment data of the paper.

| Representation | Obstacle model | Road | Mean solve time (ms) | P95 (ms) |
| --- | --- | --- | ---: | ---: |
| Direct | Ellipse | Straight road | 1.482 | 1.820 |
| Direct | Ellipse | Curve | 10.804 | 18.500 |
| Direct | Three circles | Straight road | 2.215 | 2.631 |
| Direct | Three circles | Curve | 12.909 | 20.954 |
| Lifted | Ellipse | Straight road | 1.741 | 2.099 |
| Lifted | Ellipse | Curve | 11.686 | 21.665 |
| Lifted | Three circles | Straight road | 2.579 | 3.119 |
| Lifted | Three circles | Curve | 14.902 | 23.708 |

The per-case and per-cycle data are authoritative in JSON and are aggregated into `summary.csv`. The maximum predicted RK4 defect in the SQP test is less than `2e-13`; the maximum component error of the coordinate consistency on the augmented curve is less than `3e-7`. These small errors do not constitute a safety guarantee.

The trajectory plot of the ellipse model is `results/reconstructed_sqp/trajectories.png`. The trajectories of Direct and Lifted essentially coincide, as expected from the description of the same physical model.

### Explicit RTI Feasibility Refinement: All 8 Configurations Pass

Directory: `results/reconstructed_rti_refined_run/`; log: `logs/reconstruction_rti_refined_run.log`.

The same 8 × 120 cycles were run, but with at most 5 RTI iterations allowed per cycle, continuing only when the residual exceeds the threshold, with at most 3 actually used. All calls, states, cumulative solve times, and residuals are written into the per-cycle record.

This is not the single-RTI configuration of the paper, and its time must not be passed off as the time of a single call. The maximum component error of the coordinate consistency on the augmented curve is about `1.55e-3`, clearly higher than in the full SQP; subsequent geometric evaluation should continue to use the independent true contour instead of checking only the internal constraints of the optimizer.

### Automated Checks

`logs/reconstruction_all_tests.log`: 15 passed, including 7 checks of the reconstructed model/geometry, 5 integration checks of the locally generated code, and 3 checks of the existing run tooling.

The integration checks include: compilation and feasibility of the four model combinations, the identification of a failure after deliberately setting an infeasible lateral-acceleration initial value, and the integration of the sampled road of the current `vehicle_gym.Track`. The QP errors in the infeasibility test are the expected rejection path; the upstream interface also emits several deprecation warnings, and these warnings must not be treated as evidence of no risk.

The final regression also includes the 21 upstream tests, for a total of 36 passed, see `logs/final_regression_tests.log`. The local launcher script additionally performs a 5-cycle curve compilation and run check, see `results/reconstructed_wrapper_check/summary.json`. The SHA-256 records of the source code, the dependency commits, and the main result files are in `results/reconstruction_provenance.json`.

## Differences from the Paper and the Work Still to Be Done

1. The main test is a constant-speed obstacle with a straight road and a circular arc of curvature `1/60 m^-1`; it is not the paper's experiment of 500 random runs per class, three opponent vehicles, and 20 seconds. The ego initial speed is 12 m/s, the target speed 25 m/s, the obstacle starts at `s=35 m` with speed 8 m/s, and the lateral reference for the detour is 4 m; these are settings used for verification, not a claim of recovering the original cases.
2. There are 40 control intervals, 41 state nodes, and `dt=0.1 s`. Table 4 of the paper writes `N=40`, while the node/interval counts of the old scripts differ; this is recorded explicitly here rather than assumed to be fully consistent.
3. The vehicle wheelbase, mass, body dimensions, physical boundaries, and cost values mainly follow Tables 3 and 4; the drag coefficients come from the current default parameter values of `vehicle_gym`, and the original text does not provide enough numerical data to confirm that they are fully identical. The wind speed is the paper's 20 m/s and the wind direction is 0.
4. The current implementation uses hard constraints and has no slack. How the slack-weight interface of the public script combines with the missing solver cannot be verified, so it was not inferred to be the authors' code. A Frenet validity-domain constraint `1-n*kappa>=0.1` was added to prevent singularities; this constraint must not be given to only one side in later ablations.
5. The road boundary constraint acts on the Frenet point of the rear axle, not on the complete vehicle contour. The "no collision" of the tests refers to the Cartesian rectangles of the ego vehicle and the obstacles, and cannot be interpreted as the vehicle body never crossing the road boundary.
6. The collision check tests the rectangle every 0.02 seconds, which is still not a continuous-time collision proof; the emitted `min_center_distance_m` is the distance between body centres, not a clearance. The inflation of the ellipse semi-axes follows the scheme of the paper, and it is not additionally claimed to be the exact Minkowski sum under arbitrary geometric conditions.
7. Lifted relates the two coordinate systems only through a consistent initial value and ODE propagation, without adding coordinate algebraic equalities at each node; the numerical drift is recorded separately. The simulation reconstructs the augmented initial value from the measured five states at every cycle, and this semantics must not be omitted.
8. The full SQP and the multi-RTI variant are diagnostic variants. The warm start, the exact initialization, the constraint softening, and the failure handling of the original single-RTI formulation still need to be checked, and the parameters must not be tuned backwards to match the timing.
9. acados uses the currently pinned commit with a GENERIC build of BLASFEO/HPIPM and OpenMP disabled, which differs from the paper's 2023 environment and Intel hardware; the fact that Direct is currently faster does not directly prove that the lower-dimensional scheme is generally superior to Lifted.

## Re-run Commands

Execute in `experiments/reiter20/`. The result directory uses fixed file names; use a new `--output` directory when the old records must be preserved.

```bash
# Full SQP verification, two states × two models × two roads.
bash scripts/run_reconstruction.sh --steps 120 --solver SQP \
  --lateral-reference 4 --output "$PWD/results/reconstructed_sqp_repeat"

# Original threshold check of the single RTI; this setting may exit with 1.
bash scripts/run_reconstruction.sh --steps 80 \
  --output "$PWD/results/reconstructed_rti_repeat"

# Explicitly allow RTI feasibility refinement; not to be treated as the original single-RTI result.
bash scripts/run_reconstruction.sh --steps 120 --lateral-reference 4 \
  --rti-iterations 5 --output "$PWD/results/reconstructed_rti_refined_repeat"

# Includes the test with locally generated C code.
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" \
REITER_RUN_INTEGRATION=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python -m pytest -q tests

# Only redraws the existing results.
.venv/bin/python scripts/plot_reconstruction.py results/reconstructed_sqp
```

The reconstruction dependencies can be installed by running `bash scripts/setup_acados.sh`. It requires that `.venv` already exists in this directory and that the Xcode Command Line Tools are available; all installations are written into the experiment directory. The pinned acados commit is `a58e76ad328d97aea180b55344e80eae346f8d15`; the Python package snapshot is `requirements-reconstruction.lock`, and CasADi was changed from the previous 3.8.2 to 3.8.1 because of the acados compatibility requirement. The old `requirements.lock` is retained as the environment record of the first portable check.

Two environment problems were encountered during the first build, and the logs are retained: CMake could not find Ninja inside the virtual environment, so an absolute path was specified instead; and macOS could not resolve `@rpath/libhpipm.dylib`, which was solved by the launcher script setting `DYLD_LIBRARY_PATH` before the Python process starts. The system dynamic library paths and the upstream source code were not modified.

## How to Use This for Subsequent Comparisons

It is recommended to first broaden the test coverage that has no random ambiguity, and then to decide whether to adopt the reconstruction as a formal baseline. The comparison table should be labelled `Our reimplementation of Reiter et al. [20]` and should disclose the solver settings; it must not be labelled `official implementation`. The next stage can incorporate the model of this paper, but the actual source code and the CiLQR behaviour of the engineering algorithm of this paper must still be confirmed before designing unified physical constraints and an independent geometric evaluator.
