# Acceptance Report for the Major Revision Experiments

> Historical snapshot: this report retains the experiments and limitations of 2026-10-29. After the user provided
> the full Xing text on 2026-10-30, an equation reconstruction of the original text was added and the three-way
> comparison was completed; the latest conclusions and numbers are in
> [FINAL_COMPARISON_REPORT.md](FINAL_COMPARISON_REPORT.md).
> The judgements below about the missing full Xing text and the available proxy describe the prior state.

Date: 2026-10-29. Working environment: the current Mac, ARM64, Python 3.11.15, CasADi 3.8.1, acados + HPIPM/BLASFEO GENERIC, single-threaded. The paper PDF was not modified, and the business repository was not copied to the local machine.

## 1. Acceptance Conclusion

**The independent reconstruction of the method of this paper has been compared against Reiter [20]; a strict three-paper method comparison is not yet complete.** The full text of Xing [26] and the authors' implementation are still unavailable. The current third code base is an explicitly labelled Xing-inspired proxy, and it cannot be used to complete the reviewer-required validation against the original Xing method.

The main evidence completed so far:

1. Under the same bus, scenario, time horizon, and independent evaluator, 8 formal configurations × 4 scenarios completed 3,840 closed-loop cycles with no sampled collision or boundary violation.
2. In the narrow-road scenario, the hybrid geometry version completes the overtake while the circular approximation version follows the obstacle; the geometry ablation of the same six-state model shows this same difference, which supports studying the conservatism of the collision representation rather than merely comparing different solvers.
3. The 6/9-state ablation with the same physical model, the same cost, and the same constraints, together with the fixed-state replay at N=20/40/80, actually measures the overhead benefit of removing three redundant coordinates.
4. The independent AL-CiLQR and the SQP obtain close trajectories, but the Python CiLQR has a large tail latency, so real-time performance better than Reiter cannot be claimed on this basis.
5. The Xing-inspired proxy completed on the wide road and stopped on the narrow road because of a convergence condition. It is not the original Xing method, and its failure cannot be attributed to the Xing paper.

The seconds, clearances, and progress values in this report come from stored results; they are not the original tables of the paper and not the on-vehicle performance of the original authors. There are only a few deterministic synthetic scenarios, with no statistical significance and no general superiority conclusion.

## 2. Number of Algorithms and Configurations

| Category | Configuration | Purpose |
| --- | --- | --- |
| Reiter reconstruction | Direct 5-state / Lifted 8-state, each with ellipse / 3-circle, 4 in total | Adapted comparison of the equations of paper [20] |
| This paper reconstruction | 6-state hybrid SQP | Method comparison under a common solver |
| This paper ablation | 9-state hybrid SQP | Dimensionality ablation that only adds the three Cartesian states, not the Reiter algorithm |
| This paper ablation | 6-state circles SQP | Geometry ablation that only replaces the collision representation |
| This paper reconstruction | 6-state hybrid AL-CiLQR | Independent implementation of a paper-class solver, not an engineering deployment binary |
| Xing-inspired, unverified | algebraic joint-frame QP, circle / ellipse, 2 in total | Interface and feasibility exploration of a related architecture; it cannot replace the Xing [26] baseline |

Therefore there are **3 architecture families and 10 configurations, of which 8 are formal comparison configurations and 2 are exploratory proxy configurations**; this is not 10 papers, not 10 independent algorithms, and not three faithfully reproduced paper baselines.

## 3. Sources and Implementation Mapping

### Method of This Paper

- Paper: `Joint_Frenet_Cartesian_Trajectory_Planning.pdf` two levels above this directory; the SHA-256 is recorded on every run.
- Engineering reference: the `2.1` branch of `igoliao/planning` given by the user, commit `bee02d33c0cb50a5605db98b3cac44b816cdfecf`. It was read only as a shallow, no-checkout clone in `/home/runxi.zhang/work/academic/planning-reference-2.1` on beast; it was never run directly nor ported wholesale.
- Inspected files: `trajectory_gen/algorithm/cilqr_trajectory/vehicle_model.{cc,h}`, `cilqr_traj_data.{cc,h}`, `cilqr_traj_cost.{cc,h}`, `cilqr_traj_solver.cc`, `cilqr_traj_problem.cc`, and `proto/cilqr_traj_optimizer.proto`.
- State `[s,n,alpha,v,a,kappa]`, inputs `[jerk,kappa_dot]`; Frenet dynamics, Cartesian collision geometry; the road uses a discrete approximate projection of the four corners.
- `paper_comparison/model.py` implements the model, the hybrid geometry, and the SQP; `paper_comparison/cilqr.py` implements the AL/PHR penalties, the Riccati backward recursion, the regularization, and the backtracking.

The engineering reference uses Euler integration, circular collisions, and soft-barrier CiLQR, which are not fully consistent with the formulation in the paper. This is therefore "an independent re-implementation based primarily on the paper's formulas with the engineering code as reference", and it cannot be called a direct port of the production algorithm.

### Reiter [20]

- [Author manuscript](https://publications.syscop.de/Reiter2023a.pdf), [obstacle_avoidance](https://github.com/RudolfReiter/obstacle_avoidance), [vehicle_gym](https://github.com/RudolfReiter/vehicle_gym).
- Pinned commits: obstacle_avoidance `647f3582dad37d7e52bb7e7e560e7b084b0ddfb9`; vehicle_gym `8f2aa31f4d45660813c10fb276eb4b082b145afe`; acados `a58e76ad328d97aea180b55344e80eae346f8d15`.
- The current public dependencies lack the original `VehiclePlannerAcados20222`. `reiter2023/trajectory_planner_acados_20222.py` is an independent equation reconstruction; the old original-vehicle parameter experiments are retained in `results/reconstructed_sqp` and are not mixed into the bus comparison.
- The bus comparison is adapted in `BusReiter` in `scripts/run_paper_comparison.py`; the old reconstruction modules were not modified. The force/steering-rate, Direct/Lifted, and ellipse/circles structures are retained.

### Xing [26]

[Paper DOI](https://doi.org/10.1109/LRA.2022.3194330). Details on the current access to the full paper text, open metadata retrieval, related patents, and the proxy differences are in `sources/XING_ACCESS_AND_FIDELITY.md`. No public repository that can be confirmed to belong to that paper was found; the Cartesian/Frenet projects with the same name that were retrieved cannot therefore be taken as the original implementation.

## 4. Common Conditions and Adaptations That Must Not Be Hidden

Common conditions: N=40, dt=0.1 s, 4 s prediction horizon, 12 s closed loop; ego initial speed 12 m/s, target 15 m/s, obstacle rear axle `s=35+8t,n=0`. Straight road kappa=0; curve kappa=1/60 m^-1. Wide road half-width 8.5 m, lateral target 5.5 m; narrow road half-width 4.6 m, lateral target 2.6 m. The lateral overtaking direction and the target were provided by the experiment; the complete behavior decision layer was not reproduced.

Vehicle length 6.50 m, width 2.30 m, wheelbase 4.32 m, front overhang 1.18 m, rear overhang 1.00 m. The geometric centre is 2.25 m ahead of the rear axle; this corrects the 0.09 m offset of the symmetric-body-centre assumption of vehicle_gym. Acceleration [-3,2.5] m/s², speed [0,20] m/s, steering angle 29.32°, steering rate 24.55°/s; the road/collision margin parameters of the optimizer are both 0.2 m, and **the road constraint uses an approximate projection and does not guarantee an actual full-body clearance of 0.2 m**.

The following differences must be disclosed in the paper or in supplementary material:

- For Reiter, wind, rolling resistance, and aerodynamic drag are disabled to adapt to the bus, with a=F/m; the mass continues to use the library's 1160 kg as the scaling parameter for force, and **this is not a claim that the bus mass is 1160 kg**. Its force cost changes with this scaling, so the parameters must be re-tuned before real dynamic efficiency can be compared.
- Reiter's originally simplified road constraint is uniformly strengthened to the four-corner road approximation; its terminal still retains the original dynamics-related constraints, and no terminal obstacle/contour constraints were added. This paper checks collisions and the road at the terminal as well, and sets the terminal input to zero.
- This paper has hard bounds on jerk and lateral jerk; Reiter has no acceleration state, and equivalent jerk states were not silently added to it. Differences in comfort between different models cannot all be attributed to the frame representation.
- The cost of this paper retains curvature, curvature rate, acceleration, jerk, and lateral acceleration/jerk, while also referring to the engineering code to add progress, lateral position, heading, and speed tracking. The weights of this paper are stored in the `settings` of each case and **are not the original paper's deployment tuning**. The `settings` of the old Reiter cases are only the common experimental settings; the weights and jerk bounds of this paper in them do not take effect. The actual method configuration supplements are recorded in `results/effective_method_configurations.json`, and the original records were not rewritten.
- Both model families use RK4; the engineering reference originally used Euler. The terminal of this paper uses soft tracking rather than full-state equalities at an unspecified target time.
- The printed form of Eq.34e of the paper has a dimensional problem; here the physical steering-rate bound `delta_dot=L*kappa_dot/(1+(L*kappa)^2)` is used.
- The hybrid geometry is a central capsule plus four corner circles, with all 25 part pairs checked; the insufficiently specified part filtering is not implemented. Therefore no claim is made that the collision-pair pruning speed-up was reproduced.
- The acceptance condition is that the implementation returns status=0 and the maximum model residual is <=1e-4. The status=0 of CiLQR **only indicates that feasibility passed; it is not equivalent to the converged status of SQP or to the same optimality tolerance**; its inner loop may have stopped because of a budget or a line search, and not enough information was saved to certify stationarity independently. Such trajectories are referred to below as feasibility-accepted trajectories. Failed predictions are stored in `*_rejected.json`, and no silent retry leaves only the successful solution behind.

## 5. Main Numerical Results

The native times below are the mean acados `time_tot` with the first solve removed. It is the more comparable number under a common compiled backend and **is not the complete end-to-end time**. The wall time of the Reiter runner also contains a large amount of Python diagnostic work; it must not be turned directly into a speed-up ratio against the wall time of this paper, which covers only parameter setting/solve.

### Narrow road: half-width 4.6 m, lateral target 2.6 m

All configurations in the table completed 120 steps with no reported sampled collision/boundary violation. Progress is the final Frenet s; the final obstacle s=131 m. "Overtake" uses the separate criterion that the ego rear axle passes the obstacle rear axle by 6.5 m.

| Configuration | Straight progress m | Curve progress m | Straight/curve overtake | Straight/curve native mean ms |
| --- | ---: | ---: | --- | --- |
| Reiter Direct / ellipse | 121.377 | 120.817 | no / no | 5.213 / 7.355 |
| Reiter Lifted / ellipse | 121.377 | 120.817 | no / no | 7.793 / 12.306 |
| Reiter Direct / circles | 122.489 | 121.710 | no / no | 9.720 / 14.397 |
| Reiter Lifted / circles | 122.489 | 121.710 | no / no | 13.057 / 16.649 |
| This paper 6 / hybrid SQP | 177.342 | 183.820 | yes / yes | 3.124 / 4.902 |
| This paper 9 / hybrid SQP | 177.342 | 183.820 | yes / yes | 3.790 / 5.862 |
| This paper 6 / circles SQP | 122.093 | 121.308 | no / no | 8.678 / 14.352 |

For this paper 6/hybrid compared with Reiter Lifted/ellipse, the final s increases by 55.965 m and 63.004 m respectively in these cases; compared with its own circles ablation, the increase is about 55.25 m and 62.51 m. **This is a difference in the locally optimized trajectory under this initialization; it does not prove that other representations cannot overtake globally.** Safe following should not be labelled a solve failure.

The actual minimum obstacle clearance of this paper 6/hybrid with the true rectangle is 0.625 m / 0.616 m, and the minimum road clearance is 0.430 m / 0.258 m; this is not merely the capsule distance reported by the optimizer itself. On the curve the vehicle comes close to the road boundary, so map/localization error margins should be added later; the deterministic clearance must not be interpreted directly as a real driving safety guarantee.

### Wide road: half-width 8.5 m, lateral target 5.5 m

All 8 configurations completed the overtake. Reiter Lifted/ellipse has a final s=176.594/190.591 m, and this paper 6/hybrid has 177.127/191.096 m, with close trajectories; the progress increases by only about 0.53/0.50 m, which must not be written as a significant improvement in planning quality.

The native mean of Reiter Direct/ellipse is 0.754/0.864 ms, and that of this paper 6/hybrid is 1.839/2.154 ms; here the complete hybrid geometry of this paper is **slower**. The circles ablation of this paper is 1.351/1.478 ms, which shows that the full 25-pair hybrid check has additional overhead. The narrow-road results must not be used to deduce that "this paper is faster in all scenarios".

### CiLQR: Unfavourable Results Retained

| Scenario | Mean wall ms | P95 wall ms | Fraction of warm starts over 100 ms |
| --- | ---: | ---: | ---: |
| Wide straight | 200.143 | 1614.05 | 15.97% |
| Wide curve | 196.695 | 1635.36 | 15.97% |
| Narrow straight | 253.783 | 1636.37 | 36.97% |
| Narrow curve | 277.063 | 1813.34 | 34.45% |

CiLQR completed 120 steps in all four scenarios with no sampled collision/boundary violation, and the maximum accepted residual was 9.901e-5; the final s on the narrow road was 177.340/183.846 m, close to SQP. The Jacobian/cost parts are JIT-compiled by CasADi, but the Riccati recursion and the line search are Python/NumPy; there is no corresponding "native solver-only time". **The current implementation does not satisfy the real-time requirement of 100 ms per cycle, and it is not an on-vehicle CiLQR performance benchmark of the original work.**

### Xing-inspired Proxy: Only Exploratory Results Reported

On the wide road, both geometries and both roads completed 120 steps with progress approximately equal to Reiter; the circle wall means were 54.10/57.26 ms and the ellipse means 51.47/55.80 ms. On the narrow road, the circle case stopped after 8/6 steps on straight/curve, and the ellipse case stopped after 10/6 steps; there was no sampled collision before stopping, and the stopping reasons are in the per-cycle `qp_status` and the rejected files.

These are the numerical values and convergence limitations of the proxy, not benchmark results of the original Xing method; therefore the main conclusions do not use these times or failures to claim superiority over Xing.

## 6. Whether Dimensionality Is Actually Measured

At N=40 with 2-dimensional control, the number of uneliminated variables of direct multiple shooting, `nx*(N+1)+nu*N`, is:

| State representation | State dimension | Number of uneliminated variables |
| --- | ---: | ---: |
| Reiter Direct | 5 | 285 |
| This paper direct | 6 | 326 |
| Reiter Lifted | 8 | 408 |
| This paper lifted ablation | 9 | 449 |

This paper 6 removes 123 variables relative to its own 9-state ablation, i.e. 27.39%; relative to Reiter Lifted it removes 20.10%, but relative to Reiter Direct it is **14.39% more**. This is not the QP dimension after HPIPM condensing, and it is not a memory measurement.

`scripts/run_dimension_scaling.sh` replays the same physical state sequence in the four common scenarios: initial state plus the following 20 states, 3 repetitions, N=20/40/80, 240 warm solves per representation and time horizon, with cold solves listed separately. It does not re-sample the scenarios randomly, and repeated timing does not provide confidence intervals for trajectory success.

| N | 6-state warm-start native mean ms | 9-state warm-start native mean ms | 6-state time reduction | Warm accepts for 6/9 states |
| --- | ---: | ---: | ---: | --- |
| 20 | 2.459 | 2.922 | 15.85% | 240/240 |
| 40 | 8.594 | 10.295 | 16.52% | 240/240 |
| 80 | 29.309 | 35.065 | 16.42% | 237/240 |

The means include the duration of failed calls and are aggregated with equal weight over the four scenarios. In total there were 1,512 solves and 1,482 accepts: all 24 cold starts at N=80 failed, and additionally 6 warm starts on the narrow-road curve did not pass the status code. The warm phase of each representation at N=80 also contains 15 re-initializations with a zero-control rollout after a failure, and not all of them are warm starts that translate the previous solution; the recovery strategy is listed in the provenance. These post-failure cases must not be omitted when claiming stability over the whole range. Among the 741 comparisons accepted on both sides, the maximum physical state difference is 9.48e-7, the maximum first-jerk difference is 2.91e-6, and the first curvature-rate difference is 7.42e-10, which supports that this ablation preserves essentially the same physical solution.

There is evidence supporting "the quantifiable overhead of avoiding redundant Cartesian states", and no evidence supporting "dimensionality explosion". Adding three states corresponds to a linear increase of `3*(N+1)` variables with N; the dimensionality explosion wording should be deleted or narrowed.

## 7. Road-Width Sensitivity

Adding half-widths of 4.0, 4.3, 4.6, 4.9, 5.2, and 5.5 m, with the lateral reference synchronously taken as `half_width-2 m`; each width contains two roads and three configurations. It is a joint sensitivity experiment of "road width + reference position", not a single-factor experiment that only changes the geometrically feasible set while keeping the target exactly fixed.

36 cases actually completed 4,105 cycles, 34 cases completed 120 steps, 2 stopped because of a convergence condition, and there was no sampled collision/boundary violation in the executed intervals. The progress and clearance of incomplete runs must not be directly compared in magnitude with complete 12 s runs.

| Half-width m | This paper hybrid: straight / curve | This paper circles: straight / curve | Reiter Lifted circles: straight / curve |
| --- | --- | --- | --- |
| 4.0 | following / following | following / following | following / following |
| 4.3 | overtake / abort after 9 steps | following / following | following / following |
| 4.6 | overtake / overtake | following / following | following / following |
| 4.9 | overtake / overtake | overtake / overtake | overtake / abort after 16 steps |
| 5.2 | overtake / overtake | overtake / overtake | overtake / overtake |
| 5.5 | overtake / overtake | overtake / overtake | overtake / overtake |

On the curve at half-width 4.3 m, the failure status code of the hybrid of this paper is 4, with a maximum constraint residual of about 0.224; on the curve at half-width 4.9 m, Reiter reached 150 SQP iterations with status code 2, and although the feasibility residual is very small it was still stopped according to the established acceptance conditions. Failed predictions are retained, and no special initialization was appended to rescue a particular method alone.

The independent audit also found: in the narrow-road curve run of the circles of this paper at half-width 4.9 m, the actual minimum road clearance is **0.194173 m**, below the 0.2 m set by the optimizer, even though the model residual is only about 1.47e-8. This is the error of the road projection approximation, not a failure of the solver tolerance guarantee; "no sampled boundary violation" still holds, but "a guaranteed actual road margin of 0.2 m" does not. If this is used for a safety margin conclusion, exact road projection or a verified error buffer must be adopted and the runs repeated. This round retains the result and does not quietly modify the model to cover the shortfall.

Sweep explanation: the benefit of the hybrid geometry is concentrated in some tight-space ranges; when the road is wider all methods can overtake, and when it is narrower the hybrid geometry also follows the obstacle or fails to converge. It is therefore not permissible to claim that the hybrid geometry always succeeds, or to pinpoint a global feasibility threshold for some representation. Figures and raw scalars are in `results/width_sweep/plots/width_sweep.png` and `width_sweep.csv`.

### Solver-Independent Measurement of Geometric Conservatism

To avoid attributing all following differences to geometry, a geometric audit that does not run the planner was performed: two rectangles of the same direction of travel, 6.5 m long and 2.3 m wide, sampled at 0.05 m over the relative rear-axle positions `ds∈[-12,12] m, dn∈[-7,7] m`, for a total of 135,161 points. The analytical form of the true rectangle clearance is `hypot(max(abs(ds)-6.5,0),max(abs(dn)-2.3,0))`, with >=0.2 m as the safety reference.

| Collision representation | Fraction of safe grid points conservatively rejected | Minimum lateral separation at the same longitudinal position m |
| --- | ---: | ---: |
| True rectangle (analytical reference) | not applicable | 2.500 |
| This paper hybrid | 4.52% | 2.976 |
| 3-circle | 10.48% | 3.453 |
| Reiter ellipse surrogate | 25.92% | 5.274 |

None of the three approximations accepted an unsafe point on this grid; the minimum same-longitudinal lateral separation of the hybrid is about 0.476 m lower than that of the circles. This provides a solver-independent measurement of "less conservatism", but the hybrid itself is still more conservative than the exact rectangle and cannot be called exact rectangle collision detection. The proportion is an area sampling proportion within this finite grid and **is not a probability of driving success**; it covers only same-direction relative poses and cannot be extended to a continuous guarantee over all angles.

Script `scripts/measure_geometry_conservatism.py`; the results, the complete grid, and the figure are `results/geometry_conservatism/summary.csv`, `grid.npz`, and `geometry_conservatism.png` respectively.

### Reiter Input-Cost Sensitivity

The audit found that Reiter's original `0.001*F²` is equivalent to `1345.6*a²` under the present mass scaling, while the acceleration term of this paper is `1*a²`. Therefore 16 additional cases were run, changing only the Reiter force weight to `1/mass²` so that its squared-acceleration term is 1, with the other costs and constraints unchanged; this is a single-weight sensitivity, not a full matching of the jerk state, all costs, or the terminal constraints.

1,795 cycles were actually executed and 14 of 16 cases completed: on the wide road all 8 completed the overtake, with a final s=178.108/191.990 m, slightly greater than the 177.127/191.096 m of this paper. **The direction of the small wide-road progress difference changes with the weights and therefore cannot serve as evidence of general method superiority.**

For circles on the narrow road, the four Direct/Lifted cases all completed but still followed the obstacle, with a final s=121.954/121.156 m; for ellipse, the Lifted/straight and Direct/curve cases completed but followed the obstacle, while Direct/straight and Lifted/curve aborted after 57/58 steps respectively. This shows that the hybrid/circles overtake difference on the current narrow road is not caused only by this acceleration penalty, but other weights and local initialization effects still cannot be excluded, and this does not prove a global feasibility boundary.

Script `scripts/run_reiter_cost_sensitivity.py`; raw results `results/reiter_acceleration_cost_sensitivity/summary.csv`, complete log `logs/reiter_cost_sensitivity.log`. The raw formal results were not overwritten. At execution time the source snapshot `sources/run_reiter_cost_sensitivity_executed.py` matched the provenance; only import formatting was tidied afterwards.

### Exact Road Constraint Sensitivity (not the approximate constraint of the original paper)

To address the error found by the audit, `scripts/run_exact_road_sensitivity.py` was implemented: it replaces only the road constraint of this paper, using the true corner upper and lower bounds on straight roads; for circular arcs it uses the minimum distance from the circle centre to the edges of the whole rectangle and the maximum corner distance, handling the case where the circle centre falls inside the rectangle. The other dynamics, costs, collision geometry, and initialization are unchanged. Across three curvatures, 300 random states were checked against an independent road evaluator to 1e-10.

Under half-widths 4.3/4.6/4.9 m, two roads, and the two hybrid/circles configurations, 12 cases were run, actually executing 1,219 cycles, of which 10 completed. The 4.3 m hybrid on the curve aborted after 5 steps and the 4.6 m circles on the curve aborted after 14 steps; the rest completed. The 4.6 m hybrid still completed the overtake on straight/curve with s=177.342/183.820 m; the 4.9 m circles on the curve completed the overtake, and the minimum physical road clearance changed from the 0.194173 m of the original approximate constraint to **0.200000029 m**.

An exact discrete node constraint is still not a continuous-time margin guarantee: on the 4.3 m hybrid straight road, the minimum clearance over sub-step sampling is 0.199988772 m, about 11 micrometres short of 0.2 m; this is related to inter-node propagation and numerical error. Strict continuous-time assurance would additionally require tightening the margin or continuous collision/boundary verification. This sensitivity version also introduces additional non-smooth constraints and convergence failures, so it must not be assumed to be a fixed and certified version of the original paper's implementation.

Complete results and figures: `results/exact_road_sensitivity/summary.csv` and `results/exact_road_sensitivity/plots/trajectories.png`; neither the raw formal results nor the width sweep results were overwritten.

## 8. Evaluation, Files, and Re-running

- The independent evaluator `paper_comparison/evaluation.py` uses the true rectangle, uses SAT/edge-segment distance for obstacles, and uses the whole-body edge clearance against a straight line or an annulus for the road; it includes points inside the curved road edge, not only the four corners.
- The actual closed-loop propagation is checked every 0.02 s; this is not a continuous-time collision-free proof. The comfort RMS recorded per control cycle is also not a 0.02 s continuous integration metric.
- Raw formal results: `results/paper_comparison_wide_final`, `results/paper_comparison_narrow_final`; the case JSON saves the executed closed-loop states/controls and the per-cycle diagnostics, and does not contain the complete prediction horizon of every accepted solution; on failure the rejected prediction is stored separately. The summaries are `summary.csv` / `summary.json`.
- Dimensionality results: `results/dimension_scaling/measurements.csv`, `paired_controls.csv`, `summary.csv`; failed calls are not deleted.
- Dimensionality figure: `results/dimension_scaling/dimension_scaling.png`, including the accept counts of the cold/warm phases; the plotting script `scripts/plot_dimension_scaling.py` only reads the results.
- Exploratory proxy: `results/xing_inspired_wide_v2`, `results/xing_inspired_narrow_v2`; the v1 failure record is retained as it was.
- Figures: `results/paper_comparison_plots` covers the 8 configurations; `results/three_family_comparison_plots` merges the 10 configurations and marks the unverified proxy and the aborted endpoints; each has trajectory, Cartesian trajectory, and performance plots and a full-precision `comparison.csv`.
- `results/comparison_source_audit.json` verifies the source code hash of the formal runs. The runner snapshots before and after the addition of the proxy are saved in `sources/run_paper_comparison_pre_proxy.py` and `sources/run_paper_comparison_with_proxy_v2.py` respectively, matching the original provenance; the old provenance was not rewritten.
- The dependencies are currently locked in `requirements-comparison.lock`; the local editable packages therein must be installed separately with the commands below. Read `RECONSTRUCTION.md` before building, and do not overwrite existing result directories.

```bash
cd experiments/reiter20
uv pip install --python .venv/bin/python -r requirements-comparison.lock
uv pip install --python .venv/bin/python --no-deps -e vendor/vehicle_gym
uv pip install --python .venv/bin/python --no-deps -e vendor/acados/interfaces/acados_template

# By default only the 8 formal configurations are run; a new directory avoids overwriting.
bash scripts/run_paper_comparison.sh --steps 120 --output results/new_wide
bash scripts/run_paper_comparison.sh --steps 120 --half-width 4.6 --lateral 2.6 --output results/new_narrow

# The exploratory proxy must be selected explicitly and is not mixed into the default formal configurations.
bash scripts/run_paper_comparison.sh --steps 120 \
  --methods xing_inspired_proxy_circles xing_inspired_proxy_ellipse \
  --output results/new_proxy_wide

bash scripts/run_dimension_scaling.sh --output results/new_dimension_scaling
bash scripts/run_width_sweep.sh results/new_width_sweep
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/run_reiter_cost_sensitivity.py --output results/new_cost_sensitivity
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/run_exact_road_sensitivity.py --output results/new_exact_road
ACADOS_SOURCE_DIR="$PWD/vendor/acados" \
DYLD_LIBRARY_PATH="$PWD/vendor/acados/lib" \
  .venv/bin/python scripts/measure_geometry_conservatism.py --output results/new_geometry_grid

.venv/bin/python scripts/plot_paper_comparison.py \
  --inputs results/paper_comparison_wide_final results/paper_comparison_narrow_final \
  results/xing_inspired_wide_v2 results/xing_inspired_narrow_v2 \
  --merge-by-width --cartesian --output results/new_plots
```

### Verification Status

The final run `REITER_RUN_INTEGRATION=1 ... python -m pytest -q tests`: **172 passed**, no skips; complete log `logs/comparison_final_tests_with_exact_road.log`. It includes acados compilation integration, the independent rectangle evaluation, hybrid geometry coverage, model derivatives, the physical steering rate, the algebraic frame coupling, the proxy QP, the refusal of iteration failures, the regression of the road approximation margin shortfall, and the independent check of the exact road sensitivity model. The CasADi/NumPy compatibility and acados deprecation warnings are kept in the log; they are not equivalent to a numerical verification failure. The earlier logs recorded 168 and 169 passes respectively, and all are retained.

Archive verification: `.venv/bin/python scripts/verify_comparison_results.py` does not start a solver; it checks the summaries against the per-case data, the acceptance/rejection conditions, the trajectory lengths, and the hash of the run source code, covering **104 cases, 11,469 executed closed-loop cycles, and 1,512 dimensionality replay calls**; the output is `results/comparison_archive_manifest.json`. The 104 cases include the proxy, the width sweep, the input-cost study, and the exact road sensitivity, and they cannot all be called formal paper baselines. The early pilot is not counted in these totals. The independent read-only audit record is in `AUDIT_FINDINGS.md`; the disclosure corrections made after the audit do not eliminate the model limitations.

## 9. Boundaries of the Conclusions That Can Be Used to Answer the Reviewer

Can be written: the addition of an equation reconstruction baseline of Reiter; a fair bus adaptation; the separation of the hybrid geometry and the state dimensionality into two ablations; explicit counts and a measured time-horizon extension; the reporting of the additional overhead of the complex geometry and of initialization failures; and the acknowledgement that the joint-frame idea is not proposed for the first time by this paper.

Cannot be written: that the original Xing baseline has been completed; that it is significantly faster than prior art in all scenarios; that the Python CiLQR reaches the real-time performance of the original on-vehicle system; that Reiter necessarily needs more states; that following on a narrow road is equivalent to being unsafe; or that general superiority is proven by a single set of deterministic results.

**The full Xing text must still be obtained and verified at the equation level before the three-way comparison can be submitted as a complete response.** This time no other algorithm was passed off as Xing in order to complete the three-way result, and the contribution wording of the paper was not modified; these must be revised again in combination with the post-verification results.

The supplementary source verification is archived in `sources/xing_access_followup/README.md`: Semantic Scholar / OpenAIRE did not provide an open PDF, the arXiv title query returned no result this time, and the new leads from OpenReview and DBLP are limited by access verification. No new formula-level material was obtained; this item is a real dependency of the strict three-way acceptance, not an unfinished solver run. After legally obtaining the full Xing text, replace the proxy according to the item-by-item verification checklist in that directory and re-run.
