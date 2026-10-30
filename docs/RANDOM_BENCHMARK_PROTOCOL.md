# Protocol for Batch Experiments on Random Scenarios (Frozen Before the Runs)

This file was written before running `scripts/run_random_benchmark.py`, and serves to answer reviewer comment 3
("each comparison relies on only a single hand-picked scenario; the distributions of clearance, violation rate, solve time, and failure
rate should be reported over a large set of randomly sampled scenarios").

## 1. Scenario Generation

The scenario list is determined by the `numpy` generator with the fixed seed `20261030`, and the generation logic is written entirely in
`scripts/run_random_benchmark.py::build_cases`, so it can be independently recomputed. There are 40 scenarios in total, and in each
scenario all methods share the same road, the same obstacle prediction, the same target lateral position, and the same prediction horizon.

The sampled values:

| Parameter | Value set |
| --- | --- |
| Road half-width | 4.6 m / 8.5 m |
| Lateral target position | −2.0 / −1.0 / 0.0 / 1.0 / 2.0 m (relative to the road centerline, clipped into the road by the road half-width) |
| Road curvature | −1/40 / 0 / 1/60 / 1/35 m⁻¹ |
| Obstacle starting s | 25 / 35 / 50 m |
| Obstacle speed | 6 / 8 / 11 m/s |
| Initial speed | 8 / 12 / 15 m/s |

Each scenario has 120 steps, dt = 0.1 s, N = 40, 4 s prediction, 12 s closed loop. The vehicle is a bus of 6.50 m × 2.30 m,
with a wheelbase of 4.32 m and a rear overhang of 1.00 m.

## 2. Methods and Ablations

12 configurations are run in each scenario: 4 of this paper (hybrid SQP six-state, hybrid SQP nine-state,
circles SQP six-state, hybrid AL-CiLQR six-state), 4 of Reiter (Direct/Lifted × ellipse/circles), and 2 of Xing
(keep-left corridor, yield strategy ablation), plus 2 road-constraint ablations
(`reiter_direct_ellipse_point`, `xing_joint_corridor_point`, which use the reference-point road constraint of the original texts
instead of the vehicle-body contour constraint of this paper).

The yield configuration and the point configuration are strategy/constraint ablations of the same planner and are not counted as independent algorithms.

## 3. Recording and Failure Handling

The recorded fields include: whether 120 steps were completed, whether a collision or boundary violation occurred, the minimum obstacle clearance, the minimum road clearance,
the final progress relative to the obstacle, the mean/95th-percentile solve time, the maximum constraint violation, and the lateral acceleration and jerk.

- A solver abort, a constraint violation, a collision, or a boundary violation is always recorded as a failure; there is no retry, no discarding, and no re-running with changed parameters.
- Exceptions (raised exceptions) are likewise written into the `error` field of `results.json` and counted in the failure rate.
- The predictions and controls of aborted cases are written separately into `*_rejected.json`; no evidence is discarded.
- All methods run sequentially, with the numerical library thread count set to 1.

## 4. Statistical Definitions

Aggregated per method: success rate (completed 120 steps with no collision/boundary violation), abort rate, clearance distribution (median,
minimum, 5th percentile), solve-time distribution, and violation rate. The report explicitly distinguishes:

- **Clearance** and **solve time** are comparable across methods (same vehicle, same scenarios, same evaluator);
- **solve time** is not comparable across backends (native acados vs. Python wall time), and is reported grouped by backend only;
- the 40 scenarios are a **synthetic scenario set**, not independent driving trials, and do not constitute a statistical significance conclusion.

## 5. Relationship to the Existing Results

In the first trial run, the lateral target position was found to combine with the narrow-road half-width to produce degenerate scenarios located outside the road
(`lateral=5.5 m` with `half_width=4.6 m`). This problem was fixed before the formal run: the lateral target is now drawn
from a fixed offset set and clipped into the road by `|lateral| <= half_width - 1.35 m`
(half the vehicle width, 1.15 m, plus a road margin of 0.2 m). The protocol was re-frozen after the correction, the seed is unchanged,
and the scenario list changes accordingly. The discarded trial results are not included in any statistic.

This protocol does not replace the existing four hand-crafted scenarios and the width sweep. Those results are retained as qualitative examples; this protocol provides
distribution-level evidence. If the two disagree in direction, the distribution results take precedence and the difference must be reported faithfully.

## 6. Criteria for Scenario Feasibility

In some scenarios, the lateral acceleration limit of the bus is exceeded at the given initial speed and curvature
(`v_initial² · |κ| > 5.0 m/s²`), in which case no trajectory satisfying the constraints exists, regardless of the planner.
Such scenarios are listed separately in the results table as **infeasible scenario**, are not counted in the failure rate of any method,
but are retained in the table and annotated. The protocol does not delete them because of this: the screening criterion is physical feasibility, is independent of method performance,
and can be determined directly from the scenario parameters before the runs.

Aborts and failures are still counted in the method's own statistics according to their actual occurrence.

## 7. Scenario Types: General Traffic and Overtaking Tasks

The lateral targets drawn in the first round (task = general) are near the road centerline, so the resulting scenarios are mostly following or
lateral avoidance, and do not include overtaking tasks. This is not enough to answer the reviewer, because "can it overtake on a narrow road" is precisely
the specific scenario the reviewer is concerned about.

Therefore a second round (task = overtake) was added: the lateral target is fixed on the side opposite the obstacle position (n = 0), with
`|lateral| = half_width - 1.5 m`, so that every scenario requires the planner to detour around and overtake the obstacle.
The sampling of the remaining parameters (road width, curvature, obstacle position and speed, initial speed) is the same as in the first round,
with the seed unchanged.

The results of the two rounds are reported separately and are not merged into a single success rate. The general round characterizes the distributed behaviour of the methods in general traffic,
and the overtake round characterizes the distributed behaviour of the methods on tasks where overtaking is mandatory. Merging them would hide the difference.

## 8. Observation on the Difference in the Contour Ablation

A noticeable difference in contour offset between the frozen heading deviation and the heading deviation integrated along the contour appears only on narrow roads with large yaw states
(curvature 1/60 m⁻¹ and heading deviation 0.15 rad give a lateral offset difference of about 0.19 m between the front and rear corners of the vehicle body).
In scenarios where the lateral target is close to the road center and the contour constraint is inactive, their clearance statistics are almost identical.
The conclusion of the contour ablation should therefore be taken from the narrow-road overtaking round and should not be inferred from the distribution of the general traffic scenarios.

## 9. Pruning Ablation

A new `paper6_pruned_sqp` was added: following the hierarchical pruning strategy of Section 5.2 of the paper, only the nearest pair of collision components is kept at each prediction step (replaced by the corner circle when it falls into the redundant sub-capsule region), and the indices are fixed in parameter form within a single solve. All other settings are exactly the same as `paper6_hybrid_sqp`, so the difference between the two is precisely the effect of pruning.

Pruning makes the constraint non-smooth with respect to the discrete indices, which is the issue raised in reviewer comment 3.10. The report should give the safety margin, the success rate, and the solve time of both, and should faithfully report the cases where pruning misses a genuinely colliding component.
