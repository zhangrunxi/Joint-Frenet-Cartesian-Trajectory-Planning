# Randomized benchmark, ablation of the proposed method

This directory holds a separate run of the variants of the manuscript's own
model on the same randomized scenario set as `../random_benchmark/` (same
seed, same 40 cases). It is a second, independent run: the completion rates of
the methods that also appear in `../random_benchmark/` agree with that run
case for case.

## What the numbers say

| Variant | Curve (27) | All (40) | Median solve, curve (ms) |
| --- | ---: | ---: | ---: |
| Three circles, N = 3 | 25/27 (92.6%) | 37/40 (92.5%) | 1.37 |
| Three circles | 25/27 (92.6%) | 37/40 (92.5%) | 1.37 |
| Five circles | 24/27 (88.9%) | 36/40 (90.0%) | 1.86 |
| **This paper, six states (hybrid)** | **23/27 (85.2%)** | **35/40 (87.5%)** | **1.81** |
| Frozen heading | 23/27 (85.2%) | 35/40 (87.5%) | 1.81 |
| Without curvature term | 23/27 (85.2%) | 35/40 (87.5%) | 1.69 |
| Five circles, N = 5 | 22/27 (81.5%) | 34/40 (85.0%) | 1.87 |
| Capsule condition | 18/27 (66.7%) | 29/40 (72.5%) | 1.00 |
| Pruned constraints | 14/27 (51.9%) | 22/40 (55.0%) | 41.09 |

Solve times are medians over the successful cases only. Pruning loses
completion rate: of its 13 failures on the curved road, 10 are constraint
violations (0.004 m to 1.05 m), 2 are first-frame infeasible, and 1 ran
without violation but did not complete. Violations below 1e-6 are numerical
zero and are not counted as violations. The ordering
is not monotonic in how conservative the geometry is; the circle models,
which are more conservative, complete more cases than the proposed hybrid
model. This is reported rather than explained away.

## Failure modes

`completed_steps` separates the failure modes:

- `0`: the first frame was infeasible.
- `> 0` with `max_constraint_violation > 0`: a constraint was violated.
- `> 0` with no violation: the run was legal but did not complete.
- a negative `min_obstacle_clearance_m`: a real collision.

## Re-running it

```bash
bash scripts/run_random_benchmark.sh --output results/recheck_ablation --cases 40
```
