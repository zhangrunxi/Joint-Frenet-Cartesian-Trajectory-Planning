# Randomized benchmark, overtaking task

This directory holds the run of the randomized scenario set on the
narrow-corridor overtaking task. The protocol was frozen before the run; see
`docs/RANDOM_BENCHMARK_PROTOCOL.md` for the scenario definition and the seed.

## Correction: completion is not passing

An earlier version of this README ranked the methods by `success`. On this
task that is the wrong metric: `success` only means that the 120 steps ran
without collision or road exit, so a run that followed the obstacle the whole
way counts as a success. The task asks the planner to pass. Scored by
`success and passed_obstacle`, the methods rank as follows:

| Method | Passed, all (40) | Passed, half width 4.6 m (15) | Passed, half width 8.5 m (25) |
| --- | ---: | ---: | ---: |
| This paper, AL-CiLQR | 16/40 | 4/15 | 12/25 |
| This paper, capsule condition | 15/40 | 7/15 | 8/25 |
| This paper, six states (SQP) | 10/40 | 4/15 | 6/25 |
| Reiter direct / lifted, ellipse | 11/40 | **0/15** | 11/25 |
| Reiter direct / lifted, three circles | 9/40 | **0/15** | 9/25 |
| This paper, three circles | 6/40 | **0/15** | 6/25 |
| Xing joint, keep-left corridor | 8/40 | 2/15 | 6/25 |
| Xing joint, always yield | 0/40 | 0/15 | 0/25 |

On the narrower road, the obstacle on the centre line leaves a 3.45 m gap. That
is below the minimum gap of the ellipse (5.47 m) and of the three covering
circles (3.65 m), and above that of the proposed geometry (3.18 m); see
`scripts/measure_minimum_gap.py`. On the wider road, the Reiter ellipse passes
more often than the proposed SQP formulation. The narrow-road question is
studied directly, with a gap sweep, in `../narrow/`.

The table below is the original `success` ranking. It is kept so that the
correction can be checked; **it does not measure passing**.

| Method | Completion (40 cases) | Failure mode |
| --- | ---: | --- |
| Reiter direct, ellipse | 37/40 (92.5%) | solver infeasible |
| Reiter lifted, ellipse | 37/40 (92.5%) | solver infeasible |
| Reiter direct, three circles | 34/40 (85.0%) | solver infeasible |
| Reiter lifted, three circles | 33/40 (82.5%) | solver infeasible |
| This paper, capsule condition | 33/40 (82.5%) | solver infeasible, one case runs without passing |
| This paper, AL-CiLQR | 32/40 (80.0%) | runs without passing (8) |
| This paper, three circles | 30/40 (75.0%) | solver infeasible |
| This paper, five circles | 30/40 (75.0%) | solver infeasible |
| This paper, three circles, Eq. 11-14 | 29/40 (72.5%) | solver infeasible, one case runs without passing |
| This paper, five circles, Eq. 11-14 | 28/40 (70.0%) | solver infeasible, two cases run without passing |
| This paper, pruned constraints | 28/40 (70.0%) | solver infeasible, four cases run without passing |
| **This paper, six states** | **27/40 (67.5%)** | solver infeasible (6), runs without passing (3) |
| This paper, nine states | 27/40 (67.5%) | as above |
| Reiter direct, ellipse, point bounds | 21/40 (52.5%) | runs without passing (16) |
| Xing joint, always yield | 14/40 (35.0%) | runs without passing (19) |
| This paper, frozen heading | 14/40 (35.0%) | runs without passing (16) |
| Xing joint, keep-left corridor | 10/40 (25.0%) | runs without passing (21) |
| Xing joint, keep-left corridor, point bounds | 8/40 (20.0%) | runs without passing (22) |

Two failure modes have to be kept apart:

- **A solver-infeasible failure** means the first frame had no feasible
  solution. This is a property of the constraints.
- **A run-without-passing failure** means the trajectory was legal and
  executed, but the ego vehicle never went past the obstacle. The obstacle
  clearance of those cases is 6.3 m to 35.2 m for the always-yield strategy,
  which is far enough away to show that the vehicle never approached the
  obstacle at all. This is a property of the strategy, not of the solver.

The two Xing strategies fail almost entirely in the second mode, and the
frozen-heading ablation of the manuscript's own model fails in the same way.
The proposed method fails mostly in the first mode.

The completion rate of the proposed method is **lower** than that of the
capsule-level variant of its own model. This is reported rather than
explained away.

## Reading the statistics

`summary.json` reports, per method, the completion rate and then a set of
distributions. **Each distribution carries the sample count `n` it was
computed from**, because the counts differ: a failed case contributes no
clearance and no solve time.

Solve times are only comparable within a backend. The proposed method runs on
acados/HPIPM, the Reiter reconstruction on CasADi/OSQP and the Xing
reconstruction on a self-written QP solver.

## What is in this directory

The per-method files carry the statistics of one method on one geometry; the
trajectories are dropped to keep the archive small. The fields kept are the
ones the manuscript and the response quote. `cases.json` is the scenario
definition, `protocol.json` the frozen protocol, and `summary.csv` /
`results.csv` the aggregated tables.

The run before the scenario-feasibility correction is in
`../random_benchmark_overtake_pre_correction/`, and the general task is in
`../random_benchmark/`.

## Re-running it

```bash
bash scripts/run_overtake_benchmark.sh --output results/recheck_overtake
.venv/bin/python scripts/summarize_random_benchmark.py \
    --input results/recheck_overtake --output results/recheck_overtake/summary.json
.venv/bin/python scripts/plot_random_benchmark.py \
    --input results/recheck_overtake --output results/recheck_overtake
```
