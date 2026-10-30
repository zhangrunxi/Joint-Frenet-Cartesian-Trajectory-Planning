# Randomized benchmark, general task

This directory holds the run of the randomized scenario set on the general
task. The protocol was frozen before the run; see
`docs/RANDOM_BENCHMARK_PROTOCOL.md` for the scenario definition and the seed.

## What the numbers say

The completion rate is **not** favourable to the method of the manuscript, and
the archive is published so that this is checkable:

| Method | Completion |
| --- | ---: |
| Xing joint, always yield | 39/40 (97.5%) |
| Reiter direct, three circles | 37/40 (92.5%) |
| This paper, three circles | 37/40 (92.5%) |
| This paper, six states | 35/40 (87.5%) |
| Reiter lifted, ellipse | 28/40 (70.0%) |
| Xing joint, keep-left corridor | 21/40 (52.5%) |

The completion rate alone is a misleading summary, because the methods trade
it against clearance. The median minimum obstacle clearance of the successful
cases is **4.87 m** for the always-yield strategy and **18.01 m** for the
proposed method: the always-yield strategy buys its completion rate by not
going around the obstacle, and the proposed method buys clearance by going
around it. Both numbers are in the archive.

## Reading the statistics

`summary.json` reports, per method, the completion rate and then a set of
distributions. **Each distribution carries the sample count `n` it was
computed from**, because the counts differ: a failed case contributes no
clearance and no solve time, so for the proposed method the completion rate is
over 40 cases while the clearance and timing distributions are over 37.

Solve times are only comparable within a backend. The proposed method runs on
acados/HPIPM, the Reiter reconstruction on CasADi/OSQP and the Xing
reconstruction on a self-written QP solver; the medians are 1.90 ms, 5.06 ms
and 31.53 ms respectively, which is a difference in magnitude and not a
speed-up ratio. The same applies to the failed cases of the pruned variant,
whose solver iterates until it fails and takes 55-81 ms.

## What is in this directory

The per-method files carry the statistics of one method on one geometry; the
trajectories are dropped to keep the archive small. The fields kept are the
ones the manuscript and the response quote. `cases.json` is the scenario
definition, `protocol.json` the frozen protocol, and `summary.csv` /
`results.csv` the aggregated tables.

The run before the scenario-feasibility correction is in
`../random_benchmark_pre_correction/`, and the overtaking task is in
`../random_benchmark_overtake/`.

## Re-running it

```bash
bash scripts/run_random_benchmark.sh --output results/recheck_random --cases 40
.venv/bin/python scripts/summarize_random_benchmark.py \
    --input results/recheck_random --output results/recheck_random/summary.json
.venv/bin/python scripts/plot_random_benchmark.py \
    --input results/recheck_random --output results/recheck_random
```
