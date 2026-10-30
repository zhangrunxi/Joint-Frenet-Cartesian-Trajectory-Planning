# Six- versus nine-state formulation, serial timing

This directory holds the six- and nine-state formulations of the proposed
method on the randomized scenario set, run **one at a time**. The timings in
`../random_benchmark/` were taken while other methods ran in parallel on the
same machine, so they are affected by CPU contention; this run is the one the
overhead figure is quoted from.

| Population | Six states (ms) | Nine states (ms) | Difference |
| --- | ---: | ---: | ---: |
| Curved road, successful cases (23) | 1.84 | 2.17 | +0.33 ms (+17.7%) |
| All 40 cases, successful cases (35) | 1.80 | 2.15 | +0.36 ms (+19.9%) |

The timing is the median of `mean_solve_wall_ms` over the successful cases.
The two formulations complete the same cases (23/27 on the curved road and
35/40 overall), so the nine-state formulation costs 18-20% more solve time and
buys no completion rate. The dedicated scaling run in `../dimension_scaling/`
gives 18.6-19.7%, which agrees.
