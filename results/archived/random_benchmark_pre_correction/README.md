# Pre-correction run of the randomized benchmark

This directory holds the **first** run of the randomized scenario set, before
the scenario generator was corrected. It is kept deliberately, because the
difference between this run and the corrected one is the evidence behind a
correction we report to the reviewer.

## What was wrong with the generator

The generator sampled the initial speed and the road curvature independently.
For 13 of the 40 cases this produced an initial state whose lateral
acceleration `v0^2 * |kappa|` exceeds the bound of Eq. (34f) of the
manuscript. On such a case every optimizer reports infeasibility in the very
first frame, including ours, while the always-yield strategy of the Xing
reconstruction does not have to satisfy that bound and completes 11 of the 13.

Reporting the 40 cases as one completion rate therefore measures the scenario
generator rather than the planners. Over the full set the rates are 95.0%
against 62.5%; restricted to the 27 cases with a feasible initial state they
are 100% against 92.6%. The apparent 32.5-percentage-point gap is an artifact
of the generator, and a 7.4-point gap is what the scenarios actually show.

The corrected generator enforces the initial feasibility condition with a
margin of 0.5 m/s^2, and on the overtaking task it enforces that the ego
vehicle is faster than the obstacle by at least 1.5 m/s. Fourteen of the
original overtaking cases did not satisfy that condition, and for them the
overtake was dynamically impossible.

## What is in this directory

The per-method files carry the statistics of one method on one geometry; the
trajectories are dropped to keep the archive small. The fields kept are the
ones the manuscript and the response quote. `cases.json` is the scenario
definition, `protocol.json` the frozen protocol, and `summary.csv` /
`results.csv` the aggregated tables.

The corresponding overtaking run is in `../random_benchmark_overtake_pre_correction`.

## Re-running it

The corrected generator is the one in `scripts/run_random_benchmark.py`. To
reproduce the run stored here rather than the corrected one, the generator
has to be reverted to the unconstrained sampling; the run itself is otherwise
identical.
