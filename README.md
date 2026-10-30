# Joint Frenet–Cartesian trajectory planning — comparison and ablation code

This repository contains the independent, equation-based reconstructions and the
controlled ablations used for the major revision of

> *Trajectory Planning for Autonomous Bus in Urban Environment via Joint
> Frenet–Cartesian Modeling* (ROBOT-D-26-01667).

It is released so that reviewers and later readers can inspect, re-run and
compare against the prior joint-frame formulations named in the manuscript
(Reiter et al. [20], Xing et al. [26]).

## What this is, and what it is not

Every planner here is an **independent reconstruction from the published
equations**, adapted to a common long-wheelbase bus scenario. None of them is
the original authors' deployment binary, and no author tuning is reproduced.
Where a paper does not disclose a number (for example Xing's Q/R weights, the
bus-footprint inflation, or the upstream decision logic), the choice made here
is recorded explicitly in the source and in `docs/`.

The claims this code is meant to support are deliberately narrow:

- the proposed capsule-plus-corner-circle geometry admits maneuvers that the
  tested circle and ellipse surrogates reject, in specific tight spaces;
- the redundant states of the lifted nine-state formulation cost a measurable
  amount of solve time relative to the six-state formulation of the same model;
- the published reference-point road bounds behave differently from the
  bus-contour road bounds on curved roads.

It does **not** establish that the proposed method is universally better than
joint-frame planners. In the scenarios we measured, a reconstructed Xing
keep-left corridor also completes the overtake.

## Layout

```
paper_comparison/     proposed method, Xing reconstruction, geometry evaluation
reiter2023/           Reiter reconstruction (direct elimination / lifted ODE)
scripts/              runners, verifiers and plotting
tests/                unit and acceptance tests
docs/                 protocol, comparison report and audit records
results/              archived summaries and figures
vendor/               third-party dependencies (git submodules)
```

## Setup

Python 3.11 is used. `vendor/` holds pinned third-party checkouts:

| path | upstream | commit |
| --- | --- | --- |
| `vendor/acados` | https://github.com/acados/acados | `a58e76ad` |
| `vendor/obstacle_avoidance` | https://github.com/RudolfReiter/obstacle_avoidance | `647f3582` |
| `vendor/vehicle_gym` | https://github.com/RudolfReiter/vehicle_gym | `8f2aa31f` |

```bash
git clone --recurse-submodules <this-repo>
cd <this-repo>
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements-comparison.lock
uv pip install --python .venv/bin/python --no-deps -e vendor/vehicle_gym
bash scripts/setup_acados.sh          # builds the acados shared library
```

`scripts/setup_acados.sh` builds acados with the settings used for the archived
results. On macOS the shared library is placed in `vendor/acados/lib`; the
runner shell scripts set `DYLD_LIBRARY_PATH` accordingly.

## Reproducing the archived results

```bash
# three-method comparison on wide and narrow roads
bash scripts/run_final_comparison.sh --steps 120 --output results/recheck_wide
bash scripts/run_final_comparison.sh --steps 120 --half-width 4.6 --lateral 2.6 \
  --output results/recheck_narrow

# randomized scenario set (frozen protocol, see docs/RANDOM_BENCHMARK_PROTOCOL.md)
bash scripts/run_random_benchmark.sh --output results/recheck_random --cases 40

# the overtaking-task subset of the same protocol
bash scripts/run_random_benchmark.sh --output results/recheck_random_overtake \
  --cases 40 --task overtake

# aggregate and plot the randomized results
.venv/bin/python scripts/summarize_random_benchmark.py \
  --input results/recheck_random --output results/recheck_random/summary.json
.venv/bin/python scripts/plot_random_benchmark.py \
  --input results/recheck_random --output results/recheck_random/summary.png

# contour-model accuracy as a function of the discretization count
.venv/bin/python scripts/run_ndisc_sweep.py --output results/recheck_ndisc

# collision-geometry conservatism, including the five-circle covering
.venv/bin/python scripts/measure_geometry_conservatism.py \
  --output results/recheck_geometry
.venv/bin/python scripts/measure_geometry_redundancy.py \
  --output results/recheck_redundancy

# verification of archived results against source hashes and re-simulation
.venv/bin/python scripts/verify_final_comparison.py \
  --inputs results/final_three_paper_wide_v2 results/final_three_paper_narrow_v2
```

`verify_final_comparison.py` re-checks the recorded source digests, replays the
saved trajectories with an independent geometric evaluator, and recomputes
clearance at the saved states. It does not re-time the solvers.

## Tests

```bash
.venv/bin/python -m pytest -q tests
```

The suite covers the geometry evaluators, the model's constraint construction,
the Xing reconstruction against independently derived equations, and the
acceptance rules used when archiving results.

## Two differences from the manuscript that matter when reading the numbers

**1. The collision constraint here is pairwise, the manuscript's is capsule-level.**
The manuscript states a sufficient condition on the continuous capsules,
`d_min = min_{c1 in Omega_1, c2 in Omega_2} ||c1 - c2||`, safe when
`d_min > r1 + r2`. Because the capsules are continuous, that single condition
separates the two bodies no matter where the closest pair lies. This
repository's default `PaperModel` instead checks every capsule/circle pair
individually (25 pairs), which is a strictly stronger requirement: at a
lateral separation of 2.6 m it reports +3.045 m where the capsule condition
reports +3.856 m.

Consequences when reading the archived numbers:

- the proposed method is evaluated under a **stricter** constraint than the one
  deployed in the manuscript, so its completion rate and clearances are
  **conservative lower bounds**;
- the relative ordering between methods is unaffected, because every method
  runs in the same environment;
- `PaperModel(obstacle_constraint="capsule")` and the
  `paper6_capsule_sqp` benchmark method reproduce the manuscript's own
  formulation.

**2. The pruned variant here is a discretisation of the manuscript's
pruning, not the manuscript's pruning itself.** The manuscript's Section 5.2
works on the continuous capsules and is therefore safe by construction; the
discretised variant in this repository keeps one component pair and loses the
guarantee that the capsules as a whole are separated. Its numbers therefore
describe a naive discretisation and must not be read as a property of the
manuscript's strategy. See `docs/FINAL_COMPARISON_REPORT.md`.

## Known limitations

These are stated in full in `docs/AUDIT_FINDINGS.md` and
`docs/FINAL_COMPARISON_REPORT.md`. The most important ones:

- The road-boundary projection in the proposed model is an approximation. In
  one archived width-sweep case the physical clearance was 0.194 m against a
  nominal 0.2 m margin. That shortfall is disclosed, not removed.
- The CiLQR implementation accepts on feasibility; it does not certify
  stationarity.
- Clearance is sampled at 0.02 s. This is not a continuous-time safety proof.
- Solve times across backends (acados native versus Python wall time) are not
  comparable and are never presented as method speed-ups.
- The randomized scenario set is a synthetic scenario set, not independent
  driving trials.

## Citation

If you use this code, please cite the manuscript above and the two prior
methods whose formulations are reconstructed here.
