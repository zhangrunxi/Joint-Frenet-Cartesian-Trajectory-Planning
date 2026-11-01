# Reviewer Quick Start

This guide is the shortest reproducible path for checking the comparison
reported in the revision. The repository contains independent,
equation-based reconstructions of Xing [26] and Reiter [20]; it does not
contain the original authors' deployment binaries.

## 1. Install the pinned environment

The commands below assume macOS or Linux, Python 3.11, and `uv`:

```bash
git clone --recurse-submodules \
  https://github.com/zhangrunxi/Joint-Frenet-Cartesian-Trajectory-Planning.git
cd Joint-Frenet-Cartesian-Trajectory-Planning
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements-comparison.lock
uv pip install --python .venv/bin/python --no-deps -e vendor/vehicle_gym
bash scripts/setup_acados.sh
```

The runners set the numerical-library thread counts to one and configure the
acados shared-library path. Outputs should be written to a new directory;
the scripts refuse to overwrite existing result files.

## 2. Current comparison (2026-11-01 settings)

Every method runs under one shared setting:

- shared lateral-ramp initial guess (`BENCH_INITIAL_GUESS=ramp`);
- an iterate that reaches the iteration cap is accepted only if its nonlinear
  constraint violation is at most 1e-4 (`BENCH_ACCEPT_MAXITER=1`);
- the node-0 constraint repair of the proposed SQP;
- contour discretisation `N_DISC=5` for gap/overtake and `N_DISC=10` for curb,
  passed explicitly to both the proposed and the Reiter contour;
- Xing: OSQP `solved inaccurate` iterates are accepted
  (`BENCH_XING_ACCEPT_INACCURATE=1`), the keep-left corridor is frozen within a
  frame (`BENCH_XING_FREEZE_CORRIDOR=1`), and residuals are recomputed after
  freezing so that the cached corridor cannot hide a penetration.

The archived per-case records are in `results/current/`. Reproduce the tables
from them with

```bash
.venv/bin/python scripts/summarize_current_results.py --root results/current
```

or re-run a task from scratch (one command per vehicle and task):

```bash
export BENCH_INITIAL_GUESS=ramp BENCH_ACCEPT_MAXITER=1 \
       BENCH_XING_ACCEPT_INACCURATE=1 BENCH_XING_FREEZE_CORRIDOR=1
BENCH_VEHICLE=bus BENCH_N_DISC=5  bash scripts/run_random_benchmark.sh \
  --task gap2      --output results/reviewer/v2/bus_gap2/part000
BENCH_VEHICLE=bus BENCH_N_DISC=5  bash scripts/run_random_benchmark.sh \
  --task overtake2 --output results/reviewer/v2/bus_overtake2/part000
BENCH_VEHICLE=bus BENCH_N_DISC=10 bash scripts/run_random_benchmark.sh \
  --task curb      --output results/reviewer/v2/bus_curb/part000
# repeat with BENCH_VEHICLE=suv
.venv/bin/python scripts/summarize_current_results.py --root results/reviewer
```

A gap or overtake pass requires completion, passing the obstacle, and at least
0.19 m exact-rectangle obstacle clearance; following an obstacle is not
counted. A curb strike is an exact-rectangle road clearance below zero.

### Narrow gap (`gap2`)

| Method | Bus (160) | SUV (136) |
| --- | ---: | ---: |
| Proposed hybrid | **108** | **92** |
| Reiter circles (direct / lifted) | 70 / 70 | 58 / 59 |
| Proposed circles5 / circles | 69 / 68 | 59 / 58 |
| Proposed capsule only | 36 (23 collisions) | 60 |
| Xing corridor | 23 | 44 |
| Reiter ellipse (direct / lifted) | 10 / 9 | 11 / 11 |

### Overtaking (`overtake2`, 48 cases per vehicle)

| Method | Bus | SUV |
| --- | ---: | ---: |
| Proposed hybrid | **44** | **46** |
| Proposed capsule only | 35 (6 collisions) | 45 |
| Circle models (proposed and Reiter) | 30 | 34 |
| Reiter ellipse (direct / lifted) | 0 / 0 | 2 / 1 |
| Xing corridor | 0 | 2 |

### Curb (`curb`, `N_DISC=10`)

SUV cases 0, 1, 8 and 9 are excluded: their exact body is already closer than
the prescribed 0.2 m margin at t = 0.

| Method | Bus: completed / strikes / min clearance | SUV: completed / strikes / min clearance |
| --- | --- | --- |
| Proposed deformed contour | 32/32 / 0 / 0.182 m | 28/28 / 0 / 0.188 m |
| Proposed, frozen heading | 0/32 / 32 / -0.047 m | 0/28 / 28 / -0.025 m |
| Reiter reference-point bound | 0/32 / 32 / -0.030 m | 0/28 / 28 / -0.024 m |
| Xing reference-point bound | 0/32 / 32 / -0.044 m | 0/28 / 28 / -0.026 m |
| Reiter static inflated bound | 32/32 / 0 / 0.161 m | 28/28 / 0 / 0.185 m |
| Xing full-body corridor | 0/32 / 0 / - | 0/28 / 0 / - |

The static inflated bound is also strike-free, so the proposed contour is not
claimed to dominate full-body road-boundary formulations.

## 3. Ablation and state dimension

- `results/current/v2/ablation_general` (bus, 40 general cases): hybrid 38/40,
  circles 39/40, capsule 32/40, pruned 23/40. On these wide scenarios the
  geometry models are close; this set is not evidence of an advantage.
- `results/current/v2/serial9_*`: six-state and nine-state versions of the same
  model both complete 38/40; median native solve time 1.93 ms vs 2.16 ms.
- `results/current/v2/dimension_scaling`: warm native time of the nine-state
  version is 18-23% higher at N = 20/40/60.

## 4. Geometry-only checks

- `results/current/v2/contour_per_pose_corrected`: per-pose contour error
  against `N_DISC` (`scripts/run_per_pose_contour_accuracy.py`).
- `results/current/variable_curvature_geometry_final`: static variable-curvature
  audit (`scripts/run_variable_curvature_geometry.py`, protocol in
  `docs/VARIABLE_CURVATURE_PROTOCOL.md`). Of 2,980 feasible poses out of 4,914,
  the proposed contour has 18 false-safe poses (worst overestimate 0.056 m); the
  Xing generalized rectangle has none.
- Minimum passable gap (`scripts/measure_minimum_gap.py`): hybrid 3.18 m vs
  exact rectangle 2.70 m for the bus, 2.69 m vs 2.30 m for the SUV.

## Superseded results

`results/archived/` holds earlier runs, including the 72-case frozen gap
protocol (`results/archived/narrow/`) and the harmonized sensitivity run. Those
runs predate the node-0 repair, the explicit `N_DISC` hand-off and the Xing
residual fix, and they use the older gap task. They are kept for the audit
trail only; the numbers above replace them.

## 5. Run tests

```bash
.venv/bin/python -m pytest -q tests
```

The provenance files record the protocol, source-file hashes, dependency
commits and thread settings. The tests cover the independent evaluator, the
reconstruction equations, constraint construction, acceptance rules and
scenario-feasibility checks.

## 6. Important scope limits

- The planners are equation-based reconstructions, not author-code
  reproductions or deployment-level rankings.
- The benchmark is deterministic and synthetic; clearance is sampled at
  0.02 s and is not a continuous-time safety proof.
- The proposed hybrid is not an exact rectangle: for the bus its minimum
  geometric passable gap is 3.18 m versus 2.70 m for the exact rectangle.
- The variable-curvature addition is a static geometry audit. The closed-loop
  interface (`scripts/run_variable_curvature_closed_loop.py`) is smoke-tested
  only; no formal variable-curvature closed-loop completion rate is claimed.
- Timing values from acados native time and Python wall time use different
  backends and are not a cross-method speedup claim.

For the full protocol, implementation-fidelity notes and audit trail, see:

- `docs/NARROW_BENCHMARK_PROTOCOL.md`
- `docs/RANDOM_BENCHMARK_PROTOCOL.md`
- `docs/VARIABLE_CURVATURE_PROTOCOL.md`
- `docs/RECONSTRUCTION.md`
- `docs/AUDIT_FINDINGS.md`
- `docs/FINAL_COMPARISON_REPORT.md`
