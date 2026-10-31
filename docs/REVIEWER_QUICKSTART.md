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

## 2. Run the primary frozen comparison

The primary evidence uses the frozen protocol. It keeps the method-specific
initialization and acceptance behavior, and is the table reported in the
manuscript:

```bash
BENCH_VEHICLE=bus bash scripts/run_random_benchmark.sh \
  --task gap --output results/reviewer_bus_gap_frozen
BENCH_VEHICLE=suv bash scripts/run_random_benchmark.sh \
  --task gap --output results/reviewer_suv_gap_frozen

.venv/bin/python scripts/summarize_narrow_benchmark.py --task gap \
  --input results/reviewer_bus_gap_frozen \
  --output results/reviewer_bus_gap_frozen/summary.csv
.venv/bin/python scripts/summarize_narrow_benchmark.py --task gap \
  --input results/reviewer_suv_gap_frozen \
  --output results/reviewer_suv_gap_frozen/summary.csv
```

The archived reference results are already available under
`results/archived/narrow/`. The frozen headline counts are:

| Method | Bus | SUV |
| --- | ---: | ---: |
| Proposed hybrid | 41/72 | 45/72 |
| Proposed circles | 38/72 | 38/72 |
| Xing reconstruction | 37/72 | 42/72 |
| Reiter ellipse (direct/lifted) | 2/2 | 7/2 |

A pass requires completion, passing the obstacle, and at least 0.19 m exact
obstacle clearance. The independent evaluator uses the exact rectangular
vehicle body; following an obstacle is not counted as overtaking.

## 3. Check the curb and discretization limitation

```bash
BENCH_VEHICLE=bus BENCH_INITIAL_GUESS=ramp BENCH_ACCEPT_MAXITER=1 \
  BENCH_N_DISC=10 bash scripts/run_random_benchmark.sh --task curb \
  --output results/reviewer_bus_curb_n10
BENCH_VEHICLE=suv BENCH_INITIAL_GUESS=ramp BENCH_ACCEPT_MAXITER=1 \
  BENCH_N_DISC=10 bash scripts/run_random_benchmark.sh --task curb \
  --output results/reviewer_suv_curb_n10
```

The ten-segment contour reaches minimum exact-rectangle road clearances of
0.182 m for the bus and 0.188 m for the SUV. The four excluded SUV curb poses
are invalid initial states because their exact body clearance is already below
the prescribed 0.2 m margin. Strong full-body road-boundary references are
kept as sensitivity checks; the proposed contour is not claimed to dominate
them.

## 4. Check the harmonized sensitivity analysis

The harmonized setting is not the primary ranking. It applies the node-0
repair to the proposed solver and a shared ramp initial guess plus shared
iteration-cap acceptance to all methods:

```bash
BENCH_VEHICLE=bus BENCH_INITIAL_GUESS=ramp BENCH_ACCEPT_MAXITER=1 \
  bash scripts/run_random_benchmark.sh --task gap \
  --output results/reviewer_bus_gap_harmonized
BENCH_VEHICLE=suv BENCH_INITIAL_GUESS=ramp BENCH_ACCEPT_MAXITER=1 \
  bash scripts/run_random_benchmark.sh --task gap \
  --output results/reviewer_suv_gap_harmonized
```

The archived sensitivity counts are 52/72 for the bus and 51/72 for the SUV.
They must not replace the frozen table or be interpreted as a causal
cross-method improvement, because the harmonization was introduced after the
frozen run.

## 5. Run tests and verify archived provenance

```bash
.venv/bin/python -m pytest -q tests
.venv/bin/python scripts/verify_final_comparison.py \
  --inputs results/final_three_paper_wide_v2 \
           results/final_three_paper_narrow_v2
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
- The variable-curvature addition is a static geometry audit. The available
  closed-loop interface is smoke-tested only; no formal variable-curvature
  closed-loop completion rate is claimed.
- Timing values from acados native time and Python wall time use different
  backends and are not a cross-method speedup claim.

For the full protocol, implementation-fidelity notes and audit trail, see:

- `docs/NARROW_BENCHMARK_PROTOCOL.md`
- `docs/RANDOM_BENCHMARK_PROTOCOL.md`
- `docs/RECONSTRUCTION.md`
- `docs/AUDIT_FINDINGS.md`
- `docs/FINAL_COMPARISON_REPORT.md`
