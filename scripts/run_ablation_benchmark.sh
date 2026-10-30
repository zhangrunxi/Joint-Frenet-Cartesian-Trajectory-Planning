#!/usr/bin/env bash
# Representation ablations of the proposed model, on the frozen random protocol.
#
# Only the proposed model's own variants are run here; the Reiter and Xing
# baselines are covered by run_random_benchmark.sh. The variant set is:
#
#   paper6_hybrid_sqp     proposed model, capsule plus corner circles
#   paper6_frozen_sqp     [16]-style frozen heading deviation along the contour
#   paper6_circles_sqp    three covering circles
#   paper6_circles5_sqp   five covering circles
#   paper6_circlesN3_sqp  three circles with the radius of Eq. 11-14
#   paper6_circlesN5_sqp  five circles with the radius of Eq. 11-14
#   paper6_nokappa_sqp    steering angle as the sixth state instead of curvature
#   paper6_pruned_sqp     hierarchical constraint pruning from Section 5.2
#
# Usage: bash scripts/run_ablation_benchmark.sh --output results/ablation [args]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export ACADOS_SOURCE_DIR="$ROOT/vendor/acados"
export DYLD_LIBRARY_PATH="$ROOT/vendor/acados/lib:${DYLD_LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="$ROOT/vendor/acados/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export ACADOS_OCP_SPY=0

exec .venv/bin/python scripts/run_random_benchmark.py \
  --methods paper6_hybrid_sqp paper6_frozen_sqp paper6_circles_sqp \
  paper6_circles5_sqp paper6_circlesN3_sqp paper6_circlesN5_sqp \
  paper6_nokappa_sqp paper6_pruned_sqp paper6_capsule_sqp \
  "$@"
