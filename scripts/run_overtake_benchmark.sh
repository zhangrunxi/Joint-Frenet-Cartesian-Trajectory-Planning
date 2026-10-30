#!/usr/bin/env bash
# Overtaking-task subset of the frozen random protocol.
#
# The general task set contains almost no overtaking situations, so the
# geometry comparison needs its own task type. The protocol file records the
# lateral target as half_width - 1.5 m, i.e. the bus is placed against the
# road edge and has to pass the obstacle in the remaining space.
#
# Usage: bash scripts/run_overtake_benchmark.sh --output results/overtake [args]
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

exec .venv/bin/python scripts/run_random_benchmark.py --task overtake "$@"
