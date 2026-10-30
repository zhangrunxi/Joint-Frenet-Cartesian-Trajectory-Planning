#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export ACADOS_SOURCE_DIR="$ROOT/vendor/acados"
export DYLD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MPLBACKEND=Agg
exec .venv/bin/python scripts/run_paper_comparison.py "$@"
