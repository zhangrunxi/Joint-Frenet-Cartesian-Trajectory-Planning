#!/bin/bash
# Deterministic geometry sensitivity; keep each existing result directory intact.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
OUT="${1:-results/width_sweep}"
mkdir -p "$OUT"
for pair in "4.0 2.0" "4.3 2.3" "4.6 2.6" "4.9 2.9" "5.2 3.2" "5.5 3.5"; do
    read -r width lateral <<< "$pair"
    bash scripts/run_paper_comparison.sh --steps 120 \
        --methods reiter_lifted_circles paper6_circles_sqp paper6_hybrid_sqp \
        --half-width "$width" --lateral "$lateral" \
        --output "$OUT/half_width_$width" > "$OUT/half_width_$width.log" 2>&1
done
