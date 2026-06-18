#!/usr/bin/env bash
# Run all four landmark scenarios, saving outputs into their respective results/ dirs.
set -euo pipefail
JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT="$(cd "$(dirname "$0")" && pwd)/planner.jl"
TMPFILE=$(mktemp /tmp/planner_XXXX.jl)
trap 'rm -f "$TMPFILE"' EXIT

for scenario in single dual clustered shoreline; do
    outdir="$(cd "$(dirname "$0")" && pwd)/results/sigma_50/${scenario}"
    mkdir -p "$outdir"
    echo ""
    echo "=========================================="
    echo "  Running scenario: $scenario"
    echo "  Output dir: $outdir"
    echo "=========================================="
    sed "s/^const LANDMARK_SCENARIO = :.*$/const LANDMARK_SCENARIO = :${scenario}/" "$SCRIPT" > "$TMPFILE"
    (cd "$outdir" && $JULIA "$TMPFILE") 2>&1 | tee "${outdir}/run.log"
    echo "  Done: $scenario"
done

echo ""
echo "All four scenarios complete."
