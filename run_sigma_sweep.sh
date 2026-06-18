#!/usr/bin/env bash
# Run all four scenarios at varying COMM_SIGMA values.
# Also sets COMM_RADIUS = COMM_SIGMA so the in-search approximation stays consistent.
# Outputs land in results/sigma_<value>/<scenario>/
set -euo pipefail

JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT="$(cd "$(dirname "$0")" && pwd)/planner.jl"
TMPFILE=$(mktemp /tmp/planner_XXXX.jl)
trap 'rm -f "$TMPFILE"' EXIT

SIGMAS=(50 100 150 200 300)
SCENARIOS=(single dual clustered shoreline)

for sigma in "${SIGMAS[@]}"; do
    for scenario in "${SCENARIOS[@]}"; do
        outdir="$(cd "$(dirname "$0")" && pwd)/results/sigma_${sigma}/${scenario}"
        mkdir -p "$outdir"
        echo ""
        echo "=========================================="
        echo "  Scenario: $scenario   COMM_SIGMA: ${sigma}m"
        echo "  Output dir: $outdir"
        echo "=========================================="
        sed \
            -e "s/^const LANDMARK_SCENARIO = :.*$/const LANDMARK_SCENARIO = :${scenario}/" \
            -e "s/^const COMM_SIGMA\b.*$/const COMM_SIGMA                 = ${sigma}.0/" \
            -e "s/^const COMM_RADIUS\b.*$/const COMM_RADIUS                = ${sigma}.0/" \
            "$SCRIPT" > "$TMPFILE"
        (cd "$outdir" && $JULIA "$TMPFILE") 2>&1 | tee "${outdir}/run.log"
        echo "  Done: $scenario sigma=${sigma}"
    done
done

echo ""
echo "Sigma sweep complete."
