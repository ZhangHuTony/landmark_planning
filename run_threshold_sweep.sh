#!/usr/bin/env bash
# Sweep UNC_RADIUS_THRESHOLD across all four scenarios.
# Outputs land in results/threshold_<value>/<scenario>/
set -euo pipefail

JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT="$(cd "$(dirname "$0")" && pwd)/planner.jl"
TMPFILE=$(mktemp /tmp/planner_XXXX.jl)
trap 'rm -f "$TMPFILE"' EXIT

THRESHOLDS=(3.75 3.0 2.5 2.0 1.5)
SCENARIOS=(single dual clustered shoreline)

for threshold in "${THRESHOLDS[@]}"; do
    for scenario in "${SCENARIOS[@]}"; do
        outdir="$(cd "$(dirname "$0")" && pwd)/results/threshold_${threshold}/${scenario}"
        mkdir -p "$outdir"
        echo ""
        echo "=========================================="
        echo "  Scenario: $scenario   UNC_THRESHOLD: ${threshold}m"
        echo "  Output dir: $outdir"
        echo "=========================================="
        sed \
            -e "s/^const LANDMARK_SCENARIO = :.*$/const LANDMARK_SCENARIO = :${scenario}/" \
            -e "s/^const UNC_RADIUS_THRESHOLD.*$/const UNC_RADIUS_THRESHOLD       = ${threshold}/" \
            "$SCRIPT" > "$TMPFILE"
        (cd "$outdir" && $JULIA "$TMPFILE") 2>&1 | tee "${outdir}/run.log"
        echo "  Done: $scenario threshold=${threshold}"
    done
done

echo ""
echo "Threshold sweep complete."
