#!/usr/bin/env bash
# Sweep unc_radius_threshold across all four scenarios.
# Outputs land in results/threshold_<value>/<scenario>/
set -euo pipefail

JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$SCRIPT_DIR/planner.jl"
TMPCONFIG=$(mktemp /tmp/config_XXXX.yaml)
trap 'rm -f "$TMPCONFIG"' EXIT

THRESHOLDS=(2.8 3.0 3.2 3.4 3.6)
SCENARIOS=(clustered shoreline)

for threshold in "${THRESHOLDS[@]}"; do
    for scenario in "${SCENARIOS[@]}"; do
        outdir="$SCRIPT_DIR/results/threshold_${threshold}/${scenario}"
        mkdir -p "$outdir"
        echo ""
        echo "=========================================="
        echo "  Scenario: $scenario   UNC_THRESHOLD: ${threshold}m"
        echo "  Output dir: $outdir"
        echo "=========================================="
        sed \
            -e "s/^landmark_scenario:.*$/landmark_scenario: ${scenario}/" \
            -e "s/^unc_radius_threshold:.*$/unc_radius_threshold: ${threshold}/" \
            "$SCRIPT_DIR/config.yaml" > "$TMPCONFIG"
        (cd "$outdir" && $JULIA "$SCRIPT" "$TMPCONFIG") 2>&1 | tee "${outdir}/run.log"
        echo "  Done: $scenario threshold=${threshold}"
    done
done

echo ""
echo "Threshold sweep complete."
