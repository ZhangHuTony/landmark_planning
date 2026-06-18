#!/usr/bin/env bash
# Sweep DIR_UNCERTAINTY_PER_METER across all four scenarios.
# PERP_UNCERTAINTY_PER_METER is derived (DIR / MAJ_MIN_UNC_RATIO) so it scales automatically.
# Outputs land in results/dir_unc_<value>/<scenario>/
set -euo pipefail

JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT="$(cd "$(dirname "$0")" && pwd)/planner.jl"
TMPFILE=$(mktemp /tmp/planner_XXXX.jl)
trap 'rm -f "$TMPFILE"' EXIT

DIR_UNCS=(0.05 0.10 0.20 0.30)
SCENARIOS=(single dual clustered shoreline)

for dir_unc in "${DIR_UNCS[@]}"; do
    for scenario in "${SCENARIOS[@]}"; do
        outdir="$(cd "$(dirname "$0")" && pwd)/results/dir_unc_${dir_unc}/${scenario}"
        mkdir -p "$outdir"
        echo ""
        echo "=========================================="
        echo "  Scenario: $scenario   DIR_UNC: ${dir_unc}/m"
        echo "  Output dir: $outdir"
        echo "=========================================="
        sed \
            -e "s/^const LANDMARK_SCENARIO = :.*$/const LANDMARK_SCENARIO = :${scenario}/" \
            -e "s/^const DIR_UNCERTAINTY_PER_METER.*$/const DIR_UNCERTAINTY_PER_METER  = ${dir_unc}/" \
            "$SCRIPT" > "$TMPFILE"
        (cd "$outdir" && $JULIA "$TMPFILE") 2>&1 | tee "${outdir}/run.log"
        echo "  Done: $scenario dir_unc=${dir_unc}"
    done
done

echo ""
echo "Dir uncertainty sweep complete."
