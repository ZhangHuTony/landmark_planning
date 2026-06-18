#!/usr/bin/env bash
# Stress configuration: tight threshold + landmark requires proximity + wide comm.
#
# Goal: support detours close to landmark (gets precise fix), primary stays
# on straight path, support's comm update is enough to bring primary below threshold.
#
# Changes from defaults:
#   UNC_RADIUS_THRESHOLD  3.75 → 2.3    (below straight-line unc: primary needs help)
#   VISIBILITY_SIGMA      75   → 45     (landmark only useful within ~90m; primary at
#                                        y=0 gets ~0 info from landmark at y=-250)
#   COMM_SIGMA            50   → 250    (support 250m from primary still gets weight ~0.61)
#   COMM_RADIUS           300  → 250    (align in-search taper with exact evaluator)
#   ASTAR_ITERATION_LIMIT 200k → 500k   (detour paths are deeper in the search space)
#
# Outputs land in results/stress/<scenario>/
set -euo pipefail

JULIA=/home/tonyzhang/.juliaup/bin/julia
SCRIPT="$(cd "$(dirname "$0")" && pwd)/planner.jl"
TMPFILE=$(mktemp /tmp/planner_XXXX.jl)
trap 'rm -f "$TMPFILE"' EXIT

SCENARIOS=(single dual clustered shoreline)

for scenario in "${SCENARIOS[@]}"; do
    outdir="$(cd "$(dirname "$0")" && pwd)/results/stress/${scenario}"
    mkdir -p "$outdir"
    echo ""
    echo "=========================================="
    echo "  Stress config — scenario: $scenario"
    echo "  threshold=2.3  VIS_SIGMA=45  COMM_SIGMA=250  COMM_RADIUS=250  iters=500k"
    echo "  Output dir: $outdir"
    echo "=========================================="
    sed \
        -e "s/^const LANDMARK_SCENARIO = :.*$/const LANDMARK_SCENARIO = :${scenario}/" \
        -e "s/^const UNC_RADIUS_THRESHOLD.*$/const UNC_RADIUS_THRESHOLD       = 2.0/" \
        -e "s/^const VISIBILITY_SIGMA\b.*$/const VISIBILITY_SIGMA           = 45.0/" \
        -e "s/^const COMM_SIGMA\b.*$/const COMM_SIGMA                 = 250.0/" \
        -e "s/^const COMM_RADIUS\b.*$/const COMM_RADIUS                = 250.0/" \
        -e "s/^const ASTAR_ITERATION_LIMIT.*$/const ASTAR_ITERATION_LIMIT = 500000/" \
        "$SCRIPT" > "$TMPFILE"
    (cd "$outdir" && $JULIA "$TMPFILE") 2>&1 | tee "${outdir}/run.log"
    echo "  Done: $scenario"
done

echo ""
echo "Stress config runs complete."
