#!/usr/bin/env bash
# Greedy at a rung the sweep skipped.
#
# run_constraint_sweep.jl uses method_patience=1: once a planner fails a level
# it is recorded as failing every tighter one without being run, so there is no
# _cfg for greedy at s049_p050. This takes the loosest rung greedy actually ran
# and substitutes the 50% threshold from the sweep's own trials_all.csv, which
# is exactly the run the harness would have performed. The expected outcome is
# a failure: that IS greedy's panel in the video.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SID="${SID:-s049}"
SWEEP="constraint_sweep/baseline_2026-08-17"
THR=$(awk -F, -v s="$SID" '$1=="greedy" && $2==s && $3=="50" {print $4}' "$SWEEP/trials_all.csv")
SRC=$(ls -d "$SWEEP/greedy/${SID}_p"*/ 2>/dev/null | sort | tail -1)
[ -n "$THR" ] && [ -n "$SRC" ] || { echo "no threshold or source rung for greedy/$SID"; exit 1; }
echo "greedy $SID: borrowing $(basename "$(dirname "$SRC/x")") and setting threshold $THR"
dst="video/config/baselines/${SID}_p050/greedy"
mkdir -p "$dst" && cp "$SRC/_cfg"/*.yaml "$dst"/
sed -i -e 's/^emit_csv: false/emit_csv: true/' \
       -e 's/^emit_figures: false/emit_figures: true/' \
       -e 's/^track_comm_events: false/track_comm_events: true/' \
       -e 's/^track_landmark_events: false/track_landmark_events: true/' \
       -e "s/^unc_radius_threshold: .*/unc_radius_threshold: $THR/" "$dst/main.yaml"
printf 'trace_astar: true\ntrace_cont: true\ntrace_greedy: true\n' >> "$dst/main.yaml"
grep -E "^(unc_radius_threshold|algorithms|scenario_seed)" "$dst/main.yaml"
out="video/runs/baselines/${SID}_p050/greedy"
OUTPUT_DIR="$out" ~/.juliaup/bin/julia generate_plan.jl "$dst" > "$out.log" 2>&1 || true
cat "$out/greedy/results.yaml" 2>/dev/null || tail -5 "$out.log"
