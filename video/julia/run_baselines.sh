#!/usr/bin/env bash
# Re-run ONE benchmark scenario at the 50% constraint level for all five
# planners, with geometry emitted.
#
# The sweep itself stored numbers only (config/mc/sweep.yaml sets save_csv
# false), so there are no paths to animate. The harness did leave a complete,
# self-contained config per (method, rung) in _cfg/, and a `random` scenario
# regenerates exactly from the four scenario_* keys in it, so copying that and
# flipping the emit flags reproduces the very run the benchmark scored.
#
# One Julia process per method, as run_constraint_sweep.jl does: src/config.jl
# merges every selected algorithm's yaml into one dict, so the five were never
# meant to share a process.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
SID="${SID:-s049}"
SWEEP="constraint_sweep/baseline_2026-08-17"
for m in hexspline_cl greedy formation sequential clgbt; do
  src="$SWEEP/$m/${SID}_p050/_cfg"
  dst="video/config/baselines/${SID}_p050/$m"
  [ -d "$src" ] || { echo "no _cfg for $m/${SID}_p050"; continue; }
  mkdir -p "$dst" && cp "$src"/*.yaml "$dst"/
  sed -i -e 's/^emit_csv: false/emit_csv: true/' \
         -e 's/^emit_figures: false/emit_figures: true/' \
         -e 's/^track_comm_events: false/track_comm_events: true/' \
         -e 's/^track_landmark_events: false/track_landmark_events: true/' "$dst/main.yaml"
  # video/: search-process traces for the "watch it search" panels. All
  # println-only, default off; see notes/LOGS.md for the no-op proofs.
  printf 'trace_astar: true\ntrace_cont: true\n' >> "$dst/main.yaml"
  [ "$m" = greedy ]     && echo "trace_greedy: true"     >> "$dst/main.yaml"
  [ "$m" = sequential ] && echo "trace_sequential: true" >> "$dst/main.yaml"
  out="video/runs/baselines/${SID}_p050/$m"
  echo "=== $m ==="
  OUTPUT_DIR="$out" ~/.juliaup/bin/julia generate_plan.jl "$dst" > "$out.log" 2>&1 \
    && grep -E "primary_length|primary_unc|refinement_status" "$out/$m/results.yaml" 2>/dev/null \
    || echo "  FAILED (see $out.log)"
done
echo "BASELINE RUNS DONE"
