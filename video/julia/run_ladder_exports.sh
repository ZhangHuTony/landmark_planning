#!/usr/bin/env bash
# Export the four Fig. 2 constraint-ladder rungs shown in the paper.
# One Julia process per rung: src/config.jl freezes the config into consts at
# include time, so a single process cannot hold two different rungs.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
for pct in 100 070 050 030; do
  rung="fig2_ladder/rect_r3/hexspline_cl/s001_p${pct}"
  out="video/data/ladder/scene_p${pct}.json"
  echo "=== rung p${pct} ==="
  ~/.juliaup/bin/julia video/julia/export_scene.jl "$rung" --out="$out" 2>&1 | tail -3
done
echo "LADDER EXPORT DONE"
