#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"
SID="${SID:-s049}"
for m in hexspline_cl greedy formation sequential clgbt; do
  run="video/runs/baselines/${SID}_p050/$m"
  [ -f "$run/$m/results.yaml" ] || { echo "skip $m (no run)"; continue; }
  echo "=== $m ==="
  ~/.juliaup/bin/julia video/julia/export_scene.jl "$run" --algo="$m" \
      --out="video/data/baselines/${SID}_p050/${m}.json" 2>&1 | tail -2
done
echo "BASELINE EXPORTS DONE"
