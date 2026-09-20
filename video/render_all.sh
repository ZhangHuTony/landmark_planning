#!/usr/bin/env bash
# Render every scene. QUALITY=l for drafts (480p15), h for the master (1080p30).
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
source "$ROOT/env.sh"
cd "$ROOT/manim"
Q="${QUALITY:-l}"
scenes=(
  "s01_problem.py Problem"
  "s02_escort.py Escort"
  "s03a_astar.py AStar"
  "s03b_refine.py Refine"
  "s03c_ladder.py Ladder"
  "s04_baselines.py Baselines"
  "s05_results.py Results"
  "s06_holo.py HoloRun"
  "s06_holo.py HoloMC"
  "s07_close.py Close"
)
fail=0
for s in "${scenes[@]}"; do
  set -- $s
  printf "%-26s " "$2"
  if manim -q"$Q" --disable_caching -v ERROR --progress_bar none "scenes/$1" "$2" >/dev/null 2>&1; then
    echo "ok"
  else
    echo "FAILED"; fail=1
  fi
done
exit $fail
