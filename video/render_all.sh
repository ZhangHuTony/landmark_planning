#!/usr/bin/env bash
# Render every scene. QUALITY=l for drafts (480p15), h for the master (1080p30).
#
# Renders twice: once normal (captions on, feeds the assembled master via
# concat_master.sh) and once with CONSORT_CAPTIONS=0 (no bottom caption bar --
# meters/counters/labels are untouched, they are not "subtitles"), copied flat
# into deliverables/scenes/ so each scene can be shared on its own.
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
source "$ROOT/env.sh"
cd "$ROOT/manim"
Q="${QUALITY:-l}"
QDIR=$([ "$Q" = h ] && echo 1080p30 || echo 480p15)
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
echo "=== pass 1: captions on (assembled master) ==="
for s in "${scenes[@]}"; do
  set -- $s
  printf "%-26s " "$2"
  if CONSORT_CAPTIONS=1 manim -q"$Q" --disable_caching -v ERROR --progress_bar none "scenes/$1" "$2" >/dev/null 2>&1; then
    echo "ok"
  else
    echo "FAILED"; fail=1
  fi
done

echo "=== pass 2: no captions (per-scene deliverables) ==="
NOCAP_DIR="$ROOT/render_nocap"
mkdir -p "$ROOT/deliverables/scenes"
for s in "${scenes[@]}"; do
  set -- $s
  printf "%-26s " "$2"
  if CONSORT_CAPTIONS=0 manim -q"$Q" --disable_caching -v ERROR --progress_bar none \
      --media_dir "$NOCAP_DIR" "scenes/$1" "$2" >/dev/null 2>&1; then
    src="$NOCAP_DIR/videos/$(basename "$1" .py)/$QDIR/$2.mp4"
    if [ -f "$src" ]; then
      cp "$src" "$ROOT/deliverables/scenes/$2.mp4"
      echo "ok -> deliverables/scenes/$2.mp4"
    else
      echo "ok but not found at $src"; fail=1
    fi
  else
    echo "FAILED"; fail=1
  fi
done
exit $fail
