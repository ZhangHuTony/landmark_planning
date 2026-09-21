#!/usr/bin/env bash
# Render every scene. QUALITY=l for drafts (480p15), h for the master (1080p30).
#
# Renders twice: once normal (captions on, feeds the assembled master via
# concat_master.sh) and once with CONSORT_CAPTIONS=0 (no bottom caption bar --
# meters/counters/labels are untouched, they are not "subtitles"), copied flat
# into deliverables/scenes/ so each scene can be shared on its own.
#
# Every render is then run through fix_manim_concat.sh. This is not optional
# hardening: Manim's own final-assembly step (scene_file_writer.py's
# combine_files) concatenates a scene's per-animation clips by splicing raw
# H.264 packets with no re-encode, which is fragile and was CAUGHT doing real
# damage on this project's Baselines scene -- a short (0.4 s) FadeIn near a
# splice point decoded as fully transparent in Manim's own output, even though
# the individual clip and even ffmpeg's OWN concat DEMUXER on the same files
# reproduced the identical loss. Only fully decoding each clip and re-encoding
# (the concat FILTER, not the demuxer) fixed it -- see
# ffmpeg/fix_manim_concat.sh for the full story and verification. Cheap
# (seconds per scene), so it runs on every scene, not just the one caught.
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
    out="$ROOT/render/videos/$(basename "$1" .py)/$QDIR/$2.mp4"
    "$ROOT/ffmpeg/fix_manim_concat.sh" "$out" >/dev/null 2>&1
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
    "$ROOT/ffmpeg/fix_manim_concat.sh" "$src" >/dev/null 2>&1
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
