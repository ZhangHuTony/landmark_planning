#!/usr/bin/env bash
# Concatenate the rendered scenes into the 1080p master.
#
# Every input must already be 1920x1080 / 30 fps / yuv420p, which manim.cfg
# guarantees, so the concat demuxer can stitch without re-encoding decisions
# per input. Metadata is stripped (-map_metadata -1): the project-page master
# and the review attachment must not carry a username or machine name.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
R="$ROOT/video/render"
Q="${Q:-1080p30}"
OUT="${OUT:-$ROOT/video/deliverables/consort_master_1080p.mp4}"
VOICE="${VOICE:-$ROOT/video/narration/voice.wav}"
mkdir -p "$(dirname "$OUT")"

order=(
  "s01_problem/$Q/Problem.mp4"
  "s02_escort/$Q/Escort.mp4"
  "s03a_astar/$Q/AStar.mp4"
  "s03b_refine/$Q/Refine.mp4"
  "s03c_ladder/$Q/Ladder.mp4"
  "s04_baselines/$Q/Baselines.mp4"
  "s05_results/$Q/Results.mp4"
  "s06_holo_composite.mp4"
  "s06_holo/$Q/HoloMC.mp4"
  "s07_close/$Q/Close.mp4"
)
list="$R/concat.txt"; : > "$list"
for f in "${order[@]}"; do
  p="$R/videos/$f"; [ -f "$p" ] || p="$R/$f"
  if [ -f "$p" ]; then echo "file '$p'" >> "$list"; else echo "skip (missing): $f" >&2; fi
done
echo "--- concatenating ---"; cat "$list"

if [ -f "$VOICE" ]; then
  ffmpeg -y -v error -f concat -safe 0 -i "$list" -i "$VOICE" \
    -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -r 30 \
    -c:a aac -b:a 160k -shortest -movflags +faststart -map_metadata -1 "$OUT"
else
  echo "(no narration/voice.wav yet: silent master)"
  ffmpeg -y -v error -f concat -safe 0 -i "$list" \
    -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -r 30 \
    -movflags +faststart -map_metadata -1 "$OUT"
fi
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$OUT"
echo "-> $OUT"
