#!/usr/bin/env bash
# Speed the rendered HoloOcean footage up x30 by dropping 29 of every 30 frames.
#
# closeup_auv0.mp4 was encoded 1:1 from frames_auv0/<tick>.bmp, which main.py
# writes inside the same loop iteration that calls rec.tick(). So frame n is
# tick n is row n of auv0_track, with no timestamp matching. Keeping every 30th
# frame therefore lands exactly on the ticks the Manim panel shows at 900 ticks
# per second of video: output frame k == tick 30k == Manim second k/30.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="${SRC:-$ROOT/results/2026-09-02_thr1.8b/video/closeup_auv0.mp4}"
OUT="${OUT:-$ROOT/video/render/holo_closeup_x30.mp4}"
mkdir -p "$(dirname "$OUT")"
ffmpeg -y -v error -i "$SRC" \
  -vf "select='not(mod(n\,30))',setpts=N/(30*TB),scale=960:540" \
  -r 30 -an -c:v libx264 -preset slow -crf 14 -pix_fmt yuv420p "$OUT"
ffprobe -v error -show_entries stream=nb_frames,width,height -of default=nw=1 "$OUT"
echo "-> $OUT"
