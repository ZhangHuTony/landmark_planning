#!/usr/bin/env bash
# Drop the sped-up footage into the slot HoloRun leaves empty.
#
# HoloRun reserves the left 960x540 px of the 1920x1080 frame (Manim x in
# [-7.111, 0], y in [-2, 2]) and draws nothing there, so a plain overlay needs
# no masking. The two are exactly the same 16.99 s (HoloRun's play() segments
# sum to n_ticks/900 with no waits; the footage is 510 decimated frames at
# 30 fps = 17.0 s), so eof_action=repeat is a safety net, not a fix: if the
# panel ever runs a hair longer (rounding), the footage box freezes on its
# last real frame instead of the overlay vanishing and showing raw base video
# where the footage used to be.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
R="$ROOT/video/render"
PANEL="${PANEL:-$R/videos/s06_holo/1080p30/HoloRun.mp4}"
FOOT="${FOOT:-$R/holo_closeup_x30.mp4}"
OUT="${OUT:-$R/s06_holo_composite.mp4}"
[ -f "$PANEL" ] || { echo "no HoloRun render at $PANEL (render it at -qh first)"; exit 1; }
# y-offset centers the footage vertically in whatever resolution PANEL was
# rendered at (270 at 1080p, scaled down for a draft -ql render) -- a fixed
# pixel offset only centers correctly at 1080p.
PANEL_H=$(ffprobe -v error -select_streams v:0 -show_entries stream=height -of csv=p=0 "$PANEL")
FOOT_H=$(ffprobe -v error -select_streams v:0 -show_entries stream=height -of csv=p=0 "$FOOT")
Y=$(( (PANEL_H - FOOT_H) / 2 ))
ffmpeg -y -v error -i "$PANEL" -i "$FOOT" \
  -filter_complex "[0:v][1:v]overlay=x=0:y=$Y:eof_action=repeat[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 16 -pix_fmt yuv420p "$OUT"
echo "-> $OUT"
