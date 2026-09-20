#!/usr/bin/env bash
# Drop the sped-up footage into the slot HoloRun leaves empty.
#
# HoloRun reserves the left 960x540 px of the 1920x1080 frame (Manim x in
# [-7.111, 0], y in [-2, 2]) and draws nothing there, so a plain overlay needs
# no masking. The two are already in sync by construction; check_sync.py
# verifies it against the simulator's own map frames.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
R="$ROOT/video/render"
PANEL="${PANEL:-$R/videos/s06_holo/1080p30/HoloRun.mp4}"
FOOT="${FOOT:-$R/holo_closeup_x30.mp4}"
OUT="${OUT:-$R/s06_holo_composite.mp4}"
[ -f "$PANEL" ] || { echo "no HoloRun render at $PANEL (render it at -qh first)"; exit 1; }
ffmpeg -y -v error -i "$PANEL" -i "$FOOT" \
  -filter_complex "[0:v][1:v]overlay=x=0:y=270:eof_action=pass[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 16 -pix_fmt yuv420p "$OUT"
echo "-> $OUT"
