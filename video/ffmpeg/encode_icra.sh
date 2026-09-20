#!/usr/bin/env bash
# The PaperPlaza attachment: <=180 s, <=20 MB, mp4/H.264, >=480 px, >=20 fps.
#
# Bitrate is arithmetic, not taste: 20 MB over 176 s is 20e6*8/176 = 909 kbit/s
# for everything, so video 760k + mono AAC 64k leaves ~9% for container
# overhead. Two passes, because a single-pass CBR at this bitrate wastes budget
# on the static cards and starves the footage.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IN="${IN:-$ROOT/video/deliverables/consort_master_1080p.mp4}"
ID="${ID:-XXXX}"
OUT="${OUT:-$ROOT/video/deliverables/ICRA2027_${ID}.mp4}"
DUR="${DUR:-176}"
VB="${VB:-760k}"
cd "$(dirname "$OUT")"
ffmpeg -y -v error -i "$IN" -t "$DUR" -vf scale=1280:720 \
  -c:v libx264 -preset slow -profile:v high -level 4.0 \
  -b:v "$VB" -maxrate 900k -bufsize 1800k -pix_fmt yuv420p -pass 1 -an -f mp4 /dev/null
if ffprobe -v error -select_streams a -show_entries stream=codec_type -of csv=p=0 "$IN" | grep -q audio; then
  ffmpeg -y -v error -i "$IN" -t "$DUR" -vf scale=1280:720 \
    -c:v libx264 -preset slow -profile:v high -level 4.0 \
    -b:v "$VB" -maxrate 900k -bufsize 1800k -pix_fmt yuv420p -pass 2 \
    -c:a aac -b:a 64k -ac 1 -movflags +faststart -map_metadata -1 "$OUT"
else
  ffmpeg -y -v error -i "$IN" -t "$DUR" -vf scale=1280:720 \
    -c:v libx264 -preset slow -profile:v high -level 4.0 \
    -b:v "$VB" -maxrate 900k -bufsize 1800k -pix_fmt yuv420p -pass 2 \
    -an -movflags +faststart -map_metadata -1 "$OUT"
fi
rm -f ffmpeg2pass-0.log ffmpeg2pass-0.log.mbtree
"$ROOT/video/ffmpeg/check_deliverable.sh" "$OUT"
