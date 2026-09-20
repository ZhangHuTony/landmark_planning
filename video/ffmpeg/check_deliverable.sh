#!/usr/bin/env bash
# Assert the attachment satisfies every ICRA 2027 limit before it is submitted.
set -euo pipefail
F="${1:?usage: check_deliverable.sh <file.mp4>}"
read -r CODEC W H FPS ORDER PIXFMT <<<"$(ffprobe -v error -select_streams v:0 \
  -show_entries stream=codec_name,width,height,r_frame_rate,field_order,pix_fmt \
  -of csv=p=0 "$F" | tr ',' ' ')"
DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$F")
SIZE=$(stat -c %s "$F")
fail=0
chk() { if [ "$2" = 1 ]; then echo "  ok    $1"; else echo "  FAIL  $1"; fail=1; fi; }
echo "$F"
chk "codec h264 ($CODEC)"                 "$([ "$CODEC" = h264 ] && echo 1 || echo 0)"
chk "height >= 480 ($H)"                  "$([ "$H" -ge 480 ] && echo 1 || echo 0)"
chk "fps >= 20 ($FPS)"                    "$(awk -F/ '{print ($1/$2>=20)?1:0}' <<<"$FPS")"
chk "progressive ($ORDER)"                "$([ "$ORDER" = progressive ] || [ "$ORDER" = unknown ] && echo 1 || echo 0)"
chk "pix_fmt yuv420p ($PIXFMT)"           "$([ "$PIXFMT" = yuv420p ] && echo 1 || echo 0)"
chk "duration <= 180 s ($(printf %.1f "$DUR"))" "$(awk '{print ($1<=180)?1:0}' <<<"$DUR")"
chk "size <= 20 MB ($((SIZE/1000000)) MB)"      "$([ "$SIZE" -le 20000000 ] && echo 1 || echo 0)"
if ffprobe -v error -show_entries format_tags -of default=nw=1 "$F" | grep -qiE "encoder|title|artist"; then
  echo "  note  container tags present:"; ffprobe -v error -show_entries format_tags -of default=nw=1 "$F" | sed 's/^/        /'
fi
exit $fail
