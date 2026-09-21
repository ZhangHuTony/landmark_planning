#!/usr/bin/env bash
# Work around a real Manim/PyAV bug: combine_files() in
# manim/scene/scene_file_writer.py concatenates a scene's per-animation
# "partial movie files" by demuxing raw H.264 PACKETS from each one and
# muxing them into the output with no re-encode (scene_file_writer.py's
# `combine_files`, the else branch: `partial_movies_input.demux(...)` ->
# `output_container.mux(packet)`). That is a plain stream-copy concat, and
# it is fragile: verified on this project's Baselines scene, whose 18
# partial files individually decode with correct content (checked one at a
# time), but whichever splice falls near a short (~0.4 s) animation can
# silently lose that animation's content in the FINAL file -- ffmpeg's own
# "-f concat" DEMUXER reproduces the identical corruption from the same
# packet list (so this is not Manim-specific machinery, it is the demuxer
# class of bug), while the concat FILTER (which fully decodes every input
# file as its own independent stream, then re-encodes the decoded frames)
# does not. Confirmed by direct comparison of the same 18 files through
# both paths.
#
# This script re-runs the LAST step of Manim's own pipeline correctly:
# read the partial_movie_file_list.txt Manim already wrote, decode every
# segment independently, and re-encode. Run it after any `manim` render
# whose scene has enough short animations to risk this (cheap: run it on
# everything).
set -euo pipefail
IN="${1:?usage: fix_manim_concat.sh <scene_output.mp4>}"
DIR="$(dirname "$IN")"
STEM="$(basename "$IN" .mp4)"
LIST="$DIR/partial_movie_files/$STEM/partial_movie_file_list.txt"
[ -f "$LIST" ] || { echo "no partial_movie_file_list.txt next to $IN; nothing to fix" >&2; exit 0; }

mapfile -t FILES < <(grep -oP "(?<=file 'file:).*(?=')" "$LIST")
N=${#FILES[@]}
[ "$N" -gt 0 ] || { echo "empty file list for $IN" >&2; exit 1; }

args=(); filt=""
for i in "${!FILES[@]}"; do
  args+=(-i "${FILES[$i]}")
  filt+="[$i:v]"
done
filt+="concat=n=$N:v=1:a=0[v]"

TMP="$IN.fixed.mp4"
ffmpeg -y -v error "${args[@]}" -filter_complex "$filt" -map "[v]" \
  -c:v libx264 -preset medium -crf 16 -pix_fmt yuv420p -movflags +faststart "$TMP"
mv "$TMP" "$IN"
echo "re-concatenated (concat filter, $N segments): $IN"
