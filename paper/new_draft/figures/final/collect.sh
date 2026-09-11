#!/bin/bash
# Collect the paper's final figures into this folder as PDF + EPS (what main.tex includes).
# Re-run after regenerating any figure. Sources are the scripts' own outputs under figures/.
set -e
cd "$(dirname "$0")/.."
copy() { for ext in pdf eps; do cp "$1.$ext" "final/$2.$ext"; done; }
copy figure2/rect_candidates/rect_r3/fig2_ladder_grid_hexspline_cl  fig2_ladder          # ladder_r3, staggered walls (chosen 2026-09-10)
copy fig3a_success_vs_constraint                                     fig3a_success
copy fig3b_length_vs_constraint                                      fig3b_length
copy fig4_wall_5level_box                                            fig4_wall
copy fig5a_abl_success_vs_constraint                                 fig5a_abl_success
copy fig5b_abl_length_vs_constraint                                  fig5b_abl_length
copy fig6_abl_wall_5level_box                                        fig6_abl_wall
copy fig8_mc_behind_wall                                             fig8_mc
echo "final/: $(ls final/*.pdf | wc -l) pdf, $(ls final/*.eps | wc -l) eps"
