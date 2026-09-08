#!/bin/bash
# baselines.sh <scenario_file> <tag> <U_ref> ["pcts"] ["algos"] [extra key=value ...]
# Runs each baseline planner (2 agents) at pct% of U_ref, 4 processes at a time; prints a table.
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/../../../../.." && pwd); R=$ROOT/results/fig2
scen=$1; tag=$2; uref=$3; pcts=${4:-"100 70 50 30"}; algos=${5:-"greedy formation sequential clgbt"}; shift $(( $# >= 5 ? 5 : $# ))
H100="hex_width_m=100.0 corridor_halfwidth_m=260.0 corridor_y_max_m=300.0"
i=0
for a in $algos; do for p in $pcts; do
  thr=$(python3 -c "print(round($p/100*$uref, 6))")
  $HERE/run.sh $scen ${tag}_${a}_p$p $H100 algorithms=$a num_agents=2 unc_radius_threshold=$thr emit_figures=true emit_csv=true track_comm_events=true "$@" >/dev/null &
  i=$((i+1)); [ $((i % 4)) -eq 0 ] && wait
done; done; wait
printf "%-11s %-5s %-8s %-9s %-8s %-8s %s\n" algo pct thr len unc wall_s note
for a in $algos; do for p in $pcts; do
  d=$R/${tag}_${a}_p$p; r=$(ls $d/*/results.yaml 2>/dev/null | head -1)
  thr=$(python3 -c "print(round($p/100*$uref, 3))")
  L=$(grep "^  primary_length" $r 2>/dev/null | awk '{print $2}' | cut -c1-8); U=$(grep "^  primary_unc" $r 2>/dev/null | awk '{print $2}' | cut -c1-6)
  st=$(grep refinement_status $r 2>/dev/null | awk '{print $2}')
  ok=$(python3 -c "print('OK' if '$U' not in ('','null') and float('$U') <= $thr + 1e-6 and '$st' != 'recovery_failed' else 'FAIL')" 2>/dev/null)
  printf "%-11s %-5s %-8s %-9s %-8s %-8s %s\n" $a $p $thr "${L:-null}" "${U:-null}" "$(grep -o 'wall[_a-z]*: [0-9.]*' $r 2>/dev/null | head -1 | awk '{print $2}')" "$ok $st"
done; done
