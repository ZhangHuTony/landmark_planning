#!/bin/bash
# ladder.sh <scenario_file> <tag> [pcts="100 90 80 70 60 50 40 30"] [extra key=value ...]
# Reference (1 agent, unbounded) then hexspline_cl 2-agent rungs at pct% of U_ref, 4 at a time.
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/../../../../.." && pwd); R=$ROOT/results/fig2
scen=$1; tag=$2; pcts=${3:-"100 90 80 70 60 50 40 30"}; shift $(( $# >= 3 ? 3 : $# ))
H100="hex_width_m=100.0 corridor_halfwidth_m=260.0 corridor_y_max_m=300.0"
$HERE/run.sh $scen ${tag}_ref $H100 num_agents=1 unc_radius_threshold=1.0e9 emit_figures=true emit_csv=true "$@" >/dev/null
uref=$(grep primary_disc_unc $R/${tag}_ref/hexspline_cl/results.yaml | awk '{print $2}')
lref=$(grep "^  primary_length" $R/${tag}_ref/hexspline_cl/results.yaml | awk '{print $2}')
dref=$(grep -o "FEASIBLE SOLUTION at iter [0-9]*: dist=[0-9.]*" $R/${tag}_ref/run.log | head -1 | sed 's/.*dist=//')
echo "[$tag] U_ref=$uref  L_ref(cont)=$lref  disc=$dref"
i=0
for p in $pcts; do
  thr=$(python3 -c "print(round($p/100*$uref, 6))")
  $HERE/run.sh $scen ${tag}_p$p $H100 num_agents=2 unc_radius_threshold=$thr emit_figures=true emit_csv=true track_comm_events=true "$@" >/dev/null &
  i=$((i+1)); [ $((i % 4)) -eq 0 ] && wait
done
wait
printf "%-5s %-9s %-9s %-9s %-8s %-8s %-10s\n" pct thr disc_len cont_len unc iters status
for p in $pcts; do
  d=$R/${tag}_p$p; r=$d/hexspline_cl/results.yaml
  thr=$(python3 -c "print(round($p/100*$uref, 3))")
  disc=$(grep -o "FEASIBLE SOLUTION at iter [0-9]*: dist=[0-9.]*" $d/run.log | head -1 | sed 's/.*dist=//')
  cl=$(grep "^  primary_length" $r | awk '{print $2}' | cut -c1-8)
  u=$(grep "^  primary_unc" $r | awk '{print $2}' | cut -c1-6)
  it=$(grep astar_iterations $r | awk '{print $2}')
  st=$(grep refinement_status $r | awk '{print $2}')
  printf "%-5s %-9s %-9s %-9s %-8s %-8s %-10s\n" $p $thr "${disc:-none}" "${cl:-null}" "${u:-null}" "$it" "${st:-}"
done
