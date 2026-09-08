#!/bin/bash
# run.sh <scenario_file> <tag> key=value ...   → generate_plan.jl into results/fig2/<tag>
set -e
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/../../../../.." && pwd)
scen=$1; tag=$2; shift 2
out=$ROOT/results/fig2/$tag; cfg=$out/_cfg
python3 $HERE/mkcfg.py $scen $cfg "$@" >/dev/null
cd $ROOT
OUTPUT_DIR=$out ~/.juliaup/bin/julia generate_plan.jl $cfg > $out/run.log 2>&1 || echo "rc=$?"
grep -HE "primary_length|primary_unc|primary_disc_unc|refinement_status|astar_iterations" $out/*/results.yaml 2>/dev/null | sed "s/^/[$tag] /"
