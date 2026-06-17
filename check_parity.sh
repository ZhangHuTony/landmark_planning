#!/usr/bin/env bash
# check_parity.sh — compare current results_* CSVs and run.log against .baseline copies.
# Usage: bash check_parity.sh
# Exit 0 = bit-for-bit identical; non-zero = diff found (prints differences).
set -euo pipefail

PASS=0
for s in single dual clustered shoreline; do
    echo "── Checking $s ──"
    diff_out=$(diff -r \
        --exclude="*.png" \
        "results_${s}.baseline" \
        "results_${s}" 2>&1 || true)
    if [ -z "$diff_out" ]; then
        echo "  ✓ $s: identical"
    else
        echo "  ✗ $s: DIFFERENCES FOUND"
        echo "$diff_out"
        PASS=1
    fi
done

if [ $PASS -eq 0 ]; then
    echo ""
    echo "✓ ALL SCENARIOS PASS PARITY"
else
    echo ""
    echo "✗ PARITY FAILURES DETECTED — review diffs above"
fi
exit $PASS
