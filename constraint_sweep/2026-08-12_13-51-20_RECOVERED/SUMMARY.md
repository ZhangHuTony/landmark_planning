# Constraint sweep — 2026-08-12_13-51-20

`length_ratio` = the method's primary path length divided by L_ref, the
length of the unconstrained single-agent reference path for that same
scenario. Constraint at level pct is `unc_radius_threshold = pct/100 * U_ref`,
where U_ref is the uncertainty of that reference path's DISCRETE seed — the
stage the threshold gates. scenarios.csv logs the refined U_ref_cont beside it.
Means are over SUCCESSFUL runs only, so read them next to n_success/n_total.

`wall_ratio` = this run's wall clock / the reference run's, the time cost in
the same currency as length_ratio. Both are whole subprocesses, so each
carries the same fixed overhead (Julia startup + graph build + refinement,
~17.7 s on the 600-800 m band) — that floor largely divides out, but it does
compress the ratio, so a method at 2.0x is doing far more than 2x the search.
`fail_reason` distinguishes a real failure from `not_run_after_fail` (method
dropped after method_patience failures) and `not_run_below_stop`.

| method | constraint % | success | rate | mean ratio | median ratio | mean wall | mean wall ratio |
|---|---|---|---|---|---|---|---|
| hexspline_cl | 100% | 26/30 | 0.867 | 1.0420 | 1.0303 | 36.5 s | 2.606 |
| hexspline_cl | 90% | 26/30 | 0.867 | 1.0435 | 1.0377 | 35.3 s | 2.526 |
| hexspline_cl | 80% | 25/30 | 0.833 | 1.0628 | 1.0473 | 34.6 s | 2.472 |
| hexspline_cl | 70% | 23/30 | 0.767 | 1.1197 | 1.0996 | 47.0 s | 3.350 |
| hexspline_cl | 60% | 15/30 | 0.500 | 1.1129 | 1.0845 | 45.0 s | 3.208 |
| hexspline_cl | 50% | 10/30 | 0.333 | 1.1603 | 1.1226 | 65.1 s | 4.610 |
| hexspline_cl | 40% | 5/30 | 0.167 | 1.1713 | 1.1619 | 65.3 s | 4.668 |
| hexspline_cl | 30% | 3/30 | 0.100 | 1.2623 | 1.1678 | 101.6 s | 7.318 |
| straight_cont | 100% | 8/30 | 0.267 | 0.9182 | 0.9610 | 18.8 s | 1.357 |
| straight_cont | 90% | 6/30 | 0.200 | 0.9505 | 0.9813 | 18.7 s | 1.345 |
| straight_cont | 80% | 4/30 | 0.133 | 0.9298 | 0.9721 | 18.0 s | 1.294 |
| straight_cont | 70% | 2/30 | 0.067 | 0.9728 | 0.9728 | 18.1 s | 1.303 |
| straight_cont | 60% | 2/30 | 0.067 | 0.9729 | 0.9729 | 18.0 s | 1.298 |
| straight_cont | 50% | 2/30 | 0.067 | 0.9744 | 0.9744 | 18.0 s | 1.298 |
| straight_cont | 40% | 2/30 | 0.067 | 0.9748 | 0.9748 | 17.2 s | 1.237 |
| straight_cont | 30% | 1/30 | 0.033 | 0.9529 | 0.9529 | 17.3 s | 1.266 |
| discrete_only | 100% | 21/30 | 0.700 | 1.1138 | 1.0938 | 38.5 s | 2.746 |
| discrete_only | 90% | 21/30 | 0.700 | 1.1138 | 1.0938 | 37.3 s | 2.664 |
| discrete_only | 80% | 15/30 | 0.500 | 1.1186 | 1.1068 | 38.3 s | 2.746 |
| discrete_only | 70% | 8/30 | 0.267 | 1.1435 | 1.1379 | 47.8 s | 3.448 |
| discrete_only | 60% | 5/30 | 0.167 | 1.1014 | 1.0772 | 49.3 s | 3.553 |
| discrete_only | 50% | 2/30 | 0.067 | 1.1231 | 1.1231 | 61.9 s | 4.462 |
| discrete_only | 40% | 1/30 | 0.033 | 1.1939 | 1.1939 | 104.1 s | 7.535 |
| discrete_only | 30% | 1/30 | 0.033 | 1.1939 | 1.1939 | 124.8 s | 9.035 |
| greedy | 100% | 22/30 | 0.733 | 1.0985 | 1.0896 | 17.4 s | 1.245 |
| greedy | 90% | 20/30 | 0.667 | 1.0925 | 1.0877 | 16.6 s | 1.190 |
| greedy | 80% | 15/30 | 0.500 | 1.0954 | 1.0644 | 16.9 s | 1.203 |
| greedy | 70% | 9/30 | 0.300 | 1.1100 | 1.0716 | 15.7 s | 1.121 |
| greedy | 60% | 7/30 | 0.233 | 1.1175 | 1.0880 | 16.1 s | 1.159 |
| greedy | 50% | 3/30 | 0.100 | 1.1330 | 1.1427 | 15.7 s | 1.132 |
| greedy | 40% | 0/30 | 0.000 | — | — | — | — |
| greedy | 30% | 0/30 | 0.000 | — | — | — | — |
| formation | 100% | 28/30 | 0.933 | 1.1310 | 1.1283 | 16.7 s | 1.196 |
| formation | 90% | 22/30 | 0.733 | 1.4515 | 1.3670 | 16.9 s | 1.208 |
| formation | 80% | 16/30 | 0.533 | 1.3785 | 1.3884 | 16.7 s | 1.200 |
| formation | 70% | 14/30 | 0.467 | 1.4594 | 1.4642 | 16.4 s | 1.183 |
| formation | 60% | 8/30 | 0.267 | 1.6206 | 1.4465 | 16.6 s | 1.197 |
| formation | 50% | 6/30 | 0.200 | 1.8221 | 1.6658 | 15.9 s | 1.147 |
| formation | 40% | 4/30 | 0.133 | 1.9347 | 1.7012 | 16.0 s | 1.161 |
| formation | 30% | 2/30 | 0.067 | 2.2650 | 2.2650 | 15.6 s | 1.144 |
| sequential | 100% | 29/30 | 0.967 | 1.0748 | 1.0676 | 18.0 s | 1.283 |
| sequential | 90% | 29/30 | 0.967 | 1.0898 | 1.0873 | 17.1 s | 1.225 |
| sequential | 80% | 28/30 | 0.933 | 1.0978 | 1.1019 | 16.5 s | 1.178 |
| sequential | 70% | 27/30 | 0.900 | 1.1270 | 1.1108 | 18.5 s | 1.320 |
| sequential | 60% | 18/30 | 0.600 | 1.1603 | 1.1202 | 20.2 s | 1.457 |
| sequential | 50% | 8/30 | 0.267 | 1.1305 | 1.0900 | 21.7 s | 1.569 |
| sequential | 40% | 6/30 | 0.200 | 1.2078 | 1.1855 | 18.6 s | 1.348 |
| sequential | 30% | 2/30 | 0.067 | 1.1719 | 1.1719 | 36.8 s | 2.604 |
| clgbt | 100% | 12/30 | 0.400 | 1.6805 | 1.6159 | 16.1 s | 1.160 |
| clgbt | 90% | 8/30 | 0.267 | 1.6774 | 1.6494 | 16.3 s | 1.172 |
| clgbt | 80% | 4/30 | 0.133 | 2.0218 | 2.0495 | 16.9 s | 1.220 |
| clgbt | 70% | 2/30 | 0.067 | 2.1194 | 2.1194 | 16.8 s | 1.230 |
| clgbt | 60% | 1/30 | 0.033 | 2.0572 | 2.0572 | 17.5 s | 1.279 |
| clgbt | 50% | 0/30 | 0.000 | — | — | — | — |
| clgbt | 40% | 0/30 | 0.000 | — | — | — | — |
| clgbt | 30% | 0/30 | 0.000 | — | — | — | — |
