# Constraint sweep — baseline_2026-08-17

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
| hexspline_cl | 100% | 50/50 | 1.000 | 1.1410 | 1.1276 | 20.7 s | 1.329 |
| hexspline_cl | 90% | 50/50 | 1.000 | 1.1539 | 1.1306 | 20.5 s | 1.320 |
| hexspline_cl | 80% | 50/50 | 1.000 | 1.1665 | 1.1385 | 20.9 s | 1.342 |
| hexspline_cl | 70% | 50/50 | 1.000 | 1.2194 | 1.1675 | 21.8 s | 1.403 |
| hexspline_cl | 60% | 48/50 | 0.960 | 1.3014 | 1.2417 | 29.0 s | 1.861 |
| hexspline_cl | 50% | 40/50 | 0.800 | 1.3352 | 1.3047 | 28.4 s | 1.824 |
| hexspline_cl | 40% | 30/50 | 0.600 | 1.3350 | 1.3199 | 22.7 s | 1.461 |
| hexspline_cl | 30% | 20/50 | 0.400 | 1.4096 | 1.4077 | 21.6 s | 1.396 |
| greedy | 100% | 45/50 | 0.900 | 1.2363 | 1.2191 | 15.1 s | 0.978 |
| greedy | 90% | 13/50 | 0.260 | 1.1795 | 1.1646 | 15.6 s | 1.008 |
| greedy | 80% | 10/50 | 0.200 | 1.2162 | 1.2570 | 15.6 s | 1.009 |
| greedy | 70% | 7/50 | 0.140 | 1.2100 | 1.2505 | 15.7 s | 1.021 |
| greedy | 60% | 4/50 | 0.080 | 1.2025 | 1.2033 | 15.7 s | 1.023 |
| greedy | 50% | 2/50 | 0.040 | 1.1329 | 1.1329 | 16.7 s | 1.084 |
| greedy | 40% | 2/50 | 0.040 | 1.1344 | 1.1344 | 16.7 s | 1.081 |
| greedy | 30% | 2/50 | 0.040 | 1.1333 | 1.1333 | 16.5 s | 1.071 |
| formation | 100% | 49/50 | 0.980 | 1.2204 | 1.1906 | 14.3 s | 0.922 |
| formation | 90% | 47/50 | 0.940 | 1.2371 | 1.1906 | 14.3 s | 0.923 |
| formation | 80% | 45/50 | 0.900 | 1.3383 | 1.2376 | 14.3 s | 0.923 |
| formation | 70% | 36/50 | 0.720 | 1.3393 | 1.2410 | 14.5 s | 0.936 |
| formation | 60% | 31/50 | 0.620 | 1.3666 | 1.2896 | 14.3 s | 0.923 |
| formation | 50% | 27/50 | 0.540 | 1.3263 | 1.2987 | 14.4 s | 0.927 |
| formation | 40% | 16/50 | 0.320 | 1.4465 | 1.3615 | 14.0 s | 0.906 |
| formation | 30% | 11/50 | 0.220 | 1.4783 | 1.3442 | 13.7 s | 0.891 |
| sequential | 100% | 50/50 | 1.000 | 1.1821 | 1.1530 | 14.5 s | 0.934 |
| sequential | 90% | 50/50 | 1.000 | 1.1980 | 1.1930 | 14.4 s | 0.930 |
| sequential | 80% | 50/50 | 1.000 | 1.2375 | 1.2003 | 14.3 s | 0.924 |
| sequential | 70% | 47/50 | 0.940 | 1.4141 | 1.1947 | 14.4 s | 0.930 |
| sequential | 60% | 37/50 | 0.740 | 1.3870 | 1.2175 | 14.2 s | 0.915 |
| sequential | 50% | 26/50 | 0.520 | 1.3649 | 1.2209 | 14.2 s | 0.919 |
| sequential | 40% | 12/50 | 0.240 | 2.4913 | 2.0720 | 14.2 s | 0.915 |
| sequential | 30% | 8/50 | 0.160 | 2.7601 | 2.4386 | 14.1 s | 0.912 |
| clgbt | 100% | 50/50 | 1.000 | 1.4687 | 1.3762 | 16.2 s | 1.046 |
| clgbt | 90% | 45/50 | 0.900 | 1.5823 | 1.5062 | 19.6 s | 1.262 |
| clgbt | 80% | 40/50 | 0.800 | 1.7303 | 1.6720 | 20.3 s | 1.306 |
| clgbt | 70% | 35/50 | 0.700 | 1.7770 | 1.6985 | 74.2 s | 4.753 |
| clgbt | 60% | 25/50 | 0.500 | 1.8432 | 1.7401 | 61.7 s | 3.977 |
| clgbt | 50% | 16/50 | 0.320 | 2.0097 | 2.0211 | 120.7 s | 7.811 |
| clgbt | 40% | 5/50 | 0.100 | 1.9762 | 1.8695 | 71.5 s | 4.608 |
| clgbt | 30% | 3/50 | 0.060 | 2.0731 | 2.1787 | 17.3 s | 1.120 |
