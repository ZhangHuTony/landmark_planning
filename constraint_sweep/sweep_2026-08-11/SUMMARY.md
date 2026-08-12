# Constraint sweep — sweep_2026-08-11

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
| hexspline_cl | 100% | 74/100 | 0.740 | 1.0272 | 1.0199 | 33.8 s | 2.068 |
| hexspline_cl | 95% | 74/100 | 0.740 | 1.0230 | 1.0204 | 33.9 s | 2.071 |
| hexspline_cl | 90% | 74/100 | 0.740 | 1.0314 | 1.0225 | 34.6 s | 2.116 |
| hexspline_cl | 85% | 74/100 | 0.740 | 1.0350 | 1.0379 | 35.3 s | 2.159 |
| hexspline_cl | 80% | 71/100 | 0.710 | 1.0512 | 1.0410 | 35.2 s | 2.152 |
| hexspline_cl | 75% | 69/100 | 0.690 | 1.0714 | 1.0482 | 38.7 s | 2.367 |
| hexspline_cl | 70% | 63/100 | 0.630 | 1.0937 | 1.0747 | 41.5 s | 2.538 |
| hexspline_cl | 65% | 55/100 | 0.550 | 1.0980 | 1.0678 | 40.4 s | 2.473 |
| hexspline_cl | 60% | 43/100 | 0.430 | 1.1000 | 1.0831 | 42.4 s | 2.589 |
| hexspline_cl | 55% | 32/100 | 0.320 | 1.1145 | 1.1077 | 42.5 s | 2.596 |
| hexspline_cl | 50% | 24/100 | 0.240 | 1.1118 | 1.0684 | 41.1 s | 2.501 |
| hexspline_cl | 45% | 20/100 | 0.200 | 1.1358 | 1.1049 | 43.8 s | 2.655 |
| hexspline_cl | 40% | 11/100 | 0.110 | 1.0977 | 1.0449 | 47.5 s | 2.890 |
| hexspline_cl | 35% | 9/100 | 0.090 | 1.1346 | 1.1147 | 67.8 s | 4.124 |
| hexspline_cl | 30% | 6/100 | 0.060 | 1.1820 | 1.1361 | 62.8 s | 3.820 |
| straight_cont | 100% | 20/100 | 0.200 | 0.9119 | 0.9467 | 18.3 s | 1.123 |
| straight_cont | 95% | 15/100 | 0.150 | 0.9119 | 0.9507 | 18.3 s | 1.124 |
| straight_cont | 90% | 13/100 | 0.130 | 0.9189 | 0.9722 | 18.4 s | 1.127 |
| straight_cont | 85% | 11/100 | 0.110 | 0.9454 | 0.9912 | 17.9 s | 1.096 |
| straight_cont | 80% | 5/100 | 0.050 | 0.9438 | 0.9916 | 18.3 s | 1.123 |
| straight_cont | 75% | 3/100 | 0.030 | 0.9818 | 0.9928 | 18.1 s | 1.105 |
| straight_cont | 70% | 3/100 | 0.030 | 0.9819 | 0.9930 | 18.1 s | 1.106 |
| straight_cont | 65% | 3/100 | 0.030 | 0.9820 | 0.9934 | 18.1 s | 1.108 |
| straight_cont | 60% | 3/100 | 0.030 | 0.9820 | 0.9933 | 18.2 s | 1.109 |
| straight_cont | 55% | 3/100 | 0.030 | 0.9825 | 0.9948 | 18.1 s | 1.105 |
| straight_cont | 50% | 3/100 | 0.030 | 0.9829 | 0.9961 | 18.2 s | 1.108 |
| straight_cont | 45% | 3/100 | 0.030 | 0.9830 | 0.9963 | 18.1 s | 1.108 |
| straight_cont | 40% | 2/100 | 0.020 | 0.9748 | 0.9748 | 18.3 s | 1.123 |
| straight_cont | 35% | 2/100 | 0.020 | 0.9752 | 0.9752 | 18.4 s | 1.130 |
| straight_cont | 30% | 1/100 | 0.010 | 0.9529 | 0.9529 | 19.6 s | 1.231 |
| discrete_only | 100% | 61/100 | 0.610 | 1.0989 | 1.0924 | 33.6 s | 2.047 |
| discrete_only | 95% | 56/100 | 0.560 | 1.1002 | 1.0908 | 33.2 s | 2.022 |
| discrete_only | 90% | 53/100 | 0.530 | 1.0982 | 1.0825 | 35.2 s | 2.141 |
| discrete_only | 85% | 44/100 | 0.440 | 1.0954 | 1.0815 | 34.7 s | 2.113 |
| discrete_only | 80% | 36/100 | 0.360 | 1.0904 | 1.0781 | 32.7 s | 1.993 |
| discrete_only | 75% | 27/100 | 0.270 | 1.0786 | 1.0664 | 28.1 s | 1.717 |
| discrete_only | 70% | 22/100 | 0.220 | 1.1044 | 1.0789 | 32.9 s | 2.014 |
| discrete_only | 65% | 14/100 | 0.140 | 1.0996 | 1.0608 | 28.8 s | 1.763 |
| discrete_only | 60% | 10/100 | 0.100 | 1.0783 | 1.0608 | 28.7 s | 1.748 |
| discrete_only | 55% | 9/100 | 0.090 | 1.0751 | 1.0595 | 30.6 s | 1.864 |
| discrete_only | 50% | 6/100 | 0.060 | 1.0788 | 1.0560 | 22.3 s | 1.355 |
| discrete_only | 45% | 2/100 | 0.020 | 1.0433 | 1.0433 | 18.8 s | 1.132 |
| discrete_only | 40% | 0/100 | 0.000 | — | — | — | — |
| discrete_only | 35% | 0/100 | 0.000 | — | — | — | — |
| discrete_only | 30% | 0/100 | 0.000 | — | — | — | — |
| greedy | 100% | 73/100 | 0.730 | 1.0947 | 1.0811 | 17.6 s | 1.071 |
| greedy | 95% | 70/100 | 0.700 | 1.1042 | 1.0874 | 17.5 s | 1.067 |
| greedy | 90% | 64/100 | 0.640 | 1.0980 | 1.0731 | 17.6 s | 1.072 |
| greedy | 85% | 63/100 | 0.630 | 1.1041 | 1.0744 | 17.4 s | 1.058 |
| greedy | 80% | 52/100 | 0.520 | 1.1047 | 1.0737 | 17.6 s | 1.068 |
| greedy | 75% | 38/100 | 0.380 | 1.1028 | 1.0718 | 17.3 s | 1.051 |
| greedy | 70% | 30/100 | 0.300 | 1.0935 | 1.0622 | 17.2 s | 1.046 |
| greedy | 65% | 22/100 | 0.220 | 1.0830 | 1.0604 | 17.1 s | 1.045 |
| greedy | 60% | 19/100 | 0.190 | 1.0751 | 1.0554 | 17.2 s | 1.054 |
| greedy | 55% | 10/100 | 0.100 | 1.0798 | 1.0528 | 16.7 s | 1.018 |
| greedy | 50% | 6/100 | 0.060 | 1.0649 | 1.0565 | 17.3 s | 1.054 |
| greedy | 45% | 4/100 | 0.040 | 1.0496 | 1.0303 | 16.8 s | 1.026 |
| greedy | 40% | 2/100 | 0.020 | 1.0162 | 1.0162 | 17.0 s | 1.026 |
| greedy | 35% | 2/100 | 0.020 | 1.0181 | 1.0181 | 16.9 s | 1.025 |
| greedy | 30% | 1/100 | 0.010 | 1.0001 | 1.0001 | 16.3 s | 0.987 |
| formation | 100% | 96/100 | 0.960 | 1.1367 | 1.1283 | 16.7 s | 1.021 |
| formation | 95% | 83/100 | 0.830 | 1.4244 | 1.3197 | 18.0 s | 1.095 |
| formation | 90% | 76/100 | 0.760 | 1.4676 | 1.3303 | 18.0 s | 1.100 |
| formation | 85% | 69/100 | 0.690 | 1.4895 | 1.3529 | 18.6 s | 1.135 |
| formation | 80% | 64/100 | 0.640 | 1.4747 | 1.3656 | 18.6 s | 1.136 |
| formation | 75% | 56/100 | 0.560 | 1.4854 | 1.3983 | 18.7 s | 1.141 |
| formation | 70% | 46/100 | 0.460 | 1.4981 | 1.4434 | 18.3 s | 1.118 |
| formation | 65% | 36/100 | 0.360 | 1.5500 | 1.4765 | 18.8 s | 1.146 |
| formation | 60% | 25/100 | 0.250 | 1.6101 | 1.4212 | 18.2 s | 1.113 |
| formation | 55% | 17/100 | 0.170 | 1.5566 | 1.3966 | 17.9 s | 1.089 |
| formation | 50% | 13/100 | 0.130 | 1.6396 | 1.4988 | 18.3 s | 1.112 |
| formation | 45% | 10/100 | 0.100 | 1.7274 | 1.5107 | 19.8 s | 1.205 |
| formation | 40% | 9/100 | 0.090 | 1.8116 | 1.5401 | 18.3 s | 1.110 |
| formation | 35% | 5/100 | 0.050 | 2.0039 | 1.8622 | 18.3 s | 1.116 |
| formation | 30% | 2/100 | 0.020 | 2.2650 | 2.2650 | 17.7 s | 1.106 |
| sequential | 100% | 92/100 | 0.920 | 1.0618 | 1.0395 | 20.5 s | 1.253 |
| sequential | 95% | 91/100 | 0.910 | 1.0667 | 1.0638 | 20.3 s | 1.241 |
| sequential | 90% | 90/100 | 0.900 | 1.0737 | 1.0695 | 20.2 s | 1.233 |
| sequential | 85% | 90/100 | 0.900 | 1.0746 | 1.0691 | 20.3 s | 1.236 |
| sequential | 80% | 88/100 | 0.880 | 1.0829 | 1.0903 | 20.3 s | 1.237 |
| sequential | 75% | 87/100 | 0.870 | 1.1188 | 1.0985 | 20.7 s | 1.260 |
| sequential | 70% | 81/100 | 0.810 | 1.1317 | 1.1100 | 22.0 s | 1.342 |
| sequential | 65% | 63/100 | 0.630 | 1.1459 | 1.0906 | 23.0 s | 1.407 |
| sequential | 60% | 50/100 | 0.500 | 1.1617 | 1.1156 | 25.1 s | 1.534 |
| sequential | 55% | 32/100 | 0.320 | 1.1919 | 1.0999 | 27.4 s | 1.680 |
| sequential | 50% | 24/100 | 0.240 | 1.1254 | 1.0875 | 30.8 s | 1.887 |
| sequential | 45% | 21/100 | 0.210 | 1.2412 | 1.1991 | 23.1 s | 1.413 |
| sequential | 40% | 13/100 | 0.130 | 1.2188 | 1.1676 | 28.5 s | 1.728 |
| sequential | 35% | 7/100 | 0.070 | 1.3985 | 1.3718 | 40.1 s | 2.420 |
| sequential | 30% | 3/100 | 0.030 | 1.1606 | 1.1380 | 61.1 s | 3.685 |
