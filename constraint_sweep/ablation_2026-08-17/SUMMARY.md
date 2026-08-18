# Constraint sweep — ablation_2026-08-17

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
| hexspline_cl | 100% | 30/30 | 1.000 | 1.1210 | 1.0851 | 19.7 s | 1.271 |
| hexspline_cl | 90% | 30/30 | 1.000 | 1.1232 | 1.0918 | 19.7 s | 1.271 |
| hexspline_cl | 80% | 30/30 | 1.000 | 1.1504 | 1.0972 | 20.5 s | 1.321 |
| hexspline_cl | 70% | 30/30 | 1.000 | 1.1737 | 1.1104 | 21.0 s | 1.354 |
| hexspline_cl | 60% | 29/30 | 0.967 | 1.1811 | 1.1356 | 23.2 s | 1.495 |
| hexspline_cl | 50% | 26/30 | 0.867 | 1.2246 | 1.1596 | 26.6 s | 1.715 |
| hexspline_cl | 40% | 18/30 | 0.600 | 1.3632 | 1.2326 | 22.8 s | 1.477 |
| hexspline_cl | 30% | 13/30 | 0.433 | 1.2913 | 1.2605 | 19.9 s | 1.287 |
| straight_cont | 100% | 16/30 | 0.533 | 0.9934 | 1.0000 | 17.5 s | 1.136 |
| straight_cont | 90% | 10/30 | 0.333 | 0.9830 | 0.9997 | 17.9 s | 1.157 |
| straight_cont | 80% | 7/30 | 0.233 | 1.0031 | 1.0010 | 17.8 s | 1.157 |
| straight_cont | 70% | 2/30 | 0.067 | 0.9988 | 0.9988 | 18.2 s | 1.196 |
| straight_cont | 60% | 1/30 | 0.033 | 0.9982 | 0.9982 | 18.7 s | 1.219 |
| straight_cont | 50% | 0/30 | 0.000 | — | — | — | — |
| straight_cont | 40% | 0/30 | 0.000 | — | — | — | — |
| straight_cont | 30% | 0/30 | 0.000 | — | — | — | — |
| discrete_only | 100% | 30/30 | 1.000 | 1.1958 | 1.1529 | 18.4 s | 1.188 |
| discrete_only | 90% | 30/30 | 1.000 | 1.1976 | 1.1529 | 18.3 s | 1.183 |
| discrete_only | 80% | 30/30 | 1.000 | 1.2135 | 1.1471 | 19.2 s | 1.242 |
| discrete_only | 70% | 30/30 | 1.000 | 1.2279 | 1.1529 | 19.7 s | 1.269 |
| discrete_only | 60% | 28/30 | 0.933 | 1.2157 | 1.1471 | 19.1 s | 1.237 |
| discrete_only | 50% | 24/30 | 0.800 | 1.2462 | 1.1553 | 19.7 s | 1.275 |
| discrete_only | 40% | 18/30 | 0.600 | 1.3698 | 1.2437 | 21.9 s | 1.419 |
| discrete_only | 30% | 13/30 | 0.433 | 1.2922 | 1.2605 | 19.2 s | 1.243 |
