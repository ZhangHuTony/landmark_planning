# Constraint sweep — rect_r1

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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0157 | 1.0157 | 20.4 s | 1.232 |
| hexspline_cl | 90% | 1/1 | 1.000 | 1.0157 | 1.0157 | 20.3 s | 1.222 |
| hexspline_cl | 80% | 1/1 | 1.000 | 1.0157 | 1.0157 | 20.6 s | 1.246 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0275 | 1.0275 | 17.0 s | 1.025 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0484 | 1.0484 | 20.1 s | 1.211 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0630 | 1.0630 | 19.1 s | 1.153 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.1030 | 1.1030 | 29.1 s | 1.757 |
| hexspline_cl | 30% | 1/1 | 1.000 | 1.1028 | 1.1028 | 29.4 s | 1.772 |
