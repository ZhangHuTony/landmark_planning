# Constraint sweep — long_v15e

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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0071 | 1.0071 | 54.2 s | 2.785 |
| hexspline_cl | 90% | 1/1 | 1.000 | 1.0069 | 1.0069 | 54.6 s | 2.808 |
| hexspline_cl | 80% | 1/1 | 1.000 | 1.0071 | 1.0071 | 54.1 s | 2.783 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0075 | 1.0075 | 54.2 s | 2.787 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0205 | 1.0205 | 39.6 s | 2.037 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0186 | 1.0186 | 47.4 s | 2.438 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.0170 | 1.0170 | 52.1 s | 2.679 |
| hexspline_cl | 30% | 1/1 | 1.000 | 1.0919 | 1.0919 | 45.1 s | 2.317 |
