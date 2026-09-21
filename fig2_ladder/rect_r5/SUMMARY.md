# Constraint sweep — rect_r5

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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0127 | 1.0127 | 23.4 s | 1.370 |
| hexspline_cl | 90% | 1/1 | 1.000 | 1.0125 | 1.0125 | 23.4 s | 1.368 |
| hexspline_cl | 80% | 1/1 | 1.000 | 1.0127 | 1.0127 | 23.4 s | 1.371 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0271 | 1.0271 | 17.6 s | 1.032 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0478 | 1.0478 | 19.2 s | 1.124 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0597 | 1.0597 | 18.4 s | 1.078 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.1163 | 1.1163 | 29.2 s | 1.707 |
| hexspline_cl | 30% | 1/1 | 1.000 | 1.1163 | 1.1163 | 28.7 s | 1.679 |
