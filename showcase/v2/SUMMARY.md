# Constraint sweep — v2

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
| hexspline_cl | 75% | 1/1 | 1.000 | 1.0122 | 1.0122 | 24.0 s | 1.253 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0079 | 1.0079 | 24.7 s | 1.293 |
| hexspline_cl | 65% | 1/1 | 1.000 | 1.0861 | 1.0861 | 23.8 s | 1.243 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0778 | 1.0778 | 42.6 s | 2.225 |
| hexspline_cl | 55% | 1/1 | 1.000 | 1.0738 | 1.0738 | 58.7 s | 3.067 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0411 | 1.0411 | 63.8 s | 3.336 |
| hexspline_cl | 45% | 1/1 | 1.000 | 1.0965 | 1.0965 | 107.8 s | 5.632 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.1009 | 1.1009 | 108.0 s | 5.640 |
| hexspline_cl | 35% | 1/1 | 1.000 | 1.1032 | 1.1032 | 108.2 s | 5.654 |
| hexspline_cl | 30% | 0/1 | 0.000 | — | — | — | — |
| hexspline_cl | 25% | 0/1 | 0.000 | — | — | — | — |
| hexspline_cl | 20% | 0/1 | 0.000 | — | — | — | — |
