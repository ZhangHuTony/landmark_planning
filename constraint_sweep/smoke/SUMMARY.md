# Constraint sweep — smoke

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
| hexspline_cl | 100% | 4/4 | 1.000 | 1.0352 | 1.0393 | 19.4 s | 1.352 |
| hexspline_cl | 90% | 4/4 | 1.000 | 1.0381 | 1.0276 | 18.8 s | 1.305 |
| hexspline_cl | 80% | 4/4 | 1.000 | 1.0773 | 1.0765 | 34.1 s | 2.377 |
| sequential | 100% | 2/4 | 0.500 | 1.0262 | 1.0262 | 13.8 s | 0.942 |
| sequential | 90% | 2/4 | 0.500 | 1.0322 | 1.0322 | 14.3 s | 0.978 |
| sequential | 80% | 2/4 | 0.500 | 1.2611 | 1.2611 | 13.4 s | 0.915 |
| formation | 100% | 4/4 | 1.000 | 1.0903 | 1.0790 | 14.8 s | 1.027 |
| formation | 90% | 4/4 | 1.000 | 1.4793 | 1.4347 | 15.1 s | 1.046 |
| formation | 80% | 4/4 | 1.000 | 1.4521 | 1.4120 | 15.6 s | 1.082 |
| straight_cont | 100% | 0/4 | 0.000 | — | — | — | — |
| straight_cont | 90% | 0/4 | 0.000 | — | — | — | — |
| straight_cont | 80% | 0/4 | 0.000 | — | — | — | — |
