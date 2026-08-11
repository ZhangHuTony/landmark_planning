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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0007 | 1.0007 | 35.5 s | 1.983 |
| straight_cont | 100% | 0/1 | 0.000 | — | — | — | — |
| discrete_only | 100% | 1/1 | 1.000 | 1.0448 | 1.0448 | 33.2 s | 1.855 |
| greedy | 100% | 1/1 | 1.000 | 1.0452 | 1.0452 | 18.1 s | 1.012 |
| formation | 100% | 1/1 | 1.000 | 1.0656 | 1.0656 | 18.2 s | 1.017 |
| sequential | 100% | 1/1 | 1.000 | 0.9662 | 0.9662 | 18.2 s | 1.016 |
