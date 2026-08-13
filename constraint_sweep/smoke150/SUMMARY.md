# Constraint sweep — smoke150

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
| hexspline_cl | 100% | 5/6 | 0.833 | 1.0156 | 1.0424 | 21.3 s | 1.583 |
| hexspline_cl | 90% | 5/6 | 0.833 | 1.0392 | 1.0162 | 20.7 s | 1.540 |
| hexspline_cl | 80% | 5/6 | 0.833 | 1.0515 | 1.0408 | 20.4 s | 1.517 |
| sequential | 100% | 5/6 | 0.833 | 1.1578 | 1.0912 | 13.6 s | 1.007 |
| sequential | 90% | 5/6 | 0.833 | 1.1849 | 1.2207 | 13.1 s | 0.970 |
| sequential | 80% | 5/6 | 0.833 | 1.1790 | 1.1929 | 13.1 s | 0.969 |
| formation | 100% | 6/6 | 1.000 | 1.2792 | 1.2673 | 14.1 s | 1.040 |
| formation | 90% | 4/6 | 0.667 | 1.9757 | 1.8215 | 14.3 s | 1.064 |
| formation | 80% | 3/6 | 0.500 | 2.3023 | 2.2600 | 15.1 s | 1.125 |
| straight_cont | 100% | 0/6 | 0.000 | — | — | — | — |
| straight_cont | 90% | 0/6 | 0.000 | — | — | — | — |
| straight_cont | 80% | 0/6 | 0.000 | — | — | — | — |
