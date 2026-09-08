# Constraint sweep — shapes_v14

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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0132 | 1.0132 | 21.9 s | 1.389 |
| hexspline_cl | 90% | 1/1 | 1.000 | 1.0131 | 1.0131 | 21.7 s | 1.376 |
| hexspline_cl | 80% | 1/1 | 1.000 | 1.0133 | 1.0133 | 21.8 s | 1.382 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0267 | 1.0267 | 18.3 s | 1.158 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0299 | 1.0299 | 21.6 s | 1.371 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0320 | 1.0320 | 24.0 s | 1.522 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.1120 | 1.1120 | 70.2 s | 4.454 |
| hexspline_cl | 30% | 1/1 | 1.000 | 1.1131 | 1.1131 | 69.1 s | 4.382 |
| greedy | 100% | 1/1 | 1.000 | 1.0192 | 1.0192 | 14.8 s | 0.939 |
| greedy | 90% | 1/1 | 1.000 | 1.0192 | 1.0192 | 14.3 s | 0.906 |
| greedy | 80% | 0/1 | 0.000 | — | — | — | — |
| greedy | 70% | 0/1 | 0.000 | — | — | — | — |
| greedy | 60% | 0/1 | 0.000 | — | — | — | — |
| greedy | 50% | 0/1 | 0.000 | — | — | — | — |
| greedy | 40% | 0/1 | 0.000 | — | — | — | — |
| greedy | 30% | 0/1 | 0.000 | — | — | — | — |
| formation | 100% | 0/1 | 0.000 | — | — | — | — |
| formation | 90% | 0/1 | 0.000 | — | — | — | — |
| formation | 80% | 0/1 | 0.000 | — | — | — | — |
| formation | 70% | 0/1 | 0.000 | — | — | — | — |
| formation | 60% | 0/1 | 0.000 | — | — | — | — |
| formation | 50% | 0/1 | 0.000 | — | — | — | — |
| formation | 40% | 0/1 | 0.000 | — | — | — | — |
| formation | 30% | 0/1 | 0.000 | — | — | — | — |
| sequential | 100% | 0/1 | 0.000 | — | — | — | — |
| sequential | 90% | 0/1 | 0.000 | — | — | — | — |
| sequential | 80% | 0/1 | 0.000 | — | — | — | — |
| sequential | 70% | 0/1 | 0.000 | — | — | — | — |
| sequential | 60% | 0/1 | 0.000 | — | — | — | — |
| sequential | 50% | 0/1 | 0.000 | — | — | — | — |
| sequential | 40% | 0/1 | 0.000 | — | — | — | — |
| sequential | 30% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 100% | 1/1 | 1.000 | 1.0140 | 1.0140 | 29.3 s | 1.859 |
| clgbt | 90% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 80% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 70% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 60% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 50% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 40% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 30% | 0/1 | 0.000 | — | — | — | — |
