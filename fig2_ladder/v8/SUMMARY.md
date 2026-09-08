# Constraint sweep — v8

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
| hexspline_cl | 100% | 1/1 | 1.000 | 1.0014 | 1.0014 | 21.7 s | 1.331 |
| hexspline_cl | 90% | 1/1 | 1.000 | 1.0008 | 1.0008 | 22.0 s | 1.351 |
| hexspline_cl | 80% | 1/1 | 1.000 | 1.0012 | 1.0012 | 21.3 s | 1.310 |
| hexspline_cl | 70% | 1/1 | 1.000 | 1.0031 | 1.0031 | 19.5 s | 1.197 |
| hexspline_cl | 60% | 1/1 | 1.000 | 1.0652 | 1.0652 | 17.8 s | 1.090 |
| hexspline_cl | 50% | 1/1 | 1.000 | 1.0563 | 1.0563 | 18.4 s | 1.130 |
| hexspline_cl | 40% | 1/1 | 1.000 | 1.0644 | 1.0644 | 17.8 s | 1.091 |
| hexspline_cl | 30% | 1/1 | 1.000 | 1.0648 | 1.0648 | 17.8 s | 1.092 |
| greedy | 100% | 1/1 | 1.000 | 1.0244 | 1.0244 | 15.3 s | 0.942 |
| greedy | 90% | 1/1 | 1.000 | 1.0244 | 1.0244 | 15.7 s | 0.965 |
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
| clgbt | 100% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 90% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 80% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 70% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 60% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 50% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 40% | 0/1 | 0.000 | — | — | — | — |
| clgbt | 30% | 0/1 | 0.000 | — | — | — | — |
