# Analysis — sweep 2026-08-12_13-51-20

Derived from the per-method `trials.csv` files **before they were lost** (see
`PROVENANCE.md`). The underlying rows no longer exist, so these tables cannot be
recomputed or extended from this directory.

## 1. hexspline_cl failures are budget exhaustion, not infeasibility

| quantity | value |
|---|---|
| hexspline runs actually executed | 160 |
| …hit the 1,000,000-expansion cap | 38 (24%) |
| hexspline failures (`recovery_failed` + `no_solution`) | 27 |
| …of which hit the cap | **24 (89%)** |
| hexspline *successes* that hit the cap | 14 of 133 (11%) |

Failure-reason breakdown (`not_run_after_fail` = skipped by `method_patience`, not a
real failure):

| method | reasons |
|---|---|
| hexspline_cl | not_run_after_fail 80, recovery_failed 23, no_solution 4 |
| sequential | not_run_after_fail 65, recovery_failed 28 |
| formation | not_run_after_fail 112, recovery_failed 28 |
| greedy | not_run_after_fail 134, recovery_failed 26, no_solution 4 |
| discrete_only | not_run_after_fail 137, unc_violated 25, no_solution 4 |
| straight_cont | not_run_after_fail 184, recovery_failed 29 |
| clgbt | not_run_after_fail 183, no_solution 30 |

Only 3 hexspline failures terminated with budget left — s009 (459,065 it), s011 (85,766),
s015 (359,621) — and in all three the discrete seed was already **under** threshold
(4.77<4.84, 3.369<3.408, 3.284<3.382); those are continuous-refinement failures, not
search failures.

Capped fraction was flat in `goal_dist`: 22% at 600 m, 26% at 700 m, 23% at 800 m.
Median wall clock 90.6 s for capped runs vs 21.8 s for uncapped.

## 2. Four provably-false negatives at pct=100

s005, s012, s029, s030 returned `no_solution` at **pct=100**, where
`threshold == U_ref` — the single-agent reference's own discrete uncertainty. The
2-agent feasible set is a strict superset of the 1-agent one (park the helper;
`ci_omega_det` clamps ω to [0,1] and ω=1 returns `cov_a` unchanged, so CI fusion can
never raise det), so a feasible solution exists by construction. All four burned the
full 1e6 expansions (148.1 / 170.3 / 178.7 / 156.6 s).

This is the same failure mode documented in `config/mc/hexspline_cl.yaml` for the
150k→1.5M raise; at 1e6 it is still live.

## 3. Per-scenario deepest constraint level solved

`-` = never solved at any level. Ladder: 100→30 in steps of 10.

| scen | D | Lm | Ob | hexspline | sequential | greedy | formation | discrete | straight | clgbt |
|---|---|---|---|---|---|---|---|---|---|---|
| s001 | 700 | 4 | 5 | 50 | 70 | 70 | 90 | 90 | – | – |
| s002 | 600 | 4 | 4 | 30 | 40 | 50 | 30 | – | 30 | 60 |
| s003 | 600 | 2 | 3 | 50 | 40 | 60 | 30 | 70 | – | 70 |
| s004 | 700 | 4 | 3 | 60 | 60 | 90 | 70 | 90 | – | 80 |
| s005 | 800 | 3 | 3 | – | 60 | 70 | 50 | – | – | – |
| s006 | 600 | 2 | 3 | 50 | 50 | 60 | 60 | 60 | 90 | – |
| s007 | 700 | 4 | 3 | 60 | 60 | – | 90 | 80 | – | – |
| s008 | 800 | 4 | 4 | 40 | 70 | 60 | 70 | 90 | – | 80 |
| s009 | 800 | 2 | 5 | 70 | 60 | – | 80 | 80 | – | – |
| s010 | 600 | 4 | 5 | 70 | 70 | 100 | 100 | 70 | – | 100 |
| s011 | 600 | 4 | 5 | 60 | 60 | 90 | 100 | 60 | – | – |
| s012 | 800 | 4 | 5 | – | 60 | – | 100 | – | – | – |
| s013 | 600 | 2 | 4 | 70 | 70 | – | 70 | – | 80 | – |
| s014 | 600 | 4 | 3 | 70 | 60 | – | 90 | – | – | – |
| s015 | 800 | 3 | 4 | 70 | 70 | 80 | 90 | 80 | – | 90 |
| s016 | 700 | 4 | 5 | 50 | 50 | 50 | 70 | 80 | 90 | 100 |
| s017 | 700 | 2 | 5 | 80 | 70 | 80 | – | 80 | – | – |
| s018 | 800 | 4 | 4 | 30 | 30 | 60 | 40 | 50 | 40 | – |
| s019 | 800 | 2 | 4 | 70 | 70 | 100 | 100 | 90 | – | – |
| s020 | 600 | 2 | 3 | 60 | – | – | 100 | 70 | 100 | 100 |
| s021 | 700 | 2 | 3 | 60 | 90 | 90 | 70 | 80 | 80 | 90 |
| s022 | 700 | 2 | 4 | 30 | 40 | – | 40 | 30 | – | – |
| s023 | 600 | 3 | 5 | 70 | 60 | 90 | 100 | – | – | 100 |
| s024 | 800 | 4 | 5 | 70 | 60 | 90 | – | 80 | – | – |
| s025 | 600 | 4 | 5 | 80 | 60 | 80 | 80 | – | – | – |
| s026 | 600 | 4 | 4 | 90 | 80 | 80 | 70 | 90 | 100 | – |
| s027 | 600 | 4 | 5 | 50 | 40 | 50 | 50 | 90 | – | 90 |
| s028 | 700 | 3 | 5 | 40 | 30 | 80 | 60 | 60 | – | 90 |
| s029 | 800 | 4 | 4 | – | 70 | – | 90 | – | – | – |
| s030 | 700 | 3 | 5 | – | 70 | 80 | 90 | – | – | – |

Solved-at-all / mean deepest (over solved only): hexspline 26/30 @ 58.8, sequential
29/30 @ 59.3, greedy 22/30 @ 75.5, formation 28/30 @ 74.3, discrete_only 21/30 @ 74.8,
straight_cont 8/30 @ 76.2, clgbt 12/30 @ 87.5.

## 4. Paired head-to-head (unsolved scored as 110)

| comparison | hex deeper | baseline deeper | tie | mean(base − hex) |
|---|---|---|---|---|
| vs **sequential** | 6 | 14 | 10 | **−4.7 pts** |
| vs greedy | 21 | 3 | 6 | +19.0 |
| vs formation | 20 | 6 | 4 | +11.0 |
| vs discrete_only | 21 | 0 | 9 | +19.7 |
| vs straight_cont | 25 | 0 | 5 | +35.3 |
| vs clgbt | 26 | 0 | 4 | +35.3 |

**13 of sequential's 14 wins are against a budget-exhausted hexspline run.** The lone
exception is s009 (459,065 it, `recovery_failed` with disc_unc 4.77 < thr 4.84 — a
refinement failure).

Scenarios where hexspline goes deeper: s001 (50 vs 70), s002 (30 vs 40), s008 (40 vs 70),
s020 (60 vs never), s021 (60 vs 90), s022 (30 vs 40).

## 5. Paired length ratio, hexspline vs sequential, on runs both solved

Means over the *same* (scenario, pct) pairs — unlike `summary.csv`, whose per-method means
are over different success subsets and are therefore not comparable.

| pct | n | hexspline | sequential | hex shorter |
|---|---|---|---|---|
| 100 | 25 | 1.0429 | 1.0652 | 15/25 |
| 90 | 25 | 1.0444 | 1.0804 | 16/25 |
| 80 | 23 | 1.0669 | 1.0879 | 14/23 |
| 70 | 21 | 1.1240 | 1.1181 | 10/21 |
| 60 | 11 | 1.0911 | 1.1200 | 6/11 |
| 50 | 8 | 1.1718 | 1.1305 | 5/8 |
| 40 | 4 | 1.1650 | 1.2022 | 3/4 |
| 30 | 1 | 1.1678 | 1.1199 | 0/1 |
| **all** | **118** | **1.0808** | **1.0969** | **69/118 (58%)** |

Paired delta (hex − seq): mean −0.0161, median −0.0150.

## 6. Scenario-family diagnostics

- The straight line alone was already feasible at pct=100 on **8/30** scenarios
  (s002, s006, s013, s016, s018, s020, s021, s026) — those measure no coordination.
- `formation`, a rigid body-frame slot rule, solved **28/30** at pct=100 (at 1.131 mean
  length ratio vs hexspline's 1.042), and myopic `greedy` solved 22/30. A one-helper
  problem does not appear to demand much coordination.
- State-space size: at `hex_width_m: 100` the graph is **792 heading-states** per agent
  (`Hex graph: 796 nodes … grid=13x15` on s005), so the 2-agent joint routing space is
  792² = **627,264** pairs. A 1e6 cap allows ~1.6 expansions per joint pair, and Pareto
  fronts hold several non-dominated covariances per pair — the search cannot enumerate
  its own state space once.

## 7. Conclusion

The headline "sequential beats hexspline_cl" in `SUMMARY.md` does not survive the paired
analysis. Where both finish, hexspline produces shorter paths (1.081 vs 1.097); on
deepest-constraint-solved they are statistically tied (58.8 vs 59.3); and sequential's
apparent edge is almost entirely (13/14) wins against a truncated search.

What the sweep actually measures is **anytime behaviour at a fixed 1e6-expansion budget
on a 792-node graph**, where prioritized planning returns a solution more often. That is
a legitimate finding, but it is not "sequential finds better plans."

See `../coarse_grid_ab_2026-08-12/` for the follow-up that tests the budget hypothesis
directly.
