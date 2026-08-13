# Coarsened-grid A/B — 2026-08-12

## Question

Sweep `2026-08-12_13-51-20` had `hexspline_cl` report `no_solution` at **pct=100** on
s005, s012, s029 and s030, each after burning the full `astar_iteration_limit: 1000000`.
At pct=100 the threshold *is* `U_ref`, the single-agent reference's own discrete
uncertainty, and the 2-agent feasible set is a strict superset of the 1-agent one (park
the helper; `ci_omega_det` clamps ω to [0,1] and ω=1 returns `cov_a` unchanged, so CI
fusion can never raise det). A feasible solution therefore **provably exists** on all
four, and the verdicts are false negatives.

Hypothesis: the joint state space is simply too large for the budget. At
`hex_width_m: 100` the graph carries **792 heading-states** per agent, so the 2-agent
joint routing space is 792² = **627,264** pairs — a 1e6 cap allows ~1.6 expansions per
pair, and Pareto fronts hold several non-dominated covariances per pair. The search
cannot enumerate its own state space once.

Test: hold the scenario fixed, coarsen the grid, see whether the solution appears.

## Method

Scenario **geometry is pinned** via `landmark_scenario: manual` (`pinned_geometry.tsv`,
dumped by `dump_geometry.jl` from `generate_scenario` under `config/mc`, i.e. at
`hex_width_m: 100`). Regenerating from `random` would **not** be an A/B:
`sample_scenario_params` quantises `goal_dist` in hex *columns* and
`LM_BAND_INNER = hex_width_m + visibility_range + 30`, so a different `hex_width_m`
yields different geometry.

Every physical, sensor and constraint knob is held **fixed**, `comm_range` included. Its
2×`hex_width_m` relation is deliberate (see `config/mc/main.yaml`) but re-tuning it would
move the feasible set and contaminate the comparison. `hex_width_m` is the single
variable; `corridor_halfwidth_m` / `corridor_y_max_m` stay in metres, so the row count
falls out of the coarsening rather than being set.

Coarse width is chosen per scenario so `goal_dist` stays an integer number of columns —
the goal must land on a cell centre or the discrete A* certifies uncertainty at the goal
*cell* while the shipped spline runs on to the true goal. D=800 → w=160 (5 columns),
D=700 → w=140 (5 columns). No single coarser width divides both 700 and 800.

Arms: `wC_a2` (coarse, 2 agents, thr = U_ref) and `wC_ref` (coarse, 1 agent, thr = 1e9).

## Result — all four solve, at 2.5–223× fewer expansions

| scen | w | heading-states | joint pairs | result | expansions | wall |
|---|---|---|---|---|---|---|
| s005 | 100 → 160 | 792 → 384 | 627,264 → 147,456 | `no_solution` → **ok** | 1,000,000 → **16,650** | 148 s → 22 s |
| s012 | 100 → 160 | 792 → 384 | 627,264 → 147,456 | `no_solution` → **ok** | 1,000,000 → **391,579** | 170 s → 88 s |
| s029 | 100 → 160 | 792 → 384 | 627,264 → 147,456 | `no_solution` → **ok** | 1,000,000 → **4,490** | 179 s → 20 s |
| s030 | 100 → 140 | 792 → 384 | 627,264 → 147,456 | `no_solution` → **ok** | 1,000,000 → **11,488** | 157 s → 22 s |

All four clear the constraint with margin (`primary_unc` vs threshold): s005 5.75 / 6.28,
s012 5.02 / 8.67, s029 2.54 / 3.98, s030 7.66 / 8.21. Worst case used 39% of the budget.

The budget hypothesis is confirmed: a 4.25× smaller joint space converts four provable
false negatives into solutions, with the hardest still well inside the existing cap.

## Cost — coarsening is not free

**Path length gets worse.** Ratios against the w=100 `L_ref`: s005 1.067, s012 1.165,
s029 1.203, s030 1.195 — against a fine-grid hexspline mean of 1.042 at pct=100. Part of
this is the coarser seed and part is that `L_ref` itself is a w=100 quantity; against
their own coarse 1-agent reference the ratios are 1.058 / 1.151 / 1.261 / n-a. Either
way, feasibility is being bought with length.

**Resolution loss can make a corridor unroutable.** `s030_wC_ref` (1 agent, no
uncertainty constraint, w=140) returned `no_solution` after draining its frontier at
10,374 expansions — *not* capped. Its log is full of

```
[Constraint A*] Goal popped, unc OK, but its B-spline breaches an obstacle; discarded.
```

At w=140 the 140 m hex edges make the smoothed spline cut corners hard enough to clip
obstacles that the discrete polyline cleared, and with `obstacle_edge_samples: 2` every
goal-reaching seed failed `seed_spline_clear`. This bounds how far the grid can be
coarsened on obstacle-dense scenarios and argues for raising `obstacle_edge_samples`
alongside `hex_width_m`.

Note the 2-agent run at the *same* w=140 succeeded. One plausible reading: in the
1-agent search a shorter route with dominating covariance prunes a longer one, whereas in
the joint search the support's covariance changes the dominance relation, so primary
sequences that the 1-agent search pruned survive — and one of them splines clear. That is
a hypothesis, not a measured fact; it has not been verified.

## Caveats

- **The fine-grid control arm was never run.** Four concurrent `w=100, num_agents=2` runs
  at the 1e6 cap exhausted machine memory and forced a restart (which is what destroyed
  the original sweep — see `../2026-08-12_13-51-20_RECOVERED/PROVENANCE.md`). The
  `w100_a2` rows in `results.csv` are transcribed from that sweep's `trials.csv`, not
  re-measured here. They are the same geometry, threshold and config, so the comparison
  holds — but it is a cross-run comparison, not a within-experiment control.
- This is **4 scenarios at one constraint level**, selected precisely because they were
  provable false negatives. It confirms the mechanism; it does **not** establish that
  coarsening improves hexspline's aggregate standing. That needs a full re-sweep at the
  coarser width, where the length cost above will also show up.
- The fine grid is **memory**-bound, not time-bound (`every expansion pushes a State`),
  which is why the cap cannot simply be raised to 1.5M. Coarsening relieves both.

## Files

| path | contents |
|---|---|
| `results.csv` | one row per (scenario, arm) |
| `runs/` | full run tree per arm: `results.yaml`, `run.log`, `_cfg/`, figures |
| `pinned_geometry.tsv` | exact start/goal/landmarks/obstacles per scenario |
| `dump_geometry.jl` | regenerates `pinned_geometry.tsv` from the scenario seeds |
| `ab_coarse.jl` | the A/B driver (resumable; skips runs with a `results.yaml`) |
| `run_console.log` | console output of the completed runs |
