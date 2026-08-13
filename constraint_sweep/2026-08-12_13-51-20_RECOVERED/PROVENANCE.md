# RECOVERED — constraint_sweep/2026-08-12_13-51-20

## What happened

The original sweep directory `constraint_sweep/2026-08-12_13-51-20/` (run 13:51–16:34
on 2026-08-12) was **lost** when the machine hard-restarted on 2026-08-12 ~17:11.

The restart was triggered by an out-of-memory event: an A/B experiment launched four
concurrent `hexspline_cl` runs at `hex_width_m: 100`, `num_agents: 2`, each of which
walks the full `astar_iteration_limit: 1000000` and pushes a `State` per expansion.
The sweep harness itself never generates that load — of its 6 concurrent workers only
one is ever `hexspline_cl` — so this is not a defect the sweep would hit on its own.

The directory was never tracked by git (`git log --all --diff-filter=A` finds no add)
and was not moved to Trash, so the raw run tree is **not recoverable**.

## What is in this directory

Reconstructed verbatim from the files as they were read before the crash:

| file | status |
|---|---|
| `SUMMARY.md`   | verbatim |
| `summary.csv`  | verbatim |
| `scenarios.csv`| verbatim |
| `ANALYSIS.md`  | derived tables (paired comparisons, budget diagnostics) |

## What is permanently lost

- `<method>/trials.csv` for all 7 methods (241 rows each, 1687 total). The aggregate
  numbers survive in `summary.csv`; the **per-run rows do not**. In particular the
  per-scenario `length_ratio` for individual methods at individual constraint levels
  cannot be recovered — only the per-(method, pct) means and medians in `summary.csv`.
- All per-run `results.yaml`, `run.log`, `_cfg/` and figures.
- `sweep.log` (70 KB) and `fig_length_ratio.png`.

`ANALYSIS.md` preserves the derived quantities that were computed from `trials.csv`
before it was lost, including the per-scenario deepest-solved-level table and the
failure-reason breakdown. Those numbers are reported, but the underlying rows are gone,
so they cannot be recomputed or extended.

## Reproducing

Every scenario is reproducible from its seed: `scenario_seed_base: 1067`, scenario `sNNN`
uses `seed = 1067 + N`, with `(goal_dist, n_landmarks, n_obstacles)` drawn by
`sample_scenario_params` from `MersenneTwister(seed + 1_000_000)`. `scenarios.csv` records
the drawn values, so the sweep can be re-run to regenerate the raw tree.

**Caveat**: `generate_scenario` derives `LM_BAND_INNER = hex_width_m + visibility_range + 30`
and `sample_scenario_params` quantises `goal_dist` in hex *columns*, so re-running under a
different `hex_width_m` yields **different geometry**, not the same scenarios at a different
resolution.
