# Figure 2 — constraint ladder on the `ladder_maze` scenario

`fig:ladder` in `main.tex`: one scenario, columns = constraint level (% of the
unconstrained single-agent reference uncertainty U_ref), rows = planners
(ours = `hexspline_cl`, greedy, formation, sequential, CL-GBT). The scenario is
the named preset `ladder_maze` in `src/scenario_generation.jl`; the ladder is
run by the existing harness (`run_constraint_sweep.jl --sweep
config/mc/sweep_fig2.yaml`) and drawn by `plot_ladder_grid.jl` here.

## Model and lattice (read this first)

* **Noise / sensor model: `config/mc`** (dir_unc 0.05 /m, landmark noise 0.038,
  comm range 300 m, visibility 100 m) — the same model as Fig. 3/4 and the same
  definition of the constraint level (`threshold = pct/100 · U_ref`, U_ref = the
  reference's *discrete* uncertainty). Fig. 1 was built on the working-tree
  model (dir_unc 0.01 …); the two are not the same model.
* **Lattice: `hex_width_m: 100`**, not config/mc's 150. Applied through the new
  sweep-level `overrides:` key of `sweep_fig2.yaml` (see `run_constraint_sweep.jl`,
  `SWEEP_OVERRIDES`) so the reference, the screen and every method run on it.
  Reason: on the 150 m lattice a wall-with-gap field cannot be made to pass the
  planner's seed gate (below) inside a corridor of sane length — every gap needs
  a straight run-in of one cell on each side, and at 150 m per cell three walls
  push the corridor past 2 km. At 100 m the same field fits in 1.4 km.

## The scenario (14 obstacles, 1400 m corridor, goal on row +2)

Three full-height walls with two gaps each (A: rows +1/−1, B: +1/−2, C: +2/−3)
plus six plugs sealing the row-0 and row-±3 pockets. Two lanes survive:

* **north, blind**: rows +1 → +2, 1500 m on the lattice, no landmark in range;
* **south, informative**: rows −1 → −2 → −3, past L1 (B's −2 gap), L2 (C's −3
  gap) and L3 (behind C, seen from row −2 at x = 1300/1400). The last landmark
  can only be seen 100 m short of the goal's x, from three rows below it.

Landmark 1 (L3) is Σ₀. Geometry rules the layout obeys, so the next field is not
rediscovered (all measured with `tools/gate_probe.jl`):

1. **Wall width 150 m** — covers one cell of each x-parity (rows 0/±173.2 sit at
   x ≡ 0 mod 100, rows ±86.6/±259.8 at x ≡ 50).
2. **Gap faces 60 m off the gap row's centre** (not the 43.3 m half-pitch): the
   blocked neighbour row's cell stays 26.6 m inside, and the spline gets a
   120 m channel instead of 87 m.
3. **Lane changes need room.** The seed gate (`seed_spline_clear`) certifies the
   B-spline seed with the committed-face MINVO hull; a diagonal step that lands
   on a gap row *at* a wall's face puts one segment's hull across the block's
   corner and no single face clears it (measured −23 to −53 m of slack). The
   landing cell must be ≥ 100 m before the near face (i.e. one straight cell
   before the wall) and the first lane change after a wall must come one
   straight cell after its far face. The MINVO hull hugs the *curve*, not the
   control polygon, so a straight pass through a gap is never the problem.
4. **Plugs only in cells no route uses**, at least one cell away from any
   landing cell of the shipped plans.
5. **A support ends anywhere within comm range** (only the primary must reach
   the goal), and its lattice length must not exceed the primary's. It "pays"
   for a diagonal by ending 50 m short in x — so with a 300 m comm range a
   support can spend ~6 more diagonals than the primary for free. A landmark the
   support should *not* reach for free therefore has to sit where seeing it
   leaves no steps to get back into comm range (L3, from row −3, with the goal
   on row +2).

## What the ladder does (hexspline_cl, 2 agents; `results/fig2/v8_p*`)

| pct | σ bound | lattice | spline | σ at goal | plan |
|---|---|---|---|---|---|
| 100–70 | 12.65–8.85 | 1500 | 1414–1418 | 7.6–8.9 | primary north; support through A(−1), B(−2), sees L1, climbs back into range |
| 60–30 | 7.59–3.79 | 1600 | 1508 | 3.07 | primary north **+ one wiggle before the goal** (buys the support one step); support sweeps L1, L2, L3 and relays at the goal |

Two distinct plans; the wiggle is what "spending primary length to extend the
support's reach" looks like on this lattice, and the continuous stage turns it
into a gentle bulge (the support-length ≤ primary-length constraint then pins the
primary at ~1508 m).

Rejected on the way (`results/fig2/v1–v7`; all `manual` scenarios, specs in
`v*.txt` here): v1/v2 (150 m lattice) — every goal pop killed by the seed gate,
see rule 3; v3 — north lane change 50 m after wall B's face; v4 — goal on row 0:
the support completes the whole sweep at the primary's shortest length and
every rung returns the same 0.98 plan (rule 5); v6 (300 m chambers, weaving
lanes, 13 obstacles) — the same collapse; v7 (v6 with L3 on row −3 only) — the
1800 m plan the tight rungs need is not found in 10⁶ expansions (the 1700 m
layer of joint states is too big). v5 = v8 without the plugs.

## Files

```
tools/mkcfg.py         config/mc + manual scenario + key=value overrides → a config dir
tools/run.sh           one generate_plan.jl run into results/fig2/<tag>
tools/ladder.sh        reference + hexspline_cl at 100…30 % (4 at a time), prints the table
tools/baselines.sh     the four baselines at chosen levels, prints the table
tools/gate_probe.jl    hand-typed lattice polyline → per-segment seed-gate slacks
v1.txt … v8.txt        the candidate fields (v8 == the ladder_maze preset)
plot_ladder_grid.jl    the figure, from a sweep_fig2 run
```

## Regenerating

```
~/.juliaup/bin/julia run_constraint_sweep.jl --sweep config/mc/sweep_fig2.yaml --tag v8     # → fig2_ladder/v8
~/.juliaup/bin/julia paper/new_draft/figures/figure2/plot_ladder_grid.jl fig2_ladder/v8 100,70,50,30
```

## The sweep (`fig2_ladder/v8`, harness run, all five planners)

Success is the harness's `classify`: bound met on the shipped spline, refinement
not `recovery_failed`, no real obstacle collision. Length is the shipped spline.

| | 100 % (12.65) | 90 % | 80 % | 70 % (8.85) | 60 % | 50 % (6.32) | 40 % | 30 % (3.79) |
|---|---|---|---|---|---|---|---|---|
| ours | 1416 / 7.64 | 1415 / 7.63 | 1415 / 7.67 | 1418 / 8.85 | 1506 / 3.08 | 1493 / 3.39 | 1505 / 3.07 | 1505 / 3.17 |
| greedy | 1448 / 10.95 | 1448 / 10.95 | ✗ recovery | ✗ recovery | ✗ | ✗ | ✗ | ✗ |
| formation | ✗ recovery (1967 m attempt, σ 3.99, spline hits an obstacle) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| sequential | ✗ no solution (leg 1: "no route to the goal for the primary") | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| CL-GBT | ✗ recovery (2153 m attempt, σ 4.92, min slack −5e-4) | ✗ | ✗ | ✗ | ✗ | ✗ no solution | ✗ | ✗ |

Reading it: greedy keeps the blind 1448 m route at every rung (the primary never
reasons about uncertainty) and stops passing once 10.95 exceeds the bound;
formation's seed is the whole south lane (primary takes every fix itself, 1967 m)
but its spline breaches an obstacle in this clutter and never recovers;
sequential's leg-1 search finds no primary route at all here (worth a look — the
single-agent reference on the same graph does); CL-GBT's tree reaches the goal
only at the two loosest rungs and its refinement ends 5·10⁻⁴ short of feasible.
The grid draws every attempted path faded on grey so these are visible, not blank.

Open points for the paper: whether to show 4 or all 8 columns; whether a
`recovery_failed` attempt whose bound is met (formation, CL-GBT at 100 %) should
be drawn as a partial success; and the model/lattice caveat above for the caption.
