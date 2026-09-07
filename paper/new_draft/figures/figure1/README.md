# Figure 1 candidates — support diverts to a landmark, primary keeps the short route

Three hand-built scenarios for the overview figure (`fig:overview` in
`main.tex`). In each one the **primary** (blue) follows its shortest
obstacle-avoiding route while the **support** (purple) leaves it, takes a fix
from a landmark the primary never sees, and comes back within comm range before
the goal so the fix reaches the primary through Covariance Intersection. Pick
one; register it in `SCENARIOS` (`src/scenario_generation.jl`) afterwards.

All runs use the working-tree `config/` model (dir_unc 0.01, landmark noise
0.33, comm noise 0.5, hex 100 m, comm_range 100 m, visibility 100 m). The single
landmark carries an explicit covariance `[1.0 0; 0 0.8]`, which is also Σ₀
(`lms[1].cov`), so the runs are RNG-free and reproduce from the snapshots.

## Side by side

| | A_staggered | B_walls | C_gauntlet | D_island |
|---|---|---|---|---|
| obstacles | two 2-row blocks, staggered | wall from the bottom, wall from the top | three alternating edge walls | one chamfered island |
| corridor | 1000 m | 1000 m | 1100 m | (0,0) → (1050, 86.6), 1054 m |
| landmark | (600, −300) | (700, −330) | (800, +220) | (750, −150) |
| bound (shipped `tight/`) | 2.0 | 2.0 | 2.2 | 2.0 |
| primary, lattice | **1200** (= blind 1200; alone 1400) | **1300** (= blind 1300; alone 1500) | **1400** (= blind 1400; alone 1500) | **1200** (alone 1300; blind 1400, see note) |
| primary, spline | **1065.7** (blind 1063.6; alone 1098.8) | **1120.8** (blind 1113.3; alone 1189.7) | 1253.1 (blind 1166.2; alone 1229.4) | **1100.0** (alone 1138.4; blind 1177.8, see note) |
| primary σ at goal | 2.000 | 2.000 | 2.200 | 1.799 |
| rendezvous (arc, dist, w) | 900 m, 102 m, 0.32 | 900 m, 99 m, 0.62 | 1000 m, 103 m, 0.24 | 1100 m (the goal), 101 m, 0.41 |
| primary σ across it | 2.19 → 1.86 | 2.23 → 1.74 | 2.43 → 1.89 | 2.34 → 1.80 |
| verdict | clean, minimal | **strongest picture** | most obstacles, weakest rejoin | **fully separate paths**, one rejoin at the goal |

"blind" = the same geometry, one agent, no bound (`reference/blind/`);
"alone" = one agent held to the same bound (`reference/solo/`) — it has to go
to the landmark itself, two rows deeper, which is the +200 m on the lattice.

- **A_staggered.** Block 1 covers rows 0/−86.6, block 2 rows 0/+86.6. The
  primary drops two rows and rides under both blocks (a lattice tie with the
  north-then-south S-curve, broken toward the support); the support goes one
  row deeper to the landmark and comes back up 100 m behind. Primary spline is
  within 2 m of the unconstrained optimum. Block 2 is passed underneath, so it
  shapes nothing — the simplest picture of the three.
- **B_walls.** Wall 1 rises from the bottom (shared north pass), wall 2 hangs
  from the top down to row −86.6 (primary forced to −173.2). Both agents run
  together for 800 m, then the support dips under wall 2 to the landmark and
  rejoins as the primary comes down; the primary's ellipse visibly collapses at
  the rejoin. Both walls shape the route.
- **C_gauntlet.** Three walls force a full S-weave. The support peels off
  upward after wall 2, takes the fix at the top of the last chamber and relays
  it as the primary passes ~100 m below — it does not come back down. Also the
  only candidate whose *spline* is longer than the lone agent's: the
  continuous stage caps the support's length at the primary's, and the
  support's climb is long, so the primary cannot shorten past ~1253 m whatever
  the bound. The lattice story (1400 vs 1500) still holds.
- **D_island** (added after A–C were rejected for running the two agents
  side by side, which stacks ellipses on overlapping tracks). One island over
  rows −86.6/0/+86.6, x ∈ [190, 640], top chamfered to y = 150 over
  x ∈ [300, 550]; goal moved up one row to (1050, 86.6) so the north pass is
  the short one. The primary goes over the top from the first step, the
  support goes under, sees the landmark at (750, −150) on its way up and meets
  the primary once, 100 m short of the goal (w 0.41, primary σ 2.34 → 1.80).
  The tracks never come within comm range between start and goal, so there
  are only two comm events; the shipped picture therefore draws the
  covariance **every 100 m of arc** instead (`_every100m` files, see below).
  Two caveats. (1) The support's spline is as long as the primary's, so the
  support-length ≤ primary-length constraint holds the primary at exactly
  1100.0 m; the free north route is only a few metres shorter. (2) The
  `reference/blind` run is *not* the shortest route here: its single-agent A*
  had its north seeds rejected by the spline gate and went south (1400 lattice
  / 1177.8 spline, landmark moved out of reach to (750, −900) for that run),
  so the joint run's own primary is the better "shortest" reference.
  `flat/` is the same design with a plain rectangular island (top at y = 110):
  σ 1.72 at the goal, same 1100.0 m; the chamfer just lets the primary hug the
  island more naturally.

## Folder layout

```
<cand>/tight/           shipped variant: config/, scenario_*.csv, hexspline_cl/{results.yaml,csv,figures}
                        + fig1_continuous_ellipses{,_comm}.{png,svg,pdf,eps}   ← the deliverable
<cand>/plain/           strict-bound variant (A 2.2, B 2.3, C 2.5), PNG only
D_island/flat/          rectangular-island variant of D (same bound), full run
D_island/*/fig1_continuous_ellipses_every100m{,_comm}.*   ellipse every 100 m of arc (D only)
<cand>/reference/blind  one agent, no bound     (shortest possible route)
<cand>/reference/solo   one agent, same bound   (what the primary must do without help)
```
Every variant folder is a complete `generate_plan.jl` output, so
`fig1_overview.jl` runs on it directly.

**tight vs plain.** The lattice seed's uncertainty is always higher than the
refined spline's (the polyline is longer and zig-zags). Under a strict bound
that leaves the continuous stage slack, so the optimizer has no reason to keep
the support in range and the rejoin fuses weakly (A plain: w 0.26). `tight/`
uses the documented relaxed handoff (`enable_relaxed_discrete: true`,
`absolute` δ 0.2–0.3): the A* accepts the seed at the looser gate, the barrier
then has to meet the tighter bound and does it by pulling the *support* in
(free) instead of bending the primary — B's primary is 27 m *shorter* at 2.0
than at 2.3.

## Design rules (so the next scenario is not rediscovered)

At `hex_width_m: 100` every step is 100 m (forward Δx 100, diagonal Δx 50 /
Δy ±86.6), all agents step together, and a comm only fuses between the same
or adjacent cells (100 m ⇒ w = 0.5). Hence:

1. `x_primary − x_support = 50·(diag_s − diag_p)`; a rendezvous needs that
   difference ≤ 2.
2. A landmark hidden from the primary must be ≥ ~110 m from every cell a
   shortest primary route can visit — one row past the primary's deepest row.
3. **Tie identity:** if the support can end co-located (equal diagonals), its
   route is a valid primary route of the same length that reaches the
   landmark — the primary can self-fix at no cost and the support is
   unnecessary (A's first cut, C's first cut). A support that is *needed*
   therefore always trails by 100 m on the lattice (w = 0.5); the continuous
   stage tunes that distance (99 m ⇒ 0.6, 93 m ⇒ 0.95).
4. Under this model a fix decays fast (Σ grows 1 m²/100 m along-track): the
   landmark has to sit in the last third, the rendezvous right after it.
5. Walls must span ≥ 150 m in x (both column parities); faces ~30 m past the
   blocked row centre. A 2-row block is passable on both sides — the primary
   will ride under it if that ties (A).

Rejected on the way: 2-row blocks in a slalom (primary rides the bottom row and
sees the landmark); A with block 1 three rows deep (forces the S-curve, but the
spline seed fails recovery at 2.2); C on a 1000 m corridor (spline gate rejects
every corner-cutting route near wall 3); landmark only one row deep (tie, rule 3).

## Regenerating

```
OUTPUT_DIR=results/fig1_X ~/.juliaup/bin/julia generate_plan.jl paper/new_draft/figures/figure1/<cand>/tight/config
~/.juliaup/bin/julia paper/new_draft/figures/fig1_overview.jl results/fig1_X          # writes the ellipse figures into the run
~/.juliaup/bin/julia paper/new_draft/figures/fig1_overview.jl results/fig1_X results/fig1_X 100   # ellipse every 100 m of arc instead
```
The optional third argument is the ellipse spacing in metres of each agent's
own arc (start and goal included; the comm checkpoints lie on the same grid).
0 or omitted = comm points only. Spaced output gets an `_every<N>m` suffix.
`fig1_overview.jl` re-evaluates the shipped spline with the planner's own
`evaluate_joint_discrete` and refuses to draw unless it reproduces
`results.yaml`'s `primary_unc`; ellipses are 2σ with σ × 10 (`SIGMA_SCALE`).
It prints the per-event table (arc, weight, distance, σ before → after) the
numbers above come from.

Formats: PNG preview; SVG/PDF straight from GR; EPS via Ghostscript
`eps2write` of the PDF (GR has no EPS writer). GR outlines text in **every**
vector format (the SVG has no `<text>` nodes) — geometry and fills are
editable, labels must be retyped if changed.
