# landmark_planning

Uncertainty-aware multi-agent planning for a GPS-denied AUV team.

## Quick start

```bash
# Edit parameters
nano config.yaml

# Run (outputs go to results/<timestamp>/)
julia planner.jl

# Run with a specific config file
julia planner.jl /path/to/custom_config.yaml
```

All tuning parameters live in `config.yaml` at the project root. Each run writes a copy of the config used into its output directory (`results/<timestamp>/config.yaml`) for reproducibility.

## Pipeline

1. Build heading-aware hex graph from the selected scenario's start, goal and landmarks.
2. Run joint discrete A* (`joint_astar`) for support + primary paths.
3. Run continuous B-spline refinement to minimize primary path length while enforcing uncertainty and curvature constraints.

## Configuration (`config.yaml`)

All parameters are set in `config.yaml`. The most commonly changed ones are described below.

### Mission setup

| Key | Default | Effect |
|---|---|---|
| `landmark_scenario` | `dual` | Name of a scenario in `src/scenario_generation.jl` (landmarks + obstacles + endpoints), or `manual` to define one inline — see [Scenarios](#scenarios) |
| `num_agents` | `2` | Total agents including the primary (last index) |

### Problem constraints (`config/main.yaml`)

These define the **feasible set** — what a valid plan is — so they live in
`config/main.yaml` and apply to every algorithm, not in any one planner's file.
An ablation is only a fair comparison if it is held to the same constraints as
the pipeline it is being compared against. (Obstacle enforcement knobs are in
the same file; the obstacle *geometry* belongs to the scenario.) A planner
`.yaml` that redefines one of these shadows it silently, so `load_config` warns
whenever an algorithm file overrides a `main.yaml` key.

| Key | Default | Effect |
|---|---|---|
| `unc_radius_threshold` | `7.0` | Feasibility bound on primary goal uncertainty (det-based scalar, meters). Lower = harder. |
| `unc_feas_tol` | `1e-6` | Boundary tolerance for feasibility comparisons |
| `min_turn_radius_m` | `40.0` | Minimum AUV turn radius; the curvature constraint is `κ ≤ 1/min_turn_radius_m` |

### Relaxed discrete handoff

Allows the discrete A* stage to accept seeds slightly above the strict threshold, with continuous refinement recovering feasibility.

| Key | Default | Effect |
|---|---|---|
| `enable_relaxed_discrete` | `false` | Enable relaxed discrete→continuous handoff |
| `relaxed_discrete_delta_mode` | `relative` | `absolute`: δ = `relaxed_discrete_delta_abs`; `relative`: δ = `relaxed_discrete_delta_rel × unc_radius_threshold` |
| `relaxed_discrete_delta_abs` | `0.20` | Absolute relaxation δ |
| `relaxed_discrete_delta_rel` | `0.2` | Relative relaxation δ multiplier |
| `continue_astar_on_infeasible` | `true` | Re-run A* if a relaxed seed fails continuous refinement |

Practical guidance: start with relaxed disabled. Enable with a small δ if search is too slow or frequently fails.

### A* performance

| Key | Default | Effect |
|---|---|---|
| `primary_epsilon` | `0.0` | Weighted A* suboptimality. Higher = faster, worse primary length |
| `astar_iteration_limit` | `200000` | Max expansions before the search gives up |
| `prune_by_comm_radius_joint` | `false` | Prune joint states where agents are out of comm range |
| `prune_by_primary_uncertainty` | `false` | Prune states where primary uncertainty already exceeds threshold |
| `prune_by_support_uncertainty` | `false` | Prune states where support uncertainty is too high |

### Continuous optimizer

| Key | Default | Effect |
|---|---|---|
| `cont_opt_iters` | `1000` | Adam iteration budget |
| `cont_opt_lr` | `0.5` | Adam learning rate |
| `cont_barrier_stages` | `4` | Barrier method stages |
| `cont_barrier_start` | `20.0` | Initial barrier weight μ |
| `cont_barrier_decay` | `0.35` | μ decay per stage |
| `cont_opt_h` | `1e-4` | Finite-difference step for the gradient, once feasible |
| `cont_recover_h` | `1.0` | Finite-difference step while still **infeasible** (see [Ablation](#ablation-straight_cont)). Same objective and loop — only the probe width differs. Never used by `hexspline_cl`, whose seed is feasible from iteration 1. |

The optimizer is a **single phase** for every seed: minimize primary path length
under a tangent-extended log barrier. Feasibility is an *invariant*, not a
precondition — the line search rejects any step that leaves the feasible set,
but only once feasibility has been reached. From an infeasible seed the
barrier's tangent below the knee carries a restoring gradient of `−μ/ε`
(~4 orders of magnitude above the length gradient at the defaults), so the same
objective recovers first and shortens after, with no separate restoration pass.

#### Curvature: what the discrete seed does and does not guarantee

The refinement stage constrains `κ ≤ 1/min_turn_radius_m` on every sampled point
of every agent's spline. The discrete stage is *usually* already inside that
bound — the hex graph is heading-aware and turns at most ±60° per step, and for
a control polygon of uniform legs of length `L` the splined turn peaks at
`κ·L ≈ 1.36` regardless of the route. So seeds satisfy curvature **iff**

```
hex_width_m  ≳  1.4 · min_turn_radius_m        (100 vs 56 at the defaults)
```

**This is a scaling law, not a structural guarantee, and it has three gaps:**

1. **Retuning breaks it silently.** Shrinking `hex_width_m` or raising
   `min_turn_radius_m` past that ratio makes *every* seed curvature-infeasible.
   Nothing asserts the ratio — the violation surfaces only as a refinement that
   spends its budget on recovery.
2. **The primary's final leg is not a full hex step.** The primary path ends on
   the goal *cell*, and its last waypoint is overridden to the exact goal
   position (`seed_control_points`), so the final leg is `1 hex step ± hex_r`.
   Measured on a synthetic ±60° seed, a final leg at ¼ of a step pushes sampled
   κ to 0.02–0.10 against the default limit of 0.025.
3. **Short paths.** Under 4 waypoints, `bspline_pad_controls` duplicates the end
   control points; the resulting near-zero speed makes κ blow up at the clamped
   ends (a parameterization artifact, not a real turn, but the constraint sees it).

None of the three bound in any current scenario, and the discrete stage does not
check curvature — a spline-level check at goal pops would cost ~28 µs per
candidate, more than the obstacle gate below, mostly re-deriving the constant
above. If you retune the grid or the turn radius, verify the ratio first.

### Physical / sensor model

| Key | Default | Effect |
|---|---|---|
| `dir_uncertainty_per_meter` | `0.05` | Along-track dead-reckoning drift (DVL+IMU) |
| `maj_min_unc_ratio` | `3` | Anisotropy ratio: along-track drift ÷ cross-track drift |
| `landmark_sensor_noise` | `0.038` | USBL/LBL fix accuracy in meters (landmark fusion) |
| `comm_sensor_noise` | `0.038` | USBL/LBL fix accuracy in meters (inter-agent comm fusion) |
| `bearing_noise_ratio` | `2.2` | Cross-range noise relative to along-range |
| `visibility_sigma` | `50.0` | 1σ detection range for landmark observations (meters) |
| `comm_radius` | `200.0` | Acoustic modem range for in-search approximation (meters) |
| `comm_sigma` | `100.0` | Gaussian taper scale for exact comm weighting (meters) |
| `comm_interval` | `100.0` | Arc-distance between synchronized comm checkpoints (meters) |

### Static obstacles

Hard no-go **convex-polygon** regions. Geometry is part of the **scenario**
(`src/scenario_generation.jl`), not config — `config/main.yaml` holds only the
risk/enforcement knobs below. They are enforced as a **chance-constrained feasibility filter inside the discrete joint A\***: a joint search state is rejected if any agent's belief has too high a collision probability. This is a pure feasibility check — obstacles never enter the measurement model and never modify covariance propagation.

Per agent *i* and obstacle *m*, at the state's mean μᵢ and covariance Σᵢ, the agent is **safe** iff it clears at least one polygon face *j*:

```
aⱼ·μᵢ ≥ bⱼ + r_a + z·√(aⱼᵀ(Σᵢ + Σₒ)aⱼ),   z = Φ⁻¹(1 − δ)
```

where the aⱼ are **unit** outward face normals, r_a the agent radius, and Σₒ the obstacle's optional location covariance (default zero). A joint state is infeasible if any agent collides with any obstacle. By the union bound, `P(any collision) ≤ Σδ` over all (agent, obstacle) pairs, so set `δ = Δ/(P·M)` for a total risk budget Δ across P agents and M obstacles.

| Key | Default | Effect |
|---|---|---|
| `obstacle_delta` | `0.05` | Collision-risk bound δ per (agent, obstacle) pair |
| `agent_radius` | `0.0` | Vehicle radius r_a; inflates every obstacle face outward |
| `obstacle_edge_samples` | `2` | Samples along each hex edge: `1` = destination node only, `2` = both endpoints, `≥3` adds interior points |
| `obstacle_continuous` | `true` | Also enforce avoidance in the continuous B-spline stage (MINVO-hull constraint) |

**Edge test vs. spline test.** The filter above tests the straight polyline
between hex centres, sampled at `obstacle_edge_samples` points — at the default
`2`, that is the endpoints only. The refined B-spline cuts the corners off that
polyline, so a polyline-clean seed can still enter an obstacle. Every goal
candidate is therefore additionally gated on its **spline**
(`seed_spline_clear`): the same committed-face MINVO test the continuous stage
enforces, at the same control points and the same Σ, so a candidate that passes
starts refinement already feasible on its obstacle rows. Candidates that fail are
discarded and the search continues; the count is reported as
`[Collector] N goal candidate(s) discarded`.

Polygons must be **convex** — `build_obstacle` errors on a non-convex one (convex decomposition is out of scope) — and may carry an optional location covariance `Σo` (default zero, i.e. exactly known). Φ⁻¹ is computed in-repo (Acklam's rational approximation), since `Distributions`/`StatsFuns` are not installed.

Obstacles are drawn as filled gray polygons on every planner figure. See `test_obstacles.jl` for the predicate's validation checks.

### Scenarios

A scenario bundles **landmarks + obstacles + start/goal**. All of them are defined in `src/scenario_generation.jl`'s `SCENARIOS` table; `config/main.yaml` only names one via `landmark_scenario:` (overridable per run with the `SCENARIO` env var).

| `landmark_scenario` | Start → goal | Contents |
|---|---|---|
| `single` | (0,0) → (1000,0) | 1 landmark at (600, −250); no obstacles. Minimal observation geometry |
| `dual` | (0,0) → (1000,0) | 2 landmarks at (700, 200) / (750, −250) + a box and an uncertain-location triangle. Off-axis placement creates incentive to deviate from shortest path |
| `clustered` | (0,0) → (1000,0) | 3 landmarks near (700, −200); no obstacles. All fixes come from one region |
| `shoreline` | (0,0) → (1000,0) | 5 landmarks along y ≈ −200…−300; no obstacles. Cross-track observability |
| `two_routes` | (0,0) → (1200,0) | A central island splits the corridor: short blind route (south) vs longer landmark-rich route (north) — two genuinely different distance/uncertainty trade-offs |
| `gauntlet` | (0,0) → (1200,0) | Three staggered walls force a down-up-down weave, with a landmark in each gap. Tightest curvature test |
| `behind_wall` | (0,0) → (1000,0) | The only landmark sits ~300 m off-route below a 500 m wall; a fix requires going around an end |
| `maze` | (0,0) → (1200,0) | 13 obstacles: three full-height walls, each with two gaps at different rows, plus five chamber plugs so no chamber has a straight shot. Two homotopies with two landmarks each — a shorter northern one and a longer southern weave. Densest obstacle field |
| `long_sparse` | (0,0) → (1800,0) | 1800 m of dead reckoning, 2 distant landmarks, no obstacles. Straight-line terminal uncertainty exceeds the threshold, so both must be visited. Slowest to run |

**Defining one inline instead.** Set `landmark_scenario: manual` and give the geometry directly in `config/main.yaml` as flat strings — vertices/points `x,y`, obstacle vertices separated by `;`, items by `|`, with an optional covariance after `@` as `sxx,sxy,syx,syy` (row-major). A landmark with no `@` gets a random SPD covariance. Anything omitted is empty.

```yaml
landmark_scenario: manual
start: 0,0
goal: 1000,0
landmarks: 700,200 | 750,-250 @ 1.2,0,0,0.8
obstacles: 200,-150; 260,-150; 260,-100; 200,-100 | 400,60; 460,60; 430,120 @ 4,0,0,4
```

**Generating one randomly.** Set `landmark_scenario: random` and give four
numbers; start is always `(0,0)` and goal `(scenario_goal_dist, 0)`. Driven
per-run by `run_constraint_sweep.jl`, but usable by hand
(`SCENARIO=random julia generate_plan.jl`).

```yaml
landmark_scenario: random
scenario_seed: 1001            # the scenario is a pure function of these four
scenario_goal_dist: 1000.0
scenario_n_landmarks: 2        # must be >= 1: landmark 1's covariance is Σ₀
scenario_n_obstacles: 2
```

Landmarks land at `x ∈ [0.12D, 0.88D]`, `|y| ∈ [60, 320]` m — bounded by the
sensor model at both ends. Past ~330 m off-corridor a landmark is effectively
undetectable (see the visibility sigmoid below); inside 60 m it is already
covered from the direct route, so there is no detour trade-off left to measure.
Obstacles are 3–6 sided convex polygons, non-overlapping, clear of both
endpoints. Every draw comes from a local RNG seeded by `scenario_seed`, so the
geometry does **not** depend on `generate_plan.jl`'s global `Random.seed!(42)`.
Check the generator with `julia test_scenario_generation.jl`, and eyeball a
sample with `julia test_scenario_generation.jl --preview 12`.

**Corridor limits.** `build_hex_graph` pads ±`corridor_halfwidth_m` about y=0, drops rows above `corridor_y_max_m`, and lands rows on multiples of `1.5·hex_r`, so at the defaults (`hex_width_m: 100`, `corridor_halfwidth_m: 260`, `corridor_y_max_m: 300`) the reachable rows are y ∈ {−433.0, −346.4, −259.8, −173.2, −86.6, 0, 86.6, 173.2, 259.8}. A wall meant to *block* a route must cover a whole row: hex edges are sampled only at `obstacle_edge_samples` points, so a wall thinner than the 86.6 m row spacing can be stepped over. Node **x** depends on row parity — rows {−346.4, −173.2, 0, 173.2} sit at x ≡ 0 (mod 100), rows {−433, −259.8, −86.6, 86.6, 259.8} at x ≡ 50 — so a *vertical* wall must span ≥150 m in x to cover one column of each parity, or it is stepped around on the offset row (see `maze`).

`corridor_halfwidth_m` and `corridor_y_max_m` are in **metres and do not derive
from `hex_width_m`** — they set the row *count*, which is what drives graph size.
Retune them whenever you change `hex_width_m`, in the same proportion: the
defaults over a 3.46 m row pitch would build ~150 rows, and Floyd-Warshall is
O(n³), so the run never starts. `config/hardware/main.yaml` is the worked
example (`hex_width_m: 4.0`, `corridor_halfwidth_m: 12.0`,
`corridor_y_max_m: 13.0` → the same 9 rows the defaults give).

## A* stopping rule

The joint A* stops on the first feasible path below `unc_radius_threshold`,
or gives up at `astar_iteration_limit` expansions. One seed per run.

## Ablation: `straight_cont`

Set `algorithms: straight_cont` to run the discrete search's ablation.
Everything downstream of the seed is **identical** to `hexspline_cl` — same
homotopy commitment (`build_obstacle_plan`), same constraints, same
single-phase barrier optimizer. Only the seed differs: a direct start→goal line
with `straight_cont_primary_wpts` control points instead of the joint A* path.

The straight seed passes no gate, so it typically starts outside the feasible
set and the barrier must recover it before it can shorten anything.
`refinement_status` in `results.yaml` is the measurement:

| Status | Meaning |
|---|---|
| `optimized` | A feasible iterate was found and shortened |
| `seed_only` | Seed was already feasible; no step improved it |
| `recovery_failed` | **Never reached the feasible set — this result violates constraints** (`min_slack < 0`) |

Measured across all nine scenarios (2 agents, `unc_radius_threshold: 7.0`),
`straight_cont` recovers in **1 of 9** (`gauntlet`: min_slack −48.2 → +0.022,
unc 5.55). The other eight end `recovery_failed`. Two distinct failure modes,
both of which the discrete search exists to solve:

- **Flat uncertainty landscape.** Detection probability is a logistic on
  distance with `visibility_width: 2.5` against a `visibility_range: 100`
  plateau, so a landmark 250 m off-corridor (`single`) has detection
  probability ≈ 1e-26. The uncertainty gradient there is *exponentially* zero:
  no probe width finds a descent direction, and gradient descent cannot
  discover a detour it cannot yet feel. A combinatorial search does not need a
  gradient to jump to a distant homotopy.
- **Wrong homotopy, committed.** The straight seed commits each spline segment
  to whichever obstacle face it already leans toward. In `behind_wall` the only
  landmark is on the *far* side of the wall, so lowering uncertainty requires
  breaching a committed face — the recovery is trapped in a homotopy that has
  no feasible point. A* picks the homotopy that makes the problem feasible.

Reproduce with `SCENARIO=<name> julia generate_plan.jl` after setting
`algorithms: straight_cont`.

## Monte Carlo constraint sweep

`run_constraint_sweep.jl` measures **what a tightening localization constraint
costs in path length**, over randomly generated scenarios. Per scenario:

1. **Reference** — `hexspline_cl`, 1 agent, `unc_radius_threshold: 1e9` (no
   constraint). A\* accepts the first goal pop, so this is the *shortest* path.
   Gives `L_ref` and `U_ref`, the uncertainty that path accumulates.
2. **Ladder** — `unc_radius_threshold = pct/100 · U_ref` for pct = 100, 90, 80 …
   At each level every configured method runs and reports `primary_length / L_ref`.
3. Stops once every method has failed at `pct_patience` consecutive levels — the
   levels that trigger the stop are still run and still logged.

```bash
julia run_constraint_sweep.jl                       # new sweep, timestamped tag
julia run_constraint_sweep.jl --tag mysweep         # named; RESUMES if it exists
julia run_constraint_sweep.jl --tag mysweep --summarize-only
julia run_constraint_sweep.jl --sweep path/to/other_sweep.yaml
```

Config lives in **`config/mc/`** — a self-contained snapshot of `config/`, so
`config/` stays free for interactive work and editing it mid-sweep cannot change
what is being measured. The trade-off is drift: retune the physical/sensor model
in `config/main.yaml` and you must mirror it into `config/mc/main.yaml`.
`config/mc/sweep.yaml` holds the sweep's own knobs (scenario count and ranges,
ladder, methods, `save_figures`/`save_csv`, timeout, workers).

**Adding a baseline is one config line, never code.** Methods are
`label@key=value,... | ...`, each key a config override:

```yaml
methods: hexspline_cl@algorithms=hexspline_cl,num_agents=2 | straight_cont@algorithms=straight_cont,num_agents=2
```

The harness requires exactly **two** keys from a planner's `results.yaml` —
`primary_length` and `primary_unc` — and treats everything else it finds there
as pass-through, emitting each key as an extra CSV column. So `trials.csv` has
18 columns for `hexspline_cl` and 14 for `straight_line`, and there is no
algorithm-specific logic in the sweep at all. That is why `trials.csv` lives
per-method rather than at the root.

```
results/constraint_sweep/<tag>/
  summary.csv scenarios.csv SUMMARY.md fig_length_ratio.png sweep.log
  reference/<scenario>/                 the unconstrained 1-agent run
  <method>/trials.csv                   columns vary per algorithm
  <method>/<scenario>_p<pct>/           results.yaml, config/, run.log
                                        figures/ + csv/ only if save_* is on
```

**Reading a failure.** `fail_reason` is one of `timeout`, `error`,
`no_solution`, `recovery_failed`, `unc_violated`, `obstacle_breach`. A
`no_solution` is **not** proof of infeasibility: `joint_astar` exits identically
whether the threshold is unreachable or the iteration budget ran out. Compare
`astar_iterations` against `astar_iteration_limit` in `config/mc/hexspline_cl.yaml`
(400000, cut down from the interactive 1600000) — equality means the budget was
the binding constraint, not the geometry.

## Outputs

Each run produces a timestamped directory `results/<yyyy-mm-dd_HH-MM-SS>/` containing:

| File | Description |
|---|---|
| `config.yaml` | Copy of the config used for this run |
| `fig1_joint_discrete_astar.png` | Discrete A* solution with covariance ellipses |
| `mainfig_compare_discrete_continuous_<len>.png` | Side-by-side discrete vs. refined B-spline |
| `main_ctrls.csv` | B-spline control points for the main solution |
| `comm_events.csv` | Comm fusion events (when `track_comm_events: true`) |

`discrete_only` skips the refinement stage, so instead of the compare figure it
writes `fig_discrete_spline.png` — the unrefined seed spline it ships.



## Guarantees

- A* ordering key: `f = g + (1 + primary_epsilon) × h` where `h` is Floyd-Warshall distance to goal.
- With `primary_epsilon = 0`, this is standard A* for primary-cost search.
- Strict feasibility is always evaluated against `unc_radius_threshold` (with `unc_feas_tol`).
- Relaxed discrete mode is a seed-generation speed mechanism; the final solution is always checked against the strict threshold.
- Dominance pruning uses the PSD partial order on 2×2 covariance matrices (stronger than scalar det criterion).
- Static-obstacle feasibility is chance-constrained per (agent, obstacle) pair. PSD dominance pruning stays sound under it: since frontier labels share the same node (same mean), a covariance-dominating state clears every obstacle face the dominated one clears.

## Further reading

See `COVARIANCE_MODEL.md` for a derivation of how the Kalman propagation, landmark fusion, and inter-agent cooperative localization map to the paper equations.
