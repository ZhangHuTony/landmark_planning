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
| `astar_mode` | `threshold` | `threshold`: stop on first feasible path; `limit`: collect full Pareto front |

### Uncertainty thresholds

| Key | Default | Effect |
|---|---|---|
| `unc_radius_threshold` | `3.1` | Feasibility bound on primary goal uncertainty (det-based scalar, meters). Lower = harder. |
| `unc_feas_tol` | `1e-6` | Boundary tolerance for feasibility comparisons |

### Relaxed discrete handoff

Allows the discrete A* stage to accept seeds slightly above the strict threshold, with continuous refinement recovering feasibility.

| Key | Default | Effect |
|---|---|---|
| `enable_relaxed_discrete` | `false` | Enable relaxed discrete→continuous handoff |
| `relaxed_discrete_delta_mode` | `relative` | `absolute`: δ = `relaxed_discrete_delta_abs`; `relative`: δ = `relaxed_discrete_delta_rel × unc_radius_threshold` |
| `relaxed_discrete_delta_abs` | `0.20` | Absolute relaxation δ |
| `relaxed_discrete_delta_rel` | `0.2` | Relative relaxation δ multiplier |
| `continue_astar_on_infeasible` | `true` | Re-run A* if a relaxed seed fails continuous refinement (`:threshold` mode only) |

Practical guidance: start with relaxed disabled. Enable with a small δ if search is too slow or frequently fails.

### A* performance

| Key | Default | Effect |
|---|---|---|
| `primary_epsilon` | `0.0` | Weighted A* suboptimality. Higher = faster, worse primary length |
| `astar_iteration_limit` | `200000` | Max expansions (`:limit` mode) or search budget (`:threshold` mode) |
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
| `min_turn_radius_m` | `40.0` | Minimum AUV turn radius (curvature constraint) |
| `cont_opt_h` | `1e-4` | Finite-difference step for gradient |

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
| `comm_fusion` | `ci` | Inter-agent fusion rule: `ci` (Covariance Intersection — sound, consistent under unknown cross-correlation) or `kf` (legacy weighted information-filter add — overconfident) |

### Static obstacles

Hard no-go **convex-polygon** regions. Geometry is part of the **scenario**
(`src/scenario_generation.jl`), not config — `config/main.yaml` holds only the
risk/enforcement knobs below. They are enforced as a **chance-constrained feasibility filter inside the discrete joint A\***: a joint search state is rejected if any agent's belief has too high a collision probability. This is a pure feasibility check — obstacles never enter the measurement model and never modify covariance propagation. (Continuous B-spline refinement is currently obstacle-unaware.)

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
| `two_routes` | (0,0) → (1200,0) | A central island splits the corridor: short blind route (south) vs longer landmark-rich route (north). Feeds the Pareto front two genuinely different trade-offs |
| `gauntlet` | (0,0) → (1200,0) | Three staggered walls force a down-up-down weave, with a landmark in each gap. Tightest curvature test |
| `behind_wall` | (0,0) → (1000,0) | The only landmark sits ~300 m off-route below a 500 m wall; a fix requires going around an end |
| `long_sparse` | (0,0) → (1800,0) | 1800 m of dead reckoning, 2 distant landmarks, no obstacles. Straight-line terminal uncertainty exceeds the threshold, so both must be visited. Slowest to run |

**Defining one inline instead.** Set `landmark_scenario: manual` and give the geometry directly in `config/main.yaml` as flat strings — vertices/points `x,y`, obstacle vertices separated by `;`, items by `|`, with an optional covariance after `@` as `sxx,sxy,syx,syy` (row-major). A landmark with no `@` gets a random SPD covariance. Anything omitted is empty.

```yaml
landmark_scenario: manual
start: 0,0
goal: 1000,0
landmarks: 700,200 | 750,-250 @ 1.2,0,0,0.8
obstacles: 200,-150; 260,-150; 260,-100; 200,-100 | 400,60; 460,60; 430,120 @ 4,0,0,4
```

**Corridor limits.** `build_hex_graph` pads ±260 m about y=0 and lands rows on multiples of `1.5·hex_r`, so at the default `hex_width_m: 100` the reachable rows are y ∈ {−433.0, −346.4, −259.8, −173.2, −86.6, 0, 86.6, 173.2, 259.8}. A wall meant to *block* a route must cover a whole row: hex edges are sampled only at `obstacle_edge_samples` points, so a wall thinner than the 86.6 m row spacing can be stepped over.

## A* collection modes

**`:threshold`** — stops on the first feasible path below `unc_radius_threshold`. Fast; produces a single main solution.

**`:limit`** — runs until `astar_iteration_limit` and collects the full Pareto front (non-dominated on distance vs. uncertainty). Slower but reveals the full solution space; generates one optimized path per Pareto seed.

Interaction with `continue_astar_on_infeasible`: only relevant for `:threshold` mode. In `:limit` mode the flag is ignored.

## Outputs

Each run produces a timestamped directory `results/<yyyy-mm-dd_HH-MM-SS>/` containing:

| File | Description |
|---|---|
| `config.yaml` | Copy of the config used for this run |
| `fig1_joint_discrete_astar.png` | Discrete A* solution with covariance ellipses |
| `mainfig_compare_discrete_continuous_<len>.png` | Side-by-side discrete vs. refined B-spline |
| `main_ctrls.csv` | B-spline control points for the main solution |
| `comm_events.csv` | Comm fusion events (when `track_comm_events: true`) |

When `astar_mode: limit`, additional Pareto-seed files per seed `N`:

| File | Description |
|---|---|
| `fig_pareto_discrete.png` | Pareto front (distance vs. uncertainty) |
| `pareto_<N>fig_compare_discrete_continuous_<len>.png` | Discrete vs. continuous for each Pareto seed |
| `fig_pareto_continuous_overlay.png` | All refined Pareto paths overlaid |
| `pareto_<N>_ctrls.csv` | Control points for each Pareto seed |



## Guarantees

- A* ordering key: `f = g + (1 + primary_epsilon) × h` where `h` is Floyd-Warshall distance to goal.
- With `primary_epsilon = 0`, this is standard A* for primary-cost search.
- Strict feasibility is always evaluated against `unc_radius_threshold` (with `unc_feas_tol`).
- Relaxed discrete mode is a seed-generation speed mechanism; the final solution is always checked against the strict threshold.
- Dominance pruning uses the PSD partial order on 2×2 covariance matrices (stronger than scalar det criterion).
- Static-obstacle feasibility is chance-constrained per (agent, obstacle) pair. PSD dominance pruning stays sound under it: since frontier labels share the same node (same mean), a covariance-dominating state clears every obstacle face the dominated one clears.

## Further reading

See `COVARIANCE_MODEL.md` for a derivation of how the Kalman propagation, landmark fusion, and inter-agent cooperative localization map to the paper equations.
