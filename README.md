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

1. Build heading-aware hex graph from `start_x/y`, `goal_x/y`, and the selected landmark scenario.
2. Run joint discrete A* (`joint_astar`) for support + primary paths.
3. Run continuous B-spline refinement to minimize primary path length while enforcing uncertainty and curvature constraints.

## Configuration (`config.yaml`)

All parameters are set in `config.yaml`. The most commonly changed ones are described below.

### Mission setup

| Key | Default | Effect |
|---|---|---|
| `landmark_scenario` | `shoreline` | Landmark layout: `single`, `dual`, `clustered`, `shoreline` |
| `num_agents` | `2` | Total agents including the primary (last index) |
| `astar_mode` | `threshold` | `threshold`: stop on first feasible path; `limit`: collect full Pareto front |
| `start_x/y`, `goal_x/y` | `(0,0)→(1000,0)` | Mission endpoints in meters |

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

Hard no-go **convex-polygon** regions (set in `config/main.yaml`), enforced as a **chance-constrained feasibility filter inside the discrete joint A\***: a joint search state is rejected if any agent's belief has too high a collision probability. This is a pure feasibility check — obstacles never enter the measurement model and never modify covariance propagation. (Continuous B-spline refinement is currently obstacle-unaware.)

Per agent *i* and obstacle *m*, at the state's mean μᵢ and covariance Σᵢ, the agent is **safe** iff it clears at least one polygon face *j*:

```
aⱼ·μᵢ ≥ bⱼ + r_a + z·√(aⱼᵀ(Σᵢ + Σₒ)aⱼ),   z = Φ⁻¹(1 − δ)
```

where the aⱼ are **unit** outward face normals, r_a the agent radius, and Σₒ the obstacle's optional location covariance (default zero). A joint state is infeasible if any agent collides with any obstacle. By the union bound, `P(any collision) ≤ Σδ` over all (agent, obstacle) pairs, so set `δ = Δ/(P·M)` for a total risk budget Δ across P agents and M obstacles.

| Key | Default | Effect |
|---|---|---|
| `obstacles` | *(none)* | Convex polygons (see encoding below). Absent/empty ⇒ no obstacles, filter is a no-op. |
| `obstacle_delta` | `0.05` | Collision-risk bound δ per (agent, obstacle) pair |
| `agent_radius` | `0.0` | Vehicle radius r_a; inflates every obstacle face outward |
| `obstacle_edge_samples` | `2` | Samples along each hex edge: `1` = destination node only, `2` = both endpoints, `≥3` adds interior points |

**Encoding** — a flat one-line string (the parser stores it verbatim; the planner parses it): vertices `x,y` separated by `;`, polygons by `|`, with an optional per-polygon location covariance after `@` as `sxx,sxy,syx,syy` (row-major). Polygons must be **convex** — a non-convex polygon errors out (convex decomposition is out of scope). Φ⁻¹ is computed in-repo (Acklam's rational approximation), since `Distributions`/`StatsFuns` are not installed.

```yaml
obstacles: 200,-40; 260,-40; 260,40; 200,40 | 400,10; 460,10; 430,70 @ 4,0,0,4
```

Obstacles are drawn as filled gray polygons on every planner figure. See `test_obstacles.jl` for the predicate's validation checks.

### Landmark scenarios

| `landmark_scenario` | Landmarks | Notes |
|---|---|---|
| `single` | 1 at (600, −250) | Minimal observation geometry |
| `dual` | 2 at (700, 200) and (750, −250) | Off-axis placement creates incentive to deviate from shortest path |
| `clustered` | 3 near (700, −200) | Tests behaviour when all fixes come from one region |
| `shoreline` | 5 along y ≈ −220 to −300 | Simulates a shoreline; provides cross-track observability |

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
