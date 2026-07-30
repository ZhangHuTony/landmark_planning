# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the planner

```bash
# Uses config.yaml at the repo root; outputs land in results/<timestamp>/
julia planner.jl

# Or with an explicit config file
julia planner.jl path/to/other_config.yaml
```

Julia 1.12+ is required (installed at `~/.juliaup/bin/julia`). Required packages (`Plots`, `DataStructures`) are pre-installed in the default environment. The script has no `Project.toml`; it relies on the global depot. Config parsing is a hand-rolled line parser (no YAML package dependency).

Scenario is chosen via `landmark_scenario` in `config.yaml`, overridable with the `SCENARIO` env var: `single | dual | clustered | shoreline | two_routes | gauntlet | behind_wall | long_sparse | manual`. See `README.md` for the full config reference.

## Architecture

Everything lives in a single file: `planner.jl`. It runs top-to-bottom as a script. The pipeline has three stages:

### 1. Graph construction (`build_hex_graph`)
Builds a **heading-aware hex grid** over the start→goal corridor. Each graph node is a `(hex_cell, heading)` pair, so turns are constrained to ±60° per step (forward, forward-left, forward-right). Sensor landmarks are appended as extra nodes after the routing states but are never traversed — they exist only for covariance fusion. The terminal goal node is always `graph.n` (the last node).

### 2. Discrete joint A* (`joint_astar` / `joint_astar_collect`)
Searches the joint state space of all agents simultaneously. The last agent is the **primary**; earlier agents are **supports**. Key design decisions:
- Priority key: `f = primary_dist + (1 + PRIMARY_EPSILON) * h`, where `h` is the Floyd-Warshall shortest-path distance from the primary's current node to goal.
- Pareto pruning via covariance PSD dominance: state A dominates B if `A.dist ≤ B.dist` and `cov_dominates(A.cov, B.cov)` (i.e., `B.cov - A.cov` is PSD).
- Covariance along each edge is propagated via `edge_cov_continuous` (discrete straight-line samples + information-filter Kalman update from visible landmarks).
- `ASTAR_MODE = :limit` collects all goal states up to `ASTAR_ITERATION_LIMIT` and exposes a Pareto front; `:threshold` stops on first feasible solution.
- Collected Pareto seeds are stored in the global `PARETO_COLLECTED` for later continuous refinement.

### 3. Continuous B-spline refinement (`optimize_continuous`)
Takes discrete node-sequence seeds and refines them as **clamped cubic B-splines**:
1. **Smoothing phase**: gradient-descent (Adam) on a feasibility-recovery objective to push uncertainty below threshold.
2. **Barrier phase**: barrier method with `CONT_BARRIER_STAGES` stages and decaying `μ`, minimizing primary path length subject to uncertainty and curvature constraints (`MAX_CURVATURE = 1/MIN_TURN_RADIUS_M`).
- Gradients are computed by finite differences (step size `CONT_OPT_H`).
- Control points for start/goal are fixed; only interior waypoints are free variables.

### Kalman / uncertainty model
- **State**: 2×2 position covariance matrix per agent.
- **Propagation**: dead-reckoning growth along direction of travel, anisotropic (`DIR_UNCERTAINTY_PER_METER` along-track, `PERP_UNCERTAINTY_PER_METER = DIR/3` cross-track), heading-rotated.
- **Landmark fusion**: information-filter (Joseph form) Kalman update. Detection probability is a logistic sigmoid on distance (plateaus near 1 within `VISIBILITY_RANGE`, rolls off over `VISIBILITY_WIDTH`, no hard cutoff); low-probability observations are up-weighted in noise to reduce their influence.
- **Inter-agent fusion**: bidirectional Kalman fusion at fixed `COMM_INTERVAL` arc-distance checkpoints, weighted by `comm_weight` — a logistic sigmoid taper, half-weight at `COMM_RANGE`, transition softness `COMM_WIDTH`.
- **Scalar metric**: `unc_radius(cov) = det(cov)^0.25` — equal to `σ` for isotropic covariance.

## Key tuning knobs (`config.yaml`)

All parameters live in `config.yaml`; `planner.jl` just reads `CFG[...]` into `const`s at the top of the file. Full reference in `README.md`; most commonly changed:

| Key | Effect |
|---|---|
| `unc_radius_threshold` | Feasibility bound on primary goal uncertainty |
| `astar_iteration_limit` | Max A* expansions before stopping collection |
| `astar_mode` | `limit` (Pareto collection) vs `threshold` (first feasible) |
| `primary_epsilon` | Weighted A* suboptimality factor (0 = exact) |
| `num_agents` | Total agents including primary (last index) |
| `enable_relaxed_discrete` | Allow discrete seeds above strict threshold |
| `cont_opt_iters`, `cont_opt_lr` | Continuous optimizer budget and learning rate |
| `hex_width_m` | Hex cell size; controls graph resolution |

Scenarios own their landmarks, obstacles AND start/goal — all defined in `SCENARIOS` in `src/scenario_generation.jl`, not config. Config only names one (`landmark_scenario:`), or defines one inline with `landmark_scenario: manual`.

## Outputs

Each run writes to a fresh timestamped directory `results/<yyyy-mm-dd_HH-MM-SS>/`, including a copy of the `config.yaml` used:
- `fig1_joint_discrete_astar.png` — discrete A* solution
- `main_ctrls.csv` / `pareto_N_ctrls.csv` — B-spline control points (CSV)
- `mainfig_compare_discrete_continuous_*.png` — side-by-side discrete vs. continuous comparison
- `fig_pareto_discrete.png` — Pareto front plot (`:limit` mode only)
- `fig_pareto_continuous_overlay.png` — all refined Pareto paths overlaid
- `results.yaml` — summary of iteration counts, discrete/continuous uncertainties, and Pareto seed stats
- `comm_events.csv` — inter-agent fusion events, when `track_comm_events: true`

## Workflow
- Ask clarifying questions before starting any complex or ambiguous task
- Unless told otherwise look at the files in `notes/` to get the context of the task.
- look at `notes/PLAN.md` to get a high level overview of what tasks we want to implement as well as how we plan to implement them.
- look at `notes/LOGS.md` to see where we are currently in the plan as well as what things worked and didn't work.
- make minimal changes - do not refactor unrelated code use ponytail
- create seperate commits per logical change, not one giant commit and after every commit update the files in `notes/`. 
- Never edit any code that has to do with any mathematical calculations without explicit approval.