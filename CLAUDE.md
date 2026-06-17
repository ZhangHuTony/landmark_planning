# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the planner

```bash
# Run a single scenario (outputs PNGs and CSVs to the current directory)
cd results_single && julia ../planner.jl

# Run all four scenarios into their respective results_* directories
bash run_scenarios.sh
```

Julia 1.12+ is required (installed at `~/.juliaup/bin/julia`). Required packages (`Plots`, `DataStructures`) are pre-installed in the default environment. The script has no `Project.toml`; it relies on the global depot.

To switch scenario without the shell script, edit line 39 of `planner.jl`:
```julia
const LANDMARK_SCENARIO = :single   # :single | :dual | :clustered | :shoreline
```

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
- **Landmark fusion**: information-filter (Joseph form) Kalman update. Detection probability falls off as `exp(-d²/(2·VISIBILITY_SIGMA²))`; low-probability observations are up-weighted in noise to reduce their influence.
- **Inter-agent fusion**: bidirectional Kalman fusion at fixed `COMM_INTERVAL` arc-distance checkpoints, weighted by `exp(-d²/(2·COMM_SIGMA²))`.
- **Scalar metric**: `unc_radius(cov) = det(cov)^0.25` — equal to `σ` for isotropic covariance.

## Key tuning knobs (top of `planner.jl`)

| Constant | Effect |
|---|---|
| `UNC_RADIUS_THRESHOLD` | Feasibility bound on primary goal uncertainty |
| `ASTAR_ITERATION_LIMIT` | Max A* expansions before stopping collection |
| `ASTAR_MODE` | `:limit` (Pareto collection) vs `:threshold` (first feasible) |
| `PRIMARY_EPSILON` | Weighted A* suboptimality factor (0 = exact) |
| `NUM_AGENTS` | Total agents including primary (last index) |
| `ENABLE_RELAXED_DISCRETE_FOR_CONTINUOUS` | Allow discrete seeds above strict threshold |
| `CONT_OPT_ITERS`, `CONT_OPT_LR` | Continuous optimizer budget and learning rate |
| `HEX_WIDTH_M` | Hex cell size; controls graph resolution |

## Outputs

Each run (from the working directory) produces:
- `fig1_joint_discrete_astar.png` — discrete A* solution
- `main_ctrls.csv` / `pareto_N_ctrls.csv` — B-spline control points (CSV)
- `mainfig_compare_discrete_continuous_*.png` — side-by-side discrete vs. continuous comparison
- `fig_pareto_discrete.png` — Pareto front plot (`:limit` mode only)
- `fig_pareto_continuous_overlay.png` — all refined Pareto paths overlaid

The `run_scenarios.sh` script sets the working directory to `results_<scenario>/` before running, so each scenario's outputs land in the correct folder.
