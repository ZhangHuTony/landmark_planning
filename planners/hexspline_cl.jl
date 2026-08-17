# ==========================================================================
# hexspline_cl: joint discrete A* + continuous B-spline refinement
# ==========================================================================
# Algorithm-specific tuning. Only materialized when this planner is selected
# (see src/config.jl / config/hexspline_cl.yaml).

const PRIMARY_EPSILON = Float64(CFG["primary_epsilon"])

# UNC_RADIUS_THRESHOLD / UNC_FEAS_TOL / MIN_TURN_RADIUS_M / MAX_CURVATURE are the
# shared problem constraints and live in src/config.jl (config/main.yaml), not here.

const ENABLE_RELAXED_DISCRETE_FOR_CONTINUOUS = Bool(CFG["enable_relaxed_discrete"])
const RELAXED_DISCRETE_DELTA_MODE = Symbol(CFG["relaxed_discrete_delta_mode"])
const RELAXED_DISCRETE_DELTA_ABS  = Float64(CFG["relaxed_discrete_delta_abs"])
const RELAXED_DISCRETE_DELTA_REL  = Float64(CFG["relaxed_discrete_delta_rel"])
const CONTINUE_ASTAR_ON_INFEASIBLE = Bool(CFG["continue_astar_on_infeasible"])

const PRUNE_BY_COMM_RADIUS_JOINT    = Bool(CFG["prune_by_comm_radius_joint"])
const PRUNE_BY_PRIMARY_UNCERTAINTY  = Bool(CFG["prune_by_primary_uncertainty"])
const PRUNE_BY_SUPPORT_UNCERTAINTY  = Bool(CFG["prune_by_support_uncertainty"])

const STRAIGHT_CONT_PRIMARY_WPTS = Int(CFG["straight_cont_primary_wpts"])
const ASTAR_ITERATION_LIMIT      = Int(CFG["astar_iteration_limit"])
const CONT_OPT_ITERS = Int(CFG["cont_opt_iters"])
const CONT_OPT_LR    = Float64(CFG["cont_opt_lr"])

const SUPPORT_IDLE_PENALTY   = Float64(CFG["support_idle_penalty"])
const SUPPORT_PLOT_OFFSET_M = Float64(CFG["support_plot_offset_m"])

const SPLINE_DEGREE                    = Int(CFG["spline_degree"])
const SPLINE_SAMPLES_PER_SEG           = Int(CFG["spline_samples_per_seg"])
const SPLINE_CURVATURE_SAMPLES_PER_SEG = Int(CFG["spline_curvature_samples_per_seg"])
const CONT_BARRIER_START               = Float64(CFG["cont_barrier_start"])
const CONT_BARRIER_DECAY               = Float64(CFG["cont_barrier_decay"])
const CONT_BARRIER_STAGES              = Int(CFG["cont_barrier_stages"])
const CONT_LINESEARCH_SHRINK           = Float64(CFG["cont_linesearch_shrink"])
const CONT_MIN_IMPROVEMENT             = Float64(CFG["cont_min_improvement"])

const CONT_OPT_H             = Float64(CFG["cont_opt_h"])
# Finite-difference probe used while still infeasible — see config/hexspline_cl.yaml.
# Only the step width differs from CONT_OPT_H; same objective, same loop.
const CONT_RECOVER_H         = Float64(CFG["cont_recover_h"])
const CONT_ADAM_B1           = Float64(CFG["cont_adam_b1"])
const CONT_ADAM_B2           = Float64(CFG["cont_adam_b2"])
const CONT_ADAM_EPS          = Float64(CFG["cont_adam_eps"])
const CONT_CONV_TOL          = Float64(CFG["cont_conv_tol"])
const CONT_UNC_USE_WAYPOINTS = Bool(CFG["cont_unc_use_waypoints"])

# ε — the knee where the barrier's log switches to its own tangent, and the
# strict-interior margin a seed is expected to clear. See config/hexspline_cl.yaml
# for the mixed-units caveat.
const CONT_RESTORE_MARGIN = Float64(CFG["cont_restore_margin"])

# Relaxed discrete acceptance for discrete->continuous pipeline.
# When enabled, discrete search may return seeds with goal uncertainty up to
# UNC_RADIUS_THRESHOLD + δ, and continuous refinement tries to reduce below the
# true threshold. Delta model:
#   :absolute => δ = RELAXED_DISCRETE_DELTA_ABS
#   :relative => δ = RELAXED_DISCRETE_DELTA_REL * threshold
@inline function discrete_relaxation_delta(threshold::Float64=UNC_RADIUS_THRESHOLD)
    if !ENABLE_RELAXED_DISCRETE_FOR_CONTINUOUS
        return 0.0
    end
    if RELAXED_DISCRETE_DELTA_MODE == :absolute
        return max(0.0, RELAXED_DISCRETE_DELTA_ABS)
    elseif RELAXED_DISCRETE_DELTA_MODE == :relative
        return max(0.0, RELAXED_DISCRETE_DELTA_REL * threshold)
    else
        error("Unsupported RELAXED_DISCRETE_DELTA_MODE=$(RELAXED_DISCRETE_DELTA_MODE). Use :absolute or :relative")
    end
end

@inline function effective_discrete_unc_threshold(threshold::Float64=UNC_RADIUS_THRESHOLD)
    return threshold + discrete_relaxation_delta(threshold)
end

# Single-agent A* state: (node, cumulative distance, covariance at node).
# Dominance is over (dist, covariance) using PSD order: state A dominates B iff
# A.dist ≤ B.dist AND (B.cov - A.cov) is positive semidefinite (cov_dominates).
# NOTE (obstacle chance constraint): `cov_dominates` is the Loewner/PSD order and
# frontier labels are keyed per node-signature (identical μ). At equal μ, PSD order
# implies aⱼᵀΣₐaⱼ ≤ aⱼᵀΣ_baⱼ for every face, so a dominating state clears every
# obstacle face the dominated one clears — dominance stays SOUND under the
# projected-variance obstacle test. The det-based `unc_radius` is used only for
# priority tie-breaking and the uncertainty-threshold gate, never as obstacle
# pruning, so no change is needed here. (See report; the det-vs-Loewner concern
# does not apply to this codebase's pruning.)
# `visited` is a BitVector per state for cycle detection (supports arbitrary graph sizes).
struct State
    node::Int
    dist::Float64
    cov::Matrix{Float64}
    parent::Int
    visited::BitVector
end

# --------------------------------------------------------------------------
# Clamped cubic B-spline helpers for continuous refinement
# --------------------------------------------------------------------------
@inline function pt_sub(a::Tuple{Float64,Float64}, b::Tuple{Float64,Float64})
    return (a[1] - b[1], a[2] - b[2])
end

@inline function pt_scale(a::Tuple{Float64,Float64}, s::Float64)
    return (a[1] * s, a[2] * s)
end

@inline function pt_dist(a::Tuple{Float64,Float64}, b::Tuple{Float64,Float64})
    return hypot(a[1] - b[1], a[2] - b[2])
end

function bspline_pad_controls(control_pts::Vector{Tuple{Float64,Float64}})
    n = length(control_pts)
    if n >= 4
        return control_pts
    elseif n == 3
        return [control_pts[1], control_pts[1], control_pts[2], control_pts[3], control_pts[3]]
    elseif n == 2
        return [control_pts[1], control_pts[1], control_pts[2], control_pts[2]]
    elseif n == 1
        return [control_pts[1], control_pts[1], control_pts[1], control_pts[1]]
    else
        return Tuple{Float64,Float64}[]
    end
end

function bspline_open_uniform_knots(nctrl::Int, degree::Int)
    nctrl >= degree + 1 || return Float64[]
    knots = Vector{Float64}(undef, nctrl + degree + 1)
    nspans = max(1, nctrl - degree)
    for i in 1:length(knots)
        if i <= degree + 1
            knots[i] = 0.0
        elseif i > nctrl
            knots[i] = 1.0
        else
            knots[i] = (i - degree - 1) / nspans
        end
    end
    return knots
end

@inline function bspline_find_span(u::Float64, knots::Vector{Float64}, degree::Int, nctrl::Int)
    u >= knots[end - degree] && return nctrl
    low = degree + 1
    high = nctrl
    while low <= high
        mid = (low + high) >>> 1
        if u < knots[mid]
            high = mid - 1
        elseif u >= knots[mid + 1]
            low = mid + 1
        else
            return mid
        end
    end
    return clamp(low, degree + 1, nctrl)
end

function bspline_basis_funs(span::Int, u::Float64, degree::Int, knots::Vector{Float64})
    N = zeros(Float64, degree + 1)
    left = zeros(Float64, degree)
    right = zeros(Float64, degree)
    N[1] = 1.0
    for j in 1:degree
        left[j] = u - knots[span + 1 - j]
        right[j] = knots[span + j] - u
        saved = 0.0
        for r in 1:j
            denom = right[r] + left[j - r + 1]
            temp = abs(denom) < 1e-14 ? 0.0 : N[r] / denom
            N[r] = saved + right[r] * temp
            saved = left[j - r + 1] * temp
        end
        N[j + 1] = saved
    end
    return N
end

function bspline_eval_point(control_pts::Vector{Tuple{Float64,Float64}},
                            knots::Vector{Float64},
                            degree::Int,
                            u::Float64)
    nctrl = length(control_pts)
    nctrl == 0 && return (0.0, 0.0)
    u = clamp(u, 0.0, 1.0)
    span = bspline_find_span(u, knots, degree, nctrl)
    basis = bspline_basis_funs(span, u, degree, knots)
    first = span - degree
    x = 0.0
    y = 0.0
    for j in 0:degree
        pt = control_pts[first + j]
        x += basis[j + 1] * pt[1]
        y += basis[j + 1] * pt[2]
    end
    return (x, y)
end

function bspline_derivative_control_points(control_pts::Vector{Tuple{Float64,Float64}},
                                           knots::Vector{Float64},
                                           degree::Int)
    nctrl = length(control_pts)
    nctrl <= 1 && return Tuple{Float64,Float64}[], Float64[], max(degree - 1, 0)
    dctrl = Vector{Tuple{Float64,Float64}}(undef, nctrl - 1)
    for i in 1:nctrl - 1
        denom = knots[i + degree + 1] - knots[i + 1]
        if abs(denom) < 1e-14
            dctrl[i] = (0.0, 0.0)
        else
            dctrl[i] = pt_scale(pt_sub(control_pts[i + 1], control_pts[i]), degree / denom)
        end
    end
    return dctrl, knots[2:end-1], degree - 1
end

function bspline_sample_path(control_pts::Vector{Tuple{Float64,Float64}};
                             degree::Int = SPLINE_DEGREE,
                             length_samples_per_seg::Int = SPLINE_SAMPLES_PER_SEG,
                             curvature_samples_per_seg::Int = SPLINE_CURVATURE_SAMPLES_PER_SEG)
    controls = bspline_pad_controls(control_pts)
    nctrl = length(controls)
    if nctrl == 0
        return Tuple{Float64,Float64}[], Float64[]
    elseif nctrl < degree + 1
        return copy(controls), zeros(Float64, max(0, length(controls) - 1))
    end

    knots = bspline_open_uniform_knots(nctrl, degree)
    nspans = max(1, nctrl - degree)

    n_pos_samples = nspans * length_samples_per_seg + 1
    pos_samples = Vector{Tuple{Float64,Float64}}(undef, n_pos_samples)
    for i in 1:n_pos_samples
        u = (i - 1) / (n_pos_samples - 1)
        pos_samples[i] = bspline_eval_point(controls, knots, degree, u)
    end

    d1_ctrl, d1_knots, d1_degree = bspline_derivative_control_points(controls, knots, degree)
    d2_ctrl, d2_knots, d2_degree = bspline_derivative_control_points(d1_ctrl, d1_knots, d1_degree)

    curv_us = collect(range(0.0, 1.0; length = nspans * curvature_samples_per_seg + 1))
    if length(curv_us) > 2
        curv_us = curv_us[2:end-1]
    end

    curvatures = Vector{Float64}(undef, length(curv_us))
    for (i, u) in enumerate(curv_us)
        dx, dy = bspline_eval_point(d1_ctrl, d1_knots, d1_degree, u)
        ddx, ddy = bspline_eval_point(d2_ctrl, d2_knots, d2_degree, u)
        speed2 = dx * dx + dy * dy
        curvatures[i] = speed2 < 1e-12 ? Inf : abs(dx * ddy - dy * ddx) / max(speed2^(1.5), 1e-12)
    end

    return pos_samples, curvatures
end

@inline function bspline_path_length(samples::Vector{Tuple{Float64,Float64}})
    total = 0.0
    for i in 2:length(samples)
        total += pt_dist(samples[i - 1], samples[i])
    end
    return total
end

function pad_path_to_length(path::Vector{Int}, target_len::Int)
    isempty(path) && return Int[]
    padded = copy(path)
    while length(padded) < target_len
        push!(padded, padded[end])
    end
    return padded
end

function pad_support_paths_to_primary(support_paths::Vector{Vector{Int}}, primary_path::Vector{Int})
    target_len = length(primary_path)
    return [pad_path_to_length(path, target_len) for path in support_paths]
end

# THE discrete-seed → B-spline control-point map. Node centres, supports padded to
# the primary's waypoint count, and the primary's last waypoint pinned to the exact
# goal (it ends on the goal *cell*, whose centre drifts off the goal). Padding is
# idempotent, so this is safe on already-padded paths. Shared by optimize_continuous
# and the seed gate below — they must see the same spline or the gate proves nothing.
function seed_control_points(agent_paths::Vector{Vector{Int}}, graph::LandmarkGraph)
    na = length(agent_paths)
    paths = na == 1 ? agent_paths :
            vcat(pad_support_paths_to_primary(agent_paths[1:na-1], agent_paths[na]), [agent_paths[na]])
    ctrls = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
    ctrls[na][end] = (graph.landmarks[graph.n].x, graph.landmarks[graph.n].y)
    return ctrls
end

# Does this goal candidate survive as a SPLINE? The discrete filter only tests the
# straight polyline between hex centres (src/obstacles.jl); the refined B-spline cuts
# those corners off, so a polyline-clean seed can still enter an obstacle. Gate every
# goal candidate on the same committed-face MINVO test the continuous stage enforces
# (src/minvo.jl), at the same control points and the same Σ — so a candidate that
# passes here starts the barrier phase already feasible on its obstacle rows.
# Costs one evaluate_joint_discrete per goal pop, and only in obstacle scenarios.
#
# Same story for the SUPPORT CAP, which is why it is re-tested here. The search
# prunes on graph distance (`support_dist_next <= primary_dist`), but the optimizer
# constrains spline length — and corner-cutting takes a different bite out of each
# agent, so two paths tied at equal graph distance can still come out unequal once
# splined. Nothing enforced the spline version, so A* was free to hand the barrier a
# seed already violating it; the barrier then had to LENGTHEN the primary to recover
# instead of shortening it. Cheap enough to run unconditionally (a spline resample
# per agent, at goal pops only — not per expansion, where a partial path's length is
# not even meaningful).
function seed_spline_clear(agent_paths::Vector{Vector{Int}}, graph::LandmarkGraph,
                           lms::Vector{Landmark})
    ctrls = seed_control_points(agent_paths, graph)
    na = length(ctrls)
    if na > 1
        # Same expression as the optimizer's slacks_from support row, on the same
        # control points, so a seed that passes here starts feasible on those rows.
        seed_len(c) = bspline_path_length(first(bspline_sample_path(c)))
        plen = seed_len(ctrls[na])
        for a in 1:(na - 1)
            plen + UNC_FEAS_TOL - seed_len(ctrls[a]) >= 0.0 || return false
        end
    end
    (isempty(OBSTACLES) || !OBSTACLE_CONTINUOUS) && return true
    covs, _, _, _ = evaluate_joint_discrete(ctrls, lms, length(ctrls))
    return obstacles_clear(build_obstacle_plan(ctrls, SPLINE_DEGREE), ctrls, covs)
end

# ==========================================================================
# Joint A* over all agents simultaneously
# ==========================================================================
#
# STATE
# -----
# A JointState encodes the full multi-agent configuration at one instant:
#   nodes   : current graph node for each agent (length num_agents)
#   dists   : cumulative arc-distance travelled by each agent
#   covs    : position-estimate covariance for each agent
#   g       : true cost = primary arc-distance (agents[end] is primary)
#   parent  : index into the states vector (-1 = root)
#   visited : per-agent BitVector for cycle detection
#
# HEURISTICS
# ----------
# Each agent has its own admissible heuristic:
#
#   Primary (last agent):
#     h_primary(v) = Floyd-Warshall shortest graph-path distance from v to goal.
#     Precomputed in graph.shortest_paths; O(1) lookup at each expansion.
#     This lower-bounds the remaining primary arc-distance (graph path ≤ any path),
#     so f = g + h_primary is admissible (never overestimates total primary length).
#
#   Support agent a:
#     The support does not need to reach the goal — it only needs to position
#     itself so that inter-agent comms help the primary. Its heuristic
#     contribution to f is ZERO (supports add no remaining mandatory cost to
#     the primary objective). Using a non-zero heuristic for supports would
#     risk inadmissibility because we cannot bound how far a support must
#     still travel.  h_support = 0 is trivially admissible.
#
# ADMISSIBILITY PROOF
# -------------------
# f(s) = g(s) + h(s)
#       = primary_dist_so_far + h_primary(primary_node)
#       ≤ primary_dist_so_far + remaining_primary_dist   (shortest graph path ≤ actual path)
#       ≤ true_total_primary_dist
# So f never overestimates — A* is admissible and returns optimal primary length.
#
# DOMINANCE / PARETO PRUNING
# --------------------------
# Two states at the same joint node-tuple (nodes[1..K]) are compared on:
#   (primary_dist, determinant-based uncertainty)
# State A dominates B iff A.g ≤ B.g AND unc_radius(A.covs[end]) ≤ unc_radius(B.covs[end]).
# Dominated states are discarded.  This is sound because:
#   (a) A's remaining primary cost cannot exceed B's (same heuristic from same node)
#   (b) A's primary uncertainty at goal cannot exceed B's (covariance only grows)
# so B can never produce a solution better than A.
#
# INTER-AGENT COMMUNICATION WITHIN THE SEARCH
# --------------------------------------------
# To keep the heuristic consistent with the final evaluate_joint_discrete evaluation we
# apply the same communication model inside edge_cov_continuous — but only
# when the arc-distance gap between agents falls within one COMM_INTERVAL_DIST
# window.  For each edge expansion we compute a lightweight pairwise comm
# update between the newly-expanded agent and each other agent at their
# current arc position, mirroring apply_inter_agent_discrete_comms! at graph resolution.
#
# CYCLE DETECTION
# ---------------
# Each agent carries its own BitVector of visited nodes so that the same node
# is never revisited on one agent's path (prevents infinite loops).  Different
# agents may visit the same node independently.
#
# COMPLEXITY NOTE
# ---------------
# The joint state space is O(n^K) for K agents and n nodes.  With K=3 and
# n≈130 nodes the naive bound is ~2.2M node-tuples.  Pareto pruning keeps the
# live frontier small in practice: each node-tuple retains only its Pareto-
# non-dominated (dist, unc) states, which are few for well-separated landmarks.
# The coarse comm model inside the search (graph-resolution, not Bézier-
# resolution) is an approximation — the Bézier stage later refines it — but
# it is consistent with the heuristic and keeps each expansion O(n_lm) cheap.
#
# EXPANSION STRATEGY
# ------------------
# At each step we advance ALL agents by one edge (round-robin by agent index),
# keeping the joint state synchronised by arc-distance.  Specifically, we
# always expand the agent that is furthest behind in arc-distance, breaking
# ties by agent index.  This mimics simultaneous travel without exponentiating
# the branching factor: at each expansion only one agent moves.
#
# The primary's arc-distance drives the f-value.  Support agents' arc-distance
# is bounded to [0, primary_dist] — supports must not travel further than
# the primary.


@inline function supports_within_comm_radius(nodes::Vector{Int}, graph::LandmarkGraph, primary::Int)
    primary_node = nodes[primary]
    px = graph.landmarks[primary_node].x
    py = graph.landmarks[primary_node].y
    for a in 1:(primary - 1)
        node = nodes[a]
        dx = graph.landmarks[node].x - px
        dy = graph.landmarks[node].y - py
        if dx * dx + dy * dy > COMM_RADIUS^2
            return false
        end
    end
    return true
end

struct JointState
    nodes   :: Vector{Int}          # current graph node per agent; last = primary
    covs    :: Vector{Matrix{Float64}}  # covariance at end of each path
    dists   :: Vector{Float64}      # arc-distance per agent (computed from B-spline)
    g       :: Float64              # primary arc-distance (cost)
    parent  :: Int                  # index in states vector; -1 = root
    visited :: Vector{BitVector}    # per-agent visited sets (cycle detection)
end

# The full path per agent used to be stored on every JointState, alongside the
# `parent` that already determines it. That cost O(depth) per state and nothing
# is ever freed until the search returns, so it set the memory ceiling that
# `astar_iteration_limit` really enforces. Rebuild it from the parent chain
# instead — the single-agent branch above (`State`) always worked this way.
# Called only at goal checks and in the debug plot, not on the hot path.
function joint_trace(states::Vector{JointState}, si::Int, na::Int)
    chain = Int[]
    while si != -1
        push!(chain, si)
        si = states[si].parent
    end
    reverse!(chain)
    return [[states[i].nodes[a] for i in chain] for a in 1:na]
end

@inline function joint_heuristic(nodes::Vector{Int},
                                  goal::Int,
                                  graph::LandmarkGraph)
    return graph.shortest_paths[nodes[end], goal]  # primary is the last agent
end

# ------------------------------------------------------------------
# Secondary tie-breaker score for support idling.
# Lower is better. Used after weighted primary f-values are compared.
# ------------------------------------------------------------------
@inline function support_idle_score(dists::Vector{Float64})
    na = length(dists)
    idle_count = 0
    for a in 1:(na - 1)
        dists[a] <= 1e-9 && (idle_count += 1)
    end
    return SUPPORT_IDLE_PENALTY * idle_count
end

@inline joint_node_key(nodes::Vector{Int}) = Tuple(nodes)

@inline function joint_visited_key(visited::Vector{BitVector})
    sig = Vector{Tuple{Vararg{Int}}}(undef, length(visited))
    for a in 1:length(visited)
        sig[a] = Tuple(findall(visited[a]))
    end
    return Tuple(sig)
end

function apply_joint_step_comms(covs::Vector{Matrix{Float64}},
                                nodes::Vector{Int},
                                dists::Vector{Float64},
                                graph::LandmarkGraph)
    na = length(covs)
    updated = copy(covs)
    for a in 1:(na - 1)
        xa = graph.landmarks[nodes[a]].x
        ya = graph.landmarks[nodes[a]].y
        for b in (a + 1):na
            abs(dists[a] - dists[b]) > COMM_INTERVAL_DIST && continue
            xb = graph.landmarks[nodes[b]].x
            yb = graph.landmarks[nodes[b]].y
            new_a, new_b = pairwise_comm(updated[a], updated[b], xa, ya, xb, yb)
            updated[a] = new_a
            updated[b] = new_b
        end
    end
    return updated
end

# Single-line A* status bar: iterations done, iterations left, elapsed. Rewrites
# the same line with \r, so every search loop that calls it must println() once on
# exit to release the line. Plain print + flush — ProgressMeter is not installed
# and this is the whole feature.
const ASTAR_PROGRESS_EVERY = 1000
@inline function astar_progress(iter::Int, limit::Int, t0::Float64)
    mod(iter, ASTAR_PROGRESS_EVERY) == 0 || return
    print("\r  [A*] iter $iter/$limit ($(round(100 * iter / limit, digits=1))%)  " *
          "left=$(max(0, limit - iter))  elapsed=$(round(time() - t0, digits=1))s   ")
    flush(stdout)
end

# ------------------------------------------------------------------
# Constraint-Aware A* Search
# ------------------------------------------------------------------
# Returns (paths, dists, goal_unc) where paths[end] is the primary.
#
# Key Features:
#   - Single joint search on all agents simultaneously
#   - LAZY EVALUATION: only call expensive evaluate_full_paths() when
#     f-value is competitive with best solution found
#   - EPSILON-BOUND ORDERING: primary key is weighted f=g+(1+ε)h using
#     primary shortest-path heuristic; tie-breakers prefer low uncertainty
#     and non-idle supports
#   - SUPPORT CAP: supports cannot exceed primary's arc-distance
#   - COMM RADIUS GATE: supports must stay within direct comm range of the primary
#   - PRIMARY UNCERTAINTY GATE: prune any state whose primary uncertainty exceeds
#     the threshold immediately, rather than allowing later recovery
#   - EARLY STOPPING: return once feasible solution found
#
# How it works:
#   1. Use optimistic lower-bound distances for f-value computation
#   2. Expand the OPEN list in A* order
#   3. Return immediately when the first feasible goal is popped
#
function joint_astar(graph::LandmarkGraph,
                     lms::Vector{Landmark},
                     unc_threshold::Float64,
                     num_agents::Int;
                     seed_paths::Union{Nothing, Vector{Vector{Int}}}=nothing,
                     seed_dists::Union{Nothing, Vector{Float64}}=nothing,
                     debug_animate::Bool=false,
                     debug_animate_start_iter::Int=0,
                     debug_animate_iters::Int=100000,
                     debug_animate_sample_period::Int=100,
                     debug_stop_after_animate::Bool=false,
                     debug_gif_path::String="fig_astar_partial.gif")
    n         = graph.n
    goal      = n                     # last node is goal
    na        = num_agents
    primary   = na                    # index of primary agent (last)
    # Nodes that have a direct in-edge to the terminal goal node — termination fires here.
    # ponytail: assumes stub ≈ 0m (goal on cell centre); heuristic shortest_paths[v,goal] stays valid.
    is_goal_cell = falses(n)
    for v in 1:(n-1); goal in graph.neighbors[v] && (is_goal_cell[v] = true); end

    # Fast path: single-agent weighted A* with incremental covariance updates.
    # Avoids rebuilding/evaluating full trajectories on every expansion.
    if na == 1
        w_astar = 1.0 + PRIMARY_EPSILON
        init_visited = falses(n)
        init_visited[1] = true

        states_sa = State[]
        push!(states_sa, State(1, 0.0, copy(lms[1].cov), -1, init_visited))

        pq_sa = PriorityQueue{Int, Tuple{Float64, Float64}}()
        h0 = graph.shortest_paths[1, goal]
        enqueue!(pq_sa, 1, (w_astar * h0, unc_radius(states_sa[1].cov)))

        iter_count_sa = 0
        t0 = time()

        # Anytime incumbent: the lowest-uncertainty goal candidate seen so far. Only
        # consulted if the search exhausts its budget without one under the threshold
        # (see the return below) -- a feasible goal still returns the instant it pops,
        # so the common case pays nothing for this.
        best_path_sa = Int[]; best_dist_sa = 0.0; best_unc_sa = Inf

        # Node-level Pareto frontier with SOUND covariance dominance.
        # Keep labels (distance, covariance) and only prune when an existing
        # label is better in distance and covariance PSD order.
        frontier_at_node = Dict{Int, Vector{Tuple{Float64, Matrix{Float64}}}}()
        frontier_at_node[1] = [(0.0, copy(states_sa[1].cov))]

        while !isempty(pq_sa) && iter_count_sa < ASTAR_ITERATION_LIMIT
            si = dequeue!(pq_sa)
            S = states_sa[si]
            iter_count_sa += 1
            astar_progress(iter_count_sa, ASTAR_ITERATION_LIMIT, t0)

            popped_unc = unc_radius(S.cov)
            if PRUNE_BY_PRIMARY_UNCERTAINTY && unc_exceeds_threshold(popped_unc, unc_threshold, UNC_FEAS_TOL)
                println("\n  [Constraint A*] Pruned state at iter $iter_count_sa: node=$(S.node), primary_unc=$(round(popped_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
                continue
            end

            h = graph.shortest_paths[S.node, goal]
            if is_goal_cell[S.node]
                path = Int[]
                psi = si
                while psi != -1
                    pushfirst!(path, states_sa[psi].node)
                    psi = states_sa[psi].parent
                end

                exact_covs, exact_dists = evaluate_full_paths([path], graph, lms, 1)
                exact_unc = unc_radius(exact_covs[1])
                if unc_within_threshold(exact_unc, unc_threshold, UNC_FEAS_TOL)
                    if !seed_spline_clear([path], graph, lms)
                        println("\n  [Constraint A*] Goal popped, unc OK, but its B-spline breaches an obstacle; discarded.")
                        continue
                    end
                    println()   # release the status-bar line
                    println("  ✓ FEASIBLE SOLUTION at iter $iter_count_sa: dist=$(round(exact_dists[1], digits=3)), unc=$(round(exact_unc, digits=4))")
                    println("  [Constraint A*] Single-agent complete: $(iter_count_sa) iterations, final_dist=$(round(exact_dists[1], digits=3))")
                    return [path], [exact_dists[1]], exact_unc, iter_count_sa
                end

                # Over threshold, but the barrier can still trade length for uncertainty
                # from here -- record it as a fallback seed. seed_spline_clear stays a
                # HARD gate: obstacle clearance and the support cap are topological, and
                # no local perturbation of the spline re-routes around a wall.
                if exact_unc < best_unc_sa && seed_spline_clear([path], graph, lms)
                    best_path_sa = path; best_dist_sa = exact_dists[1]; best_unc_sa = exact_unc
                    println("\n  [Constraint A*] Incumbent seed at iter $iter_count_sa: dist=$(round(exact_dists[1], digits=3)), unc=$(round(exact_unc, digits=4)) > $(round(unc_threshold, digits=4))")
                end
                continue
            end

            for u in graph.neighbors[S.node]
                S.visited[u] && continue

                nd = S.dist + graph.distance[S.node, u]
                ncov = edge_cov_continuous(S.node, u, graph, lms, S.cov)
                # Chance-constrained static-obstacle feasibility filter (no-op if none).
                if !isempty(OBSTACLES)
                    p0 = graph.landmarks[S.node]; p1 = graph.landmarks[u]
                    segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncov) || continue
                end
                nunc = unc_radius(ncov)

                if !haskey(frontier_at_node, u)
                    frontier_at_node[u] = Tuple{Float64, Matrix{Float64}}[]
                end

                # Dominated if an existing label has <= distance and covariance
                # no larger in PSD order.
                dominated = false
                for (od, ocov) in frontier_at_node[u]
                    if od <= nd + 1e-9 && cov_dominates(ocov, ncov)
                        dominated = true
                        break
                    end
                end
                dominated && continue

                # Remove labels dominated by the new label.
                kept = Tuple{Float64, Matrix{Float64}}[]
                for (od, ocov) in frontier_at_node[u]
                    if nd <= od + 1e-9 && cov_dominates(ncov, ocov)
                        continue
                    end
                    push!(kept, (od, ocov))
                end
                push!(kept, (nd, copy(ncov)))
                frontier_at_node[u] = kept

                nvis = copy(S.visited)
                nvis[u] = true
                push!(states_sa, State(u, nd, ncov, si, nvis))
                nsi = length(states_sa)
                nh = graph.shortest_paths[u, goal]
                nf = nd + w_astar * nh
                enqueue!(pq_sa, nsi, (nf, nunc))
            end
        end

        println()   # release the status-bar line
        if isempty(best_path_sa)
            println("  [Constraint A*] No feasible single-agent solution found")
            return [Int[]], [0.0], Inf, iter_count_sa
        end
        println("  [Constraint A*] Budget exhausted with no seed under threshold; " *
                "handing best incumbent to the optimizer: dist=$(round(best_dist_sa, digits=3)), " *
                "unc=$(round(best_unc_sa, digits=4)) > $(round(unc_threshold, digits=4))")
        return [best_path_sa], [best_dist_sa], best_unc_sa, iter_count_sa
    end

    # ── Initial state: all agents at node 1 (start) ──────────────────────────
    init_paths   = [fill(1, 1) for _ in 1:na]
    init_nodes   = fill(1, na)
    init_visited = [falses(n) for _ in 1:na]
    for a in 1:na; init_visited[a][1] = true; end

    # Evaluate initial state (all agents at node 1)
    init_covs, init_dists = evaluate_full_paths(init_paths, graph, lms, na)

    states = JointState[]
    push!(states, JointState(init_nodes, init_covs, init_dists, 0.0, -1, init_visited))

    # No exact-state dedup cache here. Every expansion appends one node to EVERY
    # agent, so a child's joint path is its parent's plus one step: the search
    # tree is a tree, every state's joint path is unique, and a cache keyed on it
    # could never hit. It only cost an O(depth) tuple per state (measured 0 hits
    # in 565,836 — notes/LOGS.md).

    w_astar = 1.0 + PRIMARY_EPSILON
    pq = PriorityQueue{Int, Tuple{Float64, Float64, Float64}}()
    init_h = joint_heuristic(init_nodes, goal, graph)
    init_f = init_dists[primary] + w_astar * init_h
    init_unc = unc_radius(init_covs[primary])
    enqueue!(pq, 1, (init_f, init_unc, support_idle_score(init_dists)))

    iter_count         = 0

    # Anytime incumbent, as in the single-agent branch above: lowest-uncertainty goal
    # candidate seen, returned only if the budget runs out before a feasible one.
    best_paths_j = Vector{Int}[]; best_dists_j = Float64[]; best_unc_j = Inf

    # Safe dominance frontier keyed by (joint nodes, exact visited signature).
    # This only compares states with identical future action sets.
    frontier_by_signature = Dict{Any, Vector{Tuple{Float64, Matrix{Float64}}}}()
    init_sig = (joint_node_key(init_nodes), joint_visited_key(init_visited))
    frontier_by_signature[init_sig] = [(0.0, copy(init_covs[primary]))]
    
    # Diagnostic: check if goal is reachable from start
    goal_h = graph.shortest_paths[1, goal]
    if isinf(goal_h)
        println("  [WARNING] Goal node $goal is unreachable from start node 1!")
    else
        println("  ✓ Goal reachable from node 1 with distance $goal_h")
    end

    # ── Main loop ────────────────────────────────────────────────────────────

    # --- Animation/Debugging additions ---
    animate_enabled = debug_animate
    animate_start_iter = max(1, debug_animate_start_iter)
    animate_limit = max(1, debug_animate_iters)
    animate_sample_period = max(1, debug_animate_sample_period)
    anim_frames = animate_enabled ? Plots.Animation() : nothing
    animation_saved = false

    function debug_joint_astar_plot(si::Int, best_si::Int, iter_no::Int)
        plt = plot(legend=:outerright, aspect_ratio=:equal,
                   xlabel="x (m)", ylabel="y (m)",
                   title="Constraint A* debug frame $(iter_no)")

        xs = [lm.x for lm in lms]
        ys = [lm.y for lm in lms]
        scatter!(plt, xs, ys, label="Nodes", color=:gray, markersize=3, markerstrokewidth=0)
        scatter!(plt, [graph.landmarks[1].x], [graph.landmarks[1].y],
                 label="Start", color=:green, markersize=7, markerstrokewidth=0)
        scatter!(plt, [graph.landmarks[goal].x], [graph.landmarks[goal].y],
                 label="Goal", color=:orange, marker=:star5, markersize=9, markerstrokewidth=0)

        for (ai, path) in enumerate(joint_trace(states, si, na))
            isempty(path) && continue
            px = [graph.landmarks[j].x for j in path]
            py = [graph.landmarks[j].y for j in path]
            clr = ai == primary ? :blue : :purple
            lbl = ai == primary ? "Primary (expanding)" : "Support $ai (expanding)"
            plot!(plt, px, py, label=lbl, color=clr, linewidth=2.0,
                  linestyle=(ai == primary ? :solid : :dash))
        end

        if best_si > 0
            for (ai, path) in enumerate(joint_trace(states, best_si, na))
                isempty(path) && continue
                px = [graph.landmarks[j].x for j in path]
                py = [graph.landmarks[j].y for j in path]
                clr = ai == primary ? :darkgreen : :darkgray
                lbl = ai == primary ? "Primary (best)" : "Support $ai (best)"
                plot!(plt, px, py, label=lbl, color=clr, linewidth=1.5, alpha=0.8,
                      linestyle=:dot)
            end
        end

        prim_node = states[si].nodes[primary]
        prim_h = isinf(graph.shortest_paths[prim_node, goal]) ? "∞" : "$(round(graph.shortest_paths[prim_node, goal], digits=1))"
        ann = "Iter $iter_no, prim_node=$prim_node, h=$prim_h"
        annotate!(plt, (graph.landmarks[1].x, graph.landmarks[1].y + 100), text(ann, :black, 10))
        return plt
    end

    t0 = time()
    while !isempty(pq) && iter_count < ASTAR_ITERATION_LIMIT
        si  = dequeue!(pq)
        S   = states[si]
        iter_count += 1
        astar_progress(iter_count, ASTAR_ITERATION_LIMIT, t0)

        popped_unc = unc_radius(S.covs[primary])
        if PRUNE_BY_PRIMARY_UNCERTAINTY && unc_exceeds_threshold(popped_unc, unc_threshold, UNC_FEAS_TOL)
            println("\n  [Constraint A*] Pruned joint state at iter $iter_count: primary_unc=$(round(popped_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
            continue
        end

        # --- Animation: plot only the requested iteration window, sampled sparsely ---
        if animate_enabled && iter_count >= animate_start_iter && iter_count <= animate_start_iter + animate_limit - 1 &&
           mod(iter_count - animate_start_iter, animate_sample_period) == 0
            plt = debug_joint_astar_plot(si, 0, iter_count)
            frame(anim_frames, plt)
        end

        if animate_enabled && !animation_saved && iter_count >= animate_start_iter + animate_limit - 1
            println("  [Constraint A*] Debug cutoff reached at iter $iter_count; saving partial animation to $debug_gif_path.")
            gif(anim_frames, debug_gif_path, fps=20)
            animation_saved = true
            if debug_stop_after_animate
                println("  [Constraint A*] Stopping early after animation cutoff.")
                break
            else
                println("  [Constraint A*] Continuing search after animation cutoff.")
            end
        end

        # Standard A* ordering uses f only for priority; no incumbent pruning.
        h = joint_heuristic(S.nodes, goal, graph)
        f = S.g + w_astar * h


        # Check if primary reached goal cell
        if is_goal_cell[S.nodes[primary]]
            agent_paths = joint_trace(states, si, na)
            exact_covs, exact_dists = evaluate_full_paths(agent_paths, graph, lms, na)
            exact_unc = unc_radius(exact_covs[primary])
            if unc_within_threshold(exact_unc, unc_threshold, UNC_FEAS_TOL)
                if !seed_spline_clear(agent_paths, graph, lms)
                    #println("\n  [Constraint A*] Goal popped, unc OK, but its B-spline breaches an obstacle; discarded.")
                    continue
                end
                println()   # release the status-bar line
                println("  ✓ FEASIBLE SOLUTION at iter $iter_count: dist=$(round(exact_dists[primary], digits=3)), unc=$(round(exact_unc, digits=4))")
                println("  [Constraint A*] Complete: $(iter_count) iterations, final_dist=$(round(exact_dists[primary], digits=3))")
                # Save collected debug frames before this early return (bypasses the end-of-function save below).
                if animate_enabled && !animation_saved && iter_count > 1
                    gif(anim_frames, debug_gif_path, fps=20)
                    animation_saved = true
                    println("  [Constraint A*] Animation saved to $debug_gif_path (iters $(animate_start_iter)-$(min(iter_count, animate_start_iter + animate_limit - 1)))")
                end
                return agent_paths, exact_dists, exact_unc, iter_count
            elseif exact_unc < best_unc_j && seed_spline_clear(agent_paths, graph, lms)
                # Fallback seed for the optimizer -- see the single-agent branch above.
                # seed_spline_clear stays hard here too.
                best_paths_j = agent_paths; best_dists_j = exact_dists; best_unc_j = exact_unc
                println("\n  [Constraint A*] Incumbent seed at iter $iter_count: dist=$(round(exact_dists[primary], digits=3)), unc=$(round(exact_unc, digits=4)) > $(round(unc_threshold, digits=4))")
            end
            continue
        end

        # ── Expansion: move all agents synchronously one edge at a time ──────
        candidate_nodes = Vector{Int}(undef, na)
        move_order = Vector{Int}(undef, na)
        move_order[1] = primary
        order_pos = 2
        for a in 1:na
            a == primary && continue
            move_order[order_pos] = a
            order_pos += 1
        end

        function expand_joint_moves(agent_idx::Int, primary_dist::Float64)
            if agent_idx > na
                new_covs = Vector{Matrix{Float64}}(undef, na)
                new_dists = Vector{Float64}(undef, na)
                new_visited = [copy(S.visited[a]) for a in 1:na]

                for a in 1:na
                    u = candidate_nodes[a]
                    prev_node = S.nodes[a]
                    new_dists[a] = S.dists[a] + graph.distance[prev_node, u]
                    new_covs[a] = edge_cov_continuous(prev_node, u, graph, lms, S.covs[a])
                    new_visited[a][u] = true
                end

                # Support agents must not travel further than the primary.
                for a in 1:(primary - 1)
                    new_dists[a] > new_dists[primary] && return
                end

                if PRUNE_BY_COMM_RADIUS_JOINT
                    supports_within_comm_radius(candidate_nodes, graph, primary) || return
                end

                new_covs = apply_joint_step_comms(new_covs, candidate_nodes, new_dists, graph)

                # Chance-constrained static-obstacle feasibility filter: reject if ANY
                # agent's belief collides with ANY obstacle (no-op if none). Uses the
                # post-comm-fusion covariance carried on the state.
                if !isempty(OBSTACLES)
                    for a in 1:na
                        p0 = graph.landmarks[S.nodes[a]]; p1 = graph.landmarks[candidate_nodes[a]]
                        segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), new_covs[a]) || return
                    end
                end

                new_g = new_dists[primary]

                # Support agents must also remain under the uncertainty threshold.
                if PRUNE_BY_SUPPORT_UNCERTAINTY
                    for a in 1:(primary - 1)
                        sup_unc = unc_radius(new_covs[a])
                        if unc_exceeds_threshold(sup_unc, unc_threshold, UNC_FEAS_TOL)
                            # println("  [Constraint A*] Pruned expansion: support $a unc=$(round(sup_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
                            return
                        end
                    end
                end
                

                prim_unc = unc_radius(new_covs[primary])
                if PRUNE_BY_PRIMARY_UNCERTAINTY && unc_exceeds_threshold(prim_unc, unc_threshold, UNC_FEAS_TOL)
                    # println("  [Constraint A*] Pruned expansion: primary_unc=$(round(prim_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
                    return
                end

                new_h = joint_heuristic(candidate_nodes, goal, graph)
                f_exact = new_g + w_astar * new_h

                sig_key = (joint_node_key(candidate_nodes), joint_visited_key(new_visited))
                labels = get(frontier_by_signature, sig_key, Tuple{Float64, Matrix{Float64}}[])
                for (od, ocov) in labels
                    if od <= new_g + 1e-9 && cov_dominates(ocov, new_covs[primary])
                        return
                    end
                end
                kept_labels = Tuple{Float64, Matrix{Float64}}[]
                for (od, ocov) in labels
                    if new_g <= od + 1e-9 && cov_dominates(new_covs[primary], ocov)
                        continue
                    end
                    push!(kept_labels, (od, ocov))
                end
                push!(kept_labels, (new_g, copy(new_covs[primary])))
                frontier_by_signature[sig_key] = kept_labels

                # `candidate_nodes` is the shared recursion buffer — copy it.
                push!(states, JointState(copy(candidate_nodes), new_covs, new_dists,
                                          new_g, si, new_visited))
                new_si = length(states)
                enqueue!(pq, new_si, (f_exact, prim_unc, support_idle_score(new_dists)))
                return
            end

            agent = move_order[agent_idx]
            curr_node = S.nodes[agent]
            for u in graph.neighbors[curr_node]
                S.visited[agent][u] && continue

                candidate_nodes[agent] = u
                if agent == primary
                    primary_dist_next = S.dists[primary] + graph.distance[curr_node, u]
                    expand_joint_moves(agent_idx + 1, primary_dist_next)
                else
                    support_dist_next = S.dists[agent] + graph.distance[curr_node, u]
                    support_dist_next <= primary_dist && expand_joint_moves(agent_idx + 1, primary_dist)
                end
            end
        end

        expand_joint_moves(1, S.dists[primary])
    end

    # --- Save animation if enabled ---
    if animate_enabled && iter_count > 1 && !animation_saved
        gif(anim_frames, debug_gif_path, fps=20)
        println("  [Constraint A*] Animation saved to $debug_gif_path (iters $(animate_start_iter)-$(min(iter_count, animate_start_iter + animate_limit - 1)))")
    end

    # ── No feasible goal reached ──────────────────────────────────────────────
    println()   # release the status-bar line
    if isempty(best_paths_j)
        println("  [Constraint A*] No feasible solution found")
        return [Int[] for _ in 1:na], zeros(na), Inf, iter_count
    end
    println("  [Constraint A*] Budget exhausted with no seed under threshold; " *
            "handing best incumbent to the optimizer: dist=$(round(best_dists_j[primary], digits=3)), " *
            "unc=$(round(best_unc_j, digits=4)) > $(round(unc_threshold, digits=4))")
    return best_paths_j, best_dists_j, best_unc_j, iter_count
end


# ==========================================================================
# Continuous spline optimizer — barrier-based search in control-point space
# ==========================================================================
# Optimize waypoint positions (x, y) in continuous space to minimize primary
# path length while maintaining uncertainty constraint.
#
# Free variables: (x, y) coordinates of all primary intermediate waypoints and
# all support waypoints after their starts. Primary start/goal are fixed;
# support starts are fixed.
#
# Objective  : minimize primary agent's total path length (Euclidean distances)
# Constraint : joint unc_radius(Σ_goal) ≤ cont_threshold,
#              support length ≤ primary length,
#              and curvature bounds
function optimize_continuous(paths::Vector{Vector{Int}}, graph::LandmarkGraph, landmarks::Vector{Landmark}, num_agents::Int, output_dir::String; cont_threshold::Float64=UNC_RADIUS_THRESHOLD, fig_prefix::String="", straight_seed::Bool=false)
    println("\n=== Continuous Spline Optimization (threshold=$(round(cont_threshold, digits=4))) ===")

    # Initialize continuous waypoints either from discrete paths or, when
    # straight_seed (the straight_cont pipeline), a direct start→goal line.
    all_agent_wpts = Vector{Vector{Tuple{Float64,Float64}}}(undef, length(paths))
    is_primary_mask = Vector{Bool}(undef, length(paths))

    if straight_seed
        n_agents = length(paths)
        sx = graph.landmarks[1].x
        sy = graph.landmarks[1].y
        gx = graph.landmarks[graph.n].x
        gy = graph.landmarks[graph.n].y
        n_seed = max(3, STRAIGHT_CONT_PRIMARY_WPTS)
        # Every agent — supports included — is seeded on the same start→goal line,
        # matching the discrete stage, which already hands supports [1, goal_node].
        # (Supports used to get two coincident points at start, i.e. no path at all.)
        line_wpts = Vector{Tuple{Float64,Float64}}(undef, n_seed)
        for i in 1:n_seed
            t = (i - 1) / (n_seed - 1)
            line_wpts[i] = ((1 - t) * sx + t * gx, (1 - t) * sy + t * gy)
        end
        for ai in 1:n_agents
            all_agent_wpts[ai] = copy(line_wpts)
            is_primary_mask[ai] = (ai == n_agents)
        end
    else
        # Same builder the discrete seed gate uses (seed_spline_clear), so the spline
        # checked at goal-pop time is byte-for-byte the spline refined here.
        all_agent_wpts = seed_control_points(paths, graph)
        for ai in eachindex(paths)
            is_primary_mask[ai] = (ai == length(paths))
        end
    end

    # Count free variables
    num_agents = length(paths)
    free_counts = Int[]
    for ai in 1:num_agents
        n_wpts = length(all_agent_wpts[ai])
        if ai == num_agents
            push!(free_counts, n_wpts <= 2 ? 0 : (n_wpts - 2) * 2)
        else
            push!(free_counts, n_wpts <= 1 ? 0 : (n_wpts - 1) * 2)
        end
    end
    total_free_cont = sum(free_counts)

    if total_free_cont == 0
        println("  (No intermediate waypoints to optimize)")
        # Was `return nothing`, which would have thrown at the callers' 5-way
        # destructuring. Return the seed itself with an explicit status.
        seed_ctrls = [copy(w) for w in all_agent_wpts]
        covs_seed, _, _, _ = evaluate_joint_discrete(seed_ctrls, landmarks, num_agents)
        seed_unc = (isempty(covs_seed) || isempty(covs_seed[end])) ? Inf : unc_radius(covs_seed[end][end])
        seed_len = bspline_path_length(seed_ctrls[end])
        return seed_len, seed_unc, seed_ctrls, covs_seed, seed_ctrls,
               (status = :no_free_vars, min_slack = NaN)
    end

    # Static-obstacle avoidance (MINVO enclosure, chance-constrained). Commit
    # each spline segment to the obstacle face the seed rounded; those committed
    # half-spaces enter the barrier objective below as linear rows over the MINVO
    # hull vertices, and are re-verified after refinement. No-op (empty plans)
    # when there are no obstacles.
    # Built the same way for BOTH seeds. The straight seed skips the discrete
    # filter, so its committed homotopy is whichever side each segment already
    # leans toward — for a segment sitting inside an obstacle that is the nearest
    # face, i.e. the shortest way out. That commitment being arbitrary rather than
    # searched is precisely what the ablation measures; handing it the empty
    # no-obstacle plan instead would let it "pass" by not being asked the question.
    obstacle_plans = build_obstacle_plan(all_agent_wpts, SPLINE_DEGREE)

    # pack/unpack
    function pack_waypoints(wpts_list::Vector{Vector{Tuple{Float64,Float64}}})
        flat = Float64[]
        for ai in 1:num_agents
            wpts = wpts_list[ai]
            if ai == num_agents
                if length(wpts) > 2
                    for i in 2:length(wpts)-1
                        push!(flat, wpts[i][1]); push!(flat, wpts[i][2])
                    end
                end
            else
                if length(wpts) > 1
                    for i in 2:length(wpts)
                        push!(flat, wpts[i][1]); push!(flat, wpts[i][2])
                    end
                end
            end
        end
        return flat
    end

    function unpack_waypoints(flat::Vector{Float64})
        wpts_list = Vector{Vector{Tuple{Float64,Float64}}}(undef, num_agents)
        idx = 1
        for ai in 1:num_agents
            orig_wpts = all_agent_wpts[ai]
            n_wpts = length(orig_wpts)
            wpts = Vector{Tuple{Float64,Float64}}(undef, n_wpts)
            wpts[1] = orig_wpts[1]
            if ai == num_agents
                if n_wpts > 2
                    for i in 2:n_wpts-1
                        x = flat[idx]; y = flat[idx+1]; wpts[i] = (x,y); idx += 2
                    end
                end
                if n_wpts >= 2; wpts[n_wpts] = orig_wpts[n_wpts]; end
            else
                if n_wpts > 1
                    for i in 2:n_wpts
                        x = flat[idx]; y = flat[idx+1]; wpts[i] = (x,y); idx += 2
                    end
                end
            end
            wpts_list[ai] = wpts
        end
        return wpts_list
    end

    function eval_continuous(flat::Vector{Float64})
        ctrl_list = unpack_waypoints(flat)
        sampled_paths = Vector{Vector{Tuple{Float64,Float64}}}(undef, num_agents)
        curvatures = Vector{Vector{Float64}}(undef, num_agents)
        max_curvatures = zeros(Float64, num_agents)
        for ai in 1:num_agents
            sampled_paths[ai], curvatures[ai] = bspline_sample_path(ctrl_list[ai])
            max_curvatures[ai] = isempty(curvatures[ai]) ? 0.0 : maximum(curvatures[ai])
        end
        if any(pt -> !isfinite(pt[1]) || !isfinite(pt[2]), Iterators.flatten(sampled_paths))
            empty_covs = [Matrix{Float64}[] for _ in 1:num_agents]
            support_lens = fill(Inf, max(0, num_agents - 1))
            return Inf, Inf, sampled_paths, empty_covs, curvatures, max_curvatures, ctrl_list, support_lens
        end
        unc_eval_paths = CONT_UNC_USE_WAYPOINTS ? ctrl_list : sampled_paths
        covs_all, _, _, _ = evaluate_joint_discrete(unc_eval_paths, landmarks, num_agents)
        if isempty(covs_all) || isempty(covs_all[end]) || any(!isfinite, covs_all[end][end])
            support_lens = fill(Inf, max(0, num_agents - 1))
            return Inf, Inf, sampled_paths, covs_all, curvatures, max_curvatures, ctrl_list, support_lens
        end
        goal_unc = unc_radius(covs_all[end][end])
        if !isfinite(goal_unc)
            support_lens = fill(Inf, max(0, num_agents - 1))
            return Inf, Inf, sampled_paths, covs_all, curvatures, max_curvatures, ctrl_list, support_lens
        end
        prim_len = bspline_path_length(sampled_paths[end])
        support_lens = [bspline_path_length(sampled_paths[a]) for a in 1:(num_agents - 1)]
        return prim_len, goal_unc, sampled_paths, covs_all, curvatures, max_curvatures, ctrl_list, support_lens
    end

    # ── Constraint slacks: one vector, four classes ────────────────────────────
    # slack > 0 ⇒ satisfied with margin; slack ≤ 0 ⇒ violated. Each class folds in
    # the tolerance its own gate already used (UNC_FEAS_TOL for uncertainty and
    # support length, 1e-9 for curvature, OBSTACLE_SLACK_TOL for obstacles), so
    # `all(≥ 0)` is exactly the feasibility test these five call sites used to
    # spell out inline — no tightening, no loosening.
    # Takes eval_continuous's already-unpacked results rather than a flat vector:
    # eval_continuous is the most expensive call in the optimizer (it runs
    # evaluate_joint_discrete for every agent) and the gates below all have its
    # output in hand already.
    function slacks_from(plen::Float64, unc::Float64, curvatures, ctrl_list, covs_all, support_lens)
        s = Float64[]
        push!(s, cont_threshold + UNC_FEAS_TOL - unc)
        for curvset in curvatures, κ in curvset
            isfinite(κ) && push!(s, MAX_CURVATURE + 1e-9 - κ)
        end
        for sl in support_lens
            # eval_continuous's failure paths set plen and every support length to
            # Inf together; Inf−Inf would poison min_slack with NaN, and the
            # uncertainty slack is already −Inf on exactly those paths.
            push!(s, (isfinite(plen) && isfinite(sl)) ? plen + UNC_FEAS_TOL - sl : -Inf)
        end
        obstacle_slacks!(s, obstacle_plans, ctrl_list, covs_all)
        return s
    end

    min_slack(s::Vector{Float64})  = isempty(s) ? Inf : minimum(s)
    is_feasible(s::Vector{Float64}) = min_slack(s) >= 0.0
    is_interior(s::Vector{Float64}, ε::Float64) = min_slack(s) >= ε

    # Initialize optimizer state
    flat = pack_waypoints(all_agent_wpts)
    init_len, init_unc, init_wpts, init_covs, init_curvs, init_max_curv, init_ctrls, init_support_lens = eval_continuous(flat)
    println("  Initial (discrete→spline): prim_len=$(round(init_len,digits=3)), unc=$(round(init_unc,digits=4))")

    init_slacks = slacks_from(init_len, init_unc, init_curvs, init_ctrls, init_covs, init_support_lens)

    # Feasibility is an INVARIANT once attained, not a precondition on the seed.
    # `reached_feasible` latches: while false the line search below accepts any
    # objective-improving step so the barrier's tangent can pull an infeasible seed
    # in; the moment a feasible iterate is accepted it flips true and the hard
    # feasible-only gate arms permanently. Still ONE phase and one objective — the
    # tangent-extended barrier already dominates path length ~4 orders of magnitude
    # while violated, so it recovers first and shortens after, on its own.
    # When A* returned a seed under the threshold this is true from the start and the
    # loop below is bit-for-bit the feasible-only optimizer it was; when it returned
    # its best incumbent instead, recovery runs first, as for straight_cont.
    reached_feasible = is_feasible(init_slacks)
    if !reached_feasible
        @warn "Seed is infeasible (min_slack=$(min_slack(init_slacks)), ε=$(CONT_RESTORE_MARGIN)); the barrier will try to recover it before shortening. Expected for the straight_cont ablation, and for a discrete seed that A* returned as its best incumbent after exhausting its budget."
    elseif !is_interior(init_slacks, CONT_RESTORE_MARGIN)
        @warn "Seed is feasible but not strictly interior (min_slack=$(min_slack(init_slacks)), ε=$(CONT_RESTORE_MARGIN)); the barrier is past its knee and steps will be small."
    end

    # Barrier optimizer (simplified reuse of in-file logic but using cont_threshold)
    adam_m = zeros(total_free_cont); adam_v = zeros(total_free_cont)
    # The incumbent starts as the SEED whenever the seed is feasible, so what
    # ships can never be longer than what the search handed over. Without this,
    # a feasible seed entered the race with no incumbent at all: the line search
    # accepts any barrier-objective descent, and at stage-1 μ a step that
    # LENGTHENS the path while moving interior is such a descent — so a run that
    # stalled after those steps shipped the longer iterate labeled `optimized`.
    # seed_feasible is remembered so the status below can tell "the seed won"
    # (:seed_only) from "an accepted iterate beat it" (:optimized).
    seed_feasible = reached_feasible
    best_feasible_flat = seed_feasible ? copy(flat) : nothing
    best_feasible_len  = seed_feasible ? init_len : Inf
    best_feasible_unc  = seed_feasible ? init_unc : Inf
    # Incumbent for the recovery regime, mirroring best_feasible_flat: the LEAST
    # violating iterate seen while still outside. Seeded with the seed itself, so a
    # failed recovery can never ship something further out than what it started
    # from — the last iterate is just wherever the barrier happened to stop, and on
    # a flat uncertainty landscape (`single`: landmark 250 m off-corridor, detection
    # ~1e-26) the other rows push the path around and it drifts.
    best_recover_flat = copy(flat); best_recover_slack = min_slack(init_slacks)
    prev_len = init_len

    # ── Objective — length under a tangent-extended log barrier ───────────────
    # Below the knee δ the log is continued by its own tangent at δ:
    #     B(s) = −μ·log(s)                      s > δ
    #            −μ·[ log δ + (s − δ)/δ ]       s ≤ δ
    # C¹ at the knee (value AND slope match), finite for every s including
    # negative, and carrying a constant restoring gradient −μ/δ outside. That
    # finiteness is not cosmetic: the gradient here is a forward difference, so an
    # objective returning Inf on an infeasible probe would give grad = Inf and
    # Adam's m̂/(√v̂+ε) = Inf/Inf = NaN, permanently. The quadratic this replaces
    # was C⁰ only — it dropped ~μ·|log 1e-9| ≈ 414 at μ=20 crossing the knee, and
    # its gradient VANISHED as s → δ⁻, so the restoring force was weakest exactly
    # where the iterate was about to breach. That is the mechanism behind the old
    # "a small violation is cheaper than a thin slack" pathology.
    # Feasibility itself is NOT enforced here — the tangent is finite, so shortening
    # the path can still pay for a breach. The line search rejects infeasible trials
    # instead, which is what makes feasibility an invariant.
    δ_barrier = CONT_RESTORE_MARGIN
    log_δ = log(δ_barrier)
    @inline barrier_term(s::Float64, μ::Float64) =
        s > δ_barrier ? -μ * log(s) : -μ * (log_δ + (s - δ_barrier) / δ_barrier)

    # Returns (objective, feasible) from ONE eval_continuous — the line search needs
    # both and eval_continuous is the expensive call.
    function spline_barrier_objective(flat::Vector{Float64}, barrier_mu::Float64)
        prim_len, goal_unc, _, covs_all, curvatures, _, ctrl_list, support_lens = eval_continuous(flat)
        s = slacks_from(prim_len, goal_unc, curvatures, ctrl_list, covs_all, support_lens)
        obj = prim_len
        for x in s
            obj += barrier_term(x, barrier_mu)
        end
        return obj, is_feasible(s)
    end

    stage_iters = max(25, cld(CONT_OPT_ITERS, CONT_BARRIER_STAGES))
    barrier_mu = CONT_BARRIER_START; total_iter = 0
    for stage in 1:CONT_BARRIER_STAGES
        println("  Barrier stage $(stage)/$(CONT_BARRIER_STAGES): μ=$(round(barrier_mu,digits=4))")
        for iter in 1:stage_iters
            total_iter += 1
            obj0, _ = spline_barrier_objective(flat, barrier_mu)
            # Probe width tracks the regime, not a separate phase. Reverts to
            # CONT_OPT_H the moment feasibility is reached — which is iteration 1
            # for a gated discrete seed, so that pipeline never sees
            # CONT_RECOVER_H at all. The two are equal at today's defaults; the
            # split is kept because the regimes are genuinely different (see
            # config/hexspline_cl.yaml for why the recovery probe was widened to
            # 1 m and why holding μ made that unnecessary).
            h_fd = reached_feasible ? CONT_OPT_H : CONT_RECOVER_H
            grad = zeros(total_free_cont)
            for k in 1:total_free_cont
                flat[k] += h_fd; obj1, _ = spline_barrier_objective(flat, barrier_mu); grad[k] = (obj1 - obj0) / h_fd; flat[k] -= h_fd
            end
            b1t = CONT_ADAM_B1^total_iter; b2t = CONT_ADAM_B2^total_iter
            step = zeros(total_free_cont)
            for k in 1:total_free_cont
                g = grad[k]
                adam_m[k] = CONT_ADAM_B1 * adam_m[k] + (1 - CONT_ADAM_B1) * g
                adam_v[k] = CONT_ADAM_B2 * adam_v[k] + (1 - CONT_ADAM_B2) * g^2
                m̂ = adam_m[k] / (1 - b1t); v̂ = adam_v[k] / (1 - b2t)
                step[k] = CONT_OPT_LR * m̂ / (sqrt(v̂) + CONT_ADAM_EPS)
            end
            # Rejecting infeasible trials here is what makes feasibility an
            # INVARIANT rather than a force: once feasible, no step that leaves the
            # feasible set is ever accepted, so every later iterate is feasible by
            # construction. Before feasibility is first reached the gate is off —
            # otherwise an infeasible seed rejects every trial (they are all still
            # infeasible), backtracks 12 times and stalls on iteration 1, which is
            # exactly what used to make an ungated seed unrecoverable. Objective
            # descent is still required in both regimes.
            accept(ok::Bool, obj::Float64) =
                (ok || !reached_feasible) && isfinite(obj) && obj <= obj0 - CONT_MIN_IMPROVEMENT
            trial = flat .- step; trial_obj, trial_ok = spline_barrier_objective(trial, barrier_mu); backtracks = 0
            while !accept(trial_ok, trial_obj) && backtracks < 12
                step .*= CONT_LINESEARCH_SHRINK; trial = flat .- step; trial_obj, trial_ok = spline_barrier_objective(trial, barrier_mu); backtracks += 1
            end
            if !accept(trial_ok, trial_obj)
                println("  [Spline barrier] line search stalled; stopping at stage $stage")
                break
            end
            flat .= trial
            len2, unc2, wpts2, covs2, curv2, max_curv2, ctrls2, support_lens2 = eval_continuous(flat)
            slacks2 = slacks_from(len2, unc2, curv2, ctrls2, covs2, support_lens2)
            feasible2 = is_feasible(slacks2)
            # The invariant, checked rather than assumed: once feasibility has been
            # reached it is never given back. Nearly free — eval_continuous above is
            # the expensive part.
            @assert feasible2 || !reached_feasible "Barrier accepted an infeasible iterate at stage $stage iter $total_iter"
            if feasible2 && !reached_feasible
                reached_feasible = true
                println("    → Recovered into the feasible set at iter $total_iter (min_slack=$(round(min_slack(slacks2),digits=6))); feasible-only gate now armed")
            end
            # Only one incumbent is needed now. The old code also tracked a
            # length-only `best_flat` with NO feasibility test and returned it when
            # no feasible iterate was ever found — i.e. it shipped the shortest,
            # most-violating point. That fallback stays gone: the incumbent is
            # feasible-only, which under an infeasible seed also excludes the
            # pre-recovery iterates.
            if feasible2 && len2 < best_feasible_len
                best_feasible_len = len2; best_feasible_unc = unc2; best_feasible_flat = copy(flat)
            end
            if !feasible2 && min_slack(slacks2) > best_recover_slack
                best_recover_slack = min_slack(slacks2); best_recover_flat = copy(flat)
            end
            # Length-stationarity means "converged" only while length is what is
            # being minimized. During recovery the objective is dominated by the
            # barrier tangent and length can sit still for several iterations while
            # violation drops — breaking there would abort the recovery.
            if reached_feasible && abs(len2 - prev_len) < CONT_CONV_TOL
                println("  → Converged at iter $total_iter (Δlen=$(round(abs(len2-prev_len),digits=6)))")
                break
            end
            prev_len = len2
        end
        # μ decays only from INSIDE the feasible set — that is what the decay
        # ladder is for: μ→0 walks an interior point onto the constrained optimum.
        # Outside, μ is the entire restoring force: below the knee the tangent's
        # gradient is μ/δ, so at δ=1e-3 the ladder's last stages (μ≈2e-4) push with
        # 0.2 against a path-length gradient of O(1) — the barrier switches off
        # mid-recovery and the optimizer starts BUYING violation to shorten. Seen
        # directly on the sweep's s002: unc slack drifting −0.534 → −0.572 over the
        # last 600 iterations while length fell 623.5 → 622.0.
        # No-op for a gated discrete seed, which is feasible at iteration 0.
        reached_feasible && (barrier_mu *= CONT_BARRIER_DECAY)
    end

    # best_feasible_flat is unset when no FEASIBLE step was ever accepted: either the
    # first line search stalled outright, or the seed was infeasible and recovery
    # never made it in. best_recover_flat covers both — it is the seed until some
    # iterate violates by less, so what ships is the least-violating point the run
    # ever saw. Never an "optimized but violating" point passed off as a solution;
    # the status says which.
    selected_flat = isnothing(best_feasible_flat) ? best_recover_flat : best_feasible_flat
    opt_len, opt_unc, opt_wpts, opt_covs, opt_curvs, opt_max_curv, opt_ctrls, opt_support_lens = eval_continuous(selected_flat)

    # Status of what is actually being returned, so a caller (and results.yaml) can
    # tell a certified solution from a fallback instead of having to trust that one
    # was found.
    #   :optimized       — a feasible iterate beat the seed (or recovered an
    #                      infeasible seed into the feasible set)
    #   :seed_only       — the seed was already feasible and no accepted iterate
    #                      beat its length; this is the seed spline, unshortened
    #   :recovery_failed — the seed was infeasible and the barrier never reached the
    #                      feasible set. THIS RESULT VIOLATES CONSTRAINTS (min_slack
    #                      < 0) and is the ablation's failure case — count these to
    #                      get the failure rate without the discrete search.
    final_slacks = slacks_from(opt_len, opt_unc, opt_curvs, opt_ctrls, opt_covs, opt_support_lens)
    refinement_status = isnothing(best_feasible_flat)                    ? :recovery_failed :
                        (seed_feasible && best_feasible_len >= init_len) ? :seed_only :
                                                                           :optimized
    refinement_info = (status = refinement_status, min_slack = min_slack(final_slacks))
    println("  Final: prim_len=$(round(opt_len,digits=3)), unc=$(round(opt_unc,digits=4)), threshold=$(round(cont_threshold,digits=4)), status=$(refinement_status), min_slack=$(round(min_slack(final_slacks),digits=6))")

    # Post-refinement re-verification (spec E), two views of the FINAL solution:
    #  - real_viols: does the refined MEAN curve actually enter any obstacle?
    #    (sampled-curve geometric containment — the physically meaningful check).
    #  - hull_viols: the conservative committed-face MINVO-hull certificate the
    #    barrier/gate use. A hull breach with a clear curve is not a collision:
    #    near the clamped spline ends the boundary segment's large hull drapes
    #    over a nearby obstacle the curve rounds. ponytail: enforcement is soft
    #    (barrier/penalty, no QP/projection), so a genuine breach (real_viols
    #    non-empty) is possible under tight-corner pressure — reported, not
    #    silently shipped; go hard/projected only if guaranteed clearance is needed.
    real_viols = verify_curve_collisions(opt_ctrls)
    hull_viols = verify_obstacle_clearance(obstacle_plans, opt_ctrls, opt_covs)
    if !all(isempty, obstacle_plans)
        if isempty(real_viols)
            note = isempty(hull_viols) ? "" :
                   "  ($(length(hull_viols)) hull row(s) conservatively flagged; mean curve rounds them)"
            println("  Obstacle re-verification: mean path clears all obstacles ✓$(note)")
        else
            println("  ⚠ Obstacle re-verification: $(length(real_viols)) real obstacle collision(s):")
            for (a, mi, depth) in real_viols
                println("      agent $a obstacle $mi  depth=$(round(depth, digits=4)) m")
            end
        end
    end

    # Evaluate initial discrete solution (waypoints from A* path).
    # Re-pack all_agent_wpts rather than reusing `flat`: `flat` has been mutated
    # in place by the barrier loop, so it holds the last iterate, not the seed.
    init_len, init_unc, _, _, _, _, _, _ = eval_continuous(pack_waypoints(all_agent_wpts))

    # Plot 1: Discrete solution (waypoints connected linearly)
    plt1 = make_base_plot(landmarks, graph)
    for ai in 1:num_agents
        wpts = all_agent_wpts[ai]; xs = [w[1] for w in wpts]; ys = [w[2] for w in wpts]
        is_prim = is_primary_mask[ai]; clr = is_prim ? :blue : get(agent_colors, ai, :gray)
        if !is_prim; ys = ys .+ (SUPPORT_PLOT_OFFSET_M * ai); end
        plot!(plt1, xs, ys, label=false, color=clr, linewidth=is_prim ? 2.2 : 1.3, linestyle=:dash)
    end
    annotate!(plt1, 950.0, 400.0, text("Discrete\nLen: $(round(init_len, digits=2))\nUnc: $(round(init_unc, digits=4))", 10, :left), annotationhalign=:left)
    # (Do not save individual discrete figure; only save combined comparison)

    # Plot 2: Continuous solution (fully optimized B-spline)
    plt2 = make_base_plot(landmarks, graph)
    for ai in 1:num_agents
        wpts = opt_wpts[ai]; xs = [w[1] for w in wpts]; ys = [w[2] for w in wpts]
        is_prim = is_primary_mask[ai]; clr = is_prim ? :blue : get(agent_colors, ai, :gray)
        if !is_prim; ys = ys .+ (SUPPORT_PLOT_OFFSET_M * ai); end
        plot!(plt2, xs, ys, label=false, color=clr, linewidth=is_prim ? 2.2 : 1.3)
    end
    annotate!(plt2, 950.0, 400.0, text("Continuous (Refined)\nLen: $(round(opt_len, digits=2))\nUnc: $(round(opt_unc, digits=4))", 10, :left), annotationhalign=:left)
    # (Do not save individual continuous figure; only save combined comparison)

    # Uncertainty-vs-distance profile: discrete (dashed) and continuous (solid) per agent
    d_covs, d_arcs, d_comm, d_landmark_events = evaluate_joint_discrete(all_agent_wpts, landmarks, num_agents)
    cont_eval_paths = CONT_UNC_USE_WAYPOINTS ? opt_ctrls : opt_wpts
    c_covs, c_arcs, c_comm, c_landmark_events = evaluate_joint_discrete(cont_eval_paths, landmarks, num_agents)

    # The event CSVs are meant to describe the FINAL solution, so hand the
    # continuous-eval events back to the caller. The discrete d_* events stay
    # local: they only feed the left panel of the comparison figure.
    refinement_info = merge(refinement_info, (comm_events = c_comm, landmark_events = c_landmark_events))

    # Combined side-by-side figure: discrete (left) vs continuous (right)
    save_path_figures_pair(plt1, plt2,
        fig_path(output_dir, string(fig_prefix, "fig_compare_discrete_continuous", (fig_prefix == "" ? "" : "_"), string(round(opt_len,digits=2)), ".png")),
        d_comm, c_comm, d_landmark_events, c_landmark_events)
    println("Fig combined (", fig_prefix, ") saved: Discrete Len=$(round(init_len,digits=2)) → Continuous Len=$(round(opt_len,digits=2))")
    unc_profile_fname = fig_path(output_dir, string(fig_prefix == "" ? "main" : fig_prefix, "_unc_profile.png"))
    save_unc_figures(
        [("discrete", d_arcs, d_covs, :dash, 1.2, 1.2),
         ("continuous", c_arcs, c_covs, :solid, 2.0, 1.3)],
        num_agents, unc_profile_fname; threshold=cont_threshold,
        title=string(fig_prefix == "" ? "main" : fig_prefix, " — uncertainty profile (", LANDMARK_SCENARIO, ")"))
    println("Fig uncertainty profile (", fig_prefix == "" ? "main" : fig_prefix, ") saved: ", unc_profile_fname)

    return opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls, refinement_info
end

# ==========================================================================
# Entry point: joint discrete A* seed search -> continuous B-spline refinement
# ==========================================================================
# Returns a Vector of solutions (NamedTuples: id, ctrls, primary_length,
# primary_unc) -- one "main" solution, the first feasible seed the A* finds.
# (Still a Vector: the interface in notes/PLAN.md is multi-solution.)
# Shared engine for the hexspline_cl family of pipelines. `seed_mode` picks the
# discrete seed source (:discrete = joint A*, :straight = direct start→goal line,
# no search); `run_continuous` toggles the B-spline refinement stage. The three
# public entry points below (plan_hexspline_cl / plan_straight_cont /
# plan_discrete_only) are thin wrappers selecting these two knobs.
function _plan_hexspline(scenario, graph::LandmarkGraph, output_dir::String;
                         seed_mode::Symbol, run_continuous::Bool)
    landmarks = scenario.landmarks

    # Conditional multiplicative epsilon bound summary.
    # Let L* be the unknown optimal continuous feasible primary distance.
    # Use L_lb = straight-line(start, goal) as a conservative lower bound on L*.
    δ_bound = HEX_RADIUS_M
    L_lb = max(1e-9, hypot(scenario.goal[1] - scenario.start[1], scenario.goal[2] - scenario.start[2]))
    ε_sample = (2 * δ_bound / max(1e-9, 2 * HEX_RADIUS_M)) + (2 * δ_bound / L_lb)
    w_astar = 1.0 + PRIMARY_EPSILON
    ε_total_conditional = w_astar * (1.0 + ε_sample) - 1.0
    println("Conditional ε-bound: ε_sample=$(round(ε_sample, digits=4)), w=$(round(w_astar, digits=4)), ε_total=$(round(ε_total_conditional, digits=4))")
    println("  Bound form (conditional): L_returned ≤ (1+ε_total) L* with assumptions documented in comments.")

    goal_node = graph.n

    # ── Multi-agent discrete seed search ────────────────────────────────────────
    # Joint A* searches the full joint state space of all agents simultaneously,
    # propagating covariances and applying inter-agent Kalman communication at each step.

    println("\n── MULTI-AGENT PLANNING (Synchronized Kalman) ──")
    disc_unc_threshold = effective_discrete_unc_threshold(UNC_RADIUS_THRESHOLD)
    if ENABLE_RELAXED_DISCRETE_FOR_CONTINUOUS && disc_unc_threshold > UNC_RADIUS_THRESHOLD + UNC_FEAS_TOL
        println("  Discrete acceptance relaxed for continuous handoff: threshold=$(round(UNC_RADIUS_THRESHOLD, digits=4)) -> $(round(disc_unc_threshold, digits=4))")
    end

    discrete_iter = 0

    function run_discrete_seed_search(search_unc_threshold::Float64)
        println("\n  Running joint discrete A* (threshold=$(round(search_unc_threshold, digits=4)))... (max_iters=$(ASTAR_ITERATION_LIMIT))")
        println("  Stops on the first feasible candidate under the threshold.")
        ppaths, pdists, punc, disc_iter = joint_astar(graph, landmarks, search_unc_threshold, NUM_AGENTS)
        # Recorded BEFORE the failure return, so results.yaml reports the search
        # effort either way. A* exits identically whether the threshold is
        # provably unreachable or the iteration budget simply ran out, so
        # astar_iterations == ASTAR_ITERATION_LIMIT is the only thing that tells
        # a budget-limited failure from an infeasible one -- and reporting 0 on
        # every failure made that indistinguishable.
        discrete_iter = disc_iter
        if length(ppaths) != NUM_AGENTS || any(isempty, ppaths)
            println("  ✗ No feasible path found by A*")
            return [Int[] for _ in 1:(NUM_AGENTS - 1)], Int[], Inf
        end

        support_paths = pad_support_paths_to_primary(ppaths[1:NUM_AGENTS-1], ppaths[NUM_AGENTS])
        return support_paths, ppaths[NUM_AGENTS], pdists[NUM_AGENTS]
    end


    if seed_mode == :straight
        println("\n  seed_mode=:straight — skipping discrete search and using direct start→goal seed")
        primary_path = [1, goal_node]
        primary_dist = graph.distance[1, goal_node]
        support_paths = [Int[1, goal_node] for _ in 1:(NUM_AGENTS - 1)]
    elseif seed_mode == :discrete
        support_paths, primary_path, primary_dist = run_discrete_seed_search(disc_unc_threshold)
    else
        error("Unsupported seed_mode=$(seed_mode). Use :discrete or :straight")
    end

    if isempty(primary_path)
        paths = Vector{Vector{Int}}()
        dists = Float64[]
        uncs = Float64[]
        final_global_cov = Matrix{Float64}[]
        primary_goal_unc = Inf
    else
        # All agents' paths assembled with support agents first, primary last
        paths = vcat(support_paths, [primary_path])

        # Evaluate joint covariance WITH synchronized Kalman fusion
        # This ensures all agents benefit from inter-agent communication at fixed intervals
        final_global_cov, dists = evaluate_full_paths(paths, graph, landmarks, NUM_AGENTS)
        uncs = [unc_radius(final_global_cov[a]) for a in 1:NUM_AGENTS]
        primary_goal_unc = uncs[end]

        println("\n✓ Multi-agent solution with synchronized Kalman fusion:")
        for a in 1:(NUM_AGENTS-1)
            final_node = isempty(paths[a]) ? 0 : paths[a][end]
            println("  Support agent $a: node=$(final_node), dist=$(round(dists[a], digits=1))m, goal_unc=$(round(uncs[a], digits=4))m")
        end
        primary_final_node = isempty(primary_path) ? 0 : primary_path[end]
        println("  Primary agent: node=$(primary_final_node), dist=$(round(dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m")
        println("  ├─ Communication: every $(COMM_INTERVAL)m, sigmoid taper range=$(COMM_RANGE)m width=$(COMM_WIDTH)m")
        println("  └─ All agents' uncertainties benefit from synchronized Kalman fusion at checkpoints")

        # A seed above the threshold is now an expected handoff, not a failure: A*
        # returns its best incumbent when the budget runs out and the barrier below
        # attempts recovery, exactly as it already does for the straight_cont seed.
        # Feasibility is decided once, on the REFINED spline (refinement_status /
        # min_slack), which is the only stage whose numbers are reported anyway.
        if seed_mode == :discrete && unc_exceeds_threshold(primary_goal_unc, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL)
            println("\n  Seed is above threshold (goal_unc=$(round(primary_goal_unc, digits=4)) > $(round(UNC_RADIUS_THRESHOLD, digits=4))); the continuous barrier will attempt recovery.")
        end
    end

    shortest_path = primary_path
    shortest_dist = primary_dist
    converged_iter = discrete_iter

    println("\n--- Final Results ---")
    if isempty(paths)
        println("IMPOSSIBLE: uncertainty threshold cannot be met. No continuous optimization will be run.")
    else
        # Report all agents with their uncertainties and Kalman filtering status
        for (i, path) in enumerate(paths[1:end-1])
            unc_status = ""
            if !isempty(path)
                unc_threshold_ok = unc_within_threshold(uncs[i], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓" : "✗"
                unc_status = "  goal_unc=$(round(uncs[i], digits=4))  $unc_threshold_ok"
            end
            isempty(path) ? println("Support agent $i : no path found") :
                            println("Support agent $i : ", path, "  dist=", round(dists[i], digits=3), unc_status, "  (Kalman-fused)")
        end
        threshold_status = unc_within_threshold(uncs[end], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓ met" : "✗ not met"
        println("Primary agent : ", paths[end],
                "  dist=",     round(dists[end], digits=3),
                "  goal_unc=", round(uncs[end],  digits=4),
                "  threshold=", UNC_RADIUS_THRESHOLD, " ", threshold_status, "  (Kalman-fused)")

        # Summary of Kalman filtering across all agents
        println("\n=== Kalman Filtering Summary ===")
        println("All $(length(paths)) agents performing synchronized Kalman filtering:")
        println("  • Dead-reckoning propagation: DIR=$(DIR_UNCERTAINTY_PER_METER)/m, PERP=$(PERP_UNCERTAINTY_PER_METER)/m")
        println("  • Landmark fusion: Information filter (Joseph form) at every waypoint")
        println("  • SYNCHRONIZED inter-agent communication: every $(COMM_INTERVAL)m of travel")
        println("  • Tapered sigmoid weighting: range=$(COMM_RANGE)m, width=$(COMM_WIDTH)m (soft plateau + rolloff)")
        println("  • Bidirectional Kalman fusion: all agent pairs within range at each checkpoint")
        println("  • Measurement model: bearing-angle sensor with noise=$(LANDMARK_SENSOR_NOISE), ratio=$(BEARING_NOISE_RATIO)")
        println("  └─ Support agents: goal_unc=$(round(uncs[1], digits=4))m (improved by inter-agent fusion)")
        println("  └─ Primary agent: goal_unc=$(round(uncs[end], digits=4))m (direct path with landmark updates)")
    end
    println("Converged at iteration : ", converged_iter)

    have_paths = !isempty(paths) && all(length.(paths) .> 0)

    # ==========================================================================
    # Figure 1 - discrete graph solution
    # ==========================================================================
    let
        plt1 = make_base_plot(landmarks, graph)
        comm_events_fig1 = Tuple{Float64,Int,Int,Float64,Tuple{Float64,Float64},Tuple{Float64,Float64}}[]
        landmark_events_fig1 = Tuple{Float64,Int,Int,Float64,Tuple{Float64,Float64},Tuple{Float64,Float64}}[]
        if !isempty(paths)
            println("Figure path-role mapping: supports = paths[1:$(max(length(paths)-1,0))], primary = paths[$(length(paths))]")
            # Plot waypoints and paths
            for (vi, path) in enumerate(paths)
                if !isempty(path)
                    px_  = [graph.landmarks[j].x for j in path]
                    py_  = [graph.landmarks[j].y for j in path]
                    is_primary = (vi == length(paths))
                    clr  = is_primary ? :blue : get(agent_colors, vi, :gray)
                    lbl  = is_primary ? "Primary (path[$vi])" : "Support $vi (path[$vi])"
                    if vi != length(paths)
                        py_ = py_ .+ (SUPPORT_PLOT_OFFSET_M * vi)
                        lbl *= " (offset)"
                    end
                    # Vary linestyle by support agent index: dash, dashdot, etc.
                    ls   = is_primary ? :solid : (vi == 1 ? :dash : (vi == 2 ? :dashdot : :dot))
                    lw   = is_primary ? 2.4 : 1.3
                    plot!(plt1, px_, py_, label=lbl, color=clr,
                          linewidth=lw,
                          linestyle=ls)
                    scatter!(plt1, px_, py_, label=false, color=clr, markersize=3, markerstrokewidth=0)
                end
            end

            # Evaluate using discrete waypoint evaluation
            agent_positions_fig1 = Vector{Vector{Tuple{Float64,Float64}}}(undef, length(paths))
            for (ai, path) in enumerate(paths)
                if isempty(path)
                    agent_positions_fig1[ai] = []
                else
                    agent_positions_fig1[ai] = [(graph.landmarks[i].x, graph.landmarks[i].y) for i in path]
                end
            end

            covs_all_fig1, arcs_all_fig1, comm_events_fig1, landmark_events_fig1 = evaluate_joint_discrete(agent_positions_fig1, landmarks, length(paths); debug_goal_pos=scenario.goal)

            # Primary is last agent
            prim_covs_fig1 = covs_all_fig1[end]
            prim_arcs_fig1 = arcs_all_fig1[end]
            astar_prim_len_fig1 = prim_arcs_fig1[end]
            astar_goal_unc_fig1 = unc_radius(prim_covs_fig1[end])

            println("Discrete evaluation: prim_len=$(round(astar_prim_len_fig1, digits=3)), unc=$(round(astar_goal_unc_fig1, digits=4))")

            # Draw primary covariance ellipses
            px_fig1 = [graph.landmarks[j].x for j in paths[end]]
            py_fig1 = [graph.landmarks[j].y for j in paths[end]]
            for k in 1:length(prim_covs_fig1)
                draw_covariance_ellipse!(plt1, px_fig1[k], py_fig1[k], prim_covs_fig1[k];
                                          nstd=2, color=:blue, alpha=0.10)
            end
            title!(plt1,"Fig 1: Discrete [len=$(round(astar_prim_len_fig1,digits=2)), unc=$(round(astar_goal_unc_fig1,digits=3))]")
        else
            title!(plt1,"Fig 1: No feasible path")
        end
        xlabel!(plt1,"x (m)"); ylabel!(plt1,"y (m)")
        save_path_figures(plt1, fig_path(output_dir, "fig1_joint_discrete_astar.png"),
                           comm_events_fig1, landmark_events_fig1)
        println("Fig 1 saved.")
        # Only write the discrete events when nothing downstream will: both
        # pipelines ship a spline and overwrite these CSVs with its events
        # (continuous refinement below, or the discrete_only branch) -- writing
        # the polyline's events here would describe a path we don't ship.
        if !have_paths
            if TRACK_COMM_EVENTS
                write_comm_csv(csv_path(output_dir, "comm_events.csv"), comm_events_fig1)
            end
            if TRACK_LANDMARK_EVENTS
                write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), landmark_events_fig1)
            end
        end
    end


    main_cont_uncs = Float64[]
    main_cont_len  = NaN
    main_refine    = nothing
    solutions = NamedTuple[]

    if run_continuous && have_paths  # Continuous optimization only if all agents have paths
        opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls, main_refine = optimize_continuous(paths, graph, landmarks, NUM_AGENTS, output_dir; cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main", straight_seed=(seed_mode == :straight))
        write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
        # Events from the refined spline, matching main_ctrls.csv (see the fig1
        # block above, which defers to this write).
        if TRACK_COMM_EVENTS
            write_comm_csv(csv_path(output_dir, "comm_events.csv"), main_refine.comm_events)
        end
        if TRACK_LANDMARK_EVENTS
            write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), main_refine.landmark_events)
        end
        main_cont_len = opt_len
        if !isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:NUM_AGENTS)
            main_cont_uncs = [unc_radius(opt_covs[a][end]) for a in 1:NUM_AGENTS]
        end
        push!(solutions, (id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc))
    elseif !run_continuous && have_paths
        # discrete_only pipeline: the deliverable is the B-SPLINE through the A*
        # seed's control points, not the node polyline. The spline is what the
        # agents actually fly (it cuts the hex corners); the polyline is only the
        # search's abstraction of it. Built by seed_control_points -- the same map
        # optimize_continuous seeds from -- so discrete_only ships exactly
        # hexspline_cl's seed trajectory, just unrefined.
        disc_ctrls = seed_control_points(paths, graph)
        write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), disc_ctrls)

        # Score the spline, not the polyline: length off the sampled curve, and
        # uncertainty through the same evaluation the continuous stage reports
        # (CONT_UNC_USE_WAYPOINTS, mirroring cont_eval_paths in optimize_continuous).
        disc_wpts = [first(bspline_sample_path(c)) for c in disc_ctrls]
        disc_len  = bspline_path_length(disc_wpts[end])
        disc_covs, disc_arcs, disc_comm, disc_lm = evaluate_joint_discrete(
            CONT_UNC_USE_WAYPOINTS ? disc_ctrls : disc_wpts, landmarks, NUM_AGENTS)
        disc_unc = unc_radius(disc_covs[end][end])
        println("Discrete-only spline: prim_len=$(round(disc_len, digits=3)), unc=$(round(disc_unc, digits=4))")

        # Events of the shipped spline (fig1's are the A* polyline's -- see the
        # write it defers to there).
        if TRACK_COMM_EVENTS
            write_comm_csv(csv_path(output_dir, "comm_events.csv"), disc_comm)
        end
        if TRACK_LANDMARK_EVENTS
            write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), disc_lm)
        end

        # Figure of what we ship. fig1 stays the A* diagnostic (shared with the
        # other two pipelines), so the spline gets its own.
        plt_sp = make_base_plot(landmarks, graph)
        for ai in 1:NUM_AGENTS
            is_prim = (ai == NUM_AGENTS)
            xs = [w[1] for w in disc_wpts[ai]]
            ys = [w[2] for w in disc_wpts[ai]]
            is_prim || (ys = ys .+ (SUPPORT_PLOT_OFFSET_M * ai))
            plot!(plt_sp, xs, ys, label=(is_prim ? "primary" : "support $ai (offset)"),
                  color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                  linewidth=(is_prim ? 2.4 : 1.3), linestyle=(is_prim ? :solid : :dash))
        end
        title!(plt_sp, "Discrete-only spline [len=$(round(disc_len,digits=2)), unc=$(round(disc_unc,digits=3))]")
        xlabel!(plt_sp, "x (m)"); ylabel!(plt_sp, "y (m)")
        save_path_figures(plt_sp, fig_path(output_dir, "fig_discrete_spline.png"), disc_comm, disc_lm)
        save_unc_figures([("", disc_arcs, disc_covs, :solid, 2.0, 1.3)], NUM_AGENTS,
                         fig_path(output_dir, "main_unc_profile.png");
                         threshold=UNC_RADIUS_THRESHOLD,
                         title="discrete_only — uncertainty profile ($(LANDMARK_SCENARIO))")

        # results.yaml's continuous_* fields = the shipped trajectory's metrics.
        # Non-null here now: the path IS a continuous curve, only unoptimized
        # (refinement_status stays absent, which is what marks it unrefined).
        main_cont_len  = disc_len
        main_cont_uncs = [unc_radius(disc_covs[a][end]) for a in 1:NUM_AGENTS]
        push!(solutions, (id=0, ctrls=disc_ctrls, primary_length=disc_len, primary_unc=disc_unc))
    end

    # ==========================================================================
    # Write results.yaml
    # ==========================================================================
    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        fmtlist(v) = isempty(v) ? "[]" : "[" * join(fmt4.(v), ", ") * "]"
        # FULL precision, for the values something MACHINE-READS and compares against
        # a tolerance. fmt4's worst-case error is 5e-5, fifty times UNC_FEAS_TOL
        # (1e-6) — so a run the optimizer certified feasible at min_slack=+9e-6 was
        # rounded up past its own threshold and then re-judged "unc_violated" by
        # run_constraint_sweep.jl's classify(). It also corrupted the ladder itself:
        # every threshold is pct/100 · U_ref, and U_ref is read back out of THIS
        # field in the reference run. Rounding is for the human-facing rows below.
        fmtx(x) = isfinite(x) ? repr(x) : "null"

        write(io, "main:\n")
        # The two-key contract every planner honours, batch harnesses read, and
        # nothing else in results.yaml is required to satisfy. Named for the
        # deliverable rather than the stage that produced it, so a planner with
        # no continuous phase (or one that doesn't exist yet) reports through
        # the same field instead of an awkwardly-named `continuous_*`. Both are
        # null exactly when no solution was returned.
        write(io, "  primary_length: $(fmtx(isempty(solutions) ? NaN : solutions[1].primary_length))\n")
        write(io, "  primary_unc: $(fmtx(isempty(solutions) ? NaN : solutions[1].primary_unc))\n")
        write(io, "  astar_iterations: $(converged_iter)\n")
        write(io, "  discrete_uncertainties: $(fmtlist(isempty(paths) ? Float64[] : uncs))\n")
        # The primary's SEED uncertainty, full precision, for the same machine-read
        # reason as primary_unc: run_constraint_sweep.jl anchors its ladder here.
        # It has to, because unc_radius_threshold gates the discrete A* (the
        # exact_unc check at the goal pop), not the refined spline — so a ladder
        # anchored on primary_unc asks the search to certify on the polyline a
        # bound only the spline ever met, and the reference's own seed is rejected
        # at pct=100. Usually ≥ primary_unc, since the polyline is longer than the
        # spline it seeds and so dead-reckons more — but NOT guaranteed: with the
        # constraint inactive the barrier purely shortens, and shortening can move
        # off a landmark (sweep s006: 5.8567 discrete vs 5.8603 refined).
        write(io, "  primary_disc_unc: $(fmtx(isempty(paths) || isempty(uncs) ? NaN : uncs[end]))\n")
        write(io, "  continuous_primary_length: $(fmt3(main_cont_len))\n")
        write(io, "  continuous_uncertainties: $(fmtlist(main_cont_uncs))\n")
        # What the returned trajectory actually is: a certified solution, or a
        # fallback. Without this a violating fallback is indistinguishable from an
        # optimized result in the manifest.
        if !isnothing(main_refine)
            write(io, "  refinement_status: $(main_refine.status)\n")
            write(io, "  refinement_min_slack: $(fmtx(main_refine.min_slack))\n")
        end
    end

    return solutions
end

# ── Public planner entry points (selected by name in `algorithms:`) ──
# hexspline_cl : joint A* seed + continuous B-spline refinement (full pipeline).
# straight_cont: ABLATION of the discrete search. Everything downstream of the
#   seed is identical to hexspline_cl — same homotopy commitment
#   (build_obstacle_plan), same constraints, same single-phase barrier optimizer
#   — only the seed differs: a direct start→goal line instead of the A* path.
#   The straight seed passes no gate, so it typically starts infeasible and the
#   barrier must recover it before it can shorten anything; `refinement_status:
#   recovery_failed` in results.yaml is the case where it could not, which is the
#   measurement this ablation exists to produce.
# discrete_only: joint A* seed, no continuous stage (discrete polyline output).
plan_hexspline_cl(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:discrete, run_continuous=true)

plan_straight_cont(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:straight, run_continuous=true)

plan_discrete_only(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:discrete, run_continuous=false)
