# ==========================================================================
# hexspline_cl: joint discrete A* + continuous B-spline refinement
# ==========================================================================
# Algorithm-specific tuning. Only materialized when this planner is selected
# (see src/config.jl / config/hexspline_cl.yaml).

const ASTAR_MODE      = Symbol(CFG["astar_mode"])
const PRIMARY_EPSILON = Float64(CFG["primary_epsilon"])

const UNC_RADIUS_THRESHOLD = Float64(CFG["unc_radius_threshold"])
const UNC_FEAS_TOL         = Float64(CFG["unc_feas_tol"])

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
const MIN_TURN_RADIUS_M                = Float64(CFG["min_turn_radius_m"])
const MAX_CURVATURE                    = 1.0 / MIN_TURN_RADIUS_M
const CONT_BARRIER_START               = Float64(CFG["cont_barrier_start"])
const CONT_BARRIER_DECAY               = Float64(CFG["cont_barrier_decay"])
const CONT_BARRIER_STAGES              = Int(CFG["cont_barrier_stages"])
const CONT_LINESEARCH_SHRINK           = Float64(CFG["cont_linesearch_shrink"])
const CONT_MIN_IMPROVEMENT             = Float64(CFG["cont_min_improvement"])

const CONT_OPT_H             = Float64(CFG["cont_opt_h"])
const CONT_ADAM_B1           = Float64(CFG["cont_adam_b1"])
const CONT_ADAM_B2           = Float64(CFG["cont_adam_b2"])
const CONT_ADAM_EPS          = Float64(CFG["cont_adam_eps"])
const CONT_CONV_TOL          = Float64(CFG["cont_conv_tol"])
const CONT_UNC_USE_WAYPOINTS = Bool(CFG["cont_unc_use_waypoints"])
const CONT_SMOOTH_PENALTY       = Float64(CFG["cont_smooth_penalty"])
const CONT_BARRIER_HARD_PENALTY = Float64(CFG["cont_barrier_hard_penalty"])

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
    paths   :: Vector{Vector{Int}}  # full path per agent (sequence of nodes); last = primary
    covs    :: Vector{Matrix{Float64}}  # covariance at end of each path
    dists   :: Vector{Float64}      # arc-distance per agent (computed from B-spline)
    g       :: Float64              # primary arc-distance (cost)
    parent  :: Int                  # index in states vector; -1 = root
    visited :: Vector{BitVector}    # per-agent visited sets (cycle detection)
end

@inline function joint_heuristic(paths::Vector{Vector{Int}},
                                  goal::Int,
                                  graph::LandmarkGraph)
    primary = length(paths)
    if isempty(paths[primary])
        return 0.0  # no progress yet
    end
    v = paths[primary][end]  # current node of primary
    return graph.shortest_paths[v, goal]
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

@inline function joint_node_key(paths::Vector{Vector{Int}})
    nodes = Vector{Int}(undef, length(paths))
    for a in 1:length(paths)
        nodes[a] = paths[a][end]
    end
    return Tuple(nodes)
end

@inline function joint_path_key(paths::Vector{Vector{Int}})
    key = Vector{Tuple{Vararg{Int}}}(undef, length(paths))
    for a in 1:length(paths)
        key[a] = Tuple(paths[a])
    end
    return Tuple(key)
end

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

        # Node-level Pareto frontier with SOUND covariance dominance.
        # Keep labels (distance, covariance) and only prune when an existing
        # label is better in distance and covariance PSD order.
        frontier_at_node = Dict{Int, Vector{Tuple{Float64, Matrix{Float64}}}}()
        frontier_at_node[1] = [(0.0, copy(states_sa[1].cov))]

        while !isempty(pq_sa) && iter_count_sa < ASTAR_ITERATION_LIMIT
            si = dequeue!(pq_sa)
            S = states_sa[si]
            iter_count_sa += 1

            popped_unc = unc_radius(S.cov)
            if PRUNE_BY_PRIMARY_UNCERTAINTY && unc_exceeds_threshold(popped_unc, unc_threshold, UNC_FEAS_TOL)
                println("  [Constraint A*] Pruned state at iter $iter_count_sa: node=$(S.node), primary_unc=$(round(popped_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
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
                    println("  ✓ FEASIBLE SOLUTION at iter $iter_count_sa: dist=$(round(exact_dists[1], digits=3)), unc=$(round(exact_unc, digits=4))")
                    println("  [Constraint A*] Single-agent complete: $(iter_count_sa) iterations, final_dist=$(round(exact_dists[1], digits=3))")
                    return [path], [exact_dists[1]], exact_unc, iter_count_sa
                end

                println("  [Constraint A*] Goal popped but infeasible under exact eval: unc=$(round(exact_unc, digits=4))")
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

        println("  [Constraint A*] No feasible single-agent solution found")
        return [Int[]], [0.0], Inf, iter_count_sa
    end

    # ── Initial state: all agents at node 1 (start) ──────────────────────────
    init_paths   = [fill(1, 1) for _ in 1:na]
    init_visited = [falses(n) for _ in 1:na]
    for a in 1:na; init_visited[a][1] = true; end

    # Evaluate initial state (all agents at node 1)
    init_covs, init_dists = evaluate_full_paths(init_paths, graph, lms, na)

    states = JointState[]
    push!(states, JointState(init_paths, init_covs, init_dists, 0.0, -1, init_visited))

    # Exact state cache keyed by immutable joint path tuples.
    exact_state_best = Dict{Tuple{Vararg{Tuple{Vararg{Int}}}}, Int}()
    exact_state_best[joint_path_key(init_paths)] = 1

    w_astar = 1.0 + PRIMARY_EPSILON
    pq = PriorityQueue{Int, Tuple{Float64, Float64, Float64}}()
    init_h = joint_heuristic(init_paths, goal, graph)
    init_f = init_dists[primary] + w_astar * init_h
    init_unc = unc_radius(init_covs[primary])
    enqueue!(pq, 1, (init_f, init_unc, support_idle_score(init_dists)))

    iter_count         = 0

    # Safe dominance frontier keyed by (joint nodes, exact visited signature).
    # This only compares states with identical future action sets.
    frontier_by_signature = Dict{Any, Vector{Tuple{Float64, Matrix{Float64}}}}()
    init_sig = (joint_node_key(init_paths), joint_visited_key(init_visited))
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

    function debug_joint_astar_plot(state::JointState, best_si::Int, iter_no::Int)
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

        for (ai, path) in enumerate(state.paths)
            isempty(path) && continue
            px = [graph.landmarks[j].x for j in path]
            py = [graph.landmarks[j].y for j in path]
            clr = ai == primary ? :blue : :purple
            lbl = ai == primary ? "Primary (expanding)" : "Support $ai (expanding)"
            plot!(plt, px, py, label=lbl, color=clr, linewidth=2.0,
                  linestyle=(ai == primary ? :solid : :dash))
        end

        if best_si > 0
            best = states[best_si]
            for (ai, path) in enumerate(best.paths)
                isempty(path) && continue
                px = [graph.landmarks[j].x for j in path]
                py = [graph.landmarks[j].y for j in path]
                clr = ai == primary ? :darkgreen : :darkgray
                lbl = ai == primary ? "Primary (best)" : "Support $ai (best)"
                plot!(plt, px, py, label=lbl, color=clr, linewidth=1.5, alpha=0.8,
                      linestyle=:dot)
            end
        end

        prim_node = isempty(state.paths[primary]) ? 0 : state.paths[primary][end]
        prim_h = isinf(graph.shortest_paths[prim_node, goal]) ? "∞" : "$(round(graph.shortest_paths[prim_node, goal], digits=1))"
        ann = "Iter $iter_no, prim_node=$prim_node, h=$prim_h"
        annotate!(plt, (graph.landmarks[1].x, graph.landmarks[1].y + 100), text(ann, :black, 10))
        return plt
    end

    while !isempty(pq) && iter_count < ASTAR_ITERATION_LIMIT
        si  = dequeue!(pq)
        S   = states[si]
        iter_count += 1

        popped_unc = unc_radius(S.covs[primary])
        if PRUNE_BY_PRIMARY_UNCERTAINTY && unc_exceeds_threshold(popped_unc, unc_threshold, UNC_FEAS_TOL)
            println("  [Constraint A*] Pruned joint state at iter $iter_count: primary_unc=$(round(popped_unc, digits=4)) > threshold=$(round(unc_threshold, digits=4))")
            continue
        end

        # --- Animation: plot only the requested iteration window, sampled sparsely ---
        if animate_enabled && iter_count >= animate_start_iter && iter_count <= animate_start_iter + animate_limit - 1 &&
           mod(iter_count - animate_start_iter, animate_sample_period) == 0
            plt = debug_joint_astar_plot(S, 0, iter_count)
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

        # Progress update early; when animation is enabled, match the console cadence
        # to the animation sampling period so logs align with plotted frames.
        progress_every = animate_enabled ? animate_sample_period : 100
        if iter_count <= 5 || mod(iter_count, progress_every) == 0
            prim_node = isempty(S.paths[primary]) ? 0 : S.paths[primary][end]
            prim_h = isinf(graph.shortest_paths[prim_node, goal]) ? "∞" : "$(round(graph.shortest_paths[prim_node, goal], digits=1))"
            #println("  [Constraint A*] Iter $iter_count, prim_node=$prim_node, h=$prim_h, queue_size=$(length(pq))")
        end

        # Standard A* ordering uses f only for priority; no incumbent pruning.
        h = joint_heuristic(S.paths, goal, graph)
        f = S.g + w_astar * h


        # Check if primary reached goal cell
        if is_goal_cell[S.paths[primary][end]]
            agent_paths = [copy(S.paths[a]) for a in 1:na]
            exact_covs, exact_dists = evaluate_full_paths(agent_paths, graph, lms, na)
            exact_unc = unc_radius(exact_covs[primary])
            if unc_within_threshold(exact_unc, unc_threshold, UNC_FEAS_TOL)
                println("  ✓ FEASIBLE SOLUTION at iter $iter_count: dist=$(round(exact_dists[primary], digits=3)), unc=$(round(exact_unc, digits=4))")
                println("  [Constraint A*] Complete: $(iter_count) iterations, final_dist=$(round(exact_dists[primary], digits=3))")
                # Save collected debug frames before this early return (bypasses the end-of-function save below).
                if animate_enabled && !animation_saved && iter_count > 1
                    gif(anim_frames, debug_gif_path, fps=20)
                    animation_saved = true
                    println("  [Constraint A*] Animation saved to $debug_gif_path (iters $(animate_start_iter)-$(min(iter_count, animate_start_iter + animate_limit - 1)))")
                end
                return agent_paths, exact_dists, exact_unc, iter_count
            else
                println("  [Constraint A*] Goal popped but infeasible under exact eval: unc=$(round(exact_unc, digits=4)) > $(round(unc_threshold, digits=4))")
            end
            continue
        end

        # ── Expansion: move all agents synchronously one edge at a time ──────
        # NOTE: joint_astar_collect contains a near-duplicate of this closure.
        # The two differ in: (a) pruning gates present only here, (b) exact_state_best
        # guard condition, (c) enqueue style. See joint_astar_collect for the collect variant.
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
                new_paths = [copy(S.paths[a]) for a in 1:na]
                new_covs = Vector{Matrix{Float64}}(undef, na)
                new_dists = Vector{Float64}(undef, na)
                new_visited = [copy(S.visited[a]) for a in 1:na]

                for a in 1:na
                    u = candidate_nodes[a]
                    prev_node = S.paths[a][end]
                    push!(new_paths[a], u)
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
                        p0 = graph.landmarks[S.paths[a][end]]; p1 = graph.landmarks[candidate_nodes[a]]
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

                new_h = joint_heuristic(new_paths, goal, graph)
                f_exact = new_g + w_astar * new_h

                sig_key = (joint_node_key(new_paths), joint_visited_key(new_visited))
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

                new_path_key = joint_path_key(new_paths)
                if haskey(exact_state_best, new_path_key)
                    return
                end

                push!(states, JointState(copy(new_paths), new_covs, new_dists,
                                          new_g, si, new_visited))
                new_si = length(states)
                exact_state_best[new_path_key] = new_si
                enqueue!(pq, new_si, (f_exact, prim_unc, support_idle_score(new_dists)))
                return
            end

            agent = move_order[agent_idx]
            curr_node = S.paths[agent][end]
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
    println("  [Constraint A*] No feasible solution found")
    return [Int[] for _ in 1:na], zeros(na), Inf, iter_count
end


# ------------------------------------------------------------------
# Collector variant of joint A*: run up to `max_iters` expansions and
# collect all goal states popped. Returns a vector of tuples
# (agent_paths, primary_dist, primary_unc, iter_popped).
# ------------------------------------------------------------------
function joint_astar_collect(graph::LandmarkGraph,
                             lms::Vector{Landmark},
                             unc_threshold::Float64,
                             num_agents::Int; max_iters::Int=ASTAR_ITERATION_LIMIT)
    # We'll largely reuse joint_astar logic but keep the loop focused on
    # collecting goal pops rather than returning on first feasible solution.
    n         = graph.n
    goal      = n
    na        = num_agents
    primary   = na
    is_goal_cell = falses(n)
    for v in 1:(n-1); goal in graph.neighbors[v] && (is_goal_cell[v] = true); end

    # Use single-agent fast path for na==1
    if na == 1
        # reuse single-agent search but collect all goal pops (rare)
        w_astar = 1.0 + PRIMARY_EPSILON
        init_visited = falses(n)
        init_visited[1] = true
        states_sa = State[]
        push!(states_sa, State(1, 0.0, copy(lms[1].cov), -1, init_visited))
        pq_sa = PriorityQueue{Int, Tuple{Float64, Float64}}()
        h0 = graph.shortest_paths[1, goal]
        enqueue!(pq_sa, 1, (w_astar * h0, unc_radius(states_sa[1].cov)))
        frontier_at_node = Dict{Int, Vector{Tuple{Float64, Matrix{Float64}}}}()
        frontier_at_node[1] = [(0.0, copy(states_sa[1].cov))]

        iter_count_sa = 0
        collected = []
        while !isempty(pq_sa) && iter_count_sa < max_iters
            si = dequeue!(pq_sa)
            S = states_sa[si]
            iter_count_sa += 1

            if is_goal_cell[S.node]
                path = Int[]; psi = si
                while psi != -1
                    pushfirst!(path, states_sa[psi].node)
                    psi = states_sa[psi].parent
                end
                exact_covs, exact_dists = evaluate_full_paths([path], graph, lms, 1)
                exact_unc = unc_radius(exact_covs[1])
                push!(collected, ( [path], exact_dists[1], exact_unc, iter_count_sa ))
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
                if !haskey(frontier_at_node, u)
                    frontier_at_node[u] = Tuple{Float64, Matrix{Float64}}[]
                end
                dominated = false
                for (od, ocov) in frontier_at_node[u]
                    if od <= nd + 1e-9 && cov_dominates(ocov, ncov)
                        dominated = true; break
                    end
                end
                dominated && continue
                kept = Tuple{Float64, Matrix{Float64}}[]
                for (od, ocov) in frontier_at_node[u]
                    if nd <= od + 1e-9 && cov_dominates(ncov, ocov)
                        continue
                    end
                    push!(kept, (od, ocov))
                end
                push!(kept, (nd, copy(ncov)))
                frontier_at_node[u] = kept

                nvis = copy(S.visited); nvis[u] = true
                push!(states_sa, State(u, nd, ncov, si, nvis))
                nsi = length(states_sa)
                nh = graph.shortest_paths[u, goal]
                enqueue!(pq_sa, nsi, (nd + w_astar * nh, unc_radius(ncov)))
            end
        end

        return collected
    end

    # For multi-agent case use joint A* core but collect goal pops
    init_paths   = [fill(1, 1) for _ in 1:na]
    init_visited = [falses(n) for _ in 1:na]
    for a in 1:na; init_visited[a][1] = true; end
    init_covs, init_dists = evaluate_full_paths(init_paths, graph, lms, na)
    states = JointState[]; push!(states, JointState(init_paths, init_covs, init_dists, 0.0, -1, init_visited))
    exact_state_best = Dict{Tuple{Vararg{Tuple{Vararg{Int}}}}, Int}()
    exact_state_best[joint_path_key(init_paths)] = 1
    w_astar = 1.0 + PRIMARY_EPSILON
    pq = PriorityQueue{Int, Tuple{Float64, Float64, Float64}}()
    init_h = joint_heuristic(init_paths, goal, graph)
    init_f = init_dists[primary] + w_astar * init_h
    enqueue!(pq, 1, (init_f, unc_radius(init_covs[primary]), support_idle_score(init_dists)))

    frontier_by_signature = Dict{Any, Vector{Tuple{Float64, Matrix{Float64}}}}()
    init_sig = (joint_node_key(init_paths), joint_visited_key(init_visited))
    frontier_by_signature[init_sig] = [(0.0, copy(init_covs[primary]))]

    iter_count = 0
    collected = []

    while !isempty(pq) && iter_count < max_iters
        si = dequeue!(pq); S = states[si]; iter_count += 1

        mod(iter_count, 10000) == 0 && println("  A* iter $iter_count")

        if is_goal_cell[S.paths[primary][end]]
            agent_paths = [copy(S.paths[a]) for a in 1:na]
            exact_covs, exact_dists = evaluate_full_paths(agent_paths, graph, lms, na)
            exact_unc = unc_radius(exact_covs[primary])
            push!(collected, (agent_paths, exact_dists[primary], exact_unc, iter_count))
            # continue searching for other goal candidates
            continue
        end

        # ── Expansion (near-duplicate of joint_astar's expand_joint_moves) ──────
        # Differences from joint_astar: no pruning gates, different exact_state_best
        # guard (haskey + > 0 check), inline f-value in enqueue call.
        candidate_nodes = Vector{Int}(undef, na)
        move_order = Vector{Int}(undef, na)
        move_order[1] = primary; order_pos = 2
        for a in 1:na; a == primary && continue; move_order[order_pos] = a; order_pos += 1; end

        function expand_joint_moves(agent_idx::Int, primary_dist::Float64)
            if agent_idx > na
                new_paths = [copy(S.paths[a]) for a in 1:na]
                new_covs = Vector{Matrix{Float64}}(undef, na)
                new_dists = Vector{Float64}(undef, na)
                new_visited = [copy(S.visited[a]) for a in 1:na]
                for a in 1:na
                    u = candidate_nodes[a]
                    prev_node = S.paths[a][end]
                    push!(new_paths[a], u)
                    new_dists[a] = S.dists[a] + graph.distance[prev_node, u]
                    new_covs[a] = edge_cov_continuous(prev_node, u, graph, lms, S.covs[a])
                    new_visited[a][u] = true
                end
                for a in 1:(primary - 1); new_dists[a] > new_dists[primary] && return; end
                new_covs = apply_joint_step_comms(new_covs, candidate_nodes, new_dists, graph)
                # Chance-constrained static-obstacle feasibility filter (no-op if none).
                if !isempty(OBSTACLES)
                    for a in 1:na
                        p0 = graph.landmarks[S.paths[a][end]]; p1 = graph.landmarks[candidate_nodes[a]]
                        segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), new_covs[a]) || return
                    end
                end
                new_g = new_dists[primary]
                sig_key = (joint_node_key(new_paths), joint_visited_key(new_visited))
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
                new_path_key = joint_path_key(new_paths)
                if haskey(exact_state_best, new_path_key) && exact_state_best[new_path_key] > 0
                    return
                end
                push!(states, JointState(copy(new_paths), new_covs, new_dists, new_g, si, new_visited))
                new_si = length(states)
                exact_state_best[new_path_key] = new_si
                enqueue!(pq, new_si, (new_g + w_astar * joint_heuristic(new_paths, goal, graph), unc_radius(new_covs[primary]), support_idle_score(new_dists)))
                return
            end
            agent = move_order[agent_idx]
            curr_node = S.paths[agent][end]
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

    return collected
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
        for ai in 1:(n_agents - 1)
            all_agent_wpts[ai] = [(sx, sy), (sx, sy)]
            is_primary_mask[ai] = false
        end
        prim_wpts = Vector{Tuple{Float64,Float64}}(undef, n_seed)
        for i in 1:n_seed
            t = (i - 1) / (n_seed - 1)
            prim_wpts[i] = ((1 - t) * sx + t * gx, (1 - t) * sy + t * gy)
        end
        all_agent_wpts[n_agents] = prim_wpts
        is_primary_mask[n_agents] = true
    else
        for (ai, path) in enumerate(paths)
            all_agent_wpts[ai] = [(graph.landmarks[i].x, graph.landmarks[i].y) for i in path]
            is_primary_mask[ai] = (ai == length(paths))
        end
        # Primary now terminates at the goal cell (not the terminal node), so its last
        # waypoint is the cell centre.  Override to exact goal_pos so the B-spline is
        # pinned to the true goal regardless of cell-centre drift.
        # ponytail: no-op when goal sits on a cell centre (all current scenarios).
        prim = length(paths)
        all_agent_wpts[prim][end] = (graph.landmarks[graph.n].x, graph.landmarks[graph.n].y)
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
        return nothing
    end

    # Static-obstacle avoidance (MINVO enclosure, chance-constrained). Commit
    # each spline segment to the obstacle face the discrete seed cleared; those
    # committed half-spaces enter the smoothing/barrier objectives below as
    # linear rows over the MINVO hull vertices, and are re-verified after
    # refinement. No-op (empty plans) when there are no obstacles.
    # The straight_cont pipeline (straight_seed) skips the discrete filter, so it
    # has no committed homotopy — which side of each obstacle to pass — to enforce
    # continuously; reuse the empty no-obstacle plan (see OBSTACLE_AVOIDANCE.md).
    obstacle_plans = straight_seed ? [SegFace[] for _ in eachindex(all_agent_wpts)] :
                     build_obstacle_plan(all_agent_wpts, SPLINE_DEGREE)

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

    @inline function support_length_violation(primary_len::Float64, support_lens::Vector{Float64})
        v = 0.0
        for sl in support_lens
            slack = primary_len - sl
            if slack < -UNC_FEAS_TOL
                v += (-slack)^2
            end
        end
        return v
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

    # Initialize optimizer state
    flat = pack_waypoints(all_agent_wpts)
    init_len, init_unc, init_wpts, init_covs, init_curvs, init_max_curv, init_ctrls, init_support_lens = eval_continuous(flat)
    println("  Initial (discrete→spline): prim_len=$(round(init_len,digits=3)), unc=$(round(init_unc,digits=4))")

    init_curvature_ok = all(all(isfinite(κ) ? κ <= MAX_CURVATURE + 1e-9 : true for κ in curvset) for curvset in init_curvs)
    init_support_len_ok = all(sl <= init_len + UNC_FEAS_TOL for sl in init_support_lens)
    init_obstacles_ok = obstacles_clear(obstacle_plans, init_ctrls, init_covs)
    init_feasible = unc_within_threshold(init_unc, cont_threshold, UNC_FEAS_TOL) && init_curvature_ok && init_support_len_ok && init_obstacles_ok

    if !init_feasible
        println("  [Smoothing] Initial point is infeasible; smoothing into feasible region...")
        function smoothing_objective(f::Vector{Float64})
            plen, unc, _, covs_all, curvatures, max_curvs, ctrl_list, support_lens = eval_continuous(f)
            unc_penalty = 0.0
            if unc_exceeds_threshold(unc, cont_threshold, UNC_FEAS_TOL)
                unc_penalty = CONT_SMOOTH_PENALTY * (unc - cont_threshold)^2
            end
            curv_penalty = 0.0
            for curvset in curvatures
                for κ in curvset
                    if κ > MAX_CURVATURE
                        curv_penalty += CONT_SMOOTH_PENALTY * (κ - MAX_CURVATURE)^2
                    end
                end
            end
            support_len_penalty = CONT_SMOOTH_PENALTY * support_length_violation(plen, support_lens)
            obstacle_penalty = obstacle_constraint_value(obstacle_plans, ctrl_list, covs_all; mode=:smooth)
            return plen + unc_penalty + curv_penalty + support_len_penalty + obstacle_penalty
        end

        smooth_adam_m = zeros(total_free_cont)
        smooth_adam_v = zeros(total_free_cont)
        smooth_iter = 0; smooth_max_iters = 300
        h_smooth = CONT_OPT_H; lr_smooth = CONT_OPT_LR * 0.5
        smooth_flat = copy(flat)
        while smooth_iter < smooth_max_iters
            smooth_iter += 1
            obj0 = smoothing_objective(smooth_flat)
            grad = zeros(total_free_cont)
            for k in 1:total_free_cont
                smooth_flat[k] += h_smooth; obj1 = smoothing_objective(smooth_flat); grad[k] = (obj1 - obj0) / h_smooth; smooth_flat[k] -= h_smooth
            end
            b1t = CONT_ADAM_B1^smooth_iter; b2t = CONT_ADAM_B2^smooth_iter
            step = zeros(total_free_cont)
            for k in 1:total_free_cont
                g = grad[k]
                smooth_adam_m[k] = CONT_ADAM_B1 * smooth_adam_m[k] + (1 - CONT_ADAM_B1) * g
                smooth_adam_v[k] = CONT_ADAM_B2 * smooth_adam_v[k] + (1 - CONT_ADAM_B2) * g^2
                m̂ = smooth_adam_m[k] / (1 - b1t); v̂ = smooth_adam_v[k] / (1 - b2t)
                step[k] = lr_smooth * m̂ / (sqrt(v̂) + CONT_ADAM_EPS)
            end
            trial = smooth_flat .- step; trial_obj = smoothing_objective(trial); backtracks = 0
            while (!isfinite(trial_obj) || trial_obj > obj0 - CONT_MIN_IMPROVEMENT) && backtracks < 12
                step .*= CONT_LINESEARCH_SHRINK; trial = smooth_flat .- step; trial_obj = smoothing_objective(trial); backtracks += 1
            end
            if !isfinite(trial_obj) || trial_obj > obj0 - CONT_MIN_IMPROVEMENT; break; end
            smooth_flat .= trial
            slen, sunc, swpts, scovs, scurvs, smax_curv, sctrls, ssupport_lens = eval_continuous(smooth_flat)
            support_len_ok = all(sl <= slen + UNC_FEAS_TOL for sl in ssupport_lens)
            sfeas = unc_within_threshold(sunc, cont_threshold, UNC_FEAS_TOL) && all(all(isfinite(κ) ? κ <= MAX_CURVATURE + 1e-9 : true for κ in curvset) for curvset in scurvs) && support_len_ok && obstacles_clear(obstacle_plans, sctrls, scovs)
            if mod(smooth_iter, 50) == 0
                println("    Smooth iter $(lpad(smooth_iter,3)): len=$(round(slen,digits=2)), unc=$(round(sunc,digits=4))")
            end
            if sfeas; println("    → Feasible at smoothing iter $smooth_iter"); break; end
        end
        flat = smooth_flat
        init_len, init_unc, init_wpts, init_covs, init_curvs, init_max_curv, init_ctrls, init_support_lens = eval_continuous(flat)
        println("  After smoothing: prim_len=$(round(init_len,digits=3)), unc=$(round(init_unc,digits=4))")
    else
        println("  [Smoothing] Initial point is feasible; proceeding to barrier optimization")
    end

    # Barrier optimizer (simplified reuse of in-file logic but using cont_threshold)
    adam_m = zeros(total_free_cont); adam_v = zeros(total_free_cont)
    best_flat = copy(flat); best_len = init_len; best_unc = init_unc
    best_feasible_flat = nothing; best_feasible_len = Inf; best_feasible_unc = Inf
    prev_len = init_len

    function spline_barrier_objective(flat::Vector{Float64}, barrier_mu::Float64)
        prim_len, goal_unc, _, covs_all, curvatures, _, ctrl_list, support_lens = eval_continuous(flat)
        unc_slack = cont_threshold - goal_unc
        penalty = 0.0
        if unc_slack <= 1e-9
            penalty += CONT_BARRIER_HARD_PENALTY * (1e-9 - unc_slack)^2
        else
            penalty -= barrier_mu * log(unc_slack)
        end
        for curvset in curvatures
            for κ in curvset
                slack = MAX_CURVATURE - κ
                if slack <= 1e-9
                    penalty += CONT_BARRIER_HARD_PENALTY * (1e-9 - slack)^2
                else
                    penalty -= barrier_mu * log(slack)
                end
            end
        end
        for sl in support_lens
            slack = prim_len - sl
            if slack <= 1e-9
                penalty += CONT_BARRIER_HARD_PENALTY * (1e-9 - slack)^2
            else
                penalty -= barrier_mu * log(slack)
            end
        end
        penalty += obstacle_constraint_value(obstacle_plans, ctrl_list, covs_all; mode=:barrier, barrier_mu=barrier_mu)
        return prim_len + penalty
    end

    stage_iters = max(25, cld(CONT_OPT_ITERS, CONT_BARRIER_STAGES))
    barrier_mu = CONT_BARRIER_START; total_iter = 0
    for stage in 1:CONT_BARRIER_STAGES
        println("  Barrier stage $(stage)/$(CONT_BARRIER_STAGES): μ=$(round(barrier_mu,digits=4))")
        for iter in 1:stage_iters
            total_iter += 1
            obj0 = spline_barrier_objective(flat, barrier_mu)
            grad = zeros(total_free_cont)
            for k in 1:total_free_cont
                flat[k] += CONT_OPT_H; obj1 = spline_barrier_objective(flat, barrier_mu); grad[k] = (obj1 - obj0) / CONT_OPT_H; flat[k] -= CONT_OPT_H
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
            trial = flat .- step; trial_obj = spline_barrier_objective(trial, barrier_mu); backtracks = 0
            while (!isfinite(trial_obj) || trial_obj > obj0 - CONT_MIN_IMPROVEMENT) && backtracks < 12
                step .*= CONT_LINESEARCH_SHRINK; trial = flat .- step; trial_obj = spline_barrier_objective(trial, barrier_mu); backtracks += 1
            end
            if !isfinite(trial_obj) || trial_obj > obj0 - CONT_MIN_IMPROVEMENT
                println("  [Spline barrier] line search stalled; stopping at stage $stage")
                break
            end
            flat .= trial
            len2, unc2, wpts2, covs2, curv2, max_curv2, ctrls2, support_lens2 = eval_continuous(flat)
            support_len_ok = all(sl <= len2 + UNC_FEAS_TOL for sl in support_lens2)
            feasible = unc_within_threshold(unc2, cont_threshold, UNC_FEAS_TOL) && all(all(isfinite(κ) ? κ <= MAX_CURVATURE + 1e-9 : true for κ in curvset) for curvset in curv2) && support_len_ok && obstacles_clear(obstacle_plans, ctrls2, covs2)
            if feasible && len2 < best_feasible_len
                best_feasible_len = len2; best_feasible_unc = unc2; best_feasible_flat = copy(flat)
            end
            if len2 < best_len
                best_len = len2; best_unc = unc2; best_flat = copy(flat)
            end
            if feasible && abs(len2 - prev_len) < CONT_CONV_TOL
                println("  → Converged at iter $total_iter (Δlen=$(round(abs(len2-prev_len),digits=6)))")
                break
            end
            prev_len = len2
        end
        barrier_mu *= CONT_BARRIER_DECAY
    end

    selected_flat = isnothing(best_feasible_flat) ? best_flat : best_feasible_flat
    opt_len, opt_unc, opt_wpts, opt_covs, opt_curvs, opt_max_curv, opt_ctrls, opt_support_lens = eval_continuous(selected_flat)
    println("  Final: prim_len=$(round(opt_len,digits=3)), unc=$(round(opt_unc,digits=4)), threshold=$(round(cont_threshold,digits=4))")

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

    # Evaluate initial discrete solution (waypoints from A* path)
    init_len, init_unc, _, _, _, _, _, _ = eval_continuous(flat)

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

    # Combined side-by-side figure: discrete (left) vs continuous (right)
    save_path_figures_pair(plt1, plt2,
        fig_path(output_dir, string(fig_prefix, "fig_compare_discrete_continuous", (fig_prefix == "" ? "" : "_"), string(round(opt_len,digits=2)), ".png")),
        d_comm, c_comm, d_landmark_events, c_landmark_events)
    println("Fig combined (", fig_prefix, ") saved: Discrete Len=$(round(init_len,digits=2)) → Continuous Len=$(round(opt_len,digits=2))")
    plt_unc = plot_unc_profile(
        [("discrete", d_arcs, d_covs, :dash, 1.2, 1.2),
         ("continuous", c_arcs, c_covs, :solid, 2.0, 1.3)],
        num_agents; threshold=cont_threshold,
        title=string(fig_prefix == "" ? "main" : fig_prefix, " — uncertainty profile (", LANDMARK_SCENARIO, ")"))
    unc_profile_fname = fig_path(output_dir, string(fig_prefix == "" ? "main" : fig_prefix, "_unc_profile.png"))
    savefig(plt_unc, unc_profile_fname)
    println("Fig uncertainty profile (", fig_prefix == "" ? "main" : fig_prefix, ") saved: ", unc_profile_fname)

    return opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls
end

# ==========================================================================
# Entry point: joint discrete A* seed search -> continuous B-spline refinement
# ==========================================================================
# Returns a Vector of solutions (NamedTuples: id, ctrls, primary_length,
# primary_unc). In `astar_mode: threshold` this is a single "main" solution;
# in `astar_mode: limit` it is the full Pareto front (main + one per seed) --
# see notes/PLAN.md "Pareto front handling" for why this is a Vector and not
# a single path.
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

    pareto_collected = Vector{Tuple{Vector{Vector{Int}}, Float64, Float64, Int}}()
    discrete_iter = 0

    function run_discrete_seed_search(search_unc_threshold::Float64)
        println("\n  Running joint discrete A* collector (threshold=$(round(search_unc_threshold, digits=4)))... (max_iters=$(ASTAR_ITERATION_LIMIT))")
        # :limit collects every non-dominated discrete candidate up to the
        # iteration budget and builds a genuine Pareto front (multiple seeds
        # go on to their own continuous refinement). :threshold always stops
        # at the first feasible candidate, so there is only ever one seed —
        # no front to prune/collect/plot, so that machinery is skipped
        # entirely rather than run trivially over a 1-element list.
        if ASTAR_MODE == :limit
            collected = joint_astar_collect(graph, landmarks, search_unc_threshold, NUM_AGENTS; max_iters=ASTAR_ITERATION_LIMIT)
            if isempty(collected)
                println("  ✗ No goal candidates found by A* collector")
                return [Int[] for _ in 1:(NUM_AGENTS - 1)], Int[], Inf
            end

            # Each entry: (agent_paths, dist, unc, iter)
            # Compute Pareto front (minimize dist and unc)
            tol = 1e-9
            pareto = Vector{Tuple{Vector{Vector{Int}}, Float64, Float64, Int}}()
            for (paths_c, d_c, u_c, it) in collected
                dominated = false
                for (p_paths, p_d, p_u, _) in pareto
                    if p_d <= d_c + tol && p_u <= u_c + tol
                        dominated = true; break
                    end
                end
                if dominated; continue; end
                # remove any existing pareto points dominated by this one
                keep = Vector{Tuple{Vector{Vector{Int}}, Float64, Float64, Int}}()
                for item in pareto
                    p_paths, p_d, p_u, p_it = item
                    if d_c <= p_d + tol && u_c <= p_u + tol
                        continue
                    end
                    push!(keep, item)
                end
                push!(keep, (paths_c, d_c, u_c, it))
                pareto = keep
            end

            # Sort pareto by increasing distance
            sort!(pareto, by = x -> x[2])

            # Plot Pareto front
            ds = [x[2] for x in pareto]
            us = [x[3] for x in pareto]
            pltp = plot(ds, us, seriestype=:scatter, xlabel="Primary distance", ylabel="Primary uncertainty", title="Pareto (distance vs uncertainty)", legend=false)
            for (i, (pp, pd, pu, it)) in enumerate(pareto)
                annotate!(pltp, pd, pu, text("$i", :black, 8))
            end
            savefig(pltp, fig_path(output_dir, "fig_pareto_discrete.png")); println("Fig Pareto (discrete) saved: $(length(pareto)) points")

            # Store pareto set for later continuous refinement
            pareto_collected = pareto
            discrete_iter = pareto[1][4]
            println("  Pareto seeds collected: $(length(pareto)). Continuous refinement deferred until optimizer is available.")

            # Return first Pareto entry as a representative seed (for compatibility)
            first_paths, first_d, first_u, _ = pareto[1]
            support_paths = pad_support_paths_to_primary(first_paths[1:NUM_AGENTS-1], first_paths[NUM_AGENTS])
            primary_path = first_paths[NUM_AGENTS]
            primary_dist = first_d
            return support_paths, primary_path, primary_dist

        elseif ASTAR_MODE == :threshold
            println("  Running threshold-stop A* (stops on first feasible under threshold)...")
            ppaths, pdists, punc, disc_iter = joint_astar(graph, landmarks, search_unc_threshold, NUM_AGENTS)
            if length(ppaths) != NUM_AGENTS || any(isempty, ppaths)
                println("  ✗ No feasible path found by threshold-stop A*")
                return [Int[] for _ in 1:(NUM_AGENTS - 1)], Int[], Inf
            end
            discrete_iter = disc_iter

            support_paths = pad_support_paths_to_primary(ppaths[1:NUM_AGENTS-1], ppaths[NUM_AGENTS])
            primary_path = ppaths[NUM_AGENTS]
            primary_dist = pdists[NUM_AGENTS]
            return support_paths, primary_path, primary_dist
        else
            error("Unsupported ASTAR_MODE=$(ASTAR_MODE). Use :limit or :threshold")
        end
    end

    if seed_mode == :straight
        println("\n  seed_mode=:straight — skipping discrete search and using direct start→goal seed")
        primary_path = [1, goal_node]
        primary_dist = graph.distance[1, goal_node]
        support_paths = [Int[1, 1] for _ in 1:(NUM_AGENTS - 1)]
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

        if seed_mode == :discrete && CONTINUE_ASTAR_ON_INFEASIBLE && ASTAR_MODE != :limit && unc_exceeds_threshold(primary_goal_unc, disc_unc_threshold, UNC_FEAS_TOL)
            println("\n  Seed exceeded relaxed discrete threshold (goal_unc=$(round(primary_goal_unc, digits=4)) > threshold=$(round(disc_unc_threshold, digits=4))).")
            println("  Continuing A* under relaxed threshold...")
            next_support_paths, next_primary_path, next_primary_dist = run_discrete_seed_search(disc_unc_threshold)

            if isempty(next_primary_path)
                println("  ✗ Relaxed-threshold A* did not find a feasible seed.")
                paths = Vector{Vector{Int}}()
                dists = Float64[]
                uncs = Float64[]
                final_global_cov = Matrix{Float64}[]
                primary_goal_unc = Inf
            else
                support_paths = next_support_paths
                primary_path = next_primary_path
                primary_dist = next_primary_dist
                paths = vcat(support_paths, [primary_path])
                final_global_cov, dists = evaluate_full_paths(paths, graph, landmarks, NUM_AGENTS)
                uncs = [unc_radius(final_global_cov[a]) for a in 1:NUM_AGENTS]
                primary_goal_unc = uncs[end]
                println("  ✓ Continued A* found relaxed-feasible seed: goal_unc=$(round(primary_goal_unc, digits=4))")
            end
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
        if TRACK_COMM_EVENTS
            write_comm_csv(csv_path(output_dir, "comm_events.csv"), comm_events_fig1)
        end
        if TRACK_LANDMARK_EVENTS
            write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), landmark_events_fig1)
        end
    end


    main_cont_uncs = Float64[]
    main_cont_len  = NaN
    solutions = NamedTuple[]

    have_paths = !isempty(paths) && all(length.(paths) .> 0)
    if run_continuous && have_paths  # Continuous optimization only if all agents have paths
        opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls = optimize_continuous(paths, graph, landmarks, NUM_AGENTS, output_dir; cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main", straight_seed=(seed_mode == :straight))
        write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
        main_cont_len = opt_len
        if !isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:NUM_AGENTS)
            main_cont_uncs = [unc_radius(opt_covs[a][end]) for a in 1:NUM_AGENTS]
        end
        push!(solutions, (id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc))
    elseif !run_continuous && have_paths
        # discrete_only pipeline: the discrete node polyline IS the deliverable.
        # Downstream (run_path_eval / mc_nees) reads main_ctrls.csv as waypoints
        # and evaluates it directly, so write the node coordinates (pin primary's
        # last point to the exact goal, matching the continuous stage).
        disc_ctrls = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
        disc_ctrls[end][end] = (graph.landmarks[goal_node].x, graph.landmarks[goal_node].y)
        write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), disc_ctrls)
        push!(solutions, (id=0, ctrls=disc_ctrls, primary_length=dists[end], primary_unc=uncs[end]))
    end

    # If Pareto seeds were collected earlier, run continuous optimizer for each now
    optimized_results = Tuple[]
    if run_continuous && length(pareto_collected) > 0
        println("\nRunning continuous refinement for $(length(pareto_collected)) Pareto seeds...")
        for (i, (ppaths, pdist, punc, it)) in enumerate(pareto_collected)
            println("  Refining Pareto seed $i (dist=$(round(pdist,digits=3)), unc=$(round(punc,digits=4)))")
            support_paths = pad_support_paths_to_primary(ppaths[1:NUM_AGENTS-1], ppaths[NUM_AGENTS])
            primary_path = ppaths[NUM_AGENTS]
            all_paths = vcat(support_paths, [primary_path])
            opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls = optimize_continuous(all_paths, graph, landmarks, NUM_AGENTS, output_dir; cont_threshold=punc, fig_prefix="pareto_$(i)")
            write_ctrls_csv(csv_path(output_dir, "pareto_$(i)_ctrls.csv"), opt_ctrls)
            push!(optimized_results, (i, pdist, punc, opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls))
            push!(solutions, (id=i, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc))
        end

        # Overlay optimized primary paths from Pareto seeds
        plt_overlay = make_base_plot(landmarks, graph)
        for (i, pdist, punc, opt_len, opt_unc, opt_wpts, opt_covs, opt_ctrls) in optimized_results
            prim = opt_wpts[end]
            xs = [w[1] for w in prim]; ys = [w[2] for w in prim]
            plot!(plt_overlay, xs, ys, label="pareto_$i (len=$(round(opt_len,digits=2)), unc=$(round(opt_unc,digits=4)))")
        end
        savefig(plt_overlay, fig_path(output_dir, "fig_pareto_continuous_overlay.png")); println("Fig Pareto continuous overlay saved.")
    end

    # ==========================================================================
    # Write results.yaml
    # ==========================================================================
    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        fmtlist(v) = isempty(v) ? "[]" : "[" * join(fmt4.(v), ", ") * "]"

        write(io, "astar_mode: $(ASTAR_MODE)\n")
        write(io, "main:\n")
        write(io, "  astar_iterations: $(converged_iter)\n")
        write(io, "  discrete_uncertainties: $(fmtlist(isempty(paths) ? Float64[] : uncs))\n")
        write(io, "  continuous_primary_length: $(fmt3(main_cont_len))\n")
        write(io, "  continuous_uncertainties: $(fmtlist(main_cont_uncs))\n")
        if !isempty(pareto_collected)
            write(io, "pareto_seeds:\n")
            for (idx, (pp, pd, pu, it)) in enumerate(pareto_collected)
                write(io, "  - rank: $(idx)\n")
                write(io, "    astar_iterations: $(it)\n")
                write(io, "    discrete_primary_unc: $(fmt4(pu))\n")
                r = findfirst(x -> x[1] == idx, optimized_results)
                if !isnothing(r)
                    _, _, _, clen, _, _, ccovs, _ = optimized_results[r]
                    cont_uncs = (!isempty(ccovs) && all(a -> !isempty(ccovs[a]), 1:NUM_AGENTS)) ?
                        [unc_radius(ccovs[a][end]) for a in 1:NUM_AGENTS] : Float64[]
                    write(io, "    continuous_primary_length: $(fmt3(clen))\n")
                    write(io, "    continuous_uncertainties: $(fmtlist(cont_uncs))\n")
                end
            end
        end
    end

    return solutions
end

# ── Public planner entry points (selected by name in `algorithms:`) ──
# hexspline_cl : joint A* seed + continuous B-spline refinement (full pipeline).
# straight_cont: straight start→goal seed + continuous refinement (no A* search).
# discrete_only: joint A* seed, no continuous stage (discrete polyline output).
plan_hexspline_cl(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:discrete, run_continuous=true)

plan_straight_cont(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:straight, run_continuous=true)

plan_discrete_only(scenario, graph::LandmarkGraph, output_dir::String) =
    _plan_hexspline(scenario, graph, output_dir; seed_mode=:discrete, run_continuous=false)
