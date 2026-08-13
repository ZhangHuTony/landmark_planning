# ==========================================================================
# clgbt: sampling-based joint-tree baseline, ported from CL_GBT-master — a
# standalone C++/OMPL planner ("Cooperative Localization - Guided Belief
# Tree") that grows a forward-propagating RRT over a CENTRALIZED multi-robot
# belief state (see CL_GBT-master/src/Planners/CLguidedRRT.cpp).
#
# What is ported: the TREE-GROWTH STRATEGY only.
#   - ONE tree over the JOINT state of all agents (mirrors their single
#     coupled belief tree, not a per-robot tree).
#   - Nearest-neighbor + steer-and-extend, exactly like the original's
#     sample -> nearest -> propagateWhileValid loop (CLguidedRRT.cpp:512-1089).
#   - Goal-biased sampling (their `goal_bias`) and "besties"-biased sampling
#     (their `besties_bias`, type 0 only — "replicate one robot's target onto
#     all robots"; the type-1 annulus variant has no clean analogue over
#     discrete graph nodes and is not ported).
#   - A whole extension is discarded, not partially kept, if any step along it
#     is infeasible — mirrors their `if (propCd == cd) addMotion` (a
#     propagation that stops short of its sampled duration is thrown away).
#
# What is NOT ported: their Sigma/Lambda belief-propagation math, their
# linear Cprop/Ccl (proprioceptive + relative-position) measurement model, and
# their chi-squared chance-constraint machinery. Every uncertainty number in
# THIS file comes from src/covariance.jl and src/obstacles.jl — the exact same
# calls hexspline_cl/greedy/formation/sequential make (evaluate_full_paths,
# segment_obstacle_free, unc_radius). So this baseline isolates ONE variable —
# sampling-based joint-tree growth vs. combinatorial joint A* — under
# identical uncertainty semantics, not a different physics.
#
# Also not ported: "rebranching" (their rho > 0 — splices tree branches back
# to the root via a time-constrained per-robot KD-tree search). It is off
# (rho=0) in the large majority of their own shipped configs and is by far the
# most intricate part of the original; the core forward-growing tree above is
# what the algorithm actually is in most of their own experiments.
#
# Supports have no goal of their own (this repo's problem has none for them):
# every RRT extension advances every agent jointly, so a support's path is
# always exactly as long, in steps, as the primary's — whatever the tree
# needed it to do to help the primary, same as in the original.
# ==========================================================================

const CLGBT_MAX_ITERATIONS = Int(get(CFG, "clgbt_max_iterations", 3000))
const CLGBT_GOAL_BIAS      = Float64(get(CFG, "clgbt_goal_bias", 0.1))
const CLGBT_BESTIES_BIAS   = Float64(get(CFG, "clgbt_besties_bias", 0.5))
const CLGBT_EXTEND_STEPS   = Int(get(CFG, "clgbt_extend_steps", 3))
const CLGBT_SEED           = Int(get(CFG, "clgbt_seed", 42))

# One joint-tree node: `agent_nodes` is where every agent stands after this
# node's edge; `seg` is the list of hex-graph nodes each agent stepped through
# to get here from the parent (so the full per-agent path is recoverable by
# walking parents and concatenating segments — see clgbt_reconstruct_paths).
# `cov`/`dist` are the EXACT joint-evaluated belief/arc-length at this node
# (from evaluate_full_paths, never hand-propagated).
struct ClgbtTreeNode
    parent::Int                     # 0 = root
    agent_nodes::Vector{Int}
    seg::Vector{Vector{Int}}
    cov::Vector{Matrix{Float64}}
    dist::Vector{Float64}
end

# Nodes with a direct in-edge to the terminal goal marker graph.n — same
# definition joint_astar uses (hexspline_cl.jl:561-564). graph.n itself is
# never a traversal target (see the "two traps" note in greedy.jl).
function clgbt_goal_adjacent(graph::LandmarkGraph)
    n = graph.n
    mask = falses(n)
    for v in 1:(n-1)
        n in graph.neighbors[v] && (mask[v] = true)
    end
    return mask
end

# Walk parent -> root, concatenating each edge's per-agent segment. Every
# agent starts at node 1 (build_hex_graph forces the start state to index 1).
function clgbt_reconstruct_paths(tree::Vector{ClgbtTreeNode}, idx::Int, na::Int)
    chain = Int[]
    k = idx
    while k != 0
        push!(chain, k)
        k = tree[k].parent
    end
    reverse!(chain)
    paths = [Int[1] for _ in 1:na]
    for k in chain
        for a in 1:na
            append!(paths[a], tree[k].seg[a])
        end
    end
    return paths
end

# Joint sample: primary's target is goal-biased; each support's target is
# besties-biased toward the primary's (else drawn independently). `pool` is
# every traversable non-terminal node (route_mask, minus graph.n itself).
function clgbt_sample_target(rng, pool::Vector{Int}, goal_nodes::Vector{Int},
                             na::Int, primary::Int, goal_bias::Float64, besties_bias::Float64)
    target = Vector{Int}(undef, na)
    target[primary] = rand(rng) < goal_bias ? rand(rng, goal_nodes) : rand(rng, pool)
    for a in 1:na
        a == primary && continue
        target[a] = rand(rng) < besties_bias ? target[primary] : rand(rng, pool)
    end
    return target
end

# Nearest existing tree node to a joint target: sum of per-agent Euclidean
# distance, linear scan. No KD-tree/GNAT (see config/clgbt.yaml) — this
# baseline stays small and reviewable rather than reproducing the original's
# NearestNeighborsKD_balance.h.
function clgbt_nearest(tree::Vector{ClgbtTreeNode}, graph::LandmarkGraph, target::Vector{Int}, na::Int)
    best_idx = 1
    best_d = Inf
    for (k, node) in enumerate(tree)
        d = 0.0
        for a in 1:na
            p = graph.landmarks[node.agent_nodes[a]]
            q = graph.landmarks[target[a]]
            d += hypot(p.x - q.x, p.y - q.y)
        end
        if d < best_d
            best_d = d
            best_idx = k
        end
    end
    return best_idx
end

# Steer-and-extend from tree node `near_idx` toward `target`, up to `d` joint
# single-edge steps (every agent takes the SAME number of steps, mirroring
# the original's one shared control/duration per propagated edge). Each step:
# every agent picks, among its own unvisited non-terminal neighbors, the one
# closest to ITS target by the graph's own Floyd-Warshall table (the same
# heuristic distance joint_astar uses) — then the WHOLE joint step is checked
# (support cap + chance-constrained obstacle clearance, same calls greedy.jl
# makes at its own per-step loop). One infeasible agent fails the whole step,
# which fails the whole extension: nothing partial is kept, mirroring the
# original's `propCd == cd` all-or-nothing propagation.
# Returns `nothing` on failure, else (agent_nodes, seg, cov, dist) for the new node.
function clgbt_try_extend(tree::Vector{ClgbtTreeNode}, near_idx::Int, target::Vector{Int},
                          d::Int, graph::LandmarkGraph, landmarks::Vector{Landmark},
                          na::Int, primary::Int)
    base_paths = clgbt_reconstruct_paths(tree, near_idx, na)
    visited = [Set(base_paths[a]) for a in 1:na]
    cur = copy(tree[near_idx].agent_nodes)
    seg = [Int[] for _ in 1:na]
    last_covs  = tree[near_idx].cov
    last_dists = tree[near_idx].dist

    for _ in 1:d
        next_cur = copy(cur)
        for a in 1:na
            cands = [u for u in graph.neighbors[cur[a]] if u != graph.n && !(u in visited[a])]
            isempty(cands) && return nothing
            best_u = cands[1]
            best_h = graph.shortest_paths[best_u, target[a]]
            for u in cands
                h = graph.shortest_paths[u, target[a]]
                if h < best_h
                    best_h = h; best_u = u
                end
            end
            next_cur[a] = best_u
        end

        trial_paths = [vcat(base_paths[a], seg[a], [next_cur[a]]) for a in 1:na]
        covs, dists = evaluate_full_paths(trial_paths, graph, landmarks, na)

        ok = true
        for a in 1:na
            # Support cap — supports must not outrun the primary (hexspline_cl.jl:889).
            if dists[a] > dists[primary] + UNC_FEAS_TOL
                ok = false; break
            end
            if !isempty(OBSTACLES)
                p0 = graph.landmarks[cur[a]]; p1 = graph.landmarks[next_cur[a]]
                if !segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), covs[a])
                    ok = false; break
                end
            end
        end
        ok || return nothing

        for a in 1:na
            push!(seg[a], next_cur[a])
            push!(visited[a], next_cur[a])
        end
        cur = next_cur
        last_covs = covs; last_dists = dists
    end
    return cur, seg, last_covs, last_dists
end

# Grows the joint tree from the start until a goal-adjacent, under-threshold,
# spline-clear primary node is found, or `max_iterations` is exhausted.
# Factored out of plan_clgbt (which handles I/O/figures/refinement) so the
# search itself is directly callable — see test_clgbt.jl.
# Returns (tree, solution_idx, iters); solution_idx == 0 means no solution.
function clgbt_grow_tree(graph::LandmarkGraph, landmarks::Vector{Landmark}, na::Int, primary::Int;
                         max_iterations::Int=CLGBT_MAX_ITERATIONS,
                         goal_bias::Float64=CLGBT_GOAL_BIAS,
                         besties_bias::Float64=CLGBT_BESTIES_BIAS,
                         extend_steps::Int=CLGBT_EXTEND_STEPS,
                         seed::Int=CLGBT_SEED,
                         verbose::Bool=true,
                         progress_every::Int=200)
    goal_mask = clgbt_goal_adjacent(graph)
    goal_nodes = findall(goal_mask)
    route_mask, _ = node_role_masks(graph)
    pool = [i for i in 1:(graph.n - 1) if route_mask[i]]

    root_paths = [[1] for _ in 1:na]
    root_covs, root_dists = evaluate_full_paths(root_paths, graph, landmarks, na)
    tree = ClgbtTreeNode[ClgbtTreeNode(0, fill(1, na), [Int[] for _ in 1:na], root_covs, root_dists)]
    (isempty(goal_nodes) || isempty(pool)) && return tree, 0, 0

    rng = Random.MersenneTwister(seed)
    solution_idx = 0
    iters = 0
    t0 = time()
    for iter in 1:max_iterations
        iters = iter
        if verbose && mod(iter, progress_every) == 0
            print("\r  [CL-GBT] iter $iter/$(max_iterations)  tree=$(length(tree))  " *
                  "elapsed=$(round(time() - t0, digits=1))s   ")
            flush(stdout)
        end

        target = clgbt_sample_target(rng, pool, goal_nodes, na, primary, goal_bias, besties_bias)
        near_idx = clgbt_nearest(tree, graph, target, na)
        d = rand(rng, 1:extend_steps)
        ext = clgbt_try_extend(tree, near_idx, target, d, graph, landmarks, na, primary)
        ext === nothing && continue

        agent_nodes, seg, covs, dists = ext
        push!(tree, ClgbtTreeNode(near_idx, agent_nodes, seg, covs, dists))
        new_idx = length(tree)

        if goal_mask[agent_nodes[primary]] &&
           unc_within_threshold(unc_radius(covs[primary]), UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL)
            candidate_paths = clgbt_reconstruct_paths(tree, new_idx, na)
            if seed_spline_clear(candidate_paths, graph, landmarks)
                solution_idx = new_idx
                break
            end
        end
    end
    verbose && println()   # release the status-bar line
    return tree, solution_idx, iters
end

function plan_clgbt(scenario, graph::LandmarkGraph, output_dir::String)
    landmarks = scenario.landmarks
    na = NUM_AGENTS
    primary = na

    function write_no_solution(iters::Int, tree_size::Int)
        open(joinpath(output_dir, "results.yaml"), "w") do io
            write(io, "main:\n")
            write(io, "  primary_length: null\n")
            write(io, "  primary_unc: null\n")
            write(io, "  clgbt_iterations: $(iters)\n")
            write(io, "  clgbt_tree_size: $(tree_size)\n")
            write(io, "  discrete_uncertainties: []\n")
            write(io, "  primary_disc_unc: null\n")
            write(io, "  continuous_primary_length: null\n")
            write(io, "  continuous_uncertainties: []\n")
        end
    end

    println("\n── CL-GBT PLANNING (sampling-based joint tree) ──")

    tree, solution_idx, iters = clgbt_grow_tree(graph, landmarks, na, primary)

    if solution_idx == 0
        println("  ✗ No feasible joint tree node found in $(iters) iterations (tree size $(length(tree))).")
        write_no_solution(iters, length(tree))
        return NamedTuple[]
    end

    paths = clgbt_reconstruct_paths(tree, solution_idx, na)
    seed_covs, seed_dists = evaluate_full_paths(paths, graph, landmarks, na)
    uncs = [unc_radius(seed_covs[a]) for a in 1:na]

    for a in 1:(na - 1)
        println("  Support $a: dist=$(round(seed_dists[a], digits=1))m, goal_unc=$(round(uncs[a], digits=4))m")
    end
    println("  Primary  : dist=$(round(seed_dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m " *
            "(threshold $(UNC_RADIUS_THRESHOLD)) — $(iters) iterations, tree size $(length(tree))")

    # ── Figure of the tree's solution branch, before refinement ──
    let
        agent_positions = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
        _, _, seed_comm, seed_lm = evaluate_joint_discrete(agent_positions, landmarks, na)
        plt = make_base_plot(landmarks, graph)
        for (ai, p) in enumerate(paths)
            is_prim = (ai == na)
            xs = [graph.landmarks[i].x for i in p]
            ys = [graph.landmarks[i].y for i in p]
            is_prim || (ys = ys .+ (SUPPORT_PLOT_OFFSET_M * ai))
            plot!(plt, xs, ys, label=(is_prim ? "primary" : "support $ai (offset)"),
                  color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                  linewidth=(is_prim ? 2.4 : 1.3), linestyle=(is_prim ? :solid : :dash))
            scatter!(plt, xs, ys, label=false,
                     color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                     markersize=3, markerstrokewidth=0)
        end
        title!(plt, "clgbt seed [len=$(round(seed_dists[end],digits=2)), unc=$(round(uncs[end],digits=3))]")
        xlabel!(plt, "x (m)"); ylabel!(plt, "y (m)")
        save_path_figures(plt, fig_path(output_dir, "fig_clgbt_discrete.png"), seed_comm, seed_lm)
    end

    # ── Refinement: hexspline_cl's optimizer, unchanged ──
    opt_len, opt_unc, _, opt_covs, opt_ctrls, refine =
        optimize_continuous(paths, graph, landmarks, na, output_dir;
                            cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main")

    write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
    if TRACK_COMM_EVENTS
        write_comm_csv(csv_path(output_dir, "comm_events.csv"), refine.comm_events)
    end
    if TRACK_LANDMARK_EVENTS
        write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), refine.landmark_events)
    end

    cont_uncs = (!isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:na)) ?
                [unc_radius(opt_covs[a][end]) for a in 1:na] : Float64[]

    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        fmtlist(v) = isempty(v) ? "[]" : "[" * join(fmt4.(v), ", ") * "]"
        fmtx(x) = isfinite(x) ? repr(x) : "null"
        write(io, "main:\n")
        write(io, "  primary_length: $(fmtx(opt_len))\n")
        write(io, "  primary_unc: $(fmtx(opt_unc))\n")
        write(io, "  clgbt_iterations: $(iters)\n")
        write(io, "  clgbt_tree_size: $(length(tree))\n")
        write(io, "  discrete_uncertainties: $(fmtlist(uncs))\n")
        write(io, "  primary_disc_unc: $(fmtx(uncs[end]))\n")
        write(io, "  continuous_primary_length: $(fmt3(opt_len))\n")
        write(io, "  continuous_uncertainties: $(fmtlist(cont_uncs))\n")
        write(io, "  refinement_status: $(refine.status)\n")
        write(io, "  refinement_min_slack: $(fmtx(refine.min_slack))\n")
    end

    return [(id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc)]
end
