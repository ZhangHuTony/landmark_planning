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
#     (their `besties_bias`, type 0 — "replicate one robot's target onto all
#     robots").
#   - ONE CONSTANT CONTROL per extension, held for a random number of steps —
#     their single control/controlDur per propagated edge. Each agent's leg is
#     therefore a straight segment.
#   - A whole extension is discarded, not partially kept, if any step along it
#     is infeasible — mirrors their `if (propCd == cd) addMotion` (a
#     propagation that stops short of its sampled duration is thrown away).
#
# The tree lives in CONTINUOUS R², as the original's does. It used to sample
# hex-graph node indices and steer along graph edges, which shared the joint
# A*'s discretization and so confounded the one variable this baseline exists
# to isolate. Nothing here touches the hex graph except to borrow the corridor
# extents it defines (so both arms search the same region) and the goal point.
#
# What is NOT ported: their Sigma/Lambda belief-propagation math, their
# linear Cprop/Ccl (proprioceptive + relative-position) measurement model, and
# their chi-squared chance-constraint machinery. Every uncertainty number in
# THIS file comes from src/covariance.jl and src/obstacles.jl — the exact same
# calls hexspline_cl/greedy/formation/sequential make (evaluate_joint_discrete,
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

# Arc length of one propagation step. This is NOT a free resolution knob: the
# covariance model fuses landmarks once per waypoint (src/covariance.jl,
# advance_to_arc!), so halving it doubles the number of landmark updates along
# the same geometry and hands clgbt a better sensor model than every other
# planner. Default = HEX_WIDTH_M, the hex centre-to-centre spacing, which is
# exactly the fusion cadence the graph planners get. See config/clgbt.yaml.
const CLGBT_STEP_M = Float64(get(CFG, "clgbt_step_m", HEX_WIDTH_M))
CLGBT_STEP_M == HEX_WIDTH_M || @warn "clgbt_step_m ($(CLGBT_STEP_M)) != hex_width_m " *
    "($(HEX_WIDTH_M)): clgbt's landmark-fusion cadence no longer matches the other " *
    "planners', so length/uncertainty comparisons against them are not like-for-like."

# How close the primary must get before the goal is pinned onto its path.
const CLGBT_GOAL_RADIUS_M = Float64(get(CFG, "clgbt_goal_radius_m", CLGBT_STEP_M))

# Heading change allowed per step, set so the seed satisfies the vehicle's
# curvature limit κ ≤ 1/min_turn_radius_m.
#
# The circular-arc relation Δθ = s/R is the WRONG bound here, and expensively so:
# the seed is a control POLYGON, and the clamped B-spline through it concentrates
# curvature at the corners rather than spreading it along an arc. At the defaults
# (100 m step, 40 m radius) s/R permits 143°, which measures max|κ| = 0.191 —
# 7.6× the 0.025 limit. The barrier then spends its entire budget clawing back
# feasibility instead of shortening, and finishes `recovery_failed`.
#
# So bisect on the real thing, with the same sampler the optimizer measures with.
# A sustained same-sign turn is the worst case (alternating turns measure the same
# to 3 significant figures, and marginally lower). At the defaults this yields
# ~85°/step: still freer than the hex lattice's 60° — which is the point of a
# continuous tree — but curvature-feasible by construction, so the seed reaches
# the barrier already inside its constraint set. Self-adjusts to clgbt_step_m,
# spline degree and min_turn_radius_m instead of hiding a magic angle.
function clgbt_curvature_turn_cap(step_m::Float64; corners::Int=9, bisections::Int=40)
    function max_kappa(θ::Float64)
        pts = Tuple{Float64,Float64}[(0.0, 0.0)]
        x = 0.0; y = 0.0; h = 0.0
        for _ in 1:corners
            h += θ; x += step_m * cos(h); y += step_m * sin(h)
            push!(pts, (x, y))
        end
        return maximum(abs, last(bspline_sample_path(pts)))
    end
    max_kappa(Float64(π)) <= MAX_CURVATURE && return Float64(π)
    lo = 0.0; hi = Float64(π)
    for _ in 1:bisections
        mid = 0.5 * (lo + hi)
        max_kappa(mid) <= MAX_CURVATURE ? (lo = mid) : (hi = mid)
    end
    return lo
end
const CLGBT_MAX_TURN = clgbt_curvature_turn_cap(CLGBT_STEP_M)

# Reject any branch whose primary arc exceeds this multiple of the straight-line
# start→goal distance. The hex version bounded branch depth implicitly (the
# `visited` set plus out-degree 3 strangled long wandering branches); in open R²
# nothing does, and cost per extension is linear in depth.
# ponytail: crude depth bound; it only exists to replace what `visited` did.
const CLGBT_MAX_PATH_FACTOR = Float64(get(CFG, "clgbt_max_path_factor", 3.0))

# One joint-tree node. `pos`/`heading` are where every agent stands, and which
# way it faces, after this node's edge; `seg` is the list of waypoints each agent
# stepped through to get here from the parent (so the full per-agent path is
# recoverable by walking parents and concatenating segments — see
# clgbt_reconstruct_paths). `cov`/`dist` are the EXACT joint-evaluated
# belief/arc-length at this node, never hand-propagated.
struct ClgbtTreeNode
    parent::Int                     # 0 = root
    pos::Vector{Tuple{Float64,Float64}}
    heading::Vector{Float64}
    seg::Vector{Vector{Tuple{Float64,Float64}}}
    cov::Vector{Matrix{Float64}}
    dist::Vector{Float64}
end

# Walk parent -> root, concatenating each edge's per-agent segment. Every agent
# starts at `start_xy`. Returns fresh vectors — callers mutate them (goal pin).
function clgbt_reconstruct_paths(tree::Vector{ClgbtTreeNode}, idx::Int, na::Int,
                                 start_xy::Tuple{Float64,Float64})
    chain = Int[]
    k = idx
    while k != 0
        push!(chain, k)
        k = tree[k].parent
    end
    reverse!(chain)
    paths = [Tuple{Float64,Float64}[start_xy] for _ in 1:na]
    for k in chain
        for a in 1:na
            append!(paths[a], tree[k].seg[a])
        end
    end
    return paths
end

# Final joint covariance and arc length for a set of continuous paths. The same
# call every other planner's uncertainty goes through; evaluate_joint_discrete
# already returns cumulative arc length, so no graph distance table is needed.
function clgbt_eval(paths::Vector{Vector{Tuple{Float64,Float64}}},
                    lms::Vector{Landmark}, na::Int)
    covs, arcs, _, _ = evaluate_joint_discrete(paths, lms, na)
    return [covs[a][end] for a in 1:na], [arcs[a][end] for a in 1:na], covs
end

# Joint sample: the primary's target is goal-biased; each support's target is
# besties-biased toward the primary's (else drawn independently). Targets are
# points in `box` = (xmin, xmax, ymin, ymax).
function clgbt_sample_target(rng, box::NTuple{4,Float64}, goal_xy::Tuple{Float64,Float64},
                             na::Int, primary::Int, goal_bias::Float64, besties_bias::Float64)
    xmin, xmax, ymin, ymax = box
    draw() = (xmin + rand(rng) * (xmax - xmin), ymin + rand(rng) * (ymax - ymin))
    target = Vector{Tuple{Float64,Float64}}(undef, na)
    target[primary] = rand(rng) < goal_bias ? goal_xy : draw()
    for a in 1:na
        a == primary && continue
        target[a] = rand(rng) < besties_bias ? target[primary] : draw()
    end
    return target
end

# Nearest existing tree node to a joint target: sum of per-agent Euclidean
# distance, linear scan. No KD-tree/GNAT (see config/clgbt.yaml) — this
# baseline stays small and reviewable rather than reproducing the original's
# NearestNeighborsKD_balance.h.
function clgbt_nearest(tree::Vector{ClgbtTreeNode}, target::Vector{Tuple{Float64,Float64}}, na::Int)
    best_idx = 1
    best_d = Inf
    for (k, node) in enumerate(tree)
        d = 0.0
        for a in 1:na
            p = node.pos[a]; q = target[a]
            d += hypot(p[1] - q[1], p[2] - q[2])
        end
        if d < best_d
            best_d = d
            best_idx = k
        end
    end
    return best_idx
end

# Steer-and-extend from tree node `near_idx` toward `target` by `d` steps.
#
# Each agent picks ONE heading for the whole extension — the bearing to its own
# target, clamped to CLGBT_MAX_TURN against its parent heading — and holds it,
# mirroring the original's single control per propagated edge. Every agent takes
# the SAME number of steps (their joint control/duration is shared), so support
# and primary path lengths never need reconciling.
#
# Holding a constant heading is also what makes the hex version's per-agent
# `visited` cycle guard unnecessary: a leg is a straight segment, so no extension
# can revisit its own ground, at any turn cap. It also removes the "agent reached
# its target" case entirely — the agent simply passes over it and keeps going.
# Parking there instead would be actively wrong: a repeated waypoint is a free
# landmark fusion at zero drift cost (src/covariance.jl:208-218 runs the Kalman
# update on a zero-length segment), so an RRT allowed to park would discover that
# standing still on a landmark drives covariance to the fusion floor for free.
#
# The whole joint step sequence is then checked at once (depth bound + support cap
# + chance-constrained obstacle clearance, the same calls greedy.jl makes). One
# infeasible agent fails the whole extension: nothing partial is kept, mirroring
# the original's `propCd == cd` all-or-nothing propagation.
# Returns `nothing` on failure, else (pos, heading, seg, cov, dist) for the new node.
function clgbt_try_extend(tree::Vector{ClgbtTreeNode}, near_idx::Int,
                          target::Vector{Tuple{Float64,Float64}}, d::Int,
                          landmarks::Vector{Landmark}, na::Int, primary::Int,
                          start_xy::Tuple{Float64,Float64}, max_arc::Float64)
    node = tree[near_idx]
    base = clgbt_reconstruct_paths(tree, near_idx, na, start_xy)

    heading = Vector{Float64}(undef, na)
    seg = [Vector{Tuple{Float64,Float64}}(undef, d) for _ in 1:na]
    pos = Vector{Tuple{Float64,Float64}}(undef, na)

    for a in 1:na
        px, py = node.pos[a]
        tx, ty = target[a]
        dx = tx - px; dy = ty - py
        if hypot(dx, dy) > 1e-9
            # Wrap the bearing error into (-π, π] before clamping, so a turn the
            # short way round is never mistaken for a near-full rotation.
            err = atan(dy, dx) - node.heading[a]
            heading[a] = node.heading[a] + clamp(atan(sin(err), cos(err)),
                                                 -CLGBT_MAX_TURN, CLGBT_MAX_TURN)
        else
            heading[a] = node.heading[a]     # target underfoot: hold course
        end
        cx = CLGBT_STEP_M * cos(heading[a]); cy = CLGBT_STEP_M * sin(heading[a])
        for k in 1:d
            px += cx; py += cy
            seg[a][k] = (px, py)
        end
        pos[a] = (px, py)
    end

    # ONE joint evaluation for the whole extension, not one per step:
    # evaluate_joint_discrete already returns the per-waypoint covariance, so
    # step k's obstacle check reads covs[a][nb+k] instead of re-evaluating a
    # truncated path. Comm checkpoints are keyed on arc length, so every prefix
    # checkpoint is identical either way.
    trial = [vcat(base[a], seg[a]) for a in 1:na]
    covs, arcs, _, _ = evaluate_joint_discrete(trial, landmarks, na)
    dists = [arcs[a][end] for a in 1:na]

    dists[primary] > max_arc && return nothing

    for a in 1:na
        # Support cap — supports must not outrun the primary (hexspline_cl.jl:889).
        # Vacuous while every agent takes the same number of equal steps (it was in
        # the hex version too, where every edge was exactly hex_w), but it costs one
        # comparison and goes live the moment any agent's arc is truncated. The cap
        # that actually binds is the spline-length test in seed_spline_clear.
        dists[a] > dists[primary] + UNC_FEAS_TOL && return nothing
        if !isempty(OBSTACLES)
            nb = length(base[a])
            for k in 1:d
                p0 = k == 1 ? node.pos[a] : seg[a][k-1]
                segment_obstacle_free(p0, seg[a][k], covs[a][nb + k]) || return nothing
            end
        end
    end
    return pos, heading, seg, [covs[a][end] for a in 1:na], dists
end

# Hand a continuous seed to the shared engine without re-quantizing it.
#
# optimize_continuous and seed_spline_clear key their seed off NODE POSITIONS
# (via seed_control_points) and nothing else in the graph, so the tree's solution
# is passed to them as a throwaway LandmarkGraph whose routing nodes ARE the RRT
# waypoints, plus the integer index paths into it. Node 1 is the start and the
# last node is the goal — build_hex_graph's contract, which is what lets
# seed_control_points pin the primary's final control point to the true goal.
#
# `backdrop` additionally carries the real graph's hex tiles and sensor landmarks
# across, because optimize_continuous draws its comparison figures with
# make_base_plot (hexspline_cl.jl:1406,1417) and that reads them off the graph it
# was handed. The search-time gate needs no figures, so it skips them.
#
# distance/orientation/shortest_paths are left EMPTY on purpose: nothing on this
# path reads them, and an empty matrix fails loudly if that ever changes.
function clgbt_seed_graph(paths::Vector{Vector{Tuple{Float64,Float64}}},
                          graph::LandmarkGraph; backdrop::Bool=false)
    null_cov = 1e-9 * Matrix{Float64}(I, 2, 2)
    lms = Landmark[]
    idx_paths = Vector{Vector{Int}}()
    for p in paths
        push!(idx_paths, collect((length(lms) + 1):(length(lms) + length(p))))
        for (x, y) in p
            push!(lms, Landmark(x, y, copy(null_cov)))
        end
    end

    n_sensor_start = 0
    if backdrop
        for (cx, cy) in route_tile_centers(graph)
            push!(lms, Landmark(cx, cy, copy(null_cov)))
        end
        n_sensor_start = length(lms)
        _, sensor_mask = node_role_masks(graph)
        for i in findall(sensor_mask)
            push!(lms, graph.landmarks[i])       # real Σ, so the ellipses still draw
        end
    end
    n_route_end = backdrop ? n_sensor_start : length(lms)

    push!(lms, Landmark(graph.landmarks[graph.n].x, graph.landmarks[graph.n].y, copy(null_cov)))
    n = length(lms)

    # node_role_masks classifies by degree: anything with an edge is a routing
    # node (drawn as a hex tile), anything isolated is a sensor landmark (drawn as
    # a black dot). Chain the routing nodes so they land on the right side of that
    # split; the chain has no other meaning and is never traversed.
    neighbors = [Int[] for _ in 1:n]
    for i in 1:(n_route_end - 1)
        push!(neighbors[i], i + 1)
    end
    push!(neighbors[n_route_end], n)             # goal stays a routing node
    empty_m = zeros(0, 0)
    return LandmarkGraph(n, lms, empty_m, empty_m, neighbors, empty_m), idx_paths
end

# Grows the joint tree from the start until a primary path reaching the goal
# under threshold, spline-clear, is found, or `max_iterations` is exhausted.
# Factored out of plan_clgbt (which handles I/O/figures/refinement) so the
# search itself is directly callable — see test_clgbt.jl.
# Returns (tree, solution_idx, iters, paths); solution_idx == 0 means no solution
# and `paths` is empty. `paths` carries the goal already pinned onto the primary.
function clgbt_grow_tree(graph::LandmarkGraph, landmarks::Vector{Landmark}, na::Int, primary::Int;
                         max_iterations::Int=CLGBT_MAX_ITERATIONS,
                         goal_bias::Float64=CLGBT_GOAL_BIAS,
                         besties_bias::Float64=CLGBT_BESTIES_BIAS,
                         extend_steps::Int=CLGBT_EXTEND_STEPS,
                         seed::Int=CLGBT_SEED,
                         verbose::Bool=true,
                         progress_every::Int=200)
    start_xy = (graph.landmarks[1].x, graph.landmarks[1].y)
    goal_xy  = (graph.landmarks[graph.n].x, graph.landmarks[graph.n].y)

    # Sample over exactly the region the hex corridor covers, so the two arms
    # search the same space. Only the TARGET is boxed, not the position, so a
    # path may run up to one step outside it — harmless, and it keeps the box
    # from acting as a wall the tree piles up against.
    centers = route_tile_centers(graph)
    xs = [c[1] for c in centers]; ys = [c[2] for c in centers]
    box = (minimum(xs), maximum(xs), minimum(ys), maximum(ys))
    max_arc = CLGBT_MAX_PATH_FACTOR * hypot(goal_xy[1] - start_xy[1], goal_xy[2] - start_xy[2])

    # Every agent starts on the exact start→goal bearing. The hex version snapped
    # this to the nearest 60° (src/graph.jl:96) purely because its headings were
    # discrete; the exact bearing is the faithful continuous analogue.
    θ0 = atan(goal_xy[2] - start_xy[2], goal_xy[1] - start_xy[1])

    root_paths = [Tuple{Float64,Float64}[start_xy] for _ in 1:na]
    root_covs, root_dists, _ = clgbt_eval(root_paths, landmarks, na)
    tree = ClgbtTreeNode[ClgbtTreeNode(0, fill(start_xy, na), fill(θ0, na),
                                       [Tuple{Float64,Float64}[] for _ in 1:na],
                                       root_covs, root_dists)]

    rng = Random.MersenneTwister(seed)
    solution_idx = 0
    solution_paths = Vector{Vector{Tuple{Float64,Float64}}}()
    iters = 0
    t0 = time()
    for iter in 1:max_iterations
        iters = iter
        if verbose && mod(iter, progress_every) == 0
            print("\r  [CL-GBT] iter $iter/$(max_iterations)  tree=$(length(tree))  " *
                  "elapsed=$(round(time() - t0, digits=1))s   ")
            flush(stdout)
        end

        target = clgbt_sample_target(rng, box, goal_xy, na, primary, goal_bias, besties_bias)
        near_idx = clgbt_nearest(tree, target, na)
        d = rand(rng, 1:extend_steps)
        ext = clgbt_try_extend(tree, near_idx, target, d, landmarks, na, primary, start_xy, max_arc)
        ext === nothing && continue

        pos, heading, seg, covs, dists = ext
        push!(tree, ClgbtTreeNode(near_idx, pos, heading, seg, covs, dists))
        new_idx = length(tree)

        hypot(pos[primary][1] - goal_xy[1], pos[primary][2] - goal_xy[2]) <= CLGBT_GOAL_RADIUS_M || continue

        # Pin the goal onto the primary's path and certify THAT trajectory. The hex
        # version tested the uncertainty at a goal-ADJACENT cell centre and then let
        # seed_control_points move that control point to the true goal, leaving the
        # last stretch of dead reckoning uncertified by any discrete gate
        # (run_constraint_sweep.jl:428-432 documents the same gap).
        cand = clgbt_reconstruct_paths(tree, new_idx, na, start_xy)
        if hypot(pos[primary][1] - goal_xy[1], pos[primary][2] - goal_xy[2]) < 1e-6 * CLGBT_STEP_M
            cand[primary][end] = goal_xy     # already there; appending would duplicate
        else
            push!(cand[primary], goal_xy)
        end
        cand_covs, _, _ = clgbt_eval(cand, landmarks, na)
        unc_within_threshold(unc_radius(cand_covs[primary]), UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) || continue

        sg, ipaths = clgbt_seed_graph(cand, graph)
        if seed_spline_clear(ipaths, sg, landmarks)
            solution_idx = new_idx
            solution_paths = cand
            break
        end
    end
    verbose && println()   # release the status-bar line
    return tree, solution_idx, iters, solution_paths
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

    println("\n── CL-GBT PLANNING (sampling-based joint tree, continuous R²) ──")

    tree, solution_idx, iters, paths = clgbt_grow_tree(graph, landmarks, na, primary)

    if solution_idx == 0
        println("  ✗ No feasible joint tree node found in $(iters) iterations (tree size $(length(tree))).")
        write_no_solution(iters, length(tree))
        return NamedTuple[]
    end

    seed_covs, seed_dists, _ = clgbt_eval(paths, landmarks, na)
    uncs = [unc_radius(seed_covs[a]) for a in 1:na]

    for a in 1:(na - 1)
        println("  Support $a: dist=$(round(seed_dists[a], digits=1))m, goal_unc=$(round(uncs[a], digits=4))m")
    end
    println("  Primary  : dist=$(round(seed_dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m " *
            "(threshold $(UNC_RADIUS_THRESHOLD)) — $(iters) iterations, tree size $(length(tree))")

    # ── Figure of the tree's solution branch, before refinement ──
    let
        _, _, seed_comm, seed_lm = evaluate_joint_discrete(paths, landmarks, na)
        plt = make_base_plot(landmarks, graph)
        for (ai, p) in enumerate(paths)
            is_prim = (ai == na)
            xs = [q[1] for q in p]
            ys = [q[2] for q in p]
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

    # ── Refinement: hexspline_cl's optimizer, unchanged, on the continuous seed ──
    seed_graph, seed_paths = clgbt_seed_graph(paths, graph; backdrop=true)
    opt_len, opt_unc, _, opt_covs, opt_ctrls, refine =
        optimize_continuous(seed_paths, seed_graph, landmarks, na, output_dir;
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
