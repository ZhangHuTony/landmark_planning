# ==========================================================================
# formation: baseline where the team flies as a rigid-ish formation.
#
#   Primary : a normal uncertainty-constrained A* over the hex graph — the same
#             search hexspline_cl runs, just not a joint one.
#   Supports: no freedom of their own. Each holds a fixed slot in the primary's
#             BODY frame (one hex out, at a 60° multiple off its heading), so
#             the whole team is determined by the primary's node alone.
#
# That is what makes this cheap: the supports are a deterministic function of
# the primary's (cell, heading), so the search stays single-agent-sized while
# still certifying the TEAM's fused uncertainty rather than the primary's alone.
#
# ── Why the formation deforms through turns ──────────────────────────────────
# A rigid offset cannot survive a turn. Measured on this graph, the support on
# the outside of a turn would have to cover 2× the primary's step (1.73× for the
# ±2 slots, and the inside support would have to stand still). That is real
# formation geometry — but this model has no speed: `apply_synchronized_
# propagation!` puts comm checkpoints every COMM_INTERVAL of ARC LENGTH and
# advances every agent to the same arc, so distance travelled *is* time. A
# support that covered 100 m extra is therefore rewound 100 m at the next
# checkpoint, landing ~141 m from the primary — and comm_weight(141 m) = 7.5e-8,
# far under COMM_WEIGHT_MIN, so the pair silently stops communicating for the
# rest of the run.
#
# So the support always steps exactly ONE hex, to whichever neighbouring cell
# sits closest to its ideal slot. Arc lengths stay identical to the primary's,
# every comm checkpoint fuses at full geometric weight, and the formation simply
# lags mid-turn and re-forms afterwards. Nothing in src/covariance.jl changes.
# ==========================================================================

# Body-frame slots, in 60° units off the primary's heading: +1 = 60° left,
# -1 = 60° right, ±2 = 120°, 3 = directly behind. Slot 0 is the primary's own
# cell, so it is not offerable. One slot per support, primary excluded.
const FORMATION_OFFSETS = [parse(Int, strip(s)) for s in split(String(CFG["formation_offsets"]), ",")]

# ── Cell lookup ──────────────────────────────────────────────────────────────
# The supports are placed by POSITION (a cell centre one hex away), but
# optimize_continuous consumes node indices, so positions have to map back to
# nodes. build_hex_graph does not export its (cell, heading) index, and six
# heading states share a centre anyway — any of them carries the right position.
#
# Keyed on position quantised to 1 m, with the 8 surrounding keys inserted too:
# cell centres are ≥86.6 m apart so a 1 m halo cannot collide, and it removes
# the quantisation-boundary risk of an exact key (a computed offset lands within
# ~1e-12 m of the true centre, which a bare round() can still push into the
# neighbouring bucket).
hexkey(x::Float64, y::Float64) = (round(Int, x), round(Int, y))

function build_hex_lookup(graph::LandmarkGraph)
    route_mask, _ = node_role_masks(graph)
    at = Dict{Tuple{Int,Int}, Int}()
    for i in 1:graph.n
        (route_mask[i] && i != graph.n) || continue   # skip the terminal goal marker
        kx, ky = hexkey(graph.landmarks[i].x, graph.landmarks[i].y)
        for dx in -1:1, dy in -1:1
            at[(kx + dx, ky + dy)] = i
        end
    end
    return at
end

# The six geometric neighbours of every route node, precomputed once — the
# support's step is chosen from these, so this must not be a scan inside the
# search loop. Neighbours of a hex centre are exactly one hex away at 60°
# intervals, independent of the axial parity bookkeeping in graph.jl.
function build_formation_neighbors(graph::LandmarkGraph, at::Dict{Tuple{Int,Int}, Int})
    nbrs = [Int[] for _ in 1:graph.n]
    for i in 1:graph.n
        x = graph.landmarks[i].x; y = graph.landmarks[i].y
        for j in 0:5
            u = get(at, hexkey(x + HEX_WIDTH_M * cos(j * pi / 3),
                               y + HEX_WIDTH_M * sin(j * pi / 3)), 0)
            u != 0 && u != i && push!(nbrs[i], u)
        end
    end
    # The start node carries start_pos, not its cell's centre, so if start_pos is
    # off-centre the ring above finds nothing. Fall back to the route nodes
    # nearest to each of the six directions. Runs once, at setup.
    if isempty(nbrs[1])
        x = graph.landmarks[1].x; y = graph.landmarks[1].y
        route_mask, _ = node_role_masks(graph)
        for j in 0:5
            tx = x + HEX_WIDTH_M * cos(j * pi / 3); ty = y + HEX_WIDTH_M * sin(j * pi / 3)
            best = 0; bestd = Inf
            for i in 2:(graph.n - 1)
                route_mask[i] || continue
                d = hypot(graph.landmarks[i].x - tx, graph.landmarks[i].y - ty)
                d < bestd && (bestd = d; best = i)
            end
            best != 0 && !(best in nbrs[1]) && push!(nbrs[1], best)
        end
    end
    return nbrs
end

# Heading index (0-5) the primary is travelling in on the edge from → to. The
# hex graph moves an agent along its NEW heading, so this recovers exactly the
# heading its destination node encodes — without needing build_hex_graph's index.
@inline function step_heading(graph::LandmarkGraph, from::Int, to::Int)
    dx = graph.landmarks[to].x - graph.landmarks[from].x
    dy = graph.landmarks[to].y - graph.landmarks[from].y
    return mod(round(Int, atan(dy, dx) / (pi / 3)), 6)
end

# One support's next node: the neighbour of `cur` closest to its ideal slot.
# Always exactly one hex, so arc length tracks the primary's exactly (see the
# header).
#
# The tie-break is load-bearing, not decoration. Whenever the ideal slot is more
# than one hex away (i.e. through every turn), the two neighbours flanking it are
# EXACTLY equidistant from it — 73.2 m each — and one of them is the primary's own
# cell. Breaking that tie toward the primary collapses the formation onto the
# leader and it never recovers, because from on top of the primary the tie recurs
# every step (measured: separation 0-15 m over a whole run, where 100 m was
# intended). So ties go to the candidate FARTHEST from the primary: that is what
# "hold the formation" means when the slot itself is unreachable. On straights the
# ideal slot IS reachable, distance-to-ideal is 0 for exactly one neighbour, and
# this row never runs. Distances are rounded first so a 1-ULP spread cannot
# masquerade as a preference and skip the row.
function formation_step(graph::LandmarkGraph, nbrs::Vector{Vector{Int}}, cur::Int,
                        ideal::Tuple{Float64,Float64}, prim::Tuple{Float64,Float64})
    best = 0; best_key = (Inf, Inf, typemax(Int))
    for u in nbrs[cur]
        ux = graph.landmarks[u].x; uy = graph.landmarks[u].y
        key = (round(hypot(ux - ideal[1], uy - ideal[2]), digits=6),
               -round(hypot(ux - prim[1], uy - prim[2]),  digits=6),   # then keep separation
               u)
        key < best_key && (best_key = key; best = u)
    end
    return best == 0 ? cur : best
end

# Nearest routing node to an arbitrary point. Only used to seat the supports in
# their opening slots, so a linear scan is fine — it runs (num_agents-1) times.
function nearest_route_node(graph::LandmarkGraph, x::Float64, y::Float64)
    route_mask, _ = node_role_masks(graph)
    best = 1; bestd = Inf
    for i in 1:(graph.n - 1)
        route_mask[i] || continue
        d = hypot(graph.landmarks[i].x - x, graph.landmarks[i].y - y)
        d < bestd && (bestd = d; best = i)
    end
    return best
end

# Where support `a` wants to be once the primary sits at `pnode` heading `h`.
@inline function formation_slot(graph::LandmarkGraph, pnode::Int, h::Int, k::Int)
    θ = (h + k) * pi / 3
    return (graph.landmarks[pnode].x + HEX_WIDTH_M * cos(θ),
            graph.landmarks[pnode].y + HEX_WIDTH_M * sin(θ))
end

# Advance the whole formation one step: primary `v` → `u`, supports follow.
function formation_advance(graph::LandmarkGraph, nbrs::Vector{Vector{Int}},
                           v::Int, u::Int, snodes::Vector{Int}, offsets::Vector{Int})
    h = step_heading(graph, v, u)
    prim = (graph.landmarks[u].x, graph.landmarks[u].y)
    return [formation_step(graph, nbrs, snodes[a],
                           formation_slot(graph, u, h, offsets[a]), prim)
            for a in eachindex(snodes)]
end

# ==========================================================================
# Formation A*: single-agent-sized search that certifies the TEAM's uncertainty
# ==========================================================================
# Same shape as joint_astar's na==1 fast path (weighted f, node-level Pareto
# frontier under sound covariance dominance, obstacle filter on every edge,
# exact re-evaluation at goal pops) — the difference is that each state carries
# one covariance per agent and the supports ride along deterministically. The
# frontier is keyed on the whole formation, not just the primary's node, because
# two routes reaching the same cell can leave the supports in different places.
struct FormState
    node    :: Int
    snodes  :: Vector{Int}
    dist    :: Float64
    covs    :: Vector{Matrix{Float64}}   # primary last, matching every other stage
    parent  :: Int
    visited :: BitVector
end

function formation_astar(graph::LandmarkGraph, lms::Vector{Landmark},
                         unc_threshold::Float64, na::Int,
                         offsets::Vector{Int}, nbrs::Vector{Vector{Int}})
    goal = graph.n
    is_goal_cell = falses(graph.n)
    for v in 1:(graph.n - 1)
        goal in graph.neighbors[v] && (is_goal_cell[v] = true)
    end
    w_astar = 1.0 + PRIMARY_EPSILON
    ns = na - 1

    # Supports launch ALREADY IN SLOT, not on top of the primary. Forming up from
    # co-located costs more than the two steps it looks like: from the primary's own
    # cell the ideal slot is two hexes off, which is exactly the tied case in
    # formation_step, so a degenerate start makes the formation fight to separate on
    # every subsequent step instead of just holding. The initial heading is the one
    # build_hex_graph gave the start node, so slot geometry matches the first move.
    h0 = nearest_heading_to_goal((graph.landmarks[1].x, graph.landmarks[1].y),
                                 (graph.landmarks[goal].x, graph.landmarks[goal].y))
    snodes0 = [nearest_route_node(graph, formation_slot(graph, 1, h0, offsets[a])...)
               for a in 1:ns]

    init_visited = falses(graph.n); init_visited[1] = true
    states = FormState[FormState(1, snodes0, 0.0,
                                 [copy(lms[1].cov) for _ in 1:na], -1, init_visited)]

    pq = PriorityQueue{Int, Tuple{Float64, Float64}}()
    enqueue!(pq, 1, (w_astar * graph.shortest_paths[1, goal], unc_radius(states[1].covs[na])))

    frontier = Dict{Tuple{Int, Tuple{Vararg{Int}}}, Vector{Tuple{Float64, Matrix{Float64}}}}()
    frontier[(1, Tuple(states[1].snodes))] = [(0.0, copy(states[1].covs[na]))]

    # Anytime incumbent, consulted only if the budget runs out with nothing under
    # the threshold — the barrier can still trade length for uncertainty from it.
    best_paths = Vector{Vector{Int}}(); best_dist = 0.0; best_unc = Inf
    iter = 0; t0 = time()

    rebuild(si) = begin
        prim = Int[]; sups = [Int[] for _ in 1:ns]
        p = si
        while p != -1
            pushfirst!(prim, states[p].node)
            for a in 1:ns; pushfirst!(sups[a], states[p].snodes[a]); end
            p = states[p].parent
        end
        vcat(sups, [prim])
    end

    while !isempty(pq) && iter < ASTAR_ITERATION_LIMIT
        si = dequeue!(pq)
        S = states[si]
        iter += 1
        astar_progress(iter, ASTAR_ITERATION_LIMIT, t0)

        if is_goal_cell[S.node]
            paths = rebuild(si)
            exact_covs, exact_dists = evaluate_full_paths(paths, graph, lms, na)
            exact_unc = unc_radius(exact_covs[na])
            if unc_within_threshold(exact_unc, unc_threshold, UNC_FEAS_TOL)
                if !seed_spline_clear(paths, graph, lms)
                    continue    # spline breaches an obstacle / the support cap
                end
                println()
                println("  ✓ FEASIBLE FORMATION at iter $(iter): dist=$(round(exact_dists[na], digits=3)), unc=$(round(exact_unc, digits=4))")
                return paths, exact_dists[na], exact_unc, iter
            end
            if exact_unc < best_unc && seed_spline_clear(paths, graph, lms)
                best_paths = paths; best_dist = exact_dists[na]; best_unc = exact_unc
            end
            continue
        end

        for u in graph.neighbors[S.node]
            S.visited[u] && continue
            u == goal && continue                # terminal marker, reached via is_goal_cell

            snew  = formation_advance(graph, nbrs, S.node, u, S.snodes, offsets)
            nodes = vcat(snew, [u])
            nd    = S.dist + graph.distance[S.node, u]

            ncovs = Vector{Matrix{Float64}}(undef, na)
            for a in 1:ns
                ncovs[a] = edge_cov_continuous(S.snodes[a], snew[a], graph, lms, S.covs[a])
            end
            ncovs[na] = edge_cov_continuous(S.node, u, graph, lms, S.covs[na])

            # Every agent steps exactly one hex, so all arc distances are equal and
            # apply_joint_step_comms' synchronisation gate is satisfied by construction
            # — which is the whole reason the formation deforms instead of stretching.
            ncovs = apply_joint_step_comms(ncovs, nodes, fill(nd, na), graph)

            if !isempty(OBSTACLES)
                blocked = false
                for a in 1:ns
                    p0 = graph.landmarks[S.snodes[a]]; p1 = graph.landmarks[snew[a]]
                    if !segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncovs[a])
                        blocked = true; break
                    end
                end
                if !blocked
                    p0 = graph.landmarks[S.node]; p1 = graph.landmarks[u]
                    blocked = !segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncovs[na])
                end
                blocked && continue
            end

            key = (u, Tuple(snew))
            labels = get(frontier, key, Tuple{Float64, Matrix{Float64}}[])
            any(od <= nd + 1e-9 && cov_dominates(ocov, ncovs[na]) for (od, ocov) in labels) && continue
            kept = [(od, ocov) for (od, ocov) in labels
                    if !(nd <= od + 1e-9 && cov_dominates(ncovs[na], ocov))]
            push!(kept, (nd, copy(ncovs[na])))
            frontier[key] = kept

            nvis = copy(S.visited); nvis[u] = true
            push!(states, FormState(u, snew, nd, ncovs, si, nvis))
            enqueue!(pq, length(states),
                     (nd + w_astar * graph.shortest_paths[u, goal], unc_radius(ncovs[na])))
        end
    end

    println()
    if isempty(best_paths)
        println("  [Formation A*] No feasible formation found ($(iter) iterations)")
        return Vector{Vector{Int}}(), 0.0, Inf, iter
    end
    println("  [Formation A*] Budget exhausted with no formation under threshold; " *
            "handing best incumbent to the optimizer: dist=$(round(best_dist, digits=3)), " *
            "unc=$(round(best_unc, digits=4)) > $(round(unc_threshold, digits=4))")
    return best_paths, best_dist, best_unc, iter
end

function plan_formation(scenario, graph::LandmarkGraph, output_dir::String)
    landmarks = scenario.landmarks
    ns = NUM_AGENTS - 1

    function write_no_solution(iters::Int)
        open(joinpath(output_dir, "results.yaml"), "w") do io
            write(io, "main:\n")
            write(io, "  primary_length: null\n  primary_unc: null\n")
            write(io, "  astar_iterations: $(iters)\n")
            write(io, "  discrete_uncertainties: []\n  primary_disc_unc: null\n")
            write(io, "  continuous_primary_length: null\n  continuous_uncertainties: []\n")
        end
    end

    length(FORMATION_OFFSETS) >= ns || error(
        "formation_offsets has $(length(FORMATION_OFFSETS)) slot(s) but num_agents=$(NUM_AGENTS) " *
        "needs $(ns). Add slots to config/formation.yaml (60° units; 0 is the primary's own cell).")

    # With ONE support there is a single slot to pick, and picking it wrong is the
    # planner's worst failure mode: a support held on the side away from every
    # landmark contributes nothing but a halved comm weight. So search both lateral
    # slots and keep the better — one extra search, and it removes the coin-flip.
    # With 2+ supports the roster is a config decision: the slots interact (they
    # share comm range with each other, not just the primary) and the combinatorics
    # are 5·4·… , so config/formation.yaml is taken as given rather than searched.
    rosters = ns == 1 ? [[1], [-1]] : [FORMATION_OFFSETS[1:ns]]

    println("\n── FORMATION PLANNING ──")
    println("  Supports step one hex per move, so arc length — and therefore comm timing — tracks the primary exactly.")
    println(ns == 1 ? "  One support: searching both lateral slots (+1 = 60° left, -1 = 60° right) and keeping the better." :
                      "  Body-frame slots (60° units off the primary's heading): $(rosters[1])")

    at   = build_hex_lookup(graph)
    nbrs = build_formation_neighbors(graph, at)

    # Rank on the DISCRETE seed, then refine only the winner: refinement is the
    # expensive stage and it writes the run's figures, so doing it per candidate
    # would cost a full barrier solve per slot and leave the losers' figures
    # overwritten anyway. Feasible beats infeasible; among feasible the shorter
    # primary wins (that is the objective the barrier minimises); among infeasible
    # the lower uncertainty wins (closest to recoverable).
    offsets = Int[]; paths = Vector{Vector{Int}}(); iters = 0
    best_rank = (typemax(Int), Inf)
    for roster in rosters
        p, d, u, it = formation_astar(graph, landmarks, UNC_RADIUS_THRESHOLD,
                                      NUM_AGENTS, roster, nbrs)
        iters += it        # total search effort across every slot tried
        isempty(p) && continue
        ok = unc_within_threshold(u, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL)
        rank = (ok ? 0 : 1, ok ? d : u)
        length(rosters) > 1 && println("  Slot $(roster): dist=$(round(d, digits=1)), " *
                                       "unc=$(round(u, digits=4)) $(ok ? "✓" : "✗")")
        if rank < best_rank
            best_rank = rank; paths = p; offsets = roster
        end
    end
    if isempty(paths)
        write_no_solution(iters)
        return NamedTuple[]
    end
    length(rosters) > 1 && println("  → Chose slot $(offsets)")

    seed_covs, seed_dists = evaluate_full_paths(paths, graph, landmarks, NUM_AGENTS)
    uncs = [unc_radius(seed_covs[a]) for a in 1:NUM_AGENTS]
    for a in 1:ns
        println("  Support $a (slot $(offsets[a])): dist=$(round(seed_dists[a], digits=1))m, goal_unc=$(round(uncs[a], digits=4))m")
    end
    println("  Primary : dist=$(round(seed_dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m " *
            "(threshold $(UNC_RADIUS_THRESHOLD) " *
            "$(unc_within_threshold(uncs[end], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓" : "✗"))")

    let
        agent_positions = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
        _, _, seed_comm, seed_lm = evaluate_joint_discrete(agent_positions, landmarks, NUM_AGENTS)
        plt = make_base_plot(landmarks, graph)
        for (ai, p) in enumerate(paths)
            is_prim = (ai == NUM_AGENTS)
            xs = [graph.landmarks[i].x for i in p]; ys = [graph.landmarks[i].y for i in p]
            plot!(plt, xs, ys, label=(is_prim ? "primary" : "support $ai (slot $(offsets[ai]))"),
                  color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                  linewidth=(is_prim ? 2.4 : 1.3), linestyle=(is_prim ? :solid : :dash))
            scatter!(plt, xs, ys, label=false,
                     color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                     markersize=3, markerstrokewidth=0)
        end
        title!(plt, "formation seed [len=$(round(seed_dists[end],digits=2)), unc=$(round(uncs[end],digits=3))]")
        xlabel!(plt, "x (m)"); ylabel!(plt, "y (m)")
        save_path_figures(plt, fig_path(output_dir, "fig_formation_discrete.png"), seed_comm, seed_lm)
    end

    opt_len, opt_unc, _, opt_covs, opt_ctrls, refine =
        optimize_continuous(paths, graph, landmarks, NUM_AGENTS, output_dir;
                            cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main")

    write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
    TRACK_COMM_EVENTS     && write_comm_csv(csv_path(output_dir, "comm_events.csv"), refine.comm_events)
    TRACK_LANDMARK_EVENTS && write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), refine.landmark_events)

    cont_uncs = (!isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:NUM_AGENTS)) ?
                [unc_radius(opt_covs[a][end]) for a in 1:NUM_AGENTS] : Float64[]

    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        fmtlist(v) = isempty(v) ? "[]" : "[" * join(fmt4.(v), ", ") * "]"
        fmtx(x) = isfinite(x) ? repr(x) : "null"   # full precision for machine-read keys
        write(io, "main:\n")
        write(io, "  primary_length: $(fmtx(opt_len))\n")
        write(io, "  primary_unc: $(fmtx(opt_unc))\n")
        write(io, "  astar_iterations: $(iters)\n")
        write(io, "  discrete_uncertainties: $(fmtlist(uncs))\n")
        write(io, "  primary_disc_unc: $(fmtx(uncs[end]))\n")
        write(io, "  continuous_primary_length: $(fmt3(opt_len))\n")
        write(io, "  continuous_uncertainties: $(fmtlist(cont_uncs))\n")
        write(io, "  refinement_status: $(refine.status)\n")
        write(io, "  refinement_min_slack: $(fmtx(refine.min_slack))\n")
        write(io, "  formation_offsets: [$(join(offsets, ", "))]\n")
    end

    return [(id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc)]
end
