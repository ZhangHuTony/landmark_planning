# ==========================================================================
# formation: baseline that separates routing from support placement entirely.
#
#   Primary  : its own weighted A* over (cell, belief) — formation_primary_astar
#              below. Routing is the ordinary single-agent search, but a goal
#              candidate is ACCEPTED on the uncertainty the primary has once its
#              supports are attached, never on what it can certify alone.
#   Supports : attached by walking a candidate primary path once, INSIDE the
#              acceptance test. Each holds a slot in the primary's BODY frame
#              (one hex out, at a 60° multiple off its heading) and starts on
#              the primary's own node.
#
# There is still no joint state space: the supports follow a fixed rule rather
# than being searched, so this stays the cheapest planner here — one single-agent
# A*, plus a linear placement walk per goal candidate. It is a baseline for
# "route with help in mind, but place that help by a fixed formation rule".
#
# Holding the primary to the threshold ALONE (which is what calling joint_astar
# with na=1 did) is not the same policy and is strictly more pessimistic than
# even `sequential`'s parked-helper leg 1: at na=1 the comm-fusion loop in
# apply_synchronized_propagation! is empty, so the supports contribute nothing
# at all to the belief being searched over. Every CI fusion is det-non-increasing
# by construction (ω=1 leaves the agent's own covariance untouched and is always
# in ci_omega_det's feasible set), so escorted uncertainty is never worse than
# solo — the old bound demanded the primary certify a route by itself and then
# handed it escorts its certificate never needed. It bought detour length for
# landmark coverage the formation supplies free, and at tight thresholds it was
# often unreachable at all, draining the budget and returning the
# minimum-solo-uncertainty incumbent, i.e. a long landmark detour.
#
# ── Placement policy, per support per step ───────────────────────────────────
# Pick a TARGET, then move exactly one hex toward it.
#   target = the slot, whenever the slot is legal (on the graph and clear of
#            every obstacle at the support's belief);
#          = the primary's own cell when the slot is IN VIOLATION. There is
#            always such a cell, which is why the primary can plan alone without
#            ever stranding a support.
#   step   = the target itself when it is reachable within the ±60° turn limit
#            (the steady state on any straight stretch), otherwise the admissible
#            cell closest to it. The latter is what a turn looks like: the outside
#            slot is two hexes away mid-turn, so the support lags one step and
#            re-forms once the primary straightens.
#
# ── Why steps are one hex ────────────────────────────────────────────────────
# `apply_synchronized_propagation!` puts comm checkpoints every COMM_INTERVAL of
# ARC LENGTH and advances every agent to the same arc, so in this model distance
# travelled *is* time. A support that covered 100 m extra gets rewound 100 m at
# the next checkpoint, landing ~141 m from the primary — and comm_weight(141 m)
# = 7.5e-8, under COMM_WEIGHT_MIN, so the pair silently stops communicating for
# the rest of the run. An earlier version let the violation fallback jump
# straight to the primary's cell, up to 173 m in one step; on obstacle-dense
# sweep scenarios that accumulated 200 m and 530 m of arc drift and failed the
# runs. Closing on the same target one hex at a time arrives just as surely, a
# step later, and desynchronises nothing. `formation_arc_drift` reports any
# residual.
#
# The ±60° turn limit is the same vehicle model the hex graph imposes on the
# primary by construction. Without it a support may pick any of the six
# neighbours, including a 120° pivot — measured at κ=0.0416 against
# MAX_CURVATURE=0.025, a hard constraint row, which failed whole runs in
# refinement on formations that had already been certified.
# ==========================================================================

# Body-frame slots, in 60° units off the primary's heading: +1 = 60° left,
# -1 = 60° right, ±2 = 120°, 3 = directly behind. Slot 0 is the primary's own
# cell, so it is not offerable. One slot per support, in agent order.
# Defaulted rather than required: with one support the planner picks the slot
# itself, so this key is dead for the common 2-agent case — and a hard CFG lookup
# made the planner un-runnable against any config folder that had not been told
# about it yet (run_constraint_sweep.jl copies config/mc/, a separate snapshot:
# missing file ⇒ KeyError at include time, before a single run starts).
const FORMATION_OFFSETS = [parse(Int, strip(s))
                           for s in split(String(get(CFG, "formation_offsets", "1,-1,2,-2,3")), ",")]

# ── Cell lookup ──────────────────────────────────────────────────────────────
# Supports are placed by POSITION (a cell centre one hex away) but
# optimize_continuous consumes node indices, so positions map back to nodes.
# build_hex_graph does not export its (cell, heading) index, and six heading
# states share a centre anyway — any of them carries the right position.
#
# Keyed on position quantised to 1 m, with the 8 surrounding keys inserted too:
# cell centres are ≥86.6 m apart so a 1 m halo cannot collide, and it removes the
# quantisation-boundary risk of an exact key (a computed offset lands within
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

# The six geometric neighbours of every route node, precomputed once. Neighbours
# of a hex centre are exactly one hex away at 60° intervals, independent of the
# axial parity bookkeeping in graph.jl.
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
    # off-centre the ring above finds nothing there. Fall back to the route nodes
    # nearest each of the six directions. Runs once, at setup.
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

# Heading index (0-5) of the step from → to. The hex graph moves an agent along
# its NEW heading, so for a primary edge this recovers exactly the heading its
# destination node encodes — without needing build_hex_graph's internal index.
@inline function step_heading(graph::LandmarkGraph, from::Int, to::Int)
    dx = graph.landmarks[to].x - graph.landmarks[from].x
    dy = graph.landmarks[to].y - graph.landmarks[from].y
    return mod(round(Int, atan(dy, dx) / (pi / 3)), 6)
end

# Where support `a` wants to be once the primary sits at `pnode` heading `h`.
@inline function formation_slot(graph::LandmarkGraph, pnode::Int, h::Int, k::Int)
    θ = (h + k) * pi / 3
    return (graph.landmarks[pnode].x + HEX_WIDTH_M * cos(θ),
            graph.landmarks[pnode].y + HEX_WIDTH_M * sin(θ))
end

@inline turn_ok(d::Int, head::Int) = abs(mod(d - head + 3, 6) - 3) <= 1

# Is stepping cur → u legal for a support: within ±60° of its heading, and clear
# of every obstacle at the belief it would carry on arrival?
function support_move_ok(graph::LandmarkGraph, lms::Vector{Landmark}, cur::Int, u::Int,
                         head::Int, cov::Matrix{Float64})
    turn_ok(step_heading(graph, cur, u), head) || return false
    isempty(OBSTACLES) && return true
    p0 = graph.landmarks[cur]; p1 = graph.landmarks[u]
    ncov = edge_cov_continuous(cur, u, graph, lms, cov)
    return segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncov)
end

# Is the slot a place the support may legally stand? Returns its node, or 0 when
# the slot is IN VIOLATION — off the corridor graph, or not clear of every
# obstacle at the belief the support carries. This is a position test, distinct
# from "can I reach it this step": a slot can be perfectly valid and still be two
# hexes away mid-turn, which is deformation, not violation.
function slot_node_if_clear(graph::LandmarkGraph, at::Dict{Tuple{Int,Int}, Int},
                            slot::Tuple{Float64,Float64}, cov::Matrix{Float64})
    node = get(at, hexkey(slot[1], slot[2]), 0)
    node == 0 && return 0
    for obs in OBSTACLES
        clears_obstacle(slot[1], slot[2], cov, obs, OBSTACLE_Z, AGENT_RADIUS) || return 0
    end
    return node
end

# One support's next cell. Returns (node, heading).
#
# Target selection is the whole policy: hold the slot when the slot is legal,
# fall back to the primary's own cell when it is in violation. Then move ONE hex
# toward whichever target that is — taking it directly if it is reachable this
# step, deforming toward it if it is not (which is what every turn looks like).
#
# The "one hex" part is not a detail. Jumping straight to a target 173 m away
# breaks the arc-length lockstep that keeps comm checkpoints paired, and measured
# on obstacle-dense sweep scenarios that produced 200 m and 530 m of arc drift and
# failed the runs outright. Approaching the same target at one hex per step
# reaches it just as surely, a step later, without desynchronising anything.
function choose_support_cell(graph::LandmarkGraph, lms::Vector{Landmark},
                             nbrs::Vector{Vector{Int}}, at::Dict{Tuple{Int,Int}, Int},
                             cur::Int, head::Int, cov::Matrix{Float64},
                             slot::Tuple{Float64,Float64}, pnode::Int)
    slot_node = slot_node_if_clear(graph, at, slot, cov)
    hold   = slot_node != 0                      # false ⇒ slot in violation, head for the primary
    tnode  = hold ? slot_node : pnode
    target = hold ? slot : (graph.landmarks[pnode].x, graph.landmarks[pnode].y)

    # Reachable directly? Then take it — this is the "prioritize the slot when it
    # is available" case, and the steady state on any straight stretch.
    # ADJACENCY IS PART OF REACHABLE. The slot is measured from the PRIMARY's new
    # cell, so it can sit two hexes from where the support currently stands, and a
    # turn check alone happily accepts that: step_heading is defined for any pair
    # of nodes, so a 200 m hop reads as a legal 0° turn. That is exactly how the
    # support ended up making 200 m steps and accruing arc drift.
    if tnode != cur && tnode in nbrs[cur] && support_move_ok(graph, lms, cur, tnode, head, cov)
        return tnode, step_heading(graph, cur, tnode)
    end

    # Otherwise close on the target one hex at a time.
    # The secondary row is load-bearing while holding a slot: mid-turn the slot is
    # two hexes off and the two neighbours flanking it are EXACTLY equidistant from
    # it (73.2 m each), one of which is the primary's own cell. Breaking that tie
    # toward the primary collapses the formation onto the leader and it never
    # recovers, because from on top of the primary the tie recurs every step
    # (measured: 0-15 m separation for a whole run where 100 m was intended). So
    # while holding, ties go to the candidate FARTHEST from the primary; while
    # falling back, they go to the nearest, since closing on the leader is the
    # point. Rounded first so a 1-ULP spread cannot pre-empt the row.
    px = graph.landmarks[pnode].x; py = graph.landmarks[pnode].y
    pick(relax_obstacles::Bool, relax_turn::Bool) = begin
        best = 0; best_key = (Inf, Inf, typemax(Int))
        for u in nbrs[cur]
            u == cur && continue
            if !relax_turn
                turn_ok(step_heading(graph, cur, u), head) || continue
            end
            if !relax_obstacles && !support_move_ok(graph, lms, cur, u, head, cov)
                continue
            end
            ux = graph.landmarks[u].x; uy = graph.landmarks[u].y
            dp = round(hypot(ux - px, uy - py), digits=6)
            key = (round(hypot(ux - target[1], uy - target[2]), digits=6),
                   hold ? -dp : dp,
                   u)
            key < best_key && (best_key = key; best = u)
        end
        best
    end

    # Last resorts, in order of what hurts least: an obstacle-blocked step is
    # re-checked and reported by the continuous stage, and a hard turn shows up on
    # the curvature row — but standing still desynchronises the comm schedule for
    # the whole remaining route, which nothing downstream can detect or repair.
    for (ro, rt) in ((false, false), (true, false), (true, true))
        b = pick(ro, rt)
        b != 0 && return b, step_heading(graph, cur, b)
    end
    return cur, head
end

# Walk the finished primary path once, placing every support step by step.
# Covariance is threaded here (dead reckoning + landmark fusion, no inter-agent
# fusion) purely so the obstacle test above sees a real belief rather than a
# fixed prior; the authoritative numbers come from evaluate_full_paths after.
function place_supports(graph::LandmarkGraph, lms::Vector{Landmark},
                        nbrs::Vector{Vector{Int}}, at::Dict{Tuple{Int,Int}, Int},
                        primary_path::Vector{Int}, offsets::Vector{Int})
    ns = length(offsets)
    h0 = nearest_heading_to_goal((graph.landmarks[1].x, graph.landmarks[1].y),
                                 (graph.landmarks[graph.n].x, graph.landmarks[graph.n].y))
    paths = [Int[1] for _ in 1:ns]          # supports start on the primary's node
    heads = fill(h0, ns)
    covs  = [copy(lms[1].cov) for _ in 1:ns]

    for k in 2:length(primary_path)
        v = primary_path[k-1]; u = primary_path[k]
        h = step_heading(graph, v, u)
        for a in 1:ns
            cur = paths[a][end]
            nxt, nh = choose_support_cell(graph, lms, nbrs, at, cur, heads[a], covs[a],
                                          formation_slot(graph, u, h, offsets[a]), u)
            covs[a] = edge_cov_continuous(cur, nxt, graph, lms, covs[a])
            push!(paths[a], nxt); heads[a] = nh
        end
    end
    return paths
end

# Attach supports to a candidate primary path and score the result. This is the
# acceptance test's whole content: place, evaluate ESCORTED, gate the spline.
#
# Returns (paths, offsets, unc, clear) for the best slot roster, ranked
# (spline-clear, then escorted primary uncertainty). Length is deliberately NOT a
# ranking row: every roster here reuses the SAME primary path, so primary length
# ties exactly and the row would only be reading float noise.
#
# `clear` is seed_spline_clear on the FULL roster, which is what actually ships.
# The supports' own moves are filtered as polylines between hex centres
# (support_move_ok), but the refined B-spline cuts those corners off and pads
# supports to the primary's waypoint count, so a polyline-clean formation can
# still breach — and unlike an over-threshold seed, which the barrier can trade
# length for, an obstacle breach or a support-cap violation is topological and no
# local perturbation of the spline repairs it.
function formation_attach(graph::LandmarkGraph, lms::Vector{Landmark},
                          nbrs::Vector{Vector{Int}}, at::Dict{Tuple{Int,Int}, Int},
                          primary_path::Vector{Int}, rosters::Vector{Vector{Int}})
    best_paths = Vector{Vector{Int}}(); best_offs = Int[]
    best_unc = Inf; best_clear = false; best_key = nothing
    for roster in rosters
        cand = vcat(place_supports(graph, lms, nbrs, at, primary_path, roster), [primary_path])
        covs, _ = evaluate_full_paths(cand, graph, lms, NUM_AGENTS)
        u = unc_radius(covs[NUM_AGENTS])
        clear = seed_spline_clear(cand, graph, lms)
        key = (clear ? 0 : 1, u)
        if best_key === nothing || key < best_key
            best_key = key
            best_paths = cand; best_offs = roster; best_unc = u; best_clear = clear
        end
    end
    return best_paths, best_offs, best_unc, best_clear
end

# ==========================================================================
# The primary's route, searched against the ESCORTED belief
# ==========================================================================
# Weighted A* over (cell, belief): the same f = g + (1+ε)·h on the Floyd-Warshall
# heuristic, the same node-level PSD-dominance frontier and the same anytime
# incumbent as joint_astar's single-agent branch. The one difference is the
# acceptance test — at every goal candidate the supports are attached and the
# PRIMARY'S ESCORTED uncertainty is what decides, so the search returns the
# shortest route the formation can actually escort rather than the shortest the
# primary could have certified unaided.
#
# Why this is a separate search and not a flag on joint_astar: the supports do
# not exist until a WHOLE primary path exists (place_supports walks a finished
# route), so the escorted belief cannot be threaded through the frontier the way
# sequential's joint-prefix evaluation is. It is available only at a goal pop.
#
# The frontier therefore still prunes on the SOLO (distance, covariance) pair,
# because that is all a partial path has. Two prefixes that tie there can escort
# differently, so pruning here is a heuristic rather than the sound dominance the
# single-agent branch enjoys — the same character of approximation as
# sequential's beam. There is deliberately NO uncertainty gate on popped or
# expanded states: gating on solo uncertainty is precisely the thing this search
# exists to stop doing, so PRUNE_BY_PRIMARY_UNCERTAINTY is not consulted (it is
# false in the shipped configs anyway). A* pops in f order, so the first
# candidate that clears the escorted bound is the shortest one that does.
#
# Returns (paths, offsets, unc, iters); `paths` empty means no route at all.
function formation_primary_astar(graph::LandmarkGraph, lms::Vector{Landmark},
                                 nbrs::Vector{Vector{Int}}, at::Dict{Tuple{Int,Int}, Int},
                                 rosters::Vector{Vector{Int}}, threshold::Float64)
    goal = graph.n
    is_goal_cell = falses(goal)
    for v in 1:(goal - 1); goal in graph.neighbors[v] && (is_goal_cell[v] = true); end

    init_visited = falses(goal); init_visited[1] = true
    states = State[State(1, 0.0, copy(lms[1].cov), -1, init_visited)]

    w  = 1.0 + PRIMARY_EPSILON
    pq = PriorityQueue{Int, Tuple{Float64, Float64}}()
    enqueue!(pq, 1, (w * graph.shortest_paths[1, goal], unc_radius(states[1].cov)))

    frontier = Dict{Int, Vector{Tuple{Float64, Matrix{Float64}}}}()
    frontier[1] = [(0.0, copy(states[1].cov))]

    # One incumbent, ranked exactly as formation_attach ranks rosters: a
    # spline-clear roster beats a breaching one, then lower escorted uncertainty.
    # Seeded empty, so a search that never reaches the goal reports no solution
    # while one that reaches it but never clears the bound still hands the barrier
    # its best attempt — the same degradation every other planner here performs.
    best_paths = Vector{Vector{Int}}(); best_offs = Int[]; best_unc = Inf
    best_key = nothing
    iters = 0; t0 = time()

    trace(si::Int) = begin
        path = Int[]; p = si
        while p != -1; pushfirst!(path, states[p].node); p = states[p].parent; end
        path
    end

    while !isempty(pq) && iters < ASTAR_ITERATION_LIMIT
        si = dequeue!(pq); S = states[si]
        iters += 1
        astar_progress(iters, ASTAR_ITERATION_LIMIT, t0)

        if is_goal_cell[S.node]
            cand, offs, unc, clear = formation_attach(graph, lms, nbrs, at, trace(si), rosters)
            if clear && unc_within_threshold(unc, threshold, UNC_FEAS_TOL)
                println()
                println("    ✓ escorted-feasible at iter $(iters): dist=$(round(S.dist, digits=3)), " *
                        "unc=$(round(unc, digits=4)) (slot $(offs))")
                return cand, offs, unc, iters
            end
            key = (clear ? 0 : 1, unc)
            if best_key === nothing || key < best_key
                best_key = key; best_paths = cand; best_offs = offs; best_unc = unc
            end
            continue
        end

        for u in graph.neighbors[S.node]
            S.visited[u] && continue

            ncov = edge_cov_continuous(S.node, u, graph, lms, S.cov)
            nd   = S.dist + graph.distance[S.node, u]

            # Chance-constrained obstacle filter at the primary's own belief, same
            # call site as joint_astar's expansion (hexspline_cl.jl:643).
            if !isempty(OBSTACLES)
                p0 = graph.landmarks[S.node]; p1 = graph.landmarks[u]
                segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncov) || continue
            end

            labels = get(frontier, u, Tuple{Float64, Matrix{Float64}}[])
            any(l -> l[1] <= nd + 1e-9 && cov_dominates(l[2], ncov), labels) && continue
            kept = [l for l in labels if !(nd <= l[1] + 1e-9 && cov_dominates(ncov, l[2]))]
            push!(kept, (nd, copy(ncov)))
            frontier[u] = kept

            nvis = copy(S.visited); nvis[u] = true
            push!(states, State(u, nd, ncov, si, nvis))
            enqueue!(pq, length(states),
                     (nd + w * graph.shortest_paths[u, goal], unc_radius(ncov)))
        end
    end

    println()   # release the status-bar line
    isempty(best_paths) && return best_paths, best_offs, Inf, iters
    println("    nothing cleared the escorted bound; handing the best attempt to the barrier: " *
            "unc=$(round(best_unc, digits=4)) (slot $(best_offs))" *
            (best_key[1] == 0 ? "" : " [breaches the spline gate]"))
    return best_paths, best_offs, best_unc, iters
end

# How far each support's arc length ends up from the primary's. Zero unless a
# support had to take the rung-3 fallback (see the header): comm checkpoints are
# arc-indexed, so drift here is drift in WHEN agents talk, not just where.
function formation_arc_drift(graph::LandmarkGraph, paths::Vector{Vector{Int}})
    arclen(p) = sum(graph.distance[p[i-1], p[i]] for i in 2:length(p); init=0.0)
    prim = arclen(paths[end])
    return [arclen(paths[a]) - prim for a in 1:(length(paths) - 1)]
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

    println("\n── FORMATION PLANNING ──")
    println("  Primary: own A* (formation_primary_astar); a goal candidate is accepted on the " *
            "primary's uncertainty WITH its supports attached (threshold $(UNC_RADIUS_THRESHOLD)).")

    # With ONE support there is a single slot to pick and picking it wrong is the
    # worst failure mode (a support held away from every landmark contributes
    # nothing but a halved comm weight). Placement is a linear walk over a path
    # the candidate already fixes, so trying both lateral slots is cheap. With 2+
    # supports the roster is a config decision: the slots interact and the
    # combinatorics are 5·4·… .
    rosters = ns == 1 ? [[1], [-1]] : [FORMATION_OFFSETS[1:ns]]
    println(ns == 1 ? "  One support: both lateral slots (+1 = 60° left, -1 = 60° right) tried per candidate." :
                      "  Body-frame slots (60° units off the primary's heading): $(rosters[1])")

    # Built before the search, not after: the acceptance test places supports at
    # every goal candidate, so it needs these.
    at   = build_hex_lookup(graph)
    nbrs = build_formation_neighbors(graph, at)

    paths, offsets, _, iters =
        formation_primary_astar(graph, landmarks, nbrs, at, rosters, UNC_RADIUS_THRESHOLD)
    if isempty(paths)
        println("  ✗ No route to the goal for the primary.")
        write_no_solution(iters)
        return NamedTuple[]
    end
    ns == 1 && println("  → Chose slot $(offsets)")

    seed_covs, seed_dists = evaluate_full_paths(paths, graph, landmarks, NUM_AGENTS)
    uncs = [unc_radius(seed_covs[a]) for a in 1:NUM_AGENTS]
    drift = formation_arc_drift(graph, paths)
    for a in 1:ns
        println("  Support $a (slot $(offsets[a])): dist=$(round(seed_dists[a], digits=1))m, " *
                "goal_unc=$(round(uncs[a], digits=4))m, arc drift vs primary=$(round(drift[a], digits=1))m")
    end
    println("  Primary : dist=$(round(seed_dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m " *
            "(threshold $(UNC_RADIUS_THRESHOLD) " *
            "$(unc_within_threshold(uncs[end], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓" : "✗"))")
    any(d -> abs(d) > COMM_INTERVAL / 2, drift) && println(
        "  ! A support drifted more than half a comm interval from the primary's arc; " *
        "checkpoints no longer pair them where the formation puts them.")

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
        write(io, "  formation_arc_drift: $(fmtlist(drift))\n")
    end

    return [(id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc)]
end
