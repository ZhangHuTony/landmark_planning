# ==========================================================================
# sequential: PRIORITIZED planning — every agent planned alone, in priority
# order, against the plans already committed. There is no joint state space
# anywhere in this file.
#
#   Leg 1  primary : A* over (cell, belief) with the helpers PARKED at their
#                    start poses. A parked agent never dead-reckons (it has no
#                    second waypoint to advance to), so it holds Σ₀ and acts as
#                    a free anchor for as long as the primary stays in comm
#                    range — the plan is optimistic by exactly the amount that
#                    anchor is about to walk away. That optimism is the classic
#                    pathology of prioritized planning, and leg 4 is its repair.
#                    Held to a SLACKENED bound (SEQ_LEG1_SLACK), because a
#                    primary planning while its help is still parked should not
#                    be asked to satisfy alone what the team satisfies together.
#   Leg 2..P helper : the primary's trajectory is now KNOWN and time-indexed, so
#                    each helper searches its own whole route against it,
#                    minimising the PRIMARY's TERMINAL uncertainty. Helpers not
#                    yet reached stay parked; ones already planned are fixed —
#                    that ordering IS the priority scheme.
#   Leg 4  primary : re-planned ONCE against the finished helper paths, kept
#                    only if it actually pays. Then stop: the helpers are
#                    deliberately NOT re-planned against the new primary.
#
# Against the two siblings: `greedy` fixes the primary's shortest route and
# places helpers with depth-1 myopia; `formation` fixes the route and bolts
# helpers into body-frame slots. Here every agent gets a real search with full
# lookahead, so what is measured is the cost of the DECOMPOSITION itself rather
# than of a crude helper rule.
#
# NO covariance math lives here. Every belief in this file comes out of
# evaluate_full_paths on a joint roster, which is the same authoritative
# evaluator the planner reports with — so comm fusion with the pinned agents is
# in the numbers the search reasons about, not approximated.
#
# Everything downstream of the seed (obstacle homotopy, barrier refinement,
# outputs) is hexspline_cl's, so seed choice is the only difference measured.
# ==========================================================================

# Beam cap per layer of a helper leg, and whether leg 4 runs at all. Both
# DEFAULTED rather than looked up hard: a config folder that has never been told
# about this planner (run_constraint_sweep.jl copies config/mc/, a separate
# snapshot) would otherwise KeyError at include time, before a single run starts.
const SEQ_BEAM           = Int(get(CFG, "sequential_beam", 400))
const SEQ_REPLAN_PRIMARY = Bool(get(CFG, "sequential_replan_primary", true))

# Multiplier on unc_radius_threshold for LEG 1 ONLY. Leg 1 plans the primary while
# every helper is still parked at the start, so holding it to the full constraint
# asks one agent to achieve what the whole team is meant to achieve — and as the
# constraint tightens (a sweep walks it down as a fraction of the unconstrained
# reference) that stops being merely pessimistic and starts changing the answer:
# with no goal candidate under the bound, the search drains its whole frontier and
# falls back to the MINIMUM-UNCERTAINTY route, i.e. a long landmark detour, and
# then hands the helpers a route that never needed them. Slack keeps leg 1
# returning the SHORTEST route under a reachable bound — A* pops in f order, so
# the first goal candidate that clears the bound is the shortest one that does —
# and leaves the remaining uncertainty for the helpers and leg 4 to pay off.
#
# 2.0 covers the sweep's whole 100%→50% ladder: at every rung, twice the current
# threshold is at least the unconstrained reference's own uncertainty, so the
# direct route stays available to leg 1 and the bound never becomes the thing that
# decides the plan. Leg 4 is deliberately NOT slackened — it plans against real
# helpers and is the route actually shipped, so it is held to the true constraint.
# Distinct from the engine's enable_relaxed_discrete / effective_discrete_unc_
# threshold, which relaxes the DISCRETE→CONTINUOUS handoff on the grounds that the
# barrier will recover; this one relaxes the ALONE→ESCORTED handoff. Set 1.0 to
# hold leg 1 to the strict bound, or a large value to let it ignore uncertainty
# entirely (which makes leg 1 the shortest obstacle-clear route, as `greedy` does).
const SEQ_LEG1_SLACK = Float64(get(CFG, "sequential_leg1_slack", 2.0))

# Metre-scale comparisons between sums accumulated in a different order: "equal"
# arrives as a 1-ULP spread, and an exact `<` reads that noise as a preference.
# Same value and same reason as greedy's GREEDY_TIE_TOL, which cannot be reused —
# planners/greedy.jl is only included when `greedy` itself is selected.
const SEQ_TIE_TOL = 1e-9

# The roster is every agent's CURRENT commitment, in planner order (supports
# 1..na-1, primary last): a parked agent is the single-node path [1], a committed
# one its full node sequence. Splicing a candidate into it is how every
# evaluation below is built.
@inline function seq_with(roster::Vector{Vector{Int}}, a::Int, path::Vector{Int})
    r = copy(roster)
    r[a] = path
    return r
end

# Every agent's state at step k. Truncating (rather than scoring a partial helper
# path against the primary's *whole* route) is what makes a helper's layer-k
# label an honest joint STATE: both beliefs are then read at the same instant.
# A parked agent has nothing to cut.
@inline seq_trunc(roster::Vector{Vector{Int}}, k::Int) =
    [length(p) > k ? p[1:k] : p for p in roster]

# ==========================================================================
# Legs 1 and 4 — the primary's route, every other agent pinned
# ==========================================================================
# Weighted A* over (cell, belief). Structurally the same search as joint_astar's
# single-agent fast path — same f = g + (1+ε)·h on the Floyd-Warshall heuristic,
# same node-level PSD-dominance frontier, same anytime incumbent when nothing
# clears the threshold — with the one difference that is the whole point: a
# candidate's covariance is evaluate_full_paths on the JOINT prefix, so the
# pinned agents' comm fusion is inside the belief being searched over.
#
# That costs one joint evaluation per successor (~10 µs) where joint_astar pays
# one edge_cov_continuous (~0.3 µs). A single-agent search space can afford it;
# the joint one it was written for could not. ASTAR_ITERATION_LIMIT still caps it.
function seq_primary_astar(graph::LandmarkGraph, lms::Vector{Landmark},
                           roster::Vector{Vector{Int}}, threshold::Float64)
    na = length(roster); primary = na; goal = graph.n
    is_goal_cell = falses(goal)
    for v in 1:(goal - 1); goal in graph.neighbors[v] && (is_goal_cell[v] = true); end

    init_visited = falses(goal); init_visited[1] = true
    init_covs, _ = evaluate_full_paths(seq_with(roster, primary, Int[1]), graph, lms, na)
    states = State[State(1, 0.0, init_covs[primary], -1, init_visited)]

    w  = 1.0 + PRIMARY_EPSILON
    pq = PriorityQueue{Int, Tuple{Float64, Float64}}()
    enqueue!(pq, 1, (w * graph.shortest_paths[1, goal], unc_radius(states[1].cov)))

    # Node-level Pareto frontier, sound covariance dominance — a label is only
    # pruned when an existing one is better in distance AND no larger in PSD order.
    frontier = Dict{Int, Vector{Tuple{Float64, Matrix{Float64}}}}()
    frontier[1] = [(0.0, copy(states[1].cov))]

    best_path = Int[]; best_dist = 0.0; best_unc = Inf
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

        unc = unc_radius(S.cov)
        PRUNE_BY_PRIMARY_UNCERTAINTY &&
            unc_exceeds_threshold(unc, threshold, UNC_FEAS_TOL) && continue

        if is_goal_cell[S.node]
            # No verify step here, unlike joint_astar: S.cov already IS the exact
            # joint evaluation of this prefix — it was produced by
            # evaluate_full_paths when the state was pushed, not estimated.
            path = trace(si)
            if unc_within_threshold(unc, threshold, UNC_FEAS_TOL)
                # Same hard gate joint_astar applies at its own goal pops: the
                # discrete filter only tests the polyline between hex centres, and
                # the refined spline cuts those corners off, so a polyline-clean
                # route can still enter an obstacle. Obstacle clearance and the
                # support cap are topological — no local perturbation of the spline
                # re-routes around a wall — so a candidate that fails here is
                # discarded and the search continues.
                if !seed_spline_clear(seq_with(roster, primary, path), graph, lms)
                    continue
                end
                println()
                println("    ✓ feasible at iter $(iters): dist=$(round(S.dist, digits=3)), " *
                        "unc=$(round(unc, digits=4))")
                return path, S.dist, unc, iters
            end
            # Over threshold, but the barrier can still trade length for uncertainty
            # from here — keep the lowest-uncertainty one as a fallback seed. The
            # gate stays hard on this path too, for the same reason.
            if unc < best_unc && seed_spline_clear(seq_with(roster, primary, path), graph, lms)
                best_path = path; best_dist = S.dist; best_unc = unc
            end
            continue
        end

        prefix = trace(si)          # once per expansion, not once per successor
        for u in graph.neighbors[S.node]
            S.visited[u] && continue

            covs, dists = evaluate_full_paths(seq_with(roster, primary, vcat(prefix, u)),
                                              graph, lms, na)
            ncov = covs[primary]; nd = dists[primary]

            # Chance-constrained obstacle filter at the primary's own belief, same
            # call site as joint_astar's expansion (hexspline_cl.jl:904).
            if !isempty(OBSTACLES)
                p0 = graph.landmarks[S.node]; p1 = graph.landmarks[u]
                segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), ncov) || continue
            end

            labels = get(frontier, u, Tuple{Float64, Matrix{Float64}}[])
            any(l -> l[1] <= nd + SEQ_TIE_TOL && cov_dominates(l[2], ncov), labels) && continue
            kept = [l for l in labels
                    if !(nd <= l[1] + SEQ_TIE_TOL && cov_dominates(ncov, l[2]))]
            push!(kept, (nd, copy(ncov)))
            frontier[u] = kept

            nvis = copy(S.visited); nvis[u] = true
            push!(states, State(u, nd, ncov, si, nvis))
            enqueue!(pq, length(states),
                     (nd + w * graph.shortest_paths[u, goal], unc_radius(ncov)))
        end
    end

    println()   # release the status-bar line
    isempty(best_path) && return Int[], 0.0, Inf, iters
    println("    nothing under the threshold; handing the best incumbent to the barrier: " *
            "dist=$(round(best_dist, digits=3)), unc=$(round(best_unc, digits=4))")
    return best_path, best_dist, best_unc, iters
end

# ==========================================================================
# Legs 2..P — one helper's whole trajectory against a KNOWN primary
# ==========================================================================
# A layered forward search: layer k holds every belief-distinct place this helper
# could stand at the primary's step k, each label carrying the JOINT belief at
# that instant. At the last layer that evaluation is the primary's TERMINAL
# covariance — the objective — so the helper commits to its whole route knowing
# the whole of the primary's, which is precisely what greedy's depth-1 rule cannot.
#
# Lockstep, one hex per primary step. Comm checkpoints are arc-indexed
# (apply_synchronized_propagation!), so a helper that covers extra ground is
# rewound at the next checkpoint and silently drops out of comm range for the rest
# of the run — see the arc-drift note at the top of planners/formation.jl.
#
# Pruning is PSD dominance on BOTH beliefs at equal (cell, step): a label no
# larger in either covariance, at no more arc, cannot lose — and it has at least
# the same moves available, since the only belief-dependent gate is the obstacle
# filter, which only ever closes moves as Σ grows.
#
# ponytail: SEQ_BEAM is the backstop for when dominance does not prune enough. It
# ranks a layer by primary-uncertainty-SO-FAR, which is myopic, so a very tight
# beam degrades this leg back toward greedy. Raise it (or drop the cap) if a
# scenario needs a detour whose payoff only lands later.
struct SeqLabel
    path :: Vector{Int}
    arc  :: Float64
    cf   :: Matrix{Float64}   # this helper's covariance at this step
    cp   :: Matrix{Float64}   # the primary's covariance at this step
    uf   :: Float64
    up   :: Float64
end

# Layer ranking, and at the final layer the objective itself: the primary's
# terminal uncertainty, then the helper's own (an out-of-range helper cannot move
# the primary's covariance at all, so row 1 ties and this is what sends it after
# a landmark fix instead of choosing arbitrarily), then arc, then the node
# sequence so the whole search is deterministic.
seq_rank(L::SeqLabel) = (L.up, L.uf, L.arc, L.path)

function seq_helper_search(graph::LandmarkGraph, lms::Vector{Landmark},
                           roster::Vector{Vector{Int}}, free::Int)
    na = length(roster)
    K  = length(roster[na])
    K < 2 && return Int[1]

    layer = SeqLabel[SeqLabel(Int[1], 0.0, zeros(2, 2), zeros(2, 2), 0.0, 0.0)]

    for k in 2:K
        base  = seq_trunc(roster, k)      # everyone else's state at the same step
        cells = Dict{Int, Vector{SeqLabel}}()

        for L in layer, u in graph.neighbors[L.path[end]]
            # graph.n is the terminal goal MARKER, not a cell: every goal-adjacent
            # state has an edge into it and it has none out, so a helper that
            # stepped there would have no successor at all next step. The primary's
            # own path stops at the goal-adjacent cell for the same reason.
            u == graph.n && continue

            cand = vcat(L.path, u)
            covs, dists = evaluate_full_paths(seq_with(base, free, cand), graph, lms, na)

            # Support cap — a helper must never outrun the primary
            # (hexspline_cl.jl:889); the optimizer later enforces it on spline lengths.
            dists[free] > dists[na] + SEQ_TIE_TOL && continue
            if !isempty(OBSTACLES)
                p0 = graph.landmarks[L.path[end]]; p1 = graph.landmarks[u]
                segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), covs[free]) || continue
            end

            cf = covs[free]; cp = covs[na]; arc = dists[free]
            keep = get!(cells, u, SeqLabel[])
            any(o -> o.arc <= arc + SEQ_TIE_TOL &&
                     cov_dominates(o.cf, cf) && cov_dominates(o.cp, cp), keep) && continue
            filter!(o -> !(arc <= o.arc + SEQ_TIE_TOL &&
                           cov_dominates(cf, o.cf) && cov_dominates(cp, o.cp)), keep)
            push!(keep, SeqLabel(cand, arc, cf, cp, unc_radius(cf), unc_radius(cp)))
        end

        nxt = collect(Iterators.flatten(values(cells)))
        isempty(nxt) && return nothing
        # Sorted unconditionally, not just when the cap binds: Dict iteration order
        # is an implementation detail, and the search must not depend on it.
        sort!(nxt, by=seq_rank)
        length(nxt) > SEQ_BEAM && resize!(nxt, SEQ_BEAM)
        layer = nxt
    end

    # Best-first: the layer is sorted by seq_rank, so this returns the highest-ranked
    # route that ALSO survives as a SPLINE. The per-step filter above only tests the
    # straight polyline between hex centres, but what ships is gated on the spline
    # (seed_spline_clear, in plan_sequential) — so without this the leg optimises
    # against one standard and is judged by another, with nothing downstream able to
    # repair it: the helpers are never re-planned, and leg 4 can only vary the PRIMARY,
    # which cannot fix a helper-side breach. Measured on s001/s017 of
    # constraint_sweep/2026-08-14_16-55-00: every one of leg 4's 41 and 47 goal
    # candidates was uncertainty-feasible and every one was vetoed by the helper's
    # breach, leaving the parked fallback — which discards the whole leg — as the only
    # recourse. It fired on 28 of 50 scenarios at the 100% rung.
    #
    # Exact rather than approximate: at k=K the helper's path is already the primary's
    # length, so seed_control_points pads nothing and this builds the same spline the
    # seed gate will. Affordable because it runs once per LEG, not once per expansion —
    # the cost that put the gate at goal pops to begin with (hexspline_cl.jl:308).
    # Measured ~0.2 ms per test, median 8 tests, a full 400-label beam in 0.09 s.
    for (i, L) in enumerate(layer)
        if seed_spline_clear(seq_with(roster, free, L.path), graph, lms)
            i > 1 && println("    (took rank #$(i) of $(length(layer)); " *
                             "$(i - 1) better-ranked route(s) breach as splines)")
            return L.path
        end
    end
    # Nothing in the final layer survives. Hand back the rank-best anyway — that is
    # exactly what shipped before this loop existed, so plan_sequential's parked
    # fallback takes over as it always did. 2 of the 28 measured scenarios land here.
    println("    (no route in the final layer survives as a spline)")
    return layer[1].path
end

# ==========================================================================
function plan_sequential(scenario, graph::LandmarkGraph, output_dir::String)
    landmarks = scenario.landmarks
    na = NUM_AGENTS

    function write_no_solution(iters::Int)
        open(joinpath(output_dir, "results.yaml"), "w") do io
            write(io, "main:\n")
            write(io, "  primary_length: null\n  primary_unc: null\n")
            write(io, "  astar_iterations: $(iters)\n")
            write(io, "  discrete_uncertainties: []\n  primary_disc_unc: null\n")
            write(io, "  continuous_primary_length: null\n  continuous_uncertainties: []\n")
        end
    end

    println("\n── SEQUENTIAL PLANNING ($(na) agent(s), one at a time) ──")

    # ── Leg 1: the primary, helpers parked at their start poses ──
    # Slackened bound (see SEQ_LEG1_SLACK): the point of leg 1 is the shortest
    # route the helpers will plausibly be able to escort, not the route one agent
    # can certify on its own.
    roster = [Int[1] for _ in 1:na]
    # With no helpers at all there is nothing to hand the slack TO, so leg 1 is
    # simply held to the real constraint.
    leg1_slack     = na > 1 ? SEQ_LEG1_SLACK : 1.0
    leg1_threshold = leg1_slack * UNC_RADIUS_THRESHOLD
    println("  Leg 1 — primary, $(na - 1) helper(s) parked at start (bound " *
            "$(round(leg1_threshold, digits=4)) = $(leg1_slack)x the " *
            "$(UNC_RADIUS_THRESHOLD) threshold the shipped route is held to):")
    ppath, _, _, iters = seq_primary_astar(graph, landmarks, roster, leg1_threshold)
    if isempty(ppath)
        println("  ✗ No route to the goal for the primary.")
        write_no_solution(iters)
        return NamedTuple[]
    end
    roster[na] = ppath

    # ── Legs 2..P: one helper at a time, against everything already committed ──
    for a in 1:(na - 1)
        println("  Leg $(a + 1) — helper $(a) vs the fixed primary trajectory " *
                "($(length(roster[na])) steps, beam $(SEQ_BEAM)):")
        hpath = seq_helper_search(graph, landmarks, roster, a)
        if hpath === nothing
            println("    ✗ Helper $(a) stalled: no admissible successor clears the " *
                    "support cap and the obstacle filter.")
            write_no_solution(iters)
            return NamedTuple[]
        end
        covs, _ = evaluate_full_paths(seq_with(roster, a, hpath), graph, landmarks, na)
        roster[a] = hpath
        println("    → helper $(a) placed; primary goal_unc now " *
                "$(round(unc_radius(covs[na]), digits=4))")
    end

    # ── Leg 4: re-plan the primary ONCE, now that the helpers are real ──
    # Leg 1 planned against anchors that have since walked away, so this is where
    # that optimism gets paid back — with real escorts the primary can usually hold
    # the threshold on a SHORTER route. Kept only if it measurably pays, and the
    # helpers are not re-planned in response (the algorithm stops here by design).
    replanned = false
    base_covs, base_dists = evaluate_full_paths(roster, graph, landmarks, na)
    len1 = base_dists[na]; unc1 = unc_radius(base_covs[na])
    if SEQ_REPLAN_PRIMARY && na > 1
        println("  Leg $(na + 1) — primary re-planned against the committed helper paths:")
        ppath2, _, _, iters2 = seq_primary_astar(graph, landmarks, roster,
                                                 UNC_RADIUS_THRESHOLD)
        iters += iters2
        if !isempty(ppath2)
            # Helpers were planned in lockstep with the OLD primary. A longer new
            # route lets seed_control_points park them at their last cell; a shorter
            # one must truncate, or a helper outruns the primary and the optimizer's
            # support row starts out violated.
            cand = [p[1:min(length(p), length(ppath2))] for p in roster]
            cand[na] = ppath2
            covs2, dists2 = evaluate_full_paths(cand, graph, landmarks, na)
            len2 = dists2[na]; unc2 = unc_radius(covs2[na])
            ok1 = unc_within_threshold(unc1, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL)
            ok2 = unc_within_threshold(unc2, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL)
            # Feasible beats infeasible; then LENGTH — in both branches, not just the
            # feasible one; then uncertainty, which only breaks exact length ties.
            #
            # Length has to stay in the infeasible branch. Ranking those purely on
            # uncertainty puts no price on the detour that buys it, and the search
            # will pay any amount: measured on sweep 2026-08-11_16-53-11 s004 at 60%,
            # leg 1 offered 950 m at unc 3.253 against a 2.939 threshold and the
            # re-plan offered 6050 m at 3.007, so a uncertainty-first rule spent
            # 5100 m for 0.25 m — and was STILL infeasible. That branch produced
            # every blow-up in that sweep (ratio 6.31 once, two outright failures)
            # and helped in not one case. When neither route clears the threshold the
            # shorter one is also the better seed: closing a small violation by
            # lengthening locally is exactly what the barrier does, and a route that
            # has already spent 6x the length budget leaves it nothing to work with.
            # Length rounded so a 1-ULP spread cannot pre-empt the row (formation.jl
            # ranks its slots the same way).
            # Spline clearance outranks EVERYTHING, including uncertainty feasibility.
            # An over-threshold seed is still workable — trading length for
            # uncertainty is exactly what the barrier does — but obstacle clearance
            # and the support cap are topological, and no local perturbation
            # re-routes a spline around a wall. Shipping a seed that breaches leaves
            # the optimizer starting outside the feasible set with nothing it can do,
            # which surfaces as `recovery_failed` no matter how slack the constraint
            # was: measured on probe_A3_sound, s001 p100 failed at unc 1.65 against a
            # threshold of 8.63, seed min_slack -11.05.
            #
            # Both rosters have to be tested HERE rather than relying on the gate
            # inside seq_primary_astar. That gate sees the untruncated roster, while
            # what actually ships is `cand`, whose helpers are cut to the new
            # primary's length — and the truncation can break a clearance the gate
            # had already certified.
            clear1 = seed_spline_clear(roster, graph, landmarks)
            clear2 = seed_spline_clear(cand, graph, landmarks)
            rank(clr, ok, len, unc) = (clr ? 0 : 1, ok ? 0 : 1, round(len, digits=6), unc)
            take = rank(clear2, ok2, len2, unc2) < rank(clear1, ok1, len1, unc1)
            println("    leg 1 route: len=$(round(len1, digits=1)), unc=$(round(unc1, digits=4)) " *
                    "$(ok1 ? "✓" : "✗")$(clear1 ? "" : " [breaches]")  |  " *
                    "re-planned: len=$(round(len2, digits=1)), " *
                    "unc=$(round(unc2, digits=4)) $(ok2 ? "✓" : "✗")$(clear2 ? "" : " [breaches]")  → " *
                    "$(take ? "TAKE re-plan" : "keep leg 1")")
            take && (roster = cand; replanned = true)
        else
            println("    re-plan found nothing; keeping the leg 1 route.")
        end
    end

    paths = roster
    # Last resort. The helpers are planned AFTER the leg-1 primary and are never
    # re-planned, so the combination that actually ships is a roster no gate has
    # necessarily seen — leg 1 certified the primary against PARKED helpers, and
    # leg 4 only runs (and is only taken) sometimes. If what is left still breaches,
    # fall back to the configuration leg 1 did certify: primary as planned, helpers
    # parked. A parked helper never dead-reckons, so it holds Σ₀ and still anchors —
    # weaker localization, but a seed the optimizer can start from rather than one it
    # must reject outright.
    if na > 1 && !seed_spline_clear(paths, graph, landmarks)
        parked = [a == na ? paths[na] : Int[1] for a in 1:na]
        if seed_spline_clear(parked, graph, landmarks)
            println("  Shipped roster breaches the obstacle/support gate; " *
                    "falling back to parked helpers.")
            paths = parked
        else
            println("  [warn] Shipped roster breaches and so does the parked fallback; " *
                    "the optimizer starts outside the feasible set.")
        end
    end
    seed_covs, seed_dists = evaluate_full_paths(paths, graph, landmarks, na)
    uncs = [unc_radius(seed_covs[a]) for a in 1:na]
    for a in 1:(na - 1)
        println("  Helper $(a): dist=$(round(seed_dists[a], digits=1))m, " *
                "goal_unc=$(round(uncs[a], digits=4))m")
    end
    println("  Primary  : dist=$(round(seed_dists[end], digits=1))m, " *
            "goal_unc=$(round(uncs[end], digits=4))m (threshold $(UNC_RADIUS_THRESHOLD) " *
            "$(unc_within_threshold(uncs[end], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓" : "✗"))")

    # ── Figure of the sequential seed, before refinement ──
    let
        agent_positions = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
        _, _, seed_comm, seed_lm = evaluate_joint_discrete(agent_positions, landmarks, na)
        plt = make_base_plot(landmarks, graph)
        for (ai, p) in enumerate(paths)
            is_prim = (ai == na)
            xs = [graph.landmarks[i].x for i in p]; ys = [graph.landmarks[i].y for i in p]
            plot!(plt, xs, ys, label=(is_prim ? "primary" : "helper $ai"),
                  color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                  linewidth=(is_prim ? 2.4 : 1.3), linestyle=(is_prim ? :solid : :dash))
            scatter!(plt, xs, ys, label=false,
                     color=(is_prim ? :blue : get(agent_colors, ai, :gray)),
                     markersize=3, markerstrokewidth=0)
        end
        title!(plt, "sequential seed [len=$(round(seed_dists[end],digits=2)), " *
                    "unc=$(round(uncs[end],digits=3))$(replanned ? ", re-planned" : "")]")
        xlabel!(plt, "x (m)"); ylabel!(plt, "y (m)")
        save_path_figures(plt, fig_path(output_dir, "fig_sequential_discrete.png"),
                          seed_comm, seed_lm)
    end

    # ── Refinement: hexspline_cl's optimizer, unchanged ──
    opt_len, opt_unc, _, opt_covs, opt_ctrls, refine =
        optimize_continuous(paths, graph, landmarks, na, output_dir;
                            cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main")

    write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
    TRACK_COMM_EVENTS     && write_comm_csv(csv_path(output_dir, "comm_events.csv"), refine.comm_events)
    TRACK_LANDMARK_EVENTS && write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), refine.landmark_events)

    cont_uncs = (!isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:na)) ?
                [unc_radius(opt_covs[a][end]) for a in 1:na] : Float64[]

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
        write(io, "  sequential_replanned: $(replanned)\n")
        write(io, "  sequential_leg1_length: $(fmtx(len1))\n")
    end

    return [(id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc)]
end
