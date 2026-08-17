# ==========================================================================
# greedy: baseline that splits apart the two things the joint A* does at once
# — routing the primary, and placing the helpers.
#
#   Primary : commits to its shortest obstacle-clear route with NO uncertainty
#             reasoning at all (the single-agent fast path in joint_astar).
#   Helpers : at every joint step, pick out of their ±60° successors the move
#             that minimises the PRIMARY's uncertainty at that same step.
#             Depth-1 myopia, no lookahead — that IS the baseline. A helper
#             that cannot pay a detour now for a fix later is exactly what
#             hexspline_cl's joint search is being measured against.
#
# Everything downstream of the seed — obstacle homotopy, barrier refinement,
# outputs — is hexspline_cl's, so the only thing this measures is seed choice.
# ==========================================================================

# Support cap and the ranking below both compare metre-scale quantities that are
# sums of the same terms accumulated in a different order, so "equal" arrives as
# a 1-ULP spread rather than exact equality. 1e-9 m sits ~6 orders above that
# noise and ~7 below any physically meaningful difference.
const GREEDY_TIE_TOL = 1e-9

# Lexicographic "is `a` strictly better than `b`" over the ranking keys below.
# Needs the tolerance: out of comm range the helper cannot move the primary's
# covariance at all, so row 1 is *physically* tied and differs only in the last
# bit — and an exact `<` reads that noise as a preference, which silently makes
# every later row dead code. (Measured: the helper declined a landmark fix that
# took its own uncertainty 8.22 → 0.93, because row 1 was 1 ULP "better" on the
# move that skipped it.) The final element is the node tuple, compared exactly.
@inline function greedy_better(a::Tuple, b::Tuple)
    for i in 1:(length(a) - 1)
        a[i] < b[i] - GREEDY_TIE_TOL && return true
        a[i] > b[i] + GREEDY_TIE_TOL && return false
    end
    return a[end] < b[end]
end

# Walk all agents forward in lockstep along `primary_path`, choosing each support's
# next node greedily. Returns the support node paths, or `nothing` if any support
# runs out of admissible moves (stall ⇒ the run fails; see plan_greedy).
function greedy_support_rollout(primary_path::Vector{Int}, graph::LandmarkGraph,
                                lms::Vector{Landmark}, na::Int)
    ns = na - 1
    ns == 0 && return Vector{Vector{Int}}()

    paths   = [Int[1] for _ in 1:ns]
    visited = [falses(graph.n) for _ in 1:ns]
    for a in 1:ns
        visited[a][1] = true
    end

    for k in 2:length(primary_path)
        # Only successors this support has not already occupied — the same cycle
        # guard joint_astar carries on each agent (`S.visited[agent][u] && continue`).
        # `graph.n` is excluded: it is the terminal goal MARKER, not a cell. Every
        # goal-adjacent state has an edge into it and it has none out, so a support
        # that stepped there had zero successors on the next move and stalled the
        # whole run — the primary's own path stops at the goal-adjacent cell and
        # never enters it either (seed_control_points pins the goal position instead).
        options = [[u for u in graph.neighbors[paths[a][end]] if !visited[a][u] && u != graph.n]
                   for a in 1:ns]
        for a in 1:ns
            if isempty(options[a])
                println("  ✗ Support $a stalled at step $k: no unvisited ±60° successor of " *
                        "node $(paths[a][end]).")
                return nothing
            end
        end

        best_combo = nothing
        best_key   = nothing
        for combo in Iterators.product(options...)
            prefix = Vector{Vector{Int}}(undef, na)
            for a in 1:ns
                prefix[a] = vcat(paths[a], combo[a])
            end
            prefix[na] = primary_path[1:k]

            # ONE authoritative evaluation feeds the cap, the obstacle filter and
            # the score, so all three read the numbers the planner reports.
            covs, dists = evaluate_full_paths(prefix, graph, lms, na)

            admissible = true
            for a in 1:ns
                # Support cap — supports must not outrun the primary (hexspline_cl.jl:889).
                if dists[a] > dists[na] + GREEDY_TIE_TOL
                    admissible = false; break
                end
                # Chance-constrained obstacle filter, same call site as the A*
                # expansion (hexspline_cl.jl:904), at this support's own belief.
                if !isempty(OBSTACLES)
                    p0 = graph.landmarks[paths[a][end]]; p1 = graph.landmarks[combo[a]]
                    if !segment_obstacle_free((p0.x, p0.y), (p1.x, p1.y), covs[a])
                        admissible = false; break
                    end
                end
            end
            admissible || continue

            # Lexicographic ranking. The objective is row 1; the rest only ever
            # break ties, which are guaranteed: comm_weight drops under
            # COMM_WEIGHT_MIN past ~123 m, and beyond that the support cannot move
            # the primary's covariance at all, so every candidate scores identically.
            #
            # ponytail: row 2 (the helper's OWN uncertainty) is what makes an
            # out-of-range helper go hunting for a landmark fix rather than pick
            # arbitrarily — and also what lets it chase a distant landmark and never
            # come back, since row 1 cannot see the return trip. That myopia is the
            # property this baseline exists to expose, not a bug to patch here.
            px = graph.landmarks[primary_path[k]].x; py = graph.landmarks[primary_path[k]].y
            key = (unc_radius(covs[na]),                                   # primary unc — the objective
                   sum(unc_radius(covs[a]) for a in 1:ns),                 # helpers' own unc
                   sum(hypot(graph.landmarks[combo[a]].x - px,
                             graph.landmarks[combo[a]].y - py) for a in 1:ns),  # closeness to primary
                   combo)                                                  # determinism
            if best_key === nothing || greedy_better(key, best_key)
                best_key   = key
                best_combo = combo
            end
        end

        if best_combo === nothing
            println("  ✗ Supports stalled at step $k: no successor combination clears " *
                    "the support cap and the obstacle filter.")
            return nothing
        end
        for a in 1:ns
            push!(paths[a], best_combo[a])
            visited[a][best_combo[a]] = true
        end
    end
    return paths
end

function plan_greedy(scenario, graph::LandmarkGraph, output_dir::String)
    landmarks = scenario.landmarks

    function write_no_solution(iters::Int)
        open(joinpath(output_dir, "results.yaml"), "w") do io
            write(io, "main:\n")
            write(io, "  primary_length: null\n")
            write(io, "  primary_unc: null\n")
            write(io, "  astar_iterations: $(iters)\n")
            write(io, "  discrete_uncertainties: []\n")
            write(io, "  primary_disc_unc: null\n")
            write(io, "  continuous_primary_length: null\n")
            write(io, "  continuous_uncertainties: []\n")
        end
    end

    # ── Primary: shortest obstacle-clear route, no uncertainty reasoning ──
    # joint_astar's na==1 fast path IS a shortest-path search: primary_epsilon is
    # 0 (exact), threshold=Inf disables every uncertainty gate, and it still runs
    # the obstacle filter at the primary's true running Σ plus the seed_spline_clear
    # goal gate. Same computation run_constraint_sweep.jl's reference run performs.
    println("\n── GREEDY PLANNING ──")
    println("  Primary: shortest obstacle-clear route (single-agent A*, no uncertainty bound)")
    ppaths, _, _, iters = joint_astar(graph, landmarks, Inf, 1)
    if isempty(ppaths) || isempty(ppaths[1])
        println("  ✗ No obstacle-clear route to the goal exists for the primary.")
        write_no_solution(iters)
        return NamedTuple[]
    end
    primary_path = ppaths[1]

    # ── Helpers: greedy, one step at a time ──
    println("  Helpers: greedy placement, minimising the primary's uncertainty per step")
    support_paths = greedy_support_rollout(primary_path, graph, landmarks, NUM_AGENTS)
    if support_paths === nothing
        write_no_solution(iters)
        return NamedTuple[]
    end

    paths = vcat(support_paths, [primary_path])
    # Last-resort spline gate, mirroring sequential.jl's shipped-roster fallback.
    # The primary was gated inside its own A*, but the helpers are attached
    # afterwards and nothing has spline-tested the COMBINATION: a helper whose
    # spline cuts an obstacle corner (or outruns the primary as a spline) would
    # reach the optimizer topologically stuck and die as recovery_failed — a
    # false failure the search never made. Over-threshold uncertainty stays
    # workable for the barrier, so it is not tested here; only the topological
    # rows are. The fallback parks the helpers (a parked helper never
    # dead-reckons, so it holds Σ₀ and still anchors from the start).
    if NUM_AGENTS > 1 && !seed_spline_clear(paths, graph, landmarks)
        parked = [a == NUM_AGENTS ? primary_path : Int[1] for a in 1:NUM_AGENTS]
        if seed_spline_clear(parked, graph, landmarks)
            println("  Greedy roster breaches the obstacle/support gate; " *
                    "falling back to parked helpers.")
            paths = parked
        else
            println("  [warn] Greedy roster breaches and so does the parked fallback; " *
                    "the optimizer starts outside the feasible set.")
        end
    end
    seed_covs, seed_dists = evaluate_full_paths(paths, graph, landmarks, NUM_AGENTS)
    uncs = [unc_radius(seed_covs[a]) for a in 1:NUM_AGENTS]

    for a in 1:(NUM_AGENTS - 1)
        println("  Support $a: dist=$(round(seed_dists[a], digits=1))m, goal_unc=$(round(uncs[a], digits=4))m")
    end
    println("  Primary  : dist=$(round(seed_dists[end], digits=1))m, goal_unc=$(round(uncs[end], digits=4))m " *
            "(threshold $(UNC_RADIUS_THRESHOLD) " *
            "$(unc_within_threshold(uncs[end], UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) ? "✓" : "✗"))")

    # ── Figure of the greedy seed, before refinement ──
    let
        agent_positions = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
        _, _, seed_comm, seed_lm = evaluate_joint_discrete(agent_positions, landmarks, NUM_AGENTS)
        plt = make_base_plot(landmarks, graph)
        for (ai, p) in enumerate(paths)
            is_prim = (ai == NUM_AGENTS)
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
        title!(plt, "greedy seed [len=$(round(seed_dists[end],digits=2)), unc=$(round(uncs[end],digits=3))]")
        xlabel!(plt, "x (m)"); ylabel!(plt, "y (m)")
        save_path_figures(plt, fig_path(output_dir, "fig_greedy_discrete.png"), seed_comm, seed_lm)
    end

    # ── Refinement: hexspline_cl's optimizer, unchanged ──
    # A seed above the threshold is handed to the barrier for recovery and judged
    # on refinement_status, exactly as hexspline_cl does.
    opt_len, opt_unc, _, opt_covs, opt_ctrls, refine =
        optimize_continuous(paths, graph, landmarks, NUM_AGENTS, output_dir;
                            cont_threshold=UNC_RADIUS_THRESHOLD, fig_prefix="main")

    write_ctrls_csv(csv_path(output_dir, "main_ctrls.csv"), opt_ctrls)
    if TRACK_COMM_EVENTS
        write_comm_csv(csv_path(output_dir, "comm_events.csv"), refine.comm_events)
    end
    if TRACK_LANDMARK_EVENTS
        write_landmark_csv(csv_path(output_dir, "landmark_events.csv"), refine.landmark_events)
    end

    cont_uncs = (!isempty(opt_covs) && all(a -> !isempty(opt_covs[a]), 1:NUM_AGENTS)) ?
                [unc_radius(opt_covs[a][end]) for a in 1:NUM_AGENTS] : Float64[]

    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        fmtlist(v) = isempty(v) ? "[]" : "[" * join(fmt4.(v), ", ") * "]"
        # Full precision for anything a harness MACHINE-READS against a tolerance
        # (see hexspline_cl.jl); rounding is for the human-facing rows only.
        fmtx(x) = isfinite(x) ? repr(x) : "null"
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
    end

    return [(id=0, ctrls=opt_ctrls, primary_length=opt_len, primary_unc=opt_unc)]
end
