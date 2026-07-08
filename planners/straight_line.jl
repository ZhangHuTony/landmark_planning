# ==========================================================================
# straight_line: naive baseline — direct start->goal line, no uncertainty-
# aware optimization. Supports idle at start. Gives Monte Carlo/analysis a
# comparison point against hexspline_cl.
# ==========================================================================
const STRAIGHT_LINE_PRIMARY_WPTS = Int(CFG["straight_line_primary_wpts"])

# Returns a single-element Vector of solutions (same schema as other
# planners: id, ctrls, primary_length, primary_unc) so downstream stages
# don't need to special-case a planner with no Pareto front.
function plan_straight_line(scenario, graph::LandmarkGraph, output_dir::String)
    landmarks = scenario.landmarks
    sx, sy = scenario.start
    gx, gy = scenario.goal

    # Evenly-spaced collinear control points trace the straight line exactly
    # (a B-spline through collinear controls is a subset of that same line),
    # so no B-spline evaluation is needed to produce valid "control points".
    n_wpts = max(2, STRAIGHT_LINE_PRIMARY_WPTS)
    primary_ctrls = Tuple{Float64,Float64}[((1 - t) * sx + t * gx, (1 - t) * sy + t * gy)
                                            for t in range(0.0, 1.0; length=n_wpts)]

    ctrls = Vector{Vector{Tuple{Float64,Float64}}}(undef, NUM_AGENTS)
    for a in 1:(NUM_AGENTS - 1)
        ctrls[a] = [(sx, sy), (sx, sy)]  # supports idle at start
    end
    ctrls[NUM_AGENTS] = primary_ctrls

    covs, arcs, comm_events = evaluate_joint_discrete(ctrls, landmarks, NUM_AGENTS)
    primary_len = arcs[end][end]
    primary_unc = unc_radius(covs[end][end])

    println("\n── STRAIGHT LINE PLANNING ──")
    println("Primary: dist=$(round(primary_len, digits=3)), goal_unc=$(round(primary_unc, digits=4))")

    plt = make_base_plot(landmarks, graph)
    for a in 1:NUM_AGENTS
        is_prim = a == NUM_AGENTS
        xs = [p[1] for p in ctrls[a]]
        ys = [p[2] for p in ctrls[a]]
        plot!(plt, xs, ys, label=is_prim ? "primary" : "support $a",
              color=is_prim ? :blue : get(agent_colors, a, :gray),
              linewidth=is_prim ? 2.2 : 1.3, linestyle=is_prim ? :solid : :dash)
    end
    if TRACK_COMM_EVENTS
        overlay_comm_events!(plt, comm_events)
        write_comm_csv(joinpath(output_dir, "comm_events.csv"), comm_events)
    end
    title!(plt, "straight_line: len=$(round(primary_len,digits=2)), unc=$(round(primary_unc,digits=3))")
    savefig(plt, joinpath(output_dir, "fig_straight_line.png"))

    write_ctrls_csv(joinpath(output_dir, "main_ctrls.csv"), ctrls)

    open(joinpath(output_dir, "results.yaml"), "w") do io
        fmt4(x) = isfinite(x) ? string(round(x, digits=4)) : "null"
        fmt3(x) = isfinite(x) ? string(round(x, digits=3)) : "null"
        write(io, "main:\n")
        write(io, "  continuous_primary_length: $(fmt3(primary_len))\n")
        write(io, "  continuous_uncertainties: [$(fmt4(primary_unc))]\n")
    end

    return [(id=0, ctrls=ctrls, primary_length=primary_len, primary_unc=primary_unc)]
end
