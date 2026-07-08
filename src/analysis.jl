# ==========================================================================
# Path evaluation / debug replay — shared across planners
# ==========================================================================
# Replays a saved ctrls CSV (any planner's output, or an externally-produced
# one matching the same schema — see read_ctrls_csv in src/viz.jl) through
# the shared covariance model and plots trajectory + uncertainty profile.
# `threshold` is optional since not every planner has a feasibility bound.
function run_path_eval(csv_path::String, landmarks::Vector{Landmark}, graph::LandmarkGraph,
                        output_dir::String; threshold::Union{Float64,Nothing}=nothing)
    println("\n── PATH EVAL MODE ($(csv_path)) ──")
    positions = read_ctrls_csv(csv_path)
    na = length(positions)

    covs, arcs, comm = evaluate_joint_discrete(positions, landmarks, na)
    primary = na
    is_primary_mask = [a == primary for a in 1:na]

    # Trajectory plot
    plt_traj = make_base_plot(landmarks, graph)
    for a in 1:na
        xs = [p[1] for p in positions[a]]
        ys = [p[2] for p in positions[a]]
        is_prim = is_primary_mask[a]
        clr = is_prim ? :blue : get(agent_colors, a, :gray)
        plot!(plt_traj, xs, ys, label=(is_prim ? "primary" : "support $a"),
              color=clr, linewidth=is_prim ? 2.2 : 1.3,
              linestyle=is_prim ? :solid : :dash)
        scatter!(plt_traj, xs, ys, label=false, color=clr, markersize=3, markerstrokewidth=0)
    end
    if TRACK_COMM_EVENTS; overlay_comm_events!(plt_traj, comm); end
    out_traj = joinpath(output_dir, "eval_paths_trajectory.png")
    savefig(plt_traj, out_traj)
    println("  → Saved trajectory: $out_traj")

    # Uncertainty profile
    plt_unc = plot(xlabel="distance traveled (m)", ylabel="uncertainty  det(Σ)^0.25",
                   title="eval paths — uncertainty profile ($(LANDMARK_SCENARIO))",
                   size=(900, 400), legend=:topleft)
    if threshold !== nothing
        hline!(plt_unc, [threshold], color=:red, linestyle=:dot, linewidth=1.5, label="threshold")
    end
    for a in 1:na
        is_prim = is_primary_mask[a]
        clr = is_prim ? :blue : get(agent_colors, a, :gray)
        lbl = is_prim ? "primary" : "support $a"
        plot!(plt_unc, arcs[a], unc_radius.(covs[a]), color=clr,
              linewidth=is_prim ? 2.0 : 1.3, label=lbl)
    end
    out_unc = joinpath(output_dir, "eval_paths_unc_profile.png")
    savefig(plt_unc, out_unc)
    println("  → Saved uncertainty profile: $out_unc")
end
