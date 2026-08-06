# ==========================================================================
# Path evaluation / debug replay — shared across planners
# ==========================================================================
# Replays a saved ctrls CSV (any planner's output, or an externally-produced
# one matching the same schema — see read_ctrls_csv in src/viz.jl) through
# the shared covariance model and plots trajectory + uncertainty profile.
# `threshold` is optional since not every planner has a feasibility bound.
function run_path_eval(input_csv_path::String, landmarks::Vector{Landmark}, graph::LandmarkGraph,
                        output_dir::String; threshold::Union{Float64,Nothing}=nothing)
    println("\n── PATH EVAL MODE ($(input_csv_path)) ──")
    positions = read_ctrls_csv(input_csv_path)
    na = length(positions)

    covs, arcs, comm, landmark_events = evaluate_joint_discrete(positions, landmarks, na)
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
    out_traj = fig_path(output_dir, "eval_paths_trajectory.png")
    save_path_figures(plt_traj, out_traj, comm, landmark_events)
    println("  → Saved trajectory: $out_traj")

    # Uncertainty profile
    out_unc = fig_path(output_dir, "eval_paths_unc_profile.png")
    save_unc_figures([("", arcs, covs, :solid, 2.0, 1.3)], na, out_unc; threshold=threshold,
                     title="eval paths — uncertainty profile ($(LANDMARK_SCENARIO))")
    println("  → Saved uncertainty profile: $out_unc")
end
