# ==========================================================================
# Plotting + path CSV I/O — shared by every planner
# ==========================================================================

# Raw-data CSVs and diagnostic figures live in their own subfolders under a
# run's output_dir (results.yaml stays at the top level as the run's
# manifest) so the two kinds of output are easy to tell apart at a glance.
function csv_path(output_dir::String, filename::String)
    dir = joinpath(output_dir, "csv")
    mkpath(dir)
    return joinpath(dir, filename)
end

function fig_path(output_dir::String, filename::String)
    dir = joinpath(output_dir, "figures")
    mkpath(dir)
    return joinpath(dir, filename)
end

function draw_covariance_ellipse!(plt, x, y, cov; npts=50, nstd=2, color=:red, alpha=0.3, display_scale=1.0)
    # display_scale is visualization-only (does not change estimation math)
    cov_vis = display_scale .* cov
    vals, vecs = eigen(Symmetric((cov_vis + cov_vis') / 2))
    a = nstd*sqrt(max(vals[1],0.0)); b = nstd*sqrt(max(vals[2],0.0))
    angle = atan(vecs[2,1], vecs[1,1]); θ = range(0, 2π, length=npts)
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)]
    pts = R * vcat((a.*cos.(θ))', (b.*sin.(θ))')
    plot!(plt, x.+pts[1,:], y.+pts[2,:], seriestype=:shape, color=color, alpha=alpha, label=false)
end

agent_colors = [:purple, :teal, :darkorange, :crimson, :magenta,
                :brown, :lime, :navy, :coral, :olive]

# Draw static no-go obstacles as filled convex polygons (no-op if none).
function overlay_obstacles!(plt)
    isempty(OBSTACLES) && return
    for (k, obs) in enumerate(OBSTACLES)
        xs = [v[1] for v in obs.verts]; ys = [v[2] for v in obs.verts]
        push!(xs, xs[1]); push!(ys, ys[1])   # close the ring
        plot!(plt, xs, ys, seriestype=:shape, color=:gray35, fillalpha=0.55,
              linecolor=:black, linewidth=1.5, label=(k == 1 ? "Obstacle" : false))
    end
end

function make_base_plot(landmarks, graph)
    _, sensor_mask = node_role_masks(graph)
    p = plot(legend=:outerright, aspect_ratio=:equal)
    draw_hex_tiles!(p, graph; fill_color=:aliceblue, line_color=:cadetblue,
                    fill_alpha=0.98, line_alpha=1.0)

    sensor_idx = findall(sensor_mask)
    if !isempty(sensor_idx)
        lx = [graph.landmarks[i].x for i in sensor_idx]
        ly = [graph.landmarks[i].y for i in sensor_idx]
        scatter!(p, lx, ly, label="Landmarks", color=:black, markersize=4,
                 markerstrokewidth=0)
        for i in sensor_idx
            draw_covariance_ellipse!(p, graph.landmarks[i].x, graph.landmarks[i].y,
                                     graph.landmarks[i].cov, color=:red, alpha=0.25,
                                     display_scale=400.0)
        end
    end

    scatter!(p, [graph.landmarks[1].x], [graph.landmarks[1].y],
             color=:green, markersize=8, markerstrokewidth=0, label="Start")
    scatter!(p, [graph.landmarks[graph.n].x], [graph.landmarks[graph.n].y],
             color=:orange, marker=:star5, markersize=10, markerstrokewidth=0,
             label="Goal")
    set_hex_world_limits!(p, graph)
    overlay_obstacles!(p)
    return p
end

# Plot uncertainty vs. arc-distance per agent, shared by every planner.
# `series` lets a planner overlay multiple stages on one figure (e.g.
# hexspline_cl's discrete seed vs. continuous refinement); a single-stage
# planner just passes one. Each entry is
# (label_suffix, arcs, covs, linestyle, linewidth_primary, linewidth_support).
function plot_unc_profile(series, num_agents::Int; threshold::Union{Float64,Nothing}=nothing,
                           title::String="uncertainty profile")
    plt = plot(xlabel="distance traveled (m)", ylabel="uncertainty  det(Σ)^0.25",
               title=title, size=(900, 400), legend=:topleft)
    if threshold !== nothing
        hline!(plt, [threshold], color=:red, linestyle=:dot, linewidth=1.5, label="threshold")
    end
    for a in 1:num_agents
        is_prim = a == num_agents
        clr = is_prim ? :blue : get(agent_colors, a, :gray)
        agent_lbl = is_prim ? "primary" : "support $a"
        for (suffix, arcs, covs, ls, lw_primary, lw_support) in series
            lbl = isempty(suffix) ? agent_lbl : "$agent_lbl $suffix"
            plot!(plt, arcs[a], unc_radius.(covs[a]), color=clr, linestyle=ls,
                  linewidth=(is_prim ? lw_primary : lw_support), label=lbl)
        end
    end
    return plt
end

function overlay_comm_events!(plt, events)
    isempty(events) && return
    max_t = maximum(t for (t,_,_,_,_,_) in events)
    cmap  = cgrad(:plasma)
    for (t, _, _, w, pa, pb) in events
        clr = cmap[max_t > 0 ? t / max_t : 0.0]
        plot!(plt, [pa[1], pb[1]], [pa[2], pb[2]], color=clr, alpha=w, linewidth=1, label=false)
        scatter!(plt, [pa[1], pb[1]], [pa[2], pb[2]], markershape=:diamond, markersize=5,
                 color=clr, markerstrokewidth=0, label=false)
    end
end

# Landmark-fusion analog of overlay_comm_events! — one entry per (agent,
# landmark) observation used in the Kalman update, colored by when in the
# route it happened.
function overlay_landmark_events!(plt, events)
    isempty(events) && return
    max_t = maximum(t for (t,_,_,_,_,_) in events)
    cmap  = cgrad(:viridis)
    for (t, _, _, q, apos, lpos) in events
        clr = cmap[max_t > 0 ? t / max_t : 0.0]
        plot!(plt, [apos[1], lpos[1]], [apos[2], lpos[2]], color=clr, alpha=q, linewidth=1, label=false)
        scatter!(plt, [apos[1]], [apos[2]], markershape=:utriangle, markersize=5,
                 color=clr, markerstrokewidth=0, label=false)
    end
end

# Saves a base path figure with no event overlay, plus one extra figure per
# enabled event flag (track_comm_events / track_landmark_events) — so each
# kind of event gets its own figure instead of stacking onto one. With both
# flags on this produces 3 files total; both off, just the base. Variant
# files get a "_comm"/"_landmarks" suffix inserted before the extension.
function save_path_figures(plt, base_path::String, comm_events, landmark_events)
    savefig(plt, base_path)
    stem, ext = splitext(base_path)
    if TRACK_COMM_EVENTS
        plt_comm = deepcopy(plt)
        overlay_comm_events!(plt_comm, comm_events)
        savefig(plt_comm, "$(stem)_comm$(ext)")
    end
    if TRACK_LANDMARK_EVENTS
        plt_lm = deepcopy(plt)
        overlay_landmark_events!(plt_lm, landmark_events)
        savefig(plt_lm, "$(stem)_landmarks$(ext)")
    end
end

# Two-panel variant of save_path_figures, for figures that combine a pair
# of plots side-by-side (e.g. hexspline_cl's discrete-vs-continuous compare).
function save_path_figures_pair(plt_left, plt_right, base_path::String,
                                 comm_left, comm_right, landmark_left, landmark_right)
    combine(l, r) = plot(l, r, layout=(1, 2), size=(1200, 500))
    savefig(combine(plt_left, plt_right), base_path)
    stem, ext = splitext(base_path)
    if TRACK_COMM_EVENTS
        l = deepcopy(plt_left); r = deepcopy(plt_right)
        overlay_comm_events!(l, comm_left); overlay_comm_events!(r, comm_right)
        savefig(combine(l, r), "$(stem)_comm$(ext)")
    end
    if TRACK_LANDMARK_EVENTS
        l = deepcopy(plt_left); r = deepcopy(plt_right)
        overlay_landmark_events!(l, landmark_left); overlay_landmark_events!(r, landmark_right)
        savefig(combine(l, r), "$(stem)_landmarks$(ext)")
    end
end

# Parse a saved ctrls CSV (agent,ctrl_index,x,y) back into per-agent control
# points. This is the schema an external/non-Julia baseline planner must
# match to have its paths consumed by the Monte Carlo stage.
function read_ctrls_csv(csv_path::String)
    rows = Dict{Int, Vector{Tuple{Int, Float64, Float64}}}()
    open(csv_path) do io
        readline(io)  # skip header
        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = split(line, ',')
            ai = parse(Int, parts[1])
            ci = parse(Int, parts[2])
            x  = parse(Float64, parts[3])
            y  = parse(Float64, parts[4])
            push!(get!(rows, ai, Tuple{Int, Float64, Float64}[]), (ci, x, y))
        end
    end
    na = maximum(keys(rows))
    @assert na >= 1 && all(haskey(rows, a) for a in 1:na) "CSV must have rows for agents 1..$(na)"
    return [[(x, y) for (_, x, y) in sort(rows[a], by=first)] for a in 1:na]
end

# Save control points (CSV): each row = agent, ctrl_index, x, y
function write_ctrls_csv(filename::String, ctrls::Vector{Vector{Tuple{Float64,Float64}}})
    open(filename, "w") do io
        println(io, "agent,ctrl_index,x,y")
        for (ai, ctrl) in enumerate(ctrls)
            for (ci, pt) in enumerate(ctrl)
                println(io, "$(ai),$(ci),$(pt[1]),$(pt[2])")
            end
        end
    end
    println("  → Saved control points to $filename ($(sum(length.(ctrls))) points)")
end

# Serialize the scenario landmark field (position + 2×2 covariance) so a
# downstream stage — the Monte Carlo consistency simulator, or an external
# non-Julia tool — can reproduce the EXACT landmarks the planner saw without
# re-running the RNG-seeded scenario generator. Σ₀ (initial agent covariance)
# is lms[1].cov, so this also pins the initial uncertainty.
function write_landmarks_csv(filename::String, landmarks::Vector{Landmark})
    open(filename, "w") do io
        println(io, "landmark_id,x,y,c11,c12,c21,c22")
        for (i, lm) in enumerate(landmarks)
            println(io, "$(i),$(lm.x),$(lm.y),$(lm.cov[1,1]),$(lm.cov[1,2]),$(lm.cov[2,1]),$(lm.cov[2,2])")
        end
    end
    println("  → Saved $(length(landmarks)) landmarks to $filename")
end

function read_landmarks_csv(filename::String)
    lms = Landmark[]
    open(filename) do io
        readline(io)  # skip header
        for line in eachline(io)
            isempty(strip(line)) && continue
            p = split(line, ',')
            x = parse(Float64, p[2]); y = parse(Float64, p[3])
            cov = [parse(Float64, p[4]) parse(Float64, p[5]);
                   parse(Float64, p[6]) parse(Float64, p[7])]
            push!(lms, Landmark(x, y, cov))
        end
    end
    return lms
end

function write_comm_csv(filename::String, events)
    open(filename, "w") do io
        println(io, "arc_dist,agent_a,agent_b,weight,xa,ya,xb,yb")
        for (t, a, b, w, pa, pb) in events
            println(io, "$t,$a,$b,$w,$(pa[1]),$(pa[2]),$(pb[1]),$(pb[2])")
        end
    end
    println("  → Saved $(length(events)) comm events to $filename")
end

function write_landmark_csv(filename::String, events)
    open(filename, "w") do io
        println(io, "arc_dist,agent,landmark_id,quality,xa,ya,xl,yl")
        for (t, agent, lm_id, q, apos, lpos) in events
            println(io, "$t,$agent,$lm_id,$q,$(apos[1]),$(apos[2]),$(lpos[1]),$(lpos[2])")
        end
    end
    println("  → Saved $(length(events)) landmark events to $filename")
end
