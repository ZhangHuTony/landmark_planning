# ==========================================================================
# Plotting + path CSV I/O — shared by every planner
# ==========================================================================

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
    return p
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

function write_comm_csv(filename::String, events)
    open(filename, "w") do io
        println(io, "arc_dist,agent_a,agent_b,weight,xa,ya,xb,yb")
        for (t, a, b, w, pa, pb) in events
            println(io, "$t,$a,$b,$w,$(pa[1]),$(pa[2]),$(pb[1]),$(pb[2])")
        end
    end
    println("  → Saved $(length(events)) comm events to $filename")
end
