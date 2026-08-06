# ==========================================================================
# Core data structures — landmarks and the hex-graph they sit on
# ==========================================================================

struct Landmark
    x::Float64
    y::Float64
    cov::Matrix{Float64}
end

struct LandmarkGraph
    n::Int
    landmarks::Vector{Landmark}
    distance::Matrix{Float64}
    orientation::Matrix{Float64}
    neighbors::Vector{Vector{Int}}   # sparse adjacency list (empty = fully connected)
    shortest_paths::Matrix{Float64}  # Floyd-Warshall: shortest graph distance between all pairs
end

# ==========================================================================
# Graph construction utilities
# ==========================================================================

# Floyd-Warshall: compute shortest-path distances between all pairs via graph edges
function floyd_warshall(dist::Matrix{Float64}, neighbors::Vector{Vector{Int}})
    n = size(dist, 1)
    # Initialize with edge weights (or Inf if no edge)
    sp = fill(Inf, n, n)
    for i in 1:n
        sp[i, i] = 0.0
    end
    for i in 1:n
        for j in neighbors[i]
            sp[i, j] = dist[i, j]
        end
    end
    # Standard Floyd-Warshall: O(n³)
    for k in 1:n
        for i in 1:n
            for j in 1:n
                if sp[i, k] + sp[k, j] < sp[i, j]
                    sp[i, j] = sp[i, k] + sp[k, j]
                end
            end
        end
    end
    return sp
end

function generate_graph(landmarks::Vector{Landmark}; neighbors::Vector{Vector{Int}}=Vector{Int}[])
    n = length(landmarks)
    dist   = zeros(n, n)
    orient = zeros(n, n)
    for (i, li) in enumerate(landmarks)
        for (j, lj) in enumerate(landmarks)
            dx = lj.x - li.x; dy = lj.y - li.y
            dist[i,j]   = sqrt(dx^2 + dy^2)
            orient[i,j] = atan(dy, dx)
        end
    end
    adj = isempty(neighbors) ? [collect(filter(j->j!=i, 1:n)) for i in 1:n] : neighbors
    sp = floyd_warshall(dist, adj)
    return LandmarkGraph(n, landmarks, dist, orient, adj, sp)
end

# ==========================================================================
# Hex-grid world (pointy-top) with heading-aware transitions
# ==========================================================================
# Motion primitives (pointy-top hex, 6 discrete headings at 60° intervals):
#   action 0: forward
#   action 1: forward-left  (turn left by 60°, then move)
#   action 2: forward-right (turn right by 60°, then move)
#
# Each graph node is (hex_cell, heading), so support/primary trajectories are
# constrained by vehicle heading, not just geometric adjacency.

const HEX_HEADINGS = 6

const HEX_EVEN_ROW_DELTAS = [
    (1, 0), (0, 1), (-1, 1),
    (-1, 0), (-1, -1), (0, -1)
]

const HEX_ODD_ROW_DELTAS = [
    (1, 0), (1, 1), (0, 1),
    (-1, 0), (0, -1), (1, -1)
]

@inline function hex_center(gx::Int, gy::Int, x0::Float64, y0::Float64, hex_r::Float64)
    hex_w = sqrt(3.0) * hex_r
    x = x0 + gx * hex_w + (isodd(gy) ? hex_w / 2 : 0.0)
    y = y0 + gy * (1.5 * hex_r)
    return x, y
end

@inline function nearest_heading_to_goal(start_xy::Tuple{Float64,Float64}, goal_xy::Tuple{Float64,Float64})
    dx = goal_xy[1] - start_xy[1]
    dy = goal_xy[2] - start_xy[2]
    θ = atan(dy, dx)
    # 6 headings equally spaced at 60°
    best_h = 0
    best_err = Inf
    for h in 0:5
        ϕ = h * (π / 3)
        err = abs(atan(sin(θ - ϕ), cos(θ - ϕ)))
        if err < best_err
            best_err = err
            best_h = h
        end
    end
    return best_h
end

function build_hex_graph(sensor_landmarks::Vector{Landmark},
                         start_pos::Tuple{Float64,Float64},
                         goal_pos::Tuple{Float64,Float64};
                         hex_r::Float64 = HEX_RADIUS_M,
                         padding::Int = HEX_PADDING)
    # Keep the routing graph focused on the start→goal corridor but with ample y-extent.
    xmin = min(start_pos[1], goal_pos[1])
    xmax = max(start_pos[1], goal_pos[1])
    ymin = min(start_pos[2], goal_pos[2])
    ymax = max(start_pos[2], goal_pos[2])

    # Artificially expand y-extent to allow meaningful lateral planning detours.
    # Wider corridor lets the planner reach off-axis landmark clusters.
    y_expansion = CORRIDOR_HALFWIDTH_M
    ymin -= y_expansion
    ymax += y_expansion

    hex_w = sqrt(3.0) * hex_r
    y_step = 1.5 * hex_r

    grid_w = max(3, Int(ceil((xmax - xmin) / hex_w)) + 1 + 2 * padding)
    grid_h = max(3, Int(ceil((ymax - ymin) / y_step)) + 1 + 2 * padding)
    iseven(grid_h) && (grid_h += 1)

    x0 = xmin - padding * hex_w
    # Center the hex rows around y=0 so the corridor is visually symmetric.
    y0 = -0.5 * (grid_h - 1) * y_step

    cells = Tuple{Int,Int}[]
    centers = Dict{Tuple{Int,Int}, Tuple{Float64,Float64}}()
    for gy in 0:grid_h-1
        # Filter: skip rows above CORRIDOR_Y_MAX_M and skip lowest y row (gy == 0)
        cy = y0 + gy * y_step
        if cy > CORRIDOR_Y_MAX_M || gy == 0
            continue
        end
        
        for gx in 1:(grid_w-2)  # Skip first and last columns (left and right edges)
            cell = (gx, gy)
            centers[cell] = hex_center(gx, gy, x0, y0, hex_r)
            push!(cells, cell)
        end
    end

    function nearest_cell(pos::Tuple{Float64,Float64})
        best_cell = cells[1]
        best_d2 = Inf
        for c in cells
            cc = centers[c]
            d2 = (pos[1] - cc[1])^2 + (pos[2] - cc[2])^2
            if d2 < best_d2
                best_d2 = d2
                best_cell = c
            end
        end
        return best_cell
    end

    start_cell = nearest_cell(start_pos)
    goal_cell = nearest_cell(goal_pos)
    start_heading = nearest_heading_to_goal(start_pos, goal_pos)

    # Enumerate heading-expanded routing states; force start state to index 1.
    route_states = Tuple{Tuple{Int,Int}, Int}[]  # ((gx,gy), heading0..5)
    start_state = (start_cell, start_heading)
    push!(route_states, start_state)
    for c in cells
        for h in 0:5
            st = (c, h)
            st == start_state && continue
            push!(route_states, st)
        end
    end

    route_idx = Dict{Tuple{Tuple{Int,Int}, Int}, Int}()
    for (i, st) in enumerate(route_states)
        route_idx[st] = i
    end

    n_route = length(route_states)
    n_sensor = length(sensor_landmarks)
    goal_idx = n_route + n_sensor + 1  # force goal to last node (graph.n)
    n_total = goal_idx

    null_cov = 1e-9 * Matrix{Float64}(I, 2, 2)
    all_lms = Vector{Landmark}(undef, n_total)

    # Routing nodes
    for (i, st) in enumerate(route_states)
        c, _ = st
        cx, cy = centers[c]
        all_lms[i] = Landmark(cx, cy, copy(null_cov))
    end
    # Start covariance should match the original start prior.
    all_lms[1] = Landmark(start_pos[1], start_pos[2], copy(sensor_landmarks[1].cov))

    # Sensor landmarks are appended as static observation sources (not routing nodes).
    sensor_offset = n_route
    for i in 1:n_sensor
        all_lms[sensor_offset + i] = sensor_landmarks[i]
    end

    # Terminal goal node (routing only)
    all_lms[goal_idx] = Landmark(goal_pos[1], goal_pos[2], copy(null_cov))

    neighbors = [Int[] for _ in 1:n_total]

    # Heading-constrained transitions:
    # forward (0), forward-left (-1), forward-right (+1)
    turn_options = (0, -1, +1)
    for st in route_states
        c, h = st
        from_idx = route_idx[st]
        cx, cy = c
        deltas = iseven(cy) ? HEX_EVEN_ROW_DELTAS : HEX_ODD_ROW_DELTAS
        for turn in turn_options
            nh = mod(h + turn, 6)
            dx, dy = deltas[nh + 1]
            nc = (cx + dx, cy + dy)
            if haskey(centers, nc)
                to_idx = route_idx[(nc, nh)]
                push!(neighbors[from_idx], to_idx)
            end
        end
    end

    # Any heading at goal cell can transition to terminal goal node.
    for h in 0:5
        gstate = (goal_cell, h)
        if haskey(route_idx, gstate)
            push!(neighbors[route_idx[gstate]], goal_idx)
        end
    end

    # Pairwise geometric matrices used by existing covariance propagation code.
    dist = zeros(n_total, n_total)
    orient = zeros(n_total, n_total)
    for i in 1:n_total
        xi = all_lms[i].x; yi = all_lms[i].y
        for j in 1:n_total
            dx = all_lms[j].x - xi
            dy = all_lms[j].y - yi
            dist[i, j] = hypot(dx, dy)
            orient[i, j] = atan(dy, dx)
        end
    end

    sp = floyd_warshall(dist, neighbors)
    n_edges = sum(length.(neighbors))
    println("Hex graph: $(n_total) nodes ($(n_route) heading-states + $(n_sensor) sensors + goal), ",
            "radius=$(hex_r)m, directed_edges=$(n_edges), grid=$(grid_w)x$(grid_h), ",
            "start_cell=$(start_cell), goal_cell=$(goal_cell), start_heading=$(start_heading)")

    return LandmarkGraph(n_total, all_lms, dist, orient, neighbors, sp)
end

function node_role_masks(graph::LandmarkGraph)
    n = graph.n
    indeg = zeros(Int, n)
    for i in 1:n
        for j in graph.neighbors[i]
            indeg[j] += 1
        end
    end
    route_mask = falses(n)
    sensor_mask = falses(n)
    for i in 1:n
        if !isempty(graph.neighbors[i]) || indeg[i] > 0
            route_mask[i] = true
        else
            sensor_mask[i] = true
        end
    end
    return route_mask, sensor_mask
end

function route_tile_centers(graph::LandmarkGraph)
    route_mask, _ = node_role_masks(graph)
    s = Set{Tuple{Float64,Float64}}()
    for i in 1:graph.n
        route_mask[i] || continue
        push!(s, (graph.landmarks[i].x, graph.landmarks[i].y))
    end
    return collect(s)
end

function set_hex_world_limits!(plt, graph::LandmarkGraph)
    centers = route_tile_centers(graph)
    _, sensor_mask = node_role_masks(graph)
    sensor_idx = findall(sensor_mask)

    xs = Float64[]
    ys = Float64[]

    for c in centers
        push!(xs, c[1]); push!(ys, c[2])
    end
    for i in sensor_idx
        push!(xs, graph.landmarks[i].x)
        push!(ys, graph.landmarks[i].y)
    end

    isempty(xs) && return
    m = max(HEX_RADIUS_M, 30.0)
    xlims!(plt, minimum(xs) - m, maximum(xs) + m)
    ylims!(plt, minimum(ys) - m, maximum(ys) + m)
end

function draw_hex_tiles!(plt, graph::LandmarkGraph;
                         fill_color=:lightgrey,
                         line_color=:white,
                         fill_alpha::Float64=0.98,
                         line_alpha::Float64=1.0)
    centers = route_tile_centers(graph)
    # Pointy-top orientation: vertices at 30° + k*60°
    θ = [π/6 + k*(π/3) for k in 0:6]
    for (cx, cy) in centers
        xs = [cx + HEX_RADIUS_M * cos(t) for t in θ]
        ys = [cy + HEX_RADIUS_M * sin(t) for t in θ]
        plot!(plt, xs, ys, seriestype=:shape, color=fill_color,
              alpha=fill_alpha, linecolor=line_color, linealpha=line_alpha,
              linewidth=1.0, label=false)
    end
end
