# ==========================================================================
# Uncertainty metrics and covariance math helpers
# ==========================================================================

@inline function unc_det_radius(cov::Matrix{Float64})
    d = cov[1,1] * cov[2,2] - cov[1,2] * cov[2,1]
    return max(d, 1e-18)^(0.25)
end

# Primary scalar uncertainty used by planning/constraints/reporting.
unc_radius(cov::Matrix{Float64}) = unc_det_radius(cov)


@inline function unc_within_threshold(unc::Float64, threshold::Float64, tol::Float64)
    return unc <= threshold + tol
end

@inline function unc_exceeds_threshold(unc::Float64, threshold::Float64, tol::Float64)
    return unc > threshold + tol
end

# Covariance partial order for sound dominance pruning:
# cov_a dominates cov_b iff (cov_b - cov_a) is positive semidefinite.
@inline function cov_dominates(cov_a::Matrix{Float64}, cov_b::Matrix{Float64}; tol::Float64=1e-9)
    # Check PSD for symmetric 2x2 matrix D = cov_b - cov_a via principal minors.
    d11 = cov_b[1,1] - cov_a[1,1]
    d12 = cov_b[1,2] - cov_a[1,2]
    d22 = cov_b[2,2] - cov_a[2,2]
    det = d11 * d22 - d12 * d12
    return d11 >= -tol && d22 >= -tol && det >= -tol
end

# Inline: R * diag(sd², sp²) * R' expanded to avoid intermediate allocations
@inline function growth_covariance(distance::Float64, angle::Float64)
    sd2 = (DIR_UNCERTAINTY_PER_METER  * distance)^2
    sp2 = (PERP_UNCERTAINTY_PER_METER * distance)^2
    c = cos(angle); s = sin(angle)
    diff = sd2 - sp2
    return [c*c*sd2 + s*s*sp2  c*s*diff;
            c*s*diff            s*s*sd2 + c*c*sp2]
end

# Inline 2×2 matrix inverse — avoids LinearAlgebra.inv overhead for small matrices
@inline function inv2(M::Matrix{Float64})
    a=M[1,1]; b=M[1,2]; c=M[2,1]; d=M[2,2]
    det = a*d - b*c
    inv_det = 1.0 / det
    return [d*inv_det  -b*inv_det; -c*inv_det  a*inv_det]
end

# Information-filter fuse: Σ_new = (Σ_a⁻¹ + Σ_b⁻¹)⁻¹  — two 2×2 inverses only
@inline function fuse_cov(A::Matrix{Float64}, B::Matrix{Float64})
    return inv2(inv2(A) + inv2(B))
end

# ==========================================================================
# Physics-accurate covariance fusion
# ==========================================================================
# landmark_measurement_cov(agent_pos, lm, heading)
#   Returns the effective measurement noise covariance of observing `lm` from
#   `agent_pos` while heading in direction `heading`.
#
#   Model:
#     σ_range   = SENSOR_NOISE  (along line-of-sight)
#     σ_bearing = SENSOR_NOISE * BEARING_NOISE_RATIO  (perpendicular to LOS)
#   These are rotated into world frame by the bearing angle to the landmark.
#   The covariance is then INFLATED by 1/q so that a low-quality
#   observation contributes less to the fusion — a barely-visible landmark
#   effectively has much higher noise.

# Returns the inverse of the measurement noise matrix (information form) directly,
# avoiding the allocation of S and then inv(S) separately.
# Returns nothing if landmark is outside detection range.
@inline function landmark_info(ax::Float64, ay::Float64, lm::Landmark)
    dx = lm.x - ax; dy = lm.y - ay
    d2 = dx*dx + dy*dy

    # Sensing quality: logistic sigmoid, plateaus at ~1 within VISIBILITY_RANGE,
    # smoothly rolls off over VISIBILITY_WIDTH (no hard cutoff -> clean gradients).
    d = sqrt(d2)
    q = 1.0 / (1.0 + exp((d - VISIBILITY_RANGE) / VISIBILITY_WIDTH))
    q < 1e-6 && return nothing  # far-tail early-out; contribution already negligible

    # Bearing angle to landmark (world frame) — R*D*R' expanded inline
    bearing = atan(dy, dx)
    cb = cos(bearing); sb = sin(bearing)
    σ_r2 = SENSOR_NOISE^2
    σ_b2 = (SENSOR_NOISE * BEARING_NOISE_RATIO)^2
    diff = σ_r2 - σ_b2
    # S_sensor = R*diag(σ_r2,σ_b2)*R' inline
    s11 = cb*cb*σ_r2 + sb*sb*σ_b2
    s12 = cb*sb*diff
    s22 = sb*sb*σ_r2 + cb*cb*σ_b2

    # S_total = S_sensor + lm.cov, inflated by 1/q
    inv_p = 1.0 / q
    t11 = (s11 + lm.cov[1,1]) * inv_p
    t12 = (s12 + lm.cov[1,2]) * inv_p
    t22 = (s22 + lm.cov[2,2]) * inv_p

    # Return inv(S_total) directly (2×2 inline inverse)
    det = t11*t22 - t12*t12
    det < 1e-14 && return nothing
    inv_det = 1.0 / det
    return (t22*inv_det, -t12*inv_det, t11*inv_det, q)  # (I11, I12, I22, quality) of information matrix
end

# Accumulate information matrix contributions from all visible landmarks at (ax, ay).
# Returns (I11, I12, I22) — the summed information matrix to add to the prior.
# Returns (0, 0, 0) when no landmark is close enough to contribute.
# Shared by propagate_cov_discrete and propagate_cov_continuous.
# `landmark_events`, if given, gets one (arc, agent, landmark_id, quality,
# agent_pos, landmark_pos) entry pushed per contributing landmark — the
# landmark-fusion analog of apply_synchronized_propagation!'s comm_events.
@inline function accumulate_landmark_info(ax::Float64, ay::Float64, lms::Vector{Landmark};
                                           landmark_events=nothing, agent::Int=0, arc::Float64=0.0)
    I11 = 0.0; I12 = 0.0; I22 = 0.0
    for (li, lm) in enumerate(lms)
        unc_radius(lm.cov) < 1e-8 && continue
        info = landmark_info(ax, ay, lm)
        info === nothing && continue
        I11 += info[1]; I12 += info[2]; I22 += info[3]
        if landmark_events !== nothing
            push!(landmark_events, (arc, agent, li, info[4], (ax, ay), (lm.x, lm.y)))
        end
    end
    return I11, I12, I22
end

# Apply an information-filter Kalman update to covariance `cov` using the
# accumulated information (I11, I12, I22) from visible landmarks.
# Returns the updated 2×2 covariance matrix, or `nothing` if the prior or
# posterior covariance is numerically degenerate (det < 1e-20).
# Shared by propagate_cov_discrete and propagate_cov_continuous.
@inline function kalman_info_update(cov::Matrix{Float64}, I11::Float64, I12::Float64, I22::Float64)
    det_c = cov[1,1]*cov[2,2] - cov[1,2]*cov[2,1]
    abs(det_c) < 1e-20 && return nothing
    inv_det = 1.0 / det_c
    J11 = I11 + cov[2,2]*inv_det
    J12 = I12 - cov[1,2]*inv_det
    J22 = I22 + cov[1,1]*inv_det
    det_j = J11*J22 - J12*J12
    abs(det_j) < 1e-20 && return nothing
    inv_dj = 1.0 / det_j
    return [J22*inv_dj  -J12*inv_dj; -J12*inv_dj  J11*inv_dj]
end

# propagate_cov_discrete: covariance propagation at discrete waypoints
# Input: positions (x,y) at waypoints
# Outputs: covariance at each waypoint after dead-reckoning + landmark fusion

# Propagate covariance over waypoints from_idx+1 .. to_idx, starting from init_cov.
# Writes results into covs[from_idx+1 .. to_idx] and returns the covariance at to_idx.
function propagate_segment!(covs::Vector{Matrix{Float64}},
                             positions::Vector{Tuple{Float64,Float64}},
                             lms::Vector{Landmark},
                             from_idx::Int,
                             to_idx::Int,
                             init_cov::Matrix{Float64};
                             landmark_events=nothing, agent::Int=0,
                             arcs::Union{Vector{Float64},Nothing}=nothing)
    cov = init_cov
    for i in (from_idx + 1):to_idx
        x_prev, y_prev = positions[i-1]
        x_curr, y_curr = positions[i]
        seg     = hypot(x_curr - x_prev, y_curr - y_prev)
        heading = atan(y_curr - y_prev, x_curr - x_prev)
        cov = cov + growth_covariance(seg, heading)
        arc_i = arcs === nothing ? 0.0 : arcs[i]
        I11, I12, I22 = accumulate_landmark_info(x_curr, y_curr, lms;
                                                  landmark_events=landmark_events, agent=agent, arc=arc_i)
        if I11 > 0.0 || I22 > 0.0
            updated = kalman_info_update(cov, I11, I12, I22)
            if updated !== nothing
                cov = updated
            end
        end
        covs[i] = copy(cov)
    end
    return cov
end

function propagate_cov_discrete(positions::Vector{Tuple{Float64,Float64}},
                                 lms::Vector{Landmark},
                                 init_cov::Matrix{Float64};
                                 debug_goal_pos::Union{Tuple{Float64,Float64}, Nothing} = nothing,
                                 debug_agent_id::Union{Int, Nothing} = nothing)
    n    = length(positions)
    covs = Vector{Matrix{Float64}}(undef, n)
    covs[1] = copy(init_cov)
    propagate_segment!(covs, positions, lms, 1, n, copy(init_cov))
    return covs
end

# Continuous covariance propagation along a B-spline path
# Input: xs, ys (coordinate arrays), lms (landmarks), init_cov (initial covariance)
# Output: covs (covariance at each sample point along path)
function propagate_cov_continuous(xs::Vector{Float64},
                                   ys::Vector{Float64},
                                   lms::Vector{Landmark},
                                   init_cov::Matrix{Float64};
                                   explicit_headings::Union{Vector{Float64}, Nothing} = nothing)
    n    = length(xs)
    covs = Vector{Matrix{Float64}}(undef, n)
    cov  = copy(init_cov)
    covs[1] = copy(cov)

    for i in 2:n
        x_prev, y_prev = xs[i-1], ys[i-1]
        x_curr, y_curr = xs[i], ys[i]
        
        # Segment distance and heading
        seg = hypot(x_curr - x_prev, y_curr - y_prev)
        if seg < 1e-10
            covs[i] = copy(cov)
            continue
        end
        heading = explicit_headings !== nothing ? explicit_headings[i] : atan(y_curr - y_prev, x_curr - x_prev)
        
        # 1. Dead-reckoning growth
        cov = cov + growth_covariance(seg, heading)
        
        # 2. Landmark fusion at current position
        I11, I12, I22 = accumulate_landmark_info(x_curr, y_curr, lms)
        if I11 > 0.0 || I22 > 0.0
            # Information filter update
            updated = kalman_info_update(cov, I11, I12, I22)
            if updated === nothing
                covs[i] = copy(cov); continue
            end
            cov = updated
        end
        covs[i] = copy(cov)
    end
    return covs
end

# Helper function to convert (xs, ys) lists to position tuples
function xs_ys_to_positions(xs::Vector{Vector{Float64}}, ys::Vector{Vector{Float64}})
    na = length(xs)
    agent_positions = Vector{Vector{Tuple{Float64,Float64}}}(undef, na)
    for a in 1:na
        positions = Vector{Tuple{Float64,Float64}}(undef, length(xs[a]))
        for i in 1:length(xs[a])
            positions[i] = (xs[a][i], ys[a][i])
        end
        agent_positions[a] = positions
    end
    return agent_positions
end

# ==========================================================================
# Discrete multi-agent covariance evaluation
# ==========================================================================
# Evaluate all agents at their waypoint positions with synchronized Kalman communication

function evaluate_joint_discrete(agent_positions::Vector{Vector{Tuple{Float64,Float64}}},
                                  lms::Vector{Landmark},
                                  na::Int;
                                  debug_goal_pos::Union{Tuple{Float64,Float64}, Nothing} = nothing)
    # Evaluate covariance for each agent at their waypoint positions with synchronized comms
    # All agents propagate in lock-step, fusing at fixed distance intervals
    
    # Compute arc lengths for all agents first
    all_arcs = Vector{Vector{Float64}}(undef, na)
    
    for a in 1:na
        if isempty(agent_positions[a])
            all_arcs[a] = [0.0]
        else
            arcs = Vector{Float64}(undef, length(agent_positions[a]))
            arcs[1] = 0.0
            for i in 2:length(agent_positions[a])
                x0, y0 = agent_positions[a][i-1]
                x1, y1 = agent_positions[a][i]
                arcs[i] = arcs[i-1] + hypot(x1-x0, y1-y0)
            end
            all_arcs[a] = arcs
        end
    end
    
    # Propagate covariances with synchronized Kalman fusion
    all_covs, comm_events, landmark_events = apply_synchronized_propagation!(agent_positions, all_arcs, lms, na;
                                                            debug_goal_pos=debug_goal_pos)

    return all_covs, all_arcs, comm_events, landmark_events
end

function apply_synchronized_propagation!(agent_positions::Vector{Vector{Tuple{Float64,Float64}}},
                                        all_arcs::Vector{Vector{Float64}},
                                        lms::Vector{Landmark},
                                        na::Int;
                                        debug_goal_pos::Union{Tuple{Float64,Float64}, Nothing} = nothing)
    # Interleaved propagation + comm fusion: propagate each agent up to the next
    # comm checkpoint, fuse, then continue from the post-fusion covariance.
    # This ensures a reduced covariance after comm event k seeds propagation
    # toward event k+1, matching real cooperative-localization behaviour.

    all_covs = Vector{Vector{Matrix{Float64}}}(undef, na)
    for a in 1:na
        if isempty(agent_positions[a])
            all_covs[a] = [copy(lms[1].cov)]
        else
            all_covs[a] = Vector{Matrix{Float64}}(undef, length(agent_positions[a]))
            all_covs[a][1] = copy(lms[1].cov)
        end
    end

    # Cursors: current waypoint index and running covariance per agent
    cur_idx = ones(Int, na)
    cur_cov = [copy(lms[1].cov) for _ in 1:na]

    max_arc   = maximum(arcs[end] for arcs in all_arcs)
    comm_times = 0.0:COMM_INTERVAL:max_arc
    comm_events = Tuple{Float64,Int,Int,Float64,Tuple{Float64,Float64},Tuple{Float64,Float64}}[]
    landmark_events = Tuple{Float64,Int,Int,Float64,Tuple{Float64,Float64},Tuple{Float64,Float64}}[]

    for comm_time in comm_times
        # --- Step 1: advance each agent to its nearest waypoint for this checkpoint ---
        agent_indices = Vector{Int}(undef, na)
        for a in 1:na
            nearest_idx = 1
            min_diff = abs(all_arcs[a][1] - comm_time)
            for i in 2:length(all_arcs[a])
                diff = abs(all_arcs[a][i] - comm_time)
                if diff < min_diff
                    min_diff = diff
                    nearest_idx = i
                end
            end
            # Propagate the segment (cur_idx[a], nearest_idx] starting from cur_cov[a]
            if nearest_idx > cur_idx[a]
                cur_cov[a] = propagate_segment!(all_covs[a], agent_positions[a], lms,
                                                cur_idx[a], nearest_idx, cur_cov[a];
                                                landmark_events=landmark_events, agent=a, arcs=all_arcs[a])
                cur_idx[a] = nearest_idx
            end
            agent_indices[a] = nearest_idx
        end

        # --- Step 2: pairwise Kalman fusion at this checkpoint (unchanged algebra) ---
        for sender in 1:na
            idx_s = agent_indices[sender]
            pos_s = agent_positions[sender][idx_s]
            for receiver in sender+1:na
                idx_r = agent_indices[receiver]
                pos_r = agent_positions[receiver][idx_r]
                dx = pos_s[1] - pos_r[1]
                dy = pos_s[2] - pos_r[2]
                d2_comm = dx*dx + dy*dy
                weight = comm_weight(d2_comm)
                if weight > COMM_WEIGHT_MIN
                    S_s = all_covs[sender][idx_s] + SENSOR_NOISE^2 * I(2)
                    S_r = all_covs[receiver][idx_r] + SENSOR_NOISE^2 * I(2)
                    inv_P_r = inv(all_covs[receiver][idx_r])
                    new_inv_P_r = inv_P_r + weight * inv(S_s)
                    all_covs[receiver][idx_r] = inv(new_inv_P_r)
                    inv_P_s = inv(all_covs[sender][idx_s])
                    new_inv_P_s = inv_P_s + weight * inv(S_r)
                    all_covs[sender][idx_s] = inv(new_inv_P_s)
                    push!(comm_events, (comm_time, sender, receiver, weight, pos_s, pos_r))
                end
            end
        end

        # Refresh running covariances to post-fusion values so next segment starts there
        for a in 1:na
            cur_cov[a] = all_covs[a][cur_idx[a]]
        end
    end

    # Propagate any remaining waypoints beyond the last comm checkpoint
    for a in 1:na
        n_a = length(agent_positions[a])
        if cur_idx[a] < n_a
            propagate_segment!(all_covs[a], agent_positions[a], lms,
                               cur_idx[a], n_a, cur_cov[a];
                               landmark_events=landmark_events, agent=a, arcs=all_arcs[a])
        end
    end

    return all_covs, comm_events, landmark_events
end

function edge_cov_continuous(v::Int, u::Int,
                              graph::LandmarkGraph,
                              lms::Vector{Landmark},
                              cov_in::Matrix{Float64})
    # Direct covariance propagation along path edge
    vx = graph.landmarks[v].x; vy = graph.landmarks[v].y
    ux = graph.landmarks[u].x; uy = graph.landmarks[u].y
    h_bearing = atan(uy - vy, ux - vx)
    
    # Sample along straight line between waypoints
    xs = [vx, ux]
    ys = [vy, uy]
    covs = propagate_cov_continuous(xs, ys, lms, cov_in; explicit_headings=[h_bearing, h_bearing])
    return covs[end]
end

# Inter-agent comm quality: logistic sigmoid on distance, half-weight at
# COMM_RANGE, transition softness COMM_WIDTH (no hard upper cutoff -> clean
# gradients). Co-located agents fuse at full weight: proximity doesn't
# correlate their measurements, so there's no reason to floor it out.
@inline function comm_weight(d2::Float64)
    d = sqrt(d2)
    return 1.0 / (1.0 + exp((d - COMM_RANGE) / COMM_WIDTH))
end

@inline function pairwise_comm(cov_a::Matrix{Float64}, cov_b::Matrix{Float64},
                                xa::Float64, ya::Float64,
                                xb::Float64, yb::Float64)
    d2 = (xa-xb)^2 + (ya-yb)^2
    w  = comm_weight(d2)
    w < COMM_WEIGHT_MIN && return cov_a, cov_b
    # Information-filter fusion weighted by range
    noise = SENSOR_NOISE^2
    # a receives from b
    Ib = inv2(cov_b .+ noise .* [1.0 0.0; 0.0 1.0])
    new_a = inv2(inv2(cov_a) .+ w .* Ib)
    # b receives from a
    Ia = inv2(cov_a .+ noise .* [1.0 0.0; 0.0 1.0])
    new_b = inv2(inv2(cov_b) .+ w .* Ia)
    return new_a, new_b
end

# ------------------------------------------------------------------
# Evaluate complete paths using discrete waypoint positions
# Input:  agent_paths (node sequences), graph, landmarks, num_agents
# Output: (covs, dists) where each is a vector per agent
#
# This function:
#   1. Converts each node path to (x,y) waypoints
#   2. Evaluates joint covariance via evaluate_joint_discrete (includes inter-agent comms)
#   3. Computes path distances as sum of edge distances
# ------------------------------------------------------------------
function evaluate_full_paths(agent_paths::Vector{Vector{Int}},
                              graph::LandmarkGraph,
                              lms::Vector{Landmark},
                              na::Int)
    # Convert paths to waypoint positions
    all_xs = Vector{Float64}[]
    all_ys = Vector{Float64}[]
    all_dists = Vector{Float64}[]
    
    for a in 1:na
        path = agent_paths[a]
        if isempty(path)
            push!(all_xs, Float64[])
            push!(all_ys, Float64[])
            push!(all_dists, Float64[])
        else
            xs = [graph.landmarks[i].x for i in path]
            ys = [graph.landmarks[i].y for i in path]
            push!(all_xs, xs)
            push!(all_ys, ys)
            
            # Compute cumulative distances along path
            dists = [0.0]
            for i in 2:length(path)
                edge_dist = graph.distance[path[i-1], path[i]]
                push!(dists, dists[end] + edge_dist)
            end
            push!(all_dists, dists)
        end
    end
    
    # Evaluate joint covariance via evaluate_joint_discrete (includes inter-agent communication)
    agent_positions = xs_ys_to_positions(all_xs, all_ys)
    all_covs, _, _, _ = evaluate_joint_discrete(agent_positions, lms, na)
    
    # Extract final covariances and distances
    final_covs = [all_covs[a][end] for a in 1:na]
    final_dists = [all_dists[a][end] for a in 1:na]
    
    return final_covs, final_dists
end
