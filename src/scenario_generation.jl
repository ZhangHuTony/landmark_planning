# ==========================================================================
# Landmark scenario presets (Phase 1: kept as-is; Phase 2 adds
# generate_scenario(goal_distance, landmark_density) for randomized layouts)
# ==========================================================================

function random_landmark_cov()
    # SPD covariance with randomized anisotropy/correlation.
    sx = 0.80 + 0.55 * rand()
    sy = 0.60 + 0.35 * rand()
    ρ = 0.45 * (2 * rand() - 1)
    cxy = ρ * sx * sy
    return [sx^2 cxy; cxy sy^2]
end

# Four landmark scenario configurations

function get_landmarks_single()
    # Single landmark scenario: one landmark at (600, -250)
    return Landmark[
        Landmark(600.0, -250.0, random_landmark_cov())
    ]
end

function get_landmarks_dual()
    # Dual landmark scenario: two landmarks at (250, 200) and (750, -250)
    return Landmark[
        Landmark(700.0, 200.0, random_landmark_cov()*1.1),
        Landmark(750.0, -250.0, random_landmark_cov())
    ]
end

function get_landmarks_clustered()
    # Clustered scenario: 3 landmarks clustered
    return Landmark[
        Landmark(680.0, -200.0, random_landmark_cov()*3),
        Landmark(700.0, -210.0, random_landmark_cov()*3),
        Landmark(720.0, -190.0, random_landmark_cov()*3)
    ]
end

function get_landmarks_shoreline()
    # Shoreline scenario: 5 landmarks along shoreline
    return Landmark[
        Landmark(100.0, -300.0, random_landmark_cov()*5),
        Landmark(300.0, -270.0, random_landmark_cov()*5),
        Landmark(500.0, -240.0, random_landmark_cov()*5),
        Landmark(700.0, -220.0, random_landmark_cov()*5),
        Landmark(900.0, -200.0, random_landmark_cov()*5)
    ]
end

function make_scattered_landmarks(scenario::Symbol=LANDMARK_SCENARIO)
    if scenario == :clustered
        return get_landmarks_clustered()
    elseif scenario == :single
        return get_landmarks_single()
    elseif scenario == :dual
        return get_landmarks_dual()
    elseif scenario == :shoreline
        return get_landmarks_shoreline()
    else
        error("Unknown landmark scenario: $scenario. Use :single, :dual, :clustered, :shoreline.")
    end
end

# Start/goal are baked in per scenario (not config) since each layout was
# designed around a specific corridor length. Start and goal are plain routing
# waypoints — not landmarks, no covariance meaning.
function scenario_endpoints(scenario::Symbol)
    return (0.0, 0.0), (1000.0, 0.0)
end
