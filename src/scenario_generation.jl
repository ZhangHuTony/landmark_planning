# ==========================================================================
# Scenario definitions — landmarks + obstacles + start/goal, all in one place
# ==========================================================================
# A scenario is a NamedTuple `(landmarks, obstacles, start, goal)`. Every
# predefined scenario lives in `SCENARIOS` below, NOT in config: config/main.yaml
# only *names* one via `landmark_scenario:`. The single exception is
# `landmark_scenario: manual`, which reads the whole geometry out of main.yaml
# (`start` / `goal` / `landmarks` / `obstacles`) using the same flat string
# encoding `parse_obstacles` already uses.
#
# ── Corridor geometry (see build_hex_graph) ──
# The hex grid is padded ±260 m about y=0 and rows land on multiples of
# 1.5·hex_r, so at the default `hex_width_m: 100` the reachable rows are
#   y ∈ {-433.0, -346.4, -259.8, -173.2, -86.6, 0.0, 86.6, 173.2, 259.8}
# and x spans start→goal (plus `hex_padding` columns). Two consequences for the
# geometry below:
#   * A wall that must BLOCK a route has to cover a whole row. Hex edges are only
#     sampled at `obstacle_edge_samples` points (2 = endpoints only), so a wall
#     thinner than the 86.6 m row spacing can be stepped straight over.
#   * A landmark only helps within `visibility_range` (100 m default) of a row.

function random_landmark_cov()
    # SPD covariance with randomized anisotropy/correlation.
    sx = 0.80 + 0.55 * rand()
    sy = 0.60 + 0.35 * rand()
    ρ = 0.45 * (2 * rand() - 1)
    cxy = ρ * sx * sy
    return [sx^2 cxy; cxy sy^2]
end

# Obstacles are built with `build_obstacle(verts)` — ordered CONVEX polygon
# vertices, either winding. Pass `Σo=` only when the obstacle's *location* is
# uncertain; it defaults to zero (exactly known).
#
# Scenario table. Each entry is a thunk, not a value: landmark covariances are
# drawn from the RNG, so they must be sampled when the selected scenario is
# built (under generate_plan.jl's seed), not at include time for all 8.
const SCENARIOS = Dict{Symbol, Function}(

    # ── Landmark-layout baselines (no obstacles) ──

    # One landmark, well off the direct line: minimal detour-vs-uncertainty case.
    :single => () -> (landmarks = Landmark[
                          Landmark(600.0, -250.0, random_landmark_cov())],
                      obstacles = Obstacle[],
                      start = (0.0, 0.0), goal = (1000.0, 0.0)),

    # Two landmarks on opposite sides — the primary must pick a side, or the
    # support can take the other one.
    :dual => () -> (landmarks = Landmark[
                        Landmark(700.0,  200.0, random_landmark_cov() * 1.1),
                        Landmark(750.0, -250.0, random_landmark_cov())],
                    # The two obstacles that used to live in config/main.yaml's
                    # `obstacles:` key: a box left of the corridor mid-point and
                    # an uncertain-location triangle above it.
                    obstacles = Obstacle[
                        build_obstacle([(200.0,-150.0), (260.0,-150.0), (260.0,-100.0), (200.0,-100.0)]),
                        build_obstacle([(400.0,60.0), (460.0,60.0), (430.0,120.0)];
                                       Σo = [4.0 0.0; 0.0 4.0])],
                    start = (0.0, 0.0), goal = (1000.0, 0.0)),

    # Three landmarks in a tight cluster: one detour buys three fixes. This is
    # the scenario mc_nees.jl's ci-vs-kf gate uses (repeated in-range comms).
    :clustered => () -> (landmarks = Landmark[
                             Landmark(680.0, -200.0, random_landmark_cov() * 3),
                             Landmark(700.0, -210.0, random_landmark_cov() * 3),
                             Landmark(720.0, -190.0, random_landmark_cov() * 3)],
                         obstacles = Obstacle[],
                         start = (0.0, 0.0), goal = (1000.0, 0.0)),

    # Landmarks strung along a "shore": the informative route is a long shallow
    # arc rather than a single detour.
    :shoreline => () -> (landmarks = Landmark[
                             Landmark(100.0, -300.0, random_landmark_cov() * 5),
                             Landmark(300.0, -270.0, random_landmark_cov() * 5),
                             Landmark(500.0, -240.0, random_landmark_cov() * 5),
                             Landmark(700.0, -220.0, random_landmark_cov() * 5),
                             Landmark(900.0, -200.0, random_landmark_cov() * 5)],
                         obstacles = Obstacle[],
                         start = (0.0, 0.0), goal = (1000.0, 0.0)),

    # ── Stress scenarios ──

    # TWO HOMOTOPIES. A long central island (covering rows -86.6…86.6) splits the
    # corridor into a short blind route (south) and a longer informative one
    # (north, swinging out to y≈250 for both fixes). Couples obstacle avoidance to
    # sensing: the south route is only ~1200 m but goes blind, so it never meets
    # unc_radius_threshold and the northern detour is forced.
    # Measured: the front collapses to ONE seed — the south homotopy is infeasible
    # rather than a cheaper trade-off, so this does not exercise a multi-point
    # Pareto front. Adding a southern landmark to make it feasible was tried and
    # changed nothing (the collector still returned one seed).
    :two_routes => () -> (landmarks = Landmark[
                              Landmark(500.0, 250.0, random_landmark_cov() * 2),
                              Landmark(760.0, 250.0, random_landmark_cov() * 2)],
                          obstacles = Obstacle[
                              build_obstacle([(400.0,-140.0), (800.0,-140.0), (800.0,140.0), (400.0,140.0)])],
                          start = (0.0, 0.0), goal = (1200.0, 0.0)),

    # GAUNTLET / SLALOM. Three walls alternately hanging from the top and rising
    # from the bottom force a down-up-down weave; each landmark sits in a gap, so
    # sensing and threading are coupled. Also the tightest curvature test
    # (min_turn_radius_m vs. 86.6 m row spacing).
    :gauntlet => () -> (landmarks = Landmark[
                            Landmark(350.0, -120.0, random_landmark_cov() * 2),
                            Landmark(650.0,  150.0, random_landmark_cov() * 2),
                            Landmark(950.0, -120.0, random_landmark_cov() * 2)],
                        obstacles = Obstacle[
                            # hangs down from above the top row ⇒ pass at y ≤ 0
                            build_obstacle([(200.0,60.0), (260.0,60.0), (260.0,320.0), (200.0,320.0)]),
                            # rises up from below the bottom row ⇒ pass at y ≥ 86.6
                            build_obstacle([(500.0,-460.0), (560.0,-460.0), (560.0,40.0), (500.0,40.0)]),
                            # hangs down ⇒ pass at y ≤ 0 again
                            build_obstacle([(800.0,60.0), (860.0,60.0), (860.0,320.0), (800.0,320.0)])],
                        start = (0.0, 0.0), goal = (1200.0, 0.0)),

    # LANDMARK BEHIND A WALL. The only landmark sits below a 500 m-long wall that
    # covers row -173.2. From the direct route it is ~300 m away (no fix at all);
    # reaching it means going around either end of the wall and coming back. The
    # detour is unavoidable if the uncertainty threshold is to be met.
    # It sits late (x=700) and is a strong fix (unscaled cov) on purpose: one
    # landmark this far off-route only pays for its detour if little drift is left
    # afterwards — weaken or move it earlier and the whole scenario goes
    # infeasible (exhaustive A* finds nothing under unc_radius_threshold).
    :behind_wall => () -> (landmarks = Landmark[
                               Landmark(700.0, -300.0, random_landmark_cov())],
                           obstacles = Obstacle[
                               build_obstacle([(300.0,-210.0), (700.0,-210.0), (700.0,-130.0), (300.0,-130.0)])],
                           start = (0.0, 0.0), goal = (1000.0, 0.0)),

    # LONG SPARSE CORRIDOR. 1400 m with only two landmarks, one per side: long
    # stretches of pure dead reckoning punctuated by single fixes. A blind
    # straight line drifts to ~10 (vs `unc_radius_threshold: 7`), so both fixes
    # are mandatory. Longest corridor that still plans under the default
    # `astar_iteration_limit: 500000` — the joint state space grows with corridor
    # length, and at 1800 m A* never reaches the goal at all.
    :long_sparse => () -> (landmarks = Landmark[
                               Landmark( 500.0,  230.0, random_landmark_cov() * 2),
                               Landmark(1050.0, -230.0, random_landmark_cov() * 2)],
                           obstacles = Obstacle[],
                           start = (0.0, 0.0), goal = (1400.0, 0.0)),
)

# ── Manual scenario: geometry read straight from config/main.yaml ──

# "x,y" → tuple.
function parse_xy(str::AbstractString, what::String)
    v = parse.(Float64, strip.(split(strip(str), ',')))
    length(v) == 2 || error("$what needs 'x,y', got '$str'")
    return (v[1], v[2])
end

# Flat landmark encoding, mirroring parse_obstacles:
#   "x,y [@ sxx,sxy,syx,syy] | x,y | ..."
# landmarks sep by '|', optional per-landmark covariance after '@' (4 numbers,
# row-major); omitted ⇒ random_landmark_cov(). Empty string ⇒ no landmarks.
function parse_landmarks(str::AbstractString)
    lms = Landmark[]
    s = strip(str)
    isempty(s) && return lms
    for spec in split(s, '|')
        spec = strip(spec)
        isempty(spec) && continue
        geom = spec
        cov = random_landmark_cov()
        if occursin('@', spec)
            geom, cov_spec = split(spec, '@'; limit=2)
            c = parse.(Float64, strip.(split(strip(cov_spec), ',')))
            length(c) == 4 || error("landmark cov needs 4 numbers (sxx,sxy,syx,syy), got $(length(c))")
            cov = [c[1] c[2]; c[3] c[4]]
        end
        x, y = parse_xy(geom, "landmark")
        push!(lms, Landmark(x, y, cov))
    end
    return lms
end

function manual_scenario()
    for k in ("start", "goal")
        haskey(CFG, k) || error("`landmark_scenario: manual` needs a `$k:` key (\"x,y\") in config/main.yaml")
    end
    return (landmarks = parse_landmarks(String(get(CFG, "landmarks", ""))),
            obstacles = parse_obstacles(String(get(CFG, "obstacles", ""))),
            start = parse_xy(String(CFG["start"]), "start"),
            goal  = parse_xy(String(CFG["goal"]),  "goal"))
end

function build_scenario(name::Symbol)
    sc = if name === :manual
        manual_scenario()
    elseif haskey(SCENARIOS, name)
        SCENARIOS[name]()
    else
        error("Unknown landmark_scenario: $name. Known: " *
              join(sort(string.(keys(SCENARIOS))), ", ") * ", manual.")
    end
    # ENV override, used by test_obstacle_robustness.jl / check_true_collisions.jl
    # to sweep generated obstacle fields over a scenario without editing config.
    haskey(ENV, "OBSTACLES") && (sc = merge(sc, (obstacles = parse_obstacles(ENV["OBSTACLES"]),)))
    return sc
end

const ACTIVE_SCENARIO = build_scenario(LANDMARK_SCENARIO)
# The obstacle field every stage reads (discrete A* filter, MINVO continuous
# constraint, viz). Materialized here rather than in obstacles.jl because
# geometry belongs to the scenario, not to config.
const OBSTACLES = ACTIVE_SCENARIO.obstacles
