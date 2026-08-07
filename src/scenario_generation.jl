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

function random_landmark_cov(rng)
    # SPD covariance with randomized anisotropy/correlation.
    sx = 0.80 + 0.55 * rand(rng)
    sy = 0.60 + 0.35 * rand(rng)
    ρ = 0.45 * (2 * rand(rng) - 1)
    cxy = ρ * sx * sy
    return [sx^2 cxy; cxy sy^2]
end
# The preset SCENARIOS below draw from the global stream that generate_plan.jl
# seeds with Random.seed!(42); generate_scenario passes its own local RNG so a
# generated scenario does not depend on what else consumed that stream.
# default_rng() is exactly what bare rand() uses, so presets are unchanged.
random_landmark_cov() = random_landmark_cov(Random.default_rng())

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

    # MAZE. Three full-height walls, each with TWO gaps at different rows, plus
    # five plugs so no chamber has a straight shot: the corridor becomes a set of
    # narrow passages instead of open space. Two homotopies survive, each with two
    # landmarks — north (L1 in A's high gap, L4 in the upper middle chamber) is
    # shorter; south (L2 in B's low gap, L3 in C's low gap) is a longer weave. That
    # is the distance-vs-uncertainty trade :two_routes failed to produce (there the
    # cheap side was infeasible, so the front collapsed to one seed).
    #
    # Wall geometry is pinned to the lattice, not eyeballed. At hex_width_m: 100 the
    # rows are the y values in the header comment, and node x depends on row parity:
    # rows {-346.4,-173.2,0,173.2} sit at x ≡ 0 (mod 100), rows
    # {-433,-259.8,-86.6,86.6,259.8} at x ≡ 50. So a vertical wall must span ≥150 m
    # in x to cover one column of EACH parity — a narrower one is stepped around on
    # the offset row. All three span 150 m. Gaps are the y-bands left uncovered;
    # since hex moves never skip a row, covering a row's band blocks it outright.
    #
    # Tuning knob: if both routes come out infeasible under unc_radius_threshold
    # (~1500-1800 m of arc here), drop the landmark scale *2 → *1 for stronger fixes.
    :maze => () -> (landmarks = Landmark[
                        Landmark(250.0,  173.2, random_landmark_cov() * 2),  # L1, wall A high gap
                        Landmark(590.0, -259.8, random_landmark_cov() * 2),  # L2, wall B low gap
                        Landmark(930.0, -346.4, random_landmark_cov() * 2),  # L3, wall C low gap
                        Landmark(830.0,  173.2, random_landmark_cov() * 2)], # L4, upper middle chamber
                    obstacles = Obstacle[
                        # Wall A, x ∈ [175,325] — gaps at rows 173.2 and -86.6
                        build_obstacle([(175.0, 216.5), (325.0, 216.5), (325.0, 340.0), (175.0, 340.0)]),
                        build_obstacle([(175.0, -43.3), (325.0, -43.3), (325.0, 129.9), (175.0, 129.9)]),
                        build_obstacle([(175.0,-520.0), (325.0,-520.0), (325.0,-129.9), (175.0,-129.9)]),
                        # Wall B, x ∈ [515,665] — gaps at rows 259.8 and -259.8
                        build_obstacle([(515.0,-216.5), (665.0,-216.5), (665.0, 216.5), (515.0, 216.5)]),
                        build_obstacle([(515.0,-520.0), (665.0,-520.0), (665.0,-303.1), (515.0,-303.1)]),
                        # Wall C, x ∈ [855,1005] — gaps at rows 0 and -346.4
                        build_obstacle([(855.0,  43.3), (1005.0,  43.3), (1005.0, 340.0), (855.0, 340.0)]),
                        build_obstacle([(855.0,-303.1), (1005.0,-303.1), (1005.0, -43.3), (855.0, -43.3)]),
                        build_obstacle([(855.0,-520.0), (1005.0,-520.0), (1005.0,-389.7), (855.0,-389.7)]),
                        # Chamber plugs: each kills a cell or two (named in the trailing
                        # comment), forcing a row change mid-chamber rather than sealing
                        # it. Each is offset ≥1 free column from the wall it follows, so
                        # a gap always keeps an onward move; if a run comes back with no
                        # path at all, a plug swallowing its gap's exit is the first
                        # thing to check.
                        build_obstacle([(1050.0,-216.5), (1150.0,-216.5), (1150.0,-129.9), (1050.0,-129.9)])], # (1100,-173.2)
                    start = (0.0, 0.0), goal = (1200.0, 0.0)),

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

# ── Random scenario generation (`landmark_scenario: random`) ──
# A corridor scenario drawn from four numbers, for statistical benchmarking
# (run_constraint_sweep.jl) rather than the hand-tuned presets above.
#
# Every draw comes from a LOCAL RNG seeded by `seed`, never the global stream,
# so a scenario is reproducible from (seed, goal_dist, n_landmarks,
# n_obstacles) alone — independent of generate_plan.jl's hardcoded
# Random.seed!(42) and of whatever else consumed that stream first.

# n convex obstacles (3–6 sides) in the corridor, non-overlapping and clear of
# both endpoints. Convexity is GUARANTEED, not hoped for: `build_obstacle`
# errors on a non-convex or degenerate polygon, so vertices are placed at
# evenly-spaced angles on a rotated ellipse with jitter strictly under half the
# angular spacing. Points taken in angular order on a convex curve are always
# convex, and the jitter bound keeps any two vertices apart (a zero-length edge
# is also a `build_obstacle` error).
#
# The semi-axis ranges are deliberately wide: a barrier spanning less than
# ~150 m does not cover both hex column parities and gets stepped around
# trivially (see the corridor-geometry note at the top of this file), so some
# obstacles have to be big enough to actually redirect a route.
#
# ponytail: near-duplicate of `gen_obstacles` in test_obstacle_robustness.jl.
# That one is rectangles-only and hardcodes the 0→1000 corridor, and its RNG
# stream pins that harness's reproducible output — generalizing it would move
# every obstacle it has ever placed, so it is deliberately left alone.
function random_convex_obstacles(n::Int, rng, goal_x::Float64)
    obs   = Obstacle[]
    boxes = NTuple{4,Float64}[]           # placed AABBs, for the overlap test
    tries = 0
    while length(obs) < n && tries < 20000
        tries += 1
        k  = rand(rng, 3:6)               # triangle … hexagon
        a  = 25.0 + 85.0 * rand(rng)      # semi-axes, polygon-local
        b  = 20.0 + 40.0 * rand(rng)
        φ  = 2π * rand(rng)               # rotation, so long axis can lie either way
        cx = goal_x * (0.12 + 0.76 * rand(rng))
        # Triangular on [-240, 240] (difference of two uniforms), peaked on the
        # direct start→goal line instead of flat across the corridor. An obstacle
        # sitting ON the line is the case worth sampling: getting past it is a
        # left-or-right choice, and a local optimizer seeded with the straight
        # line cannot cross to the other side once it has picked. Flat placement
        # produced that by luck in roughly a third of scenarios; this makes it
        # ~60%, without capping the range or special-casing any scenario.
        cy = 240.0 * (rand(rng) - rand(rng))

        # Clear of start and goal. Centre distance against the circumradius is
        # conservative (the polygon is inscribed in the a×b ellipse).
        r_out = max(a, b)
        (hypot(cx, cy) < 90.0 + r_out || hypot(cx - goal_x, cy) < 90.0 + r_out) && continue

        step  = 2π / k
        verts = Tuple{Float64,Float64}[]
        for i in 0:k-1
            θ = i * step + (0.7 * step) * (rand(rng) - 0.5)   # |jitter| < 0.35·step
            px = a * cos(θ); py = b * sin(θ)
            push!(verts, (cx + px * cos(φ) - py * sin(φ),
                          cy + px * sin(φ) + py * cos(φ)))
        end

        xs = first.(verts); ys = last.(verts)
        box = (minimum(xs), minimum(ys), maximum(xs), maximum(ys))
        any(r -> !(box[3] + 12.0 < r[1] || box[1] - 12.0 > r[3] ||
                   box[4] + 12.0 < r[2] || box[2] - 12.0 > r[4]), boxes) && continue

        push!(obs, build_obstacle(verts))
        push!(boxes, box)
    end
    length(obs) == n || @warn "placed only $(length(obs))/$n obstacles (corridor saturated)"
    return obs
end

function generate_scenario(; seed::Int, goal_dist::Real,
                             n_landmarks::Int, n_obstacles::Int)
    # build_hex_graph indexes sensor_landmarks[1] unconditionally, and landmark
    # 1's covariance is Σ₀ for every agent — an empty field is a BoundsError,
    # not an easy scenario.
    n_landmarks >= 1 || error("generate_scenario needs n_landmarks >= 1, got $(n_landmarks)")
    rng = Random.MersenneTwister(seed)
    D   = Float64(goal_dist)

    # Landmarks are pushed OFF the direct start→goal line on purpose: that is
    # what makes a fix cost a detour, which is the whole trade-off under test.
    #
    # The inner edge is 200 m, not the 60 m it used to be, and the number comes
    # straight out of the sensor model rather than taste. visibility_width is 2.5
    # against a visibility_range of 100, so detection is effectively a hard wall
    # at 100 m: p(105 m) = 0.12, p(110 m) = 0.018. Two free rides followed:
    #   * |y| <= 100  — the PRIMARY sees it from the direct line, no detour at all.
    #   * |y| <= 200  — the SUPPORT sees it while sitting 100 m off the line, which
    #                   is exactly comm_range, so it keeps fusing with a primary
    #                   that never left the straight path.
    # Sweep 2026-08-07_16-46-28 is what this is fixing: s001 drew landmarks at
    # -83.7/-99.8 and s003 at -83.9, and on both straight_cont held ratio 1.0000
    # from p100 all the way to p30 — it never had to move. s005 (all landmarks
    # 233-285 m out) is the only scenario where it was forced off the line.
    # Outer edge 320 m: reachable hex rows stop at ±259.8, and 320 - 259.8 = 60 m
    # is still well inside the 100 m detection wall.
    # ponytail: hardcoded because it is derived from visibility_range/comm_range,
    # which are themselves fixed in config/mc/main.yaml. If those move, this must.
    lms = Landmark[]
    for _ in 1:n_landmarks
        x = D * (0.12 + 0.76 * rand(rng))
        y = (rand(rng) < 0.5 ? -1.0 : 1.0) * (200.0 + 120.0 * rand(rng))
        push!(lms, Landmark(x, y, random_landmark_cov(rng)))
    end

    return (landmarks = lms,
            obstacles = random_convex_obstacles(n_obstacles, rng, D),
            start = (0.0, 0.0), goal = (D, 0.0))
end

function build_scenario(name::Symbol)
    sc = if name === :random
        generate_scenario(seed        = Int(CFG["scenario_seed"]),
                          goal_dist   = Float64(CFG["scenario_goal_dist"]),
                          n_landmarks = Int(CFG["scenario_n_landmarks"]),
                          n_obstacles = Int(CFG["scenario_n_obstacles"]))
    elseif name === :manual
        manual_scenario()
    elseif haskey(SCENARIOS, name)
        SCENARIOS[name]()
    else
        error("Unknown landmark_scenario: $name. Known: " *
              join(sort(string.(keys(SCENARIOS))), ", ") * ", manual, random.")
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
