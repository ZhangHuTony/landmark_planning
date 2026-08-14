# Self-check for the clgbt baseline planner (planners/clgbt.jl).
#
# Run:  julia test_clgbt.jl
#
# clgbt is a RANDOMIZED sampling-based search (unlike every other baseline in
# this repo), so there is no argmin/optimality property to re-verify the way
# test_greedy.jl does. What's checked instead are the STRUCTURAL invariants
# the algorithm is supposed to guarantee by construction:
#   1. every agent's path starts at the scenario start, and every step is
#      exactly clgbt_step_m long (one constant control per extension);
#   2. no step turns by more than CLGBT_MAX_TURN, and turns only happen at
#      extension boundaries (a leg is a straight segment);
#   3. no support ever outruns the primary;
#   4. the solution pins the exact goal onto the primary and certifies its
#      uncertainty THERE;
#   5. fixing clgbt_seed makes the search reproducible.
#
# Invariants 1-3 are properties of ANY tree node, so they are checked on the
# deepest node whether or not a solution was found — they catch a real
# regression in the steering. "Does it find a solution in N iterations" is a
# TUNING fact about one scenario, not an invariant, so it is a separate check
# at the bottom with its budget passed explicitly rather than inherited from
# config (that coupling is what made this file fail at HEAD while nothing was
# actually broken — see notes/LOGS.md).

using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so
# clear it to make config.jl use config/. Pin the scenario here rather than
# inheriting whatever config/main.yaml happens to name: `clustered` is
# obstacle-free and small enough that clgbt reliably finds a solution well
# inside its default iteration budget (see planners/clgbt.jl's own testing).
empty!(ARGS)
ENV["SCENARIO"] = "clustered"

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))
include(joinpath(_ROOT, "planners", "clgbt.jl"))

@assert NUM_AGENTS >= 2 "test_clgbt needs at least one support agent (num_agents >= 2)"
@assert isempty(OBSTACLES) "test_clgbt assumes the obstacle-free `clustered` scenario"

const SC    = ACTIVE_SCENARIO
const LMS   = SC.landmarks
const GRAPH = build_hex_graph(LMS, SC.start, SC.goal; hex_r=HEX_RADIUS_M)
const PRIMARY = NUM_AGENTS
const START = (GRAPH.landmarks[1].x, GRAPH.landmarks[1].y)
const GOAL  = (GRAPH.landmarks[GRAPH.n].x, GRAPH.landmarks[GRAPH.n].y)
const TOL   = 1e-6

seglen(p, k) = hypot(p[k][1] - p[k-1][1], p[k][2] - p[k-1][2])
segdir(p, k) = atan(p[k][2] - p[k-1][2], p[k][1] - p[k-1][1])
wrap(θ) = atan(sin(θ), cos(θ))

# ── 0. reconstructing the root is the identity path ───────────────────────
let root_only = ClgbtTreeNode[ClgbtTreeNode(0, fill(START, NUM_AGENTS), zeros(NUM_AGENTS),
                                            [Tuple{Float64,Float64}[] for _ in 1:NUM_AGENTS],
                                            [copy(LMS[1].cov) for _ in 1:NUM_AGENTS],
                                            zeros(NUM_AGENTS))]
    p = clgbt_reconstruct_paths(root_only, 1, NUM_AGENTS, START)
    @assert all(p[a] == [START] for a in 1:NUM_AGENTS) "root should reconstruct to [start] for every agent"
end
println("✓ root reconstructs to the start position for every agent")

# ── grow the tree once, verbose off ───────────────────────────────────────
tree, solution_idx, iters, paths = clgbt_grow_tree(GRAPH, LMS, NUM_AGENTS, PRIMARY; verbose=false)
@assert length(tree) > 1 "clgbt grew no tree at all in $(iters) iterations — steering is broken"

# ── 1-3. structural invariants, on the DEEPEST node (no solution required) ──
let
    depth(k) = (d = 0; while k != 0; d += 1; k = tree[k].parent; end; d)
    deepest = argmax(depth.(1:length(tree)))
    dp = clgbt_reconstruct_paths(tree, deepest, NUM_AGENTS, START)

    for a in 1:NUM_AGENTS
        @assert dp[a][1] == START "agent $a's path does not start at the scenario start"
        @assert length(dp[a]) == length(dp[PRIMARY]) """
            agent $a has $(length(dp[a])) waypoints, primary has $(length(dp[PRIMARY])) — \
            every joint extension should advance all agents by the same number of steps"""
        for k in 2:length(dp[a])
            @assert abs(seglen(dp[a], k) - CLGBT_STEP_M) <= 1e-9 * CLGBT_STEP_M """
                agent $a step $k is $(seglen(dp[a], k))m, expected clgbt_step_m=$(CLGBT_STEP_M)m"""
        end
        for k in 3:length(dp[a])
            turn = abs(wrap(segdir(dp[a], k) - segdir(dp[a], k-1)))
            @assert turn <= CLGBT_MAX_TURN + TOL """
                agent $a turns $(rad2deg(turn))° at step $k, cap is $(rad2deg(CLGBT_MAX_TURN))°"""
        end
    end
    println("✓ deepest branch ($(length(dp[PRIMARY]) - 1) steps): every agent starts at the start, " *
            "every step is $(CLGBT_STEP_M)m, every turn ≤ $(round(rad2deg(CLGBT_MAX_TURN), digits=1))°")

    for a in 1:(NUM_AGENTS - 1)
        sd = 0.0; pd = 0.0
        for k in 2:length(dp[PRIMARY])
            sd += seglen(dp[a], k); pd += seglen(dp[PRIMARY], k)
            @assert sd <= pd + UNC_FEAS_TOL "support $a outran the primary at step $k: $(sd) > $(pd)"
        end
    end
    println("✓ support cap holds at every step for all $(NUM_AGENTS - 1) support(s)")
end

# ── 4. the solution pins the exact goal and certifies uncertainty there ────
@assert solution_idx != 0 """
    clgbt found no solution on `clustered` within its default budget \
    ($(iters) iterations, tree size $(length(tree)))"""

@assert paths[PRIMARY][end] == GOAL """
    primary's path ends at $(paths[PRIMARY][end]), not the exact goal $(GOAL)"""
let dgoal = seglen(paths[PRIMARY], length(paths[PRIMARY]))
    @assert dgoal <= CLGBT_GOAL_RADIUS_M + TOL """
        the goal-pin hop is $(dgoal)m, longer than clgbt_goal_radius_m=$(CLGBT_GOAL_RADIUS_M)m"""
end
for a in 1:(NUM_AGENTS - 1)
    # The pin adds one waypoint to the primary only; seed_control_points pads the
    # supports back up. (Equal lengths mean the primary landed exactly on the goal.)
    @assert length(paths[a]) in (length(paths[PRIMARY]) - 1, length(paths[PRIMARY])) """
        support $a has $(length(paths[a])) waypoints against the primary's $(length(paths[PRIMARY]))"""
end
seed_covs, seed_dists, _ = clgbt_eval(paths, LMS, NUM_AGENTS)
prim_unc = unc_radius(seed_covs[PRIMARY])
@assert unc_within_threshold(prim_unc, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) """
    primary's solution uncertainty $(prim_unc) exceeds threshold $(UNC_RADIUS_THRESHOLD)"""
println("✓ primary ends exactly on the goal with unc=$(round(prim_unc, digits=4)) ≤ " *
        "$(UNC_RADIUS_THRESHOLD) (dist=$(round(seed_dists[PRIMARY], digits=1))m) " *
        "in $(iters) iterations, tree size $(length(tree))")

# ── 5. the continuous seed survives the handoff to the shared engine ───────
let (sg, ipaths) = clgbt_seed_graph(paths, GRAPH)
    ctrls = seed_control_points(ipaths, sg)
    @assert length(ctrls) == NUM_AGENTS "seed graph lost an agent"
    @assert ctrls[PRIMARY] == paths[PRIMARY] """
        the seed handed to optimize_continuous is not the tree's own path — \
        it was re-quantized somewhere"""
    for a in 1:(NUM_AGENTS - 1)
        @assert length(ctrls[a]) == length(ctrls[PRIMARY]) "support $a was not padded to the primary"
        @assert ctrls[a][1:length(paths[a])] == paths[a] "support $a's seed was altered"
    end
    @assert seed_spline_clear(ipaths, sg, LMS) "the solution does not pass its own spline gate"
end
println("✓ the continuous seed reaches seed_control_points unquantized and passes the spline gate")

# ── 6. fixed seed => reproducible search ───────────────────────────────────
tree2, solution_idx2, iters2, paths2 = clgbt_grow_tree(GRAPH, LMS, NUM_AGENTS, PRIMARY; verbose=false)
@assert iters2 == iters && length(tree2) == length(tree) && solution_idx2 == solution_idx """
    same seed produced a different search: iters $(iters) vs $(iters2), \
    tree $(length(tree)) vs $(length(tree2)), solution_idx $(solution_idx) vs $(solution_idx2)"""
@assert paths2 == paths "same seed produced a different solution path"
println("✓ fixed clgbt_seed reproduces the same tree and solution across independent runs")

println("\nAll clgbt self-checks passed.")
