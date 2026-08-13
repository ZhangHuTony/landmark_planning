# Self-check for the clgbt baseline planner (planners/clgbt.jl).
#
# Run:  julia test_clgbt.jl
#
# clgbt is a RANDOMIZED sampling-based search (unlike every other baseline in
# this repo), so there is no argmin/optimality property to re-verify the way
# test_greedy.jl does. What's checked instead are the STRUCTURAL invariants
# the algorithm is supposed to guarantee by construction:
#   1. the returned discrete solution is a genuine walk in the hex graph
#      (every consecutive pair is a real edge), starts at the start node, and
#      the primary ends on a goal-adjacent cell with unc under threshold;
#   2. no agent revisits a node within its own path (the steering cycle guard);
#   3. no support ever outruns the primary (every joint extension advances
#      every agent by the SAME number of steps, so this should hold trivially
#      -- this test is what would catch it if that invariant were ever broken);
#   4. fixing clgbt_seed makes the search reproducible (same tree size, same
#      solution, same discrete length/uncertainty) across independent calls.

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

# ── 0. reconstructing the root is the identity path ───────────────────────
let root_only = ClgbtTreeNode[ClgbtTreeNode(0, fill(1, NUM_AGENTS), [Int[] for _ in 1:NUM_AGENTS],
                                            [copy(LMS[1].cov) for _ in 1:NUM_AGENTS],
                                            zeros(NUM_AGENTS))]
    p = clgbt_reconstruct_paths(root_only, 1, NUM_AGENTS)
    @assert all(p[a] == [1] for a in 1:NUM_AGENTS) "root should reconstruct to [1] for every agent"
end
println("✓ root reconstructs to the start node for every agent")

# ── grow the tree once, verbose off ───────────────────────────────────────
tree, solution_idx, iters = clgbt_grow_tree(GRAPH, LMS, NUM_AGENTS, PRIMARY; verbose=false)
@assert solution_idx != 0 "clgbt found no solution on `clustered` within its default budget " *
                          "($(iters) iterations, tree size $(length(tree))) -- either the default " *
                          "iteration budget regressed or the search itself is broken"
paths = clgbt_reconstruct_paths(tree, solution_idx, NUM_AGENTS)

# ── 1. every path is a genuine walk; primary ends goal-adjacent under threshold ──
for a in 1:NUM_AGENTS
    @assert paths[a][1] == 1 "agent $a's path does not start at the start node"
    for k in 2:length(paths[a])
        @assert paths[a][k] in GRAPH.neighbors[paths[a][k-1]] """
            agent $a step $k is not a real graph edge: $(paths[a][k-1]) -> $(paths[a][k])"""
    end
end
goal_mask = clgbt_goal_adjacent(GRAPH)
@assert goal_mask[paths[PRIMARY][end]] "primary's path does not end on a goal-adjacent cell"
seed_covs, seed_dists = evaluate_full_paths(paths, GRAPH, LMS, NUM_AGENTS)
prim_unc = unc_radius(seed_covs[PRIMARY])
@assert unc_within_threshold(prim_unc, UNC_RADIUS_THRESHOLD, UNC_FEAS_TOL) """
    primary's solution uncertainty $(prim_unc) exceeds threshold $(UNC_RADIUS_THRESHOLD)"""
println("✓ every agent's path is a real graph walk; primary ends goal-adjacent " *
        "(dist=$(round(seed_dists[PRIMARY], digits=1))m, unc=$(round(prim_unc, digits=4))) " *
        "in $(iters) iterations, tree size $(length(tree))")

# ── 2. no agent revisits a node within its own path ───────────────────────
for a in 1:NUM_AGENTS
    @assert length(paths[a]) == length(Set(paths[a])) "agent $a revisits a node in its own path"
end
println("✓ no agent revisits a node within its own path")

# ── 3. no support ever outruns the primary, at every step ─────────────────
for a in 1:(NUM_AGENTS - 1)
    @assert length(paths[a]) == length(paths[PRIMARY]) """
        support $a has $(length(paths[a])) waypoints, primary has $(length(paths[PRIMARY])) -- \
        every joint extension should advance all agents by the same number of steps"""
    sd = 0.0; pd = 0.0
    for k in 2:length(paths[PRIMARY])
        sd += GRAPH.distance[paths[a][k-1], paths[a][k]]
        pd += GRAPH.distance[paths[PRIMARY][k-1], paths[PRIMARY][k]]
        @assert sd <= pd + UNC_FEAS_TOL "support $a outran the primary at step $k: $(sd) > $(pd)"
    end
end
println("✓ support cap holds at every step for all $(NUM_AGENTS - 1) support(s)")

# ── 4. fixed seed => reproducible search ───────────────────────────────────
tree2, solution_idx2, iters2 = clgbt_grow_tree(GRAPH, LMS, NUM_AGENTS, PRIMARY; verbose=false)
@assert iters2 == iters && length(tree2) == length(tree) && solution_idx2 == solution_idx """
    same seed produced a different search: iters $(iters) vs $(iters2), \
    tree $(length(tree)) vs $(length(tree2)), solution_idx $(solution_idx) vs $(solution_idx2)"""
paths2 = clgbt_reconstruct_paths(tree2, solution_idx2, NUM_AGENTS)
@assert paths2 == paths "same seed produced a different discrete solution path"
println("✓ fixed clgbt_seed reproduces the same tree and solution across independent runs")

println("\nAll clgbt self-checks passed.")
