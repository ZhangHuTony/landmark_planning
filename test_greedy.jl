# Self-check for the greedy baseline planner (planners/greedy.jl).
#
# Run:  julia test_greedy.jl
#
# Verifies the three properties that define this planner, on the obstacle-free
# `clustered` scenario:
#   1. the primary really does take a MINIMUM-length route — its arc length
#      equals the Floyd-Warshall shortest distance to the node it ends on, and
#      that node is the cheapest of all goal-adjacent cells;
#   2. no support ever outruns the primary (the support cap the optimizer's
#      spline-length row later depends on);
#   3. the helper's move really is the argmin of the documented ranking — the
#      rejected successors are re-scored (from the ranking's definition, not by
#      calling the rollout) and none is strictly better.
# Plus a unit test of the ranking comparator itself, which check 3 cannot cover:
# it re-derives the keys but compares them with the same greedy_better.

using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so
# clear it to make config.jl use config/. Pin the scenario here rather than
# inheriting whatever config/main.yaml happens to name: these assertions are
# about an obstacle-free layout with landmarks off the corridor.
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
include(joinpath(_ROOT, "planners", "greedy.jl"))

@assert NUM_AGENTS >= 2 "test_greedy needs at least one support agent (num_agents >= 2)"
@assert isempty(OBSTACLES) "test_greedy assumes the obstacle-free `clustered` scenario"

# ── 0. the ranking comparator treats a 1-ULP row-1 spread as a TIE ───────────
# Real keys, measured on `single`: out of comm range the helper cannot move the
# primary's covariance, so row 1 is physically identical and differs only in the
# last bit — and it happened to be 1 ULP *worse* on the move that collected a
# landmark fix (helper unc 8.22 -> 0.93). Under an exact `<` the shadowing move
# wins on that noise and rows 2-4 never run at all.
let shadow   = (7.7134147053872475, 8.2241, 100.0, (1,)),
    landmark = (7.713414705387248,  0.9284, 200.0, (2,))
    @assert greedy_better(landmark, shadow) "row 1 noise outranks a real row 2 difference"
    @assert !greedy_better(shadow, landmark)
    # ...but a difference well above the tolerance must still decide row 1.
    @assert greedy_better((7.70, 9.9, 100.0, (1,)), landmark)
end
println("✓ ranking comparator ties row 1 at 1-ULP and still respects real differences")

const SC    = ACTIVE_SCENARIO
const LMS   = SC.landmarks
const GRAPH = build_hex_graph(LMS, SC.start, SC.goal; hex_r=HEX_RADIUS_M)

ppaths, _, _, _ = joint_astar(GRAPH, LMS, Inf, 1)
@assert !isempty(ppaths) && !isempty(ppaths[1]) "primary found no route on `clustered`"
const PP = ppaths[1]

# ── 1. the primary's route is minimum-length ──────────────────────────────────
# joint_astar terminates on a goal-ADJACENT cell, not the terminal node, so
# compare against the shortest distance to that cell — and check no other
# goal-adjacent cell is cheaper, which is what makes it the minimum overall.
prim_len = sum(GRAPH.distance[PP[i-1], PP[i]] for i in 2:length(PP))
@assert isapprox(prim_len, GRAPH.shortest_paths[1, PP[end]]; atol=1e-9) """
    primary route is not a shortest path to its own end node:
    walked $(prim_len) vs shortest $(GRAPH.shortest_paths[1, PP[end]])"""
best_entry = minimum(GRAPH.shortest_paths[1, v] for v in 1:(GRAPH.n - 1)
                     if GRAPH.n in GRAPH.neighbors[v])
@assert prim_len <= best_entry + 1e-9 """
    a cheaper goal-adjacent cell exists: $(best_entry) < $(prim_len)"""
println("✓ primary route is minimum-length: $(round(prim_len, digits=3)) m over $(length(PP)) nodes")

# ── 2. supports never outrun the primary ─────────────────────────────────────
const SUPPORTS = greedy_support_rollout(PP, GRAPH, LMS, NUM_AGENTS)
@assert SUPPORTS !== nothing "greedy rollout stalled on `clustered`"
for a in 1:(NUM_AGENTS - 1)
    @assert length(SUPPORTS[a]) == length(PP) "support $a is not in lockstep with the primary"
    sd = 0.0; pd = 0.0
    for k in 2:length(PP)
        sd += GRAPH.distance[SUPPORTS[a][k-1], SUPPORTS[a][k]]
        pd += GRAPH.distance[PP[k-1], PP[k]]
        @assert sd <= pd + GREEDY_TIE_TOL """
            support $a outran the primary at step $k: $(sd) > $(pd)"""
    end
end
println("✓ support cap holds at every step for all $(NUM_AGENTS - 1) support(s)")

# ── 3. every chosen move really is the argmin of the ranking ─────────────────
# Recompute the full ranking key for the chosen successor and for each rejected
# one, exactly as greedy_support_rollout does, and assert nothing beats it.
function ranking_key(prefix_supports, combo, k)
    prefix = Vector{Vector{Int}}(undef, NUM_AGENTS)
    for a in 1:(NUM_AGENTS - 1)
        prefix[a] = vcat(prefix_supports[a], combo[a])
    end
    prefix[NUM_AGENTS] = PP[1:k]
    covs, dists = evaluate_full_paths(prefix, GRAPH, LMS, NUM_AGENTS)
    for a in 1:(NUM_AGENTS - 1)
        dists[a] > dists[NUM_AGENTS] + GREEDY_TIE_TOL && return nothing   # support cap
    end
    px = GRAPH.landmarks[PP[k]].x; py = GRAPH.landmarks[PP[k]].y
    return (unc_radius(covs[NUM_AGENTS]),
            sum(unc_radius(covs[a]) for a in 1:(NUM_AGENTS - 1)),
            sum(hypot(GRAPH.landmarks[combo[a]].x - px,
                      GRAPH.landmarks[combo[a]].y - py) for a in 1:(NUM_AGENTS - 1)),
            combo)
end

function check_argmin()
    n_checked = 0
    for k in 2:length(PP)
        sofar    = [SUPPORTS[a][1:k-1] for a in 1:(NUM_AGENTS - 1)]
        chosen   = Tuple(SUPPORTS[a][k] for a in 1:(NUM_AGENTS - 1))
        key_best = ranking_key(sofar, chosen, k)
        @assert key_best !== nothing "the move greedy chose at step $k violates the support cap"

        visited = [Set(SUPPORTS[a][1:k-1]) for a in 1:(NUM_AGENTS - 1)]
        options = [[u for u in GRAPH.neighbors[SUPPORTS[a][k-1]] if !(u in visited[a])]
                   for a in 1:(NUM_AGENTS - 1)]
        for combo in Iterators.product(options...)
            combo == chosen && continue
            key = ranking_key(sofar, combo, k)
            key === nothing && continue
            n_checked += 1
            @assert !greedy_better(key, key_best) """
                step $k: rejected successor $(combo) outranks the chosen $(chosen)
                  rejected: $(key)
                  chosen  : $(key_best)"""
        end
    end
    return n_checked
end
println("✓ every chosen move is the argmin ($(check_argmin()) rejected alternatives re-scored)")

println("\nAll greedy self-checks passed.")
