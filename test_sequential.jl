# Self-check for the sequential baseline planner (planners/sequential.jl).
#
# Run:  julia test_sequential.jl
#
# Covers ONE property, the one the helper leg had no guard for: the route
# seq_helper_search returns must survive as a SPLINE, not merely as the straight
# polyline its per-step filter tests (planners/sequential.jl:274). Those are
# different geometries — the refined spline cuts the polyline's corners — and what
# ships is gated on the spline (seed_spline_clear, called from plan_sequential).
# Nothing downstream can repair a mismatch: the helpers are never re-planned, and
# leg 4 can only vary the PRIMARY, so a helper-side breach vetoes every leg-4
# candidate and the parked fallback throws the whole leg away. Measured on
# constraint_sweep/2026-08-14_16-55-00, that fired on 28 of 50 scenarios.
#
# `two_routes` is pinned because it actually EXERCISES the guard: its two
# highest-ranked helper routes breach and the third does not. A scenario where
# rank 1 already clears would pass this file even with the guard deleted, which is
# why the first assertion checks that the walk happened at all.

using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates

Random.seed!(42)   # same seed generate_plan.jl uses; the hex grid is jittered

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so clear
# it to make config.jl use config/. Pin the scenario rather than inheriting
# whatever config/main.yaml names: this assertion is about an obstacle layout.
empty!(ARGS)
ENV["SCENARIO"] = "two_routes"

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))
include(joinpath(_ROOT, "planners", "sequential.jl"))

@assert NUM_AGENTS >= 2 "test_sequential needs at least one helper (num_agents >= 2)"
@assert !isempty(OBSTACLES) "test_sequential needs an obstacle scenario; got none"
@assert OBSTACLE_CONTINUOUS "seed_spline_clear is a no-op unless obstacle_continuous is on"

const SCEN  = ACTIVE_SCENARIO
const LMS   = SCEN.landmarks
const GRAPH = build_hex_graph(LMS, SCEN.start, SCEN.goal; hex_r=HEX_RADIUS_M)

# ── Leg 1, then the helper leg — the two calls plan_sequential makes ─────────
# Both are noisy (A* progress bar, the helper leg's own rank message), and the
# rank message is the evidence the guard ran, so capture rather than silence it.
# redirect_stdout needs a real stream here, not an IOBuffer.
roster = [Int[1] for _ in 1:NUM_AGENTS]
let (path, io) = mktemp()
    global ppath
    ppath, _, _, _ = redirect_stdout(io) do
        seq_primary_astar(GRAPH, LMS, roster, SEQ_LEG1_SLACK * UNC_RADIUS_THRESHOLD)
    end
    close(io)
end
@assert !isempty(ppath) "leg 1 found no route on `two_routes`; the fixture has drifted"
roster[NUM_AGENTS] = ppath

hpath = nothing
helper_log = let (path, io) = mktemp()
    global hpath
    hpath = redirect_stdout(() -> seq_helper_search(GRAPH, LMS, roster, 1), io)
    close(io)
    read(path, String)
end
@assert hpath !== nothing "helper leg stalled on `two_routes`; the fixture has drifted"

# ── 1. the guard ran ────────────────────────────────────────────────────────
# Without this the file would still pass on a scenario whose rank-1 route happens
# to clear, i.e. it would stop being a regression test the moment the fixture
# shifted. `two_routes` is chosen precisely because rank 1 does NOT clear.
@assert occursin("took rank #", helper_log) """
    seq_helper_search returned its rank-1 route without testing it as a spline.
    Either the best-first walk over the final layer is gone, or `two_routes` no
    longer breaches at rank 1 and this fixture needs re-picking.
      helper leg said: $(isempty(strip(helper_log)) ? "(nothing)" : strip(helper_log))"""
println("✓ the final-layer walk ran: $(strip(helper_log))")

# ── 2. and it returned something that actually survives ─────────────────────
# Same call, same roster shape, that plan_sequential's ship-time gate makes — so
# passing here is passing there, and the parked fallback is never reached.
@assert seed_spline_clear(seq_with(roster, 1, hpath), GRAPH, LMS) """
    seq_helper_search returned a route that breaches as a spline. plan_sequential
    will discard the helper entirely (park it at the start) and ship a roster with
    no escort, which is the failure this guard exists to prevent."""
println("✓ the returned helper route survives as a spline (no parked fallback)")

# Worth reporting, not asserting: the guard costs primary uncertainty, because the
# layer is ranked by exactly that and anything past rank 1 is worse by definition.
let (covs, _) = evaluate_full_paths(seq_with(roster, 1, hpath), GRAPH, LMS, NUM_AGENTS)
    println("  primary goal_unc with the escort: $(round(unc_radius(covs[end]), digits=4))")
end

println("\nAll sequential self-checks passed.")
