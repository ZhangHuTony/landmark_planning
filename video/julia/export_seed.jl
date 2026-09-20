# ==========================================================================
# export_seed.jl - the discrete A* seed for a run, with no planner edit
# ==========================================================================
#   julia video/julia/export_seed.jl <run_dir> [--out=seed.json]
#
# joint_astar is a plain function returning (paths, dists, unc, iterations), so
# the discrete stage can be re-run from outside the planner exactly as
# _plan_hexspline calls it (planners/hexspline_cl.jl:1508), including the same
# relaxed-discrete threshold. That gives the video the lattice route and the
# seed control polygon -- the zig-zag the refinement starts from -- without
# touching a planner file.
#
# What this canNOT give is the ORDER cells were expanded in, or the optimizer's
# iterates: those exist only inside the two loops and are never returned. Those
# need the logging taps described in the plan, which are pending approval.
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", ".."))
length(ARGS) >= 1 || error("usage: julia export_seed.jl <run_dir> [--out=X]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_DIR = abspath_or(ARGS[1])
_opt(name, dflt) = begin
    i = findfirst(a -> startswith(a, "--$(name)="), ARGS)
    i === nothing ? dflt : split(ARGS[i], "=", limit = 2)[2]
end
const ALGO = _opt("algo", "hexspline_cl")
const OUT_PATH = abspath_or(_opt("out", joinpath(RUN_DIR, "seed.json")))
const FLAT = isfile(joinpath(RUN_DIR, "results.yaml"))
const ALGO_DIR = FLAT ? RUN_DIR : joinpath(RUN_DIR, ALGO)
const CFG_DIR = isdir(joinpath(RUN_DIR, "config")) ? joinpath(RUN_DIR, "config") :
                joinpath(RUN_DIR, "_cfg")

empty!(ARGS); push!(ARGS, CFG_DIR)
using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates
Random.seed!(42)
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))

landmarks = read_landmarks_csv(joinpath(RUN_DIR, "scenario_landmarks.csv"))
graph = build_hex_graph(landmarks, ACTIVE_SCENARIO.start, ACTIVE_SCENARIO.goal;
                        hex_r = HEX_RADIUS_M)

# The same threshold _plan_hexspline searches under.
thr = effective_discrete_unc_threshold(UNC_RADIUS_THRESHOLD)
@printf("Re-running joint A* (threshold %.4f, limit %d)...\n", thr, ASTAR_ITERATION_LIMIT)
t0 = time()
paths, dists, unc, iters = joint_astar(graph, landmarks, thr, NUM_AGENTS)
@printf("  %d iterations, primary dist %.3f, seed sigma %.4f  (%.1f s)\n",
        iters, dists[end], unc, time() - t0)

function yaml_value(path, key)
    for l in eachline(path)
        m = match(Regex("^\\s*" * key * ":\\s*(\\S+)"), l)
        m === nothing || return m.captures[1]
    end
    nothing
end
res = joinpath(ALGO_DIR, "results.yaml")
for (k, got) in (("astar_iterations", iters), ("primary_disc_unc", unc))
    want = yaml_value(res, k)
    want === nothing && continue
    ok = k == "astar_iterations" ? string(got) == want :
         abs(got - parse(Float64, want)) <= 1e-6 * max(1.0, abs(got))
    ok || @warn "re-run $k = $got but results.yaml says $want"
end

ctrls = seed_control_points(paths, graph)
seed_pts = [[(graph.landmarks[i].x, graph.landmarks[i].y) for i in p] for p in paths]
covs, arcs, comm, lmev = evaluate_joint_discrete(ctrls, landmarks, NUM_AGENTS)

jnum(x::Real) = isfinite(x) ? @sprintf("%.10g", x) : "null"
jstr(s) = '"' * String(s) * '"'
jlist(xs) = "[" * join(xs, ",") * "]"
jpt(p) = jlist((jnum(p[1]), jnum(p[2])))
jobj(kvs) = "{" * join((jstr(k) * ":" * v for (k, v) in kvs), ",") * "}"

doc = jobj([
    "astar_iterations" => string(iters),
    "seed_unc" => jnum(unc),
    "threshold" => jnum(thr),
    "primary_dist" => jnum(dists[end]),
    "agents" => jlist(jobj([
        "primary" => (a == NUM_AGENTS ? "true" : "false"),
        "nodes" => jlist(string(i) for i in paths[a]),
        "path" => jlist(jpt(p) for p in seed_pts[a]),
        "ctrls" => jlist(jpt(p) for p in ctrls[a]),
        "unc" => jlist(jnum(unc_radius(c)) for c in covs[a]),
    ]) for a in 1:NUM_AGENTS),
])
mkpath(dirname(OUT_PATH))
open(io -> println(io, doc), OUT_PATH, "w")
println("  -> $(OUT_PATH)")
