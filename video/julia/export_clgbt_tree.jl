# ==========================================================================
# export_clgbt_tree.jl - dump the CL-GBT search tree, no planner edit
# ==========================================================================
#   julia video/julia/export_clgbt_tree.jl <cfg_dir> <run_dir> <out_dir>
#
# clgbt_grow_tree already returns the full tree; it seeds its own RNG from
# clgbt_seed (42 in every config), so calling it here reproduces exactly the
# tree the run's own results.yaml reports (checked below). No planner edit was
# needed for this one -- only the export was missing.
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf
const _ROOT = normpath(joinpath(@__DIR__, "..", ".."))
length(ARGS) >= 3 || error("usage: julia export_clgbt_tree.jl <cfg_dir> <run_dir> <out_dir>")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const CFG_DIR = abspath_or(ARGS[1])
const RUN_DIR = abspath_or(ARGS[2])
const OUT_DIR = abspath_or(ARGS[3])

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
include(joinpath(_ROOT, "planners", "clgbt.jl"))

landmarks = read_landmarks_csv(joinpath(RUN_DIR, "scenario_landmarks.csv"))
graph = build_hex_graph(landmarks, ACTIVE_SCENARIO.start, ACTIVE_SCENARIO.goal;
                        hex_r = HEX_RADIUS_M)

@printf("Re-running clgbt_grow_tree (seed %d)...\n", CLGBT_SEED)
t0 = time()
tree, sol_idx, iters, paths = clgbt_grow_tree(graph, landmarks, NUM_AGENTS, NUM_AGENTS)
@printf("  %d iterations, %d tree nodes, solution at node %d  (%.1f s)\n",
        iters, length(tree), sol_idx, time() - t0)

function yaml_value(path, key)
    for l in eachline(path)
        m = match(Regex("^\\s*" * key * ":\\s*(\\S+)"), l)
        m === nothing || return m.captures[1]
    end
    nothing
end
res = joinpath(RUN_DIR, "clgbt", "results.yaml")
isfile(res) || (res = joinpath(RUN_DIR, "results.yaml"))  # flattened sweep rung
for (k, got) in (("clgbt_iterations", iters), ("clgbt_tree_size", length(tree)))
    want = yaml_value(res, k)
    want === nothing && continue
    string(got) == want || @warn "re-run $k = $got but results.yaml says $want"
end

# on_solution: every node on the parent chain from sol_idx to the root.
on_sol = falses(length(tree))
let k = sol_idx
    while k != 0
        on_sol[k] = true
        k = tree[k].parent
    end
end

mkpath(OUT_DIR)
open(joinpath(OUT_DIR, "clgbt_tree_nodes.csv"), "w") do io
    println(io, "node,parent,agent,x,y,heading,dist,unc,c11,c12,c22,on_solution")
    for (i, nd) in enumerate(tree)
        for a in 1:NUM_AGENTS
            x, y = nd.pos[a]
            c = nd.cov[a]
            @printf(io, "%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
                    i, nd.parent, a, x, y, nd.heading[a], nd.dist[a],
                    unc_radius(c), c[1,1], c[1,2], c[2,2], on_sol[i])
        end
    end
end
open(joinpath(OUT_DIR, "clgbt_solution.csv"), "w") do io
    println(io, "agent,i,x,y")
    for a in 1:NUM_AGENTS, (i, p) in enumerate(paths[a])
        @printf(io, "%d,%d,%.6f,%.6f\n", a, i, p[1], p[2])
    end
end
println("  -> ", joinpath(OUT_DIR, "clgbt_tree_nodes.csv"))
println("  -> ", joinpath(OUT_DIR, "clgbt_solution.csv"))
