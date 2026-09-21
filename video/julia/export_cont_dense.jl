# ==========================================================================
# export_cont_dense.jl - sample every logged refinement iterate as a spline
# ==========================================================================
#   julia video/julia/export_cont_dense.jl <cfg_dir> <cont_trace.csv> <out_dir> [every]
#
# The refinement trace records CONTROL POINTS per accepted iterate. Drawing the
# curve they define means evaluating the same clamped cubic B-spline the
# planner evaluates, so it is done here with the planner's own
# bspline_sample_path rather than reimplemented in Python -- one sampler, no
# second copy of the math to drift (the same argument
# figures/figure2/export_ladder_paths.jl makes).
#
# Writes:
#   cont_dense.csv   iter,agent,i,x,y     the curve per kept iterate
#   cont_steps.csv   iter,stage,mu,len,unc,feasible,min_slack,backtracks
#                    one row per kept iterate (the trace's per-iterate scalars)
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf
const _ROOT = normpath(joinpath(@__DIR__, "..", ".."))
length(ARGS) >= 3 || error("usage: julia export_cont_dense.jl <cfg_dir> <trace.csv> <out_dir> [every]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const CFG_DIR   = abspath_or(ARGS[1])
const TRACE_CSV = abspath_or(ARGS[2])
const OUT_DIR   = abspath_or(ARGS[3])
const EVERY     = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 5
const SEG       = 12

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

# ── read the trace ───────────────────────────────────────────────────────
rows = Dict{Int, Dict{Int, Vector{Tuple{Float64,Float64}}}}()
meta = Dict{Int, NTuple{7, Any}}()
open(TRACE_CSV) do io
    readline(io)
    for line in eachline(io)
        f = split(line, ',')
        length(f) < 12 && continue
        it = parse(Int, f[1]); ag = parse(Int, f[9])
        x = parse(Float64, f[11]); y = parse(Float64, f[12])
        push!(get!(get!(rows, it, Dict{Int, Vector{Tuple{Float64,Float64}}}()), ag,
                   Tuple{Float64,Float64}[]), (x, y))
        haskey(meta, it) || (meta[it] = (f[2], f[3], parse(Float64, f[4]),
                                         parse(Float64, f[5]), f[6], f[7], f[8]))
    end
end
iters = sort(collect(keys(rows)))
last_it = iters[end]
# always keep the seed and the shipped result; thin the middle
keep = [it for it in iters if it == 0 || it == last_it || it <= 40 || it % EVERY == 0]
@printf("%d iterates logged, keeping %d (seed, every %d, and the last)\n",
        length(iters), length(keep), EVERY)

mkpath(OUT_DIR)
open(joinpath(OUT_DIR, "cont_dense.csv"), "w") do io
    println(io, "iter,agent,i,x,y")
    for it in keep, ag in sort(collect(keys(rows[it])))
        pts, _ = bspline_sample_path(rows[it][ag]; length_samples_per_seg = SEG)
        for (i, p) in enumerate(pts)
            @printf(io, "%d,%d,%d,%.6f,%.6f\n", it, ag, i, p[1], p[2])
        end
    end
end
# The RAW control points of each kept iterate -- not resampled, so an
# animation can move a fixed number of dots (one per hex node) smoothly
# between iterates instead of resampling a changing-length curve. Iteration 0
# here is seed_control_points itself, i.e. the hex cell centres exactly.
open(joinpath(OUT_DIR, "cont_ctrls.csv"), "w") do io
    println(io, "iter,agent,ctrl_index,x,y")
    for it in keep, ag in sort(collect(keys(rows[it])))
        for (k, p) in enumerate(rows[it][ag])
            @printf(io, "%d,%d,%d,%.6f,%.6f\n", it, ag, k, p[1], p[2])
        end
    end
end
open(joinpath(OUT_DIR, "cont_steps.csv"), "w") do io
    println(io, "iter,stage,mu,len,unc,feasible,min_slack,backtracks")
    for it in keep
        m = meta[it]
        @printf(io, "%d,%s,%s,%.6f,%.6f,%s,%s,%s\n", it, m[1], m[2], m[3], m[4], m[5], m[6], m[7])
    end
end
@printf("  len %.2f -> %.2f m,  sigma %.4f -> %.4f\n",
        meta[0][3], meta[last_it][3], meta[0][4], meta[last_it][4])
println("  -> ", joinpath(OUT_DIR, "cont_dense.csv"))
