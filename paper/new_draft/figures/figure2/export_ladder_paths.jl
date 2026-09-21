# ==========================================================================
# export_ladder_paths.jl — dump Fig. 2's spline paths as sampled points
# ==========================================================================
#   julia paper/new_draft/figures/figure2/export_ladder_paths.jl \
#         <sweep_root> [pcts] [out_csv] [methods]
#
# Companion to plot_fig2_ladder_mpl.py, which draws Fig. 2 in matplotlib. The
# panels' curves are clamped cubic B-splines and the runs save only their
# control points (csv/main_ctrls.csv), so something has to evaluate the spline.
# That is done HERE, by the planner's own bspline_sample_path, rather than
# reimplemented in Python: one sampler, no second copy of the math to drift.
#
# Writes method,pct,agent,i,x,y — one row per sampled point, the same
# `length_samples_per_seg = 12` plot_ladder_grid.jl draws with. Everything else
# the figure needs (obstacles, landmarks, lengths, uncertainties) is already
# beside each run as CSV, so the Python side reads that directly.
# ==========================================================================

using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", "..", "..", ".."))
length(ARGS) >= 1 || error("usage: julia export_ladder_paths.jl <sweep_root> [pcts] [out_csv] [methods]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_ROOT = abspath_or(ARGS[1])
const PCTS     = length(ARGS) >= 2 ? parse.(Int, split(ARGS[2], ',')) : [100, 70, 50, 30]
const OUT_CSV  = length(ARGS) >= 3 ? abspath_or(ARGS[3]) : joinpath(RUN_ROOT, "figures", "fig2_paths.csv")
const ONLY     = length(ARGS) >= 4 ? String.(split(ARGS[4], ',')) : String[]
isdir(RUN_ROOT) || error("no such sweep root: $(RUN_ROOT)")

const METHOD_DIRS = filter(d -> isfile(joinpath(RUN_ROOT, d, "trials.csv")),
                           filter(d -> isdir(joinpath(RUN_ROOT, d)), readdir(RUN_ROOT)))
isempty(METHOD_DIRS) && error("no method folder with a trials.csv under $(RUN_ROOT)")
const PREFERRED = ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]
const METHODS = isempty(ONLY) ?
    vcat(filter(m -> m in METHOD_DIRS, PREFERRED),
         sort(filter(m -> !(m in PREFERRED), METHOD_DIRS))) :
    (all(m -> m in METHOD_DIRS, ONLY) ? ONLY : error("no trials.csv for some of $(ONLY) under $(RUN_ROOT)"))

rung_dir(m, pct) = joinpath(RUN_ROOT, m, "s001_p$(lpad(pct, 3, '0'))")
const CFG_DIR = let c = filter(isdir, [joinpath(rung_dir(m, p), "_cfg") for m in METHODS for p in PCTS])
    isempty(c) && error("no _cfg snapshot under any requested rung")
    first(c)
end

# src/config.jl reads its config directory from ARGS[1]; hand it the rung's
# snapshot so the spline degree is the one that actually ran.
empty!(ARGS); push!(ARGS, CFG_DIR)
using DataStructures, LinearAlgebra, Statistics, Random, Dates
Random.seed!(42)
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))               # read_ctrls_csv
include(joinpath(_ROOT, "planners", "hexspline_cl.jl")) # bspline_sample_path

mkpath(dirname(OUT_CSV))
open(OUT_CSV, "w") do io
    println(io, "method,pct,agent,i,x,y")
    for m in METHODS, pct in PCTS
        csv = joinpath(rung_dir(m, pct), "csv", "main_ctrls.csv")
        isfile(csv) || continue
        ctrls = read_ctrls_csv(csv)
        for (a, ctrl) in enumerate(ctrls)
            length(ctrl) < 2 && continue
            pts = first(bspline_sample_path(ctrl; length_samples_per_seg = 12))
            for (i, q) in enumerate(pts)
                @printf(io, "%s,%d,%d,%d,%.6f,%.6f\n", m, pct, a, i, q[1], q[2])
            end
        end
    end
end
println("  → $(OUT_CSV)")
