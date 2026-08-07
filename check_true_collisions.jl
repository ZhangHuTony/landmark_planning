# Ground-truth collision check for the obstacle-robustness sweep.
#
# The planner's spec-E re-verification only tests each segment's *committed*
# face, so a reported breach can be a false alarm when the refined curve rounded
# the OTHER side of a (convex) obstacle. This script answers the real question:
# does the final continuous B-spline's MEAN trajectory geometrically enter any
# obstacle, and by how deep?
#
# It reuses the repo's exact clamped-spline evaluator (bspline_sample_path) and
# regenerates each run's obstacle field from the same seed the sweep used
# (gen_obstacles), so no per-run obstacle log is needed.
#
#   julia check_true_collisions.jl

empty!(ARGS); push!(ARGS, joinpath(@__DIR__, "config"))
include(joinpath(@__DIR__, "src", "config.jl"))
include(joinpath(@__DIR__, "src", "obstacles.jl"))
include(joinpath(@__DIR__, "src", "minvo.jl"))
include(joinpath(@__DIR__, "src", "graph.jl"))
include(joinpath(@__DIR__, "src", "covariance.jl"))
include(joinpath(@__DIR__, "planners", "hexspline_cl.jl"))
include(joinpath(@__DIR__, "test_obstacle_robustness.jl"))   # gen_obstacles, SCENARIOS, COUNTS, OUTROOT (guarded)

using Random, Printf
# obstacle_penetration (Σ=0 geometric containment) is shared from src/minvo.jl.

# Read a *_ctrls.csv → Dict(agent => control points).
function read_ctrls(path::String)
    agents = Dict{Int, Vector{Tuple{Float64,Float64}}}()
    for (i, line) in enumerate(eachline(path))
        i == 1 && continue                       # header
        f = split(line, ',')
        a = parse(Int, f[1]); x = parse(Float64, f[3]); y = parse(Float64, f[4])
        push!(get!(agents, a, Tuple{Float64,Float64}[]), (x, y))
    end
    return agents
end

# Deepest true penetration of one control-point file across all agents/obstacles.
function worst_collision(csv::String, obstacles::Vector{Obstacle})
    agents = read_ctrls(csv)
    worst = 0.0; where_ = ""
    for (a, ctrls) in sort(collect(agents))
        length(ctrls) < 2 && continue
        pts, _ = bspline_sample_path(ctrls; length_samples_per_seg = 40)
        for (x, y) in pts, (oi, obs) in enumerate(obstacles)
            p = obstacle_penetration(x, y, obs)
            if p > worst
                worst = p; where_ = "agent $a / obstacle $oi"
            end
        end
    end
    return worst, where_
end

rows = String[]
push!(rows, "| scenario | obstacles | main path collision (m) | worst any-seed collision (m) | where (worst) |")
push!(rows, "|---|---|---|---|---|")
for sc in SCENARIOS, n in COUNTS
    tag = "$(sc)_$(n)"
    dir = joinpath(OUTROOT, tag, "hexspline_cl", "csv")
    isdir(dir) || (push!(rows, "| $sc | $n | (no output) | | |"); continue)
    obstacles, _ = let (obs_str, _) = gen_obstacles(n, MersenneTwister(hash((sc, n)))); (parse_obstacles(obs_str), nothing) end

    main_csv = joinpath(dir, "main_ctrls.csv")
    main_col = isfile(main_csv) ? worst_collision(main_csv, obstacles)[1] : NaN

    worst = 0.0; wwhere = "—"
    for f in readdir(dir)
        endswith(f, "_ctrls.csv") || continue
        w, wl = worst_collision(joinpath(dir, f), obstacles)
        w > worst && (worst = w; wwhere = "$f  ($wl)")
    end
    push!(rows, "| $sc | $n | $(@sprintf("%.2f", main_col)) | $(@sprintf("%.2f", worst)) | $wwhere |")
    println("$tag: main=$(@sprintf("%.2f", main_col))  worst=$(@sprintf("%.2f", worst))  [$wwhere]")
end

report = join(rows, "\n") * "\n"
write(joinpath(OUTROOT, "TRUE_COLLISIONS.md"), report)
println("\n" * report)
