# Dump the EXACT generated geometry for a scenario as the `manual` encoding, so
# an A/B can pin it and vary hex_width_m alone. Must be run under config/mc so
# LM_BAND_INNER (= HEX_WIDTH_M + VISIBILITY_RANGE + 30) matches the sweep's.
using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates
const _ROOT = "/home/tony-zhang/Research/landmark_planning"
empty!(ARGS); push!(ARGS, joinpath(_ROOT, "config", "mc"))
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))

# (scenario_id, seed, D, n_landmarks, n_obstacles) straight out of scenarios.csv
const CASES = [("s005", 1072, 800.0, 3, 3),
               ("s012", 1079, 800.0, 4, 5),
               ("s029", 1096, 800.0, 4, 4),
               ("s030", 1097, 700.0, 3, 5)]

open(joinpath(@__DIR__, "geometry.txt"), "w") do io
    for (sid, seed, D, nl, no) in CASES
        sc = generate_scenario(seed = seed, goal_dist = D, n_landmarks = nl, n_obstacles = no)
        lm = join(["$(l.x),$(l.y) @ $(l.cov[1,1]),$(l.cov[1,2]),$(l.cov[2,1]),$(l.cov[2,2])"
                   for l in sc.landmarks], " | ")
        ob = join([join(["$(v[1]),$(v[2])" for v in o.verts], "; ") for o in sc.obstacles], " | ")
        println(io, "$(sid)\tstart\t0.0,0.0")
        println(io, "$(sid)\tgoal\t$(D),0.0")
        println(io, "$(sid)\tlandmarks\t$(lm)")
        println(io, "$(sid)\tobstacles\t$(ob)")
    end
end
println("wrote geometry.txt")
