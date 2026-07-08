using Plots
using DataStructures
using LinearAlgebra
using Statistics
using Random
using Dates

Random.seed!(42)  # Reproducible jittered grid sampling

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))
include(joinpath(_ROOT, "src", "analysis.jl"))
for algo in ALGORITHMS
    include(joinpath(_ROOT, "planners", "$(algo).jl"))
end

const OUTPUT_DIR = joinpath("results", Dates.format(now(), "yyyy-mm-dd_HH-MM-SS"))
mkpath(OUTPUT_DIR)

# Snapshot only main.yaml + the selected algorithms' own config files — not
# every *.yaml sitting in config/, so the saved run only documents what
# actually ran.
run_config_dir = joinpath(OUTPUT_DIR, "config")
mkpath(run_config_dir)
cp(joinpath(_CONFIG_DIR, "main.yaml"), joinpath(run_config_dir, "main.yaml"); force=true)
for algo in ALGORITHMS
    algo_cfg = joinpath(_CONFIG_DIR, "$(algo).yaml")
    isfile(algo_cfg) && cp(algo_cfg, joinpath(run_config_dir, "$(algo).yaml"); force=true)
end

# ── Scenario + graph (shared across every selected planner) ──
const RUN_SCENARIO = Symbol(get(ENV, "SCENARIO", string(LANDMARK_SCENARIO)))
landmarks = make_scattered_landmarks(RUN_SCENARIO)
START_POS, GOAL_POS = scenario_endpoints(RUN_SCENARIO)
scenario = (landmarks=landmarks, start=START_POS, goal=GOAL_POS)
println("Landmark scenario: $(RUN_SCENARIO) — $(length(landmarks)) landmarks")

graph = build_hex_graph(landmarks, START_POS, GOAL_POS; hex_r=HEX_RADIUS_M)

# ponytail: eval-only mode — replay a saved (or externally-produced) path CSV, skip planning
if haskey(ENV, "EVAL_PATHS_CSV")
    run_path_eval(ENV["EVAL_PATHS_CSV"], landmarks, graph, OUTPUT_DIR)
    exit()
end

# ── Run every selected planner, each into its own output subfolder ──
for algo in ALGORITHMS
    algo_dir = joinpath(OUTPUT_DIR, algo)
    mkpath(algo_dir)
    plan_fn = getfield(Main, Symbol("plan_$(algo)"))
    solutions = plan_fn(scenario, graph, algo_dir)
    println("\n[$(algo)] produced $(length(solutions)) solution(s) -> $(algo_dir)")
end
