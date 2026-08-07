# ----------------------
# Config loading
# ----------------------
function load_yaml(path::String)
    cfg = Dict{String, Any}()
    for line in eachline(path)
        line = strip(line)
        (isempty(line) || startswith(line, '#')) && continue
        idx = findfirst(':', line)
        idx === nothing && continue
        k = strip(line[1:prevind(line, idx)])
        v = line[nextind(line, idx):end]
        # Strip a trailing inline comment (e.g. "100.0  # note") before parsing.
        cidx = findfirst('#', v)
        v = strip(cidx === nothing ? v : v[1:prevind(v, cidx)])
        isempty(v) && continue
        if v == "true"; cfg[k] = true
        elseif v == "false"; cfg[k] = false
        else
            ni = tryparse(Int, v)
            if ni !== nothing; cfg[k] = ni
            else
                nf = tryparse(Float64, v)
                cfg[k] = nf !== nothing ? nf : v
            end
        end
    end
    return cfg
end

split_algorithms(cfg::Dict) = [strip(s) for s in split(String(cfg["algorithms"]), ",")]

# Planner variants that reuse another planner's engine (source .jl + tuning
# .yaml) rather than shipping their own. `straight_cont` (straight-line seed +
# continuous refinement) and `discrete_only` (A* seed, no continuous stage) are
# just the hexspline_cl engine run through a different pipeline, so they share
# its code and config. `engine_of(algo)` returns the owning engine (or the algo
# itself when it has none), used both here (config) and in generate_plan.jl
# (includes / config snapshot).
const PLANNER_ENGINE = Dict("straight_cont" => "hexspline_cl",
                            "discrete_only" => "hexspline_cl")
engine_of(algo) = get(PLANNER_ENGINE, String(algo), String(algo))

# Reads config/main.yaml, then merges config/<algo>.yaml for each algorithm
# named in main.yaml's `algorithms:` list. A planner's own config file only
# needs to exist when that planner is actually selected to run — so running
# just `straight_line` never requires `hexspline_cl.yaml` to be present/valid.
# Alias planners also pull their engine's .yaml (loaded first, so an optional
# per-alias override file can still win).
function load_config(dir::String)
    cfg = load_yaml(joinpath(dir, "main.yaml"))
    shared = Set(keys(cfg))
    for algo in split_algorithms(cfg)
        for name in unique([engine_of(algo), algo])
            path = joinpath(dir, "$(name).yaml")
            isfile(path) || continue
            algo_cfg = load_yaml(path)
            # A per-planner file silently shadowing a main.yaml key is a footgun:
            # main.yaml owns the shared problem constraints, so an override there
            # would let one algorithm be judged against a different feasible set
            # than the ablation it is compared to — invisibly, since the merged CFG
            # looks fine. Warn rather than error: an alias .yaml overriding its
            # engine's tuning is a documented use, and this only fires for keys
            # main.yaml itself defines.
            for k in intersect(shared, keys(algo_cfg))
                @warn "config: $(name).yaml overrides main.yaml key `$k` ($(cfg[k]) -> $(algo_cfg[k])). Shared keys — especially problem constraints — belong in main.yaml only."
            end
            merge!(cfg, algo_cfg)
        end
    end
    return cfg
end

const _CONFIG_DIR = isempty(ARGS) ? joinpath(@__DIR__, "..", "config") : ARGS[1]
const CFG = load_config(_CONFIG_DIR)
const ALGORITHMS = split_algorithms(CFG)

# ----------------------
# Shared constants — guaranteed present in config/main.yaml regardless of
# which algorithms are selected. Algorithm-specific constants (e.g. A*/
# B-spline tuning) are materialized inside their own planners/*.jl file
# instead, so they only need to exist when that planner actually runs.
# ----------------------
const NUM_AGENTS         = Int(CFG["num_agents"])
# ENV override resolved here (not in generate_plan.jl) so obstacles.jl can pick
# the scenario's `obstacles_<scenario>` key at include time.
const LANDMARK_SCENARIO  = Symbol(get(ENV, "SCENARIO", String(CFG["landmark_scenario"])))

# Physical scale: all coordinates and distances in meters.
# Platform: AUV with DVL+IMU dead reckoning, acoustic landmark fixes.
const DIR_UNCERTAINTY_PER_METER  = Float64(CFG["dir_uncertainty_per_meter"])
const MAJ_MIN_UNC_RATIO          = Int(CFG["maj_min_unc_ratio"])
const PERP_UNCERTAINTY_PER_METER = DIR_UNCERTAINTY_PER_METER / MAJ_MIN_UNC_RATIO
# Random-walk process-noise rates (m² of variance per m travelled), used by
# growth_covariance as Q = q·Δs. Linear in arc length, so a path's uncertainty
# no longer depends on how finely it was sampled.
#
# UNC_CALIB_SEG_M is a CALIBRATION constant, not a sensor datasheet number: it
# is the segment length at which this model reproduces the legacy quadratic
# form (q·Δs == (rate·Δs)² exactly when Δs == UNC_CALIB_SEG_M). It is pinned to
# 100 m — today's hex_width_m — so every uniform-hex path reports the number it
# reported before, and unc_radius_threshold / barrier tuning carry over
# unchanged. Deliberately NOT derived from HEX_WIDTH_M: changing the mesh must
# not rescale the physics, which is the resolution dependence being fixed here.
const UNC_CALIB_SEG_M = 100.0
const Q_DIR_PER_M     = DIR_UNCERTAINTY_PER_METER^2  * UNC_CALIB_SEG_M
const Q_PERP_PER_M    = PERP_UNCERTAINTY_PER_METER^2 * UNC_CALIB_SEG_M
const MARKER_PROPORTION          = Float64(CFG["marker_proportion"])
const LANDMARK_SENSOR_NOISE      = Float64(CFG["landmark_sensor_noise"])
const COMM_SENSOR_NOISE          = Float64(CFG["comm_sensor_noise"])
const BEARING_NOISE_RATIO        = Float64(CFG["bearing_noise_ratio"])
const COMM_RADIUS                = Float64(CFG["comm_radius"])
const VISIBILITY_RANGE           = Float64(CFG["visibility_range"])
const VISIBILITY_WIDTH           = Float64(CFG["visibility_width"])
const COMM_INTERVAL              = Float64(CFG["comm_interval"])
const COMM_RANGE                 = Float64(CFG["comm_range"])
const COMM_WIDTH                 = Float64(CFG["comm_width"])
const COMM_WEIGHT_MIN            = Float64(CFG["comm_weight_min"])
const COMM_INTERVAL_DIST         = Float64(CFG["comm_interval_dist"])

# Problem constraints — the feasible set, shared by every algorithm rather than
# owned by one planner's .yaml. An ablation is only a fair comparison if it is
# held to the same constraints as the full pipeline, so these live here even
# though hexspline_cl is currently their only consumer.
const UNC_RADIUS_THRESHOLD       = Float64(CFG["unc_radius_threshold"])
const UNC_FEAS_TOL               = Float64(CFG["unc_feas_tol"])
const MIN_TURN_RADIUS_M          = Float64(CFG["min_turn_radius_m"])
const MAX_CURVATURE              = 1.0 / MIN_TURN_RADIUS_M
const HEX_WIDTH_M                = Float64(CFG["hex_width_m"])
const HEX_RADIUS_M               = HEX_WIDTH_M / sqrt(3.0)
const HEX_PADDING                = Int(CFG["hex_padding"])
# Corridor extent, in metres, used by build_hex_graph. These do NOT derive from
# hex_width_m, so they must be retuned alongside it: the row pitch is 1.5·hex_r,
# and leaving the 260/300 defaults under a small hex would build hundreds of rows
# (Floyd-Warshall is O(n³) — it never finishes). Defaults are the values that
# were hardcoded in graph.jl, so an existing config that omits them is unchanged.
const CORRIDOR_HALFWIDTH_M       = Float64(get(CFG, "corridor_halfwidth_m", 260.0))
const CORRIDOR_Y_MAX_M           = Float64(get(CFG, "corridor_y_max_m",     300.0))
const TRACK_COMM_EVENTS          = Bool(CFG["track_comm_events"])
const TRACK_LANDMARK_EVENTS      = Bool(CFG["track_landmark_events"])
# Emission gates for the two DIAGNOSTIC output kinds. results.yaml is never
# gated — it is the run manifest, and batch harnesses read nothing else. Both
# default true, so an existing config that omits them is unchanged; a Monte
# Carlo sweep turns them off so a few hundred runs don't render thousands of
# PNGs nobody looks at.
const EMIT_FIGURES               = Bool(get(CFG, "emit_figures", true))
const EMIT_CSV                   = Bool(get(CFG, "emit_csv",     true))
