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

# Reads config/main.yaml, then merges config/<algo>.yaml for each algorithm
# named in main.yaml's `algorithms:` list. A planner's own config file only
# needs to exist when that planner is actually selected to run — so running
# just `straight_line` never requires `hexspline_cl.yaml` to be present/valid.
function load_config(dir::String)
    cfg = load_yaml(joinpath(dir, "main.yaml"))
    for algo in split_algorithms(cfg)
        path = joinpath(dir, "$(algo).yaml")
        isfile(path) && merge!(cfg, load_yaml(path))
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
const LANDMARK_SCENARIO  = Symbol(CFG["landmark_scenario"])

# Physical scale: all coordinates and distances in meters.
# Platform: AUV with DVL+IMU dead reckoning, acoustic landmark fixes.
const DIR_UNCERTAINTY_PER_METER  = Float64(CFG["dir_uncertainty_per_meter"])
const MAJ_MIN_UNC_RATIO          = Int(CFG["maj_min_unc_ratio"])
const PERP_UNCERTAINTY_PER_METER = DIR_UNCERTAINTY_PER_METER / MAJ_MIN_UNC_RATIO
const MARKER_PROPORTION          = Float64(CFG["marker_proportion"])
const SENSOR_NOISE               = Float64(CFG["sensor_noise"])
const BEARING_NOISE_RATIO        = Float64(CFG["bearing_noise_ratio"])
const COMM_RADIUS                = Float64(CFG["comm_radius"])
const VISIBILITY_RANGE           = Float64(CFG["visibility_range"])
const VISIBILITY_WIDTH           = Float64(CFG["visibility_width"])
const COMM_INTERVAL              = Float64(CFG["comm_interval"])
const COMM_RANGE                 = Float64(CFG["comm_range"])
const COMM_WIDTH                 = Float64(CFG["comm_width"])
const COMM_WEIGHT_MIN            = Float64(CFG["comm_weight_min"])
const COMM_INTERVAL_DIST         = Float64(CFG["comm_interval_dist"])
const COMM_FUSION                = Symbol(CFG["comm_fusion"])   # :ci (Covariance Intersection) | :kf (legacy)
const HEX_WIDTH_M                = Float64(CFG["hex_width_m"])
const HEX_RADIUS_M               = HEX_WIDTH_M / sqrt(3.0)
const HEX_PADDING                = Int(CFG["hex_padding"])
const TRACK_COMM_EVENTS          = Bool(CFG["track_comm_events"])
const TRACK_LANDMARK_EVENTS      = Bool(CFG["track_landmark_events"])
