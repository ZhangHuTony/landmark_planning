# ==========================================================================
# export_scene.jl — dump one planner run as scene.json for the Manim video
# ==========================================================================
#   julia video/julia/export_scene.jl <run_dir> [--algo=hexspline_cl]
#                                     [--out=<path.json>] [--alone] [--draw-seg=12]
#
# Everything on screen in the video comes from a run that already happened:
# the config snapshot that run wrote, the landmark field it saw, and the
# control points it shipped. Covariances are re-derived by the planner's OWN
# evaluate_joint_discrete on the same samples eval_continuous scored, and the
# script refuses to write unless that reproduces results.yaml's primary_unc.
# So the ellipses in the animation are the planner's numbers, not a replay
# model living in Python. Same contract as fig1_overview.jl, which this is a
# stripped-down clone of (no plotting, richer per-sample output).
#
# Two run layouts are accepted:
#   nested    <run>/config, <run>/scenario_*.csv, <run>/<algo>/{results.yaml,csv/}
#             (what generate_plan.jl writes)
#   flattened <rung>/{config,csv,results.yaml,scenario_*.csv}
#             (what run_constraint_sweep.jl leaves after flatten_algo_dir!)
#
# --alone additionally evaluates the PRIMARY's own path with na=1, i.e. the
# same route flown with no support at all. That is the "problem" scene: the
# comm loop in evaluate_joint_discrete runs `for a in 1:(na-1)`, so at na=1
# nothing fuses and the track is dead reckoning plus whatever landmarks the
# primary can see by itself.
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", ".."))
length(ARGS) >= 1 || error("usage: julia export_scene.jl <run_dir> [--algo=X] [--out=Y] [--alone]")

abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_DIR = abspath_or(ARGS[1])
_opt(name, dflt) = begin
    i = findfirst(a -> startswith(a, "--$(name)="), ARGS)
    i === nothing ? dflt : split(ARGS[i], "=", limit = 2)[2]
end
const ALGO      = _opt("algo", "hexspline_cl")
const WANT_ALONE = any(==("--alone"), ARGS)
const DRAW_SEG  = parse(Int, _opt("draw-seg", "12"))

# ── Resolve the layout ───────────────────────────────────────────────────
const FLAT = isfile(joinpath(RUN_DIR, "results.yaml"))
const ALGO_DIR = FLAT ? RUN_DIR : joinpath(RUN_DIR, ALGO)
const CFG_DIR  = isdir(joinpath(RUN_DIR, "config")) ? joinpath(RUN_DIR, "config") :
                 isdir(joinpath(RUN_DIR, "_cfg"))   ? joinpath(RUN_DIR, "_cfg")   :
                 error("no config snapshot (config/ or _cfg/) under $(RUN_DIR)")
isfile(joinpath(ALGO_DIR, "results.yaml")) || error("no results.yaml under $(ALGO_DIR)")
const OUT_PATH = abspath_or(_opt("out", joinpath(RUN_DIR, "scene.json")))

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so
# the run's own snapshot is swapped in before the includes (same dance as
# plot_showcase.jl / fig1_overview.jl / export_ladder_paths.jl).
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

# ── What the run saw ─────────────────────────────────────────────────────
# The CSV is the authority on landmarks: a preset scenario draws random
# covariances the snapshot alone cannot regenerate.
landmarks = read_landmarks_csv(joinpath(RUN_DIR, "scenario_landmarks.csv"))
START_POS, GOAL_POS = ACTIVE_SCENARIO.start, ACTIVE_SCENARIO.goal
graph = build_hex_graph(landmarks, START_POS, GOAL_POS; hex_r = HEX_RADIUS_M)

ctrls = read_ctrls_csv(joinpath(ALGO_DIR, "csv", "main_ctrls.csv"))
na = length(ctrls)
na == NUM_AGENTS || error("main_ctrls.csv holds $(na) agents, config says $(NUM_AGENTS)")

# The samples eval_continuous scored (config default per-segment count), and
# a denser set purely for drawing a smooth curve.
wpts       = [first(bspline_sample_path(c)) for c in ctrls]
draw_paths = [first(bspline_sample_path(c; length_samples_per_seg = DRAW_SEG)) for c in ctrls]
eval_paths = CONT_UNC_USE_WAYPOINTS ? ctrls : wpts
covs, arcs, comm, lm_events = evaluate_joint_discrete(eval_paths, landmarks, na)

cumarc(p) = (a = zeros(Float64, length(p));
             for i in 2:length(p); a[i] = a[i-1] + hypot(p[i][1]-p[i-1][1], p[i][2]-p[i-1][2]); end; a)

# ── Self-check against the run manifest ──────────────────────────────────
function yaml_value(path::String, key::String)
    for l in eachline(path)
        m = match(Regex("^\\s*" * key * ":\\s*(\\S+)"), l)
        m === nothing && continue
        return m.captures[1]
    end
    return nothing
end
const RESULTS = joinpath(ALGO_DIR, "results.yaml")
prim_unc_str = yaml_value(RESULTS, "primary_unc")
(prim_unc_str === nothing || prim_unc_str == "null") &&
    error("$(RESULTS) reports no solution (primary_unc null) — nothing to export")
prim_unc_run  = parse(Float64, prim_unc_str)
prim_unc_here = unc_radius(covs[end][end])
abs(prim_unc_here - prim_unc_run) <= 1e-6 * max(1.0, abs(prim_unc_run)) ||
    error("re-evaluated primary uncertainty $(prim_unc_here) ≠ results.yaml $(prim_unc_run); " *
          "the snapshot/CSVs do not reproduce this run")
@printf("Self-check ok: primary σ_T %.6f reproduces results.yaml (%s)\n", prim_unc_here, ALGO)

# ── The same route with no support (scene 1) ─────────────────────────────
alone = nothing
if WANT_ALONE
    a_covs, a_arcs, _, _ = evaluate_joint_discrete([eval_paths[na]], landmarks, 1)
    alone = (arc = a_arcs[1], cov = a_covs[1], pos = eval_paths[na])
    @printf("  alone: primary σ_T without the support = %.6f (bound %.4f)\n",
            unc_radius(a_covs[1][end]), UNC_RADIUS_THRESHOLD)
end

# ── Comm checkpoints, including the ones that were out of range ──────────
# evaluate_joint_discrete only records a fusion when the weight clears
# COMM_WEIGHT_MIN, so a scene that only drew `comm` would show nothing at the
# checkpoints where the link was too weak. The weight is recomputed HERE, in
# Julia, by the planner's own comm_weight, so Python never evaluates the taper.
function pos_at_arc(path, arc, s)
    s <= 0 && return path[1]
    s >= arc[end] && return path[end]
    j = searchsortedfirst(arc, s)
    j <= 1 && return path[1]
    t = (s - arc[j-1]) / max(arc[j] - arc[j-1], 1e-12)
    (path[j-1][1] + t*(path[j][1]-path[j-1][1]), path[j-1][2] + t*(path[j][2]-path[j-1][2]))
end
checkpoints = Any[]
if na >= 2
    max_arc = maximum(arcs[a][end] for a in 1:na)
    for s in 0.0:COMM_INTERVAL:max_arc
        pp = pos_at_arc(eval_paths[na], arcs[na], s)
        for b in 1:(na-1)
            pb = pos_at_arc(eval_paths[b], arcs[b], s)
            d2 = (pp[1]-pb[1])^2 + (pp[2]-pb[2])^2
            w  = comm_weight(d2)
            push!(checkpoints, (arc = s, a = b, b = na, pa = pb, pb = pp,
                                d = sqrt(d2), w = w, fused = w > COMM_WEIGHT_MIN))
        end
    end
end

# ── JSON (hand-rolled: the depot carries no JSON package) ────────────────
jnum(x::Real) = isfinite(x) ? @sprintf("%.10g", x) : "null"
jstr(s) = '"' * replace(String(s), "\\" => "\\\\", "\"" => "\\\"") * '"'
jlist(xs) = "[" * join(xs, ",") * "]"
jpt(p) = jlist((jnum(p[1]), jnum(p[2])))
jmat(m) = jlist(jlist(jnum(m[i, j]) for j in 1:size(m, 2)) for i in 1:size(m, 1))
jbool(b) = b ? "true" : "false"
jobj(kvs) = "{" * join((jstr(k) * ":" * v for (k, v) in kvs), ",") * "}"
# Σ is symmetric: three numbers per sample instead of four nested lists keeps
# a 200-sample two-agent track readable and roughly halves the file.
jcov3(c) = jlist((jnum(c[1,1]), jnum(c[1,2]), jnum(c[2,2])))
jyaml(k) = (v = yaml_value(RESULTS, k); v === nothing || v == "null" ? "null" : v)

agent_name(a) = a == na ? "primary" : "support $(a)"

sensor_idx = findall(last(node_role_masks(graph)))
scene = jobj([
    "run" => jobj(["dir" => jstr(RUN_DIR), "algo" => jstr(ALGO),
                   "scenario" => jstr(String(LANDMARK_SCENARIO))]),
    "cfg" => jobj([
        "unc_radius_threshold" => jnum(UNC_RADIUS_THRESHOLD),
        "comm_interval" => jnum(COMM_INTERVAL), "comm_range" => jnum(COMM_RANGE),
        "comm_width" => jnum(COMM_WIDTH), "comm_weight_min" => jnum(COMM_WEIGHT_MIN),
        "visibility_range" => jnum(VISIBILITY_RANGE), "hex_width_m" => jnum(HEX_WIDTH_M),
        "num_agents" => string(NUM_AGENTS),
        "dir_uncertainty_per_meter" => jnum(DIR_UNCERTAINTY_PER_METER),
        "cont_unc_use_waypoints" => jbool(CONT_UNC_USE_WAYPOINTS)]),
    "results" => jobj([
        "primary_length" => jyaml("primary_length"),
        "primary_unc" => jyaml("primary_unc"),
        "primary_disc_unc" => jyaml("primary_disc_unc"),
        "continuous_primary_length" => jyaml("continuous_primary_length"),
        "astar_iterations" => jyaml("astar_iterations"),
        "clgbt_iterations" => jyaml("clgbt_iterations"),
        "refinement_status" => jstr(something(yaml_value(RESULTS, "refinement_status"), "unknown"))]),
    "hex" => jobj(["radius" => jnum(HEX_RADIUS_M),
                   "centers" => jlist(jpt(c) for c in sort(route_tile_centers(graph)))]),
    # every graph node, so an A* expansion trace can be resolved to geometry
    "nodes" => jlist(jlist((string(i), jnum(graph.landmarks[i].x), jnum(graph.landmarks[i].y)))
                     for i in 1:graph.n),
    "obstacles" => jlist(jlist(jpt(v) for v in o.verts) for o in OBSTACLES),
    "landmarks" => jlist(jobj(["x" => jnum(graph.landmarks[i].x),
                               "y" => jnum(graph.landmarks[i].y),
                               "cov" => jmat(graph.landmarks[i].cov)]) for i in sensor_idx),
    "start" => jpt((graph.landmarks[1].x, graph.landmarks[1].y)),
    "goal"  => jpt((graph.landmarks[graph.n].x, graph.landmarks[graph.n].y)),
    "agents" => jlist(begin
        da = cumarc(draw_paths[a])
        jobj(["label" => jstr(agent_name(a)),
              "primary" => jbool(a == na),
              "ctrls" => jlist(jpt(p) for p in ctrls[a]),
              # dense, for drawing the curve
              "draw" => jobj(["x" => jlist(jnum(p[1]) for p in draw_paths[a]),
                              "y" => jlist(jnum(p[2]) for p in draw_paths[a]),
                              "arc" => jlist(jnum(s) for s in da)]),
              # the samples the planner actually scored, with their Σ
              "eval" => jobj(["basis" => jstr(CONT_UNC_USE_WAYPOINTS ? "controls" : "waypoints"),
                              "x" => jlist(jnum(p[1]) for p in eval_paths[a]),
                              "y" => jlist(jnum(p[2]) for p in eval_paths[a]),
                              "arc" => jlist(jnum(s) for s in arcs[a]),
                              "cov" => jlist(jcov3(c) for c in covs[a]),
                              "unc" => jlist(jnum(unc_radius(c)) for c in covs[a])])])
    end for a in 1:na),
    "alone" => (alone === nothing ? "null" :
        jobj(["x" => jlist(jnum(p[1]) for p in alone.pos),
              "y" => jlist(jnum(p[2]) for p in alone.pos),
              "arc" => jlist(jnum(s) for s in alone.arc),
              "cov" => jlist(jcov3(c) for c in alone.cov),
              "unc" => jlist(jnum(unc_radius(c)) for c in alone.cov)])),
    "comm" => jlist(jobj(["arc" => jnum(t), "a" => string(a), "b" => string(b),
                          "w" => jnum(w), "pa" => jpt(pa), "pb" => jpt(pb)])
                    for (t, a, b, w, pa, pb) in comm),
    "checkpoints" => jlist(jobj(["arc" => jnum(c.arc), "a" => string(c.a), "b" => string(c.b),
                                 "d" => jnum(c.d), "w" => jnum(c.w), "fused" => jbool(c.fused),
                                 "pa" => jpt(c.pa), "pb" => jpt(c.pb)]) for c in checkpoints),
    "landmark_events" => jlist(jobj(["arc" => jnum(t), "agent" => string(ag),
                                     "lm" => string(li), "q" => jnum(q),
                                     "pa" => jpt(pa), "pl" => jpt(pl)])
                               for (t, ag, li, q, pa, pl) in lm_events),
])
mkpath(dirname(OUT_PATH))
open(io -> println(io, scene), OUT_PATH, "w")
@printf("  -> %s (%.1f KB)\n", OUT_PATH, filesize(OUT_PATH)/1024)
