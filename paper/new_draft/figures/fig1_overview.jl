# ==========================================================================
# fig1_overview.jl — Fig. 1: the refined plan with a covariance ellipse at
# every inter-agent communication point, or every <spacing> m of arc
# ==========================================================================
#   julia paper/new_draft/figures/fig1_overview.jl <run_dir> [<out_dir>] [<spacing_m>]
#
# <spacing_m> > 0 draws an ellipse for every agent every <spacing_m> m of its
# own arc length (start and goal included) instead of only at the comm
# checkpoints; the output stems then carry an `_every<spacing>m` suffix.
# Default 0 = comm points only.
#
# <run_dir> is a generate_plan.jl output folder that ran hexspline_cl.
# Everything is re-derived from what that run wrote — its config snapshot
# (<run>/config), the landmark field it saw (scenario_landmarks.csv) and the
# shipped spline (hexspline_cl/csv/main_ctrls.csv). Covariances come from the
# planner's own evaluate_joint_discrete on the same spline samples
# optimize_continuous scored (eval_continuous in planners/hexspline_cl.jl),
# and the script refuses to draw unless that reproduces results.yaml's
# primary_unc: the ellipses are the run's numbers, not a re-simulation.
#
# Drawing only calls existing viz.jl helpers (make_base_plot,
# draw_covariance_ellipse!, overlay_comm_events!); nothing in src/ changes.
#
# Output (<out_dir> defaults to <run_dir>):
#   fig1_continuous_ellipses_gr.{png,svg,pdf}         GR preview, paths + ellipses
#   fig1_continuous_ellipses_gr_comm.{png,svg,pdf}    + the run's comm overlay
#   fig1_scene.json                                   the scene, for the renderer
#   fig1_continuous_ellipses.{png,svg,pdf,eps}        ← the deliverable
#   fig1_continuous_ellipses_comm.{png,svg,pdf,eps}   ← the deliverable
#
# The deliverable files are drawn a SECOND time, by fig1_overview_mpl.py, in
# the same matplotlib style as Figs. 3–4 (see make_figs_baseline.py). Reason:
# GR has no EPS writer, so the GR route had to go through Ghostscript
# eps2write, and PostScript has no transparency — every alpha'd fill in this
# figure (ellipses, obstacles, hex tiles) made Ghostscript flatten the whole
# page to one 10000×5600 bitmap, so nothing was selectable in Illustrator. The
# matplotlib route pre-blends alpha onto the background instead (over(), as in
# make_figs_baseline.py), so its EPS is real vector: one path per hex, per
# ellipse, per track. GR also outlines text in every vector format; matplotlib
# keeps live <text> in the SVG.
#
# The GR figures are kept as the visual reference the scene is checked against;
# they are no longer what the paper includes, hence the _gr stem and no EPS.
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", "..", ".."))
length(ARGS) >= 1 || error("usage: julia fig1_overview.jl <run_dir> [<out_dir>]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_DIR  = abspath_or(ARGS[1])
const OUT_DIR  = length(ARGS) >= 2 ? abspath_or(ARGS[2]) : RUN_DIR
const ELLIPSE_SPACING_M = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 0.0
ELLIPSE_SPACING_M >= 0 || error("spacing must be ≥ 0 (0 = comm points only)")
const CFG_DIR  = joinpath(RUN_DIR, "config")
const ALGO_DIR = joinpath(RUN_DIR, "hexspline_cl")
isdir(CFG_DIR)  || error("no config snapshot at $(CFG_DIR)")
isdir(ALGO_DIR) || error("no hexspline_cl output under $(RUN_DIR)")

# Ellipses are 2σ, with σ scaled by this factor for visibility (real σ is
# ~1–2 m on a 1 km plot). draw_covariance_ellipse!'s display_scale multiplies
# the COVARIANCE, so it gets the square.
const SIGMA_SCALE = 10.0

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so
# the run's snapshot is swapped in before the includes (same dance as
# plot_showcase.jl / mc_nees.jl). The snapshot pins the hex lattice, the sensor
# model and — for a `manual` scenario — the geometry itself.
empty!(ARGS); push!(ARGS, CFG_DIR)
using Plots, DataStructures, LinearAlgebra, Statistics, Random, Dates
Random.seed!(42)
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))   # ACTIVE_SCENARIO, OBSTACLES
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))     # bspline_sample_path, read_ctrls_csv

# ── What the run saw ─────────────────────────────────────────────────────
# The CSV is the authority on landmarks (a preset scenario draws random
# covariances; the snapshot cannot regenerate those, the CSV recorded them).
landmarks = read_landmarks_csv(joinpath(RUN_DIR, "scenario_landmarks.csv"))
let sc = ACTIVE_SCENARIO.landmarks
    same = length(sc) == length(landmarks) &&
           all(hypot(sc[i].x - landmarks[i].x, sc[i].y - landmarks[i].y) < 1e-9 for i in eachindex(sc))
    same || @warn "landmark positions in the config snapshot differ from scenario_landmarks.csv; using the CSV"
end
START_POS, GOAL_POS = ACTIVE_SCENARIO.start, ACTIVE_SCENARIO.goal
graph = build_hex_graph(landmarks, START_POS, GOAL_POS; hex_r = HEX_RADIUS_M)

ctrls = read_ctrls_csv(joinpath(ALGO_DIR, "csv", "main_ctrls.csv"))
na = length(ctrls)
na == NUM_AGENTS || error("main_ctrls.csv holds $(na) agents, config says $(NUM_AGENTS)")

# Same samples optimize_continuous scored: bspline_sample_path with its
# config defaults, then the waypoints-or-controls switch of eval_continuous.
wpts = [first(bspline_sample_path(c)) for c in ctrls]
eval_paths = CONT_UNC_USE_WAYPOINTS ? ctrls : wpts
covs, arcs, comm, lm_events = evaluate_joint_discrete(eval_paths, landmarks, na)

# ── Self-check against the run manifest ──────────────────────────────────
function yaml_value(path::String, key::String)
    for l in eachline(path)
        m = match(Regex("^\\s*" * key * ":\\s*(\\S+)"), l)
        m === nothing && continue
        return m.captures[1]
    end
    return nothing
end
results_yaml = joinpath(ALGO_DIR, "results.yaml")
prim_unc_str = yaml_value(results_yaml, "primary_unc")
(prim_unc_str === nothing || prim_unc_str == "null") &&
    error("$(results_yaml) reports no solution (primary_unc null)")
prim_unc_run = parse(Float64, prim_unc_str)
prim_unc_here = unc_radius(covs[end][end])
abs(prim_unc_here - prim_unc_run) <= 1e-6 * max(1.0, abs(prim_unc_run)) ||
    error("re-evaluated primary uncertainty $(prim_unc_here) ≠ results.yaml primary_unc $(prim_unc_run); " *
          "the snapshot/CSVs do not reproduce this run")
println("Self-check ✓ primary goal uncertainty $(round(prim_unc_here, digits = 4)) reproduces results.yaml")
println("  primary_length=$(yaml_value(results_yaml, "primary_length"))  " *
        "refinement_status=$(yaml_value(results_yaml, "refinement_status"))  " *
        "threshold=$(UNC_RADIUS_THRESHOLD)")

# ── Console table: what each comm event did ──────────────────────────────
# pre = last sample strictly before the checkpoint arc (covariance before the
# exchange), post = first sample at/after it (fused, plus at most one sample
# of extra dead reckoning). The same "post" index places the ellipses.
post_idx(a, t) = something(findfirst(>=(t - 1e-9), arcs[a]), length(arcs[a]))
pre_idx(a, t)  = max(1, something(findlast(<(t - 1e-9), arcs[a]), 1))
agent_name(a)  = a == na ? "primary" : "support $(a)"
println("\nComm events (arc, weight, distance, σ = det(Σ)^¼ before → after):")
for (t, a, b, w, pa, pb) in comm
    d = hypot(pa[1] - pb[1], pa[2] - pb[2])
    @printf("  arc %6.1f  w %.3f  d %6.1f m", t, w, d)
    for (ag, _) in ((a, pa), (b, pb))
        @printf("   %-9s %.3f → %.3f", agent_name(ag),
                unc_radius(covs[ag][pre_idx(ag, t)]), unc_radius(covs[ag][post_idx(ag, t)]))
    end
    println()
end
let seen = sort(unique(ev[2] for ev in lm_events))
    println("Landmark observations by: ", isempty(seen) ? "nobody" : join(agent_name.(seen), ", "))
end

# ── Figure ───────────────────────────────────────────────────────────────
agent_color(a) = a == na ? :blue : get(agent_colors, a, :gray)
# Supports are drawn with the same small y offset optimize_continuous uses, so
# co-located agents stay distinguishable.
y_offset(a) = a == na ? 0.0 : SUPPORT_PLOT_OFFSET_M * a

plt = make_base_plot(landmarks, graph)
for a in 1:na
    xs = [w[1] for w in wpts[a]]; ys = [w[2] + y_offset(a) for w in wpts[a]]
    plot!(plt, xs, ys, color = agent_color(a), linewidth = (a == na ? 2.2 : 1.3),
          label = agent_name(a))
end
# Each ellipse is recorded as it is drawn so the matplotlib renderer redraws
# exactly the ones GR did — same centre, same Σ, same colour.
const ELLIPSE_RECORDS = Any[]
function ellipse!(ag, pos, idx)
    x, y, cov = pos[1], pos[2] + y_offset(ag), covs[ag][idx]
    push!(ELLIPSE_RECORDS, (x = x, y = y, cov = cov, color = String(Symbol(agent_color(ag)))))
    draw_covariance_ellipse!(plt, x, y, cov; nstd = 2, color = agent_color(ag), alpha = 0.22,
                             display_scale = SIGMA_SCALE^2)
end
if ELLIPSE_SPACING_M > 0
    # Every <spacing> m of each agent's own arc, at the first sample at/after
    # that arc (same rule as the comm-point ellipses), start and goal included.
    # The comm checkpoints sit on this grid too (comm_interval is a multiple),
    # so the fused states are among these.
    for ag in 1:na
        nmax = floor(Int, arcs[ag][end] / ELLIPSE_SPACING_M + 1e-6)
        for k in 0:nmax
            idx = post_idx(ag, k * ELLIPSE_SPACING_M)
            ellipse!(ag, eval_paths[ag][idx], idx)
        end
    end
    ell_label = @sprintf("covariance every %d m (2σ, ×%d)", Int(ELLIPSE_SPACING_M), Int(SIGMA_SCALE))
else
    for (t, a, b, w, pa, pb) in comm
        for (ag, pos) in ((a, pa), (b, pb))
            ellipse!(ag, pos, post_idx(ag, t))
        end
    end
    ell_label = @sprintf("fused covariance (2σ, ×%d)", Int(SIGMA_SCALE))
end
# Legend entry for the ellipses (an off-plot marker; shapes carry no legend key).
scatter!(plt, [NaN], [NaN], marker = :circle, markersize = 9, color = :blue, alpha = 0.3,
         markerstrokewidth = 0, label = ell_label)
plot!(plt, xlabel = "x (m)", ylabel = "y (m)", size = (1000, 560))

# ── GR figures: the visual reference the matplotlib render is checked against.
# No EPS — Ghostscript's eps2write flattens this figure's alpha to a bitmap.
function save_all(p, stem::String)
    for ext in ("png", "svg", "pdf")
        savefig(p, "$(stem).$(ext)")
    end
    println("  → $(stem).{png,svg,pdf}")
end

mkpath(OUT_DIR)
suffix = ELLIPSE_SPACING_M > 0 ? @sprintf("_every%dm", Int(ELLIPSE_SPACING_M)) : ""
save_all(plt, joinpath(OUT_DIR, "fig1_continuous_ellipses_gr" * suffix))
plt_comm = deepcopy(plt)
overlay_comm_events!(plt_comm, comm)
save_all(plt_comm, joinpath(OUT_DIR, "fig1_continuous_ellipses_gr" * suffix * "_comm"))

# ── Scene dump ───────────────────────────────────────────────────────────
# Everything the figure draws, in world coordinates, so fig1_overview_mpl.py
# can redraw it without re-running the planner. Hand-rolled JSON: the depot
# carries no JSON package (same reason config parsing is hand-rolled).
jnum(x::Real) = @sprintf("%.6g", x)
jstr(s) = '"' * replace(String(s), "\\" => "\\\\", "\"" => "\\\"") * '"'
jlist(xs) = "[" * join(xs, ",") * "]"
jpt(p) = jlist((jnum(p[1]), jnum(p[2])))
jmat(m) = jlist(jlist(jnum(m[i, j]) for j in 1:size(m, 2)) for i in 1:size(m, 1))
jobj(kvs) = "{" * join((jstr(k) * ":" * v for (k, v) in kvs), ",") * "}"

let sensor_idx = findall(last(node_role_masks(graph)))
    scene = jobj([
        "stem"      => jstr("fig1_continuous_ellipses" * suffix),
        "xlabel"    => jstr("x (m)"),
        "ylabel"    => jstr("y (m)"),
        # sorted: route_tile_centers collects a Set, whose order is not stable
        "hex_radius"   => jnum(HEX_RADIUS_M),
        "hex_centers"  => jlist(jpt(c) for c in sort(route_tile_centers(graph))),
        "obstacles"    => jlist(jlist(jpt(v) for v in o.verts) for o in OBSTACLES),
        "sensor_landmarks" => jlist(jobj(["x" => jnum(graph.landmarks[i].x),
                                          "y" => jnum(graph.landmarks[i].y),
                                          "cov" => jmat(graph.landmarks[i].cov)])
                                    for i in sensor_idx),
        "landmark_display_scale" => jnum(400.0),   # make_base_plot's
        "start" => jpt((graph.landmarks[1].x, graph.landmarks[1].y)),
        "goal"  => jpt((graph.landmarks[graph.n].x, graph.landmarks[graph.n].y)),
        "agents" => jlist(jobj(["label"   => jstr(agent_name(a)),
                                "color"   => jstr(String(Symbol(agent_color(a)))),
                                "primary" => (a == na ? "true" : "false"),
                                "xs" => jlist(jnum(w[1]) for w in wpts[a]),
                                "ys" => jlist(jnum(w[2] + y_offset(a)) for w in wpts[a])])
                          for a in 1:na),
        "ellipses" => jlist(jobj(["x" => jnum(e.x), "y" => jnum(e.y),
                                  "cov" => jmat(e.cov), "color" => jstr(e.color)])
                            for e in ELLIPSE_RECORDS),
        "ellipse_nstd"          => "2",
        "ellipse_display_scale" => jnum(SIGMA_SCALE^2),
        "ellipse_label"         => jstr(ell_label),
        "comm" => jlist(jobj(["t" => jnum(t), "w" => jnum(w),
                              "pa" => jpt(pa), "pb" => jpt(pb)])
                        for (t, _, _, w, pa, pb) in comm),
    ])
    global SCENE_PATH = joinpath(OUT_DIR, "fig1_scene" * suffix * ".json")
    open(io -> println(io, scene), SCENE_PATH, "w")
    println("  → $(SCENE_PATH)")
end

# ── The paper's files: matplotlib, in the Figs. 3–4 style ────────────────
let script = joinpath(@__DIR__, "fig1_overview_mpl.py"),
    cands  = String[]
    haskey(ENV, "FIG1_PYTHON") && push!(cands, ENV["FIG1_PYTHON"])
    push!(cands, joinpath(homedir(), "Research", "multiagent_base", ".venv", "bin", "python"))
    for name in ("python3", "python")
        w = Sys.which(name); w === nothing || push!(cands, w)
    end
    py = findfirst(c -> isfile(c) && success(`$(c) -c "import matplotlib, numpy"`), cands)
    if py === nothing
        @warn "no python with matplotlib+numpy found; run it yourself:\n  <python> $(script) $(SCENE_PATH) $(OUT_DIR)"
    else
        run(`$(cands[py]) $(script) $(SCENE_PATH) $(OUT_DIR)`)
    end
end
