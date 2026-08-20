# ==========================================================================
# plot_showcase.jl — the constraint ladder, drawn
# ==========================================================================
# Turns a `scenario_name`-mode run of run_constraint_sweep.jl (see
# config/mc/sweep_showcase.yaml) into the figures that are the point of it:
# what the PLAN looks like at each rung of the uncertainty constraint, not the
# aggregate ratio curves summarize() draws for a random population.
#
#   julia plot_showcase.jl showcase/<tag>
#
# Reads, per rung, <root>/<method>/s001_p<NNN>/csv/main_ctrls.csv (needs the
# sweep's `save_csv: true`) plus <root>/<method>/trials.csv for the threshold and
# the achieved length/uncertainty. Config comes from a rung's own `_cfg` snapshot,
# so the hex lattice, the scenario and the sensor model are exactly what ran —
# nothing here re-derives them.
#
# Output: <root>/figures/fig_ladder_{overlay,panels,staircase}.png
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = @__DIR__
const RUN_ROOT = let a = isempty(ARGS) ? error("usage: julia plot_showcase.jl <sweep root>") : ARGS[1]
    isabspath(a) ? a : joinpath(_ROOT, a)
end
isdir(RUN_ROOT) || error("no such sweep root: $(RUN_ROOT)")

# The method folder is whichever one holds a trials.csv — this file does not need
# to know the label, and a showcase sweep runs exactly one method.
const METHOD_DIR = let cands = filter(d -> isfile(joinpath(RUN_ROOT, d, "trials.csv")),
                                      filter(d -> isdir(joinpath(RUN_ROOT, d)), readdir(RUN_ROOT)))
    length(cands) == 1 || error("expected exactly one method folder with a trials.csv under " *
                                "$(RUN_ROOT), found $(cands)")
    joinpath(RUN_ROOT, cands[1])
end

# Any rung's _cfg is the config that rung ran under; they differ only in
# unc_radius_threshold, which plotting does not read. src/config.jl consumes
# ARGS[1] as its config DIRECTORY at include time, so it must be swapped in
# before the includes (same dance as mc_nees.jl / run_constraint_sweep.jl).
const RUNG_DIRS = sort(filter(d -> occursin(r"_p\d+$", d) && isdir(joinpath(METHOD_DIR, d)),
                              readdir(METHOD_DIR)))
isempty(RUNG_DIRS) && error("no <scenario>_p<pct> run folders under $(METHOD_DIR)")
const CFG_DIR = let c = joinpath(METHOD_DIR, RUNG_DIRS[end], "_cfg")
    isdir(c) || error("no config snapshot at $(c)")
    c
end

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
include(joinpath(_ROOT, "planners", "hexspline_cl.jl"))   # bspline_sample_path

# ── Rungs ────────────────────────────────────────────────────────────────
# trials.csv is the authority on what a rung achieved; the ctrls CSV is only its
# geometry. A rung with no ctrls file (a failure, or save_csv off) is kept in the
# staircase and skipped in the path figures — dropping it entirely would hide
# where the planner gives out, which is half of what the ladder shows.
function read_table(path::String)
    isfile(path) || error("missing $(path)")
    lines = filter(!isempty, strip.(readlines(path)))
    hdr = String.(split(lines[1], ','))
    [Dict(zip(hdr, String.(split(l, ',')))) for l in lines[2:end]]
end

num(r, k) = (v = get(r, k, ""); isempty(v) || v == "null" ? nothing : tryparse(Float64, v))

rungs = []
for r in read_table(joinpath(METHOD_DIR, "trials.csv"))
    pct = parse(Int, r["pct"])
    csv = joinpath(METHOD_DIR, "$(r["scenario_id"])_p$(lpad(pct,3,'0'))", "csv", "main_ctrls.csv")
    push!(rungs, (pct   = pct,
                  thr   = something(num(r, "threshold"), NaN),
                  ok    = r["success"] == "true",
                  len   = num(r, "primary_length"),
                  unc   = num(r, "primary_unc"),
                  why   = get(r, "fail_reason", ""),
                  ctrls = isfile(csv) ? read_ctrls_csv(csv) : nothing))
end
sort!(rungs, by = x -> -x.pct)          # loosest first
isempty(rungs) && error("trials.csv holds no rows")

# ── Distinct plans ───────────────────────────────────────────────────────
# Consecutive rungs often ship the SAME plan — the constraint is slack there, so
# nothing moved. Collapsing them is what makes the panel figure readable: one
# panel per plan, labelled with the loosest and tightest bound it survives. The
# comparison is on control points, not length, because two different routes can
# tie on length to float noise (see the formation slot-ranking note in LOGS).
same(a, b) = a !== nothing && b !== nothing && length(a) == length(b) &&
             all(length(a[i]) == length(b[i]) &&
                 all(hypot(a[i][j][1] - b[i][j][1], a[i][j][2] - b[i][j][2]) < 1.0
                     for j in eachindex(a[i])) for i in eachindex(a))

groups = []                             # (rungs sharing one plan) — successes only
for r in rungs
    r.ok && r.ctrls !== nothing || continue
    if !isempty(groups) && same(groups[end][end].ctrls, r.ctrls)
        push!(groups[end], r)
    else
        push!(groups, [r])
    end
end
isempty(groups) && error("no successful rung has a main_ctrls.csv — was the sweep run with save_csv: true?")

# The scenario/graph the runs planned on. build_hex_graph runs Floyd-Warshall, so
# this is the expensive line in the file; it is paid once for every figure.
const SCEN  = ACTIVE_SCENARIO
const GRAPH = build_hex_graph(SCEN.landmarks, SCEN.start, SCEN.goal; hex_r = HEX_RADIUS_M)

sample(ctrl) = first(bspline_sample_path(ctrl; length_samples_per_seg = 12))
xs(pts) = [p[1] for p in pts]
ys(pts) = [p[2] for p in pts]

# Loose → tight, so the eye reads the sequence as the constraint tightening.
rung_color(i, n) = n == 1 ? cgrad(:viridis)[0.0] : cgrad(:viridis)[(i - 1) / (n - 1)]
lbl(g) = length(g) == 1 ? @sprintf("σ ≤ %.2f m", g[1].thr) :
                          @sprintf("σ ≤ %.2f–%.2f m", g[end].thr, g[1].thr)

# ── 1. Overlay: every distinct plan on one map ───────────────────────────
plt = make_base_plot(SCEN.landmarks, GRAPH)
for (i, g) in enumerate(groups)
    c = rung_color(i, length(groups))
    ctrls = g[1].ctrls
    for (a, ctrl) in enumerate(ctrls)
        length(ctrl) < 2 && continue
        pts = sample(ctrl)
        primary = a == length(ctrls)
        plot!(plt, xs(pts), ys(pts); color = c, linewidth = primary ? 3 : 1.6,
              linestyle = primary ? :solid : :dash,
              label = primary ? lbl(g) : false)
    end
end
plot!(plt, title = "hexspline_cl: the plan vs. the uncertainty bound (solid = primary, dashed = support)",
      size = (1200, 620))
figdir = joinpath(RUN_ROOT, "figures"); mkpath(figdir)
savefig(plt, joinpath(figdir, "fig_ladder_overlay.png"))

# ── 2. Panels: one per distinct plan ─────────────────────────────────────
panels = Plots.Plot[]
for (i, g) in enumerate(groups)
    p = make_base_plot(SCEN.landmarks, GRAPH)
    plot!(p, legend = false)
    c = rung_color(i, length(groups))
    ctrls = g[1].ctrls
    for (a, ctrl) in enumerate(ctrls)
        length(ctrl) < 2 && continue
        pts = sample(ctrl)
        primary = a == length(ctrls)
        plot!(p, xs(pts), ys(pts); color = c, linewidth = primary ? 3 : 1.6,
              linestyle = primary ? :solid : :dash)
    end
    plot!(p, title = @sprintf("%s   len %.0f m   unc %.2f", lbl(g), g[1].len, g[1].unc),
          titlefontsize = 9)
    push!(panels, p)
end
savefig(plot(panels..., layout = (length(panels), 1), size = (1000, 330 * length(panels))),
        joinpath(figdir, "fig_ladder_panels.png"))

# ── 3. Staircase: primary length vs. the bound ──────────────────────────
ok = filter(r -> r.ok && r.len !== nothing, rungs)
top = plot(xlabel = "uncertainty bound σ (m)", ylabel = "primary path length (m)",
           title = "what the constraint costs", legend = :topright,
           xflip = true, size = (900, 380))
plot!(top, [r.thr for r in ok], [r.len for r in ok];
      seriestype = :steppre, color = "#2a78d6", linewidth = 2,
      marker = :circle, ms = 4, markerstrokewidth = 0, label = "primary length")
# Where the plan actually changed shape — the rungs the panels above show.
vline!(top, [g[1].thr for g in groups[2:end]], color = :gray60, ls = :dash, lw = 1,
       label = "plan changes")
bot = plot(xlabel = "uncertainty bound σ (m)", ylabel = "achieved unc  det(Σ)^0.25",
           legend = :topleft, xflip = true)
plot!(bot, [r.thr for r in ok], [r.unc for r in ok]; color = "#eb6834", linewidth = 2,
      marker = :circle, ms = 4, markerstrokewidth = 0, label = "achieved")
plot!(bot, [r.thr for r in ok], [r.thr for r in ok]; color = :red, ls = :dot, lw = 1.5,
      label = "the bound")
savefig(plot(top, bot, layout = grid(2, 1, heights = [0.6, 0.4]), size = (900, 700)),
        joinpath(figdir, "fig_ladder_staircase.png"))

# ── Console summary ──────────────────────────────────────────────────────
println("── $(basename(RUN_ROOT)): $(length(groups)) distinct plan(s) over $(length(rungs)) rungs ──")
for (i, g) in enumerate(groups)
    @printf("  plan %d  %-22s  len %8.1f  unc %6.3f  (pct %s)\n", i, lbl(g), g[1].len, g[1].unc,
            join((r.pct for r in g), ","))
end
for r in rungs
    r.ok || @printf("  pct %3d  σ ≤ %6.3f  FAILED (%s)\n", r.pct, r.thr, r.why)
end
println("  → $(figdir)")
