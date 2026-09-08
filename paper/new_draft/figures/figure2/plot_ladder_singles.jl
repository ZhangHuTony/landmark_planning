# ==========================================================================
# plot_ladder_singles.jl — one figure per rung of ONE planner (default hexspline_cl)
# ==========================================================================
#   julia paper/new_draft/figures/figure2/plot_ladder_singles.jl <sweep_root> [pcts] [method] [out_dir]
#
# Same scenery and path styling as plot_ladder_grid.jl, one rung per file:
#   <out_dir>/fig2_<method>_p<pct>.{png,svg,pdf,eps}
# <pcts> default 100,70,50,30; <method> default hexspline_cl; <out_dir> default
# <sweep_root>/figures/singles. Title carries the level, the bound, the shipped
# length and σ. Failed rungs are still drawn (faded, grey ground) so nothing is
# silently skipped.
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", "..", "..", ".."))
length(ARGS) >= 1 || error("usage: julia plot_ladder_singles.jl <sweep_root> [pcts] [method] [out_dir]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_ROOT = abspath_or(ARGS[1])
const PCTS     = length(ARGS) >= 2 ? parse.(Int, split(ARGS[2], ',')) : [100, 70, 50, 30]
const METHOD   = length(ARGS) >= 3 ? ARGS[3] : "hexspline_cl"
const OUT_DIR  = length(ARGS) >= 4 ? abspath_or(ARGS[4]) : joinpath(RUN_ROOT, "figures", "singles")
isdir(joinpath(RUN_ROOT, METHOD)) || error("no method folder $(METHOD) under $(RUN_ROOT)")

rung_dir(pct) = joinpath(RUN_ROOT, METHOD, "s001_p$(lpad(pct, 3, '0'))")
const CFG_DIR = let c = filter(isdir, [joinpath(rung_dir(p), "_cfg") for p in PCTS])
    isempty(c) && error("no _cfg snapshot under any requested rung"); first(c)
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

function read_table(path::String)
    isfile(path) || error("missing $(path)")
    lines = filter(!isempty, strip.(readlines(path)))
    hdr = String.(split(lines[1], ','))
    [Dict(zip(hdr, String.(split(l, ',')))) for l in lines[2:end]]
end
num(r, k) = (v = get(r, k, ""); isempty(v) || v == "null" ? nothing : tryparse(Float64, v))
const TRIALS = Dict(parse(Int, r["pct"]) => r for r in read_table(joinpath(RUN_ROOT, METHOD, "trials.csv")))

const SCEN = ACTIVE_SCENARIO
const XL = (-60.0, SCEN.goal[1] + 60.0)
const YL = (-CORRIDOR_Y_MAX_M - 45.0, CORRIDOR_Y_MAX_M + 45.0)
sample(ctrl) = first(bspline_sample_path(ctrl; length_samples_per_seg = 12))
agent_color(a, na) = a == na ? :blue : get(agent_colors, a, :gray)

function scenery(; ground = :white)
    p = plot(legend = :outerright, aspect_ratio = :equal, xlims = XL, ylims = YL,
             background_color_inside = ground, framestyle = :box, grid = false,
             xlabel = "x (m)", ylabel = "y (m)", tickfontsize = 7, guidefontsize = 8)
    overlay_obstacles!(p)
    for lm in SCEN.landmarks
        draw_covariance_ellipse!(p, lm.x, lm.y, lm.cov, color = :red, alpha = 0.18, display_scale = 400.0)
    end
    scatter!(p, [lm.x for lm in SCEN.landmarks], [lm.y for lm in SCEN.landmarks],
             color = :black, markersize = 3, markerstrokewidth = 0, label = "Landmarks")
    scatter!(p, [SCEN.start[1]], [SCEN.start[2]], color = :green, markersize = 6, markerstrokewidth = 0, label = "Start")
    scatter!(p, [SCEN.goal[1]], [SCEN.goal[2]], color = :orange, marker = :star5, markersize = 9, markerstrokewidth = 0, label = "Goal")
    return p
end

function save_all(p, stem::String)
    for ext in ("png", "svg", "pdf"); savefig(p, "$(stem).$(ext)"); end
    gs = Sys.which("gs")
    gs === nothing ? (@warn "ghostscript (gs) not found — no EPS written for $(stem)") :
        run(`$(gs) -q -dNOPAUSE -dBATCH -sDEVICE=eps2write -o $(stem).eps $(stem).pdf`)
    println("  → $(stem).{png,svg,pdf" * (gs === nothing ? "" : ",eps") * "}")
end

mkpath(OUT_DIR)
for pct in PCTS
    r = get(TRIALS, pct, nothing)
    r === nothing && (println("  (no trials row for $(pct)% — skipped)"); continue)
    ok  = r["success"] == "true"
    csv = joinpath(rung_dir(pct), "csv", "main_ctrls.csv")
    p = scenery(ground = ok ? :white : RGB(0.90, 0.90, 0.90))
    if isfile(csv)
        ctrls = read_ctrls_csv(csv); na = length(ctrls)
        for (a, ctrl) in enumerate(ctrls)
            length(ctrl) < 2 && continue
            pts = sample(ctrl); primary = a == na
            plot!(p, [q[1] for q in pts], [q[2] + (primary ? 0.0 : SUPPORT_PLOT_OFFSET_M * a) for q in pts];
                  color = agent_color(a, na), linewidth = primary ? 2.2 : 1.4,
                  linestyle = primary ? :solid : :dash, alpha = ok ? 1.0 : 0.4,
                  label = primary ? "primary" : "support $(a)")
        end
    end
    thr = something(num(r, "threshold"), NaN)
    status = ok ? @sprintf("%.0f m,  σ = %.2f m", something(num(r, "primary_length"), NaN), something(num(r, "primary_unc"), NaN)) :
                  "✗ " * replace(get(r, "fail_reason", ""), "_" => " ")
    plot!(p, title = @sprintf("%d %%  (σ ≤ %.2f m)   %s", pct, thr, status), titlefontsize = 9,
          size = (1000, 420), left_margin = 3Plots.mm, bottom_margin = 3Plots.mm)
    save_all(p, joinpath(OUT_DIR, @sprintf("fig2_%s_p%03d", METHOD, pct)))
end
