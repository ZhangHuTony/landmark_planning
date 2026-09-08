# ==========================================================================
# plot_ladder_grid.jl — Fig. 2: the constraint ladder as a planners × bounds grid
# ==========================================================================
#   julia paper/new_draft/figures/figure2/plot_ladder_grid.jl <sweep_root> [pcts] [out_dir]
#
# <sweep_root> is a run_constraint_sweep.jl output in `scenario_name` mode
# (config/mc/sweep_fig2.yaml → fig2_ladder/<tag>). <pcts> is the comma list of
# ladder levels to show as columns (default 100,70,50,30); rows are the methods
# in the order of the sweep's method folders that hold a trials.csv. <out_dir>
# defaults to <sweep_root>/figures.
#
# Each cell draws the plan that rung SHIPPED (csv/main_ctrls.csv, sampled with
# the planner's own bspline_sample_path) over the scenario's obstacles and
# landmarks. trials.csv decides success (the harness's classify: bound met,
# refinement not recovery_failed, no real obstacle breach): a failed rung keeps
# its attempted path, drawn faded on a grey ground with the failure reason, so
# the figure shows WHAT each baseline tried rather than an empty box. A rung
# with no plan at all (no_solution) is grey and empty.
#
# Config comes from a rung's `_cfg` snapshot, so the hex lattice (100 m here,
# via the sweep's `overrides`), the preset geometry and the sensor model are
# exactly what ran. Only viz.jl helpers draw the scenery; nothing in src/ moves.
# Output: fig2_ladder_grid.{png,svg,pdf,eps} (EPS via Ghostscript, as fig1).
# ==========================================================================

ENV["GKSwstype"] = "100"
using Printf

const _ROOT = normpath(joinpath(@__DIR__, "..", "..", "..", ".."))
length(ARGS) >= 1 || error("usage: julia plot_ladder_grid.jl <sweep_root> [pcts] [out_dir]")
abspath_or(p) = isabspath(p) ? p : joinpath(_ROOT, p)
const RUN_ROOT = abspath_or(ARGS[1])
const PCTS     = length(ARGS) >= 2 ? parse.(Int, split(ARGS[2], ',')) : [100, 70, 50, 30]
const OUT_DIR  = length(ARGS) >= 3 ? abspath_or(ARGS[3]) : joinpath(RUN_ROOT, "figures")
isdir(RUN_ROOT) || error("no such sweep root: $(RUN_ROOT)")

# Method folders, in the order the sweep's methods.yaml listed them if that order
# is recoverable from all_trials.csv; otherwise alphabetical.
const METHOD_DIRS = filter(d -> isfile(joinpath(RUN_ROOT, d, "trials.csv")),
                           filter(d -> isdir(joinpath(RUN_ROOT, d)), readdir(RUN_ROOT)))
isempty(METHOD_DIRS) && error("no method folder with a trials.csv under $(RUN_ROOT)")
const PREFERRED = ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]
const METHODS = vcat(filter(m -> m in METHOD_DIRS, PREFERRED), sort(filter(m -> !(m in PREFERRED), METHOD_DIRS)))
const LABEL = Dict("hexspline_cl" => "ours", "greedy" => "greedy", "formation" => "formation",
                   "sequential" => "sequential", "clgbt" => "CL-GBT")

rung_dir(m, pct) = joinpath(RUN_ROOT, m, "s001_p$(lpad(pct, 3, '0'))")
const CFG_DIR = let c = filter(isdir, [joinpath(rung_dir(m, p), "_cfg") for m in METHODS for p in PCTS])
    isempty(c) && error("no _cfg snapshot under any requested rung")
    first(c)
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
function read_table(path::String)
    isfile(path) || error("missing $(path)")
    lines = filter(!isempty, strip.(readlines(path)))
    hdr = String.(split(lines[1], ','))
    [Dict(zip(hdr, String.(split(l, ',')))) for l in lines[2:end]]
end
num(r, k) = (v = get(r, k, ""); isempty(v) || v == "null" ? nothing : tryparse(Float64, v))

struct Cell
    ok::Bool; why::String; thr::Float64
    len::Union{Nothing,Float64}; unc::Union{Nothing,Float64}
    ctrls::Union{Nothing,Vector{Vector{Tuple{Float64,Float64}}}}
end
cells = Dict{Tuple{String,Int},Cell}()
for m in METHODS, r in read_table(joinpath(RUN_ROOT, m, "trials.csv"))
    pct = parse(Int, r["pct"]); pct in PCTS || continue
    csv = joinpath(rung_dir(m, pct), "csv", "main_ctrls.csv")
    cells[(m, pct)] = Cell(r["success"] == "true", get(r, "fail_reason", ""),
                          something(num(r, "threshold"), NaN), num(r, "primary_length"),
                          num(r, "primary_unc"), isfile(csv) ? read_ctrls_csv(csv) : nothing)
end
isempty(cells) && error("trials.csv rows cover none of the requested levels $(PCTS)")

# ── Scenery ──────────────────────────────────────────────────────────────
const SCEN = ACTIVE_SCENARIO
const GRAPH = build_hex_graph(SCEN.landmarks, SCEN.start, SCEN.goal; hex_r = HEX_RADIUS_M)
sample(ctrl) = first(bspline_sample_path(ctrl; length_samples_per_seg = 12))

const XL = (-60.0, SCEN.goal[1] + 60.0)
const YL = (-CORRIDOR_Y_MAX_M - 45.0, CORRIDOR_Y_MAX_M + 45.0)

function scenery(; ground = :white)
    p = plot(legend = false, aspect_ratio = :equal, xlims = XL, ylims = YL,
             background_color_inside = ground, framestyle = :box, grid = false,
             xticks = 0:500:SCEN.goal[1], yticks = -200:200:200, tickfontsize = 6)
    overlay_obstacles!(p)
    for lm in SCEN.landmarks
        draw_covariance_ellipse!(p, lm.x, lm.y, lm.cov, color = :red, alpha = 0.18, display_scale = 400.0)
    end
    scatter!(p, [lm.x for lm in SCEN.landmarks], [lm.y for lm in SCEN.landmarks],
             color = :black, markersize = 3, markerstrokewidth = 0)
    scatter!(p, [SCEN.start[1]], [SCEN.start[2]], color = :green, markersize = 5, markerstrokewidth = 0)
    scatter!(p, [SCEN.goal[1]], [SCEN.goal[2]], color = :orange, marker = :star5, markersize = 8, markerstrokewidth = 0)
    return p
end

agent_color(a, na) = a == na ? :blue : get(agent_colors, a, :gray)
function draw_paths!(p, ctrls; alpha = 1.0)
    na = length(ctrls)
    for (a, ctrl) in enumerate(ctrls)
        length(ctrl) < 2 && continue
        pts = sample(ctrl)
        primary = a == na
        plot!(p, [q[1] for q in pts], [q[2] + (primary ? 0.0 : SUPPORT_PLOT_OFFSET_M * a) for q in pts];
              color = agent_color(a, na), linewidth = primary ? 2.0 : 1.2,
              linestyle = primary ? :solid : :dash, alpha = alpha)
    end
end

# ── Grid ─────────────────────────────────────────────────────────────────
panels = Plots.Plot[]
for (i, m) in enumerate(METHODS), (j, pct) in enumerate(PCTS)
    c = get(cells, (m, pct), nothing)
    p = scenery(ground = (c === nothing || !c.ok) ? RGB(0.90, 0.90, 0.90) : :white)
    if c === nothing
        annotate!(p, (XL[1] + XL[2]) / 2, 0.0, text("not run", 8, :gray40))
    else
        c.ctrls === nothing || draw_paths!(p, c.ctrls; alpha = c.ok ? 1.0 : 0.4)
    end
    # Status goes in the panel TITLE (the field fills the whole frame, so any in-plot
    # label sits on an obstacle); the first row prepends the column header.
    status = c === nothing ? "not run" :
             c.ok && c.len !== nothing && c.unc !== nothing ? @sprintf("%.0f m,  σ = %.2f m", c.len, c.unc) :
             c.ok ? "ok" : "✗ " * replace(c.why, "_" => " ")
    hdr = i == 1 ? @sprintf("%d %%  (σ ≤ %.2f m)\n", pct, c === nothing ? NaN : c.thr) : ""
    plot!(p, title = hdr * status, titlefontsize = i == 1 ? 8 : 7,
          titlefontcolor = (c !== nothing && !c.ok) ? :firebrick : :black)
    j == 1 && plot!(p, ylabel = get(LABEL, m, m), guidefontsize = 9)
    push!(panels, p)
end
nr, nc = length(METHODS), length(PCTS)
fig = plot(panels..., layout = (nr, nc), size = (nc * 400, nr * 190 + 40),
           left_margin = 2Plots.mm, bottom_margin = 1Plots.mm, top_margin = 1Plots.mm)

function save_all(p, stem::String)
    for ext in ("png", "svg", "pdf"); savefig(p, "$(stem).$(ext)"); end
    gs = Sys.which("gs")
    gs === nothing ? (@warn "ghostscript (gs) not found — no EPS written for $(stem)") :
        run(`$(gs) -q -dNOPAUSE -dBATCH -sDEVICE=eps2write -o $(stem).eps $(stem).pdf`)
    println("  → $(stem).{png,svg,pdf" * (gs === nothing ? "" : ",eps") * "}")
end
mkpath(OUT_DIR)
save_all(fig, joinpath(OUT_DIR, "fig2_ladder_grid"))

# ── Console summary ──────────────────────────────────────────────────────
println("\n── $(basename(RUN_ROOT)): $(nr) planners × $(nc) levels ──")
@printf("%-13s", "")
for pct in PCTS; @printf(" %14s", "$(pct)%"); end; println()
for m in METHODS
    @printf("%-13s", get(LABEL, m, m))
    for pct in PCTS
        c = get(cells, (m, pct), nothing)
        s = c === nothing ? "—" : c.ok ? @sprintf("%.0f/%.2f", c.len, c.unc) : "✗ " * c.why
        @printf(" %14s", s)
    end
    println()
end
