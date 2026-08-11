# Heatmaps of where the random scenario generator puts landmarks and obstacles.
# Includes the REAL generator (src/scenario_generation.jl) rather than
# re-implementing the draw, and self-checks the first 50 samples against the
# scenarios.csv the sweep actually wrote.
#
# Run:  NSAMP=5000 HMOUT=<dir> julia scenario_heatmaps.jl

using Random, Plots, Printf
gr()

const _ROOT = "/home/tonyzhang/projects/landmark_planning"
const NSAMP  = parse(Int, get(ENV, "NSAMP", "5000"))
const OUTDIR = get(ENV, "HMOUT", ".")

# config.jl reads its config dir from ARGS[1]; point it at the sweep's folder.
empty!(ARGS); push!(ARGS, joinpath(_ROOT, "config", "mc"))

include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))

# ── The sweep's own parameter draw (run_constraint_sweep.jl:358) ─────────────
const SWEEP = load_yaml(joinpath(_ROOT, "config", "mc", "sweep.yaml"))
function sample_scenario_params(seed::Int)
    rng = Random.MersenneTwister(seed + 1_000_000)
    dmin = Float64(SWEEP["goal_dist_min"]); dmax = Float64(SWEEP["goal_dist_max"])
    lo, hi = ceil(Int, dmin / HEX_WIDTH_M), floor(Int, dmax / HEX_WIDTH_M)
    D  = HEX_WIDTH_M * rand(rng, lo:hi)
    nl = rand(rng, Int(SWEEP["n_landmarks_min"]):Int(SWEEP["n_landmarks_max"]))
    no = rand(rng, Int(SWEEP["n_obstacles_min"]):Int(SWEEP["n_obstacles_max"]))
    return (D=D, nl=nl, no=no)
end
const SEED_BASE = Int(SWEEP["scenario_seed_base"])

# ── Self-check: reproduce the swept scenarios.csv exactly ────────────────────
function selfcheck(csv::String)
    isfile(csv) || (@warn "no scenarios.csv to check against: $csv"; return)
    bad = 0
    for ln in Iterators.drop(eachline(csv), 1)
        f = split(ln, ',')
        i = parse(Int, f[1][2:end])
        p = sample_scenario_params(SEED_BASE + i)
        got  = (p.D, p.nl, p.no)
        want = (parse(Float64, f[3]), parse(Int, f[4]), parse(Int, f[5]))
        got == want || (bad += 1; @warn "$(f[1]) mismatch got=$got want=$want")
    end
    bad == 0 ? println("  self-check: reproduced scenarios.csv exactly ✓") :
               error("self-check failed on $bad scenario(s) — draw does not match the sweep")
end

# ── Grid ────────────────────────────────────────────────────────────────────
# Absolute metres. The sweep mixes D ∈ {600,700,800}, and every distance that
# matters (visibility_range, comm_range, corridor half-width) is absolute, so
# the population is plotted as-is rather than normalised by D.
const BIN = 20.0
const XLO, XHI = -100.0, 900.0
const YLO, YHI = -420.0,  420.0
const NX = Int((XHI - XLO) / BIN)
const NY = Int((YHI - YLO) / BIN)
const xcent = [XLO + BIN * (i - 0.5) for i in 1:NX]
const ycent = [YLO + BIN * (j - 0.5) for j in 1:NY]

ix(x) = (i = floor(Int, (x - XLO) / BIN) + 1; 1 <= i <= NX ? i : 0)
iy(y) = (j = floor(Int, (y - YLO) / BIN) + 1; 1 <= j <= NY ? j : 0)
clampx(i, d) = clamp(i == 0 ? d : i, 1, NX)
clampy(j, d) = clamp(j == 0 ? d : j, 1, NY)

inside(o::Obstacle, x, y) = all(o.A * [x, y] .<= o.b)

# ── Sample ──────────────────────────────────────────────────────────────────
lm_count  = zeros(Float64, NY, NX)   # landmarks per bin
obs_count = zeros(Float64, NY, NX)   # scenarios in which the bin is covered
n_lm = 0; n_obs = 0; goal_hist = Dict{Float64,Int}()

println("sampling $(NSAMP) scenarios…")
for i in 1:NSAMP
    seed = SEED_BASE + i
    p = sample_scenario_params(seed)
    goal_hist[p.D] = get(goal_hist, p.D, 0) + 1
    sc = generate_scenario(seed=seed, goal_dist=p.D,
                           n_landmarks=p.nl, n_obstacles=p.no)

    for l in sc.landmarks
        a = ix(l.x); b = iy(l.y)
        (a == 0 || b == 0) && continue
        lm_count[b, a] += 1.0
        global n_lm += 1
    end

    # Occupancy, not centroids: a bin counts once per scenario if ANY obstacle
    # covers its centre. That is the quantity a planner actually feels.
    covered = falses(NY, NX)
    for o in sc.obstacles
        global n_obs += 1
        xs = first.(o.verts); ys = last.(o.verts)
        a0 = clampx(ix(minimum(xs)), minimum(xs) < XLO ? 1 : NX)
        a1 = clampx(ix(maximum(xs)), maximum(xs) < XLO ? 1 : NX)
        b0 = clampy(iy(minimum(ys)), minimum(ys) < YLO ? 1 : NY)
        b1 = clampy(iy(maximum(ys)), maximum(ys) < YLO ? 1 : NY)
        for a in a0:a1, b in b0:b1
            covered[b, a] || (inside(o, xcent[a], ycent[b]) && (covered[b, a] = true))
        end
    end
    obs_count .+= covered
    i % 1000 == 0 && println("  $i / $(NSAMP)")
end

lm_density = 100 .* lm_count ./ NSAMP        # landmarks per 100 scenarios per bin
obs_prob   = 100 .* obs_count ./ NSAMP       # % of scenarios with this bin blocked

# ── Palette: single-hue sequential ramps, light → dark ───────────────────────
const SURFACE = "#fcfcfb"
const BLUE_RAMP   = cgrad(["#fcfcfb", "#cde2fb", "#9ec5f4", "#5598e7",
                           "#2a78d6", "#184f95", "#0d366b"])
const ORANGE_RAMP = cgrad(["#fcfcfb", "#fbe0d3", "#f7bfa2", "#f2916a",
                           "#eb6834", "#b64720", "#7d2f13"])

# Guides shared by both panels: the corridor the planner may use, the bands the
# sensor model defines, and the three goal positions the sweep draws.
function overlay!(plt)
    plot!(plt, [XLO, XHI], [0.0, 0.0], color=:black, lw=1.2, alpha=0.45,
          ls=:dash, label="direct start→goal line")
    for (y, lab) in ((100.0, "visibility_range (100 m)"), (-100.0, ""))
        plot!(plt, [XLO, XHI], [y, y], color=:black, lw=1.0, alpha=0.35,
              ls=:dot, label=lab)
    end
    for (y, lab) in ((200.0, "rigid-formation team reach (200 m)"), (-200.0, ""))
        plot!(plt, [XLO, XHI], [y, y], color="#008300", lw=1.5, alpha=0.85, label=lab)
    end
    for (y, lab) in ((260.0, "corridor_halfwidth_m (260 m)"), (-260.0, ""))
        plot!(plt, [XLO, XHI], [y, y], color="#e34948", lw=1.7, label=lab)
    end
    scatter!(plt, [0.0], [0.0], marker=:star5, ms=10, color=:black,
             markerstrokewidth=0, label="start")
    scatter!(plt, [600.0, 700.0, 800.0], [0.0, 0.0, 0.0], marker=:diamond, ms=6,
             color=:black, markerstrokewidth=0, label="goal (D = 600 / 700 / 800)")
end

function panel(z, ramp, ttl, cblabel, fname)
    plt = heatmap(xcent, ycent, z, c=ramp, aspect_ratio=:equal,
                  xlims=(XLO, XHI), ylims=(YLO, YHI),
                  colorbar_title="\n" * cblabel,
                  background_color=SURFACE, background_color_inside=SURFACE,
                  size=(1200, 780), dpi=160, framestyle=:box, grid=false,
                  titlefontsize=11, guidefontsize=10, tickfontsize=9,
                  legendfontsize=8, colorbar_titlefontsize=8,
                  left_margin=6Plots.mm, right_margin=10Plots.mm,
                  bottom_margin=6Plots.mm, top_margin=4Plots.mm)
    overlay!(plt)
    title!(plt, ttl)
    xlabel!(plt, "x (m)"); ylabel!(plt, "y (m)")
    plot!(plt, legend=:outertop, legend_columns=3)
    savefig(plt, joinpath(OUTDIR, fname))
    println("  wrote $(joinpath(OUTDIR, fname))")
end

selfcheck(joinpath(_ROOT, "results", "constraint_sweep",
                   "2026-08-10_18-17-16", "scenarios.csv"))

panel(lm_density, BLUE_RAMP,
      "Landmark placement density — $(NSAMP) sampled scenarios, $(n_lm) landmarks\n" *
      "generate_scenario:  x ~ D·U(0.12, 0.88),   |y| ~ U(200, 320),   sides ALTERNATE along x (opening side = fair coin)",
      "landmarks per 100 scenarios per $(Int(BIN))×$(Int(BIN)) m bin",
      "fig_landmark_density.png")

panel(obs_prob, ORANGE_RAMP,
      "Obstacle occupancy — $(NSAMP) sampled scenarios, $(n_obs) obstacles\n" *
      "random_convex_obstacles:  cx ~ D·U(0.12, 0.88),   cy ~ 240·(U−U) triangular, peaked on the line",
      "% of scenarios in which this bin is blocked",
      "fig_obstacle_density.png")

# ── Numbers worth having beside the pictures ────────────────────────────────
lmband(lo, hi) = sum(lm_count[j, i] for j in 1:NY, i in 1:NX
                     if lo <= abs(ycent[j]) < hi) / max(n_lm, 1)
onlinerows = [j for j in 1:NY if abs(ycent[j]) < 20]
println()
@printf("landmarks with |y| < 100 m  (primary sees them from the line) : %5.2f%%\n", 100lmband(0, 100))
@printf("landmarks with |y| < 200 m  (rigid formation already reaches) : %5.2f%%\n", 100lmband(0, 200))
@printf("landmarks with |y| > 260 m  (outside the hex corridor)        : %5.2f%%\n", 100lmband(260, 1e6))
@printf("peak obstacle occupancy                                       : %5.2f%% of scenarios\n",
        maximum(obs_prob))
@printf("mean obstacle occupancy on the direct line (|y| < 20 m)       : %5.2f%% of scenarios\n",
        sum(obs_prob[onlinerows, :]) / (length(onlinerows) * NX))
println("goal distances drawn: ", sort(collect(goal_hist)))
