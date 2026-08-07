# ==========================================================================
# mc_nees.jl — Phase 3a: Monte Carlo filter-consistency (NEES) test
# ==========================================================================
# Replays a planner's saved path under sampled process/measurement/comm noise,
# with the agents glued to the nominal plan ("Option B": x_true = x_plan), and
# reports averaged NEES for the primary agent. Purpose: verify the covariance/
# fusion math in src/covariance.jl is self-consistent — i.e. inter-agent comm
# fusion is NOT double-counting shared information (not overconfident).
#
#   A consistent filter → mean NEES ≈ state dim = 2; a double-counting one
#   gives NEES > 2 once the same pair fuses repeatedly.
#
# Usage:
#   julia mc_nees.jl <path/to/<planner>/csv> [--trials M] [--seed S]
#
# All covariance ALGEBRA is reused verbatim from src/covariance.jl (READ-ONLY):
# growth_covariance, accumulate_landmark_info, kalman_info_update, inv2,
# ci_comm, ci_omega_det, comm_weight. The net-new logic here is the noise
# SAMPLING and the info-form MEAN updates that the analytic planner omits.
#
# Measurement model (truth/sampling side):
#   - process:  x̂ += Δ + η,  η ~ N(0, Q=growth_covariance)          [truth frozen to plan]
#   - landmark: aggregate info-noise n ~ N(0, I_tot),  I_tot from accumulate_landmark_info
#               at the plan position; x̂⁺ = Σ⁺(Σ⁻⁻¹x̂ + I_tot·x_true + n)
#   - comm:     RELATIVE, correlated — z_self = x̂_other + (x_true_self − x_true_other) + v,
#               v ~ N(0, COMM_SENSOR_NOISE²·I).  z carries the OTHER agent's estimate
#               error, so repeated fusion of the same pair shares history -- the thing a
#               naive information add double-counts.  The 1/w taper inflates only the
#               filter's assumed cov (as ci_comm does), never the truth-side draw v.
# Each choice above was derived so that Cov(e[k]) = Σ[k] for a consistent update.
# ==========================================================================

using LinearAlgebra
using Random
using Printf
using Plots

const N_SCATTER = 600   # per-waypoint estimate-error samples kept for the consistency plot

# ---- CLI (parsed before config.jl, which consumes ARGS[1] as the config dir) ----
length(ARGS) >= 1 || error("usage: julia mc_nees.jl <path/to/<planner>/csv> [--trials M] [--seed S]")

function _flag(name::String, default)
    i = findfirst(==(name), ARGS)
    (i === nothing || i == length(ARGS)) && return default
    return ARGS[i+1]
end

const CSV_DIR = abspath(ARGS[1])
isdir(CSV_DIR) || error("not a directory: $CSV_DIR")
const M_TRIALS  = parse(Int, _flag("--trials", "2000"))
const BASE_SEED = parse(Int, _flag("--seed", "1"))
const PLOTS     = findfirst(==("--plots"), ARGS) !== nothing

# Resolve  <run>/<planner>/csv  ->  <run>/config  and  <run>/scenario_landmarks.csv
const PLANNER_DIR = dirname(CSV_DIR)
const RUN_DIR     = dirname(PLANNER_DIR)
const CONFIG_DIR  = joinpath(RUN_DIR, "config")
isfile(joinpath(CONFIG_DIR, "main.yaml")) || error("no main.yaml under $CONFIG_DIR (expected <run>/config/)")

# config.jl reads ARGS[1] as its config dir at include-time — point it at the run's config.
empty!(ARGS); push!(ARGS, CONFIG_DIR)

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))
include(joinpath(_ROOT, "src", "viz.jl"))

# ponytail: mc only supports the polyline-of-control-points evaluation the planner
# uses under cont_unc_use_waypoints=true. If false, the planner scored the sampled
# spline instead and the MC path would have to be re-sampled via bspline_sample_path.
if haskey(CFG, "cont_unc_use_waypoints") && CFG["cont_unc_use_waypoints"] == false
    error("cont_unc_use_waypoints=false: planner scored the sampled spline; mc_nees only supports the control-point polyline. Add spline sampling to enable.")
end

# --------------------------------------------------------------------------
# Gaussian sampling: x ~ N(0, cov) for a symmetric PSD 2×2. Degenerate/zero
# covariance (e.g. Q at seg→0) yields a zero draw rather than a failed Cholesky.
# --------------------------------------------------------------------------
function sample_gauss(cov::AbstractMatrix, rng::AbstractRNG)
    A = (Matrix(cov) .+ Matrix(cov)') ./ 2
    (A[1,1] <= 1e-18 && A[2,2] <= 1e-18) && return [0.0, 0.0]
    F = cholesky(Symmetric(A), check=false)
    issuccess(F) || return [0.0, 0.0]   # rank-deficient — no noise
    return F.L * randn(rng, 2)
end

function cum_arc(pos::Vector{Tuple{Float64,Float64}})
    isempty(pos) && return [0.0]
    arcs = Vector{Float64}(undef, length(pos)); arcs[1] = 0.0
    for i in 2:length(pos)
        arcs[i] = arcs[i-1] + hypot(pos[i][1]-pos[i-1][1], pos[i][2]-pos[i-1][2])
    end
    return arcs
end

function nearest_index(arcs::Vector{Float64}, t::Float64)
    nidx = 1; best = abs(arcs[1] - t)
    for i in 2:length(arcs)
        d = abs(arcs[i] - t)
        d < best && (best = d; nidx = i)
    end
    return nidx
end

# --------------------------------------------------------------------------
# Propagate covariance + estimate mean over waypoints (from_idx, to_idx],
# mirroring covariance.jl:propagate_segment! exactly for the covariance, and
# threading the matching info-form mean update with sampled noise.
# --------------------------------------------------------------------------
function propagate_segment_mc!(covs, means, pos, lms, from_idx, to_idx, cov, mean, rng)
    for i in (from_idx+1):to_idx
        xp, yp = pos[i-1]; xc, yc = pos[i]
        dx = xc - xp; dy = yc - yp
        seg = hypot(dx, dy); heading = atan(dy, dx)
        Q = growth_covariance(seg, heading)
        cov = cov + Q                                   # Σ⁻  (identical to library)
        η = seg < 1e-10 ? [0.0, 0.0] : sample_gauss(Q, rng)
        mean = mean .+ [dx, dy] .+ η                    # x̂ dead-reckoning
        I11, I12, I22 = accumulate_landmark_info(xc, yc, lms)   # at truth = plan position
        if I11 > 0.0 || I22 > 0.0
            updated = kalman_info_update(cov, I11, I12, I22)    # Σ⁺
            if updated !== nothing
                Itot  = [I11 I12; I12 I22]
                nmeas = sample_gauss(Itot, rng)                 # ~ N(0, I_tot)
                ξ     = inv2(cov) * mean .+ Itot * [xc, yc] .+ nmeas
                mean  = updated * ξ
                cov   = updated
            end
        end
        covs[i]  = copy(cov)
        means[i] = copy(mean)
    end
    return cov, mean
end

# --------------------------------------------------------------------------
# Bidirectional inter-agent comm fusion at a checkpoint. Covariance ops are
# ci_comm verbatim from covariance.jl; the mean uses the matching info weights
# on the relative-correlated pseudo-msmt.
# xt* = truth = plan position. Returns (Σs⁺, x̂s⁺, Σr⁺, x̂r⁺).
# --------------------------------------------------------------------------
function comm_fuse(Σs, x̂s, xts, Σr, x̂r, xtr, w, rng)
    R = COMM_SENSOR_NOISE^2 .* [1.0 0.0; 0.0 1.0]
    # Pseudo-measurements: z_self = x̂_other + (x_true_self − x_true_other) + v
    z_s = x̂r .+ (xts .- xtr) .+ sample_gauss(R, rng)   # s's msmt, from r's estimate
    z_r = x̂s .+ (xtr .- xts) .+ sample_gauss(R, rng)   # r's msmt, from s's estimate
    Σs⁺, Σr⁺ = ci_comm(Σs, Σr, w)                  # covariance verbatim from library
    Ia = inv2(Σs); Itb = inv2((Σr .+ R) ./ w)      # s ← r : ω and info weights
    ωa = ci_omega_det(Ia, Itb)
    Ib = inv2(Σr); Ita = inv2((Σs .+ R) ./ w)      # r ← s
    ωb = ci_omega_det(Ib, Ita)
    x̂s⁺ = Σs⁺ * (ωa .* (Ia * x̂s) .+ (1.0-ωa) .* (Itb * z_s))
    x̂r⁺ = Σr⁺ * (ωb .* (Ib * x̂r) .+ (1.0-ωb) .* (Ita * z_r))
    return Σs⁺, x̂s⁺, Σr⁺, x̂r⁺
end

# --------------------------------------------------------------------------
# One Monte Carlo trial. Walks the same checkpoint structure as
# apply_synchronized_propagation!, threading covariance (deterministic) and
# the noisy estimate mean. Returns per-agent (covs, means); truth = positions.
# --------------------------------------------------------------------------
function run_trial(positions, lms, na, rng)
    lens = [length(positions[a]) for a in 1:na]
    arcs = [cum_arc(positions[a]) for a in 1:na]
    Σ0   = copy(lms[1].cov)

    covs  = [Vector{Matrix{Float64}}(undef, lens[a]) for a in 1:na]
    means = [Vector{Vector{Float64}}(undef, lens[a]) for a in 1:na]
    for a in 1:na
        covs[a][1] = copy(Σ0)
        e0 = sample_gauss(Σ0, rng)                      # x̂[1] = x_true[1] − e0  ⇒ e[1] ~ N(0,Σ0)
        means[a][1] = [positions[a][1][1], positions[a][1][2]] .- e0
    end

    cur_idx  = ones(Int, na)
    cur_cov  = [copy(Σ0) for _ in 1:na]
    cur_mean = [copy(means[a][1]) for a in 1:na]

    max_arc = maximum(arcs[a][end] for a in 1:na)
    for comm_time in 0.0:COMM_INTERVAL:max_arc
        agent_indices = Vector{Int}(undef, na)
        for a in 1:na
            nidx = nearest_index(arcs[a], comm_time)
            if nidx > cur_idx[a]
                cur_cov[a], cur_mean[a] = propagate_segment_mc!(
                    covs[a], means[a], positions[a], lms, cur_idx[a], nidx, cur_cov[a], cur_mean[a], rng)
                cur_idx[a] = nidx
            end
            agent_indices[a] = nidx
        end
        for s in 1:na, r in (s+1):na
            is = agent_indices[s]; ir = agent_indices[r]
            ps = positions[s][is]; pr = positions[r][ir]
            w = comm_weight((ps[1]-pr[1])^2 + (ps[2]-pr[2])^2)
            w > COMM_WEIGHT_MIN || continue
            Σs⁺, x̂s⁺, Σr⁺, x̂r⁺ = comm_fuse(
                covs[s][is], means[s][is], [ps[1], ps[2]],
                covs[r][ir], means[r][ir], [pr[1], pr[2]], w, rng)
            covs[s][is] = Σs⁺; means[s][is] = x̂s⁺
            covs[r][ir] = Σr⁺; means[r][ir] = x̂r⁺
        end
        for a in 1:na
            cur_cov[a]  = covs[a][cur_idx[a]]
            cur_mean[a] = means[a][cur_idx[a]]
        end
    end
    for a in 1:na
        if cur_idx[a] < lens[a]
            propagate_segment_mc!(covs[a], means[a], positions[a], lms,
                                  cur_idx[a], lens[a], cur_cov[a], cur_mean[a], rng)
        end
    end
    return covs, means
end

# --------------------------------------------------------------------------
# Aggregate M trials into per-waypoint NEES for the primary agent.
# --------------------------------------------------------------------------
function run_nees(positions, lms, na, M, base_seed)
    primary = na
    np = length(positions[primary])
    Σ_track = Vector{Matrix{Float64}}(undef, np)   # deterministic across trials
    nees_sum = zeros(np)
    n_keep = min(M, N_SCATTER)
    errs = [Vector{Tuple{Float64,Float64}}() for _ in 1:np]   # primary estimate errors (subsample)

    for m in 1:M
        rng = MersenneTwister(base_seed + m)       # per-trial seed ⇒ common random numbers across runs
        covs, means = run_trial(positions, lms, na, rng)
        m == 1 && (Σ_track = covs[primary])
        for k in 1:np
            e = [positions[primary][k][1], positions[primary][k][2]] .- means[primary][k]
            nees_sum[k] += dot(e, inv2(Σ_track[k]) * e)
            m <= n_keep && push!(errs[k], (e[1], e[2]))
        end
    end
    return nees_sum ./ M, Σ_track, errs
end

# --------------------------------------------------------------------------
# Hermetic self-check: single agent, one landmark, no comms. Process + landmark
# consistency alone must give mean NEES ≈ 2. Fails loudly if the harness math
# is broken, independent of any results folder.
# --------------------------------------------------------------------------
function selfcheck()
    lms = [Landmark(50.0, 25.0, [1.0 0.0; 0.0 1.0])]
    positions = [[(float(x), 0.0) for x in 0.0:5.0:120.0]]
    nees, _, _ = run_nees(positions, lms, 1, 3000, 90000)
    mean_nees = sum(nees) / length(nees)
    @assert abs(mean_nees - 2.0) < 0.25 "self-check FAILED: process+landmark mean NEES = $(round(mean_nees,digits=3)) (expected ≈ 2)"
    println("  self-check OK: process+landmark mean NEES = $(round(mean_nees, digits=3)) ≈ 2")
end

# --------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------
function read_nees_csv(file)
    steps = Int[]; vals = Float64[]; lo = 0.0; hi = 0.0
    open(file) do io
        readline(io)
        for line in eachline(io)
            isempty(strip(line)) && continue
            p = split(line, ',')
            push!(steps, parse(Int, p[1])); push!(vals, parse(Float64, p[2]))
            lo = parse(Float64, p[3]); hi = parse(Float64, p[4])
        end
    end
    return steps, vals, lo, hi
end

# NEES vs step.
function plot_nees_curves(csv_dir, out_png)
    plt = plot(xlabel="waypoint (step)", ylabel="mean NEES (primary)",
               title="Filter consistency — NEES vs step", size=(820, 460), legend=:topright)
    lo = hi = nothing
    let f = joinpath(csv_dir, "nees.csv")
        if isfile(f)
            steps, vals, lo, hi = read_nees_csv(f)
            plot!(plt, steps, vals, marker=:circle, ms=4, lw=2, color=:seagreen, label="ci")
        end
    end
    if lo !== nothing
        hspan!(plt, [lo, hi], color=:green, alpha=0.12, label="consistent band")
        hline!(plt, [2.0], color=:black, ls=:dash, lw=1, label="expected n=2")
    end
    savefig(plt, out_png)
    println("  → $(out_png)")
end

function ellipse_xy(cov, nstd; npts=80)
    vals, vecs = eigen(Symmetric(cov))
    a = nstd * sqrt(max(vals[1], 0.0)); b = nstd * sqrt(max(vals[2], 0.0))
    ang = atan(vecs[2,1], vecs[1,1]); θ = range(0, 2π, length=npts)
    R = [cos(ang) -sin(ang); sin(ang) cos(ang)]
    pts = R * vcat((a .* cos.(θ))', (b .* sin.(θ))')
    return pts[1, :], pts[2, :]
end

# World-space consistency plot: at every true waypoint, filled 1σ/2σ/3σ bands
# (the filter's claimed spread, from Σ[k]) drawn concentrically in distinct
# colors, with each run's primary position estimate (est = truth − e) scattered
# over them and the planned path overlaid. A consistent filter keeps estimates
# within the bands in the expected proportions (~39%/86%/99% inside 1/2/3σ);
# an overconfident filter spills its estimates past the bands.
function plot_consistency(errs, Σ_track, positions_primary, out_png)
    np = length(Σ_track)
    px = [p[1] for p in positions_primary]; py = [p[2] for p in positions_primary]
    plt = plot(size=(2000, 620), legend=:topright, aspect_ratio=:equal,
               xlabel="x (m)", ylabel="y (m)",
               title="1σ/2σ/3σ bands (filter Σ) at true waypoints vs run estimates")
    plot!(plt, px, py, color=:black, lw=1.5, alpha=0.6, label="planned path")

    # Draw outer→inner so inner bands sit on top; distinct color per band.
    bands = [(3.0, :crimson, 0.13), (2.0, :gold, 0.22), (1.0, :seagreen, 0.38)]
    for (ns, clr, a) in bands
        for k in 1:np
            ex, ey = ellipse_xy(Σ_track[k], ns)
            plot!(plt, px[k] .+ ex, py[k] .+ ey, seriestype=:shape,
                  fillcolor=clr, fillalpha=a, linealpha=0.0,
                  label=(k == 1 ? "$(Int(ns))σ" : false))
        end
    end

    # Run estimates (est = truth − e), all waypoints pooled.
    sx = Float64[]; sy = Float64[]
    for k in 1:np, e in errs[k]
        push!(sx, px[k] - e[1]); push!(sy, py[k] - e[2])
    end
    scatter!(plt, sx, sy, ms=1.2, msw=0, color=:navy, alpha=0.28, label="run estimates")
    scatter!(plt, px, py, marker=:star5, ms=5, color=:black, label="true waypoints")
    savefig(plt, out_png)
    println("  → $(out_png)")
end

# ==========================================================================
# Main
# ==========================================================================
println("mc_nees — filter-consistency (NEES) test")
selfcheck()

# Landmarks: read the serialized field (pins Σ₀ = lms[1].cov and every fusion noise).
const LM_CSV = joinpath(RUN_DIR, "scenario_landmarks.csv")
isfile(LM_CSV) || error("scenario_landmarks.csv not found at $LM_CSV — re-run generate_plan.jl to serialize it")
landmarks = read_landmarks_csv(LM_CSV)

# Path: control points, one polyline per agent (primary = last).
const CTRLS_CSV = joinpath(CSV_DIR, "main_ctrls.csv")
isfile(CTRLS_CSV) || error("main_ctrls.csv not found at $CTRLS_CSV")
ctrls = read_ctrls_csv(CTRLS_CSV)
positions = [Tuple{Float64,Float64}[(x, y) for (x, y) in a] for a in ctrls]
const NA = length(positions)
const PRIMARY = NA

println("  run: $(RUN_DIR)")
println("  landmarks: $(length(landmarks))   agents: $(NA) (primary = $(PRIMARY))   trials: $(M_TRIALS)")

# Fidelity check: our deterministic covariance track must match the library's
# evaluate_joint_discrete waypoint-for-waypoint.
let
    lib_covs, _, _, _ = evaluate_joint_discrete(positions, landmarks, NA)
    _, Σ_track, _ = run_nees(positions, landmarks, NA, 1, 0)
    maxdiff = maximum(abs(unc_radius(Σ_track[k]) - unc_radius(lib_covs[PRIMARY][k])) for k in 1:length(positions[PRIMARY]))
    @assert maxdiff < 1e-9 "covariance track diverges from library by $(maxdiff) — mc loop does not mirror apply_synchronized_propagation!"
    println("  covariance fidelity OK: matches evaluate_joint_discrete (max Δ = $(maxdiff))")
    println("  terminal primary uncertainty det(Σ)^0.25 = $(round(unc_radius(lib_covs[PRIMARY][end]), digits=4))  (cf. results.yaml continuous_uncertainties)")
end

# Run the Monte Carlo.
nees, Σ_track, errs = run_nees(positions, landmarks, NA, M_TRIALS, BASE_SEED)

const N_DIM = 2
const HALFWIDTH = 1.96 * sqrt(2.0 * N_DIM / M_TRIALS)   # normal approx to the χ²/M band (no Distributions.jl)
const LO = N_DIM - HALFWIDTH
const HI = N_DIM + HALFWIDTH
classify(x) = x > HI ? "overconfident" : x < LO ? "pessimistic" : "ok"

# Write nees.csv into the same csv/ folder.
const OUT_CSV = joinpath(CSV_DIR, "nees.csv")
open(OUT_CSV, "w") do io
    println(io, "step,mean_nees,lower,upper,expected,flag")
    for k in 1:length(nees)
        @printf(io, "%d,%.6f,%.6f,%.6f,%d,%s\n", k, nees[k], LO, HI, N_DIM, classify(nees[k]))
    end
end

overall = sum(nees) / length(nees)
terminal = nees[end]
n_over = count(x -> x > HI, nees)
n_under = count(x -> x < LO, nees)

open(joinpath(CSV_DIR, "nees_meta.yaml"), "w") do io
    println(io, "fusion: ci")
    println(io, "trials: $(M_TRIALS)")
    println(io, "base_seed: $(BASE_SEED)")
    println(io, "state_dim: $(N_DIM)")
    println(io, "nees_band: [$(round(LO,digits=4)), $(round(HI,digits=4))]")
    println(io, "scenario: $(LANDMARK_SCENARIO)")
    println(io, "num_agents: $(NA)")
    println(io, "primary_index: $(PRIMARY)")
    println(io, "num_landmarks: $(length(landmarks))")
    println(io, "sigma0_source: lms[1].cov (first landmark, from scenario_landmarks.csv)")
    println(io, "source_csv: $(CTRLS_CSV)")
    println(io, "landmark_csv: $(LM_CSV)")
    println(io, "mean_nees: $(round(overall,digits=4))")
    println(io, "terminal_nees: $(round(terminal,digits=4))")
    println(io, "waypoints_overconfident: $(n_over)")
    println(io, "waypoints_pessimistic: $(n_under)")
    println(io, "comm_measurement_model: relative-correlated (z_self = x_hat_other + rel_truth + v, v~N(0,COMM_SENSOR_NOISE^2 I); 1/w taper filter-side only)")
end

println("  NEES band (consistent): [$(round(LO,digits=3)), $(round(HI,digits=3))]")
println("  mean NEES over waypoints: $(round(overall,digits=3))   terminal: $(round(terminal,digits=3))")
println("  waypoints overconfident (>band): $(n_over) / $(length(nees))")
println("  → $(OUT_CSV)")

if PLOTS
    plot_nees_curves(CSV_DIR, joinpath(CSV_DIR, "nees_curves.png"))
    plot_consistency(errs, Σ_track, positions[PRIMARY], joinpath(CSV_DIR, "consistency.png"))
end
