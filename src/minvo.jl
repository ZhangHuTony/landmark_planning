# ==========================================================================
# MINVO-enclosure chance-constrained static-obstacle avoidance — CONTINUOUS
# (B-spline refinement) stage.
# ==========================================================================
# Extends the discrete-stage feasibility filter (src/obstacles.jl) to the
# clamped cubic B-spline. PURE FEASIBILITY: obstacles never enter the
# measurement model and never modify covariance propagation. This file only
# READS the Σ that evaluate_joint_discrete already carries.
#
# Idea. A cubic B-spline segment j lies inside the convex hull of its MINVO
# control points  Q_mv = Q_bs · M_seg  (Q_bs = the segment's 4 B-spline control
# points as columns). Each MINVO vertex is LINEAR in the control points, so
# "vertex on the safe side of an inflated obstacle face" is a linear inequality
# in the optimizer's free variables. A convex obstacle is avoided by committing
# each segment to ONE face (the homotopy side the discrete seed chose) — a
# committed half-space is convex; the raw "outside a convex polygon" disjunction
# is not. See build_obstacle_plan for the per-segment face commitment.
#
# Reuses from src/obstacles.jl (no forking): Obstacle (faces A,b unit outward
# normals, offsets, Σo), OBSTACLES, OBSTACLE_Z = Φ⁻¹(1−δ), AGENT_RADIUS, and the
# same anisotropic inflation  aⱼ·x ≥ bⱼ + r_a + z·√(aⱼᵀ(Σ+Σₒ)aⱼ).
#
# Runtime dependency: bspline_pad_controls / bspline_open_uniform_knots /
# bspline_basis_funs live in planners/hexspline_cl.jl and are used only inside
# functions here (never at include time), so include order does not matter as
# long as both files are loaded before the planner runs.

using LinearAlgebra

# Continuous-stage obstacle enforcement toggle (default on). Off ⇒ empty plans
# ⇒ zero effect (the discrete-stage filter is unaffected). Lets you ablate /
# A-B the continuous constraint. Materialized from CFG like the discrete knobs.
const OBSTACLE_CONTINUOUS = Bool(get(CFG, "obstacle_continuous", true))

# --------------------------------------------------------------------------
# A_mv — degree-3 MINVO basis matrix, getA_MV(3, [-1,1]) from mit-acl/minvo,
# ported verbatim from mit-acl/mader (mader_types.hpp `A_pos_mv_rest`).
# Row i = monomial coefficients [t³, t², t, 1] of MINVO basis function i, so
# b_mv(t) = A_MV · [t³;t²;t;1]. (The interval is baked into these decimals;
# the enclosure test verifies the curve lies inside conv(Q_mv) regardless.)
# --------------------------------------------------------------------------
const A_MV = [
    -3.4416308968564117698463178385282   6.9895481477801393310755884158425  -4.4622887507045296828778191411402   0.91437149978080234369315348885721;
     6.6792587327074839365081970754545 -11.845989901556746914934592496138    5.2523596690684613008670567069203   0.0;
    -6.6792587327074839365081970754545   8.1917862965657040064115790301003  -1.5981560640774179482548333908198   0.085628500219197656306846511142794;
     3.4416308968564117698463178385282  -3.3353445427890959784633650997421   0.80808514571348655231020075007109 -0.0000000000000000084567769453869345852581318467855
]
const A_MV_INV = inv(A_MV)

# --------------------------------------------------------------------------
# Per-segment B-spline basis matrix, derived NUMERICALLY from this repo's own
# clamped open-uniform knot vector + Cox-de Boor basis (bspline_basis_funs).
# This auto-handles clamped boundary segments (each distinct) vs. the uniform
# interior, and is guaranteed consistent with how the repo evaluates the spline
# — no hand-transcribed boundary matrices. Returns 4×4 A_bs with row r = monomial
# coefficients [t³,t²,t,1] of the basis function for control point (seg+r−1),
# t ∈ [0,1] local to the segment.
# --------------------------------------------------------------------------
function bs_segment_matrix(nctrl::Int, degree::Int, seg::Int)
    knots = bspline_open_uniform_knots(nctrl, degree)
    span  = degree + seg                       # find-span index for segment `seg`
    uL = knots[span]; uR = knots[span + 1]
    m  = degree + 1
    ts = [(k - 1) / (m - 1) for k in 1:m]       # sample params in [0,1]
    V  = [ts[k]^(degree - (c - 1)) for k in 1:m, c in 1:m]   # Vandermonde (high→low power)
    A_bs = Matrix{Float64}(undef, m, m)
    Bvals = Vector{Float64}(undef, m)
    for r in 1:m                                # one basis function at a time
        for k in 1:m
            u = uL + ts[k] * (uR - uL)
            N = bspline_basis_funs(span, u, degree, knots)
            Bvals[k] = N[r]
        end
        A_bs[r, :] = V \ Bvals                  # exact cubic fit
    end
    return A_bs
end

# M_seg = A_bs · inv(A_mv) so that Q_mv = Q_bs · M_seg. Cached per padded
# control count (segment types are fixed by count for a clamped uniform spline).
const _MSEG_CACHE = Dict{Int,Vector{Matrix{Float64}}}()
function segment_M_matrices(nctrl_padded::Int, degree::Int)
    get!(_MSEG_CACHE, nctrl_padded) do
        nspans = max(1, nctrl_padded - degree)
        [bs_segment_matrix(nctrl_padded, degree, s) * A_MV_INV for s in 1:nspans]
    end
end

# Mirror bspline_pad_controls' padding as an index map: padded position k came
# from unpadded control pad_source_indices(n)[k]. Lets us look up the Σ carried
# on the original (unpadded) control points for a padded segment.
function pad_source_indices(n::Int)
    n >= 4 && return collect(1:n)
    n == 3 && return [1, 1, 2, 3, 3]
    n == 2 && return [1, 1, 2, 2]
    n == 1 && return [1, 1, 1, 1]
    return Int[]
end

# One committed obstacle-avoidance row-set for an agent: segment `seg`, its
# conversion matrix, the 4 unpadded source control indices (for Σ lookup), the
# obstacle, and the committed face.
struct SegFace
    seg    :: Int
    Mseg   :: Matrix{Float64}
    src4   :: NTuple{4,Int}
    obs    :: Int
    face   :: Int
end

# 2×4 matrix of a padded segment's control points (columns = control points).
@inline function seg_control_matrix(padded::Vector{Tuple{Float64,Float64}}, seg::Int)
    Q = Matrix{Float64}(undef, 2, 4)
    @inbounds for k in 1:4
        p = padded[seg + k - 1]
        Q[1, k] = p[1]; Q[2, k] = p[2]
    end
    return Q
end

# Commit a face per (agent, segment, obstacle) from the DISCRETE SEED geometry:
# the face whose worst (min-over-MINVO-vertices) signed clearance is largest —
# i.e. the side the discrete path went around. Held fixed through refinement
# (keeps every row convex). Empty when there are no obstacles.
function build_obstacle_plan(seed_ctrls::Vector{Vector{Tuple{Float64,Float64}}}, degree::Int)
    plans = [SegFace[] for _ in eachindex(seed_ctrls)]
    (isempty(OBSTACLES) || !OBSTACLE_CONTINUOUS) && return plans
    for a in eachindex(seed_ctrls)
        padded = bspline_pad_controls(seed_ctrls[a])
        np = length(padded)
        np < degree + 1 && continue
        srcmap = pad_source_indices(length(seed_ctrls[a]))
        Ms = segment_M_matrices(np, degree)
        for s in eachindex(Ms)
            Qmv = seg_control_matrix(padded, s) * Ms[s]     # 2×4 seed MINVO vertices
            src4 = ntuple(k -> srcmap[s + k - 1], 4)
            for (mi, obs) in enumerate(OBSTACLES)
                best_face = 0; best_margin = -Inf
                for j in 1:length(obs.b)
                    ax = obs.A[j, 1]; ay = obs.A[j, 2]
                    minc = Inf
                    for v in 1:4
                        c = ax * Qmv[1, v] + ay * Qmv[2, v] - obs.b[j]
                        minc = min(minc, c)
                    end
                    if minc > best_margin
                        best_margin = minc; best_face = j
                    end
                end
                best_face > 0 && push!(plans[a], SegFace(s, Ms[s], src4, mi, best_face))
            end
        end
    end
    return plans
end

# Project source control index (1..nctrl) onto the covariance track index
# (1..length). Identity when Σ is carried per control point (cont_unc_use_
# waypoints: true); a proportional fallback when Σ is per spline sample.
@inline function cov_index(src::Int, nctrl::Int, len::Int)
    (nctrl <= 1 || len <= 1) && return clamp(src, 1, len)
    return clamp(round(Int, (src - 1) / (nctrl - 1) * (len - 1)) + 1, 1, len)
end

# Worst (largest) projected variance aᵀ(Σ+Σₒ)a over the 4 control points of a
# segment — the "re-propagated, worst-of-segment" Σ choice.
@inline function worst_proj_var(ax::Float64, ay::Float64, obs::Obstacle,
                                covs_a::Vector{Matrix{Float64}}, src4::NTuple{4,Int}, nctrl::Int)
    q = 0.0
    len = length(covs_a)
    @inbounds for k in 1:4
        Σ = covs_a[cov_index(src4[k], nctrl, len)]
        S11 = Σ[1,1] + obs.Σo[1,1]; S12 = Σ[1,2] + obs.Σo[1,2]
        S21 = Σ[2,1] + obs.Σo[2,1]; S22 = Σ[2,2] + obs.Σo[2,2]
        qk = ax * (S11 * ax + S12 * ay) + ay * (S21 * ax + S22 * ay)
        q = max(q, qk)
    end
    return q
end

# Barrier/penalty value of the obstacle rows, mirroring the curvature term's
# treatment exactly. mode=:smooth → CONT_SMOOTH_PENALTY·violation²;
# mode=:barrier → −μ·log(slack), or CONT_BARRIER_HARD_PENALTY·(1e-9−slack)² when
# the committed half-space is (nearly) breached.
function obstacle_constraint_value(plans::Vector{Vector{SegFace}},
                                   ctrls::Vector{Vector{Tuple{Float64,Float64}}},
                                   covs::Vector{Vector{Matrix{Float64}}};
                                   mode::Symbol, barrier_mu::Float64 = 0.0)
    total = 0.0
    for a in eachindex(plans)
        isempty(plans[a]) && continue
        padded = bspline_pad_controls(ctrls[a])
        nctrl = length(ctrls[a])
        for sf in plans[a]
            obs = OBSTACLES[sf.obs]
            ax = obs.A[sf.face, 1]; ay = obs.A[sf.face, 2]
            quad = worst_proj_var(ax, ay, obs, covs[a], sf.src4, nctrl)
            rhs = obs.b[sf.face] + AGENT_RADIUS + OBSTACLE_Z * sqrt(max(quad, 0.0))
            Qmv = seg_control_matrix(padded, sf.seg) * sf.Mseg
            for v in 1:4
                slack = ax * Qmv[1, v] + ay * Qmv[2, v] - rhs
                if mode === :smooth
                    slack < 0.0 && (total += CONT_SMOOTH_PENALTY * slack^2)
                else
                    if slack <= 1e-9
                        total += CONT_BARRIER_HARD_PENALTY * (1e-9 - slack)^2
                    else
                        total -= barrier_mu * log(slack)
                    end
                end
            end
        end
    end
    return total
end

# Committed-face MINVO-hull clearance certificate — the CONVEX feasibility gate
# the optimizer uses (obstacles_clear). Re-checks every committed MINVO row
# against its inflated obstacle at the FINAL controls+Σ; returns
# (agent, seg, obstacle, face, depth) for each breached vertex (depth > tol).
# SOUND but CONSERVATIVE: clearing the committed face ⇒ the whole hull (hence the
# curve segment) is outside that half-space ⇒ outside the obstacle. It can
# over-report, though — the MINVO hull bounds the curve loosely, so near the
# clamped spline ends the large boundary-segment hull can drape over a small
# nearby obstacle the curve actually rounds. Use verify_curve_collisions for the
# physically-real (sampled-curve) collision report.
function verify_obstacle_clearance(plans::Vector{Vector{SegFace}},
                                   ctrls::Vector{Vector{Tuple{Float64,Float64}}},
                                   covs::Vector{Vector{Matrix{Float64}}};
                                   tol::Float64 = 1e-6)
    viols = Tuple{Int,Int,Int,Int,Float64}[]
    for a in eachindex(plans)
        isempty(plans[a]) && continue
        padded = bspline_pad_controls(ctrls[a])
        nctrl = length(ctrls[a])
        for sf in plans[a]
            obs = OBSTACLES[sf.obs]
            ax = obs.A[sf.face, 1]; ay = obs.A[sf.face, 2]
            quad = worst_proj_var(ax, ay, obs, covs[a], sf.src4, nctrl)
            rhs = obs.b[sf.face] + AGENT_RADIUS + OBSTACLE_Z * sqrt(max(quad, 0.0))
            Qmv = seg_control_matrix(padded, sf.seg) * sf.Mseg
            worst = 0.0
            for v in 1:4
                depth = rhs - (ax * Qmv[1, v] + ay * Qmv[2, v])
                worst = max(worst, depth)
            end
            worst > tol && push!(viols, (a, sf.seg, sf.obs, sf.face, worst))
        end
    end
    return viols
end

# How deep a point sits inside a convex obstacle: 0 if outside (clears some
# face), else the min distance to a face (unit normals ⇒ signed clearance). This
# is the Σ=0 geometric containment — "is the mean point physically inside".
@inline function obstacle_penetration(x::Float64, y::Float64, obs::Obstacle)
    d = Inf
    @inbounds for j in 1:length(obs.b)
        s = obs.b[j] - (obs.A[j, 1] * x + obs.A[j, 2] * y)
        s < 0.0 && return 0.0
        d = min(d, s)
    end
    return d
end

# Physically-real collision report: densely sample each agent's refined clamped
# B-spline and test true polygon containment of the MEAN curve (Σ=0). Returns
# (agent, obstacle, depth) for every obstacle the curve actually enters. Unlike
# verify_obstacle_clearance (committed-face MINVO hull, conservative), the depth
# here is a real geometric collision, not a hull/committed-face artifact — so a
# curve that merely rounds a nearby obstacle is NOT reported. Needs no committed
# plan or Σ (pure geometry); reuses the repo's own spline evaluator.
function verify_curve_collisions(ctrls::Vector{Vector{Tuple{Float64,Float64}}};
                                 samples_per_seg::Int = 40, tol::Float64 = 1e-6)
    viols = Tuple{Int,Int,Float64}[]
    (isempty(OBSTACLES) || !OBSTACLE_CONTINUOUS) && return viols
    for a in eachindex(ctrls)
        length(ctrls[a]) < 2 && continue
        pts, _ = bspline_sample_path(ctrls[a]; length_samples_per_seg = samples_per_seg)
        for (mi, obs) in enumerate(OBSTACLES)
            worst = 0.0
            for (x, y) in pts
                worst = max(worst, obstacle_penetration(x, y, obs))
            end
            worst > tol && push!(viols, (a, mi, worst))
        end
    end
    return viols
end

@inline obstacles_clear(plans, ctrls, covs) = isempty(verify_obstacle_clearance(plans, ctrls, covs))
