# ==========================================================================
# Static-obstacle chance constraint — discrete A* feasibility filter
# ==========================================================================
# Hard no-go CONVEX-polygon obstacles. This is a PURE FEASIBILITY CHECK on the
# belief (mean μ, covariance Σ) already carried on each A* search state:
# obstacles do NOT enter the measurement model and NEVER modify covariance
# propagation. Nothing here writes Σ — it is read-only w.r.t. the estimator.
#
# Per agent i, per obstacle m, evaluated at (μ_i, Σ_i) at risk δ:
#   convex obstacle = intersection of half-planes {x : aⱼ·x ≤ bⱼ}, aⱼ UNIT
#   outward normals (‖aⱼ‖=1). Agent-vs-obstacle is SAFE iff THERE EXISTS a
#   face j clearing the chance constraint (a disjunction):
#       aⱼ·μ_i ≥ bⱼ + r_a + z·√(aⱼᵀ(Σ_i + Σₒ_m)aⱼ),   z = Φ⁻¹(1−δ)
#   r_a  = agent radius (inflates each face outward), Σₒ_m = obstacle location
#   covariance (2×2, DEFAULT ZERO). Clearing ANY face ⇒ the belief lies outside
#   the inflated polygon. If NO face clears, that (agent, obstacle) pair is in
#   collision. A joint state is INFEASIBLE if ANY agent collides with ANY
#   obstacle — such states are never enqueued/expanded.
#
# RISK / union bound: with one δ per (agent, obstacle) pair, the total collision
# probability obeys  P(any collision) ≤ Σ δ  over all P·M pairs. For a target
# total budget Δ, set δ = Δ/(P·M).
#
# Φ⁻¹ (standard-normal inverse CDF) uses Acklam's rational approximation:
# Distributions.jl / StatsFuns.jl are NOT installed (only Plots +
# DataStructures), so quantile(Normal(),·) / norminvcdf are unavailable.

# Acklam's algorithm — |abs error| < 1.15e-9 over 0<p<1.
function norminv(p::Float64)
    (0.0 < p < 1.0) || throw(DomainError(p, "norminv expects 0 < p < 1"))
    a = (-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
          1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00)
    b = (-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
          6.680131188771972e+01, -1.328068155288572e+01)
    c = (-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
         -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00)
    d = (7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
         3.754408661907416e+00)
    plow = 0.02425; phigh = 1 - plow
    if p < plow
        q = sqrt(-2*log(p))
        return (((((c[1]*q+c[2])*q+c[3])*q+c[4])*q+c[5])*q+c[6]) /
               ((((d[1]*q+d[2])*q+d[3])*q+d[4])*q+1)
    elseif p ≤ phigh
        q = p - 0.5; r = q*q
        return (((((a[1]*r+a[2])*r+a[3])*r+a[4])*r+a[5])*r+a[6])*q /
               (((((b[1]*r+b[2])*r+b[3])*r+b[4])*r+b[5])*r+1)
    else
        q = sqrt(-2*log(1-p))
        return -(((((c[1]*q+c[2])*q+c[3])*q+c[4])*q+c[5])*q+c[6]) /
                ((((d[1]*q+d[2])*q+d[3])*q+d[4])*q+1)
    end
end

struct Obstacle
    verts :: Vector{Tuple{Float64,Float64}}  # polygon vertices, in order (for viz + build)
    A     :: Matrix{Float64}                  # m×2 UNIT outward face normals (row j = aⱼ)
    b     :: Vector{Float64}                  # m offsets: face j = {x : aⱼ·x ≤ bⱼ}
    Σo    :: Matrix{Float64}                  # 2×2 location covariance (default zero)
end

# Build a convex-polygon obstacle from ordered vertices. Errors on a non-convex
# polygon (convex decomposition is a separate task).
function build_obstacle(verts::Vector{Tuple{Float64,Float64}}; Σo::Matrix{Float64}=zeros(2,2))
    n = length(verts)
    n ≥ 3 || error("obstacle polygon needs ≥3 vertices, got $n")
    area2 = 0.0                                  # 2× signed area (shoelace) → winding
    for j in 1:n
        (x1,y1) = verts[j]; (x2,y2) = verts[mod1(j+1,n)]
        area2 += x1*y2 - x2*y1
    end
    s = sign(area2)                              # +1 CCW, -1 CW
    s == 0 && error("degenerate obstacle polygon (zero area)")
    A = Matrix{Float64}(undef, n, 2)
    b = Vector{Float64}(undef, n)
    for j in 1:n
        (x0,y0) = verts[mod1(j-1,n)]
        (x1,y1) = verts[j]
        (x2,y2) = verts[mod1(j+1,n)]
        ex = x2-x1; ey = y2-y1                    # edge j: vⱼ → vⱼ₊₁
        L = hypot(ex, ey)
        L > 0 || error("obstacle polygon has a zero-length edge at vertex $j")
        ax = s*ey/L; ay = -s*ex/L                 # UNIT outward normal (s orients away from interior)
        A[j,1] = ax; A[j,2] = ay
        b[j] = ax*x1 + ay*y1
        cross = (x1-x0)*ey - (y1-y0)*ex           # turn direction at vertex j
        s*cross < -1e-9 && error("non-convex obstacle polygon at vertex $j; convex decomposition is a separate task")
    end
    return Obstacle(verts, A, b, Σo)
end

# Parse the flat main.yaml string encoding (load_yaml keeps it as a raw string):
#   "x,y; x,y; ... [@ sxx,sxy,syx,syy]  |  x,y; ..."
# vertices sep by ';', x/y by ',', polygons by '|', optional per-polygon Σo
# after '@' (4 numbers, row-major; default zero). Empty string ⇒ no obstacles.
function parse_obstacles(str::AbstractString)
    obs = Obstacle[]
    s = strip(str)
    isempty(s) && return obs
    for poly_spec in split(s, '|')
        poly_spec = strip(poly_spec)
        isempty(poly_spec) && continue
        Σo = zeros(2,2)
        geom = poly_spec
        if occursin('@', poly_spec)
            geom, cov_spec = split(poly_spec, '@'; limit=2)
            cvals = parse.(Float64, strip.(split(strip(cov_spec), ',')))
            length(cvals) == 4 || error("obstacle Σo needs 4 numbers (sxx,sxy,syx,syy), got $(length(cvals))")
            Σo = [cvals[1] cvals[2]; cvals[3] cvals[4]]
        end
        verts = Tuple{Float64,Float64}[]
        for vtx in split(strip(geom), ';')
            vtx = strip(vtx)
            isempty(vtx) && continue
            xy = parse.(Float64, strip.(split(vtx, ',')))
            length(xy) == 2 || error("obstacle vertex needs 'x,y', got '$vtx'")
            push!(verts, (xy[1], xy[2]))
        end
        push!(obs, build_obstacle(verts; Σo=Σo))
    end
    return obs
end

# SAFE (clears the obstacle) iff ∃ face j: aⱼ·μ ≥ bⱼ + r_a + z·√(aⱼᵀ(Σ+Σₒ)aⱼ).
# General anisotropic Σ — no isotropy assumption. Inlined 2×2 algebra to avoid
# allocation in the A* hot loop.
@inline function clears_obstacle(μx::Float64, μy::Float64, Σ::AbstractMatrix,
                                 obs::Obstacle, z::Float64, r_a::Float64)
    S11 = Σ[1,1] + obs.Σo[1,1]; S12 = Σ[1,2] + obs.Σo[1,2]
    S21 = Σ[2,1] + obs.Σo[2,1]; S22 = Σ[2,2] + obs.Σo[2,2]
    @inbounds for j in 1:length(obs.b)
        ax = obs.A[j,1]; ay = obs.A[j,2]
        quad = ax*(S11*ax + S12*ay) + ay*(S21*ax + S22*ay)   # aⱼᵀ(Σ+Σₒ)aⱼ ≥ 0
        if ax*μx + ay*μy ≥ obs.b[j] + r_a + z*sqrt(max(quad, 0.0))
            return true
        end
    end
    return false
end

# Edge feasibility: sample the μ_prev→μ_dest segment (Σ held at the endpoint
# value, per spec) and require every sample to clear every obstacle. nsamp=1
# checks the destination only; nsamp≥2 includes both endpoints (default) plus
# evenly-spaced interior points.
function segment_obstacle_free(μprev::NTuple{2,Float64}, μdest::NTuple{2,Float64},
                               Σ::AbstractMatrix)
    isempty(OBSTACLES) && return true
    nsamp = OBSTACLE_EDGE_SAMPLES
    @inbounds for si in 1:nsamp
        t = nsamp == 1 ? 1.0 : (si-1)/(nsamp-1)
        μx = μprev[1] + t*(μdest[1]-μprev[1])
        μy = μprev[2] + t*(μdest[2]-μprev[2])
        for obs in OBSTACLES
            clears_obstacle(μx, μy, Σ, obs, OBSTACLE_Z, AGENT_RADIUS) || return false
        end
    end
    return true
end

# ── Config-materialized constants (safe empty defaults ⇒ zero behavior change
#    for scenarios with no `obstacles:` key). ──
const OBSTACLE_DELTA        = Float64(get(CFG, "obstacle_delta", 0.05))   # risk per (agent, obstacle) pair
const OBSTACLE_Z            = norminv(1.0 - OBSTACLE_DELTA)                # Φ⁻¹(1−δ), computed once
const AGENT_RADIUS          = Float64(get(CFG, "agent_radius", 0.0))
const OBSTACLE_EDGE_SAMPLES = Int(get(CFG, "obstacle_edge_samples", 2))
const OBSTACLES             = parse_obstacles(String(get(CFG, "obstacles", "")))
@assert OBSTACLE_EDGE_SAMPLES ≥ 1 "obstacle_edge_samples must be ≥ 1"
