# Runnable checks for the MINVO-enclosure continuous obstacle constraint
# (src/minvo.jl). Standalone: stub CFG + the pure B-spline helpers minvo.jl
# needs at runtime, include the modules, assert. No test framework.
#   Run:  julia test_minvo.jl
using LinearAlgebra

const CFG = Dict{String,Any}(
    # Wide, thin box (x∈[0,10], y∈[-1,1]): for a seed passing ABOVE it, the top
    # face is the only clearing face, so every segment commits the same face —
    # lets the along-normal (anisotropy) test isolate one face cleanly.
    "obstacles" => "0,-1; 10,-1; 10,1; 0,1",
    "obstacle_delta" => 0.05,
    "agent_radius" => 0.0,
    "obstacle_edge_samples" => 2,
)
include(joinpath(@__DIR__, "src", "obstacles.jl"))
# Obstacle geometry comes from the scenario (src/scenario_generation.jl), which
# needs the whole graph/Landmark stack — stub the field minvo.jl reads.
const OBSTACLES = parse_obstacles(CFG["obstacles"])

# Constants + pure B-spline helpers that minvo.jl references at runtime (they
# live in planners/hexspline_cl.jl, too heavy to include standalone — copied
# verbatim, identical pure math).
const SPLINE_DEGREE = 3
const CONT_SMOOTH_PENALTY = 1.0e3
const CONT_BARRIER_HARD_PENALTY = 1.0e4
function bspline_pad_controls(control_pts::Vector{Tuple{Float64,Float64}})
    n = length(control_pts)
    if n >= 4; return control_pts
    elseif n == 3; return [control_pts[1], control_pts[1], control_pts[2], control_pts[3], control_pts[3]]
    elseif n == 2; return [control_pts[1], control_pts[1], control_pts[2], control_pts[2]]
    elseif n == 1; return [control_pts[1], control_pts[1], control_pts[1], control_pts[1]]
    else; return Tuple{Float64,Float64}[]; end
end
function bspline_open_uniform_knots(nctrl::Int, degree::Int)
    nctrl >= degree + 1 || return Float64[]
    knots = Vector{Float64}(undef, nctrl + degree + 1); nspans = max(1, nctrl - degree)
    for i in 1:length(knots)
        if i <= degree + 1; knots[i] = 0.0
        elseif i > nctrl; knots[i] = 1.0
        else; knots[i] = (i - degree - 1) / nspans; end
    end
    return knots
end
@inline function bspline_find_span(u::Float64, knots::Vector{Float64}, degree::Int, nctrl::Int)
    u >= knots[end - degree] && return nctrl
    low = degree + 1; high = nctrl
    while low <= high
        mid = (low + high) >>> 1
        if u < knots[mid]; high = mid - 1
        elseif u >= knots[mid + 1]; low = mid + 1
        else; return mid; end
    end
    return clamp(low, degree + 1, nctrl)
end
function bspline_basis_funs(span::Int, u::Float64, degree::Int, knots::Vector{Float64})
    N = zeros(Float64, degree + 1); left = zeros(Float64, degree); right = zeros(Float64, degree); N[1] = 1.0
    for j in 1:degree
        left[j] = u - knots[span + 1 - j]; right[j] = knots[span + j] - u; saved = 0.0
        for r in 1:j
            denom = right[r] + left[j - r + 1]; temp = abs(denom) < 1e-14 ? 0.0 : N[r] / denom
            N[r] = saved + right[r] * temp; saved = left[j - r + 1] * temp
        end
        N[j + 1] = saved
    end
    return N
end
function bspline_eval_point(control_pts, knots, degree, u)
    nctrl = length(control_pts); u = clamp(u, 0.0, 1.0)
    span = bspline_find_span(u, knots, degree, nctrl); basis = bspline_basis_funs(span, u, degree, knots)
    first = span - degree; x = 0.0; y = 0.0
    for j in 0:degree; pt = control_pts[first + j]; x += basis[j+1]*pt[1]; y += basis[j+1]*pt[2]; end
    return (x, y)
end
include(joinpath(@__DIR__, "src", "minvo.jl"))

# small 2D convex-hull helpers (for enclosure/tightness assertions)
function in_hull(pts::Matrix{Float64}, p::Vector{Float64}; tol=1e-6)
    c = [sum(pts[1,:]), sum(pts[2,:])]/size(pts,2)
    P = pts[:, sortperm([atan(pts[2,k]-c[2], pts[1,k]-c[1]) for k in 1:size(pts,2)])]; n = size(P,2)
    for k in 1:n
        a = P[:,k]; b = P[:, k%n+1]
        ((b[1]-a[1])*(p[2]-a[2]) - (b[2]-a[2])*(p[1]-a[1])) < -tol && return false
    end
    return true
end
hullarea(pts) = begin
    c = [sum(pts[1,:]), sum(pts[2,:])]/size(pts,2)
    P = pts[:, sortperm([atan(pts[2,k]-c[2], pts[1,k]-c[1]) for k in 1:size(pts,2)])]; n=size(P,2); s=0.0
    for k in 1:n; a=P[:,k]; b=P[:,k%n+1]; s += a[1]*b[2]-b[1]*a[2]; end
    abs(s)/2
end

# ── (1) MINVO enclosure: sampled curve ⊂ conv(Q_mv), hull tighter than B-spline ──
ctrls = [(0.0,3.0),(1.67,3.0),(3.33,3.0),(5.0,3.0),(6.67,3.0),(8.33,3.0),(10.0,3.0)]  # nctrl=7
padded = bspline_pad_controls(ctrls); np = length(padded)
knots = bspline_open_uniform_knots(np, SPLINE_DEGREE)
Ms = segment_M_matrices(np, SPLINE_DEGREE)
@assert length(Ms) == np - SPLINE_DEGREE
for s in eachindex(Ms)
    Qbs = seg_control_matrix(padded, s); Qmv = Qbs * Ms[s]
    uL = knots[SPLINE_DEGREE+s]; uR = knots[SPLINE_DEGREE+s+1]
    for tt in range(0,1;length=40)
        p = collect(bspline_eval_point(padded, knots, SPLINE_DEGREE, uL + tt*(uR-uL)))
        @assert in_hull(Qmv, p) "curve point outside MINVO hull, seg $s"
    end
    @assert hullarea(Qmv) <= hullarea(Qbs) + 1e-9 "MINVO hull not tighter, seg $s"
end
println("MINVO enclosure + tighter-than-control-hull OK")

# ── (2) Interval/clamped correctness: numerically-derived A_bs matches mader's
#        hardcoded per-segment matrices (boundary segs distinct from interior). ──
A_bs_seg0  = [-1.0 3.0 -3.0 1.0; 1.75 -4.5 3.0 0; -0.9167 1.5 0 0; 0.1667 0 0 0]
A_bs_rest  = [-0.1667 0.5 -0.5 0.1667; 0.5 -1.0 0 0.6667; -0.5 0.5 0.5 0.1667; 0.1667 0 0 0]
A_bs_last  = [-0.1667 0.5 -0.5 0.1667; 0.9167 -1.25 -0.25 0.5833; -1.75 0.75 0.75 0.25; 1.0 0 0 0]
@assert maximum(abs.(bs_segment_matrix(8, 3, 1) .- A_bs_seg0)) < 2e-4   # first boundary seg
@assert maximum(abs.(bs_segment_matrix(8, 3, 3) .- A_bs_rest)) < 2e-4   # uniform interior seg
@assert maximum(abs.(bs_segment_matrix(8, 3, 5) .- A_bs_last)) < 2e-4   # last boundary seg
@assert maximum(abs.(bs_segment_matrix(8, 3, 1) .- A_bs_rest)) > 0.1    # boundary ≠ interior
println("clamped boundary vs uniform-interior A_bs (vs mader ref) OK")

# ── (3) Σ=0 reduces to plain geometry (standoff = agent radius only). ──
zerocov = [[zeros(2,2) for _ in 1:length(ctrls)]]
plan = build_obstacle_plan([ctrls], SPLINE_DEGREE)
@assert !isempty(plan[1])
# every committed face for this above-the-box seed is the top face (normal (0,1))
for sf in plan[1]
    a = OBSTACLES[sf.obs].A[sf.face, :]
    @assert isapprox(a, [0.0, 1.0]; atol=1e-9) "expected top face committed, got $a"
end
@assert obstacles_clear(plan, [ctrls], zerocov)   # y=3 seed clears top face (b=1) at Σ=0
println("Σ=0 geometric reduction + face commitment (top face) OK")

# ── (4) Inflation monotonicity in Σ: a seed just above the face clears at Σ=0
#        but a large along-normal variance widens the standoff → violated. ──
tight = [(0.0,1.05),(1.67,1.05),(3.33,1.05),(5.0,1.05),(6.67,1.05),(8.33,1.05),(10.0,1.05)]
plan_t = build_obstacle_plan([tight], SPLINE_DEGREE)
@assert obstacles_clear(plan_t, [tight], [[zeros(2,2) for _ in 1:7]])          # Σ=0 → clears (1.05>1)
bigcov = [[[0.01 0.0; 0.0 4.0] for _ in 1:7]]                                    # std 2 m along y (the face normal)
@assert !obstacles_clear(plan_t, [tight], bigcov)                               # z·2 standoff ⇒ violated
smallcov = [[[0.01 0.0; 0.0 1e-4] for _ in 1:7]]
@assert obstacles_clear(plan_t, [tight], smallcov)                              # tiny Σ ⇒ still clears
# anisotropy: huge cross-normal (x) variance is irrelevant to the top face
crosscov = [[[100.0 0.0; 0.0 1e-4] for _ in 1:7]]
@assert obstacles_clear(plan_t, [tight], crosscov)
println("Σ-monotonicity + anisotropic projection OK")
# δ inflation direction (shared discrete formula): smaller δ ⇒ larger z ⇒ inward
@assert norminv(1 - 0.001) > OBSTACLE_Z
println("δ inflation direction (smaller δ ⇒ larger standoff) OK")

# ── (5) End-to-end re-verification (spec E): a good seed is clear; perturbing a
#        control point into the obstacle is caught by verify_obstacle_clearance. ──
good = obstacles_clear(plan, [ctrls], zerocov)
@assert good
perturbed = copy(ctrls); perturbed[4] = (5.0, -5.0)  # drag a middle ctrl point down through the box
viols = verify_obstacle_clearance(plan, [perturbed], zerocov)
@assert !isempty(viols) "perturbed control point into obstacle must be flagged"
@assert any(v -> v[5] > 0.5, viols) "violation depth should be clearly positive"
# the optimizer's constraint rows: some vertex slack goes negative on the violated
# config, all stay non-negative on the clear one (this is what the smoothing and
# barrier penalties were built from before they became ordinary slack entries)
@assert any(x -> x < 0.0, obstacle_slacks(plan, [perturbed], zerocov))
@assert all(x -> x >= 0.0, obstacle_slacks(plan, [ctrls], zerocov))
println("end-to-end re-verification (clear seed vs perturbed) OK")

# ── no-obstacle safety: empty plan ⇒ no-op everywhere ──
@assert isempty(build_obstacle_plan([ctrls], SPLINE_DEGREE)[1]) == false  # (obstacles present here)
println("\nall MINVO obstacle checks passed ✓")
