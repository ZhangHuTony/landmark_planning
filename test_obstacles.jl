# Runnable checks for the static-obstacle chance-constraint filter
# (src/obstacles.jl). Standalone: stub CFG, include the module, assert.
# No test framework.   Run:  julia test_obstacles.jl

const CFG = Dict{String,Any}(
    "obstacles" => "0,0; 10,0; 10,10; 0,10",   # square [0,10]²
    "obstacle_delta" => 0.05,
    "agent_radius" => 0.0,
    "obstacle_edge_samples" => 2,
)
include(joinpath(@__DIR__, "src", "obstacles.jl"))

# ── norminv (Acklam) self-check ──
@assert abs(norminv(0.5)) < 1e-9
@assert abs(norminv(0.975) - 1.959963984540054) < 1e-7
@assert abs(norminv(0.95)  - 1.6448536269514722) < 1e-7
@assert norminv(1 - 0.05) ≈ OBSTACLE_Z
println("norminv OK  (z(0.95)=$(round(OBSTACLE_Z, digits=5)))")

sq = build_obstacle([(0.0,0.0),(10.0,0.0),(10.0,10.0),(0.0,10.0)])
@assert size(sq.A) == (4,2)
@assert all(abs(hypot(sq.A[j,1], sq.A[j,2]) - 1.0) < 1e-12 for j in 1:4)   # UNIT normals
Σ0 = zeros(2,2)

# (1) Σ=0 and δ→0.5 (z→0, r_a=0) reduces to plain geometric containment.
z0 = norminv(1 - 0.5)
@assert z0 == 0.0
@assert !clears_obstacle(5.0, 5.0, Σ0, sq, z0, 0.0)     # inside  ⇒ collision
@assert  clears_obstacle(-1.0, 5.0, Σ0, sq, z0, 0.0)    # outside ⇒ safe
@assert  clears_obstacle(11.0, 5.0, Σ0, sq, z0, 0.0)
println("geometric containment (Σ=0, δ=0.5) OK")

# (2) Monotonicity: at a fixed mean just outside the x=0 face, larger Σ OR
#     smaller δ (larger z) thickens the inflation ⇒ flips safe→collision.
μ = (-1.0, 5.0)
Σbig = [4.0 0.0; 0.0 4.0]                    # std 2 m
z_lo = norminv(1 - 0.40)                     # loose
z_hi = norminv(1 - 0.001)                    # tight ≈ 3.09
@assert  clears_obstacle(μ..., Σ0,   sq, z_hi, 0.0)     # Σ=0 ⇒ margin 0 ⇒ safe
@assert  clears_obstacle(μ..., Σbig, sq, z_lo, 0.0)     # loose δ ⇒ safe
@assert !clears_obstacle(μ..., Σbig, sq, z_hi, 0.0)     # tight δ + big Σ ⇒ collision
@assert  clears_obstacle(μ..., [0.1 0.0;0.0 0.1], sq, z_lo, 0.0)   # small Σ ⇒ safe
@assert !clears_obstacle(μ..., [64.0 0.0;0.0 64.0], sq, z_lo, 0.0) # big Σ (same z) ⇒ collision
println("monotonicity (Σ↑ / δ↓ ⇒ thicker standoff) OK")

# (3) mean just outside a face with LARGE Σ ⇒ flagged infeasible.
@assert !clears_obstacle(-0.5, 5.0, [4.0 0.0;0.0 4.0], sq, norminv(1-0.05), 0.0)
println("just-outside + large Σ ⇒ infeasible OK")

# Anisotropy: only variance ALONG the face normal matters. At the x=0 face,
# huge cross-track (y) variance is irrelevant; huge along-track (x) variance
# inflates the standoff.
@assert  clears_obstacle(-1.0, 5.0, [0.01 0.0; 0.0 100.0], sq, norminv(1-0.05), 0.0)
@assert !clears_obstacle(-1.0, 5.0, [100.0 0.0; 0.0 0.01], sq, norminv(1-0.05), 0.0)
println("anisotropic projection OK")

# Agent radius inflates: clears at r_a=0, collides at large r_a.
@assert  clears_obstacle(-1.0, 5.0, Σ0, sq, 0.0, 0.5)
@assert !clears_obstacle(-1.0, 5.0, Σ0, sq, 0.0, 2.0)
println("agent-radius inflation OK")

# CW winding yields the same outward normals as CCW.
sq_cw = build_obstacle([(0.0,0.0),(0.0,10.0),(10.0,10.0),(10.0,0.0)])
@assert !clears_obstacle(5.0, 5.0, Σ0, sq_cw, 0.0, 0.0)
@assert  clears_obstacle(-1.0, 5.0, Σ0, sq_cw, 0.0, 0.0)
println("winding-independent OK")

# Non-convex ('dart') polygon must error.
errors(v) = try (build_obstacle(v); false) catch; true end
@assert errors([(0.0,0.0),(10.0,0.0),(5.0,3.0),(10.0,10.0),(0.0,10.0)]) "non-convex polygon should error"
@assert errors([(0.0,0.0),(10.0,0.0)]) "polygon with <3 vertices should error"
println("non-convex guard OK")

# Parse round-trip, incl. optional Σo suffix.
obs2 = parse_obstacles("0,0; 4,0; 4,4; 0,4 | 100,0; 104,0; 102,4 @ 1,0,0,2")
@assert length(obs2) == 2
@assert obs2[2].Σo == [1.0 0.0; 0.0 2.0]
@assert isempty(parse_obstacles("")) && isempty(parse_obstacles("   "))
println("parse (2 polys + Σo, empty) OK")

# Edge sampler (uses the stub OBSTACLES = the [0,10]² square): a segment that
# stays outside is free; one whose destination lands inside is blocked.
@assert  segment_obstacle_free((-5.0,5.0), (-1.0,5.0), Σ0)
@assert !segment_obstacle_free((-5.0,5.0), ( 5.0,5.0), Σ0)
println("segment sampler OK")

println("\nall obstacle checks passed ✓")
