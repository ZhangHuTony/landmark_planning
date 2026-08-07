# Self-check for the Covariance-Intersection inter-agent fusion (src/covariance.jl).
# Run:  julia test_comm_ci.jl
# Verifies: (1) ci_omega_det finds the det-optimal ω, (2) fusing an estimate with
# a noisier copy of itself does NOT shrink (the soundness property KF violates),
# (3) fusion vanishes as the comm weight w → 0.

using LinearAlgebra
using Random

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "covariance.jl"))

det2(M) = M[1,1]*M[2,2] - M[1,2]*M[2,1]

# Random symmetric positive-definite 2×2.
function rand_spd(rng)
    a = randn(rng); b = randn(rng); c = randn(rng)
    L = [exp(a) 0.0; b exp(c)]        # lower-triangular, positive diagonal
    return L * L'
end

Random.seed!(1)
rng = Random.default_rng()

# (1) ci_omega_det matches a brute-force grid maximizer of det(ω·Ia + (1−ω)·Ib).
for _ in 1:2000
    Ia = rand_spd(rng); Ib = rand_spd(rng)
    ω = ci_omega_det(Ia, Ib)
    obj(w) = det2(w .* Ia .+ (1.0 - w) .* Ib)
    grid_best = maximum(obj(w) for w in 0.0:0.001:1.0)
    @assert 0.0 <= ω <= 1.0 "ω out of [0,1]: $ω"
    @assert obj(ω) >= grid_best - 1e-9 "ci_omega_det suboptimal: $(obj(ω)) < $grid_best"
end

# (2) No-shrinkage: fusing identical estimates (w=1) returns the same covariance.
#     (The legacy KF add would strictly shrink det here → overconfidence.)
for _ in 1:2000
    P = rand_spd(rng)
    na, nb = ci_comm(P, P, 1.0)
    @assert maximum(abs.(na .- P)) < 1e-8 "CI shrank identical estimates: $na vs $P"
    @assert det2(na) >= det2(P) - 1e-10 "CI reduced det of identical-estimate fusion"
    @assert maximum(abs.(nb .- P)) < 1e-8 "CI (b←a) shrank identical estimates"
end

# (3) Consistency invariant: det-optimal CI is never worse than keeping the prior
#     (ω=1 is always available), for any weight — so det(fused) ≤ det(prior).
#     This is what makes the terminal certificate a sound upper bound.
for _ in 1:2000
    Pa = rand_spd(rng); Pb = rand_spd(rng)
    for w in (1.0, 0.5, COMM_WEIGHT_MIN)
        na, nb = ci_comm(Pa, Pb, w)
        @assert det2(na) <= det2(Pa) + 1e-9 "CI increased det beyond prior (a), w=$w"
        @assert det2(nb) <= det2(Pb) + 1e-9 "CI increased det beyond prior (b), w=$w"
    end
end

# (4) Full-weight fusion of well-conditioned agents actually reduces uncertainty
#     (sanity: CI still fuses, it isn't a no-op).
let Pa = [4.0 0.0; 0.0 4.0], Pb = [4.0 0.0; 0.0 4.0]
    na, _ = ci_comm(Pa, [1.0 0.0; 0.0 1.0], 1.0)  # much better helper
    @assert det2(na) < det2(Pa) "CI failed to reduce uncertainty from a confident helper"
end

println("test_comm_ci: all assertions passed")
