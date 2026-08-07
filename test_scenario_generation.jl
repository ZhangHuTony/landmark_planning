# Self-check for the random scenario generator
# (src/scenario_generation.jl:generate_scenario / random_convex_obstacles).
#
# Run:  julia test_scenario_generation.jl
#       julia test_scenario_generation.jl --preview 12   # + PNG of 12 scenarios
#
# Verifies the properties run_constraint_sweep.jl depends on:
#   1. a scenario is a pure function of (seed, goal_dist, n_landmarks,
#      n_obstacles) — including independence from the GLOBAL RNG, which
#      generate_plan.jl seeds with a hardcoded Random.seed!(42),
#   2. exact landmark/obstacle counts,
#   3. landmarks land in the band where they can actually be sensed but are
#      still off the direct line,
#   4. every generated obstacle is convex and non-degenerate (build_obstacle
#      throws otherwise, so this is really a stress test of the ellipse-angle
#      construction), non-overlapping, and clear of start/goal,
#   5. n_landmarks = 0 is refused rather than producing a BoundsError deep in
#      build_hex_graph.

using Plots, LinearAlgebra, Random

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so our
# own flags have to be taken off ARGS first or they get read as a path.
# (Same dance as mc_nees.jl:62.) Emptying it makes config.jl use config/.
const FLAGS = copy(ARGS)
empty!(ARGS)

const _ROOT = @__DIR__
include(joinpath(_ROOT, "src", "config.jl"))
include(joinpath(_ROOT, "src", "obstacles.jl"))
include(joinpath(_ROOT, "src", "minvo.jl"))
include(joinpath(_ROOT, "src", "graph.jl"))
include(joinpath(_ROOT, "src", "scenario_generation.jl"))

same(a, b) = a.start == b.start && a.goal == b.goal &&
    length(a.landmarks) == length(b.landmarks) &&
    all(a.landmarks[i].x == b.landmarks[i].x && a.landmarks[i].y == b.landmarks[i].y &&
        a.landmarks[i].cov == b.landmarks[i].cov for i in eachindex(a.landmarks)) &&
    length(a.obstacles) == length(b.obstacles) &&
    all(a.obstacles[i].verts == b.obstacles[i].verts for i in eachindex(a.obstacles))

aabb(o) = (minimum(first.(o.verts)), minimum(last.(o.verts)),
           maximum(first.(o.verts)), maximum(last.(o.verts)))

gen(seed, D, nl, no) = generate_scenario(seed=seed, goal_dist=D, n_landmarks=nl, n_obstacles=no)

# ── (1) Reproducible from its four arguments, and only those ────────────────
a = gen(7, 1000.0, 3, 3)
b = gen(7, 1000.0, 3, 3)
@assert same(a, b) "same seed produced different scenarios"
@assert !same(a, gen(8, 1000.0, 3, 3)) "different seeds produced the same scenario"

# The global RNG must not leak in: perturb it and regenerate.
Random.seed!(999); rand(37)
@assert same(a, gen(7, 1000.0, 3, 3)) "generator depends on the global RNG stream"

# ── (2)-(4) Structural properties, swept over many seeds/shapes ─────────────
n_sides = Set{Int}()
for seed in 1:150
    D  = 600.0 + 600.0 * (seed % 7) / 6
    nl = 1 + seed % 4
    no = seed % 5
    sc = gen(seed, D, nl, no)

    @assert length(sc.landmarks) == nl "landmark count $(length(sc.landmarks)) != $nl (seed $seed)"
    @assert length(sc.obstacles) == no "obstacle count $(length(sc.obstacles)) != $no (seed $seed)"
    @assert sc.start == (0.0, 0.0) && sc.goal == (D, 0.0)

    for lm in sc.landmarks
        @assert 0.12D - 1e-9 <= lm.x <= 0.88D + 1e-9 "landmark x=$(lm.x) outside corridor (seed $seed)"
        # Inner edge 200 m is the point of the band: inside it a landmark is a free
        # fix (100 m = the primary sees it from the direct line; 200 m = a support
        # one comm_range off the line sees it), and a free fix measures nothing.
        @assert 200.0 - 1e-9 <= abs(lm.y) <= 320.0 + 1e-9 "landmark |y|=$(abs(lm.y)) outside the sensible band (seed $seed)"
        # Σ₀ comes from landmarks[1].cov, so a non-SPD draw would poison every agent.
        @assert issymmetric(lm.cov) && det(lm.cov) > 0 && lm.cov[1,1] > 0 "landmark cov not SPD (seed $seed)"
    end

    boxes = aabb.(sc.obstacles)
    for (i, o) in enumerate(sc.obstacles)
        # Reaching here at all means build_obstacle accepted it: convex, >=3
        # vertices, non-zero area, no zero-length edge.
        push!(n_sides, length(o.verts))
        @assert 3 <= length(o.verts) <= 6 "obstacle with $(length(o.verts)) vertices (seed $seed)"
        cx = sum(first.(o.verts)) / length(o.verts)
        cy = sum(last.(o.verts)) / length(o.verts)
        @assert hypot(cx, cy) >= 90.0 "obstacle sits on start (seed $seed)"
        @assert hypot(cx - D, cy) >= 90.0 "obstacle sits on goal (seed $seed)"
        for j in (i+1):length(sc.obstacles)
            p, q = boxes[i], boxes[j]
            @assert (p[3] + 12.0 < q[1] || p[1] - 12.0 > q[3] ||
                     p[4] + 12.0 < q[2] || p[2] - 12.0 > q[4]) "obstacles $i/$j overlap (seed $seed)"
        end
    end
end
@assert n_sides == Set(3:6) "expected triangles through hexagons, got $(sort(collect(n_sides)))"

# ── (5) An empty landmark field is refused, not silently generated ──────────
refused = try; gen(1, 1000.0, 0, 1); false; catch; true; end
@assert refused "n_landmarks=0 should error (Σ₀ comes from landmarks[1])"

println("test_scenario_generation ✓  (reproducible, counts exact, landmarks in band, " *
        "obstacles convex/disjoint/clear, shapes $(sort(collect(n_sides))) sides)")

# ── Optional: render N scenarios so the geometry can be eyeballed ───────────
# PLAN.md Phase 2 asks for visual inspection before trusting the generator.
if "--preview" in FLAGS
    i = findfirst(==("--preview"), FLAGS)
    n = (i < length(FLAGS) && tryparse(Int, FLAGS[i+1]) !== nothing) ? parse(Int, FLAGS[i+1]) : 12
    outdir = get(ENV, "PREVIEW_DIR", joinpath(_ROOT, "results", "verify", "scenario_generation"))
    mkpath(outdir)
    rng = Random.MersenneTwister(0)
    panels = []
    for k in 1:n
        D  = 50.0 * round((600.0 + 600.0 * rand(rng)) / 50.0)
        nl = rand(rng, 1:4)
        no = rand(rng, 0:4)
        sc = gen(1000 + k, D, nl, no)
        p = plot(aspect_ratio=:equal, legend=false, xlims=(-80, D + 80), ylims=(-450, 320),
                 title="s$(1000+k)  D=$(Int(D))  L=$nl  O=$no", titlefontsize=7,
                 tickfontsize=5, grid=false)
        # Reachable hex rows at hex_width_m=100 — landmarks must be within
        # visibility_range of one of these to be worth anything.
        for y in (-433.0, -346.4, -259.8, -173.2, -86.6, 0.0, 86.6, 173.2, 259.8)
            plot!(p, [0, D], [y, y], color=:gray88, lw=0.4)
        end
        for o in sc.obstacles
            vs = vcat(o.verts, [o.verts[1]])
            plot!(p, first.(vs), last.(vs), seriestype=:shape, fillcolor=:gray55,
                  fillalpha=0.75, linecolor=:gray30, lw=0.6)
        end
        plot!(p, [0, D], [0, 0], color=:dodgerblue, ls=:dash, lw=1)
        scatter!(p, first.([(lm.x, lm.y) for lm in sc.landmarks]),
                 last.([(lm.x, lm.y) for lm in sc.landmarks]),
                 marker=:star5, ms=6, color=:darkorange, markerstrokewidth=0)
        scatter!(p, [0.0, D], [0.0, 0.0], ms=4, color=:seagreen, markerstrokewidth=0)
        push!(panels, p)
    end
    rows = ceil(Int, n / 2)
    out = joinpath(outdir, "preview.png")
    savefig(plot(panels..., layout=(rows, 2), size=(1400, 230 * rows)), out)
    println("  → $(out)")
    println("  orange stars = landmarks, gray = obstacles, blue dashed = direct route,")
    println("  green = start/goal, faint lines = reachable hex rows.")
end
