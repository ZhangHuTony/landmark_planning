# Obstacle-avoidance robustness harness.
#
# Runs the hexspline_cl planner across every landmark scenario × a set of
# obstacle counts (default 5, 10, 20), driving each run purely through the
# SCENARIO / OBSTACLES / OUTPUT_DIR env overrides (no config edits). For each
# run it extracts the two robustness signals the pipeline already emits:
#   1. discrete feasibility  — did joint A* find a path around the obstacles?
#   2. continuous clearance   — the spec-E re-verification per refined seed
#      ("all MINVO segments clear ✓" vs. "N segment/face violation(s)" + depth).
#
# Obstacles are random axis-aligned rectangles (always convex) placed in the
# corridor, non-overlapping, clear of start(0,0)/goal(1000,0). Generation is
# seeded per (scenario,count) so a rerun reproduces the same fields.
#
# Usage:
#   julia test_obstacle_robustness.jl              # run the full sweep
#   julia test_obstacle_robustness.jl --selfcheck  # validate the generator only

using Random, Printf

const SCENARIOS = ["single", "dual", "clustered", "shoreline"]
# Override with e.g. ROBUSTNESS_COUNTS=20 to run only the 20-obstacle cases.
const COUNTS    = haskey(ENV, "ROBUSTNESS_COUNTS") ?
    parse.(Int, split(ENV["ROBUSTNESS_COUNTS"], ',')) : [5, 10, 20]
const OUTROOT   = get(ENV, "ROBUSTNESS_OUTROOT", joinpath(@__DIR__, "results", "obstacle_robustness"))

# n random non-overlapping axis-aligned rectangles in the routing corridor.
# Returned as the config `obstacles:` string (CCW verts, polygons sep by '|').
function gen_obstacles(n::Int, rng::AbstractRNG)
    rects = NTuple{4,Float64}[]   # (x0, y0, x1, y1)
    tries = 0
    while length(rects) < n && tries < 20000
        tries += 1
        w  = 25 + 30*rand(rng); h = 25 + 30*rand(rng)
        cx = 120 + 760*rand(rng); cy = -170 + 340*rand(rng)
        x0 = cx - w/2; x1 = cx + w/2; y0 = cy - h/2; y1 = cy + h/2
        (hypot(cx, cy) < 90 || hypot(cx-1000, cy) < 90) && continue        # clear endpoints
        any(r -> !(x1+12 < r[1] || x0-12 > r[3] || y1+12 < r[2] || y0-12 > r[4]), rects) && continue  # non-overlap (12 m gap)
        push!(rects, (x0, y0, x1, y1))
    end
    length(rects) == n || @warn "placed only $(length(rects))/$n obstacles (corridor saturated)"
    obs_str = join(["$(x0),$(y0); $(x1),$(y0); $(x1),$(y1); $(x0),$(y1)" for (x0,y0,x1,y1) in rects], " | ")
    return obs_str, length(rects)
end

# Pull the robustness signals out of one run's captured stdout + results.yaml.
function parse_run(logfile::String, outdir::String)
    txt = read(logfile, String)
    no_path  = occursin("No feasible path", txt)
    # New re-verification wording: "mean path clears all obstacles ✓" (clear, with
    # an optional "(N hull row(s) conservatively flagged…)" note) vs. "M real
    # obstacle collision(s):" + "agent A obstacle O depth=D m" lines. depth= now
    # appears only on REAL collisions, so maxdepth is a true collision depth.
    clears   = count(_ -> true, eachmatch(r"mean path clears all obstacles", txt))
    breaches = count(_ -> true, eachmatch(r"real obstacle collision", txt))
    depths   = [parse(Float64, m.captures[1]) for m in eachmatch(r"depth=([-\d.]+)", txt)]
    maxdepth = isempty(depths) ? 0.0 : maximum(depths)
    hullnotes = [parse(Int, m.captures[1]) for m in eachmatch(r"\((\d+) hull row", txt)]
    hull_flagged = isempty(hullnotes) ? 0 : maximum(hullnotes)
    # Main solution length / terminal uncertainties from results.yaml, if written.
    len = "—"; unc = "—"
    ry = joinpath(outdir, "hexspline_cl", "results.yaml")
    if isfile(ry)
        for line in eachline(ry)
            occursin("continuous_primary_length:", line) && len == "—" && (len = strip(split(line, ':')[2]))
            occursin("continuous_uncertainties:",  line) && unc == "—" && (unc = strip(split(line, ':', limit=2)[2]))
        end
    end
    return (; no_path, clears, breaches, maxdepth, hull_flagged, len, unc)
end

function run_sweep()
    mkpath(OUTROOT)
    rows = String[]
    push!(rows, "| scenario | obstacles | placed | discrete path | seeds clear | seeds w/ real collision | max real depth (m) | hull rows flagged | main cont. len | main cont. unc |")
    push!(rows, "|---|---|---|---|---|---|---|---|---|---|")
    for sc in SCENARIOS, n in COUNTS
        tag = "$(sc)_$(n)"
        rng = MersenneTwister(hash((sc, n)))
        obs, placed = gen_obstacles(n, rng)
        outdir = joinpath(OUTROOT, tag)
        logf   = joinpath(OUTROOT, "$(tag).log")
        println("▶ $(tag): $(placed) obstacles → running planner …")
        env = copy(ENV)
        env["SCENARIO"]   = sc
        env["OBSTACLES"]  = obs
        env["OUTPUT_DIR"] = outdir
        ok = true
        try
            open(logf, "w") do io
                run(pipeline(setenv(`$(Base.julia_cmd()) $(joinpath(@__DIR__, "generate_plan.jl"))`, env);
                             stdout=io, stderr=io))
            end
        catch e
            ok = false
            @warn "run $(tag) exited nonzero" exception=e
        end
        r = parse_run(logf, outdir)
        dp = !ok ? "ERROR" : (r.no_path ? "none" : "yes")
        push!(rows, "| $(sc) | $(n) | $(placed) | $(dp) | $(r.clears) | $(r.breaches) | $(@sprintf("%.2f", r.maxdepth)) | $(r.hull_flagged) | $(r.len) | $(r.unc) |")
        println("   ↳ discrete=$(dp)  clear=$(r.clears)  real_collision=$(r.breaches)  max_real_depth=$(@sprintf("%.2f", r.maxdepth))")
    end
    report = join(rows, "\n") * "\n"
    write(joinpath(OUTROOT, "SUMMARY.md"), report)
    println("\n" * report)
    println("Summary → $(joinpath(OUTROOT, "SUMMARY.md"))")
end

# ── self-check: generator produces the requested count, convex, non-overlapping ──
function selfcheck()
    for n in COUNTS
        obs, placed = gen_obstacles(n, MersenneTwister(hash(("chk", n))))
        @assert placed == n "expected $n obstacles, placed $placed"
        polys = split(obs, '|')
        @assert length(polys) == n
        # every polygon: 4 verts, axis-aligned rectangle (⇒ convex), inside corridor
        for p in polys
            verts = [parse.(Float64, split(strip(v), ',')) for v in split(strip(p), ';')]
            @assert length(verts) == 4
            xs = getindex.(verts, 1); ys = getindex.(verts, 2)
            @assert all(100 .< xs .< 900) && all(-200 .< ys .< 200)
        end
    end
    println("selfcheck ✓  (generator: exact count, 4-vertex rectangles, in-corridor)")
end

# Only auto-run when executed directly (so check_true_collisions.jl can reuse
# gen_obstacles / SCENARIOS / COUNTS by including this file).
if abspath(PROGRAM_FILE) == @__FILE__
    "--selfcheck" in ARGS ? selfcheck() : run_sweep()
end
