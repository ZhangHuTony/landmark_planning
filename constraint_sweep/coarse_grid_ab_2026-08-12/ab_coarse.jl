# ==========================================================================
# A/B: does coarsening the hex grid recover the four provably-feasible p100
# failures (s005, s012, s029, s030) that hexspline_cl reported no_solution on?
# ==========================================================================
# The scenario GEOMETRY is pinned via `landmark_scenario: manual` (dumped from
# generate_scenario at hex_width_m=100), so the only variable across arms is
# graph resolution. Regenerating from `random` would NOT be an A/B: the draw
# quantises goal_dist in hex COLUMNS and LM_BAND_INNER = hex_width + visibility
# + 30, so a different hex_width yields different geometry.
#
# Every physical / sensor / constraint knob is held FIXED, comm_range included
# (its 2x hex_width relation is deliberate but re-tuning it would move the
# feasible set and contaminate the comparison).
#
# Arms per scenario:
#   w100_a2  hex_width=100, 2 agents, thr = U_ref   -- reproduces the sweep row
#   wCOARSE_a2  coarse grid, 2 agents, thr = U_ref  -- the question
#   wCOARSE_ref coarse grid, 1 agent,  thr = 1e9    -- coarse L_ref / U_ref
# Coarse width is chosen so goal_dist stays an integer number of columns
# (D=800 -> 160, D=700 -> 140): the goal must land on a cell centre or the
# discrete A* certifies at the goal CELL while the spline runs to the true goal.
using Printf, Dates

const _ROOT = "/home/tony-zhang/Research/landmark_planning"
const MC    = joinpath(_ROOT, "config", "mc")
const OUT   = joinpath(@__DIR__, "ab")
const GEOM  = joinpath(@__DIR__, "geometry.txt")

# U_ref / L_ref from constraint_sweep/2026-08-12_13-51-20/scenarios.csv (w=100).
const CASES = Dict(
    "s005" => (D=800.0, coarse=160.0, U_ref=6.275937249565342, L_ref=954.529),
    "s012" => (D=800.0, coarse=160.0, U_ref=8.666140311810961, L_ref=988.224),
    "s029" => (D=800.0, coarse=160.0, U_ref=3.9848691333907107, L_ref=989.700),
    "s030" => (D=700.0, coarse=140.0, U_ref=8.214047735387338, L_ref=852.675))

geom = Dict{String,Dict{String,String}}()
for ln in eachline(GEOM)
    isempty(strip(ln)) && continue
    sid, k, v = split(ln, '\t'; limit=3)
    get!(geom, String(sid), Dict{String,String}())[String(k)] = String(v)
end

# Verbatim from run_constraint_sweep.jl: an override is rewritten in whichever
# file already DEFINES the key (astar_iteration_limit lives in hexspline_cl.yaml
# and would be shadowed if written into main.yaml), else appended to main.yaml.
function write_run_config(dest::String, overrides::Dict{String,String})
    mkpath(dest)
    files = sort(filter(f -> endswith(f, ".yaml") && f != "sweep.yaml", readdir(MC)))
    contents = Dict(f => readlines(joinpath(MC, f)) for f in files)
    for (k, v) in overrides
        placed = false
        pat = Regex("^\\s*" * k * "\\s*:")
        for f in files, (i, ln) in enumerate(contents[f])
            (startswith(strip(ln), "#") || match(pat, ln) === nothing) && continue
            contents[f][i] = "$(k): $(v)"; placed = true
        end
        placed || push!(contents["main.yaml"], "$(k): $(v)")
    end
    for f in files
        write(joinpath(dest, f), join(contents[f], "\n") * "\n")
    end
    return dest
end

function run_one(cfg_dir, outdir, logfile, timeout_s)
    mkpath(outdir); mkpath(dirname(logfile))
    env = copy(ENV); env["OUTPUT_DIR"] = outdir
    killed = Ref(false); t0 = time(); rc = -1
    open(logfile, "w") do io
        p = run(pipeline(setenv(`$(Base.julia_cmd()) $(joinpath(_ROOT, "generate_plan.jl")) $(cfg_dir)`, env);
                         stdout=io, stderr=io); wait=false)
        timer = Timer(_ -> (process_running(p) && (killed[] = true; kill(p, Base.SIGKILL))), float(timeout_s))
        try wait(p) finally close(timer) end
        rc = p.exitcode != 0 ? p.exitcode : (p.termsignal != 0 ? 128 + p.termsignal : 0)
    end
    return (rc=rc, killed=killed[], wall=time() - t0)
end

function flatten!(outdir, algo)
    src = joinpath(outdir, algo); isdir(src) || return
    for f in readdir(src); mv(joinpath(src, f), joinpath(outdir, f); force=true) end
    rm(src)
end

function parse_results(path)
    d = Dict{String,String}(); isfile(path) || return d
    for ln in eachline(path)
        s = strip(ln)
        (isempty(s) || startswith(s, "#") || startswith(s, "-")) && continue
        i = findfirst(':', s); i === nothing && continue
        v = strip(s[nextind(s, i):end]); isempty(v) && continue
        d[String(strip(s[1:prevind(s, i)]))] = String(v)
    end
    return d
end
getnum(d, k) = (v = get(d, k, "null"); v == "null" ? nothing : tryparse(Float64, v))

const SLOTS = Base.Semaphore(6)
rows = Vector{Dict{String,String}}()
lk = ReentrantLock()

jobs = NamedTuple[]
for (sid, c) in sort(collect(CASES); by=first)
    g = geom[sid]
    base = Dict("landmark_scenario" => "manual",
                "start" => g["start"], "goal" => g["goal"],
                "landmarks" => g["landmarks"], "obstacles" => g["obstacles"],
                "algorithms" => "hexspline_cl",
                "emit_figures" => "true", "emit_csv" => "false")
    push!(jobs, (sid=sid, arm="w100_a2",   w=100.0,    na=2, thr=c.U_ref, base=base))
    push!(jobs, (sid=sid, arm="wC_a2",     w=c.coarse, na=2, thr=c.U_ref, base=base))
    push!(jobs, (sid=sid, arm="wC_ref",    w=c.coarse, na=1, thr=1.0e9,   base=base))
end

println("── coarsened-grid A/B, $(length(jobs)) runs ──")
@sync for j in jobs
    @async Base.acquire(SLOTS) do
        rundir = joinpath(OUT, "$(j.sid)_$(j.arm)")
        # Resume: a run that already produced results.yaml is not re-executed (the
        # first pass was killed with four controls still in flight). wall.txt is
        # written beside it because wall clock is not recoverable from results.yaml.
        if isfile(joinpath(rundir, "results.yaml"))
            w = tryparse(Float64, isfile(joinpath(rundir, "wall.txt")) ?
                                  strip(read(joinpath(rundir, "wall.txt"), String)) : "")
            r = (rc=0, killed=false, wall=something(w, NaN))
        else
            ov = merge(j.base, Dict("hex_width_m" => string(j.w),
                                    "num_agents" => string(j.na),
                                    "unc_radius_threshold" => @sprintf("%.10g", j.thr)))
            write_run_config(joinpath(rundir, "_cfg"), ov)
            r = run_one(joinpath(rundir, "_cfg"), rundir, joinpath(rundir, "run.log"), 900)
            flatten!(rundir, "hexspline_cl")
            write(joinpath(rundir, "wall.txt"), @sprintf("%.1f", r.wall))
        end
        res = parse_results(joinpath(rundir, "results.yaml"))
        logtxt = isfile(joinpath(rundir, "run.log")) ? read(joinpath(rundir, "run.log"), String) : ""
        nodes = (m = match(r"Hex graph: (\d+) nodes", logtxt); m === nothing ? "" : m[1])
        L = getnum(res, "primary_length"); U = getnum(res, "primary_unc")
        why = r.killed ? "timeout" :
              r.rc != 0 ? "error" :
              (L === nothing || !isfinite(L)) ? "no_solution" :
              get(res, "refinement_status", "") == "recovery_failed" ? "recovery_failed" :
              (U === nothing || !isfinite(U)) ? "no_solution" :
              U > j.thr + 1e-6 ? "unc_violated" :
              occursin("real obstacle collision", logtxt) ? "obstacle_breach" : ""
        row = merge(res, Dict("scenario_id" => j.sid, "arm" => j.arm,
                              "hex_width_m" => string(j.w), "num_agents" => string(j.na),
                              "threshold" => @sprintf("%.6f", j.thr), "nodes" => nodes,
                              "success" => string(isempty(why)), "fail_reason" => why,
                              "wall_s" => @sprintf("%.1f", r.wall),
                              "length_ratio" => (isempty(why) && L !== nothing) ?
                                                @sprintf("%.4f", L / CASES[j.sid].L_ref) : ""))
        lock(lk) do
            push!(rows, row)
            @printf("  %-5s %-8s w=%-5.0f a=%d nodes=%-5s %-16s it=%-9s len=%-9s wall=%.0fs\n",
                    j.sid, j.arm, j.w, j.na, nodes,
                    isempty(why) ? "ok" : "FAIL $(why)",
                    get(res, "astar_iterations", "-"),
                    L === nothing ? "-" : @sprintf("%.1f", L), r.wall)
        end
    end
end

core = ["scenario_id","arm","hex_width_m","num_agents","nodes","threshold","success",
        "fail_reason","astar_iterations","primary_length","primary_unc","primary_disc_unc",
        "length_ratio","refinement_status","wall_s"]
extras = sort(collect(setdiff(union((Set(keys(r)) for r in rows)...), Set(core))))
csvq(s) = (occursin(',', s) || occursin('"', s)) ? "\"" * replace(s, "\"" => "\"\"") * "\"" : String(s)
open(joinpath(OUT, "ab.csv"), "w") do io
    println(io, join(vcat(core, extras), ","))
    for r in sort(rows; by = r -> (r["scenario_id"], r["arm"]))
        println(io, join((csvq(get(r, c, "")) for c in vcat(core, extras)), ","))
    end
end
println("wrote $(joinpath(OUT, "ab.csv"))")
