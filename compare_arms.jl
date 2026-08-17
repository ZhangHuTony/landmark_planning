# Compare probe-sweep ARMS: success rate per rung, budget exhaustion, and the
# search-shape counters ASTAR_STATS added in Phase 1.
#
#   julia compare_arms.jl probe_A3_sound probe_A4_heuristic ...
#
# Reads <arm>/<method>/trials.csv, which run_constraint_sweep.jl writes with a
# per-algorithm column set — so every field is looked up BY NAME, never by index.
# Means are over successful runs only, printed next to n so a small denominator is
# visible rather than implied.

const CAP = 1_000_000   # astar_iteration_limit; a run at exactly this is budget-limited

splitcsv(line) = split(chomp(line), ',')

function read_trials(path)
    isfile(path) || return Dict{String,String}[]
    lines = readlines(path)
    isempty(lines) && return Dict{String,String}[]
    hdr = splitcsv(lines[1])
    [Dict(zip(hdr, splitcsv(l))) for l in lines[2:end] if !isempty(chomp(l))]
end

num(r, k) = (v = get(r, k, ""); isempty(v) ? nothing : tryparse(Float64, v))
istrue(r, k) = lowercase(get(r, k, "")) == "true"

function arm_report(arm, method)
    rows = read_trials(joinpath(@__DIR__, "constraint_sweep", arm, method, "trials.csv"))
    isempty(rows) && (println("  (no trials.csv for $method)"); return)

    # `not_run_*` rows are bookkeeping for early stops, not attempts.
    real = [r for r in rows if !startswith(get(r, "fail_reason", ""), "not_run")]
    pcts = sort(unique(Int[parse(Int, r["pct"]) for r in real]), rev=true)

    println("  $method — $(length(real)) real runs of $(length(rows)) rows")
    println("    pct   succ/n   rate    mean_iters  capped  mean_gen   mean_push  prune%  mean_wall")
    for p in pcts
        sel = [r for r in real if parse(Int, r["pct"]) == p]
        ok  = [r for r in sel if istrue(r, "success")]
        its = filter(!isnothing, [num(r, "astar_iterations") for r in sel])
        gen = filter(!isnothing, [num(r, "astar_generated") for r in sel])
        psh = filter(!isnothing, [num(r, "astar_pushed") for r in sel])
        prn = filter(!isnothing, [num(r, "astar_dominance_prunes") for r in sel])
        wal = filter(!isnothing, [num(r, "wall_s") for r in sel])
        capped = count(x -> x >= CAP, its)
        mean(v) = isempty(v) ? NaN : sum(v) / length(v)
        # Each planner writes its own column set — `sequential` has no astar_* counters
        # at all — so every aggregate here has to survive an empty column.
        ifmt(v, w) = lpad(isnan(v) ? "-" : string(round(Int, v)), w)
        ffmt(v, w, d) = lpad(isnan(v) ? "-" : string(round(v, digits=d)), w)
        prate = (isempty(gen) || sum(gen) == 0) ? NaN : 100 * sum(prn) / sum(gen)
        println("    $(lpad(p,3))   $(lpad(length(ok),2))/$(rpad(length(sel),2))  " *
                "$(rpad(round(length(ok)/max(1,length(sel)), digits=3),6))  " *
                "$(ifmt(mean(its),10))  $(lpad(capped,6))  " *
                "$(ifmt(mean(gen),9))  $(ifmt(mean(psh),9))  " *
                "$(ffmt(prate,6,1))  $(ffmt(mean(wal),9,1))")
    end

    wins = [r for r in real if istrue(r, "success")]
    lr = filter(!isnothing, [num(r, "length_ratio") for r in wins])
    mlr = isempty(lr) ? "-" : string(round(sum(lr) / length(lr), digits=4))
    println("    overall: success $(length(wins))/$(length(real)), " *
            "mean length_ratio $(mlr) (n=$(length(lr)))")
end

isempty(ARGS) && error("usage: julia compare_arms.jl <arm_tag> [<arm_tag> ...]")
for arm in ARGS
    println("\n══ $arm ══")
    for method in ("hexspline_cl", "sequential")
        isdir(joinpath(@__DIR__, "constraint_sweep", arm, method)) && arm_report(arm, method)
    end
end
println()
