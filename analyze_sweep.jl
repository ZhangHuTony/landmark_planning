# Post-sweep analysis in the format of
# constraint_sweep/2026-08-12_13-51-20_RECOVERED/ANALYSIS.md, so a new sweep can be
# read against that one table-for-table.
#
#   julia analyze_sweep.jl <tag> [reference_method]
#
# Produces, for hexspline_cl against each baseline:
#   1. budget-exhaustion diagnostic   2. per-scenario deepest level solved
#   3. paired head-to-head            4. paired length ratio on runs BOTH solved
#
# The paired tables are the point. Per-method means in summary.csv are taken over
# DIFFERENT success subsets, so they are not comparable to each other; these compare
# only (scenario, pct) cells where both methods ran.

const CAP = 1_000_000    # astar_iteration_limit
const UNSOLVED = 110     # score for "never solved", one rung above the top

splitcsv(line) = split(chomp(line), ',')

function read_table(path)
    isfile(path) || return Dict{String,String}[]
    lines = readlines(path)
    isempty(lines) && return Dict{String,String}[]
    hdr = splitcsv(lines[1])
    [Dict(zip(hdr, splitcsv(l))) for l in lines[2:end] if !isempty(chomp(l))]
end

num(r, k) = (v = get(r, k, ""); isempty(v) ? nothing : tryparse(Float64, v))
istrue(r, k) = lowercase(get(r, k, "")) == "true"
mean(v) = isempty(v) ? NaN : sum(v) / length(v)
med(v) = isempty(v) ? NaN : sort(v)[cld(length(v), 2)]
r4(x) = isnan(x) ? "-" : string(round(x, digits=4))

length(ARGS) >= 1 || error("usage: julia analyze_sweep.jl <tag> [reference_method]")
tag = ARGS[1]
ref = length(ARGS) >= 2 ? ARGS[2] : "hexspline_cl"
root = joinpath(@__DIR__, "constraint_sweep", tag)
isdir(root) || error("no sweep at $root")

methods = sort([basename(d) for d in readdir(root; join=true) if
                isdir(d) && isfile(joinpath(d, "trials.csv"))])
trials = Dict(m => read_table(joinpath(root, m, "trials.csv")) for m in methods)
scens = read_table(joinpath(root, "scenarios.csv"))
# Only scenarios that actually entered the ladder; a rejected draw has no L_ref.
used = [s for s in scens if !isempty(get(s, "L_ref", "")) && isempty(get(s, "reject_reason", ""))]
sids = sort(unique([s["scenario_id"] for s in used]))

# (scenario, pct) -> row, per method. `not_run_*` rows are early-stop bookkeeping.
cell = Dict(m => Dict((r["scenario_id"], parse(Int, r["pct"])) => r
                      for r in trials[m] if !startswith(get(r, "fail_reason", ""), "not_run"))
            for m in methods)
pcts = sort(unique([p for m in methods for (_, p) in keys(cell[m])]), rev=true)

println("# Analysis — sweep $tag\n")
println("Scenarios: $(length(sids)) used of $(length(scens)) drawn " *
        "($(count(s -> !isempty(get(s, "reject_reason", "")), scens)) rejected). " *
        "Methods: $(join(methods, ", ")).\n")

# ── 1. Budget exhaustion ─────────────────────────────────────────────────────
println("## 1. Budget exhaustion ($(ref))\n")
rrows = collect(values(cell[ref]))
its = [(r, num(r, "astar_iterations")) for r in rrows]
capped = [r for (r, i) in its if i !== nothing && i >= CAP]
fails = [r for r in rrows if !istrue(r, "success")]
capped_fails = [r for r in fails if (i = num(r, "astar_iterations")) !== nothing && i >= CAP]
println("| quantity | value |"); println("|---|---|")
println("| runs executed | $(length(rrows)) |")
println("| …hit the $(CAP)-expansion cap | $(length(capped)) ($(round(100*length(capped)/max(1,length(rrows)), digits=1))%) |")
println("| failures | $(length(fails)) |")
println("| …of which capped | $(length(capped_fails)) ($(isempty(fails) ? "-" : string(round(100*length(capped_fails)/length(fails), digits=1)))%) |")
println()
for m in methods
    fr = Dict{String,Int}()
    for r in trials[m]
        istrue(r, "success") && continue
        k = get(r, "fail_reason", ""); fr[k] = get(fr, k, 0) + 1
    end
    println("- **$m** failures: ", isempty(fr) ? "none" :
            join(["$k $v" for (k, v) in sort(collect(fr); by=x -> -x[2])], ", "))
end

# ── 2. Deepest level solved, per scenario ────────────────────────────────────
deepest = Dict(m => Dict{String,Int}() for m in methods)
for m in methods, sid in sids
    solved = [p for p in pcts if haskey(cell[m], (sid, p)) && istrue(cell[m][(sid, p)], "success")]
    isempty(solved) || (deepest[m][sid] = minimum(solved))
end
println("\n## 2. Deepest constraint level solved (`-` = never)\n")
println("CENSORED AT THE LADDER FLOOR ($(minimum(pcts))%): a method showing $(minimum(pcts)) " *
        "was still succeeding when the ladder ran out, so its true depth is unknown " *
        "and the mean below is a LOWER bound. Compare depths only against a sweep " *
        "with the same floor.\n")
println("| scen | ", join(methods, " | "), " |")
println("|---", repeat("|---", length(methods)), "|")
for sid in sids
    println("| $sid | ", join([haskey(deepest[m], sid) ? string(deepest[m][sid]) : "–"
                               for m in methods], " | "), " |")
end
println()
for m in methods
    d = collect(values(deepest[m]))
    println("- **$m**: solved $(length(d))/$(length(sids)), mean deepest $(isempty(d) ? "-" : string(round(mean(Float64.(d)), digits=1)))")
end

# ── 3. Paired head-to-head, unsolved scored as UNSOLVED ──────────────────────
println("\n## 3. Paired head-to-head vs $(ref) (unsolved = $UNSOLVED)\n")
println("| comparison | $(ref) deeper | baseline deeper | tie | mean(base − $(ref)) |")
println("|---|---|---|---|---|")
for m in methods
    m == ref && continue
    rw, bw, tie, deltas = 0, 0, 0, Float64[]
    for sid in sids
        a = get(deepest[ref], sid, UNSOLVED); b = get(deepest[m], sid, UNSOLVED)
        a < b ? (rw += 1) : b < a ? (bw += 1) : (tie += 1)
        push!(deltas, b - a)
    end
    println("| vs **$m** | $rw | $bw | $tie | $(round(mean(deltas), digits=1)) |")
end

# ── 4. Paired length ratio on runs BOTH solved ───────────────────────────────
for m in methods
    m == ref && continue
    println("\n## 4. Paired length ratio, $(ref) vs $m (both solved)\n")
    println("| pct | n | $(ref) | $m | $(ref) shorter |")
    println("|---|---|---|---|---|")
    allr, allb, allwin, alln = Float64[], Float64[], 0, 0
    for p in pcts
        rs, bs, win = Float64[], Float64[], 0
        for sid in sids
            ra = get(cell[ref], (sid, p), nothing); rb = get(cell[m], (sid, p), nothing)
            (ra === nothing || rb === nothing) && continue
            (istrue(ra, "success") && istrue(rb, "success")) || continue
            la = num(ra, "length_ratio"); lb = num(rb, "length_ratio")
            (la === nothing || lb === nothing) && continue
            push!(rs, la); push!(bs, lb); la < lb && (win += 1)
        end
        isempty(rs) && continue
        println("| $p | $(length(rs)) | $(r4(mean(rs))) | $(r4(mean(bs))) | $win/$(length(rs)) |")
        append!(allr, rs); append!(allb, bs); allwin += win; alln += length(rs)
    end
    if alln > 0
        println("| **all** | **$alln** | **$(r4(mean(allr)))** | **$(r4(mean(allb)))** | " *
                "**$allwin/$alln ($(round(100*allwin/alln, digits=0))%)** |")
        d = allb .- allr
        println("\nPaired delta ($(ref) − $m): mean $(r4(-mean(d))), median $(r4(-med(d))).")
    end
end
println()
