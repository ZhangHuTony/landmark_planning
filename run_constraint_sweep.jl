# ==========================================================================
# run_constraint_sweep.jl — Monte Carlo sweep over the localization constraint
# ==========================================================================
# Measures what a tightening uncertainty constraint costs in path length.
# Per randomly generated scenario:
#
#   1. REFERENCE: hexspline_cl, 1 agent, unc_radius_threshold = 1e9 (i.e. no
#      constraint). A* accepts the first goal pop, so this is the SHORTEST
#      path. Yields L_ref and U_ref = the uncertainty of that path's DISCRETE
#      seed — the quantity unc_radius_threshold actually gates (see the anchor
#      note in main()). The refined value is recorded alongside as U_ref_cont;
#      U_ref > U_ref_cont always, and pct=100 means "constrain the search to
#      exactly what the unconstrained reference's own search certified".
#   2. Walk the constraint down: threshold = pct/100 * U_ref for
#      pct = 100, 90, 80 ... and at each level run every method still in play.
#   3. Record primary_length / L_ref and wall_s / wall_ref, plus whether a
#      solution was found.
#
# Reported per (method, pct): mean length ratio over successes, mean wall-clock
# ratio over successes, and n_success / n_total.
#
# ── What does NOT count ──
# n_scenarios counts scenarios whose REFERENCE succeeded; the sweep keeps drawing
# until it has that many. A scenario with no reference has no L_ref to normalise
# against and contributes no rows, so counting it just shrinks the sample.
#
# ── Two early stops, both recorded rather than omitted ──
# `pct_patience`    — every method failed at N consecutive levels: abandon the
#                     scenario, remaining levels logged `not_run_below_stop`.
# `method_patience` — ONE method failed N times running: drop just that method
#                     for this scenario even while others keep succeeding,
#                     remaining levels logged `not_run_after_fail`.
# Both rest on success being monotone in the constraint, which holds for ~99% of
# method×scenario pairs (see method_patience in sweep.yaml for the exceptions and
# why they exist). Skipped levels are written as explicit failure rows so
# summarize() keeps the full scenario count in its denominator.
#
# ── Why subprocesses ──
# src/config.jl freezes every knob into a module-level `const` at include time,
# so a single process can hold exactly one configuration. Each run therefore
# gets its own generated config directory and its own `julia generate_plan.jl`
# process. Same pattern as test_obstacle_robustness.jl, plus a timeout.
#
# ── Why this works for algorithms that do not exist yet ──
# The harness knows exactly TWO planner-specific things about a METHOD UNDER
# TEST, both keys under `main:` in results.yaml: `primary_length` and
# `primary_unc`. (The reference run is additionally read for
# `primary_disc_unc`, but the reference is hardcoded to hexspline_cl anyway —
# it is the yardstick, not a subject.) Everything else
# it finds in that file is passed through verbatim as extra CSV columns, so
# hexspline_cl contributes astar_iterations/refinement_status, straight_line
# contributes almost nothing, and a future planner contributes whatever it
# writes. There is no `if algo == ...` anywhere below. Because the columns
# therefore differ per algorithm, trials.csv lives under each method's own
# folder; only summary.csv and scenarios.csv (built from the two contract keys
# alone) are shared at the sweep root.
#
# Usage:
#   julia run_constraint_sweep.jl                    # new sweep, timestamped tag
#   julia run_constraint_sweep.jl --tag mysweep      # named; RESUMES if it exists
#   julia run_constraint_sweep.jl --tag mysweep --summarize-only
#   julia run_constraint_sweep.jl --sweep path/to/other_sweep.yaml
# ==========================================================================

using Plots, Printf, Random, Dates, Statistics

# src/config.jl consumes ARGS[1] as its config DIRECTORY at include time, so our
# own flags must come off ARGS first (same dance as mc_nees.jl:62). We point it
# at config/mc so UNC_FEAS_TOL below is the sweep's own tolerance.
const FLAGS = copy(ARGS)
empty!(ARGS)

const _ROOT = @__DIR__
const MC_CONFIG_DIR = joinpath(_ROOT, "config", "mc")
push!(ARGS, MC_CONFIG_DIR)
include(joinpath(_ROOT, "src", "config.jl"))   # load_yaml, CFG, UNC_FEAS_TOL

function flag(name::String, default)
    i = findfirst(==(name), FLAGS)
    (i === nothing || i == length(FLAGS)) && return default
    return FLAGS[i+1]
end

const SWEEP = load_yaml(flag("--sweep", joinpath(MC_CONFIG_DIR, "sweep.yaml")))
const TAG   = flag("--tag", Dates.format(now(), "yyyy-mm-dd_HH-MM-SS"))
const ROOT  = joinpath(_ROOT, String(SWEEP["outroot"]), TAG)

# Bounds concurrent planner subprocesses. Levels are sequential (the early stop
# depends on their outcome), so this only ever throttles methods within a level.
const SLOTS = Base.Semaphore(max(1, Int(SWEEP["workers"])))

# Categorical series colors, fixed order, never cycled: all 8 slots of a
# palette validated for colorblind separation (worst adjacent CVD ΔE 9.1,
# normal-vision 19.6 — unchanged from the prior 5-slot cut; the worst-adjacent
# pairs both fall inside slots 1-5). Three of these sit under 3:1 against a
# light surface, which obliges a non-color reading of the same data —
# summary.csv / SUMMARY.md are it. Marker shape is a second, redundant
# encoding so identity is never color-alone. 8 slots covers hexspline_cl,
# straight_cont, discrete_only, greedy, formation, sequential, clgbt with one
# spare; a 9th method cycles back to slot 1 (mod1 below) rather than erroring,
# but at that point it should get its own 9th slot instead — a cycled color
# silently makes two series indistinguishable in the plot, which is what
# happened to sequential/clgbt against hexspline_cl/straight_cont before this
# was extended from 5 to 8.
const SERIES_COLORS  = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
const SERIES_MARKERS = [:circle, :rect, :diamond, :utriangle, :star5, :dtriangle, :pentagon, :cross]

# ==========================================================================
# Methods:  "label@key=value,key=value | label@..."  (bare "algo" also works)
# ==========================================================================
struct Method
    label     :: String
    algorithm :: String
    overrides :: Dict{String,String}
end

function parse_methods(spec::AbstractString)
    ms = Method[]
    for entry in split(spec, '|')
        entry = strip(entry)
        isempty(entry) && continue
        label, kvs = occursin('@', entry) ? split(entry, '@'; limit=2) : (entry, "")
        label = String(strip(label))
        ov = Dict{String,String}()
        for kv in split(kvs, ',')
            kv = strip(kv)
            isempty(kv) && continue
            occursin('=', kv) || error("method `$label`: expected key=value, got `$kv`")
            k, v = split(kv, '='; limit=2)
            ov[String(strip(k))] = String(strip(v))
        end
        algo = String(get(ov, "algorithms", label))
        occursin(',', algo) && error("method `$label`: `algorithms` must name exactly ONE planner " *
                                     "(the sweep needs to know which results.yaml is its own), got `$algo`")
        ov["algorithms"] = algo
        push!(ms, Method(label, algo, ov))
    end
    isempty(ms) && error("no methods configured (config/mc/sweep.yaml: methods)")
    length(unique(m.label for m in ms)) == length(ms) ||
        error("method labels must be unique — they name the output folders")
    return ms
end

const METHODS = parse_methods(String(SWEEP["methods"]))

# Trivial-scenario screen. OFF by default, so an ablation sweep draws exactly the
# population it always did — see reject_trivial_straight in sweep.yaml for why
# baselines and ablations want different populations.
const REJECT_TRIVIAL = Bool(get(SWEEP, "reject_trivial_straight", false))
# Reuses the METHODS grammar so the screen is configured the same way a method is,
# and so it can be pointed at a different planner without touching this file.
const SCREEN = first(parse_methods(String(get(SWEEP, "reject_trivial_spec",
                     "straight_cont@algorithms=straight_cont,num_agents=2"))))

# ==========================================================================
# Config generation: copy config/mc/*.yaml (minus sweep.yaml), patch overrides
# ==========================================================================
# An override is rewritten in whichever file already DEFINES the key, and in
# every such file. That matters: astar_iteration_limit lives in
# hexspline_cl.yaml, so writing it into main.yaml would be silently shadowed at
# merge time (algorithm files merge last) and would trip load_config's
# shared-key warning. Keys defined nowhere are appended to main.yaml.
function write_run_config(dest::String, overrides::Dict{String,String})
    mkpath(dest)
    # Any sweep_*.yaml is harness config, never planner config: load_config only ever
    # reads main.yaml plus <algorithm>.yaml, so copying one in is inert rather than
    # wrong — but it lands in every run's _cfg/ and reads like it was used.
    files = sort(filter(f -> endswith(f, ".yaml") && !startswith(f, "sweep"), readdir(MC_CONFIG_DIR)))
    contents = Dict(f => readlines(joinpath(MC_CONFIG_DIR, f)) for f in files)
    for (k, v) in overrides
        placed = false
        pat = Regex("^\\s*" * k * "\\s*:")
        for f in files, (i, ln) in enumerate(contents[f])
            (startswith(strip(ln), "#") || match(pat, ln) === nothing) && continue
            contents[f][i] = "$(k): $(v)"
            placed = true
        end
        placed || push!(contents["main.yaml"], "$(k): $(v)")
    end
    for f in files
        write(joinpath(dest, f), join(contents[f], "\n") * "\n")
    end
    return dest
end

# ==========================================================================
# One planner subprocess, with a hard wall-clock kill
# ==========================================================================
function run_one(cfg_dir::String, outdir::String, logfile::String, timeout_s::Real)
    mkpath(outdir); mkpath(dirname(logfile))
    env = copy(ENV); env["OUTPUT_DIR"] = outdir
    killed = Ref(false)
    t0 = time()
    rc = -1
    open(logfile, "w") do io
        p = run(pipeline(setenv(`$(Base.julia_cmd()) $(joinpath(_ROOT, "generate_plan.jl")) $(cfg_dir)`, env);
                         stdout=io, stderr=io); wait=false)
        timer = Timer(_ -> (process_running(p) && (killed[] = true; kill(p, Base.SIGKILL))), float(timeout_s))
        try
            wait(p)
        finally
            close(timer)
        end
        # A process killed by a SIGNAL reports exitcode 0, so exitcode alone would
        # read an OOM kill mid-sweep as a clean run that simply found nothing.
        rc = p.exitcode != 0 ? p.exitcode : (p.termsignal != 0 ? 128 + p.termsignal : 0)
    end
    return (rc=rc, killed=killed[], wall=time() - t0)
end

# generate_plan.jl writes <outdir>/<algo>/{results.yaml,figures,csv}. Lift that
# one directory up so a run folder is not <label>/<sid>_p090/<label>/figures.
function flatten_algo_dir!(outdir::String, algo::String)
    src = joinpath(outdir, algo)
    isdir(src) || return
    for f in readdir(src)
        mv(joinpath(src, f), joinpath(outdir, f); force=true)
    end
    rm(src)
end

# Every `key: scalar` line in results.yaml, flattened. Deliberately schema-free:
# whatever a planner chooses to report becomes a column. Block headers (`main:`,
# empty value) and list items are skipped.
function parse_results_yaml(path::String)
    d = Dict{String,String}()
    isfile(path) || return d
    for ln in eachline(path)
        s = strip(ln)
        (isempty(s) || startswith(s, "#") || startswith(s, "-")) && continue
        i = findfirst(':', s)
        i === nothing && continue
        v = strip(s[nextind(s, i):end])
        isempty(v) && continue
        d[String(strip(s[1:prevind(s, i)]))] = String(v)
    end
    return d
end

getnum(d, k) = (v = get(d, k, "null"); v == "null" ? nothing : tryparse(Float64, v))

# Success is decided from the two contract keys plus signals that are OPTIONAL:
# a planner that never writes refinement_status simply cannot fail that way.
function classify(r, res::Dict{String,String}, logtxt::String, threshold::Float64)
    r.killed  && return (false, "timeout")
    r.rc != 0 && return (false, "error")
    L = getnum(res, "primary_length")
    U = getnum(res, "primary_unc")
    (L === nothing || !isfinite(L)) && return (false, "no_solution")
    get(res, "refinement_status", "") == "recovery_failed" && return (false, "recovery_failed")
    (U === nothing || !isfinite(U)) && return (false, "no_solution")
    U > threshold + UNC_FEAS_TOL && return (false, "unc_violated")
    occursin("real obstacle collision", logtxt) && return (false, "obstacle_breach")
    return (true, "")
end

# ==========================================================================
# CSV (quote-aware: pass-through values like "[5.18, 5.80]" contain commas)
# ==========================================================================
csvq(s::AbstractString) = (occursin(',', s) || occursin('"', s)) ?
    "\"" * replace(s, "\"" => "\"\"") * "\"" : String(s)

function splitcsv(line::AbstractString)
    out = String[]; buf = IOBuffer(); inq = false
    chars = collect(line); i = 1
    while i <= length(chars)
        c = chars[i]
        if inq
            if c == '"'
                if i < length(chars) && chars[i+1] == '"'; print(buf, '"'); i += 1 else inq = false end
            else; print(buf, c) end
        elseif c == '"'; inq = true
        elseif c == ','; push!(out, String(take!(buf)))
        else; print(buf, c) end
        i += 1
    end
    push!(out, String(take!(buf)))
    return out
end

const CORE_COLS = ["scenario_id", "pct", "threshold", "method", "algorithm", "num_agents",
                   "success", "fail_reason", "primary_length", "primary_unc",
                   "length_ratio", "wall_s", "wall_ratio"]

function write_table(path::String, rows::Vector{Dict{String,String}}, core::Vector{String})
    extras = isempty(rows) ? String[] :
             sort(collect(setdiff(union((Set(keys(r)) for r in rows)...), Set(core))))
    cols = vcat(core, extras)
    open(path, "w") do io
        println(io, join(cols, ","))
        for r in rows
            println(io, join((csvq(get(r, c, "")) for c in cols), ","))
        end
    end
end

function read_table(path::String)
    rows = Dict{String,String}[]
    isfile(path) || return rows
    lines = readlines(path)
    isempty(lines) && return rows
    cols = splitcsv(lines[1])
    for ln in lines[2:end]
        isempty(strip(ln)) && continue
        v = splitcsv(ln)
        push!(rows, Dict(cols[i] => (i <= length(v) ? v[i] : "") for i in eachindex(cols)))
    end
    return rows
end

# ==========================================================================
# Aggregation — runs standalone over an existing sweep (--summarize-only)
# ==========================================================================
function summarize(root::String, methods::Vector{Method})
    out = Dict{String,String}[]
    for m in methods
        rows = read_table(joinpath(root, m.label, "trials.csv"))
        isempty(rows) && continue
        for pct in sort(unique(parse(Int, r["pct"]) for r in rows), rev=true)
            at = filter(r -> parse(Int, r["pct"]) == pct, rows)
            ok = filter(r -> r["success"] == "true", at)
            ratios = [parse(Float64, r["length_ratio"]) for r in ok if !isempty(r["length_ratio"])]
            # Time to a SOLUTION: successes only, so a method is not flattered by
            # its own fast failures. `not_run_after_fail` rows carry wall_s 0.0 and
            # are failures, so they never reach these.
            num(rs, k) = Float64[x for x in (tryparse(Float64, get(r, k, "")) for r in rs)
                                 if x !== nothing]
            walls  = num(ok, "wall_s")
            wratio = num(ok, "wall_ratio")
            push!(out, Dict(
                "method" => m.label, "pct" => string(pct),
                "n_total" => string(length(at)), "n_success" => string(length(ok)),
                "success_rate" => @sprintf("%.3f", length(ok) / length(at)),
                "mean_length_ratio"   => isempty(ratios) ? "" : @sprintf("%.4f", mean(ratios)),
                "median_length_ratio" => isempty(ratios) ? "" : @sprintf("%.4f", median(ratios)),
                "mean_wall_s"         => isempty(walls)  ? "" : @sprintf("%.1f", mean(walls)),
                "median_wall_s"       => isempty(walls)  ? "" : @sprintf("%.1f", median(walls)),
                "mean_wall_ratio"     => isempty(wratio) ? "" : @sprintf("%.3f", mean(wratio)),
                "median_wall_ratio"   => isempty(wratio) ? "" : @sprintf("%.3f", median(wratio))))
        end
    end
    cols = ["method", "pct", "n_total", "n_success", "success_rate",
            "mean_length_ratio", "median_length_ratio",
            "mean_wall_s", "median_wall_s", "mean_wall_ratio", "median_wall_ratio"]
    write_table(joinpath(root, "summary.csv"), out, cols)

    open(joinpath(root, "SUMMARY.md"), "w") do io
        println(io, "# Constraint sweep — $(basename(root))\n")
        println(io, "`length_ratio` = the method's primary path length divided by L_ref, the")
        println(io, "length of the unconstrained single-agent reference path for that same")
        println(io, "scenario. Constraint at level pct is `unc_radius_threshold = pct/100 * U_ref`,")
        println(io, "where U_ref is the uncertainty of that reference path's DISCRETE seed — the")
        println(io, "stage the threshold gates. scenarios.csv logs the refined U_ref_cont beside it.")
        println(io, "Means are over SUCCESSFUL runs only, so read them next to n_success/n_total.\n")
        println(io, "`wall_ratio` = this run's wall clock / the reference run's, the time cost in")
        println(io, "the same currency as length_ratio. Both are whole subprocesses, so each")
        println(io, "carries the same fixed overhead (Julia startup + graph build + refinement,")
        println(io, "~17.7 s on the 600-800 m band) — that floor largely divides out, but it does")
        println(io, "compress the ratio, so a method at 2.0x is doing far more than 2x the search.")
        println(io, "`fail_reason` distinguishes a real failure from `not_run_after_fail` (method")
        println(io, "dropped after method_patience failures) and `not_run_below_stop`.\n")
        println(io, "| method | constraint % | success | rate | mean ratio | median ratio | mean wall | mean wall ratio |")
        println(io, "|---|---|---|---|---|---|---|---|")
        dash(v) = isempty(v) ? "—" : v
        for r in out
            println(io, "| $(r["method"]) | $(r["pct"])% | $(r["n_success"])/$(r["n_total"]) | " *
                        "$(r["success_rate"]) | $(dash(r["mean_length_ratio"])) | " *
                        "$(dash(r["median_length_ratio"])) | " *
                        "$(isempty(r["mean_wall_s"]) ? "—" : r["mean_wall_s"] * " s") | " *
                        "$(dash(r["mean_wall_ratio"])) |")
        end
    end

    plot_summary(root, methods, out)
    return out
end

# Two stacked panels sharing one x-axis — never a second y-scale on one plot.
function plot_summary(root::String, methods::Vector{Method}, rows::Vector{Dict{String,String}})
    isempty(rows) && return
    # Both panels are pinned to the SAME x range and ticks. They are stacked to
    # be read together, and Plots would otherwise scale each to its own data —
    # the top panel stops early wherever every method has failed.
    all_pcts = sort(unique(parse(Int, r["pct"]) for r in rows), rev=true)
    xl = (minimum(all_pcts) - 2, maximum(all_pcts) + 2)
    top = plot(ylabel="mean length / L_ref", legend=:topleft, grid=:y, gridalpha=0.25,
               framestyle=:box, titlefontsize=10, xlims=xl, xticks=all_pcts,
               title="Path-length cost of the localization constraint")
    mid = plot(ylabel="success rate", legend=false, grid=:y, gridalpha=0.25,
               framestyle=:box, ylims=(-0.05, 1.05), xlims=xl, xticks=all_pcts)
    bot = plot(xlabel="constraint level (% of the unconstrained reference uncertainty U_ref)",
               ylabel="mean wall / reference", legend=false, grid=:y, gridalpha=0.25,
               framestyle=:box, xlims=xl, xticks=all_pcts)
    for (i, m) in enumerate(methods)
        mine = sort(filter(r -> r["method"] == m.label, rows), by = r -> parse(Int, r["pct"]))
        isempty(mine) && continue
        clr = SERIES_COLORS[mod1(i, length(SERIES_COLORS))]
        mrk = SERIES_MARKERS[mod1(i, length(SERIES_MARKERS))]
        by_pct = Dict(parse(Int, r["pct"]) => r for r in mine)
        pcts = sort(collect(keys(by_pct)))
        # NaN, not a dropped point: a level where every run failed has no ratio,
        # and joining across it would draw a line through data that isn't there.
        ratio(p) = (v = get(by_pct[p], "mean_length_ratio", ""); isempty(v) ? NaN : parse(Float64, v))
        wall(p) = (v = get(by_pct[p], "mean_wall_ratio", ""); isempty(v) ? NaN : parse(Float64, v))
        plot!(top, pcts, ratio.(pcts),
              color=clr, lw=2, marker=mrk, ms=5, markerstrokewidth=0, label=m.label)
        plot!(mid, pcts, [parse(Float64, by_pct[p]["success_rate"]) for p in pcts],
              color=clr, lw=2, marker=mrk, ms=5, markerstrokewidth=0, label=m.label)
        plot!(bot, pcts, wall.(pcts),
              color=clr, lw=2, marker=mrk, ms=5, markerstrokewidth=0, label=m.label)
    end
    hline!(top, [1.0], color=:gray70, ls=:dash, lw=1, label="reference (ratio 1.0)")
    hline!(bot, [1.0], color=:gray70, ls=:dash, lw=1, label=false)
    out = joinpath(root, "fig_length_ratio.png")
    savefig(plot(top, mid, bot, layout=grid(3, 1, heights=[0.44, 0.28, 0.28]), size=(900, 950),
                 xflip=true, left_margin=6Plots.mm, bottom_margin=5Plots.mm), out)
    println("  → $(out)")
end

# ==========================================================================
# Sweep
# ==========================================================================
function sample_scenario_params(seed::Int)
    # Drawn from a stream OFFSET from the geometry seed, so scenario size and
    # landmark placement are not correlated through a shared stream position.
    rng  = Random.MersenneTwister(seed + 1_000_000)
    # Drawn in hex COLUMNS, not metres, so the goal always lands on a y=0 cell
    # centre. build_hex_graph spaces that row's centres HEX_WIDTH_M apart with the
    # start on one of them; a goal between two columns is rounded to the nearer
    # cell, and the discrete A* then certifies its uncertainty at that CELL while
    # seed_control_points ships a spline extended to the true goal — up to half a
    # column of dead reckoning no discrete gate ever sees.
    dmin = Float64(SWEEP["goal_dist_min"]); dmax = Float64(SWEEP["goal_dist_max"])
    lo, hi = ceil(Int, dmin / HEX_WIDTH_M), floor(Int, dmax / HEX_WIDTH_M)
    hi >= lo || error("goal_dist range [$(dmin), $(dmax)] contains no multiple of " *
                      "hex_width_m ($(HEX_WIDTH_M)) — widen it or shrink the hexes")
    D = HEX_WIDTH_M * rand(rng, lo:hi)
    nl   = rand(rng, Int(SWEEP["n_landmarks_min"]):Int(SWEEP["n_landmarks_max"]))
    no   = rand(rng, Int(SWEEP["n_obstacles_min"]):Int(SWEEP["n_obstacles_max"]))
    return (D=D, nl=nl, no=no)
end

scenario_keys(seed, p) = Dict("landmark_scenario" => "random",
                              "scenario_seed" => string(seed),
                              "scenario_goal_dist" => string(p.D),
                              "scenario_n_landmarks" => string(p.nl),
                              "scenario_n_obstacles" => string(p.no))

function main()
    mkpath(ROOT)
    logio = open(joinpath(ROOT, "sweep.log"), "a")
    say(s) = (println(s); println(logio, s); flush(logio))

    if "--summarize-only" in FLAGS
        summarize(ROOT, METHODS)
        say("summarized $(ROOT)")
        return
    end

    save_figs = Bool(get(SWEEP, "save_figures", false))
    save_csv  = Bool(get(SWEEP, "save_csv", false))
    timeout   = Float64(SWEEP["run_timeout_s"])
    patience  = Int(SWEEP["pct_patience"])
    method_patience = Int(get(SWEEP, "method_patience", 1))
    pcts      = collect(Int(SWEEP["pct_start"]):-Int(SWEEP["pct_step"]):Int(SWEEP["pct_floor"]))

    # Resume: previously completed (scenario, pct) rows are kept and skipped.
    trials = Dict(m.label => read_table(joinpath(ROOT, m.label, "trials.csv")) for m in METHODS)
    done   = Dict(m.label => Set((r["scenario_id"], r["pct"]) for r in trials[m.label]) for m in METHODS)
    scen_rows = read_table(joinpath(ROOT, "scenarios.csv"))
    scen_done = Dict(r["scenario_id"] => r for r in scen_rows)
    lk = ReentrantLock()

    say("── constraint sweep $(TAG) ──")
    say("  methods: $(join((m.label for m in METHODS), ", "))")
    say("  ladder:  $(join(pcts, ", "))%   patience=$(patience)   " *
        "method_patience=$(method_patience)   timeout=$(Int(timeout))s")
    say("  scenarios: drawing until $(Int(SWEEP["n_scenarios"])) references succeed " *
        "(guard: $(Int(get(SWEEP, "max_scenario_draws", 4 * Int(SWEEP["n_scenarios"])))) draws)")
    say("  output:  $(ROOT)")

    # Draw until n_scenarios have a USABLE reference, rather than drawing exactly
    # n_scenarios and losing the ones whose reference failed. A failed reference
    # yields no L_ref/U_ref, so there is nothing to normalise against and the
    # scenario contributes no rows at all — counting it toward the total silently
    # shrank the sample (this sweep asked for 50 and measured 42). sid stays tied
    # to the DRAW index, not the accepted count, so `seed = base + i` is still
    # recoverable from a scenario_id and rejected draws keep their slot.
    want  = Int(SWEEP["n_scenarios"])
    cap   = Int(get(SWEEP, "max_scenario_draws", 4 * want))
    accepted = 0
    i = 0
    while accepted < want
        i += 1
        if i > cap
            say("  ! stopped after $(cap) draws with only $(accepted)/$(want) usable " *
                "scenarios — references are failing more often than the guard allows")
            break
        end
        sid  = @sprintf("s%03d", i)
        seed = Int(SWEEP["scenario_seed_base"]) + i
        p    = sample_scenario_params(seed)
        base = scenario_keys(seed, p)

        # ── Reference: 1 agent, no uncertainty constraint ⇒ the shortest path ──
        if haskey(scen_done, sid)
            ref = scen_done[sid]
            L_ref = tryparse(Float64, ref["L_ref"]); U_ref = tryparse(Float64, ref["U_ref"])
            (L_ref === nothing || U_ref === nothing) && (say("  $(sid): previously rejected, skipping"); continue)
            # A scenario the screen threw out has a perfectly good L_ref/U_ref, so the
            # empty-L_ref test above cannot see it — it needs its own column. Absent on
            # sweeps written before the screen existed, which is exactly right: they
            # were never screened, so nothing was rejected for this reason.
            rr = get(ref, "reject_reason", "")
            isempty(rr) || (say("  $(sid): previously rejected ($(rr)), skipping"); continue)
            # Sweeps written before wall_ref existed have no such column; a missing
            # value disables the ratio for that scenario instead of erroring.
            wall_ref = something(tryparse(Float64, get(ref, "wall_ref", "")), NaN)
            accepted += 1
            say("  $(sid): resumed  D=$(Int(p.D)) L=$(p.nl) O=$(p.no)  L_ref=$(round(L_ref,digits=1)) U_ref=$(round(U_ref,digits=3))")
        else
            rdir = joinpath(ROOT, "reference", sid)
            ov = merge(base, Dict("algorithms" => "hexspline_cl", "num_agents" => "1",
                                  "unc_radius_threshold" => "1.0e9",
                                  "emit_figures" => string(save_figs), "emit_csv" => string(save_csv)))
            write_run_config(joinpath(rdir, "_cfg"), ov)
            r = run_one(joinpath(rdir, "_cfg"), rdir, joinpath(rdir, "run.log"), timeout)
            flatten_algo_dir!(rdir, "hexspline_cl")
            res = parse_results_yaml(joinpath(rdir, "results.yaml"))
            # U_ref is the reference's DISCRETE (seed) uncertainty, not its refined
            # primary_unc. unc_radius_threshold gates hexspline_cl's A* at the goal
            # pop, on the node polyline — and that polyline is always longer, hence
            # always less certain, than the spline it seeds (s001: 6.5154 over
            # 1100.0 m vs 4.0231 over 800.2 m). Anchored on primary_unc, pct=100
            # asked the search to certify on the polyline a bound only the spline
            # ever met, so the reference's own seed was inadmissible and every
            # hexspline_cl row came back no_solution. This is the one planner-
            # specific key the harness reads, and only from the REFERENCE, which is
            # already hardcoded to hexspline_cl above; methods under test still
            # report through primary_length/primary_unc alone.
            # Full precision on the way out too: at pct=100 the threshold IS U_ref,
            # and %.4f rounding down by 5e-5 rejects the reference seed against a
            # 1e-6 UNC_FEAS_TOL. A resume reads this column back.
            L_ref = getnum(res, "primary_length"); U_ref = getnum(res, "primary_disc_unc")
            row = Dict("scenario_id" => sid, "seed" => string(seed),
                       "goal_dist" => string(p.D), "n_landmarks" => string(p.nl),
                       "n_obstacles" => string(p.no),
                       "L_ref" => L_ref === nothing ? "" : @sprintf("%.3f", L_ref),
                       "U_ref" => U_ref === nothing ? "" : repr(U_ref),
                       "U_ref_cont" => (v = getnum(res, "primary_unc"); v === nothing ? "" : repr(v)),
                       # The yardstick for wall_ratio on every trial below. Same
                       # subprocess shape as a method run (Julia startup + graph
                       # build + refinement), so the ~17.7 s fixed overhead is in
                       # both numerator and denominator and largely divides out.
                       "wall_ref" => @sprintf("%.1f", r.wall))
            wall_ref = r.wall
            push!(scen_rows, row)
            write_table(joinpath(ROOT, "scenarios.csv"), scen_rows,
                        ["scenario_id","seed","goal_dist","n_landmarks","n_obstacles",
                         "L_ref","U_ref","U_ref_cont","wall_ref","reject_reason"])
            if L_ref === nothing || U_ref === nothing || !isfinite(L_ref) || !isfinite(U_ref) || U_ref <= 0
                # No shortest path ⇒ nothing to normalise against. Recorded with
                # empty L_ref/U_ref so a resume skips it instead of retrying.
                say("  $(sid): REFERENCE FAILED (rc=$(r.rc) killed=$(r.killed)) — " *
                    "scenario skipped, does NOT count toward n_scenarios")
                continue
            end
            # ── Trivial-scenario screen (baseline comparisons only) ──
            # Screened at pcts[1], the LOOSEST rung the ladder will ever apply: if the
            # straight line already clears that, the scenario hands every method a free
            # pass at the top of its ladder and measures no coordination at all.
            # Rejected draws keep their scenarios.csv row (with L_ref/U_ref intact, so
            # the rejection is auditable and a resume can skip it) but do not count
            # toward n_scenarios.
            if REJECT_TRIVIAL
                sthr = pcts[1] / 100 * U_ref
                sdir = joinpath(ROOT, "screen", sid)
                sov  = merge(base, SCREEN.overrides,
                             Dict("unc_radius_threshold" => @sprintf("%.10g", sthr),
                                  "emit_figures" => string(save_figs), "emit_csv" => string(save_csv)))
                write_run_config(joinpath(sdir, "_cfg"), sov)
                sr = run_one(joinpath(sdir, "_cfg"), sdir, joinpath(sdir, "run.log"), timeout)
                flatten_algo_dir!(sdir, SCREEN.algorithm)
                sres = parse_results_yaml(joinpath(sdir, "results.yaml"))
                slog = isfile(joinpath(sdir, "run.log")) ? read(joinpath(sdir, "run.log"), String) : ""
                sok, swhy = classify(sr, sres, slog, sthr)
                sL = getnum(sres, "primary_length"); sU = getnum(sres, "primary_unc")
                if sok
                    row["reject_reason"] = "trivial_straight"
                    write_table(joinpath(ROOT, "scenarios.csv"), scen_rows,
                                ["scenario_id","seed","goal_dist","n_landmarks","n_obstacles",
                                 "L_ref","U_ref","U_ref_cont","wall_ref","reject_reason"])
                    say("  $(sid): TRIVIAL — $(SCREEN.label) already meets the $(pcts[1])% bound " *
                        "(unc $(sU === nothing ? "?" : round(sU,digits=3)) <= $(round(sthr,digits=3)), " *
                        "len $(sL === nothing ? "?" : round(sL,digits=1))) — rejected, does NOT " *
                        "count toward n_scenarios  ($(round(sr.wall,digits=1))s)")
                    continue
                end
                say("  $(sid): screen ok — $(SCREEN.label) fails the $(pcts[1])% bound ($(swhy))")
            end
            accepted += 1
            say("  $(sid): D=$(Int(p.D)) L=$(p.nl) O=$(p.no)  L_ref=$(round(L_ref,digits=1)) " *
                "U_ref=$(round(U_ref,digits=3))  ($(round(r.wall,digits=1))s)  [$(accepted)/$(want)]")
        end

        # ── Ladder ──
        consec_allfail = 0
        # Per-method failure streak. A method that has failed `method_patience`
        # times in a row stops being retried at stricter levels for THIS scenario,
        # regardless of how the other methods are doing — the constraint only gets
        # harder going down, so retrying is almost always spending minutes to
        # re-derive the same failure. "Almost": see method_patience in sweep.yaml
        # for the ~1% of pairs that do recover, and the mechanism behind it.
        consec_fail = Dict(m.label => 0 for m in METHODS)
        for pct in pcts
            threshold = pct / 100 * U_ref
            todo = [m for m in METHODS if !((sid, string(pct)) in done[m.label])]
            outcomes = Dict{String,Bool}()
            for m in METHODS   # already-done rows still count toward the early stop
                (sid, string(pct)) in done[m.label] || continue
                prev = first(filter(r -> r["scenario_id"] == sid && r["pct"] == string(pct), trials[m.label]))
                outcomes[m.label] = prev["success"] == "true"
            end

            @sync for m in todo
                @async Base.acquire(SLOTS) do
                    rundir = joinpath(ROOT, m.label, "$(sid)_p$(lpad(pct,3,'0'))")
                    ov = merge(base, m.overrides,
                               Dict("unc_radius_threshold" => @sprintf("%.10g", threshold),
                                    "emit_figures" => string(save_figs), "emit_csv" => string(save_csv)))
                    write_run_config(joinpath(rundir, "_cfg"), ov)
                    r = run_one(joinpath(rundir, "_cfg"), rundir, joinpath(rundir, "run.log"), timeout)
                    flatten_algo_dir!(rundir, m.algorithm)
                    res = parse_results_yaml(joinpath(rundir, "results.yaml"))
                    logtxt = isfile(joinpath(rundir, "run.log")) ? read(joinpath(rundir, "run.log"), String) : ""
                    ok, why = classify(r, res, logtxt, threshold)
                    L = getnum(res, "primary_length")

                    row = merge(res, Dict(
                        "scenario_id" => sid, "pct" => string(pct),
                        "threshold" => @sprintf("%.4f", threshold),
                        "method" => m.label, "algorithm" => m.algorithm,
                        "num_agents" => get(m.overrides, "num_agents", "?"),
                        "success" => string(ok), "fail_reason" => why,
                        "length_ratio" => (ok && L !== nothing) ? @sprintf("%.4f", L / L_ref) : "",
                        "wall_s" => @sprintf("%.1f", r.wall),
                        # Time cost in the same currency as length_ratio: this run's
                        # wall clock over the reference's. Recorded for FAILURES too
                        # — an expensive failure is the interesting kind, and the
                        # summary averages successes only.
                        "wall_ratio" => isfinite(wall_ref) && wall_ref > 0 ?
                                        @sprintf("%.4f", r.wall / wall_ref) : ""))
                    lock(lk) do
                        push!(trials[m.label], row)
                        push!(done[m.label], (sid, string(pct)))
                        outcomes[m.label] = ok
                        write_table(joinpath(ROOT, m.label, "trials.csv"), trials[m.label], CORE_COLS)
                        say("    $(sid) p$(pct)  $(rpad(m.label,16))  " *
                            (ok ? "ok   ratio=$(row["length_ratio"])" : "FAIL $(why)") *
                            "   $(row["wall_s"])s")
                    end
                end
            end

            # ── Per-method drop ──
            # Written as explicit rows at the moment of the drop, for every level
            # still below, rather than tracked in a separate "dropped" set: the
            # `todo` filter above already skips anything in `done`, so recording
            # the rows IS the exclusion, and there is no second piece of state that
            # can disagree with the CSV. Same denominator argument as the
            # all-methods stop below — the levels must appear as failures, not go
            # missing, or summarize() divides by the easy scenarios only.
            if method_patience > 0
                rest = pcts[findfirst(==(pct), pcts)+1:end]
                for m in METHODS
                    haskey(outcomes, m.label) || continue
                    consec_fail[m.label] = outcomes[m.label] ? 0 : consec_fail[m.label] + 1
                    (consec_fail[m.label] < method_patience || isempty(rest)) && continue
                    newly = 0
                    for q in rest
                        (sid, string(q)) in done[m.label] && continue
                        push!(trials[m.label], Dict(
                            "scenario_id" => sid, "pct" => string(q),
                            "threshold" => @sprintf("%.4f", q / 100 * U_ref),
                            "method" => m.label, "algorithm" => m.algorithm,
                            "num_agents" => get(m.overrides, "num_agents", "?"),
                            "success" => "false", "fail_reason" => "not_run_after_fail",
                            "length_ratio" => "", "wall_s" => "0.0", "wall_ratio" => ""))
                        push!(done[m.label], (sid, string(q)))
                        newly += 1
                    end
                    if newly > 0
                        write_table(joinpath(ROOT, m.label, "trials.csv"), trials[m.label], CORE_COLS)
                        say("    $(sid): $(m.label) failed $(consec_fail[m.label])x in a row " *
                            "(through $(pct)%) — dropped, $(newly) stricter level(s) recorded unrun")
                    end
                end
            end

            all_failed = !isempty(outcomes) && !any(values(outcomes))
            consec_allfail = all_failed ? consec_allfail + 1 : 0
            if patience > 0 && consec_allfail >= patience
                say("    $(sid): every method failed at $(patience) consecutive levels " *
                    "(down to $(pct)%) — constraint is impossible, moving on")
                # Record the skipped levels as failures rather than omitting them.
                # The early stop is a COMPUTE shortcut resting on monotonicity —
                # the feasible set at a lower pct is a subset of this one — but if
                # the rows are simply absent, summarize() divides by however many
                # scenarios happened to reach that level, and those are the EASY
                # ones. That inflated every success_rate below the first stop
                # (n_total fell 10 → 8 → 7 → 6 while the rate appeared to hold).
                # Writing the assumption down keeps the denominator the full
                # scenario count and leaves the shortcut auditable by fail_reason.
                for q in pcts[findfirst(==(pct), pcts)+1:end], m in METHODS
                    (sid, string(q)) in done[m.label] && continue
                    push!(trials[m.label], Dict(
                        "scenario_id" => sid, "pct" => string(q),
                        "threshold" => @sprintf("%.4f", q / 100 * U_ref),
                        "method" => m.label, "algorithm" => m.algorithm,
                        "num_agents" => get(m.overrides, "num_agents", "?"),
                        "success" => "false", "fail_reason" => "not_run_below_stop",
                        "length_ratio" => "", "wall_s" => "0.0"))
                    push!(done[m.label], (sid, string(q)))
                end
                for m in METHODS
                    write_table(joinpath(ROOT, m.label, "trials.csv"), trials[m.label], CORE_COLS)
                end
                break
            end
        end
    end

    summarize(ROOT, METHODS)
    say("── done: $(ROOT) ──")
    close(logio)
end

main()
