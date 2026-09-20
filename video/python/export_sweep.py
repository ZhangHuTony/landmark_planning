#!/usr/bin/env python3
"""Export the benchmark aggregates and the video's headline numbers.

Reads the sweep summaries and the Monte Carlo per-trial CSV that the paper's
own figure scripts read, and writes:

    video/data/sweep/levels.json     one record per (planner, constraint level)
    video/data/holo/numbers.json     every number the closing cards print

Every value is computed from those CSVs, then checked against the figure the
paper quotes, so a scene can never drift from the paper by hand-typing.
"""
import csv
import json
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "constraint_sweep" / "baseline_2026-08-17"
ABL = ROOT / "constraint_sweep" / "ablation_2026-08-17"
MC = ROOT / "paper/new_draft/figures/fig8_mc_behind_wall_summary.csv"
OUT_SWEEP = ROOT / "video/data/sweep"
OUT_HOLO = ROOT / "video/data/holo"

METHODS = ["hexspline_cl", "formation", "sequential", "clgbt", "greedy"]
LEVELS = [100, 90, 80, 70, 60, 50, 40, 30]
DAGGER_MIN = 5          # fewer solved than this is a dagger entry in Table I


def read(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def level_stats(trials, methods):
    """Success rate and mean length ratio per (method, level), over solved runs."""
    out = {}
    for m in methods:
        for pct in LEVELS:
            rows = [r for r in trials if r["method"] == m and int(r["pct"]) == pct]
            if not rows:
                continue
            ok = [r for r in rows if r["success"] == "true"]
            ratios = [float(r["length_ratio"]) for r in ok if r["length_ratio"]]
            walls = [float(r["wall_s"]) for r in ok if r["wall_s"]]
            out[f"{m}@{pct}"] = {
                "method": m, "pct": pct,
                "n_total": len(rows), "n_success": len(ok),
                "success_rate": len(ok) / len(rows),
                "mean_ratio": statistics.fmean(ratios) if ratios else None,
                "sd_ratio": statistics.stdev(ratios) if len(ratios) > 1 else (0.0 if ratios else None),
                "median_wall_s": statistics.median(walls) if walls else None,
                # Table I marks a mean over fewer than five solved scenarios
                "dagger": 0 < len(ok) < DAGGER_MIN,
            }
    return out


def close(a, b, tol, what):
    assert a is not None and abs(a - b) <= tol, f"{what}: computed {a}, paper says {b}"


def main():
    base = level_stats(read(BASE / "trials_all.csv"), METHODS)
    abl = level_stats(read(ABL / "trials_all.csv"),
                      ["hexspline_cl", "discrete_only", "straight_cont"])

    # --- checks against the numbers the paper prints -------------------
    close(base["hexspline_cl@50"]["success_rate"], 0.80, 1e-9, "ours success @50")
    close(base["formation@50"]["success_rate"], 0.54, 1e-9, "formation success @50")
    close(base["greedy@50"]["success_rate"], 0.04, 1e-9, "greedy success @50")
    close(base["greedy@50"]["mean_ratio"], 1.1329, 5e-4, "greedy ratio @50")
    close(base["sequential@30"]["mean_ratio"], 2.7601, 5e-4, "sequential ratio @30")
    close(base["hexspline_cl@50"]["median_wall_s"], 18.4, 0.2, "ours median wall @50")
    close(base["formation@50"]["median_wall_s"], 13.8, 0.2, "formation median wall @50")
    assert base["greedy@50"]["dagger"], "greedy @50 should be a dagger entry (2 of 50 solved)"

    # refinement shortening: discrete-only length over the full pipeline's
    short = {}
    for pct in LEVELS:
        f, d = abl.get(f"hexspline_cl@{pct}"), abl.get(f"discrete_only@{pct}")
        if f and d and f["mean_ratio"]:
            short[pct] = d["mean_ratio"] / f["mean_ratio"] - 1.0
    assert 0.06 < short[100] < 0.07, f"refinement gain at 100% = {short[100]}"

    # --- Monte Carlo ---------------------------------------------------
    mc = read(MC)
    thr = float(mc[0]["unc_threshold_m"])
    pred = float(mc[0]["planner_predicted_unc_m"])
    bound_met = sum(1 for r in mc if float(r["terminal_unc_m"]) <= thr)
    clear = sum(1 for r in mc
                if float(r["max_penetration_m"]) <= 0.0
                and int(r["collision_sensor_hits"]) == 0)
    goal = sum(1 for r in mc if r["reached_goal"].strip().lower() == "true")
    errs = [(float(r["term_err_x"]), float(r["term_err_y"])) for r in mc]
    n = len(errs)
    mx = statistics.fmean(e[0] for e in errs)
    my = statistics.fmean(e[1] for e in errs)
    sxx = sum((e[0] - mx) ** 2 for e in errs) / (n - 1)
    syy = sum((e[1] - my) ** 2 for e in errs) / (n - 1)
    sxy = sum((e[0] - mx) * (e[1] - my) for e in errs) / (n - 1)
    emp = (sxx * syy - sxy * sxy) ** 0.25      # same functional as scoring.unc_radius

    assert (len(mc), bound_met, clear, goal) == (30, 28, 30, 30), \
        f"MC: n={len(mc)} bound={bound_met} clear={clear} goal={goal}"
    close(pred, 1.796, 1e-3, "predicted sigma_T")
    close(emp, 1.7025, 2e-3, "empirical sigma_T")

    OUT_SWEEP.mkdir(parents=True, exist_ok=True)
    OUT_HOLO.mkdir(parents=True, exist_ok=True)
    (OUT_SWEEP / "levels.json").write_text(json.dumps(
        {"baseline": base, "ablation": abl, "levels": LEVELS,
         "refinement_gain": short}, indent=1))

    numbers = {
        "headline": {
            "ours_success_50": base["hexspline_cl@50"]["success_rate"],
            "formation_success_50": base["formation@50"]["success_rate"],
            "collisions": f"{len(mc) - clear}/{len(mc)}",
            "bound_met": f"{bound_met}/{len(mc)}",
            "predicted_sigma": pred,
            "empirical_sigma": emp,
            "prediction_error": abs(pred - emp) / emp,
        },
        "mc": {
            "n": len(mc), "goal": goal, "clear": clear, "bound_met": bound_met,
            "threshold": thr, "predicted": pred, "empirical": emp,
            "mean_marginal": statistics.fmean(float(r["terminal_unc_m"]) for r in mc),
            "sample_cov": [[sxx, sxy], [sxy, syy]],
            "mean_offset": [mx, my],
            "planned_length_m": float(mc[0]["planned_length_m"]),
            "errors": errs,
        },
        "timing": {
            "ours_median_wall_50": base["hexspline_cl@50"]["median_wall_s"],
            "formation_median_wall_50": base["formation@50"]["median_wall_s"],
        },
        "refinement_gain": short,
    }
    (OUT_HOLO / "numbers.json").write_text(json.dumps(numbers, indent=1))
    print("checks passed")
    print(f"  ours/formation success @50 : {base['hexspline_cl@50']['success_rate']:.0%}"
          f" vs {base['formation@50']['success_rate']:.0%}")
    print(f"  collisions {len(mc)-clear}/{len(mc)}   bound met {bound_met}/{len(mc)}")
    print(f"  predicted {pred:.3f} m vs empirical {emp:.3f} m "
          f"({abs(pred-emp)/emp:.1%} apart)")
    print(f"  refinement shortening 100%..50%: "
          f"{', '.join(f'{100*short[p]:.1f}%' for p in (100, 90, 80, 70, 60, 50))}")


if __name__ == "__main__":
    main()
