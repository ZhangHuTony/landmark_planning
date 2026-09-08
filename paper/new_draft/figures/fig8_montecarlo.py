#!/usr/bin/env python3
"""Fig. 8 + Table `tab:mc` of the paper, from a Monte Carlo set of HoloOcean trials.

    python fig8_montecarlo.py <mc_run_dir> [--prefix mc] [--out DIR]
                              [--stem fig8_montecarlo] [--nstd 1] [--width 3.5]

`<mc_run_dir>` is the planner run directory that `monte_carlo.py` flew, holding
one `<prefix><i>/` subdirectory per trial with `run_log.npz` + `sim_report.yaml`.

Writes into <out> (default: this figures/ directory):
  <stem>.{pdf,svg,eps,png}   the figure. (a) planned path vs. mean executed
                             path, both agents, with the n-sigma cross-track
                             envelope drawn on the map; (b) the same envelope
                             at true scale against arc length -- on a 1.9 km
                             corridor a 4 m band is a hairline in (a), so the
                             number the panel is about needs its own axis
  <stem>_summary.csv         one row per trial, the numbers behind the table
and prints the LaTeX row for `tab:mc`.

Why this exists rather than `multiagent_base/viz.py:fig_monte_carlo`. That
function is the quick visual check and writes PDF+PNG only, with `alpha=0.3`
fills. PostScript has no transparency: an alpha'd patch makes Ghostscript
flatten the whole page into one bitmap, which is exactly what made Fig. 1's EPS
a single unselectable object in Illustrator. So the alphas here are pre-blended
onto white (`over_white`) and all four formats agree -- same treatment as
`make_figs_baseline.py` (Figs. 3-4) and `fig1_overview_mpl.py` (Fig. 1), whose
rcParams and `save()` this file mirrors so the whole paper stays in one
typographic family.

The aggregation is `monte_carlo.aggregate`'s, deliberately unchanged: each
trial's GROUND-TRUTH track is resampled onto a shared arc grid capped at the
SHORTEST trial (never extrapolate past a run that stalled), and each trial's
deviation from the mean path is projected onto the mean path's local normal --
a tube, not independent x/y bands, which would only be right for a straight
line.

Run it with the simulator's interpreter, which has matplotlib and numpy:
    ~/Research/multiagent_base/.venv/bin/python fig8_montecarlo.py <mc_run_dir>
`plan_io` is imported from that same checkout because it owns the planner's and
the recorder's file schemas; re-implementing its YAML reader here is how the two
would drift.
"""
import csv
import math
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

SIM_REPO = os.environ.get("MULTIAGENT_BASE",
                          os.path.expanduser("~/Research/multiagent_base"))
sys.path.insert(0, SIM_REPO)
import plan_io  # noqa: E402  -- owns read_yaml and the run_log/plan schemas

HERE = os.path.dirname(os.path.abspath(__file__))

# Same rcParams as make_figs_baseline.py and fig1_overview_mpl.py. ps.fonttype
# is 3, not 42: matplotlib's Type 42 embedding of Nimbus Roman writes a font
# Ghostscript rejects ("invalidfont in definefont") and the EPS will not open.
# svg.fonttype "none" keeps labels as live text for Illustrator.
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Nimbus Roman", "Times New Roman", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.linewidth": 0.6,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 6.5,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.5,
    "ytick.major.size": 2.5,
    "pdf.fonttype": 42,
    "ps.fonttype": 3,
    "svg.fonttype": "none",
})

COLUMN_IN = 3.5          # IEEE conference column, as Figs. 1 and 3-4
GRID_POINTS = 100        # monte_carlo.py's, so the two figures agree
DEFAULT_NSTD = 1         # main.tex says "one standard-deviation envelope"

# viz.jl's palette, via multiagent_base/viz.py, so Figs. 1 and 8 agree on
# who is who.
PRIMARY_COLOR = "#0000FF"
SUPPORT_COLORS = ["#800080", "#008080", "#FF8C00", "#DC143C"]
OBSTACLE_FILL, OBSTACLE_ALPHA = "#595959", 0.55
BAND_ALPHA = 0.30
PLANNED_COLOR = "#404040"

LW_OBSTACLE, LW_PLANNED, LW_PRIMARY, LW_SUPPORT = 0.6, 0.7, 1.1, 0.8
MS_LANDMARK, MS_START, MS_GOAL = 2.6, 4.2, 6.5
DEV_PANEL_IN = 0.62      # height of the cross-track panel, inches


def over_white(color, alpha):
    """Flatten `color` at `alpha` onto white.

    Used instead of set_alpha: the PostScript backend has no transparency, so
    an alpha'd patch comes out opaque in the EPS and stops matching the PDF.
    """
    rgb = matplotlib.colors.to_rgb(color)
    return tuple(1.0 + alpha * (v - 1.0) for v in rgb)


def save(fig, out_dir, stem):
    """PDF for LaTeX, SVG and EPS for Illustrator, PNG to eyeball."""
    for ext, kw in (("pdf", {}), ("svg", {}), ("eps", {}), ("png", {"dpi": 300})):
        fig.savefig(os.path.join(out_dir, f"{stem}.{ext}"), **kw)
    print(f"  -> {os.path.join(out_dir, stem)}.{{pdf,svg,eps,png}}")


# ---------------------------------------------------------------------------
# Trials
# ---------------------------------------------------------------------------

class Trial:
    """One flown trial: its run_log arrays and its sim_report verdict."""

    def __init__(self, path):
        self.dir = path
        self.tag = os.path.basename(path)
        self.z = np.load(os.path.join(path, "run_log.npz"), allow_pickle=False)
        self.report = plan_io.read_yaml(os.path.join(path, "sim_report.yaml"))
        self.agents = [str(a) for a in self.z["agents"]]
        self.primary = str(self.z["primary"])

    def node(self, name, field):
        k = f"{name}_node_{field}"
        return self.z[k] if k in self.z.files else np.empty(0)

    def terminal_error_xy(self):
        """Estimate minus truth at the primary's last node, in world x/y.

        The sample covariance of this across trials is the empirical terminal
        sigma -- the honest number to put beside the planner's prediction,
        since `terminal_unc_m` is the estimator's own belief about itself.
        """
        est, truth = self.node(self.primary, "est"), self.node(self.primary, "truth")
        if len(est) == 0 or len(truth) == 0:
            return None
        return np.asarray(est[-1][:2], float) - np.asarray(truth[-1][:2], float)


def find_trials(run_dir, prefix):
    pat = re.compile(rf"^{re.escape(prefix)}(\d+)$")
    out = []
    for name in os.listdir(run_dir):
        m = pat.match(name)
        p = os.path.join(run_dir, name)
        if m and os.path.isfile(os.path.join(p, "run_log.npz")) \
              and os.path.isfile(os.path.join(p, "sim_report.yaml")):
            out.append((int(m.group(1)), p))
    return [p for _, p in sorted(out)]


def aggregate(trials):
    """monte_carlo.aggregate, on a list of Trials.

    Returns {agent: {grid, mean_xy, std_cross, normal}} and the per-agent
    trial count. `grid` is the shared arc-length axis panel (b) plots against.
    """
    agg, n_used = {}, {}
    first = trials[0]
    for name in first.agents:
        usable = []
        for t in trials:
            a, tr = t.node(name, "arc"), t.node(name, "truth")
            if len(a) > 1 and len(tr) == len(a):
                usable.append((np.asarray(a, float), np.asarray(tr, float)))
        if len(usable) < 2:
            print(f"warning: fewer than 2 usable trials for {name}, no band drawn")
            continue

        # Cap the shared grid at the SHORTEST trial's flown arc so the band
        # never extrapolates past a trial that stalled early.
        grid = np.linspace(0.0, min(a[-1] for a, _ in usable), GRID_POINTS)
        paths = np.array([
            np.column_stack([np.interp(grid, a, tr[:, 0]), np.interp(grid, a, tr[:, 1])])
            for a, tr in usable
        ])                                        # (n_trials, GRID_POINTS, 2)
        mean_xy = paths.mean(axis=0)

        tangent = np.gradient(mean_xy, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        cross = np.einsum("tgk,gk->tg", paths - mean_xy, normal)
        agg[name] = {"grid": grid, "mean_xy": mean_xy,
                     "std_cross": cross.std(axis=0), "normal": normal}
        n_used[name] = len(usable)
    return agg, n_used


# ---------------------------------------------------------------------------
# Table
# ---------------------------------------------------------------------------

def summarize(trials, out_csv):
    """Per-trial rows for `tab:mc`, plus the aggregate the table prints."""
    rows = []
    for t in trials:
        r, tk, e = t.report["result"], t.report["tracking"], t.report["estimation"]
        err = t.terminal_error_xy()
        rows.append({
            "trial": t.tag,
            "reached_goal": r["reached_goal"],
            "goal_distance_m": r["goal_distance_m"],
            "max_penetration_m": r["max_penetration_m"],
            "max_hull_penetration_m": r["max_hull_penetration_m"],
            "collision_sensor_hits": r["collision_sensor_hits"],
            "n_skipped_waypoints": len(r["skipped_waypoints"]) if isinstance(r["skipped_waypoints"], list) else 0,
            "rms_cross_track_m": tk["rms_cross_track_m"],
            "max_cross_track_m": tk["max_cross_track_m"],
            "arc_length_m": tk["arc_length_m"],
            "ticks": tk["ticks"],
            "terminal_est_error_m": e["terminal_est_error_m"],
            "terminal_unc_m": e["terminal_unc_m"],
            "unc_threshold_m": e["unc_threshold_m"],
            "planner_predicted_unc_m": e["planner_predicted_unc_m"],
            "comm_events": e["comm_events"],
            "landmark_events": e["landmark_events"],
            "term_err_x": err[0] if err is not None else float("nan"),
            "term_err_y": err[1] if err is not None else float("nan"),
        })
        # A support that loses the plan stops delivering fixes, which shows up
        # on the primary as a high terminal sigma -- keep it in the same row.
        for k, v in t.report.items():
            if not k.startswith("support_"):
                continue
            name = k[len("support_"):]
            rows[-1][f"{name}_rms_cross_track_m"] = v["rms_cross_track_m"]
            rows[-1][f"{name}_max_penetration_m"] = v["max_penetration_m"]
            rows[-1][f"{name}_n_skipped_waypoints"] = (
                len(v["skipped_waypoints"]) if isinstance(v["skipped_waypoints"], list) else 0)

    with open(out_csv, "w", newline="") as f:
        fields = list(rows[0])
        for r in rows[1:]:
            fields += [k for k in r if k not in fields]
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(rows)
    print(f"  -> {out_csv}")

    n = len(rows)
    thr = rows[0]["unc_threshold_m"]
    pred = rows[0]["planner_predicted_unc_m"]
    reached = sum(bool(r["reached_goal"]) for r in rows)
    clear = sum(r["max_penetration_m"] <= 0.0 and r["collision_sensor_hits"] == 0 for r in rows)
    bound_met = sum(r["terminal_unc_m"] <= thr for r in rows)

    # Empirical terminal sigma: det(sample covariance of the true terminal
    # error)^{1/4}, the same scalar the planner's unc_radius reports.
    errs = np.array([[r["term_err_x"], r["term_err_y"]] for r in rows], float)
    errs = errs[np.isfinite(errs).all(axis=1)]
    emp_sigma = float(np.linalg.det(np.cov(errs.T, bias=False)) ** 0.25) if len(errs) > 2 else float("nan")
    mean_marginal = float(np.mean([r["terminal_unc_m"] for r in rows]))

    stats = {
        "n": n, "reached": reached, "clear": clear, "bound_met": bound_met,
        "threshold_m": thr, "predicted_sigma_m": pred,
        "empirical_sigma_m": emp_sigma, "mean_marginal_sigma_m": mean_marginal,
        "max_penetration_m": max(r["max_penetration_m"] for r in rows),
        "mean_rms_cross_track_m": float(np.mean([r["rms_cross_track_m"] for r in rows])),
        "max_cross_track_m": max(r["max_cross_track_m"] for r in rows),
        "mean_arc_length_m": float(np.mean([r["arc_length_m"] for r in rows])),
        "max_ticks": max(r["ticks"] for r in rows),
        "skipped_waypoints": sum(r["n_skipped_waypoints"] for r in rows),
    }

    print("\n  trials                 %d" % n)
    print("  reached goal           %d/%d" % (reached, n))
    print("  obstacle-clear         %d/%d  (max penetration %.3f m)" % (clear, n, stats["max_penetration_m"]))
    print("  terminal bound met     %d/%d  (threshold %.4f m)" % (bound_met, n, thr))
    print("  predicted sigma_T      %.4f m  (planner, on the control polygon)" % pred)
    print("  mean estimator sigma_T %.4f m" % mean_marginal)
    print("  empirical sigma_T      %.4f m  (sample cov of the true terminal error)" % emp_sigma)
    print("  cross-track            %.3f m RMS (mean over trials), %.3f m worst" % (
        stats["mean_rms_cross_track_m"], stats["max_cross_track_m"]))
    sup_skips = sum(v for r in rows for k, v in r.items()
                    if k.endswith("_n_skipped_waypoints"))
    stats["support_skipped_waypoints"] = sup_skips
    print("  skipped waypoints      %d primary, %d support" % (
        stats["skipped_waypoints"], sup_skips))
    print("\n  tab:mc row:")
    print("    ladder\\_shapes @ 30\\%% & %d & %d/%d & %d/%d & %.2f\\,/\\,%.2f \\\\" % (
        n, n - clear, n, bound_met, n, pred, emp_sigma))
    return stats


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def draw(trials, agg, nstd, width_in):
    z = trials[0].z
    primary = trials[0].primary
    obstacles = [np.asarray(z[f"obstacle{i}_verts"], float)
                 for i in range(int(z["n_obstacles"]))]
    landmarks = np.asarray(z["landmarks"], float).reshape(-1, 2)

    xs = [p[0] for a in agg.values() for p in a["mean_xy"]] + list(landmarks[:, 0])
    ys = [p[1] for a in agg.values() for p in a["mean_xy"]] + list(landmarks[:, 1])
    for v in obstacles:
        xs += list(v[:, 0]); ys += list(v[:, 1])
    for name in trials[0].agents:
        ref = np.asarray(z[f"{name}_ref"], float)
        xs += list(ref[:, 0]); ys += list(ref[:, 1])
    pad = 40.0
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad

    # Eight legend entries at ncol=3 wrap to three rows; compute it rather
    # than hard-coding, or the legend grows into panel (b)'s x label.
    n_entries = 5 + len([n for n in trials[0].agents if n in agg]) + 1
    left, right, top = 0.50, 0.06, 0.10
    xlabel_h, legend_row_h = 0.30, 0.145
    legend_h = math.ceil(n_entries / 3) * legend_row_h + 0.10
    gap = 0.06                       # between the map's x label and panel (b)
    ax_w = width_in - left - right
    ax_h = ax_w * (y1 - y0) / (x1 - x0)          # equal aspect fixes the map
    dev_h = DEV_PANEL_IN
    fig_h = top + ax_h + xlabel_h + gap + dev_h + xlabel_h + legend_h

    fig = plt.figure(figsize=(width_in, fig_h))
    ax = fig.add_axes([left / width_in,
                       (legend_h + xlabel_h + dev_h + gap + xlabel_h) / fig_h,
                       ax_w / width_in, ax_h / fig_h])
    ax.set_aspect("equal")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    for sp in ax.spines.values():
        sp.set_zorder(6)

    dev = fig.add_axes([left / width_in, (legend_h + xlabel_h) / fig_h,
                        ax_w / width_in, dev_h / fig_h])
    dev.set_xlabel("arc length (m)")
    dev.set_ylabel("cross-track (m)")

    obstacle_handle = None
    for verts in obstacles:
        p = Polygon(verts, closed=True, facecolor=over_white(OBSTACLE_FILL, OBSTACLE_ALPHA),
                    edgecolor="black", linewidth=LW_OBSTACLE, zorder=1)
        ax.add_patch(p)
        obstacle_handle = obstacle_handle or p

    planned_handle = None
    for name in trials[0].agents:
        ref = np.asarray(z[f"{name}_ref"], float)
        line, = ax.plot(ref[:, 0], ref[:, 1], linestyle=(0, (3, 2)),
                        color=PLANNED_COLOR, linewidth=LW_PLANNED, zorder=2,
                        label="Planned")
        planned_handle = planned_handle or line

    band_handles, mean_handles = [], []
    for name in trials[0].agents:
        if name not in agg:
            continue
        a = agg[name]
        mean_xy, std_cross, normal = a["mean_xy"], a["std_cross"], a["normal"]
        c = PRIMARY_COLOR if name == primary else SUPPORT_COLORS[
            (int(name[3:]) - 1) % len(SUPPORT_COLORS)]
        role = "primary" if name == primary else "support"
        lw = LW_PRIMARY if name == primary else LW_SUPPORT

        band = np.vstack([mean_xy + nstd * std_cross[:, None] * normal,
                          (mean_xy - nstd * std_cross[:, None] * normal)[::-1]])
        # Pre-blended, never set_alpha -- see over_white.
        fill = ax.fill(band[:, 0], band[:, 1], facecolor=over_white(c, BAND_ALPHA),
                       edgecolor="none", zorder=3,
                       label=rf"$\pm{nstd:g}\sigma$ cross-track (per agent)")[0]
        line, = ax.plot(mean_xy[:, 0], mean_xy[:, 1], "-", color=c, zorder=4,
                        linewidth=lw, solid_joinstyle="round",
                        label=f"{role.capitalize()} ({name}) mean flown")

        # (b) the same envelope, at true scale: on a 1.9 km corridor the map's
        # band is a hairline, and its width is the quantity the figure is for.
        dev.fill_between(a["grid"], -nstd * std_cross, nstd * std_cross,
                         facecolor=over_white(c, BAND_ALPHA), edgecolor="none",
                         zorder=2 if name == primary else 1)
        dev.plot(a["grid"], nstd * std_cross, "-", color=c, linewidth=lw * 0.7,
                 zorder=3)
        dev.plot(a["grid"], -nstd * std_cross, "-", color=c, linewidth=lw * 0.7,
                 zorder=3)
        if name == primary:
            band_handles.append(fill)
        mean_handles.append(line)
    dev.axhline(0.0, color="0.5", linewidth=0.4, zorder=0)

    lm_handle, = ax.plot(landmarks[:, 0], landmarks[:, 1], linestyle="none",
                         marker="o", markersize=MS_LANDMARK, color="black",
                         markeredgewidth=0, zorder=5, label="Landmarks")
    start = agg[primary]["mean_xy"][0] if primary in agg \
        else np.asarray(z[f"{primary}_ref"], float)[0]
    start_handle, = ax.plot(*start, linestyle="none", marker="o", markersize=MS_START,
                            color="green", markeredgewidth=0, zorder=5, label="Start")
    goal_handle, = ax.plot(*np.asarray(z["goal_xy"], float), linestyle="none",
                           marker="*", markersize=MS_GOAL, color="orange",
                           markeredgewidth=0, zorder=5, label="Goal")

    handles = [planned_handle, lm_handle, start_handle, goal_handle]
    if obstacle_handle is not None:
        obstacle_handle.set_label("Obstacle")
        handles.append(obstacle_handle)
    handles += mean_handles + band_handles
    handles = [h for h in handles if h is not None]
    fig.legend(handles=handles, labels=[h.get_label() for h in handles],
               loc="lower center", bbox_to_anchor=(0.5, 0.0), ncol=3, frameon=False,
               handlelength=1.6, columnspacing=1.2, handletextpad=0.5, borderaxespad=0.2)
    return fig


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    opts = dict(prefix="mc", out=HERE, stem="fig8_montecarlo",
                nstd=str(DEFAULT_NSTD), width=str(COLUMN_IN))
    for a in argv[1:]:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            opts[k] = v
        elif a.startswith("--"):
            sys.exit(f"use --key=value, got {a}")
    run_dir = os.path.abspath(args[0])
    nstd, width = float(opts["nstd"]), float(opts["width"])

    paths = find_trials(run_dir, opts["prefix"])
    if len(paths) < 2:
        sys.exit(f"need at least 2 trials in {run_dir} with prefix {opts['prefix']!r}, found {len(paths)}")
    trials = [Trial(p) for p in paths]
    print(f"{len(trials)} trials: {', '.join(t.tag for t in trials)}")

    os.makedirs(opts["out"], exist_ok=True)
    summarize(trials, os.path.join(opts["out"], f"{opts['stem']}_summary.csv"))

    agg, n_used = aggregate(trials)
    for name, k in n_used.items():
        print(f"  aggregated {k}/{len(trials)} trials for {name}")
    fig = draw(trials, agg, nstd, width)
    save(fig, opts["out"], opts["stem"])
    plt.close(fig)


if __name__ == "__main__":
    main(sys.argv)
