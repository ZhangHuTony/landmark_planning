#!/usr/bin/env python3
"""Figs. 3-6 of the paper and the rows of Table I, from the two constraint sweeps.

Baseline  constraint_sweep/baseline_2026-08-17 -- 5 planners x 50 screened scenarios
  fig3a_success_vs_constraint.{pdf,svg,eps,png}   success rate against constraint level
  fig3b_length_vs_constraint.{pdf,svg,eps,png}    mean primary length / L_ref over
                                                  the solved scenarios, same x (the
                                                  SD is in Table I, not drawn)
  fig4_wall_5level_box.{pdf,svg,eps,png}          <- the one in the paper
  fig4_wall_5level_box_log.{pdf,svg,eps,png}      spare, log axis, nothing clipped
  fig4_wall_5level_box_linear.{pdf,svg,eps,png}   spare, linear axis, nothing clipped
  fig4_wall_alllevel_box.{pdf,svg,eps,png}        spare, all eight levels, text width
  fig4_wall_{5,all}level_violin.{pdf,svg,eps,png} spare
Ablation  constraint_sweep/ablation_2026-08-17 -- 3 arms x 30 UNscreened scenarios
  fig5a_abl_success_vs_constraint / fig5b_abl_length_vs_constraint   same two plots
  fig6_abl_wall_5level_box.{pdf,svg,eps,png}      <- the one in the paper
  fig6_abl_wall_alllevel_box.{pdf,svg,eps,png}    spare
Table I   the LaTeX rows (mean +- SD length ratio at 100/70/50/30; the success
          rate is Figs. 3a/5a's) are printed to stdout so main.tex is pasted
          from here, never typed.

The two sweeps are different scenario populations (the baseline sweep rejects
trivial-straight draws, the ablation sweep does not), so the ablation arms are
only ever compared with hexspline_cl from the ablation sweep itself.

Wall-clock figures show successful runs only and hide the fliers beyond
1.5 x IQR; the y axis spans the whiskers that remain. All of them are LINEAR
(user's call, 2026-09-14: a log axis over barely one decade gives the reader
no cue that the tick spacing is logarithmic, and reads as a broken linear
axis). Linear only works because exactly one group runs away -- CL-GBT at 60%,
Q3 = 61 s with a legitimate whisker to 124 s, against 13-22 s everywhere else.
That group is CLIPPED by `clip_above` and carries a caret plus its real
numbers, so the reader loses nothing; see `_clip_note`. The wide all-level
spare stays on a log axis, where three CL-GBT groups (70/60/50%) exceed the
cap and three clip notes would be clutter. The ablation arms all live in
16-21 s and need neither. Groups with too few successes to summarize are
drawn as their individual runs instead; see MIN_BOX / MIN_VIOLIN.

Each figure is written four ways: PDF is what LaTeX includes, SVG and EPS both
open in Illustrator (SVG keeps live text; see `save`), PNG is a preview.

Run with any python that has matplotlib, e.g.:
  ~/Research/multiagent_base/.venv/bin/python make_figs_baseline.py
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/baseline_2026-08-17"))
ABL_SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/ablation_2026-08-17"))

# Fixed identity -> color assignment (dataviz reference palette, slots 1-5).
# Keep this order/color per method in EVERY figure of the paper.
METHODS = [
    ("hexspline_cl", "Ours",       "#2a78d6", "D"),
    ("formation",    "Formation",  "#eb6834", "s"),
    ("sequential",   "Sequential", "#1baf7a", "^"),
    ("clgbt",        "CL-GBT",     "#eda100", "o"),
    ("greedy",       "Greedy",     "#e87ba4", "v"),
]
# Ablation arms of hexspline_cl itself. Ours keeps its blue; the two arms take
# the violet / green slots reserved for them (they never share a figure with
# the baselines, so they only have to separate from the blue and each other).
ARMS = [
    ("hexspline_cl",  "Full pipeline",         "#2a78d6", "D"),
    ("discrete_only", "Discrete only",         "#4a3aa7", "p"),
    ("straight_cont", "Straight seed + cont.", "#008300", "X"),
]

LVL_PCTS = [100, 90, 80, 70, 60, 50, 40, 30]
TABLE_LEVELS = [100, 70, 50, 30]

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
    # Type 3 for PS, unlike the PDF above: matplotlib's Type 42 embedding of
    # Nimbus Roman writes a font Ghostscript rejects outright ("invalidfont in
    # definefont"), so the EPS would not open. Type 3 renders correctly; text
    # arrives in Illustrator as outlines rather than live text -- use the SVG
    # if you need to edit the labels. Only the PDF goes into the paper, and it
    # keeps Type 42.
    "ps.fonttype": 3,
    # keep SVG text as text, not outlines, so labels stay editable in
    # Illustrator; it substitutes a font only if Nimbus Roman is missing there
    "svg.fonttype": "none",
})


def over_white(color, alpha):
    """Flatten `color` at `alpha` onto white.

    Used instead of set_alpha: the PostScript backend has no transparency, so
    an alpha'd patch would come out opaque in the EPS and not match the PDF.
    Pre-blending keeps all four formats identical.
    """
    r, g, b = matplotlib.colors.to_rgb(color)
    return tuple(1 - alpha * (1 - c) for c in (r, g, b))


def save(fig, stem):
    """PDF for LaTeX, SVG and EPS for Illustrator, PNG to eyeball."""
    for ext, kw in (("pdf", {}), ("svg", {}), ("eps", {}),
                    ("png", {"dpi": 300})):
        fig.savefig(os.path.join(HERE, f"{stem}.{ext}"), **kw)


def read_trials(sweep):
    """Every row of the sweep's trials_all.csv."""
    with open(os.path.join(sweep, "trials_all.csv")) as f:
        return list(csv.DictReader(f))


def level_stats(rows, methods):
    """method -> pct -> dict(n, k, rate, mean, sd, wall).

    n is the number of scenarios at that level, k the number solved, rate the
    success rate in %, mean/sd the sample statistics of length_ratio over the
    k solved runs (sd = 0 for k = 1, both None for k = 0), and wall the
    wall_s of the solved runs. Rows the harness never executed
    (fail_reason not_run_*) carry wall_s = 0 and count only toward n.
    """
    out = {m: {} for m, *_ in methods}
    for r in rows:
        m = r["method"]
        if m not in out:
            continue
        d = out[m].setdefault(int(r["pct"]), dict(n=0, ok=[], wall=[]))
        d["n"] += 1
        if r["success"] == "true":
            d["ok"].append(float(r["length_ratio"]))
            d["wall"].append(float(r["wall_s"]))
    for per_level in out.values():
        assert sorted(per_level, reverse=True) == LVL_PCTS, \
            "sweep levels changed -- LVL_PCTS must follow"
        for d in per_level.values():
            k = len(d["ok"])
            d["k"] = k
            d["rate"] = 100.0 * k / d["n"]
            d["mean"] = float(np.mean(d["ok"])) if k else None
            d["sd"] = (float(np.std(d["ok"], ddof=1)) if k > 1
                       else (0.0 if k else None))
    return out


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------- Figs. 3 / 5 ----
# Two plots per sweep, both with the constraint level on x (inverted, so the
# bound tightens left to right) and the same frame, so they stack in one
# column float. Identity is the paper-wide color plus marker shape; Ours is
# the heavier line and draws over the others.
FIG_W, FIG_H = 3.5, 1.8   # 2.05 until 2026-09-10; shortened to fit the ICRA page limit
AX_RECT = (0.145, 0.235, 0.845, 0.745)


def _level_axes(ax):
    ax.set_xticks(LVL_PCTS)
    ax.set_xlim(103, 27)  # constraint tightens to the right
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", labelpad=2)
    ax.grid(axis="y", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)


def _style(methods, mi):
    """Line/marker kwargs for entry `mi` of `methods`; entry 0 is Ours."""
    _, _, color, marker = methods[mi]
    primary = mi == 0
    return dict(color=color, marker=marker, lw=1.6 if primary else 1.0,
                ms=4.4 if primary else 3.8, mfc=color, mec="0.15", mew=0.5,
                zorder=6 if primary else 4 - 0.1 * mi)


def _key(fig, methods, **kw):
    handles = [plt.Line2D([], [], label=label, **_style(methods, mi))
               for mi, (_, label, _, _) in enumerate(methods)]
    fig.legend(handles=handles, loc="lower center", ncol=len(methods),
               bbox_to_anchor=(AX_RECT[0] + AX_RECT[2] / 2, -0.008),
               frameon=False, fontsize=6.5, handlelength=1.7,
               handletextpad=0.4, columnspacing=1.0, borderaxespad=0.0, **kw)


def fig_success(stats, methods, stem):
    """Success rate (%) against constraint level, one line per planner."""
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    ax = fig.add_axes(AX_RECT)
    for mi, (m, _, _, _) in enumerate(methods):
        ax.plot(LVL_PCTS, [stats[m][p]["rate"] for p in LVL_PCTS],
                **_style(methods, mi))
    _level_axes(ax)
    ax.set_ylabel("success rate (%)")
    ax.set_ylim(-4, 107)
    ax.set_yticks([0, 25, 50, 75, 100])
    _key(fig, methods)
    save(fig, stem)
    plt.close(fig)


def fig_length(stats, methods, stem, ylim, yticks):
    """Mean primary length / L_ref over the solved runs, same x.

    Levels a planner never solved are left out. The across-scenario SD is
    reported in Table I rather than drawn: as bars it was as wide as the gaps
    between planners (Sequential 2.8 +- 1.5 at 30%), and more scenarios would
    not shrink it -- it measures how much the detour varies from one random
    scenario to the next, not how well the mean is known (that is SD/sqrt(n)).
    """
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    ax = fig.add_axes(AX_RECT)
    ax.axhline(1.0, color="0.6", lw=0.6, ls=(0, (1, 2)), zorder=1)
    for mi, (m, _, _, _) in enumerate(methods):
        pcts = [p for p in LVL_PCTS if stats[m][p]["k"] > 0]
        ax.plot(pcts, [stats[m][p]["mean"] for p in pcts], **_style(methods, mi))
    _level_axes(ax)
    ax.set_ylabel(r"primary length / $L_\mathrm{ref}$")
    ax.set_ylim(*ylim)
    ax.set_yticks(yticks)
    _key(fig, methods)
    save(fig, stem)
    plt.close(fig)


# ---------------------------------------------------------- Figs. 4 / 6 ----
# Wall-clock split by constraint level. Pooling all eight levels into one box
# per planner, as this figure used to, averaged away the only interesting thing
# in it: four of the five planners cost the same however tight the bound gets,
# and CL-GBT's median climbs while its tail runs to 730 s.
#
# Success rates fall off a cliff at the tight end (Greedy solves 2 of 50 at
# 30%), so the tight groups have almost nothing in them. Rather than draw a
# "box" over two runs, anything below the threshold is drawn as its individual
# runs -- a tick per successful trial. Violins need more support than boxes
# because a KDE over eight points is mostly kernel.
MIN_BOX, MIN_VIOLIN = 5, 10
LEVELS_5 = [100, 80, 60, 40, 30]  # the cut that fits a single column
WALL_TICKS = [10, 15, 20, 30, 50, 70, 100, 150, 200, 300, 500, 700]
WALL_LIM = (11, 900)  # violins only; boxes size their axis to the whiskers


def _wall_axes(ax, mode, lim):
    """Shared y scale. `mode` is "log", "linear" or "log10data" (violins)."""
    if mode == "log":
        ax.set_yscale("log")
        ticks = [t for t in WALL_TICKS if lim[0] <= t <= lim[1]]
        ax.set_yticks(ticks)
        ax.set_yticklabels([str(t) for t in ticks])
        ax.yaxis.set_minor_formatter(NullFormatter())
        ax.set_ylim(*lim)
    elif mode == "linear":
        ax.set_ylim(*lim)
    else:
        # violins are built on log10(wall) against a linear axis: a KDE has to
        # be estimated in the space the reader sees it in, and on a log axis a
        # linear-space KDE turns every distribution into a spike at the bottom
        ticks = [t for t in WALL_TICKS if lim[0] <= t <= lim[1]]
        ax.set_yticks(np.log10(ticks))
        ax.set_yticklabels([str(t) for t in ticks])
        ax.set_ylim(*np.log10(lim))
    ax.set_ylabel("wall-clock time (s)")
    ax.grid(axis="y", which="major", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)


def _clip_note(ax, x, color, q3, hi):
    """Mark a box the axis cuts off, and say in numbers where it really ends.

    A caret sits on the frame over the clipped box; the text goes to its right
    because the neighbouring groups are all far down the axis there. Without
    this the cut box would read as "runs off the top by some unknown amount".
    """
    y0, y1 = ax.get_ylim()
    ax.plot([x], [y1], marker="^", ms=3.0, mew=0, color=color,
            clip_on=False, zorder=6)
    ax.text(x + 0.55, y1 - 0.045 * (y1 - y0),
            "Q3 %.0f s, whisker %.0f s" % (q3, hi),
            fontsize=6, color=color, ha="left", va="top", zorder=6)


def fig4(stats, methods, levels, kind, stem, width, yscale="log",
         clip_above=None):
    """One grouped figure: `kind` is "box" or "violin".

    Boxes hide their fliers (beyond 1.5 x IQR) and the axis is fitted to what
    is left -- the whiskers and the small-n tick strips -- with `yscale` "log"
    or "linear". Violins are the old log10-space spares and keep WALL_LIM.

    `clip_above` (linear only) keeps a single runaway group from flattening
    everything else: any drawn value above it is left out of the axis fit, so
    that group's box is cut by the frame and gets a `_clip_note` instead.
    """
    wide = width > 4
    fig = plt.figure(figsize=(width, 2.15 if wide else 1.8))  # column version shortened 2026-09-10
    ax = fig.add_axes((0.075 if wide else 0.145, 0.245 if wide else 0.235,
                       0.915 if wide else 0.845, 0.735 if wide else 0.745))

    step = len(methods) + 1.3  # one slot per planner, then a gap
    box_w = 0.78 if wide else 0.70
    drawn = []  # every y the reader will see; the box axis is fitted to it
    over = []   # (x, color, Q3, whisker) for groups `clip_above` cuts off

    def keep(vals):
        """The part of `vals` the axis is fitted to."""
        return [v for v in vals if clip_above is None or v <= clip_above]

    for gi, pct in enumerate(levels):
        for mi, (m, _, color, _) in enumerate(methods):
            vals = stats[m].get(pct, {}).get("wall", [])
            if not vals:
                continue
            x = gi * step + mi
            enough = len(vals) >= (MIN_BOX if kind == "box" else MIN_VIOLIN)
            if not enough:
                # every successful run, one tick each
                ax.plot([x] * len(vals),
                        vals if kind == "box" else np.log10(vals),
                        ls="none", marker="_", ms=3.2, mew=0.8, color=color,
                        zorder=3)
                drawn += keep(vals)
            elif kind == "box":
                bp = ax.boxplot([vals], positions=[x], widths=box_w,
                                patch_artist=True, showfliers=False,
                                medianprops=dict(color="0.15", lw=0.8),
                                whiskerprops=dict(lw=0.6, color="0.35"),
                                capprops=dict(lw=0.6, color="0.35"))
                patch = bp["boxes"][0]
                patch.set_facecolor(over_white(color, 0.45))
                patch.set_edgecolor(color)
                patch.set_linewidth(0.7)
                caps = [float(c.get_ydata()[0]) for c in bp["caps"]]
                drawn += keep(caps)
                if clip_above is not None and max(caps) > clip_above:
                    over.append((x, color, float(np.percentile(vals, 75)),
                                 max(caps)))
            else:
                lv = np.log10(vals)
                vp = ax.violinplot([lv], positions=[x], widths=box_w * 1.15,
                                   showextrema=False, showmedians=True)
                for body in vp["bodies"]:
                    body.set_facecolor(over_white(color, 0.45))
                    body.set_edgecolor(color)
                    body.set_linewidth(0.7)
                    body.set_alpha(1.0)  # violinplot defaults to 0.3; no alpha
                vp["cmedians"].set_color("0.15")
                vp["cmedians"].set_linewidth(0.8)

    ax.set_xlim(-1.1, (len(levels) - 1) * step + len(methods))
    ax.set_xticks([gi * step + (len(methods) - 1) / 2
                   for gi in range(len(levels))])
    ax.set_xticklabels([str(p) for p in levels])
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", labelpad=2)
    if kind == "violin":
        _wall_axes(ax, "log10data", WALL_LIM)
    elif yscale == "log":
        _wall_axes(ax, "log", (min(drawn) * 0.9, max(drawn) * 1.15))
    else:
        pad = 0.08 * (max(drawn) - min(drawn))
        _wall_axes(ax, "linear", (min(drawn) - pad, max(drawn) + pad))
        for note in over:
            _clip_note(ax, *note)

    handles = [plt.Rectangle((0, 0), 1, 1, fc=over_white(c, 0.45), ec=c,
                             lw=0.7, label=label)
               for _, label, c, _ in methods]
    fig.legend(handles=handles, loc="lower center", ncol=len(methods),
               bbox_to_anchor=(0.075 + 0.915 / 2 if wide else 0.145 + 0.845 / 2,
                               -0.008),
               frameon=False, fontsize=6.5, handlelength=1.1,
               handletextpad=0.4, columnspacing=1.1, borderaxespad=0.0)

    save(fig, stem)
    plt.close(fig)


# ---------------------------------------------------------------- Table I --
def table_rows(stats, methods, levels=TABLE_LEVELS):
    """Print the LaTeX rows: mean +- SD length ratio per level (no success
    rate -- that is what Figs. 3a/5a show)."""
    for m, label, _, _ in methods:
        cells = []
        for p in levels:
            d = stats[m][p]
            cells.append("--" if d["k"] == 0 else
                         f"{d['mean']:.2f}$\\pm${d['sd']:.2f}")
        print(f"    {label:<22s} & " + " & ".join(cells) + r" \\")


if __name__ == "__main__":
    base = level_stats(read_trials(SWEEP), METHODS)
    fig_success(base, METHODS, "fig3a_success_vs_constraint")
    fig_length(base, METHODS, "fig3b_length_vs_constraint",
               ylim=(0.95, 2.9), yticks=[1.0, 1.5, 2.0, 2.5])
    # 30 s cuts CL-GBT at 60% (whisker 124 s) and nothing else: the next
    # highest drawn value anywhere in this figure is Ours at 30%, 27.2 s.
    fig4(base, METHODS, LEVELS_5, "box", "fig4_wall_5level_box", 3.5, "linear",
         clip_above=30)
    # Same five levels as the paper's Fig. 4, on the log axis it used until
    # 2026-09-14 and with nothing clipped: CL-GBT's 124 s whisker is drawn in
    # full, at the cost of tick spacing the reader has to know is logarithmic.
    # Kept as the alternative to the linear+clipped version above.
    fig4(base, METHODS, LEVELS_5, "box", "fig4_wall_5level_box_log", 3.5, "log")
    # And the third combination: linear, also unclipped, so the axis has to
    # reach CL-GBT's 124 s whisker and the other four planners compress into
    # the bottom of the panel. Kept because it is the alternative the paper's
    # version is chosen against, not because it is readable.
    fig4(base, METHODS, LEVELS_5, "box", "fig4_wall_5level_box_linear", 3.5,
         "linear")
    fig4(base, METHODS, LVL_PCTS, "box", "fig4_wall_alllevel_box", 7.16, "log")
    fig4(base, METHODS, LEVELS_5, "violin", "fig4_wall_5level_violin", 3.5)
    fig4(base, METHODS, LVL_PCTS, "violin", "fig4_wall_alllevel_violin", 7.16)

    abl = level_stats(read_trials(ABL_SWEEP), ARMS)
    fig_success(abl, ARMS, "fig5a_abl_success_vs_constraint")
    fig_length(abl, ARMS, "fig5b_abl_length_vs_constraint",
               ylim=(0.95, 1.45), yticks=[1.0, 1.1, 1.2, 1.3, 1.4])
    fig4(abl, ARMS, LEVELS_5, "box", "fig6_abl_wall_5level_box", 3.5, "linear")
    fig4(abl, ARMS, LVL_PCTS, "box", "fig6_abl_wall_alllevel_box", 7.16, "linear")

    print("wrote figs 3-6 to", HERE)
    print("Table I rows (mean+-SD length ratio at",
          "/".join(map(str, TABLE_LEVELS)), "%):")
    table_rows(base, METHODS)
    print(r"    \midrule")
    table_rows(abl, ARMS)
