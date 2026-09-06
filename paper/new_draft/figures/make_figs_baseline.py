#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_length_vs_constraint.{pdf,svg,eps,png}   <- the one in the paper
        success rate against constraint level, one line per planner. Identity
        is marker shape; the line itself carries a shared viridis ramp (green =
        at the reference length, dark violet = the worst detour) encoding the
        median path-length ratio along it, keyed by the vertical bar at the
        right. Planner key runs along the bottom.
        fig3_success_on_y.{pdf,svg,eps,png}   <- spare, both outcomes on axes
        fig3_success_on_x.{pdf,svg,eps,png}   <- spare, same but transposed
        These two put success rate against median length ratio and hand the
        constraint level to a discrete 8-swatch YlGnBu fill instead. Generated
        so the three can be compared at column width.
Fig. 4  fig4_wall_5level_box.{pdf,svg,eps,png}       <- the one in the paper
        fig4_wall_alllevel_box.{pdf,svg,eps,png}     <- spare, all 8 levels
        fig4_wall_5level_violin.{pdf,svg,eps,png}    <- spare
        fig4_wall_alllevel_violin.{pdf,svg,eps,png}  <- spare
        wall-clock time per planner, split by constraint level rather than
        pooled over the sweep -- pooling hid that only CL-GBT's cost moves with
        the bound. Log scale. The 5-level cuts fit a column; the 8-level ones
        need the full text width. Groups with too few successes to summarize
        are drawn as their individual runs instead; see MIN_BOX / MIN_VIOLIN.

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
from matplotlib.collections import LineCollection
from matplotlib.colors import (BoundaryNorm, LinearSegmentedColormap,
                               ListedColormap, PowerNorm)
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/baseline_2026-08-17"))

# Fixed identity -> color assignment (dataviz reference palette, slots 1-5).
# Keep this order/color per method in EVERY figure of the paper.
METHODS = [
    ("hexspline_cl", "Ours",       "#2a78d6", "D"),
    ("formation",    "Formation",  "#eb6834", "s"),
    ("sequential",   "Sequential", "#1baf7a", "^"),
    ("clgbt",        "CL-GBT",     "#eda100", "o"),
    ("greedy",       "Greedy",     "#e87ba4", "v"),
]

# Fig. 3 puts both outcomes on the axes -- success rate against median length
# ratio -- and hands the sweep variable, the constraint level, to color. So the
# figure reads as a cost/reliability trade: the good corner is high success and
# short path, top left when success is on y and top right when it is on x (that
# one inverts its length axis so shorter still means higher).
#
# The scale lives in the marker FILLS, not the line. The sweep has exactly eight
# levels, so it gets exactly eight swatches: a discrete step is easier to match
# back to the key than a point on a continuous ramp, and a filled 5 pt glyph
# carries a flat color far better than a 1 pt line does. That leaves the lines
# free to be pure structure -- every baseline is the same dotted gray, so the
# eye groups them as "the others" and identity falls to marker shape, with Ours
# the one solid black trajectory through them.
#
# The color column above is still the paper-wide identity color and is what
# Fig. 4 (and Figs. 5-6) paint with; Fig. 3 no longer uses it.
# YlGnBu sampled at the eight sweep levels. DARK = the higher number, the
# ordinary sequential convention: 100% of U_ref is dark navy and the scale runs
# back through blue, teal and green to a pale yellow-green at 30%. Its measured
# separation is mid-pack (worst-case CVD dE 4.5 over normal/protan/deutan,
# against 8.9 for cividis and 6.6 for viridis; see figures/figure3/README.md),
# which is affordable here because the levels are also ordered along each line
# -- a reader who cannot separate two adjacent swatches can still read which
# came first.
LVL_PCTS = [100, 90, 80, 70, 60, 50, 40, 30]
LVL_CMAP = ListedColormap(
    plt.get_cmap("YlGnBu")(np.linspace(0.10, 1.0, len(LVL_PCTS))))
# bin edges at 25, 35, ... 105, so each level falls in the middle of its swatch
LVL_BOUNDS = np.arange(min(LVL_PCTS) - 5, max(LVL_PCTS) + 6, 10)
LVL_NORM = BoundaryNorm(LVL_BOUNDS, LVL_CMAP.N)

# --- the paper figure's scale: median length ratio, painted along the line ---
# viridis, reversed and cut off below its yellow end. Reversed so DARK = the
# long detour; cut at 0.80 (#7ad151) so the ramp tops out at a clear green
# rather than running on into yellow, which no thin line carries on white
# paper. So: green = paths at the reference length, through teal and blue, to
# viridis's own dark violet for the worst detours. Perceptually uniform and
# CVD-safe across the whole span.
LEN_CMAP = LinearSegmentedColormap.from_list(
    "viridis_r_trim", plt.get_cmap("viridis")(np.linspace(0.80, 0.0, 256)))
LEN_MIN, LEN_MAX = 1.10, 2.45
# sqrt: most of the data sits in 1.1-1.5, so a linear ramp would spend most of
# its range on the two planners that blow out past 2.0
LEN_NORM = PowerNorm(gamma=0.5, vmin=LEN_MIN, vmax=LEN_MAX, clip=True)
LEN_TICKS = [1.2, 1.4, 1.7, 2.0, 2.4]

# Glyphs are identity only, so they take one neutral fill and stay out of the
# ramp's way. A light warm neutral is the one fill that stays legible against
# every step from green to dark violet.
MARKER_FACE = "#dfddd6"

# Ours gets its own fill so it is findable at a glance. A plain yellow works
# here where gold did not, and for a reason worth knowing: the ramp's short end
# is a mid-lightness green (#7ad151), so separation is mostly a question of
# lightness. The golds and ambers sit at that same lightness and collide
# (#ffc300 dE 2.5, #e8c800 2.6, #f2d024 4.4); a light yellow clears it easily.
# #ffdd00 measures worst-case CVD dE 8.0 against the ramp -- better than the
# gold it replaced (7.9) and the coral before that (7.9) -- and dE 17.3 from
# the neutral glyph fill. It is only 1.35:1 against white, which is why the
# 0.15 ring matters: on a yellow glyph the edge, not the fill, does the work of
# holding the shape.
PRIMARY_FACE = "#ffdd00"

# --- the spare figures' scale: constraint level, in discrete marker fills ---
# Identity, all of it non-color: one dotted gray for every baseline, so they
# read as a single background population, and solid black for Ours.
BASE_GRAY = "0.55"
BASE_DASH = (1.1, 1.35)
PRIMARY_COLOR = "0.0"
LEGEND_FACE = "0.92"  # the key shows shapes; fills are the colorbar's job

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


def read_summary():
    """method -> pct -> (median_length_ratio, success_rate)"""
    out = {}
    with open(os.path.join(SWEEP, "summary.csv")) as f:
        for r in csv.DictReader(f):
            out.setdefault(r["method"], {})[int(r["pct"])] = (
                float(r["median_length_ratio"]),
                float(r["success_rate"]),
            )
    return out


def read_wall():
    """method -> pct -> [wall_s of successful runs]"""
    out = {}
    for m, _, _, _ in METHODS:
        with open(os.path.join(SWEEP, m, "trials.csv")) as f:
            per_level = {}
            for r in csv.DictReader(f):
                if r["success"] == "true":
                    per_level.setdefault(int(r["pct"]), []).append(
                        float(r["wall_s"]))
            out[m] = per_level
    return out


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------------- Fig. 3 ----
def length_bar(ax):
    r"""The length-ratio scale, lying under the plot.

    It was tried standing in the right margin, and it does not fit: at column
    width the y label takes ~0.38 in and the bar plus its ticks and rotated
    label another ~0.45 in, which leaves the plot 2.67 in against the 3.07 in
    it has here. Growing the figure past \linewidth is not a way out either --
    \includegraphics scales it straight back down and shrinks the type with it.
    """
    # pcolormesh, not imshow: imshow embeds the bar as a raster block, which
    # arrives in Illustrator as a non-editable, resolution-locked image.
    edges = np.linspace(LEN_MIN, LEN_MAX, 257)
    mesh = ax.pcolormesh(edges, [0, 1], (0.5 * (edges[:-1] + edges[1:]))[None],
                         cmap=LEN_CMAP, norm=LEN_NORM, shading="flat")
    mesh.set_edgecolor("face")  # else hairline seams between quads in vector out
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlim(LEN_MIN, LEN_MAX)
    ax.set_xticks(LEN_TICKS)
    ax.set_xticklabels([f"{t:.1f}" for t in LEN_TICKS], fontsize=6.5)
    ax.tick_params(axis="x", length=2.0, width=0.6, pad=1.5, colors="0.25")
    ax.set_xlabel(r"median length / $L_\mathrm{ref}$", fontsize=7, labelpad=1.5)
    for sp in ax.spines.values():
        sp.set_linewidth(0.5)
        sp.set_color("0.55")


def gradient_line(ax, xs, ys, vals, lw, zorder):
    """Polyline whose color follows `vals` (one per vertex) along the ramp."""
    n = 32  # sub-segments per data interval; enough that the ramp reads smooth
    t = np.linspace(0, 1, n + 1)
    x, y, v = (np.concatenate([np.interp(t, [0, 1], [a[i], a[i + 1]])[:-1]
                               for i in range(len(a) - 1)] + [a[-1:]])
               for a in (np.asarray(xs, float), np.asarray(ys, float),
                         np.asarray(vals, float)))
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1),
                        cmap=LEN_CMAP, norm=LEN_NORM, lw=lw,
                        capstyle="round", zorder=zorder)
    lc.set_array(0.5 * (v[:-1] + v[1:]))
    ax.add_collection(lc)


def fig3(summary):
    # the plot itself keeps 08_'s dimensions exactly -- 3.073 x 1.696 in. The
    # figure is taller instead, and the two keys stack in the space that buys:
    # length scale under the x label, planner key under that.
    fig = plt.figure(figsize=(3.5, 2.75))
    ax = fig.add_axes((0.108, 0.375, 0.878, 1.696 / 2.75))
    cax = fig.add_axes((0.305, 0.208, 0.600, 0.046))
    pcts = sorted(next(iter(summary.values())).keys(), reverse=True)  # 100..30

    handles = []
    for m, label, _, marker in METHODS:
        rows = summary[m]
        ratio = [rows[p][0] for p in pcts]
        rate = [rows[p][1] for p in pcts]
        primary = m == "hexspline_cl"
        gradient_line(ax, pcts, rate, ratio, 2.2 if primary else 1.4, zorder=2)
        # the primary's line stays *under* the others (they share y = 100% with
        # it down to the 80% level), but its glyphs sit on top of theirs
        ax.plot(pcts, rate, ls="none", marker=marker, ms=4.4 if primary else 3.8,
                mfc=PRIMARY_FACE if primary else MARKER_FACE, mec="0.15",
                mew=0.6, zorder=6 if primary else 4)
        # marker only: shape is the whole identity channel, so a line swatch
        # here would just imply a line color the plot does not have
        handles.append(plt.Line2D([], [], ls="none", marker=marker,
                                  ms=4.4 if primary else 3.8, mew=0.6,
                                  mfc=PRIMARY_FACE if primary else MARKER_FACE,
                                  mec="0.15", label=label))

    ax.invert_xaxis()  # constraint tightens to the right
    ax.set_xticks(pcts)
    ax.set_xlim(103, 27)
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", labelpad=2)
    ax.set_ylabel("success rate (%)")
    ax.set_ylim(-0.04, 1.07)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0", "25", "50", "75", "100"])
    ax.grid(axis="y", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)
    # one horizontal row under the plot, centred on the axes rather than the
    # figure -- the colorbar occupies the right margin
    fig.legend(handles=handles, loc="lower center", ncol=len(METHODS),
               bbox_to_anchor=(0.108 + 0.878 / 2, -0.004), frameon=False,
               fontsize=6.5, handlelength=1.0, handletextpad=0.35,
               columnspacing=1.1, borderaxespad=0.0)

    length_bar(cax)
    save(fig, "fig3_length_vs_constraint")
    plt.close(fig)


def level_bar(ax):
    """The shared constraint-level key: one swatch per sweep level."""
    # The C row has to ASCEND with x, because LVL_BOUNDS does. Handing it
    # LVL_PCTS (which runs 100..30) put every block under the wrong tick -- the
    # swatch labelled 100 was painted with level 30's color, so the whole
    # figure read backwards. Keep these two in the same order.
    levels = sorted(LVL_PCTS)
    # pcolormesh, not imshow: imshow embeds the bar as a raster block, which
    # arrives in Illustrator as a non-editable, resolution-locked image.
    ax.pcolormesh(LVL_BOUNDS, [0, 1], np.array(levels, float)[None],
                  cmap=LVL_CMAP, norm=LVL_NORM, shading="flat",
                  edgecolors="w", linewidth=0.6)  # white gutters = eight blocks
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlim(LVL_BOUNDS[0], LVL_BOUNDS[-1])  # 30 at the left, 100 at the right
    ax.set_xticks(levels)
    ax.set_xticklabels([str(p) for p in levels], fontsize=6.5)
    ax.tick_params(axis="x", length=0, pad=2.0, colors="0.25")
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", fontsize=7,
                  labelpad=1.5)
    for s in ax.spines.values():
        s.set_linewidth(0.5)
        s.set_color("0.55")


def fig3_scatter(summary, success_on):
    """One figure; `success_on` is "y" or "x" and picks which axis it takes."""
    fig = plt.figure(figsize=(3.5, 2.75))
    ax = fig.add_axes((0.115 if success_on == "y" else 0.128, 0.285,
                       0.871 if success_on == "y" else 0.858, 0.685))
    cax = fig.add_axes((0.305, 0.108, 0.600, 0.048))

    assert sorted(next(iter(summary.values())), reverse=True) == LVL_PCTS, \
        "sweep levels changed -- LVL_PCTS and the eight swatches must follow"

    handles = []
    for m, label, _, marker in METHODS:
        rows = summary[m]
        length = [rows[p][0] for p in LVL_PCTS]
        rate = [100 * rows[p][1] for p in LVL_PCTS]
        xs, ys = (length, rate) if success_on == "y" else (rate, length)
        primary = m == "hexspline_cl"
        color = PRIMARY_COLOR if primary else BASE_GRAY
        style = dict(color=color, lw=1.5 if primary else 0.9,
                     dashes=(None, None) if primary else BASE_DASH)
        ax.plot(xs, ys, zorder=2, solid_capstyle="round",
                dash_capstyle="round", **style)
        # fills are the level scale and nothing else touches them; every marker
        # keeps the same dark ring
        ax.scatter(xs, ys, c=LVL_PCTS, cmap=LVL_CMAP, norm=LVL_NORM,
                   marker=marker, s=36 if primary else 25, edgecolors="0.15",
                   linewidths=0.6, zorder=6 if primary else 4)
        handles.append(plt.Line2D([], [], marker=marker,
                                  ms=5.4 if primary else 4.6, mfc=LEGEND_FACE,
                                  mec="0.15", mew=0.6, label=label, **style))

    length_lim, length_ticks = (1.05, 2.55), [1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4]
    length_label = r"median length / $L_\mathrm{ref}$"
    rate_lim, rate_ticks, rate_label = (-4, 107), [0, 25, 50, 75, 100], \
        r"success rate (%)"
    if success_on == "y":
        ax.set_xlim(*length_lim); ax.set_xticks(length_ticks)
        ax.set_xlabel(length_label, labelpad=2)
        ax.set_ylim(*rate_lim); ax.set_yticks(rate_ticks)
        ax.set_ylabel(rate_label)
    else:
        ax.set_xlim(*rate_lim); ax.set_xticks(rate_ticks)
        ax.set_xlabel(rate_label, labelpad=2)
        ax.set_ylim(*length_lim); ax.set_yticks(length_ticks)
        ax.set_ylabel(length_label)
        ax.invert_yaxis()  # shorter paths stay higher up whichever axis they use
    ax.grid(color="0.88", lw=0.5, zorder=0)  # both axes: each is a measurement
    ax.set_axisbelow(True)
    despine(ax)
    # into the corner nothing occupies -- no planner solves nearly everything
    # while also walking twice the reference distance. Which corner that is
    # flips with the layout, since inverting the length axis moves it.
    loc = "upper right" if success_on == "y" else "lower right"
    ax.legend(handles=handles, loc=loc, frameon=False, fontsize=6.5,
              handlelength=1.9, handletextpad=0.5, borderaxespad=0.2,
              labelspacing=0.32)

    level_bar(cax)
    save(fig, f"fig3_success_on_{success_on}")
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 4 ----
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
WALL_TICKS = [10, 20, 50, 100, 200, 500]
WALL_LIM = (11, 900)


def _wall_axes(ax, levels, log_axis):
    """Shared y scale. `log_axis` False means the data arrived as log10."""
    if log_axis:
        ax.set_yscale("log")
        ax.set_yticks(WALL_TICKS)
        ax.set_yticklabels([str(t) for t in WALL_TICKS])
        ax.yaxis.set_minor_formatter(NullFormatter())
        ax.set_ylim(*WALL_LIM)
    else:
        # violins are built on log10(wall) against a linear axis: a KDE has to
        # be estimated in the space the reader sees it in, and on a log axis a
        # linear-space KDE turns every distribution into a spike at the bottom
        ax.set_yticks(np.log10(WALL_TICKS))
        ax.set_yticklabels([str(t) for t in WALL_TICKS])
        ax.set_ylim(*np.log10(WALL_LIM))
    ax.set_ylabel("wall-clock time (s)")
    ax.grid(axis="y", which="major", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)


def fig4(wall, levels, kind, stem, width):
    """One grouped figure: `kind` is "box" or "violin"."""
    wide = width > 4
    fig = plt.figure(figsize=(width, 2.15 if wide else 2.05))
    ax = fig.add_axes((0.075 if wide else 0.145, 0.245 if wide else 0.235,
                       0.915 if wide else 0.845, 0.735 if wide else 0.745))

    step = len(METHODS) + 1.3  # one slot per planner, then a gap
    box_w = 0.78 if wide else 0.70
    for gi, pct in enumerate(levels):
        for mi, (m, _, color, _) in enumerate(METHODS):
            vals = wall[m].get(pct, [])
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
            elif kind == "box":
                bp = ax.boxplot([vals], positions=[x], widths=box_w,
                                patch_artist=True,
                                medianprops=dict(color="0.15", lw=0.8),
                                whiskerprops=dict(lw=0.6, color="0.35"),
                                capprops=dict(lw=0.6, color="0.35"),
                                flierprops=dict(marker=".", ms=1.6, mec="none",
                                                mfc=over_white("0.55", 0.6)))
                patch = bp["boxes"][0]
                patch.set_facecolor(over_white(color, 0.45))
                patch.set_edgecolor(color)
                patch.set_linewidth(0.7)
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

    ax.set_xlim(-1.1, (len(levels) - 1) * step + len(METHODS))
    ax.set_xticks([gi * step + (len(METHODS) - 1) / 2
                   for gi in range(len(levels))])
    ax.set_xticklabels([str(p) for p in levels])
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", labelpad=2)
    _wall_axes(ax, levels, log_axis=(kind == "box"))

    handles = [plt.Rectangle((0, 0), 1, 1, fc=over_white(c, 0.45), ec=c,
                             lw=0.7, label=label)
               for _, label, c, _ in METHODS]
    fig.legend(handles=handles, loc="lower center", ncol=len(METHODS),
               bbox_to_anchor=(0.075 + 0.915 / 2 if wide else 0.145 + 0.845 / 2,
                               -0.008),
               frameon=False, fontsize=6.5, handlelength=1.1,
               handletextpad=0.4, columnspacing=1.1, borderaxespad=0.0)

    save(fig, stem)
    plt.close(fig)


if __name__ == "__main__":
    summary = read_summary()
    fig3(summary)
    fig3_scatter(summary, "y")
    fig3_scatter(summary, "x")
    wall = read_wall()
    for kind in ("box", "violin"):
        fig4(wall, LEVELS_5, kind, f"fig4_wall_5level_{kind}", 3.5)
        fig4(wall, LVL_PCTS, kind, f"fig4_wall_alllevel_{kind}", 7.16)
    print("wrote fig3/fig4 to", HERE)
