#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_length_vs_constraint.{pdf,svg,eps,png}
        cost against reliability: median path-length ratio (y, inverted, so
        shorter is higher) against success rate (x). Each planner is one
        trajectory swept out as the constraint tightens; planner identity is
        marker shape, and the line carries one shared viridis ramp encoding
        which constraint level each point came from (green = loose, dark
        violet = tight). Top right is best.
Fig. 4  fig4_wallclock.{pdf,svg,eps,png}
        box plot of wall-clock time per planner over all successful runs
        (log scale; CL-GBT's tail spans 12.5 -> 730 s).

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
from matplotlib.colors import LinearSegmentedColormap, Normalize
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
# figure reads as a cost/reliability trade: y is inverted, so up is a shorter
# path and right is a higher success rate, and the best corner is top right.
# Each planner is a trajectory through that plane, and the ramp says where along
# the sweep you are, which position alone cannot (Greedy jumps from 90% to 26%
# success between two adjacent levels).
#
# Identity is carried by marker shape alone; the markers take a neutral fill so
# they read as tags on the line rather than a second color channel. (Dash
# patterns are not an option here -- a gradient line is a LineCollection of
# sub-segments each shorter than a dash period, so a linestyle on it renders
# solid.)
#
# The color column above is still the paper-wide identity color and is what
# Fig. 4 (and Figs. 5-6) paint with; Fig. 3 no longer uses it.
# viridis, cut off below its yellow end. Cut at 0.74 (#58c765) so the ramp tops
# out at a clear green rather than running on into yellow-green and yellow,
# which no thin line carries on white paper. DARK = the tight constraint: the
# loose levels are green, and the ramp runs through teal and blue to viridis's
# own dark violet at 30%. Perceptually uniform and CVD-safe across the whole
# span. Tried and rejected against this data: cividis (its midtones are the same
# gray as the marker fill), magma and inferno (their warm end collides with the
# coral primary glyph below), and single-hue Blues (its light end is too faint
# for a 1.4 pt line).
LVL_CMAP = LinearSegmentedColormap.from_list(
    "viridis_trim", plt.get_cmap("viridis")(np.linspace(0.0, 0.74, 256)))

# Glyphs are identity only, so they take one neutral fill and stay out of the
# ramp's way. Painting them the per-planner palette colors was tried and reads
# worse: the palette's green and blue land inside the ramp and are read as
# values, not labels. A light warm neutral is the one fill that stays legible
# against every step from green to dark violet.
MARKER_FACE = "#dfddd6"

# Ours gets its own fill so it is findable at a glance. Coral is the only
# candidate that clears both gates: worst CVD dE 8.2 against the ramp (gold
# managed 4.8 -- it collides with the green end under deuteranopia) and 3.7:1
# against white, the only one of the candidates that stands on its own. It is
# warm, so it cannot be mistaken for a step of a green-to-violet ramp.
PRIMARY_FACE = "#e8503a"

LVL_MIN, LVL_MAX = 30, 100
LVL_NORM = Normalize(vmin=LVL_MIN, vmax=LVL_MAX)

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
    """PDF for LaTeX, SVG and EPS for Illustrator, PNG to eyeball.

    Note for whoever opens the SVG: Fig. 3's gradient lines arrive as a run of
    short stroked segments, not one path -- that is how a per-vertex color ramp
    has to be expressed. Group them before moving a line around.
    """
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
    """method -> [wall_s of successful runs]"""
    out = {}
    for m, _, _, _ in METHODS:
        with open(os.path.join(SWEEP, m, "trials.csv")) as f:
            out[m] = [float(r["wall_s"]) for r in csv.DictReader(f)
                      if r["success"] == "true"]
    return out


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------------- Fig. 3 ----
def level_bar(ax, pcts):
    """The shared constraint-level scale, running loose (left) to tight."""
    # pcolormesh, not imshow: imshow embeds the bar as a raster block, which
    # arrives in Illustrator as a non-editable, resolution-locked image.
    edges = np.linspace(LVL_MIN, LVL_MAX, 257)
    mesh = ax.pcolormesh(edges, [0, 1], (0.5 * (edges[:-1] + edges[1:]))[None],
                         cmap=LVL_CMAP, norm=LVL_NORM, shading="flat")
    mesh.set_edgecolor("face")  # else hairline seams between quads in vector out
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_xlim(LVL_MAX, LVL_MIN)  # constraint tightens to the right
    ax.set_xticks(pcts)
    ax.set_xticklabels([str(p) for p in pcts], fontsize=6.5)
    ax.tick_params(axis="x", length=2.0, width=0.6, pad=1.5, colors="0.25")
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)", fontsize=7,
                  labelpad=1.5)
    for s in ax.spines.values():
        s.set_linewidth(0.5)
        s.set_color("0.55")


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
                        cmap=LVL_CMAP, norm=LVL_NORM, lw=lw,
                        capstyle="round", zorder=zorder)
    lc.set_array(0.5 * (v[:-1] + v[1:]))
    ax.add_collection(lc)


def fig3(summary):
    fig = plt.figure(figsize=(3.5, 2.75))
    ax = fig.add_axes((0.128, 0.285, 0.858, 0.685))
    cax = fig.add_axes((0.305, 0.108, 0.600, 0.048))
    pcts = sorted(next(iter(summary.values())).keys(), reverse=True)  # 100..30

    handles = []
    for m, label, _, marker in METHODS:
        rows = summary[m]
        ratio = [rows[p][0] for p in pcts]
        rate = [100 * rows[p][1] for p in pcts]
        primary = m == "hexspline_cl"
        lw = 2.2 if primary else 1.4
        gradient_line(ax, rate, ratio, pcts, lw, zorder=2)
        # the primary's line stays *under* the others (they converge on the
        # top-right corner with it), but its glyphs sit on top of theirs
        ax.plot(rate, ratio, ls="none", marker=marker, ms=4.4 if primary else 3.8,
                mfc=PRIMARY_FACE if primary else MARKER_FACE, mec="0.15",
                mew=0.6, zorder=6 if primary else 4)
        # marker only: shape is the whole identity channel, so a line swatch
        # here would just imply a line color the plot does not have
        handles.append(plt.Line2D([], [], ls="none", marker=marker,
                                  ms=4.4 if primary else 3.8, mew=0.6,
                                  mfc=PRIMARY_FACE if primary else MARKER_FACE,
                                  mec="0.15", label=label))

    ax.set_xlim(-4, 107)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel(r"success rate (%)", labelpad=2)
    ax.set_ylabel(r"median length / $L_\mathrm{ref}$")
    ax.set_ylim(1.05, 2.55)
    ax.set_yticks([1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4])
    ax.invert_yaxis()  # shorter paths higher up, so the best corner is top right
    ax.grid(color="0.88", lw=0.5, zorder=0)  # both axes: x is a measurement now
    ax.set_axisbelow(True)
    despine(ax)
    # lower right is the one empty quadrant -- nothing is both slow-to-succeed
    # and long
    ax.legend(handles=handles, loc="lower right", frameon=False, fontsize=6.5,
              handlelength=1.0, handletextpad=0.4, borderaxespad=0.2,
              labelspacing=0.32)

    level_bar(cax, pcts)
    save(fig, "fig3_length_vs_constraint")
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 4 ----
def fig4(wall):
    fig, ax = plt.subplots(figsize=(3.5, 1.7))
    data = [wall[m] for m, _, _, _ in METHODS]
    pos = range(1, len(METHODS) + 1)

    bp = ax.boxplot(data, positions=list(pos), widths=0.55, patch_artist=True,
                    medianprops=dict(color="0.15", lw=1.0),
                    whiskerprops=dict(lw=0.7, color="0.35"),
                    capprops=dict(lw=0.7, color="0.35"),
                    flierprops=dict(marker=".", ms=2.0, mec="none",
                                    mfc=over_white("0.55", 0.6)))
    for patch, (_, _, color, _) in zip(bp["boxes"], METHODS):
        patch.set_facecolor(over_white(color, 0.45))
        patch.set_edgecolor(color)
        patch.set_linewidth(0.9)

    ax.set_yscale("log")
    ax.set_yticks([10, 20, 50, 100, 200, 500])
    ax.set_yticklabels(["10", "20", "50", "100", "200", "500"])
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.set_ylim(11, 900)
    ax.set_ylabel("wall-clock time (s)")
    ax.set_xticks(list(pos))
    ax.set_xticklabels([f"{label}\n(n={len(wall[m])})" for m, label, _, _ in METHODS],
                       fontsize=6.5)
    ax.grid(axis="y", which="major", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)

    fig.tight_layout(pad=0.25)
    save(fig, "fig4_wallclock")
    plt.close(fig)


if __name__ == "__main__":
    fig3(read_summary())
    fig4(read_wall())
    print("wrote fig3/fig4 to", HERE)
