#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_success_on_y.{pdf,svg,eps,png}   <- the one in the paper
        fig3_success_on_x.{pdf,svg,eps,png}   <- same data, axes swapped
        cost against reliability: success rate against median path-length
        ratio. Each planner is one trajectory swept out as the constraint
        tightens; planner identity is marker shape, and the marker fills step
        through one shared 8-swatch YlGnBu scale saying which constraint level
        each point came from (dark = a loose bound, pale = a tight one).
        Both orientations are written so they can be compared at column width;
        whichever is not in main.tex is a spare.
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
from matplotlib.colors import BoundaryNorm, ListedColormap
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


def fig3(summary, success_on):
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
    summary = read_summary()
    fig3(summary, "y")
    fig3(summary, "x")
    fig4(read_wall())
    print("wrote fig3/fig4 to", HERE)
