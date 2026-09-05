#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_length_vs_constraint.{pdf,png}
        success rate vs. constraint level, one line per planner; the color of
        each segment/marker runs along that planner's own two-color ramp and
        encodes the median path-length ratio at that level (palette hue =
        short, a hue ~156 deg away and darker = long).
Fig. 4  fig4_wallclock.{pdf,png}
        box plot of wall-clock time per planner over all successful runs
        (log scale; CL-GBT's tail spans 12.5 -> 730 s).

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
from matplotlib.colors import LinearSegmentedColormap, PowerNorm
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/baseline_2026-08-17"))

# Fixed identity -> color assignment (dataviz reference palette, slots 1-5).
# Keep this order/color per method in EVERY figure of the paper.
METHODS = [
    ("hexspline_cl", "Ours",       "#2a78d6", "o"),
    ("formation",    "Formation",  "#eb6834", "s"),
    ("sequential",   "Sequential", "#1baf7a", "^"),
    ("clgbt",        "CL-GBT",     "#eda100", "D"),
    ("greedy",       "Greedy",     "#e87ba4", "v"),
]

# Fig. 3 encodes median length ratio as a two-color ramp per method. Each ramp
# starts on the method's fixed palette color (paths at the reference length) and
# travels ~156 deg around the OKLCh hue circle while dropping 0.27 in lightness,
# so the two ends are unmistakably different colors: blue -> plum-brown, orange
# -> violet, green -> deep crimson, yellow -> blue, pink -> teal.
#
# The rotations were searched, not picked. What has to hold is that two methods
# at *similar* length ratios stay apart, and this set is the best found on that
# measure: worst same-level CVD dE 6.1, against 5.8 for a 41 deg rotation and
# 6.1 for the flat palette -- i.e. nearly 4x the hue travel costs nothing.
# Comparing every color to every color of every other method (a method at t=0
# does share the plot with another at t=1) no scheme survives: 1.4 here, 2.0 at
# 41 deg, and 3.1 is the best any rotation achieves. That bar is unsatisfiable
# for 25 colors on a 5-hue budget, so identity rests on marker shape and on each
# line being a continuous path -- not on color alone.
#
# Stops were stepped in OKLCh, so RGB interpolation between them stays on the
# perceptual path; 7 of them because the arcs are long.
RAMPS = {
    "hexspline_cl": ("#2a78d6", "#645cbc", "#784693", "#7a3665",
                     "#702c3b", "#5d2919", "#452703"),
    "formation":    ("#eb6834", "#e14d56", "#ce3774", "#b32990",
                     "#9220a7", "#6c19b7", "#4014bc"),
    "sequential":   ("#1baf7a", "#5b9a3e", "#7c810e", "#87680b",
                     "#8b5008", "#922b06", "#850c31"),
    "clgbt":        ("#eda100", "#b5a816", "#74a940", "#14a26c",
                     "#118e87", "#0e7b92", "#1066a0"),
    "greedy":       ("#e87ba4", "#c177bd", "#9476c4", "#6576ba",
                     "#3673a1", "#0b6c7f", "#0d615b"),
}
CMAPS = {m: LinearSegmentedColormap.from_list(m, stops)
         for m, stops in RAMPS.items()}

# Length-ratio -> ramp position. The medians pile up in 1.1-1.4 with a thin
# tail out to 2.44 (sequential at 30%), so a sqrt norm spends the ramp where
# the data is; the legend strip carries the resulting non-uniform ticks.
LEN_MIN, LEN_MAX = 1.10, 2.45
LEN_NORM = PowerNorm(gamma=0.5, vmin=LEN_MIN, vmax=LEN_MAX, clip=True)
LEN_TICKS = [1.2, 1.4, 1.7, 2.0, 2.4]

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
})


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
def gradient_line(ax, xs, ys, vals, cmap, lw, zorder):
    """Polyline whose color follows `vals` (one per vertex) along the ramp."""
    n = 32  # sub-segments per data interval; enough that the ramp reads smooth
    t = np.linspace(0, 1, n + 1)
    x, y, v = (np.concatenate([np.interp(t, [0, 1], [a[i], a[i + 1]])[:-1]
                               for i in range(len(a) - 1)] + [a[-1:]])
               for a in (np.asarray(xs, float), np.asarray(ys, float),
                         np.asarray(vals, float)))
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, norm=LEN_NORM, lw=lw,
                        capstyle="round", zorder=zorder)
    lc.set_array(0.5 * (v[:-1] + v[1:]))
    ax.add_collection(lc)


def ramp_legend(ax):
    """Legend + colorbar in one: a ramp strip per method, shared value axis."""
    grad = LEN_NORM(np.linspace(LEN_MIN, LEN_MAX, 256))
    for i, (m, _, _, marker) in enumerate(METHODS):
        ax.imshow(CMAPS[m](grad)[None], aspect="auto", interpolation="bilinear",
                  extent=(LEN_MIN, LEN_MAX, i + 0.33, i - 0.33), zorder=2)
        ax.plot(-0.022, i, marker=marker, ms=2.8, mec="none", color="0.25",
                transform=ax.get_yaxis_transform(), clip_on=False, zorder=3)

    ax.set_ylim(len(METHODS) - 0.5, -0.5)
    ax.set_xlim(LEN_MIN, LEN_MAX)
    ax.set_yticks(range(len(METHODS)))
    ax.set_yticklabels([label for _, label, _, _ in METHODS], fontsize=6.5)
    ax.tick_params(axis="y", length=0, pad=11)
    ax.set_xticks(LEN_TICKS)
    ax.set_xticklabels([f"{t:.1f}" for t in LEN_TICKS], fontsize=6.5)
    ax.tick_params(axis="x", length=2.0, width=0.6, pad=1.5, colors="0.25")
    ax.set_xlabel(r"median length / $L_\mathrm{ref}$", fontsize=7, labelpad=1.5)
    for s in ax.spines.values():
        s.set_visible(False)


def fig3(summary):
    fig = plt.figure(figsize=(3.5, 2.75))
    ax = fig.add_axes((0.108, 0.395, 0.878, 0.585))
    cax = fig.add_axes((0.255, 0.120, 0.660, 0.132))
    pcts = sorted(next(iter(summary.values())).keys(), reverse=True)  # 100..30

    # Ours is drawn widest and *under* the others, so where a baseline sits on
    # top of it (they share y = 100% down to the 80% level) both still read;
    # its markers go last so the emphasized series stays legible.
    for m, _, _, marker in METHODS:
        rows = summary[m]
        ratio = [rows[p][0] for p in pcts]
        rate = [rows[p][1] for p in pcts]
        primary = m == "hexspline_cl"
        gradient_line(ax, pcts, rate, ratio, CMAPS[m],
                      lw=1.9 if primary else 1.1, zorder=2 if primary else 3)
        ax.scatter(pcts, rate, c=ratio, cmap=CMAPS[m], norm=LEN_NORM,
                   marker=marker, s=14 if primary else 12, linewidths=0.45,
                   edgecolors="white", zorder=5 if primary else 4)

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

    ramp_legend(cax)
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.pdf"))
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.png"), dpi=300)
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
                    flierprops=dict(marker=".", ms=2.0, mfc="0.55", mec="none",
                                    alpha=0.6))
    for patch, (_, _, color, _) in zip(bp["boxes"], METHODS):
        patch.set_facecolor(color)
        patch.set_alpha(0.45)
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
    fig.savefig(os.path.join(HERE, "fig4_wallclock.pdf"))
    fig.savefig(os.path.join(HERE, "fig4_wallclock.png"), dpi=300)
    plt.close(fig)


if __name__ == "__main__":
    fig3(read_summary())
    fig4(read_wall())
    print("wrote fig3/fig4 to", HERE)
