#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_length_vs_constraint.{pdf,png}
        success rate vs. constraint level, one line per planner. Planner
        identity is marker shape + dash pattern; marker fill is one shared
        viridis ramp encoding the median path-length ratio at that level.
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
from matplotlib.colors import PowerNorm
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/baseline_2026-08-17"))

# Fixed identity -> color assignment (dataviz reference palette, slots 1-5).
# Keep this order/color per method in EVERY figure of the paper.
METHODS = [
    ("hexspline_cl", "Ours",       "#2a78d6", "o", (0, ())),
    ("formation",    "Formation",  "#eb6834", "s", (0, (4.5, 1.5))),
    ("sequential",   "Sequential", "#1baf7a", "^", (0, (0.8, 1.5))),
    ("clgbt",        "CL-GBT",     "#eda100", "D", (0, (5, 1.5, 1, 1.5))),
    ("greedy",       "Greedy",     "#e87ba4", "v", (0, (1.8, 1.5))),
]

# Fig. 3 splits the two channels instead of overloading color with both. Length
# ratio gets ONE shared ramp, so a marker's color means the same thing on every
# line and the planners are directly comparable -- which per-planner ramps could
# not do. Identity moves entirely to marker shape + dash pattern, and the line
# itself goes neutral gray (Ours darker and solid, so it still leads).
#
# This is what five per-planner ramps could never buy: 25 colors on a 5-hue
# budget are not mutually distinguishable at any rotation (best worst-pair CVD
# dE 3.1, measured), whereas one ramp has no cross-planner pairs to separate at
# all. Viridis is perceptually uniform and CVD-safe by construction.
#
# The color column above is still the paper-wide identity color and is what
# Fig. 4 (and Figs. 5-6) paint with; Fig. 3 no longer uses it.
LEN_CMAP = plt.get_cmap("viridis")

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
    for m, _, _, _, _ in METHODS:
        with open(os.path.join(SWEEP, m, "trials.csv")) as f:
            out[m] = [float(r["wall_s"]) for r in csv.DictReader(f)
                      if r["success"] == "true"]
    return out


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------------- Fig. 3 ----
def length_bar(ax):
    """The one shared length-ratio scale, ticked linearly in the value."""
    grad = LEN_NORM(np.linspace(LEN_MIN, LEN_MAX, 256))
    ax.imshow(LEN_CMAP(grad)[None], aspect="auto", interpolation="bilinear",
              extent=(LEN_MIN, LEN_MAX, 0, 1))
    ax.set_yticks([])
    ax.set_xlim(LEN_MIN, LEN_MAX)
    ax.set_xticks(LEN_TICKS)
    ax.set_xticklabels([f"{t:.1f}" for t in LEN_TICKS], fontsize=6.5)
    ax.tick_params(axis="x", length=2.0, width=0.6, pad=1.5, colors="0.25")
    ax.set_xlabel(r"median length / $L_\mathrm{ref}$", fontsize=7, labelpad=1.5)
    for s in ax.spines.values():
        s.set_linewidth(0.5)
        s.set_color("0.55")


def fig3(summary):
    fig = plt.figure(figsize=(3.5, 2.55))
    ax = fig.add_axes((0.108, 0.305, 0.878, 0.665))
    cax = fig.add_axes((0.305, 0.115, 0.600, 0.052))
    pcts = sorted(next(iter(summary.values())).keys(), reverse=True)  # 100..30

    handles = []
    for m, label, _, marker, dash in METHODS:
        rows = summary[m]
        ratio = [rows[p][0] for p in pcts]
        rate = [rows[p][1] for p in pcts]
        primary = m == "hexspline_cl"
        lw, ink = (1.5, "0.30") if primary else (1.0, "0.55")
        ax.plot(pcts, rate, color=ink, lw=lw, ls=dash, zorder=2,
                solid_capstyle="round")
        ax.scatter(pcts, rate, c=ratio, cmap=LEN_CMAP, norm=LEN_NORM,
                   marker=marker, s=17 if primary else 15, linewidths=0.4,
                   edgecolors="0.25", zorder=4)
        handles.append(plt.Line2D([], [], color=ink, lw=lw, ls=dash,
                                  marker=marker, ms=3.2, mfc="0.85",
                                  mec="0.25", mew=0.4, label=label))

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
    ax.legend(handles=handles, loc="upper right", frameon=False, fontsize=6.5,
              handlelength=2.2, borderaxespad=0.1, labelspacing=0.28)

    length_bar(cax)
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.pdf"))
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.png"), dpi=300)
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 4 ----
def fig4(wall):
    fig, ax = plt.subplots(figsize=(3.5, 1.7))
    data = [wall[m] for m, _, _, _, _ in METHODS]
    pos = range(1, len(METHODS) + 1)

    bp = ax.boxplot(data, positions=list(pos), widths=0.55, patch_artist=True,
                    medianprops=dict(color="0.15", lw=1.0),
                    whiskerprops=dict(lw=0.7, color="0.35"),
                    capprops=dict(lw=0.7, color="0.35"),
                    flierprops=dict(marker=".", ms=2.0, mfc="0.55", mec="none",
                                    alpha=0.6))
    for patch, (_, _, color, _, _) in zip(bp["boxes"], METHODS):
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
    ax.set_xticklabels([f"{label}\n(n={len(wall[m])})" for m, label, _, _, _ in METHODS],
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
