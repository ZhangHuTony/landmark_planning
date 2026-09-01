#!/usr/bin/env python3
"""Fig. 3 and Fig. 4 of the paper, from constraint_sweep/baseline_2026-08-17.

Fig. 3  fig3_length_vs_constraint.{pdf,png}
        median path-length ratio vs. constraint level, one line per planner,
        opacity of each segment/marker = success rate at that level.
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
from matplotlib.ticker import NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.normpath(os.path.join(HERE, "../../../constraint_sweep/baseline_2026-08-17"))

# Fixed identity -> color assignment (dataviz reference palette, slots 1-5).
# Keep this order/color per method in EVERY figure of the paper.
METHODS = [
    ("hexspline_cl", "Ours",       "#2a78d6"),
    ("formation",    "Formation",  "#eb6834"),
    ("sequential",   "Sequential", "#1baf7a"),
    ("clgbt",        "CL-GBT",     "#eda100"),
    ("greedy",       "Greedy",     "#e87ba4"),
]

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
    for m, _, _ in METHODS:
        with open(os.path.join(SWEEP, m, "trials.csv")) as f:
            out[m] = [float(r["wall_s"]) for r in csv.DictReader(f)
                      if r["success"] == "true"]
    return out


def alpha(rate):
    """Success rate -> opacity; floor keeps a 4%-success line printable."""
    return 0.15 + 0.85 * rate


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------------- Fig. 3 ----
def fig3(summary):
    fig, ax = plt.subplots(figsize=(3.5, 2.05))
    pcts = sorted(next(iter(summary.values())).keys(), reverse=True)  # 100..30

    ax.axhline(1.0, color="0.55", lw=0.7, ls=(0, (4, 3)), zorder=1)

    handles = []
    for m, label, color in METHODS:
        rows = summary[m]
        lw = 1.6 if m == "hexspline_cl" else 1.1
        for a, b in zip(pcts[:-1], pcts[1:]):
            seg_alpha = alpha(0.5 * (rows[a][1] + rows[b][1]))
            ax.plot([a, b], [rows[a][0], rows[b][0]], color=color, lw=lw,
                    alpha=seg_alpha, solid_capstyle="round", zorder=3)
        for p in pcts:
            ax.plot(p, rows[p][0], marker="o", ms=2.6, mec="none",
                    color=color, alpha=alpha(rows[p][1]), zorder=4)
        handles.append(plt.Line2D([], [], color=color, lw=lw, marker="o",
                                  ms=2.6, mec="none", label=label))

    ax.invert_xaxis()  # constraint tightens to the right
    ax.set_xticks(pcts)
    ax.set_xlabel(r"constraint level (% of $U_\mathrm{ref}$)")
    ax.set_ylabel(r"median length / $L_\mathrm{ref}$")
    ax.set_ylim(0.93, 2.55)
    ax.grid(axis="y", color="0.88", lw=0.5, zorder=0)
    ax.set_axisbelow(True)
    despine(ax)
    ax.legend(handles=handles, loc="upper left", frameon=False,
              handlelength=1.5, borderaxespad=0.2, labelspacing=0.35)

    fig.tight_layout(pad=0.25)
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.pdf"))
    fig.savefig(os.path.join(HERE, "fig3_length_vs_constraint.png"), dpi=300)
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 4 ----
def fig4(wall):
    fig, ax = plt.subplots(figsize=(3.5, 1.7))
    data = [wall[m] for m, _, _ in METHODS]
    pos = range(1, len(METHODS) + 1)

    bp = ax.boxplot(data, positions=list(pos), widths=0.55, patch_artist=True,
                    medianprops=dict(color="0.15", lw=1.0),
                    whiskerprops=dict(lw=0.7, color="0.35"),
                    capprops=dict(lw=0.7, color="0.35"),
                    flierprops=dict(marker=".", ms=2.0, mfc="0.55", mec="none",
                                    alpha=0.6))
    for patch, (_, _, color) in zip(bp["boxes"], METHODS):
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
    ax.set_xticklabels([f"{label}\n(n={len(wall[m])})" for m, label, _ in METHODS],
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
