#!/usr/bin/env python3
"""Fig. 2 of the paper -- the constraint ladder -- drawn in matplotlib.

    python plot_fig2_ladder_mpl.py <sweep_root> [--pcts 100,70,50,30]
                                   [--methods hexspline_cl] [--paths CSV]
                                   [--out DIR] [--stem fig2_ladder_grid]
                                   [--width 7.16]

Columns are constraint levels (% of the unconstrained reference uncertainty),
rows are planners; each cell draws the plan that rung shipped over the
scenario's obstacles and landmarks. Same content as `plot_ladder_grid.jl`,
which drew it with Plots.jl/GR and converted PDF->EPS through Ghostscript.
This draws it with the rcParams and `save()` of `make_figs_baseline.py`
(Figs. 3-6), `fig1_overview_mpl.py` (Fig. 1) and `fig8_montecarlo.py` (Fig. 8),
so Fig. 2 stops being the one figure in a different typographic family and the
EPS comes straight out of matplotlib's PostScript backend.

Sizing is the other reason: GR rendered a 16 in canvas that LaTeX then squeezed
into \\textwidth, so an 8 pt label printed at 3.6 pt and had to be scaled back
up by hand (FIG2_FONT_SCALE). Here the figure is built at its final 7.16 in, so
8 pt is 8 pt.

Inputs, all of them files the sweep already wrote:
  <root>/<method>/trials.csv                success, threshold, length, sigma
  <root>/<method>/s001_p<pct>/scenario_{obstacles,landmarks}.csv   the scenery
  <root>/<method>/s001_p<pct>/_cfg/main.yaml                corridor_y_max_m
  --paths CSV                               the spline paths, sampled by
                                            `export_ladder_paths.jl` with the
                                            planner's own bspline_sample_path
                                            (the runs save control points only,
                                            and the sampler is not reimplemented
                                            here -- one copy of that math)

Alphas are pre-blended onto white (`over_white`) rather than set as alpha:
PostScript has no transparency, and an alpha'd patch makes Ghostscript flatten
the page into a bitmap -- the failure Fig. 1 had. So all four formats agree.

The shipped figure (2026-09-11):
  python plot_fig2_ladder_mpl.py ../../../../fig2_ladder/rect_r3 \\
      --methods=hexspline_cl --stem=fig2_ladder_ours_rect_r3 --out=..
"""
import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))

LABEL = {"hexspline_cl": "ours", "greedy": "greedy", "formation": "formation",
         "sequential": "sequential", "clgbt": "CL-GBT"}
PREFERRED = ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]

# The Julia figure's colours, by name, so the two renders are the same picture:
# Plots.jl :blue / :purple for primary / support, :green start, :orange goal,
# gray35 obstacles at 0.55 and red landmark ellipses at 0.18, both over white.
PRIMARY_C = "#0000ff"
SUPPORT_C = "#800080"
START_C = "#008000"
GOAL_C = "#ffa500"
OBSTACLE_C, OBSTACLE_A = "#595959", 0.55
ELLIPSE_C, ELLIPSE_A = "#ff0000", 0.18
FAIL_GROUND = "#e5e5e5"

ELLIPSE_SCALE = 400.0   # viz.jl's display_scale: ellipses are cosmetic here
ELLIPSE_NSTD = 2

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Nimbus Roman", "Times New Roman", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.linewidth": 0.6,
    "xtick.labelsize": 6.5,
    "ytick.labelsize": 6.5,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.0,
    "ytick.major.size": 2.0,
    "pdf.fonttype": 42,
    # Type 3 for PS, as in make_figs_baseline.py: matplotlib's Type 42
    # embedding of Nimbus Roman writes a font Ghostscript rejects outright,
    # so the EPS would not open. Type 3 renders; text arrives in Illustrator
    # as outlines. Only the PDF goes into the paper, and it keeps Type 42.
    "ps.fonttype": 3,
    "svg.fonttype": "none",
})


def over_white(color, alpha):
    """Flatten `color` at `alpha` onto white -- see the module docstring."""
    r, g, b = matplotlib.colors.to_rgb(color)
    return tuple(1 - alpha * (1 - c) for c in (r, g, b))


def save(fig, out_dir, stem):
    """PDF for LaTeX, SVG and EPS for Illustrator, PNG to eyeball."""
    for ext, kw in (("pdf", {}), ("svg", {}), ("eps", {}), ("png", {"dpi": 300})):
        fig.savefig(os.path.join(out_dir, f"{stem}.{ext}"), **kw)
    print(f"  -> {os.path.join(out_dir, stem)}.{{pdf,svg,eps,png}}")


def rung_dir(root, method, pct):
    return os.path.join(root, method, f"s001_p{pct:03d}")


def read_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def num(row, key):
    v = (row.get(key) or "").strip()
    if v in ("", "null"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def read_cells(root, methods, pcts):
    """(method, pct) -> the rung's outcome, from each method's trials.csv."""
    cells = {}
    for m in methods:
        for r in read_rows(os.path.join(root, m, "trials.csv")):
            pct = int(r["pct"])
            if pct not in pcts:
                continue
            cells[(m, pct)] = {
                "ok": r.get("success") == "true",
                "why": (r.get("fail_reason") or "").strip(),
                "thr": num(r, "threshold"),
                "len": num(r, "primary_length"),
                "unc": num(r, "primary_unc"),
            }
    if not cells:
        raise SystemExit(f"trials.csv rows cover none of the levels {pcts}")
    return cells


def read_scenery(root, methods, pcts):
    """Obstacle rings and landmarks, from whichever rung wrote them."""
    for m in methods:
        for pct in pcts:
            d = rung_dir(root, m, pct)
            fo = os.path.join(d, "scenario_obstacles.csv")
            fl = os.path.join(d, "scenario_landmarks.csv")
            if not (os.path.isfile(fo) and os.path.isfile(fl)):
                continue
            rings = {}
            for r in read_rows(fo):
                rings.setdefault(int(r["obstacle_id"]), []).append(
                    (int(r["vertex_index"]), float(r["x"]), float(r["y"])))
            obstacles = [[(x, y) for _, x, y in sorted(v)] for _, v in sorted(rings.items())]
            landmarks = [(float(r["x"]), float(r["y"]),
                          np.array([[float(r["c11"]), float(r["c12"])],
                                    [float(r["c21"]), float(r["c22"])]]))
                         for r in read_rows(fl)]
            return obstacles, landmarks, d
    raise SystemExit("no rung carries scenario_obstacles.csv / scenario_landmarks.csv")


def read_cfg_float(cfg_dir, key, default):
    """One value out of a _cfg snapshot -- the configs are line-parsed here too."""
    path = os.path.join(cfg_dir, "main.yaml")
    if os.path.isfile(path):
        for line in open(path):
            line = line.split("#")[0].strip()
            if line.startswith(key + ":"):
                try:
                    return float(line.split(":", 1)[1])
                except ValueError:
                    break
    return default


def read_paths(csv_path):
    """(method, pct) -> [agent 1 points, agent 2 points, ...], sampled by Julia."""
    out = {}
    for r in read_rows(csv_path):
        key = (r["method"], int(r["pct"]))
        out.setdefault(key, {}).setdefault(int(r["agent"]), []).append(
            (int(r["i"]), float(r["x"]), float(r["y"])))
    return {k: [np.array([(x, y) for _, x, y in sorted(v)])
                for _, v in sorted(agents.items())]
            for k, agents in out.items()}


def ellipse_ring(x, y, cov, npts=50):
    """viz.jl's draw_covariance_ellipse!, at its display_scale."""
    cov_vis = ELLIPSE_SCALE * cov
    vals, vecs = np.linalg.eigh((cov_vis + cov_vis.T) / 2)
    a = ELLIPSE_NSTD * np.sqrt(max(vals[0], 0.0))
    b = ELLIPSE_NSTD * np.sqrt(max(vals[1], 0.0))
    angle = np.arctan2(vecs[1, 0], vecs[0, 0])
    th = np.linspace(0, 2 * np.pi, npts)
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    pts = rot @ np.vstack((a * np.cos(th), b * np.sin(th)))
    return x + pts[0], y + pts[1]


def draw_scenery(ax, obstacles, landmarks, start, goal, xlim, ylim, ground):
    ax.set_facecolor(ground)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(0, goal[0] + 1, 500))
    ax.set_yticks([-200, 0, 200])
    for s in ax.spines.values():
        s.set_linewidth(0.6)
    # ellipses first, then obstacles, so an obstacle still covers an ellipse
    for lx, ly, cov in landmarks:
        ex, ey = ellipse_ring(lx, ly, cov)
        ax.fill(ex, ey, facecolor=over_white(ELLIPSE_C, ELLIPSE_A), edgecolor="none",
                zorder=1.5)
    for ring in obstacles:
        ax.add_patch(Polygon(ring, closed=True, facecolor=over_white(OBSTACLE_C, OBSTACLE_A),
                             edgecolor="black", linewidth=0.6, zorder=2))
    ax.plot([lx for lx, _, _ in landmarks], [ly for _, ly, _ in landmarks], "o",
            color="black", markersize=1.4, markeredgewidth=0, zorder=3)
    ax.plot([start[0]], [start[1]], "o", color=START_C, markersize=2.6,
            markeredgewidth=0, zorder=4)
    ax.plot([goal[0]], [goal[1]], "*", color=GOAL_C, markersize=5.0,
            markeredgewidth=0, zorder=4)


def draw_paths(ax, agents, support_offset, faded):
    n = len(agents)
    for a, pts in enumerate(agents, start=1):
        if len(pts) < 2:
            continue
        primary = a == n
        color = PRIMARY_C if primary else SUPPORT_C
        alpha = 1.0 if not faded else 0.4
        ax.plot(pts[:, 0], pts[:, 1] + (0.0 if primary else support_offset * a),
                color=over_white(color, alpha),
                linewidth=0.9 if primary else 0.6,
                linestyle="-" if primary else (0, (4, 2)), zorder=5,
                solid_capstyle="round")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--pcts", default="100,70,50,30")
    ap.add_argument("--methods", default="")
    ap.add_argument("--paths", default="")
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--stem", default="fig2_ladder_grid")
    ap.add_argument("--width", type=float, default=7.16)  # \textwidth, as Fig. 4's wide form
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    pcts = [int(p) for p in args.pcts.split(",")]
    have = [d for d in sorted(os.listdir(root))
            if os.path.isfile(os.path.join(root, d, "trials.csv"))]
    if args.methods:
        methods = args.methods.split(",")
        missing = [m for m in methods if m not in have]
        if missing:
            raise SystemExit(f"no trials.csv for {missing} under {root}")
    else:
        methods = [m for m in PREFERRED if m in have] + \
                  sorted(m for m in have if m not in PREFERRED)
    paths_csv = args.paths or os.path.join(HERE, "fig2_paths.csv")
    if not os.path.isfile(paths_csv):
        raise SystemExit(f"no sampled paths at {paths_csv} -- run export_ladder_paths.jl first")

    cells = read_cells(root, methods, pcts)
    obstacles, landmarks, geom_dir = read_scenery(root, methods, pcts)
    paths = read_paths(paths_csv)
    cfg_dir = os.path.join(geom_dir, "_cfg")
    y_max = read_cfg_float(cfg_dir, "corridor_y_max_m", 300.0)
    support_offset = 3.0  # hexspline_cl.yaml's support_plot_offset_m, cosmetic

    # The primary is the last agent: its spline runs start -> goal, so the two
    # markers come from the plan rather than from a second copy of the scenario.
    any_path = paths[(methods[0], pcts[0])]
    start = tuple(any_path[-1][0])
    goal = tuple(any_path[-1][-1])

    xlim = (-60.0, goal[0] + 60.0)
    ylim = (-y_max - 45.0, y_max + 45.0)

    nr, nc = len(methods), len(pcts)
    # Explicit geometry: equal-aspect panels sized from the data, so the strip
    # has no slack for matplotlib to distribute unevenly.
    left = 0.34 if nr == 1 else 0.60
    right, gap_x, bottom, top_pad = 0.04, 0.30, 0.24, 0.03
    title_h = 0.30                     # two lines of title over each column
    row_gap = 0.10
    panel_w = (args.width - left - right - gap_x * (nc - 1)) / nc
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    fig_h = bottom + nr * panel_h + (nr - 1) * (row_gap + bottom) + title_h * nr + top_pad
    fig = plt.figure(figsize=(args.width, fig_h))

    for i, m in enumerate(methods):
        for j, pct in enumerate(pcts):
            c = cells.get((m, pct))
            x0 = (left + j * (panel_w + gap_x)) / args.width
            y0 = (fig_h - top_pad - (i + 1) * title_h - (i + 1) * panel_h
                  - i * (row_gap + bottom)) / fig_h
            ax = fig.add_axes([x0, y0, panel_w / args.width, panel_h / fig_h])
            ok = bool(c and c["ok"])
            draw_scenery(ax, obstacles, landmarks, start, goal, xlim, ylim,
                         "white" if ok else FAIL_GROUND)
            if c is None:
                ax.text(np.mean(xlim), 0.0, "not run", ha="center", va="center",
                        fontsize=7, color="0.4")
            elif (m, pct) in paths:
                draw_paths(ax, paths[(m, pct)], support_offset, faded=not ok)

            if c is None:
                status = "not run"
            elif ok and c["len"] is not None and c["unc"] is not None:
                status = f"{c['len']:.0f} m,  $\\sigma = {c['unc']:.2f}$ m"
            elif ok:
                status = "ok"
            else:
                status = "✗ " + c["why"].replace("_", " ")
            head = ""
            if i == 0:
                thr = c["thr"] if c and c["thr"] is not None else float("nan")
                head = f"{pct} %  ($\\sigma \\leq {thr:.2f}$ m)\n"
            ax.set_title(head + status, fontsize=7.5, pad=2.5,
                         color="firebrick" if (c is not None and not ok) else "black",
                         linespacing=1.35)
            if j == 0 and nr > 1:
                ax.set_ylabel(LABEL.get(m, m), fontsize=8)

    os.makedirs(args.out, exist_ok=True)
    save(fig, os.path.abspath(args.out), args.stem)

    print(f"\n-- {os.path.basename(root)}: {nr} planners x {nc} levels --")
    for m in methods:
        line = f"{LABEL.get(m, m):<13}"
        for pct in pcts:
            c = cells.get((m, pct))
            s = "-" if c is None else (f"{c['len']:.0f}/{c['unc']:.2f}" if c["ok"]
                                       else "x " + c["why"])
            line += f" {s:>14}"
        print(line)


if __name__ == "__main__":
    main()
