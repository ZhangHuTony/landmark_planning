#!/usr/bin/env python3
"""Fig. 1 of the paper, drawn from fig1_overview.jl's scene dump.

    python fig1_overview_mpl.py <fig1_scene*.json> [<out_dir>] [--width IN]

Writes <stem>.{pdf,svg,eps,png} and <stem>_comm.{pdf,svg,eps,png} into
<out_dir> (default: the scene file's own folder), where <stem> comes from the
scene.

Why this exists. fig1_overview.jl draws with GR, and GR has no EPS writer, so
its EPS had to come from Ghostscript's eps2write. PostScript has no
transparency, and this figure is full of alpha'd fills (hex tiles, obstacles,
covariance ellipses), so Ghostscript flattened the entire page into one
10000x5600 bitmap -- opening it in Illustrator gave a single unselectable
object. Figs. 3-4 (make_figs_baseline.py) do not have that problem because
matplotlib's PS backend writes real vector paths and drops alpha; that file
therefore pre-blends every alpha onto its background (`over_white`) so all four
formats agree. This script does the same for Fig. 1: one path per hex tile, per
obstacle, per ellipse, per track, and live text in the SVG.

Style follows make_figs_baseline.py exactly (serif, 8/7/6.5 pt, 0.6 pt axes,
column width), so Fig. 1 sits in the same typographic family as Figs. 3-4.
"""
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

# Same rcParams as make_figs_baseline.py -- see its comments for why ps.fonttype
# is 3 (matplotlib's Type 42 Nimbus Roman is rejected by Ghostscript) and why
# svg.fonttype is "none" (keeps labels as live text in Illustrator).
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

COLUMN_IN = 3.5      # IEEE conference column; Figs. 3-4 are 252 pt = 3.5 in

# viz.jl's palette, as Plots.jl resolves it. Everything here is a CSS4 name
# matplotlib knows too, except Julia's X11-numbered grays.
JULIA_COLORS = {"gray35": "#595959"}

# viz.jl's alphas, kept here rather than in the scene: they are how the figure
# is drawn, not what the run produced.
HEX_FILL, HEX_FILL_ALPHA, HEX_LINE = "aliceblue", 0.98, "cadetblue"
OBSTACLE_FILL, OBSTACLE_ALPHA = "gray35", 0.55
LANDMARK_ELLIPSE, LANDMARK_ELLIPSE_ALPHA = "red", 0.25
COV_ELLIPSE_ALPHA = 0.22

# Line widths and marker sizes, in points at column width. GR's were set for a
# 1000x560 px figure; at 3.5 in everything is roughly halved.
LW_HEX, LW_OBSTACLE, LW_PRIMARY, LW_SUPPORT, LW_ELLIPSE, LW_COMM = 0.35, 0.6, 1.1, 0.7, 0.4, 0.6
MS_LANDMARK, MS_START, MS_GOAL, MS_COMM = 2.6, 4.2, 6.5, 2.6


def color(name):
    return JULIA_COLORS.get(name, name)


def over(c, alpha, base=(1.0, 1.0, 1.0)):
    """Flatten `c` at `alpha` onto `base`.

    Same trick as make_figs_baseline.py's over_white, with an explicit base:
    the PostScript backend has no transparency, so an alpha'd patch would come
    out fully opaque in the EPS and not match the PDF. Pre-blending keeps all
    four formats identical. `base` is the hex field rather than white for
    anything drawn on top of the tiles, so the blend matches what GR shows.
    """
    rgb = matplotlib.colors.to_rgb(color(c))
    return tuple(b + alpha * (v - b) for v, b in zip(rgb, base))


HEX_BG = over(HEX_FILL, HEX_FILL_ALPHA)   # what the tiles leave under everything


def ellipse_points(x, y, cov, nstd, display_scale, npts=50):
    """viz.jl's draw_covariance_ellipse!, as points.

    display_scale multiplies the COVARIANCE (viz.jl scales sigma, so it passes
    the square). Both eigen decompositions are LAPACK's and return ascending
    eigenvalues; an eigenvector sign flip rotates the parametrization by pi,
    which maps the ellipse onto itself.
    """
    cov_vis = display_scale * np.asarray(cov, dtype=float)
    vals, vecs = np.linalg.eigh((cov_vis + cov_vis.T) / 2)
    a = nstd * math.sqrt(max(vals[0], 0.0))
    b = nstd * math.sqrt(max(vals[1], 0.0))
    angle = math.atan2(vecs[1, 0], vecs[0, 0])
    th = np.linspace(0, 2 * math.pi, npts)
    R = np.array([[math.cos(angle), -math.sin(angle)],
                  [math.sin(angle), math.cos(angle)]])
    pts = R @ np.vstack((a * np.cos(th), b * np.sin(th)))
    return pts[0] + x, pts[1] + y


def world_limits(scene):
    """graph.jl's set_hex_world_limits!: route tiles + sensor landmarks, padded."""
    xs = [c[0] for c in scene["hex_centers"]] + [l["x"] for l in scene["sensor_landmarks"]]
    ys = [c[1] for c in scene["hex_centers"]] + [l["y"] for l in scene["sensor_landmarks"]]
    m = max(scene["hex_radius"], 30.0)
    return (min(xs) - m, max(xs) + m), (min(ys) - m, max(ys) + m)


def draw(scene, with_comm, width_in=COLUMN_IN):
    (x0, x1), (y0, y1) = world_limits(scene)

    # Margins in inches; the axes box height follows from equal aspect, and the
    # figure height from that plus the legend strip underneath.
    left, right, top = 0.52, 0.04, 0.05
    xlabel_h, legend_rows, legend_row_h = 0.30, 3, 0.135
    legend_h = legend_rows * legend_row_h + 0.05
    ax_w = width_in - left - right
    ax_h = ax_w * (y1 - y0) / (x1 - x0)
    fig_h = top + ax_h + xlabel_h + legend_h

    fig = plt.figure(figsize=(width_in, fig_h))
    ax = fig.add_axes([left / width_in, (xlabel_h + legend_h) / fig_h,
                       ax_w / width_in, ax_h / fig_h])
    ax.set_aspect("equal")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_xlabel(scene["xlabel"])
    ax.set_ylabel(scene["ylabel"])
    for s in ax.spines.values():
        s.set_zorder(5)

    # --- hex lattice (one patch per tile, so each is its own object) --------
    r, th = scene["hex_radius"], [math.pi / 6 + k * math.pi / 3 for k in range(6)]
    unit = [(math.cos(t), math.sin(t)) for t in th]   # pointy-top, as draw_hex_tiles!
    for cx, cy in scene["hex_centers"]:
        ax.add_patch(Polygon([(cx + r * ux, cy + r * uy) for ux, uy in unit],
                             closed=True, facecolor=HEX_BG, edgecolor=color(HEX_LINE),
                             linewidth=LW_HEX, zorder=0))

    # --- obstacles ---------------------------------------------------------
    obstacle_handle = None
    for verts in scene["obstacles"]:
        p = Polygon(verts, closed=True, facecolor=over(OBSTACLE_FILL, OBSTACLE_ALPHA, HEX_BG),
                    edgecolor="black", linewidth=LW_OBSTACLE, zorder=1)
        ax.add_patch(p)
        obstacle_handle = obstacle_handle or p

    # --- landmarks: the field's own covariance, then the marker ------------
    for lm in scene["sensor_landmarks"]:
        ex, ey = ellipse_points(lm["x"], lm["y"], lm["cov"], scene["ellipse_nstd"],
                                scene["landmark_display_scale"])
        ax.fill(ex, ey, facecolor=over(LANDMARK_ELLIPSE, LANDMARK_ELLIPSE_ALPHA, HEX_BG),
                edgecolor=over(LANDMARK_ELLIPSE, LANDMARK_ELLIPSE_ALPHA, HEX_BG),
                linewidth=LW_ELLIPSE, zorder=1.5)

    # --- covariance ellipses, under the tracks so the tracks stay visible ---
    # (GR drew them over the tracks at alpha 0.22; pre-blended they are opaque,
    # so they change places with the tracks instead of hiding them.)
    for e in scene["ellipses"]:
        ex, ey = ellipse_points(e["x"], e["y"], e["cov"], scene["ellipse_nstd"],
                                scene["ellipse_display_scale"])
        ax.fill(ex, ey, facecolor=over(e["color"], COV_ELLIPSE_ALPHA, HEX_BG),
                edgecolor=over(e["color"], COV_ELLIPSE_ALPHA, HEX_BG),
                linewidth=LW_ELLIPSE, zorder=2)

    # --- tracks ------------------------------------------------------------
    track_handles = []
    for a in scene["agents"]:
        line, = ax.plot(a["xs"], a["ys"], color=color(a["color"]),
                        linewidth=LW_PRIMARY if a["primary"] else LW_SUPPORT,
                        solid_joinstyle="round", label=a["label"], zorder=3)
        track_handles.append(line)

    lm_handle, = ax.plot([l["x"] for l in scene["sensor_landmarks"]],
                         [l["y"] for l in scene["sensor_landmarks"]],
                         linestyle="none", marker="o", markersize=MS_LANDMARK,
                         color="black", markeredgewidth=0, label="Landmarks", zorder=4)
    start_handle, = ax.plot(*scene["start"], linestyle="none", marker="o",
                            markersize=MS_START, color=color("green"),
                            markeredgewidth=0, label="Start", zorder=4)
    goal_handle, = ax.plot(*scene["goal"], linestyle="none", marker="*",
                           markersize=MS_GOAL, color=color("orange"),
                           markeredgewidth=0, label="Goal", zorder=4)

    # --- comm overlay: one segment + two diamonds per fusion event ---------
    if with_comm and scene["comm"]:
        cmap = plt.get_cmap("plasma")
        max_t = max(c["t"] for c in scene["comm"])
        for c in scene["comm"]:
            clr = cmap(c["t"] / max_t if max_t > 0 else 0.0)[:3]
            (ax_, ay_), (bx_, by_) = c["pa"], c["pb"]
            # GR keyed the segment's alpha to the fusion weight; pre-blended,
            # a weak exchange fades toward the background exactly as it did.
            ax.plot([ax_, bx_], [ay_, by_], color=over(clr, c["w"], HEX_BG),
                    linewidth=LW_COMM, zorder=4.5)
            ax.plot([ax_, bx_], [ay_, by_], linestyle="none", marker="D",
                    markersize=MS_COMM, color=clr, markeredgewidth=0, zorder=4.5)

    # --- legend ------------------------------------------------------------
    # The ellipses are fills with no legend key of their own, so they get a
    # proxy marker, as in fig1_overview.jl.
    ellipse_proxy = Line2D([], [], linestyle="none", marker="o", markersize=4,
                           markerfacecolor=over("blue", 0.3, HEX_BG),
                           markeredgecolor=over("blue", 0.3, HEX_BG),
                           label=scene["ellipse_label"])
    handles = [lm_handle, start_handle, goal_handle]
    if obstacle_handle is not None:
        obstacle_handle.set_label("Obstacle")
        handles.append(obstacle_handle)
    handles += track_handles + [ellipse_proxy]
    fig.legend(handles=handles, labels=[h.get_label() for h in handles],
               loc="lower center", bbox_to_anchor=(0.5, 0.0),
               ncol=3, frameon=False, handlelength=1.6, columnspacing=1.2,
               handletextpad=0.5, borderaxespad=0.2)
    return fig


def save(fig, out_dir, stem):
    """PDF for LaTeX, SVG and EPS for Illustrator, PNG to eyeball."""
    for ext, kw in (("pdf", {}), ("svg", {}), ("eps", {}), ("png", {"dpi": 300})):
        fig.savefig(os.path.join(out_dir, f"{stem}.{ext}"), **kw)
    print(f"  -> {os.path.join(out_dir, stem)}.{{pdf,svg,eps,png}}")


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    width = COLUMN_IN
    for a in argv[1:]:
        if a.startswith("--width="):
            width = float(a.split("=", 1)[1])
    if not args:
        sys.exit(__doc__)
    scene_path = args[0]
    out_dir = args[1] if len(args) > 1 else os.path.dirname(os.path.abspath(scene_path))
    with open(scene_path) as f:
        scene = json.load(f)
    os.makedirs(out_dir, exist_ok=True)
    for with_comm, suffix in ((False, ""), (True, "_comm")):
        if with_comm and not scene["comm"]:
            continue
        fig = draw(scene, with_comm, width)
        save(fig, out_dir, scene["stem"] + suffix)
        plt.close(fig)


if __name__ == "__main__":
    main(sys.argv)
