"""Three more search visuals, each matching a genuinely different algorithm.

wavefront_replay (astarreplay.py) covers every priority-queue weighted A* in
this codebase (the joint search, formation's primary, sequential's primary
legs). The three baselines whose search is NOT a priority queue get their own
visual grammar here, each chosen to match what the algorithm actually does
rather than forcing everything into one look:

  greedy_feelers   -- per-step, no-lookahead: short local "which move next"
                      segments, not a growing path.
  layered_beam     -- sequential's helper search already IS a wavefront (the
                      beam is thinned to survivors every layer), so this is
                      the one search that needs no reordering trick at all.
  clgbt_tree       -- branches drawn in push order, already spatially coherent
                      since every new node extends an existing nearby one.
"""
import csv
from collections import defaultdict

import numpy as np
from manim import VGroup, Dot, Line, ValueTracker, always_redraw

from .mobjects import polyline


def load_greedy_trace(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                "step": int(r["step"]),
                "combo": [int(x) for x in r["combo"].split(";")],
                "primary_unc": float(r["primary_unc"]),
                "admissible": r["admissible"] == "true",
                "chosen": r["chosen"] == "true",
            })
    return rows


def greedy_feelers_mobject(rows, node_xy, support_pos_by_step, world, step_tracker,
                           color_ok, color_bad):
    """At the current (integer) step, short segments from the support's CURRENT
    position to each candidate combo node, solid for the chosen one, faded red
    for rejected ones. Matches the real algorithm: a one-hex, no-lookahead
    decision made fresh at every step, not a path being extended.
    """
    by_step = defaultdict(list)
    for r in rows:
        by_step[r["step"]].append(r)

    def build():
        k = int(round(step_tracker.get_value()))
        cur = support_pos_by_step.get(k)
        if cur is None:
            return VGroup()
        grp = VGroup()
        for r in by_step.get(k, []):
            node = r["combo"][0] if r["combo"] else None
            if node is None or node not in node_xy:
                continue
            x, y = node_xy[node]
            if r["chosen"]:
                grp.add(Line(world.pt(*cur), world.pt(x, y), stroke_color=color_ok,
                             stroke_width=3.6))
                grp.add(Dot(world.pt(x, y), radius=0.05, color=color_ok))
            else:
                op = 0.85 if r["admissible"] else 0.35
                grp.add(Line(world.pt(*cur), world.pt(x, y), stroke_color=color_bad,
                             stroke_width=1.6, stroke_opacity=op))
        return grp
    return always_redraw(build)


def load_layered_beam(path, helper_key):
    """rows: layer -> list of (node, arc, helper_unc, primary_unc, path:list[int])"""
    layers = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["helper"] != helper_key:
                continue
            layers[int(r["layer"])].append({
                "node": int(r["node"]), "arc": float(r["arc"]),
                "helper_unc": float(r["helper_unc"]), "primary_unc": float(r["primary_unc"]),
                "path": [int(x) for x in r["path"].split(";")],
            })
    return layers


def layered_beam_mobject(layers, node_xy, world, layer_tracker, color, max_dots=60):
    """always_redraw: every surviving label up to the current layer, as dots
    fading with distance-in-layers from the current one -- already exactly the
    "wavefront" shape (dominance + beam already did the thinning), so no
    reordering is needed here.
    """
    max_layer = max(layers) if layers else 1

    def build():
        k = int(np.clip(round(layer_tracker.get_value()), 1, max_layer))
        grp = VGroup()
        for dl in range(0, 3):
            lk = k - dl
            if lk not in layers:
                continue
            opacity = 1.0 - dl * 0.4
            pts = layers[lk][:max_dots]
            for lbl in pts:
                if lbl["node"] not in node_xy:
                    continue
                x, y = node_xy[lbl["node"]]
                grp.add(Dot(world.pt(x, y), radius=0.045, color=color, fill_opacity=opacity))
        return grp
    return always_redraw(build)


def winning_path_mobject(layers, node_xy, world, color):
    """The best-ranked survivor of the final layer -- what seq_helper_search
    actually returns (its own spline-clearance re-check may pick a different
    rank, but the top of the layer is what the search itself preferred)."""
    max_layer = max(layers)
    best = min(layers[max_layer], key=lambda L: (L["primary_unc"], L["helper_unc"]))
    pts = np.array([node_xy[n] for n in best["path"] if n in node_xy])
    if len(pts) < 2:
        return VGroup()
    return polyline(world, pts[:, 0], pts[:, 1], stroke_color=color, stroke_width=2.6,
                    stroke_opacity=0.9)


def load_clgbt_tree(path):
    """node -> {parent, agents: {a: (x,y)}, on_solution}"""
    nodes = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            n = int(r["node"])
            if n not in nodes:
                nodes[n] = {"parent": int(r["parent"]), "agents": {}, "on_solution": r["on_solution"] == "true"}
            nodes[n]["agents"][int(r["agent"])] = (float(r["x"]), float(r["y"]))
    return nodes


def clgbt_tree_mobject(tree, world, n_tracker, agent_colors, thin=1):
    """Branches in push order (node index == push order already, so no
    reordering trick is needed -- unlike best-first search, RRT-style growth
    only ever extends from an existing nearby node, so it is spatially
    coherent by construction). `thin` draws every Nth node to keep a ~15k-node
    tree animatable.
    """
    order = sorted(tree)
    kept = [i for i in order if i % thin == 0 or tree[i]["on_solution"]]

    def build():
        k = int(round(n_tracker.get_value()))
        grp = VGroup()
        for i in kept:
            if i > k:
                break
            nd = tree[i]
            if nd["parent"] == 0 or nd["parent"] not in tree:
                continue
            pnd = tree[nd["parent"]]
            for a, col in enumerate(agent_colors, start=1):
                if a not in nd["agents"] or a not in pnd["agents"]:
                    continue
                x0, y0 = pnd["agents"][a]
                x1, y1 = nd["agents"][a]
                op = 0.9 if nd["on_solution"] else 0.35
                w = 2.2 if nd["on_solution"] else 1.0
                grp.add(Line(world.pt(x0, y0), world.pt(x1, y1), stroke_color=col,
                             stroke_width=w, stroke_opacity=op))
        return grp
    return always_redraw(build)
