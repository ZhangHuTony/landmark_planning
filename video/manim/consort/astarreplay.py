"""Shared "wavefront" replay of a priority-queue search trace.

Design chosen with the user after watching the reference GIF
(fig_astar_partial.gif): that GIF draws the CURRENT candidate's whole path
every frame in raw pop order, which jumps between very different branches
frame to frame because best-first search does not explore space in any
spatial order. Here the SAME real events are replayed sorted by the primary's
cumulative distance g (monotonically non-decreasing by construction), so the
frontier sweeps outward like a classic Dijkstra animation instead of jumping.
Nothing is invented: every path shown is a real state that was really popped;
only the PLAYBACK ORDER changes, and it is stated on screen.

One reader, four callers: the joint A* (2 agents, fig1/astar_trace.csv), the
formation and sequential-primary panels (1 agent each, their own
astar_trace*.csv -- same schema, since those searches are structurally the
same weighted A* as the engine's single-agent branch).
"""
import csv
from collections import defaultdict

import numpy as np
from manim import (
    VGroup, Cross, Square, Text, ValueTracker, always_redraw, UP, DOWN,
)

from .mobjects import polyline
from .palette import BAD, FONT


def load_trace(path):
    """Returns (pops, pushes, prunes), each a list of
    (iter, si, parent, g, nodes:list[int], reason)."""
    pops, pushes, prunes = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            nodes = [int(n) for n in r["nodes"].split(";")]
            g = float(r["g"]) if r["g"] not in ("", None) else 0.0
            rec = (int(r["iter"]), int(r["si"]), int(r["parent"]), g, nodes, r["reason"])
            {"pop": pops, "push": pushes, "prune": prunes}.get(r["ev"], []).append(rec)
    return pops, pushes, prunes


def build_paths_by_state(pops, pushes):
    """si -> full node-path (per agent) by walking parent pointers.

    Both pops and pushes carry (si, parent, nodes); a state's own nodes are
    only recorded once (whichever event logged it first), so both lists feed
    the same table.
    """
    node_of = {}
    parent_of = {}
    for _, si, parent, _, nodes, _ in pops:
        node_of.setdefault(si, nodes)
        parent_of.setdefault(si, parent)
    for _, si, parent, _, nodes, _ in pushes:
        node_of.setdefault(si, nodes)
        parent_of.setdefault(si, parent)

    cache = {}

    def path_of(si):
        if si in cache:
            return cache[si]
        chain = []
        k = si
        seen = set()
        while k != -1 and k != 0 and k in node_of and k not in seen:
            seen.add(k)
            chain.append(node_of[k])
            k = parent_of.get(k, -1)
        chain.reverse()
        cache[si] = chain
        return chain

    return path_of


class WavefrontReplay:
    """Everything needed to animate one search's frontier growing by g.

    `world`, `node_xy` (id -> (x,y)), and `agent_colors` (list, primary last)
    are the only scene-specific inputs; `max_g` is where the ValueTracker
    should end up.
    """

    def __init__(self, trace_path, node_xy, agent_colors, world, keep=14, fade_span=None):
        pops, pushes, prunes = load_trace(trace_path)
        self.n_agents = len(agent_colors)
        self.total_pops = len(pops)
        path_of = build_paths_by_state(pops, pushes)

        # One entry per pop, in G ORDER (not pop order) -- this is the whole
        # point: replaying by distance-from-start turns a jittery best-first
        # trace into a steadily growing wavefront.
        entries = []
        for it, si, parent, g, nodes, _ in pops:
            chain = path_of(si) + [nodes]
            entries.append((g, it, chain))
        entries.sort(key=lambda e: e[0])
        self.entries = entries
        self.max_g = entries[-1][0] if entries else 1.0
        self.fade_span = fade_span or max(self.max_g * 0.12, 1.0)
        self.keep = keep
        self.node_xy = node_xy
        self.world = world
        self.agent_colors = agent_colors
        self.prunes = prunes  # (iter, si, parent, g, nodes, reason), g-unordered

    def path_points(self, agent, chain):
        return np.array([self.node_xy[step[agent]] for step in chain
                         if agent < len(step) and step[agent] in self.node_xy])

    def mobject(self, g_tracker):
        """always_redraw group: the last `keep` frontier paths at or before
        the tracker's g, each agent's own colour, older ones fading out."""
        def build():
            g_now = g_tracker.get_value()
            i = np.searchsorted([e[0] for e in self.entries], g_now, side="right")
            window = self.entries[max(0, i - self.keep):i]
            grp = VGroup()
            for g, it, chain in window:
                age = np.clip((g_now - g) / self.fade_span, 0.0, 1.0)
                opacity = 0.85 * (1.0 - age) + 0.06
                for a in range(self.n_agents):
                    pts = self.path_points(a, chain)
                    if len(pts) < 2:
                        continue
                    grp.add(polyline(self.world, pts[:, 0], pts[:, 1],
                                     stroke_color=self.agent_colors[a],
                                     stroke_width=2.4, stroke_opacity=float(opacity)))
            return grp
        return always_redraw(build)

    def prune_flash_mobject(self, g_tracker, flash_span=None):
        """Brief red X's at the g of each rejected expansion, near the tracker."""
        flash_span = flash_span or self.fade_span * 0.6
        by_g = sorted(self.prunes, key=lambda p: p[3])
        gs = [p[3] for p in by_g]

        def build():
            g_now = g_tracker.get_value()
            i = np.searchsorted(gs, g_now, side="right")
            j = np.searchsorted(gs, g_now - flash_span, side="left")
            grp = VGroup()
            for p in by_g[j:i]:
                _, _, _, g, nodes, reason = p
                agent = 0
                if ":" in reason:
                    try:
                        agent = int(reason.split(":")[1]) - 1
                    except ValueError:
                        pass
                node = nodes[min(agent, len(nodes) - 1)]
                if node not in self.node_xy:
                    continue
                x, y = self.node_xy[node]
                age = np.clip((g_now - g) / flash_span, 0.0, 1.0)
                grp.add(Cross(Square(side_length=0.14), stroke_color=BAD,
                              stroke_width=3).set_opacity(1.0 - age)
                        .move_to(self.world.pt(x, y)))
            return grp
        return always_redraw(build)

    def counter_mobject(self, g_tracker, ink_color, muted_color, corner=None, buff=0.35):
        """`corner` (e.g. UR) is applied INSIDE the redraw, not after -- an
        always_redraw mobject rebuilds from scratch every frame, so a
        position set on the wrapper only sticks for one frame otherwise."""
        def build():
            g_now = g_tracker.get_value()
            i = np.searchsorted([e[0] for e in self.entries], g_now, side="right")
            grp = VGroup(
                Text(f"{i}", font=FONT, font_size=34, color=ink_color),
                Text(f"of {self.total_pops} expansions", font=FONT, font_size=15,
                     color=muted_color),
            ).arrange(DOWN, buff=0.04)
            if corner is not None:
                grp.to_corner(corner, buff=buff)
            return grp
        return always_redraw(build)


def reason_counts(prunes):
    c = defaultdict(int)
    for _, _, _, _, _, reason in prunes:
        c[reason.split(":")[0]] += 1
    return dict(c)
