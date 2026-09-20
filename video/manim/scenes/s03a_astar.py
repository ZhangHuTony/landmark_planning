"""Scene 3a (part of the method block) - the joint search.

The lattice cells light up in the order A* actually popped them, for both
agents at once, because the search runs over the joint state of the whole team.
Rejected expansions are shown too, with the reason the planner recorded.

Replayed from video/data/fig1/astar_trace.csv, written by the trace_astar flag
in the planner (pure logging, default off; a run with it on is byte-identical
to one with it off). On this run: 1748 pops, matching results.yaml's
astar_iterations, 8878 pushes and 2702 rejected expansions.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import DATA, load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import Caption
from consort.palette import PRIMARY, SUPPORT, INK, MUTED, BAD, FONT

REASON_TEXT = {
    "obstacle": "its belief would clip an obstacle",
    "goal_spline": "reached the goal, but the spline breaches an obstacle",
    "dominated": "same cell, no shorter and no more certain",
    "primary_unc": "primary would exceed the bound",
    "support_unc": "support would exceed the bound",
    "comm_radius": "support out of comm range",
    "support_dist": "support would outrun the primary",
    "pop_unc": "already over the bound when popped",
}


def load_trace(path):
    pops, prunes = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            nodes = [int(n) for n in r["nodes"].split(";")]
            rec = (int(r["iter"]), nodes, r["reason"])
            if r["ev"] == "pop":
                pops.append(rec)
            elif r["ev"] == "prune":
                prunes.append(rec)
    return pops, prunes


class AStar(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        pops, prunes = load_trace(DATA / "fig1" / "astar_trace.csv")
        nodes = {int(i): (float(x), float(y)) for i, x, y in sc.raw["nodes"]}
        world = World.from_scene(sc, width=10.2, height=4.5, center=UP * 0.55)

        base = base_map(world, sc)
        base.lattice.set_opacity(0.85)
        self.add(base)
        cap = Caption("The search runs over the joint state of both agents at once.")
        self.add(cap)

        # first pop index per (agent, cell); a cell is a lattice position, and
        # many heading states share one, so take the earliest
        first = [defaultdict(lambda: None), defaultdict(lambda: None)]
        n_ag = len(pops[0][1])
        for it, nd, _ in pops:
            for a, node in enumerate(nd):
                p = nodes.get(node)
                if p is None:
                    continue
                key = (round(p[0], 3), round(p[1], 3))
                if first[a][key] is None:
                    first[a][key] = it
        total = pops[-1][0]

        prog = ValueTracker(0.0)
        r = world.length(sc.hex_radius) * 0.92

        def field(a, color):
            items = sorted(first[a].items(), key=lambda kv: kv[1])

            def f():
                k = prog.get_value()
                g = VGroup()
                for (x, y), it in items:
                    if it > k:
                        break
                    age = 1.0 - min((k - it) / max(total, 1), 1.0)
                    g.add(RegularPolygon(6, radius=r, start_angle=PI / 2,
                                         fill_color=color,
                                         fill_opacity=0.16 + 0.42 * age,
                                         stroke_width=0).move_to(world.pt(x, y)))
                return g
            return f

        self.add(always_redraw(field(n_ag - 1, PRIMARY)),
                 always_redraw(field(0, SUPPORT)))

        counter = always_redraw(lambda: VGroup(
            Text(f"{int(prog.get_value())}", font=FONT, font_size=42, color=INK),
            Text("expansions", font=FONT, font_size=19, color=MUTED),
        ).arrange(DOWN, buff=0.06).to_corner(UR, buff=0.35))
        key = VGroup(
            Text("primary states", font=FONT, font_size=20, color=PRIMARY),
            Text("support states", font=FONT, font_size=20, color=SUPPORT),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.10).to_corner(UL, buff=0.35)
        self.add(counter, key)

        self.play(prog.animate.set_value(total * 0.45), run_time=3.4, rate_func=linear)
        cap.set_text("Expanding both agents together is what lets it decide which "
                     "landmark to use and when to relay.")
        self.play(prog.animate.set_value(total), run_time=3.6, rate_func=linear)

        # one rejected expansion, with the planner's own reason
        counts = defaultdict(int)
        for _, _, why in prunes:
            counts[why] += 1
        top = max(counts, key=counts.get)
        ex = next(p for p in reversed(prunes) if p[2] == top)
        pt = nodes.get(ex[1][-1])
        if pt:
            mark = VGroup(
                Cross(Square(side_length=0.22), stroke_color=BAD, stroke_width=4)
                .move_to(world.pt(*pt)),
                Text(REASON_TEXT.get(top, top), font=FONT, font_size=19, color=BAD),
            )
            mark[1].next_to(mark[0], UP, buff=0.12)
            self.play(FadeIn(mark), run_time=0.5)
            cap.set_text(f"{sum(counts.values())} expansions were rejected outright: "
                         f"{counts[top]} because {REASON_TEXT.get(top, top)}.")
            self.wait(2.0)
            self.play(FadeOut(mark), run_time=0.4)

        cap.set_text(f"The first feasible goal state it pops is the seed: "
                     f"{sc.result('primary_disc_unc', 0):.2f} m at the goal.")
        self.wait(2.0)
