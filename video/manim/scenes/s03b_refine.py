"""Scene 3b (part of the method block) - continuous refinement.

The discrete search returns a route through hex cell centres. Those centres
become the control points of a clamped cubic B-spline, and the refinement
shortens the primary's path subject to the same uncertainty and curvature
constraints. This plays the optimizer's ACCEPTED ITERATES in order.

Data, all from the same traced run:
  fig1/seed.json      the A* route (joint_astar re-run externally: 1748
                      expansions, seed sigma 1.9201, 1200 m lattice route)
  fig1/cont_dense.csv each kept iterate's control points sampled as a spline,
                      by the planner's own bspline_sample_path
  fig1/cont_steps.csv that iterate's length, sigma, stage and barrier mu

The trace comes from the trace_cont flag, which is pure logging: the run that
produced it is byte-identical to the same run with the flag off, and its last
iterate is the shipped 1100.0000 m at sigma 1.7988.
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
from consort.mobjects import Caption, polyline
from consort.palette import PRIMARY, SUPPORT, INK, MUTED, FONT


def load_iterates():
    dense = defaultdict(lambda: defaultdict(list))
    with open(DATA / "fig1" / "cont_dense.csv") as f:
        for r in csv.DictReader(f):
            dense[int(r["iter"])][int(r["agent"])].append((float(r["x"]), float(r["y"])))
    steps = {}
    with open(DATA / "fig1" / "cont_steps.csv") as f:
        for r in csv.DictReader(f):
            steps[int(r["iter"])] = {"len": float(r["len"]), "unc": float(r["unc"]),
                                     "stage": int(r["stage"]), "mu": float(r["mu"])}
    order = sorted(dense)
    return order, {k: {a: np.asarray(v, float) for a, v in d.items()}
                   for k, d in dense.items()}, steps


class Refine(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        order, dense, steps = load_iterates()
        world = World.from_scene(sc, width=10.0, height=4.4, center=UP * 0.55 + LEFT * 0.7)

        base = base_map(world, sc)
        base.lattice.set_opacity(0.5)
        self.add(base)
        cap = Caption("The search returns a route through hex cell centres.")
        self.add(cap)

        # iterate 0 is the spline through the seed's control points
        n_ag = max(dense[0])
        seed_ctrl = {a: np.asarray(t.ctrls, float)
                     for a, t in enumerate(sc.agents, start=1)}

        dots = VGroup(*[Dot(world.pt(*q), radius=0.05,
                            color=PRIMARY if a == n_ag else SUPPORT)
                        for a in sorted(dense[0]) for q in seed_ctrl[a]])
        self.play(FadeIn(dots), run_time=0.7)
        cap.set_text("Those centres become the control points of a clamped cubic B-spline.")
        self.wait(1.2)

        idx = ValueTracker(0.0)

        def curve(a, color, w):
            def f():
                k = order[int(np.clip(round(idx.get_value()), 0, len(order) - 1))]
                p = dense[k][a]
                return polyline(world, p[:, 0], p[:, 1], stroke_color=color, stroke_width=w)
            return f

        self.add(always_redraw(curve(1, SUPPORT, 3.0)),
                 always_redraw(curve(n_ag, PRIMARY, 4.0)))

        def readout():
            k = order[int(np.clip(round(idx.get_value()), 0, len(order) - 1))]
            st = steps[k]
            return VGroup(
                Text(f"iteration {k}", font=FONT, font_size=21, color=MUTED),
                Text(f"{st['len']:.1f} m", font=FONT, font_size=44, color=PRIMARY),
                Text(f"σ = {st['unc']:.2f} m   ū = {sc.bound:g} m",
                     font=FONT, font_size=21, color=INK),
                Text(f"barrier stage {st['stage']}", font=FONT, font_size=17, color=MUTED),
            ).arrange(DOWN, buff=0.10).to_edge(RIGHT, buff=0.45).shift(UP * 0.4)

        self.add(always_redraw(readout))
        lattice_note = Text(f"lattice route {1200:.0f} m  →  spline through those points "
                            f"{steps[order[0]]['len']:.0f} m",
                            font=FONT, font_size=19, color=MUTED)
        lattice_note.to_corner(UL, buff=0.32)
        self.add(lattice_note)

        cap.set_text("Refinement then shortens the primary's path under the same "
                     "uncertainty and curvature constraints.")
        self.play(idx.animate.set_value(len(order) - 1), run_time=8.0, rate_func=linear)
        self.wait(0.8)

        final = Text(f"{len(order)} of the optimizer's accepted iterates, in order",
                     font=FONT, font_size=18, color=MUTED)
        final.next_to(cap, UP, buff=0.22)
        self.play(FadeIn(final), run_time=0.5)
        cap.set_text("Across the benchmark the refinement takes 2 to 7% off the "
                     "lattice route.")
        self.wait(2.0)
