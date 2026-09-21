"""Scene 3b (part of the method block) - continuous refinement.

Three stages, matching what actually happens: the discrete route through hex
cell centres (a polyline), those same points becoming B-spline control points
(no shape change yet -- a spline through only 13 points still kinks at every
vertex), then the optimizer moving both the curve AND its control points
together into the shipped result.

Data, all from the same traced run:
  fig1/seed.json      the A* route (joint_astar re-run externally: 1748
                      expansions, seed sigma 1.9201, 1200 m lattice route)
  fig1/cont_ctrls.csv each kept iterate's RAW control points (iteration 0 is
                      seed_control_points itself -- verified byte-identical to
                      seed.json's path, i.e. the hex centres exactly)
  fig1/cont_dense.csv each kept iterate's control points sampled as a spline,
                      by the planner's own bspline_sample_path
  fig1/cont_steps.csv that iterate's length, sigma, stage and barrier mu

The trace comes from the trace_cont flag, which is pure logging: the run that
produced it is byte-identical to the same run with the flag off, and its last
iterate is the shipped 1100.0000 m at sigma 1.7988.
"""
import csv
import json
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
    ctrls = defaultdict(lambda: defaultdict(list))
    with open(DATA / "fig1" / "cont_ctrls.csv") as f:
        for r in csv.DictReader(f):
            ctrls[int(r["iter"])][int(r["agent"])].append((float(r["x"]), float(r["y"])))
    steps = {}
    with open(DATA / "fig1" / "cont_steps.csv") as f:
        for r in csv.DictReader(f):
            steps[int(r["iter"])] = {"len": float(r["len"]), "unc": float(r["unc"]),
                                     "stage": int(r["stage"]), "mu": float(r["mu"])}
    order = sorted(dense)
    as_np = lambda d: {k: {a: np.asarray(v, float) for a, v in dd.items()} for k, dd in d.items()}
    return order, as_np(dense), as_np(ctrls), steps


class Refine(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        seed = json.loads((DATA / "fig1" / "seed.json").read_text())
        order, dense, ctrls, steps = load_iterates()
        n_ag = max(ctrls[0])
        world = World.from_scene(sc, width=10.0, height=4.4, center=UP * 0.55 + LEFT * 0.7)

        base = base_map(world, sc)
        base.lattice.set_opacity(0.5)
        self.add(base)
        cap = Caption("The search returns a route through hex cell centres.")
        self.add(cap)

        def path_colors(a):
            return PRIMARY if a == n_ag else SUPPORT

        # --- stage 1: the discrete polyline through hex centres -----------
        seed0 = ctrls[0]
        lines = {a: polyline(world, seed0[a][:, 0], seed0[a][:, 1],
                             stroke_color=path_colors(a), stroke_width=3.4)
                 for a in sorted(seed0)}
        for a in sorted(seed0):
            self.play(Create(lines[a]), run_time=0.8)

        # --- stage 2: those vertices become control points -----------------
        cap.set_text("Those same points become the control points of a "
                     "clamped cubic B-spline.")
        dots = {a: VGroup(*[Dot(world.pt(*q), radius=0.05, color=path_colors(a))
                            for q in seed0[a]]) for a in sorted(seed0)}
        self.play(*[FadeIn(dots[a]) for a in sorted(seed0)], run_time=0.7)
        ctrl_lab = Text("control points", font=FONT, font_size=19, color=MUTED)
        ctrl_lab.next_to(dots[n_ag][len(dots[n_ag]) // 2], UP, buff=0.18)
        self.play(FadeIn(ctrl_lab), run_time=0.4)
        self.wait(1.0)
        self.play(FadeOut(ctrl_lab), run_time=0.3)

        # --- stage 3: curve AND control points move together ----------------
        idx = ValueTracker(0.0)

        def cur_iter():
            return order[int(np.clip(round(idx.get_value()), 0, len(order) - 1))]

        def curve(a):
            def f():
                k = cur_iter()
                p = dense[k][a]
                return polyline(world, p[:, 0], p[:, 1], stroke_color=path_colors(a),
                                stroke_width=4.0 if a == n_ag else 3.0)
            return f

        curves = {a: always_redraw(curve(a)) for a in sorted(seed0)}
        for a in sorted(seed0):
            self.remove(lines[a])
            self.add(curves[a])

        def ctrl_dots(a):
            def f():
                k = cur_iter()
                pts = ctrls[k].get(a, ctrls[0][a])
                return VGroup(*[Dot(world.pt(*q), radius=0.05, color=path_colors(a))
                               for q in pts])
            return f

        for a in sorted(seed0):
            self.remove(dots[a])
            self.add(always_redraw(ctrl_dots(a)))

        def readout():
            k = cur_iter()
            st = steps[k]
            return VGroup(
                Text(f"iteration {k}", font=FONT, font_size=21, color=MUTED),
                Text(f"{st['len']:.1f} m", font=FONT, font_size=44, color=PRIMARY),
                Text(f"σ = {st['unc']:.2f} m   ū = {sc.bound:g} m",
                     font=FONT, font_size=21, color=INK),
                Text(f"barrier stage {st['stage']}", font=FONT, font_size=17, color=MUTED),
            ).arrange(DOWN, buff=0.10).to_edge(RIGHT, buff=0.45).shift(UP * 0.4)

        self.add(always_redraw(readout))
        lattice_note = Text(f"lattice route {seed['primary_dist']:.0f} m  →  spline through "
                            f"those points {steps[order[0]]['len']:.0f} m",
                            font=FONT, font_size=19, color=MUTED)
        lattice_note.to_corner(UL, buff=0.32)
        self.add(lattice_note)

        cap.set_text("The optimizer moves the curve and its control points "
                     "together, under the same uncertainty and curvature "
                     "constraints.")
        self.play(idx.animate.set_value(len(order) - 1), run_time=8.0, rate_func=linear)
        self.wait(0.8)

        final = Text(f"{len(order)} of the optimizer's accepted iterates, in order",
                     font=FONT, font_size=18, color=MUTED)
        final.next_to(cap, UP, buff=0.22)
        self.play(FadeIn(final), run_time=0.5)
        cap.set_text("Across the benchmark the refinement takes 2 to 7% off the "
                     "lattice route.")
        self.wait(2.0)
