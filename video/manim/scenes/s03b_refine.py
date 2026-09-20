"""Scene 3b (part of the method block) - continuous refinement.

The discrete search returns a route through hex cell centres. Those centres
become the control points of a clamped cubic B-spline, and the refinement
shortens the primary's path subject to the same uncertainty and curvature
constraints.

Two data files, both re-derived from the shipped run with no planner edit:
  fig1/seed.json   the A* seed (joint_astar re-run externally; it reproduces the
                   run's 1748 iterations and seed sigma 1.9201)
  fig1/scene.json  the refined control points the run actually shipped

WHAT THIS DOES NOT SHOW: the optimizer's own iterates. Those live inside
optimize_continuous and are never returned, so the morph here is a tween
between the seed and the shipped result, not a replay of Adam. It is captioned
as such. Playing the real iterates needs the logging tap in the plan, which is
pending approval; s03a_astar.py is blocked on the same tap.
"""
import json
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import DATA, load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import Caption, polyline
from consort.palette import PRIMARY, SUPPORT, INK, MUTED, FONT


class Refine(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        seed = json.loads((DATA / "fig1" / "seed.json").read_text())
        world = World.from_scene(sc, width=10.2, height=4.6, center=UP * 0.55)

        base = base_map(world, sc)
        base.lattice.set_opacity(0.55)
        self.add(base)
        cap = Caption("The search returns a route through hex cell centres.")
        self.add(cap)

        seed_pts = {int(a["primary"]): np.asarray(a["ctrls"], float)
                    for a in seed["agents"]}
        sp, ss = seed_pts[1], seed_pts[0]

        # the seed polygon: straight hops between cell centres
        seed_line_p = polyline(world, sp[:, 0], sp[:, 1], stroke_color=PRIMARY,
                               stroke_width=3.4)
        seed_line_s = polyline(world, ss[:, 0], ss[:, 1], stroke_color=SUPPORT,
                               stroke_width=2.8)
        dots_p = VGroup(*[Dot(world.pt(*p), radius=0.055, color=PRIMARY) for p in sp])
        dots_s = VGroup(*[Dot(world.pt(*p), radius=0.045, color=SUPPORT) for p in ss])
        self.play(Create(seed_line_s), Create(seed_line_p), run_time=1.2)
        self.play(FadeIn(dots_s), FadeIn(dots_p), run_time=0.6)

        panel = VGroup(
            Text(f"lattice route   {seed['primary_dist']:.0f} m",
                 font=FONT, font_size=26, color=PRIMARY),
            Text(f"seed σ   {seed['seed_unc']:.2f} m", font=FONT, font_size=22, color=INK),
            Text(f"found in {seed['astar_iterations']} expansions",
                 font=FONT, font_size=19, color=MUTED),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12).to_corner(UL, buff=0.32)
        self.play(FadeIn(panel), run_time=0.5)
        cap.set_text("Those centres become the control points of a clamped cubic B-spline.")
        self.wait(1.6)

        # the shipped refinement
        p, s = sc.primary, sc.supports[0]
        ref_p = polyline(world, p.draw_x, p.draw_y, stroke_color=PRIMARY, stroke_width=4.0)
        ref_s = polyline(world, s.draw_x, s.draw_y, stroke_color=SUPPORT, stroke_width=3.2)
        ref_dots_p = VGroup(*[Dot(world.pt(*q), radius=0.055, color=PRIMARY)
                              for q in p.ctrls])
        ref_dots_s = VGroup(*[Dot(world.pt(*q), radius=0.045, color=SUPPORT)
                              for q in s.ctrls])

        cap.set_text("Refinement shortens the primary's path under the same "
                     "uncertainty and curvature constraints.")
        panel2 = VGroup(
            Text(f"refined path   {sc.result('primary_length'):.0f} m",
                 font=FONT, font_size=26, color=PRIMARY),
            Text(f"terminal σ   {sc.result('primary_unc'):.2f} m  <  ū = {sc.bound:g} m",
                 font=FONT, font_size=22, color=INK),
            Text(f"{100*(1 - sc.result('primary_length')/seed['primary_dist']):.0f}% shorter "
                 f"than the lattice route", font=FONT, font_size=19, color=MUTED),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12).to_corner(UL, buff=0.32)

        self.play(Transform(seed_line_p, ref_p), Transform(seed_line_s, ref_s),
                  Transform(dots_p, ref_dots_p), Transform(dots_s, ref_dots_s),
                  Transform(panel, panel2), run_time=2.4)
        self.wait(1.0)

        honest = Text("seed → shipped result; the optimizer's iterates are not logged",
                      font=FONT, font_size=17, color=MUTED)
        honest.next_to(cap, UP, buff=0.22)
        self.play(FadeIn(honest), run_time=0.5)
        cap.set_text("Across the benchmark the refinement takes 2 to 7% off the "
                     "lattice route.")
        self.wait(2.2)
