"""Scene 1 (0:00-0:12) - the problem.

One AUV flies the primary's own route with no support. Its covariance grows by
dead reckoning and the terminal uncertainty ends over the bound.

The track is the `alone` block of video/data/fig1/scene.json: the planner's own
evaluate_joint_discrete run on the same path with na=1, so nothing fuses.
"""
import sys
from pathlib import Path
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import cov_ellipse, UncMeter, AUVGlyph, Caption, polyline
from consort.geometry import make_local_scale
from consort.palette import PRIMARY, INK, MUTED, BAD, FONT


class Problem(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        world = World.from_scene(sc, width=10.4, height=5.0, center=LEFT * 1.15 + UP * 0.30)
        alone = sc.alone
        arc_end = alone.arc_end

        base = base_map(world, sc, with_lattice=False)
        base.landmarks.set_opacity(0.25)      # present, but this agent cannot use them
        self.add(base)

        cap = Caption("Without GPS, dead reckoning drifts.")
        self.add(cap)

        arc = ValueTracker(0.0)
        meter = UncMeter(sc.bound, vmax=2.6, height=2.9, label="terminal σ")
        meter.to_edge(RIGHT, buff=0.85).shift(UP * 0.35)
        meter.add_updater(lambda m: m.set_value(alone.unc_at(arc.get_value())))
        self.add(meter)

        scale_at = make_local_scale(sc.obstacles, default=6.0, floor=1.5, margin=3.0)
        trail = always_redraw(lambda: polyline(
            world, *alone.upto(arc.get_value()), stroke_color=PRIMARY, stroke_width=4.0))
        ell = always_redraw(lambda: cov_ellipse(
            world, alone.pos_at(arc.get_value()), alone.cov_at(arc.get_value()),
            nstd=2, sigma_scale=scale_at, color=PRIMARY))
        auv = AUVGlyph(PRIMARY)
        auv.add_updater(lambda m: m.place(world, *alone.pos_at(arc.get_value()),
                                          heading=alone.heading_at(arc.get_value())))
        note = Text("2σ, magnified for visibility",
                    font=FONT, font_size=17, color=MUTED)
        note.next_to(meter, DOWN, buff=0.55)
        self.add(trail, ell, auv, note)

        self.play(arc.animate.set_value(arc_end * 0.55), run_time=4.0, rate_func=linear)
        cap.set_text("A single agent has nothing to correct against.")
        self.play(arc.animate.set_value(arc_end), run_time=4.4, rate_func=linear)
        meter.clear_updaters()

        cap.set_text("It reaches the goal too uncertain to be useful.")
        verdict = Text(f"σ = {alone.unc_at(arc_end):.2f} m  >  ū = {sc.bound:g} m",
                       font=FONT, font_size=30, color=BAD)
        verdict.to_corner(UL, buff=0.35)
        self.play(FadeIn(verdict), run_time=0.6)
        self.wait(2.0)
