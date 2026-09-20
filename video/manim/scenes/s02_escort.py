"""Scene 2 (0:12-0:40) - the escort problem, Fig. 1 animated.

The lattice draws in, the support dives to the landmark and its ellipse
collapses, every 100 m checkpoint flashes (grey when the two are out of comm
range, green when they fuse), and the one fusion at the goal pulls the primary
under the bound.

Data: video/data/fig1/scene.json, exported from the run behind Fig. 1
(paper/new_draft/figures/figure1/D_island/tight). Nothing is recomputed here.
"""
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import cov_ellipse, UncMeter, AUVGlyph, Caption, polyline
from consort.palette import (
    PRIMARY, SUPPORT, COMM, COMM_DEAD, LANDMARK, INK, MUTED, FONT,
)

SIGMA_SCALE = 10.0      # sigma x10 for visibility, as Fig. 1 does
FLASH_WIN = 26.0        # metres of arc either side of a checkpoint that a flash spans
LM_WIN = 30.0


class Escort(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        world = World.from_scene(sc, width=10.4, height=5.0, center=LEFT * 1.15 + UP * 0.30)
        primary, support = sc.primary, sc.supports[0]
        arc_end = primary.arc_end

        base = base_map(world, sc)
        cap = Caption("")
        self.add(cap)

        # --- the lattice the search runs on ---
        cap.set_text("Both agents are searched on one heading-aware hex lattice.")
        self.play(LaggedStart(*[Create(h) for h in base.lattice],
                              lag_ratio=0.004, run_time=2.0))
        self.play(FadeIn(base.obstacles), FadeIn(base.landmarks),
                  FadeIn(base.start), FadeIn(base.goal), run_time=0.8)

        lm = sc.landmarks[0]
        lm_lab = Text("landmark", font=FONT, font_size=19, color=LANDMARK)
        lm_lab.next_to(world.pt(lm["x"], lm["y"]), RIGHT, buff=0.14)
        self.play(FadeIn(lm_lab), run_time=0.4)

        # --- the scrubber every updater reads ---
        arc = ValueTracker(0.0)
        meter = UncMeter(sc.bound, vmax=2.6, height=2.9)
        meter.to_edge(RIGHT, buff=0.85).shift(UP * 0.35)
        meter.add_updater(lambda m: m.set_value(primary.unc_at(arc.get_value())))
        self.add(meter)

        trail_p = always_redraw(lambda: polyline(
            world, *primary.upto(arc.get_value()), stroke_color=PRIMARY, stroke_width=4.0))
        trail_s = always_redraw(lambda: polyline(
            world, *support.upto(arc.get_value()), stroke_color=SUPPORT, stroke_width=3.4))

        ell_p = always_redraw(lambda: cov_ellipse(
            world, primary.pos_at(arc.get_value()), primary.cov_at(arc.get_value()),
            nstd=2, sigma_scale=SIGMA_SCALE, color=PRIMARY, fill_opacity=0.20))
        ell_s = always_redraw(lambda: cov_ellipse(
            world, support.pos_at(arc.get_value()), support.cov_at(arc.get_value()),
            nstd=2, sigma_scale=SIGMA_SCALE, color=SUPPORT, fill_opacity=0.20))

        auv_p = AUVGlyph(PRIMARY)
        auv_s = AUVGlyph(SUPPORT)
        auv_p.add_updater(lambda m: m.place(world, *primary.pos_at(arc.get_value()),
                                            heading=primary.heading_at(arc.get_value())))
        auv_s.add_updater(lambda m: m.place(world, *support.pos_at(arc.get_value()),
                                            heading=support.heading_at(arc.get_value())))

        # --- comm checkpoints: one flash per 100 m, colour says whether it fused ---
        cps = sc.checkpoints

        def comm_link():
            a = arc.get_value()
            near = min(cps, key=lambda c: abs(c["arc"] - a)) if cps else None
            if near is None or abs(near["arc"] - a) > FLASH_WIN or near["arc"] <= 0:
                return VGroup()
            fade = 1.0 - abs(near["arc"] - a) / FLASH_WIN
            col = COMM if near["fused"] else COMM_DEAD
            pa, pb = support.pos_at(a), primary.pos_at(a)
            ln = DashedLine(world.pt(*pa), world.pt(*pb), stroke_color=col,
                            stroke_width=4.0 if near["fused"] else 2.2,
                            dash_length=0.12).set_opacity(fade)
            txt = Text(f"comm at {near['arc']:.0f} m: fused, w = {near['w']:.2f}"
                       if near["fused"]
                       else f"comm at {near['arc']:.0f} m: {near['d']:.0f} m apart, out of range",
                       font=FONT, font_size=20, color=col).set_opacity(fade)
            txt.to_edge(UP, buff=0.22)
            return VGroup(ln, txt)

        # --- the support's landmark sightings ---
        lmes = [e for e in sc.landmark_events if e["q"] > 0.05]

        def lm_ray():
            a = arc.get_value()
            hit = [e for e in lmes if abs(e["arc"] - a) < LM_WIN]
            if not hit:
                return VGroup()
            e = min(hit, key=lambda e: abs(e["arc"] - a))
            fade = 1.0 - abs(e["arc"] - a) / LM_WIN
            ray = DashedLine(world.pt(*support.pos_at(a)), world.pt(*e["pl"]),
                             stroke_color=LANDMARK, stroke_width=2.6,
                             dash_length=0.09).set_opacity(fade)
            return VGroup(ray)

        self.add(trail_s, trail_p, ell_s, ell_p,
                 always_redraw(comm_link), always_redraw(lm_ray), auv_s, auv_p)

        sig_note = Text("covariance drawn at 2σ, σ ×10", font=FONT, font_size=17, color=MUTED)
        sig_note.next_to(meter, DOWN, buff=0.55)
        self.add(sig_note)

        cap.set_text("The support has no goal of its own. It is routed only to keep "
                     "the primary under the bound.")
        self.play(arc.animate.set_value(620), run_time=7.0, rate_func=linear)

        cap.set_text("It diverts to the landmark. Its own uncertainty collapses.")
        self.play(arc.animate.set_value(820), run_time=4.5, rate_func=linear)

        cap.set_text("Out of comm range for 800 m, the primary is on dead reckoning.")
        self.play(arc.animate.set_value(1060), run_time=4.5, rate_func=linear)

        cap.set_text("One fusion at the goal brings the primary back under ū.")
        self.play(arc.animate.set_value(arc_end), run_time=3.0, rate_func=linear)
        meter.clear_updaters()
        self.wait(1.2)

        final = VGroup(
            Text(f"primary σ = {primary.unc_at(arc_end):.2f} m  <  ū = {sc.bound:g} m",
                 font=FONT, font_size=25, color=INK),
            Text(f"path {sc.result('primary_length', 0):.0f} m "
                 f"(alone it would need a longer route and still miss the bound)",
                 font=FONT, font_size=20, color=MUTED),
        ).arrange(DOWN, buff=0.16).to_corner(UL, buff=0.32)
        self.play(FadeIn(final), run_time=0.8)
        self.wait(1.4)
