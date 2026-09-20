"""Scene 4 (1:15-1:45) - the baselines, on one scenario at one bound.

Each baseline keeps the covariance model, the obstacle test and the refinement,
and removes exactly one piece of joint routing. Four panels say which piece,
then all five primaries are overlaid with their lengths and terminal sigma.

Scenario s049 of the 50-scenario benchmark at the 50% constraint level, re-run
with geometry emitted (the sweep itself stored numbers only). Every length here
is byte-identical to that sweep's trials_all.csv; greedy is the rung the sweep
skipped under method_patience and is re-run at the same threshold, where it
still fails.
"""
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mobjects import Caption, polyline, obstacle_poly
from consort.palette import METHODS, PRIMARY, SUPPORT, GOAL, LANDMARK, INK, MUTED, OK, BAD, FONT

SID = "s049_p050"
PANELS = [
    ("greedy",     "removes lookahead",
     "the support takes the best one-step move and never reaches a landmark"),
    ("formation",  "removes support routing",
     "the support is locked one cell off the primary, so the primary detours itself"),
    ("sequential", "removes the joint state space",
     "the primary plans with the support parked, then detours alone"),
    ("clgbt",      "replaces lattice search with sampling",
     "a tree grows in the joint space and returns the first path it finds"),
]


class Baselines(Scene):
    def construct(self):
        scenes = {m: load_scene(f"baselines/{SID}/{m}.json")
                  for m in ["hexspline_cl"] + [p[0] for p in PANELS]}
        ours = scenes["hexspline_cl"]
        bound = ours.bound

        cap = Caption("Each baseline removes one piece of joint routing. "
                      "Same scenario, same bound.")
        self.add(cap)

        # --- 2x2 grid ---
        slots = [LEFT * 3.4 + UP * 1.55, RIGHT * 3.4 + UP * 1.55,
                 LEFT * 3.4 + DOWN * 1.45, RIGHT * 3.4 + DOWN * 1.45]
        grid = VGroup()
        for (m, removes, how), c in zip(PANELS, slots):
            s = scenes[m]
            w = World.from_scene(s, width=5.7, height=2.05, center=c)
            ok = s.result("primary_unc", 1e9) <= bound + 1e-9
            g = VGroup(obstacle_poly(w, s.obstacles[0]) if s.obstacles else VGroup())
            for o in s.obstacles[1:]:
                g.add(obstacle_poly(w, o))
            for lm in s.landmarks:
                g.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                      .set(width=0.13).move_to(w.pt(lm["x"], lm["y"])))
            g.add(Star(n=5, outer_radius=0.10, color=GOAL, fill_opacity=1,
                       stroke_width=0).move_to(w.pt(*s.goal)))
            for t in s.supports:
                g.add(polyline(w, t.draw_x, t.draw_y, stroke_color=SUPPORT, stroke_width=2.0))
            g.add(polyline(w, s.primary.draw_x, s.primary.draw_y,
                           stroke_color=METHODS[m][1], stroke_width=2.8))
            head = VGroup(
                Text(METHODS[m][0], font=FONT, font_size=23, color=METHODS[m][1]),
                Text(f"✓ σ = {s.result('primary_unc'):.2f} m" if ok
                     else f"✗ σ = {s.result('primary_unc'):.2f} m > ū",
                     font=FONT, font_size=19, color=OK if ok else BAD),
            ).arrange(RIGHT, buff=0.24)
            head.next_to(g, UP, buff=0.06)
            sub = Text(removes, font=FONT, font_size=17, color=MUTED)
            sub.next_to(g, DOWN, buff=0.06)
            grid.add(VGroup(g, head, sub))

        for panel, (m, removes, how) in zip(grid, PANELS):
            self.play(FadeIn(panel), run_time=0.45)
            cap.set_text(f"{METHODS[m][0]} {removes}: {how}.")
            self.wait(1.7)

        self.wait(0.6)
        self.play(FadeOut(grid), run_time=0.6)

        # --- all five primaries, same frame ---
        w = World.from_scene(ours, width=9.4, height=4.0, center=UP * 0.75)
        base = VGroup(*[obstacle_poly(w, o) for o in ours.obstacles])
        for lm in ours.landmarks:
            base.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                     .set(width=0.17).move_to(w.pt(lm["x"], lm["y"])))
        base.add(Dot(w.pt(*ours.start), radius=0.07, color=INK))
        base.add(Star(n=5, outer_radius=0.15, color=GOAL, fill_opacity=1,
                      stroke_width=0).move_to(w.pt(*ours.goal)))
        self.play(FadeIn(base), run_time=0.5)
        cap.set_text("All five primary paths, same scenario, same bound.")

        rows = VGroup()
        for m in ["hexspline_cl"] + [p[0] for p in PANELS]:
            s = scenes[m]
            ok = s.result("primary_unc", 1e9) <= bound + 1e-9
            ln = polyline(w, s.primary.draw_x, s.primary.draw_y,
                          stroke_color=METHODS[m][1],
                          stroke_width=4.0 if m == "hexspline_cl" else 2.4)
            self.play(Create(ln), run_time=0.5)
            rows.add(VGroup(
                Text(METHODS[m][0], font=FONT, font_size=20, color=METHODS[m][1]),
                Text(f"{s.result('primary_length'):.0f} m", font=FONT, font_size=20, color=INK),
                Text("✓" if ok else "✗", font=FONT, font_size=20, color=OK if ok else BAD),
            ).arrange(RIGHT, buff=0.30))
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.13).to_corner(UL, buff=0.32)
        self.play(FadeIn(rows), run_time=0.6)

        note = Text(f"one scenario at ū = {bound:.2f} m; the aggregate comes next",
                    font=FONT, font_size=19, color=MUTED)
        note.next_to(cap, UP, buff=0.22)
        self.play(FadeIn(note), run_time=0.5)
        self.wait(2.4)
