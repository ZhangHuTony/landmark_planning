"""Scene 3c (part of the method block) - the constraint ladder.

One scenario, four terminal bounds. As u-bar tightens from 100% to 30% of the
single-agent reference, the plan changes shape and the primary's path gets
longer. This is Fig. 2 of the paper, animated: the numbers on screen are the
rungs of fig2_ladder/rect_r3 and match Table I.
"""
import sys
from pathlib import Path
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import Caption, polyline
from consort.palette import PRIMARY, SUPPORT, INK, MUTED, FONT

RUNGS = ["100", "070", "050", "030"]
STORY = {
    "100": "Loosest bound: the support ducks into the north pocket for the weak "
           "landmark there and rejoins.",
    "070": "Tighter: it takes the southern lane to a stronger landmark and relays "
           "from behind the last wall.",
    "050": "Tighter still: the primary adds a turn before the goal so the support "
           "can relay from closer.",
    "030": "Tightest: the primary dips to the centre line so the support can reach "
           "the last landmark in time.",
}


class Ladder(Scene):
    def construct(self):
        scenes = {p: load_scene(f"ladder/scene_p{p}.json") for p in RUNGS}
        s0 = scenes["100"]
        world = World.from_scene(s0, width=11.6, height=4.1, center=UP * 0.35)

        base = base_map(world, s0, with_lattice=False)
        self.add(base)

        cap = Caption("The search decides which landmark to use, and when to relay.")
        self.add(cap)

        def lines(s):
            p, sup = s.primary, s.supports[0]
            return (polyline(world, p.draw_x, p.draw_y, stroke_color=PRIMARY, stroke_width=4.2),
                    polyline(world, sup.draw_x, sup.draw_y, stroke_color=SUPPORT,
                             stroke_width=3.2))

        pl, sl = lines(s0)
        self.add(sl, pl)

        def panel(s, pct):
            ref = 1712.1        # the single-agent reference for this scenario
            return VGroup(
                Text(f"bound  ū = {s.bound:.2f} m", font=FONT, font_size=26, color=INK),
                Text(f"{pct}% of the reference", font=FONT, font_size=20, color=MUTED),
                Text(f"primary path   {s.result('primary_length'):.0f} m",
                     font=FONT, font_size=26, color=PRIMARY),
                Text(f"terminal σ   {s.result('primary_unc'):.2f} m",
                     font=FONT, font_size=24, color=INK),
                Text(f"reference {ref:.0f} m", font=FONT, font_size=18, color=MUTED),
            ).arrange(DOWN, aligned_edge=LEFT, buff=0.11).to_corner(UL, buff=0.28)

        pan = panel(s0, "100")
        self.add(pan)
        cap.set_text(STORY["100"])
        self.wait(2.2)

        for pct in RUNGS[1:]:
            s = scenes[pct]
            npl, nsl = lines(s)
            self.play(Transform(pl, npl), Transform(sl, nsl),
                      Transform(pan, panel(s, str(int(pct)))), run_time=1.5)
            cap.set_text(STORY[pct])
            self.wait(2.0)

        summary = Text("A tighter bound is paid for in primary path length: "
                       "1741 m → 1890 m.", font=FONT, font_size=25, color=INK)
        summary.next_to(cap, UP, buff=0.30)
        self.play(FadeIn(summary), run_time=0.6)
        self.wait(1.8)
