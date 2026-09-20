"""Scene 7 (2:50-3:00) - the closing card.

Three numbers, all read from video/data/holo/numbers.json, which
video/python/export_sweep.py recomputes from the sweep and Monte Carlo CSVs and
checks against the paper before writing.

Set ANON=1 in the environment for the PaperPlaza cut: the project page line
becomes a placeholder, since that attachment must carry no URL.
"""
import json
import math
import os
import sys
from pathlib import Path
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import DATA
from consort.palette import INK, MUTED, OK, FONT

ANON = os.environ.get("ANON", "1") == "1"


class Close(Scene):
    def construct(self):
        n = json.loads((DATA / "holo" / "numbers.json").read_text())["headline"]

        def card(big, small, color=INK):
            return VGroup(
                Text(big, font=FONT, font_size=50, color=color, weight="BOLD"),
                Text(small, font=FONT, font_size=21, color=MUTED),
            ).arrange(DOWN, buff=0.22)

        cards = VGroup(
            card(f"{100*n['ours_success_50']:.0f}%  vs  {100*n['formation_success_50']:.0f}%",
                 "scenarios solved at half the reference bound"),
            card(n["collisions"].replace("/", " / "), "collisions in closed-loop simulation", OK),
            card(f"within {math.ceil(100 * n['prediction_error']):.0f}%",
                 "predicted terminal uncertainty vs. the flown sample"),
        ).arrange(DOWN, buff=0.62)

        title = Text("Escorted navigation under a localisation constraint",
                     font=FONT, font_size=30, color=INK)
        title.to_edge(UP, buff=0.7)

        self.play(FadeIn(title), run_time=0.6)
        for c in cards:
            self.play(FadeIn(c, shift=UP * 0.16), run_time=0.45)
        self.wait(1.6)

        page = Text("project page: anonymised for review" if ANON
                    else "project page: [anonymized].github.io",
                    font=FONT, font_size=22, color=MUTED)
        page.to_edge(DOWN, buff=0.55)
        self.play(FadeIn(page), run_time=0.5)
        self.wait(1.8)
