"""Smoke test: confirms manim renders text + shapes + writes an mp4."""
from manim import *


class Smoke(Scene):
    def construct(self):
        t = Text("CONSORT — σ, ū, Σ, ×30", color=BLACK, font_size=48)
        c = Circle(radius=1.2, color=BLUE).next_to(t, DOWN, buff=0.6)
        self.play(Write(t), run_time=1.0)
        self.play(Create(c), run_time=1.0)
