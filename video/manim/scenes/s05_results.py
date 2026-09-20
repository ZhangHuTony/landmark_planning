"""Scene 5 (1:45-2:15) - the benchmark, as a moving trade-off.

x = fraction of the 50 scenarios solved, y = mean primary path length over the
solved ones (normalised by the single-agent reference). Bottom right is better:
solve more, fly less. A tracker sweeps the constraint level from 100% down to
30% and every planner's dot walks its own trade-off curve.

Two honesty devices, both from the paper's Table I: a dot fades as its planner
solves fewer scenarios, and a planner averaging over fewer than five solved
scenarios is drawn hollow. That is what stops Greedy's 1.13 at 4% success from
looking like the shortest path in the room.
"""
import json
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import DATA
from consort.mobjects import Caption
from consort.palette import METHODS, INK, MUTED, FONT

LEVELS = [100, 90, 80, 70, 60, 50, 40, 30]
YCLIP = 2.2
SHAPES = {"hexspline_cl": "D", "formation": "s", "sequential": "^",
          "clgbt": "o", "greedy": "v"}


def glyph(kind, color, size=0.20, hollow=False):
    if kind == "D":
        m = Square(side_length=size * 1.5).rotate(PI / 4)
    elif kind == "s":
        m = Square(side_length=size * 1.4)
    elif kind == "^":
        m = Triangle().set(width=size * 1.8)
    elif kind == "v":
        m = Triangle().set(width=size * 1.8).rotate(PI)
    else:
        m = Circle(radius=size * 0.8)
    m.set_stroke(color, width=2.6)
    m.set_fill(WHITE if hollow else color, opacity=1.0 if not hollow else 1.0)
    return m


class Results(Scene):
    def construct(self):
        d = json.loads((DATA / "sweep" / "levels.json").read_text())
        base = d["baseline"]

        ax = Axes(x_range=[0, 105, 20], y_range=[0.95, YCLIP, 0.2],
                  x_length=8.4, y_length=4.7,
                  axis_config={"color": MUTED, "stroke_width": 2,
                               "include_ticks": True, "font_size": 22},
                  tips=False).shift(LEFT * 1.5 + DOWN * 0.15)
        xlab = Text("scenarios solved (%)", font=FONT, font_size=24, color=INK)
        xlab.next_to(ax.x_axis, DOWN, buff=0.38)
        ylab = Text("path length / reference", font=FONT, font_size=24, color=INK)
        ylab.rotate(PI / 2).next_to(ax.y_axis, LEFT, buff=0.32)
        self.add(ax, xlab, ylab,
                 VGroup(*[Text(f"{v}", font=FONT, font_size=19, color=MUTED)
                          .next_to(ax.c2p(v, 0.95), DOWN, buff=0.14) for v in (0, 20, 40, 60, 80, 100)]),
                 VGroup(*[Text(f"{v:.1f}", font=FONT, font_size=19, color=MUTED)
                          .next_to(ax.c2p(0, v), LEFT, buff=0.14) for v in (1.0, 1.4, 1.8, 2.2)]))

        better = Text("better", font=FONT, font_size=22, color=INK)
        better.next_to(ax.c2p(96, 1.03), UP, buff=0.22)
        arrow = Arrow(ax.c2p(72, 1.30), ax.c2p(96, 1.03), buff=0, color=INK,
                      stroke_width=3, max_tip_length_to_length_ratio=0.12)
        self.add(better, arrow)

        # per-planner series, in sweep order
        series = {}
        for m in METHODS:
            pts = []
            for pct in LEVELS:
                r = base.get(f"{m}@{pct}")
                if not r:
                    continue
                pts.append((pct, 100 * r["success_rate"],
                            r["mean_ratio"] if r["mean_ratio"] else None,
                            r["n_success"], r["dagger"]))
            series[m] = pts

        lvl = ValueTracker(100.0)

        def interp(m):
            """Where planner m sits at the current (continuous) constraint level."""
            pts = [p for p in series[m] if p[2] is not None]
            if not pts:
                return None
            L = lvl.get_value()
            xs = [p[0] for p in pts]
            if L >= xs[0]:
                p = pts[0]
                return p[1], p[2], p[3], p[4]
            if L <= xs[-1]:
                p = pts[-1]
                return p[1], p[2], p[3], p[4]
            for i in range(len(pts) - 1):
                a, b = pts[i], pts[i + 1]
                if b[0] <= L <= a[0]:
                    t = (a[0] - L) / max(a[0] - b[0], 1e-9)
                    return (a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2]),
                            a[3] + t * (b[3] - a[3]), b[4] if t > 0.5 else a[4])
            return None

        dots, traces = VGroup(), VGroup()
        for m, (name, color) in METHODS.items():
            def mk(m=m, color=color):
                st = interp(m)
                if st is None:
                    return VGroup()
                sr, ratio, nsucc, dag = st
                g = glyph(SHAPES[m], color, hollow=bool(dag))
                g.move_to(ax.c2p(sr, min(ratio, YCLIP)))
                # opacity carries how many scenarios the mean is actually over
                g.set_opacity(float(np.clip(0.25 + 0.75 * nsucc / 50.0, 0.2, 1.0)))
                if ratio > YCLIP:                       # off the top of the axis
                    tip = Triangle(color=color, fill_opacity=1.0, stroke_width=0)
                    tip.set(width=0.16).move_to(ax.c2p(sr, YCLIP) + UP * 0.18)
                    return VGroup(g, tip)
                return g
            dm = always_redraw(mk)
            dots.add(dm)
            tr = TracedPath(dm.get_center, stroke_color=color, stroke_width=2.0,
                            stroke_opacity=0.45)
            traces.add(tr)
        self.add(traces, dots)

        key = VGroup(*[VGroup(glyph(SHAPES[m], c, size=0.15),
                              Text(n, font=FONT, font_size=21, color=INK))
                       .arrange(RIGHT, buff=0.16)
                       for m, (n, c) in METHODS.items()]) \
            .arrange(DOWN, aligned_edge=LEFT, buff=0.19).to_edge(RIGHT, buff=0.55).shift(UP * 0.6)
        self.add(key)

        counter = always_redraw(lambda: VGroup(
            Text("bound", font=FONT, font_size=22, color=MUTED),
            Text(f"{lvl.get_value():.0f}%", font=FONT, font_size=54, color=INK),
            Text("of the reference", font=FONT, font_size=18, color=MUTED),
        ).arrange(DOWN, buff=0.07).to_edge(RIGHT, buff=0.55).shift(DOWN * 2.05))
        self.add(counter)

        cap = Caption("Fifty randomised scenarios. The bound tightens; every planner pays.")
        self.add(cap)
        self.wait(1.4)

        self.play(lvl.animate.set_value(50), run_time=8.0, rate_func=linear)
        cap.set_text("At half the reference bound we solve 80% against 54% "
                     "for the strongest baseline.")
        o, f = base["hexspline_cl@50"], base["formation@50"]
        hi = VGroup(
            Text(f"Ours  {100*o['success_rate']:.0f}%  at {o['mean_ratio']:.2f}×",
                 font=FONT, font_size=23, color=METHODS['hexspline_cl'][1]),
            Text(f"Formation  {100*f['success_rate']:.0f}%  at {f['mean_ratio']:.2f}×",
                 font=FONT, font_size=23, color=METHODS['formation'][1]),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12).to_corner(UL, buff=0.4)
        self.play(FadeIn(hi), run_time=0.6)
        self.wait(2.6)
        self.play(FadeOut(hi), run_time=0.4)

        cap.set_text("Greedy's short paths are an average over two scenarios: "
                     "hollow marks fewer than five.")
        self.play(lvl.animate.set_value(30), run_time=6.0, rate_func=linear)
        seq = base["sequential@30"]
        note = Text(f"Sequential runs off the axis at {seq['mean_ratio']:.2f}×",
                    font=FONT, font_size=22, color=METHODS['sequential'][1])
        note.to_corner(UL, buff=0.4)
        self.play(FadeIn(note), run_time=0.5)
        cap.set_text(f"Median wall clock: {d['baseline']['hexspline_cl@50']['median_wall_s']:.0f} s "
                     f"against {d['baseline']['formation@50']['median_wall_s']:.0f} s.")
        self.wait(2.6)
