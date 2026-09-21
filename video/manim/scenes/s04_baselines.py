"""Scene 4 (1:15-1:45) - the baselines, actually searching.

Each panel replays that planner's own search, from its own trace (all
println-only, default-off taps in the four planner files, no-op proven the
same way as the joint search's), then settles into the ++/-- verdict. Four
different visual grammars because the four algorithms are structurally
different, not four windows onto the same thing:

  Greedy      the primary's small single-agent A* (wavefront_replay), then the
              support's per-step, no-lookahead choice (short local "feelers"
              from its current position, not a growing path -- that IS the
              algorithm: a fresh one-hex decision every step, never a frontier)
  Formation   its own weighted A* for the primary (wavefront_replay again --
              structurally the same search, only the acceptance test differs)
  Sequential  leg 1's small A* (parked helper, wavefront_replay), then the
              helper's layered beam search (dots surviving at each layer,
              already thinned by dominance -- no reordering needed, unlike a
              priority queue). Leg 4's re-plan was tried and NOT taken on this
              scenario (sequential_replanned: false), so it is named but not
              re-animated -- showing an unused 10,000-pop search would misstate
              what the shipped path is descended from.
  CL-GBT      branches in push order -- already spatially coherent (RRT-style
              growth always extends an existing nearby node), so this is the
              one search that needed no reordering trick at all.

Scenario s049 at the 50% level, all five re-run with geometry: every length
here is byte-identical to constraint_sweep/baseline_2026-08-17/trials_all.csv.
"""
import csv
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mobjects import Caption, polyline, obstacle_poly
from consort.astarreplay import WavefrontReplay
from consort.searchviz import (
    load_greedy_trace, greedy_feelers_mobject, load_layered_beam, layered_beam_mobject,
    winning_path_mobject, load_clgbt_tree, clgbt_tree_mobject,
)
from consort.palette import METHODS, PRIMARY, SUPPORT, GOAL, LANDMARK, INK, MUTED, OK, BAD, FONT

SID = "s049_p050"
D = str(Path(__file__).resolve().parents[2] / "data" / "baselines" / SID) + "/"


def small_world(s, center):
    return World.from_scene(s, width=5.7, height=2.3, center=center)


class Baselines(Scene):
    def construct(self):
        scenes = {m: load_scene(f"baselines/{SID}/{m}.json")
                  for m in ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]}
        ours = scenes["hexspline_cl"]
        bound = ours.bound
        nodes = {name: {int(i): (float(x), float(y)) for i, x, y in scenes[name].raw["nodes"]}
                 for name in ("greedy", "formation")}

        cap = Caption("Each baseline runs its own search. Same scenario, same bound.")
        self.add(cap)

        slots = {"greedy": LEFT * 3.4 + UP * 1.55, "formation": RIGHT * 3.4 + UP * 1.55,
                 "sequential": LEFT * 3.4 + DOWN * 1.45, "clgbt": RIGHT * 3.4 + DOWN * 1.45}
        titles = VGroup()
        panel_static = {}
        for m, c in slots.items():
            s = scenes[m]
            w = small_world(s, c)
            g = VGroup(*[obstacle_poly(w, o) for o in s.obstacles])
            for lm in s.landmarks:
                g.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                      .set(width=0.13).move_to(w.pt(lm["x"], lm["y"])))
            g.add(Star(n=5, outer_radius=0.10, color=GOAL, fill_opacity=1,
                       stroke_width=0).move_to(w.pt(*s.goal)))
            g.add(Dot(w.pt(*s.start), radius=0.045, color=INK))
            self.add(g)
            panel_static[m] = w
            head = Text(METHODS[m][0], font=FONT, font_size=23, color=METHODS[m][1])
            head.next_to(g, UP, buff=0.06)
            self.add(head)
            titles.add(head)

        # ---------------- Greedy: small A* + per-step feelers -------------
        cap.set_text("Greedy: the primary plans alone, then the support picks "
                     "the best one-step move, no lookahead.")
        w = panel_static["greedy"]
        wf_g = WavefrontReplay(D + "greedy_astar_trace.csv", nodes["greedy"],
                               [SUPPORT, PRIMARY], w, keep=6)
        gtick = ValueTracker(0.0)
        self.add(wf_g.mobject(gtick))
        self.play(gtick.animate.set_value(wf_g.max_g), run_time=1.4, rate_func=linear)

        greedy_rows = load_greedy_trace(D + "greedy_trace.csv")
        primary_path_nodes = None
        with open(D + "greedy_astar_trace.csv") as f:
            for r in csv.DictReader(f):
                if r["ev"] == "goal":
                    primary_path_nodes = None  # path not needed directly
        # support position by step: derive by walking the chosen combos
        support_pos = {2: (0.0, 0.0)}
        step = 2
        for r in sorted(greedy_rows, key=lambda r: r["step"]):
            if r["chosen"]:
                node = r["combo"][0]
                support_pos[r["step"] + 1] = nodes["greedy"].get(node, support_pos[r["step"]])
        step_tracker = ValueTracker(2.0)
        self.add(greedy_feelers_mobject(greedy_rows, nodes["greedy"], support_pos, w,
                                        step_tracker, OK, BAD))
        max_step = max(r["step"] for r in greedy_rows)
        self.play(step_tracker.animate.set_value(max_step), run_time=1.6, rate_func=linear)
        gv = scenes["greedy"]
        ok = gv.result("primary_unc", 1e9) <= bound + 1e-9
        self.add(Text("✗ support never reaches a fix" if not ok else "✓",
                      font=FONT, font_size=15, color=BAD if not ok else OK)
                 .next_to(w.pt(gv.goal[0], gv.goal[1]), DOWN, buff=0.30))
        self.wait(0.5)

        # ---------------- Formation: its own A* --------------------------
        cap.set_text("Formation: its own weighted A*, but a goal candidate is "
                     "only accepted once its supports are attached.")
        w = panel_static["formation"]
        wf_f = WavefrontReplay(D + "formation_astar_trace.csv", nodes["formation"],
                               [PRIMARY], w, keep=8)
        ftick = ValueTracker(0.0)
        self.add(wf_f.mobject(ftick))
        self.play(ftick.animate.set_value(wf_f.max_g), run_time=1.8, rate_func=linear)
        fv = scenes["formation"]
        okf = fv.result("primary_unc", 1e9) <= bound + 1e-9
        self.add(Text(f"✓ σ = {fv.result('primary_unc'):.2f} m" if okf else "✗",
                      font=FONT, font_size=15, color=OK if okf else BAD)
                 .next_to(w.pt(fv.goal[0], fv.goal[1]), DOWN, buff=0.30))
        self.wait(0.5)

        # ---------------- Sequential: leg1 + helper layered beam ----------
        cap.set_text("Sequential: the primary plans alone first, then the "
                     "helper searches its whole route against it.")
        w = panel_static["sequential"]
        seq_nodes = {int(i): (float(x), float(y)) for i, x, y in scenes["sequential"].raw["nodes"]}
        wf_s1 = WavefrontReplay(D + "sequential_astar_trace_leg1.csv", seq_nodes,
                                [PRIMARY], w, keep=6)
        s1tick = ValueTracker(0.0)
        self.add(wf_s1.mobject(s1tick))
        self.play(s1tick.animate.set_value(wf_s1.max_g), run_time=1.2, rate_func=linear)

        layers = load_layered_beam(D + "sequential_helper_trace.csv", "helper1")
        ltick = ValueTracker(1.0)
        self.add(layered_beam_mobject(layers, seq_nodes, w, ltick, SUPPORT))
        self.play(ltick.animate.set_value(max(layers)), run_time=1.8, rate_func=linear)
        self.add(winning_path_mobject(layers, seq_nodes, w, SUPPORT))
        sv = scenes["sequential"]
        oks = sv.result("primary_unc", 1e9) <= bound + 1e-9
        self.add(Text(f"✓ σ = {sv.result('primary_unc'):.2f} m" if oks else "✗",
                      font=FONT, font_size=15, color=OK if oks else BAD)
                 .next_to(w.pt(sv.goal[0], sv.goal[1]), DOWN, buff=0.30))
        self.wait(0.5)

        # ---------------- CL-GBT: tree growth ------------------------------
        cap.set_text("CL-GBT: a random tree grows in the joint space and "
                     "returns the first path it finds.")
        w = panel_static["clgbt"]
        tree = load_clgbt_tree(D + "clgbt_tree_nodes.csv")
        n_tracker = ValueTracker(0.0)
        self.add(clgbt_tree_mobject(tree, w, n_tracker, [SUPPORT, PRIMARY], thin=30))
        self.play(n_tracker.animate.set_value(max(tree)), run_time=2.2, rate_func=linear)
        cv = scenes["clgbt"]
        okc = cv.result("primary_unc", 1e9) <= bound + 1e-9
        self.add(Text(f"✓ but {cv.result('primary_length'):.0f} m" if okc else "✗",
                      font=FONT, font_size=15, color=OK if okc else BAD)
                 .next_to(w.pt(cv.goal[0], cv.goal[1]), DOWN, buff=0.30))
        self.wait(0.6)

        # ---------------- all five primaries overlaid ----------------------
        self.play(*[FadeOut(m) for m in self.mobjects if m not in (cap,)], run_time=0.6)
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
        for m in ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]:
            s = scenes[m]
            ok_m = s.result("primary_unc", 1e9) <= bound + 1e-9
            ln = polyline(w, s.primary.draw_x, s.primary.draw_y,
                          stroke_color=METHODS[m][1],
                          stroke_width=4.0 if m == "hexspline_cl" else 2.4)
            self.play(Create(ln), run_time=0.5)
            rows.add(VGroup(
                Text(METHODS[m][0], font=FONT, font_size=20, color=METHODS[m][1]),
                Text(f"{s.result('primary_length'):.0f} m", font=FONT, font_size=20, color=INK),
                Text("✓" if ok_m else "✗", font=FONT, font_size=20, color=OK if ok_m else BAD),
            ).arrange(RIGHT, buff=0.30))
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.13).to_corner(UL, buff=0.32)
        self.play(FadeIn(rows), run_time=0.6)

        note = Text(f"one scenario at ū = {bound:.2f} m; the aggregate comes next",
                    font=FONT, font_size=19, color=MUTED)
        note.next_to(cap, UP, buff=0.22)
        self.play(FadeIn(note), run_time=0.5)
        self.wait(2.2)
