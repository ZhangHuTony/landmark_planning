"""Scene 4 (1:15-1:45) - the baselines, actually searching, one at a time.

Each baseline gets the full frame to itself, in sequence: base map, then that
planner's OWN search (from its own trace -- all println-only, default-off
taps in the four planner files, no-op proven the same way as the joint
search's), then the search clutter fades and the clean final route (primary
and every support, in their identity colours, nothing else on top) settles
in with the verdict. Four different visual grammars because the four
algorithms are structurally different, not four windows onto the same thing:

  Greedy      the primary's small single-agent A* (wavefront_replay, in
              PRIMARY's own colour -- this search never touches the support,
              so it is drawn blue like every other primary-only search in the
              video, not purple), then the support's per-step, no-lookahead
              choice: short local "feelers" from its current position, not a
              growing path, because that IS the algorithm -- a fresh one-hex
              decision every step, never a frontier.
  Formation   its own weighted A* for the primary (wavefront_replay again --
              structurally the same search, only the acceptance test differs).
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

Colour convention, held constant across every panel and every other scene in
the video: PRIMARY (blue) is always the primary agent, SUPPORT (purple) is
always a support/helper, whatever the search's own internal bookkeeping calls
them.

Scenario s049 at the 50% level, all five re-run with geometry: every length
here is byte-identical to constraint_sweep/baseline_2026-08-17/trials_all.csv.
"""
import sys
from pathlib import Path
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


class Baselines(Scene):
    def construct(self):
        self.scenes = {m: load_scene(f"baselines/{SID}/{m}.json")
                       for m in ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]}
        self.bound = self.scenes["hexspline_cl"].bound
        self.cap = Caption("Each baseline runs its own search. Same scenario, same bound.")
        self.add(self.cap)
        self.wait(0.8)

        self.run_greedy()
        self.run_formation()
        self.run_sequential()
        self.run_clgbt()
        self.run_overlay()

    # ------------------------------------------------------------------
    def base_panel(self, method):
        """Full-frame base map for one baseline's own scenario copy, plus a
        title. Returns (world, static_group)."""
        s = self.scenes[method]
        world = World.from_scene(s, width=11.0, height=4.6, center=UP * 0.35)
        g = VGroup(*[obstacle_poly(world, o) for o in s.obstacles])
        for lm in s.landmarks:
            g.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                  .set(width=0.20).move_to(world.pt(lm["x"], lm["y"])))
        g.add(Star(n=5, outer_radius=0.16, color=GOAL, fill_opacity=1,
                   stroke_width=0).move_to(world.pt(*s.goal)))
        g.add(Dot(world.pt(*s.start), radius=0.07, color=INK))
        title = Text(METHODS[method][0], font=FONT, font_size=30, color=METHODS[method][1])
        title.to_corner(UL, buff=0.35)
        self.play(FadeIn(g), FadeIn(title), run_time=0.5)
        return world, VGroup(g, title)

    def final_path(self, method, world):
        """The clean, shipped route: primary + every support, nothing else."""
        s = self.scenes[method]
        grp = VGroup()
        for t in s.supports:
            grp.add(polyline(world, t.draw_x, t.draw_y, stroke_color=SUPPORT, stroke_width=3.0))
        grp.add(polyline(world, s.primary.draw_x, s.primary.draw_y,
                         stroke_color=PRIMARY, stroke_width=4.0))
        return grp

    def verdict_text(self, method, world, note=""):
        s = self.scenes[method]
        ok = s.result("primary_unc", 1e9) <= self.bound + 1e-9
        msg = (f"✓ {s.result('primary_length'):.0f} m, σ = {s.result('primary_unc'):.2f} m"
               if ok else f"✗ σ = {s.result('primary_unc'):.2f} m > ū")
        if note:
            msg += "  (" + note + ")"
        return Text(msg, font=FONT, font_size=22, color=OK if ok else BAD).to_corner(UR, buff=0.35)

    def clear_panel(self, *keep):
        self.play(*[FadeOut(m) for m in self.mobjects if m not in keep and m is not self.cap],
                  run_time=0.5)

    # ------------------------------------------------------------------
    def run_greedy(self):
        self.cap.set_text("Greedy: the primary plans alone, then the support "
                          "picks the best one-step move, no lookahead.")
        world, static = self.base_panel("greedy")
        nodes = {int(i): (float(x), float(y)) for i, x, y in self.scenes["greedy"].raw["nodes"]}

        wf = WavefrontReplay(D + "greedy_astar_trace.csv", nodes, [PRIMARY], world, keep=8)
        g = ValueTracker(0.0)
        wf_mob = wf.mobject(g)
        self.add(wf_mob)
        self.play(g.animate.set_value(wf.max_g), run_time=2.0, rate_func=linear)

        greedy_rows = load_greedy_trace(D + "greedy_trace.csv")
        support_pos = {2: (0.0, 0.0)}
        for r in sorted(greedy_rows, key=lambda r: r["step"]):
            if r["chosen"]:
                support_pos[r["step"] + 1] = nodes.get(r["combo"][0], support_pos[r["step"]])
        step_tracker = ValueTracker(2.0)
        feelers = greedy_feelers_mobject(greedy_rows, nodes, support_pos, world,
                                         step_tracker, OK, BAD)
        self.add(feelers)
        max_step = max(r["step"] for r in greedy_rows)
        self.play(step_tracker.animate.set_value(max_step), run_time=2.2, rate_func=linear)
        self.wait(0.4)

        wf_mob.clear_updaters(); feelers.clear_updaters()
        self.play(FadeOut(wf_mob), FadeOut(feelers), run_time=0.5)
        self.cap.set_text("The final route: the support never gets close "
                          "enough to fix the primary.")
        final = self.final_path("greedy", world)
        verdict = self.verdict_text("greedy", world)
        self.play(Create(final), run_time=1.0)
        self.play(FadeIn(verdict), run_time=0.4)
        self.wait(1.4)
        self.clear_panel()

    def run_formation(self):
        self.cap.set_text("Formation: its own weighted A*, but a goal "
                          "candidate is only accepted once its supports are attached.")
        world, static = self.base_panel("formation")
        nodes = {int(i): (float(x), float(y)) for i, x, y in self.scenes["formation"].raw["nodes"]}

        wf = WavefrontReplay(D + "formation_astar_trace.csv", nodes, [PRIMARY], world, keep=10)
        g = ValueTracker(0.0)
        wf_mob = wf.mobject(g)
        self.add(wf_mob)
        self.play(g.animate.set_value(wf.max_g), run_time=2.6, rate_func=linear)
        self.wait(0.4)

        wf_mob.clear_updaters()
        self.play(FadeOut(wf_mob), run_time=0.5)
        self.cap.set_text("The final route: the support holds a fixed slot "
                          "off the primary's heading.")
        final = self.final_path("formation", world)
        verdict = self.verdict_text("formation", world)
        self.play(Create(final), run_time=1.0)
        self.play(FadeIn(verdict), run_time=0.4)
        self.wait(1.4)
        self.clear_panel()

    def run_sequential(self):
        self.cap.set_text("Sequential: the primary plans alone first, then "
                          "the helper searches its whole route against it.")
        world, static = self.base_panel("sequential")
        nodes = {int(i): (float(x), float(y)) for i, x, y in self.scenes["sequential"].raw["nodes"]}

        wf1 = WavefrontReplay(D + "sequential_astar_trace_leg1.csv", nodes, [PRIMARY], world, keep=8)
        g1 = ValueTracker(0.0)
        wf1_mob = wf1.mobject(g1)
        self.add(wf1_mob)
        self.play(g1.animate.set_value(wf1.max_g), run_time=1.6, rate_func=linear)

        layers = load_layered_beam(D + "sequential_helper_trace.csv", "helper1")
        ltick = ValueTracker(1.0)
        beam = layered_beam_mobject(layers, nodes, world, ltick, SUPPORT)
        self.add(beam)
        self.play(ltick.animate.set_value(max(layers)), run_time=2.2, rate_func=linear)
        winner = winning_path_mobject(layers, nodes, world, SUPPORT)
        self.add(winner)
        self.wait(0.5)

        wf1_mob.clear_updaters(); beam.clear_updaters()
        self.play(FadeOut(wf1_mob), FadeOut(beam), FadeOut(winner), run_time=0.5)
        self.cap.set_text("The final route (a re-plan of the primary was "
                          "tried afterward and not taken).")
        final = self.final_path("sequential", world)
        verdict = self.verdict_text("sequential", world)
        self.play(Create(final), run_time=1.0)
        self.play(FadeIn(verdict), run_time=0.4)
        self.wait(1.4)
        self.clear_panel()

    def run_clgbt(self):
        self.cap.set_text("CL-GBT: a random tree grows in the joint space "
                          "and returns the first path it finds.")
        world, static = self.base_panel("clgbt")
        tree = load_clgbt_tree(D + "clgbt_tree_nodes.csv")
        n_tracker = ValueTracker(0.0)
        tree_mob = clgbt_tree_mobject(tree, world, n_tracker, [SUPPORT, PRIMARY], thin=25)
        self.add(tree_mob)
        self.play(n_tracker.animate.set_value(max(tree)), run_time=3.0, rate_func=linear)
        self.wait(0.4)

        tree_mob.clear_updaters()
        self.play(FadeOut(tree_mob), run_time=0.5)
        self.cap.set_text("The final route: it succeeds, but at roughly "
                          "twice the length of the others.")
        final = self.final_path("clgbt", world)
        verdict = self.verdict_text("clgbt", world)
        self.play(Create(final), run_time=1.0)
        self.play(FadeIn(verdict), run_time=0.4)
        self.wait(1.4)
        self.clear_panel()

    # ------------------------------------------------------------------
    def run_overlay(self):
        ours = self.scenes["hexspline_cl"]
        world = World.from_scene(ours, width=9.4, height=4.0, center=UP * 0.75)
        base = VGroup(*[obstacle_poly(world, o) for o in ours.obstacles])
        for lm in ours.landmarks:
            base.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                     .set(width=0.17).move_to(world.pt(lm["x"], lm["y"])))
        base.add(Dot(world.pt(*ours.start), radius=0.07, color=INK))
        base.add(Star(n=5, outer_radius=0.15, color=GOAL, fill_opacity=1,
                      stroke_width=0).move_to(world.pt(*ours.goal)))
        self.play(FadeIn(base), run_time=0.5)
        self.cap.set_text("All five primary paths, same scenario, same bound.")

        rows = VGroup()
        for m in ["hexspline_cl", "greedy", "formation", "sequential", "clgbt"]:
            s = self.scenes[m]
            ok_m = s.result("primary_unc", 1e9) <= self.bound + 1e-9
            ln = polyline(world, s.primary.draw_x, s.primary.draw_y,
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

        note = Text(f"one scenario at ū = {self.bound:.2f} m; the aggregate comes next",
                    font=FONT, font_size=19, color=MUTED)
        note.next_to(self.cap, UP, buff=0.22)
        self.play(FadeIn(note), run_time=0.5)
        self.wait(2.2)
