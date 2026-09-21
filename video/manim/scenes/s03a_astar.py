"""Scene 3a (part of the method block) - the joint search, replayed as a wavefront.

Design chosen with the user after the reference GIF (fig_astar_partial.gif)
turned out too jittery to read: that GIF draws the current candidate's whole
path every frame in raw pop order, which jumps between very different
branches because best-first search has no spatial order. Here the SAME real
pops are replayed sorted by the primary's cumulative distance -- a real
quantity, monotonically growing -- so the frontier sweeps outward smoothly.
Nothing is invented; only the playback order changes, and the caption says so.

Replayed from video/data/fig1/astar_trace.csv (trace_astar, pure logging,
default off; a run with it on is byte-identical to one with it off). On this
run: 1748 pops matching results.yaml's astar_iterations, 2702 rejected
expansions, split 1517/1076 between the two agents (fixed: the trace now
records WHICH agent triggered each obstacle rejection, since the two are not
always at the same place).
"""
import sys
from pathlib import Path
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import load_scene
from consort.world import World
from consort.mapview import base_map
from consort.mobjects import Caption
from consort.astarreplay import WavefrontReplay, load_trace, reason_counts
from consort.palette import PRIMARY, SUPPORT, INK, MUTED, BAD, FONT

REASON_TEXT = {
    "obstacle": "its belief would clip an obstacle",
    "goal_spline": "reached the goal, but the spline breaches an obstacle",
    "dominated": "same cell, no shorter and no more certain",
    "primary_unc": "primary would exceed the bound",
    "support_unc": "support would exceed the bound",
    "comm_radius": "support out of comm range",
    "support_dist": "support would outrun the primary",
    "pop_unc": "already over the bound when popped",
}


class AStar(Scene):
    def construct(self):
        sc = load_scene("fig1/scene.json")
        nodes = {int(i): (float(x), float(y)) for i, x, y in sc.raw["nodes"]}
        world = World.from_scene(sc, width=10.2, height=4.5, center=UP * 0.55)

        base = base_map(world, sc)
        self.add(base)
        cap = Caption("The search runs over the joint state of both agents at "
                      "once, replayed here ordered by distance from the start.")
        self.add(cap)

        wf = WavefrontReplay("../data/fig1/astar_trace.csv", nodes,
                            [SUPPORT, PRIMARY], world)
        g = ValueTracker(0.0)
        self.add(wf.mobject(g), wf.prune_flash_mobject(g))

        counter = wf.counter_mobject(g, INK, MUTED, corner=UR, buff=0.35)
        key = VGroup(
            Text("primary frontier", font=FONT, font_size=20, color=PRIMARY),
            Text("support frontier", font=FONT, font_size=20, color=SUPPORT),
            Text("✗ rejected expansion", font=FONT, font_size=18, color=BAD),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.10).to_corner(UL, buff=0.35)
        self.add(counter, key)

        self.play(g.animate.set_value(wf.max_g * 0.5), run_time=3.4, rate_func=linear)
        cap.set_text("Expanding both agents together is what lets it decide "
                     "which landmark to use and when to relay.")
        self.play(g.animate.set_value(wf.max_g), run_time=3.6, rate_func=linear)

        # the dominant rejection reason, called out with the CORRECT agent
        pops, pushes, prunes = load_trace("../data/fig1/astar_trace.csv")
        counts = reason_counts(prunes)
        top = max(counts, key=counts.get)
        by_agent = {}
        for _, _, _, _, ns, reason in prunes:
            if not reason.startswith(top):
                continue
            a = int(reason.split(":")[1]) - 1 if ":" in reason else 0
            by_agent.setdefault(a, 0)
            by_agent[a] += 1
        ex = next(p for p in reversed(prunes) if p[5].startswith(top))
        a_idx = int(ex[5].split(":")[1]) - 1 if ":" in ex[5] else 0
        pt = nodes.get(ex[4][a_idx])
        agent_color = SUPPORT if a_idx == 0 else PRIMARY
        if pt:
            mark = VGroup(
                Cross(Square(side_length=0.22), stroke_color=BAD, stroke_width=4)
                .move_to(world.pt(*pt)),
                Text(f"{'support' if a_idx == 0 else 'primary'}: "
                     f"{REASON_TEXT.get(top, top)}",
                     font=FONT, font_size=19, color=agent_color),
            )
            mark[1].next_to(mark[0], UP, buff=0.12)
            self.play(FadeIn(mark), run_time=0.5)
            cap.set_text(f"{sum(counts.values())} expansions were rejected outright: "
                         f"{counts[top]} because {REASON_TEXT.get(top, top)}.")
            self.wait(2.0)
            self.play(FadeOut(mark), run_time=0.4)

        cap.set_text(f"The first feasible goal state it pops is the seed: "
                     f"{sc.result('primary_disc_unc', 0):.2f} m at the goal.")
        self.wait(2.0)
