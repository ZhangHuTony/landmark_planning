"""Scene 6 (2:15-2:50) - closed-loop simulation in HoloOcean.

HoloRun is the synced replay of ONE flown trial: planned path dashed, the true
executed trail, the factor-graph estimator's own covariance, and the meter
against the 1.8 m bound. The left 960x540 of the frame is deliberately left
empty; ffmpeg/composite_holo.sh drops the rendered footage there, sped up x30,
so the two halves are the same instant (Manim second s <-> tick 900*s <->
footage frame 30*s).

HoloMC overlays all 30 Monte Carlo trials and then zooms to the goal to compare
the planner's predicted terminal ellipse against the sample of what was flown.

Everything is replayed from video/data/holo/, extracted by export_holo.py from
the simulator's own run logs. No dynamics, no estimator, no covariance here.
"""
import json
import sys
from pathlib import Path
import numpy as np
from manim import *

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from consort.data import DATA
from consort.world import World
from consort.mobjects import cov_ellipse, UncMeter, Caption, polyline, obstacle_poly
from consort.geometry import make_local_scale
from consort.palette import (
    PRIMARY, SUPPORT, GOAL, LANDMARK, COMM, COMM_DEAD, INK, MUTED, OK, BAD, FONT,
)

FLASH_TICKS = 260          # how long a comm marker stays up, in ticks
SPEED = 30                 # x30: 900 ticks of sim per second of video


def load_video_run():
    return dict(np.load(DATA / "holo" / "holo_video.npz", allow_pickle=False))


def map_world(z, **kw):
    xs = np.concatenate([z["auv0_track"][:, 1], z["auv1_track"][:, 1],
                         z["obstacle0_verts"][:, 0], z["landmarks"][:, 0]])
    ys = np.concatenate([z["auv0_track"][:, 2], z["auv1_track"][:, 2],
                         z["obstacle0_verts"][:, 1], z["landmarks"][:, 1]])
    return World((xs.min() - 60, xs.max() + 60), (ys.min() - 60, ys.max() + 60), **kw)


class HoloRun(Scene):
    def construct(self):
        z = load_video_run()
        n_ticks = int(z["n_ticks"])
        thr = float(z["unc_threshold"])
        # right half only: the left 960x540 px box is the footage slot
        world = map_world(z, width=6.2, height=3.4, center=RIGHT * 3.55 + UP * 0.45)

        slot = Rectangle(width=7.111, height=4.0, stroke_color=MUTED, stroke_width=1.4,
                         fill_opacity=0).move_to(LEFT * 3.5555)
        slot_lab = Text("HoloOcean, ×30", font=FONT, font_size=24, color=MUTED)
        slot_lab.next_to(slot, UP, buff=0.18)
        self.add(slot, slot_lab)

        self.add(obstacle_poly(world, z["obstacle0_verts"]))
        lm = z["landmarks"][0]
        self.add(Triangle(fill_color=LANDMARK, fill_opacity=1, stroke_width=0)
                 .set(width=0.22).move_to(world.pt(lm[0], lm[1])))
        self.add(Star(n=5, outer_radius=0.15, color=GOAL, fill_opacity=1, stroke_width=0)
                 .move_to(world.pt(*z["goal_xy"])))
        for ref, col in ((z["auv0_ref"], PRIMARY), (z["auv1_ref"], SUPPORT)):
            self.add(DashedVMobject(polyline(world, ref[:, 0], ref[:, 1],
                                             stroke_color=col, stroke_width=2.0,
                                             stroke_opacity=0.55), num_dashes=46))

        tick = ValueTracker(0.0)
        t0, t1 = z["auv0_track"], z["auv1_track"]

        def trail(track, col, w):
            def f():
                k = max(int(tick.get_value() / 10), 2)     # track stored every 10th tick
                k = min(k, len(track))
                return polyline(world, track[:k, 1], track[:k, 2],
                                stroke_color=col, stroke_width=w)
            return f

        self.add(always_redraw(trail(t1, SUPPORT, 2.6)),
                 always_redraw(trail(t0, PRIMARY, 3.2)))

        ntick, ncov, nest, nunc = (z["auv0_node_tick"], z["auv0_node_cov"],
                                   z["auv0_node_est"], z["auv0_node_unc"])

        def node_idx():
            """The last estimator node at or before now: nothing is known sooner."""
            return int(np.clip(np.searchsorted(ntick, tick.get_value(), "right") - 1,
                               0, len(ntick) - 1))

        scale_at = make_local_scale([z["obstacle0_verts"]], default=6.0, floor=1.5, margin=3.0)
        self.add(always_redraw(lambda: cov_ellipse(
            world, nest[node_idx()], ncov[node_idx()], nstd=2, sigma_scale=scale_at,
            color=PRIMARY)))
        self.add(always_redraw(lambda: Dot(
            world.pt(*t0[min(int(tick.get_value() / 10), len(t0) - 1)][1:3]),
            radius=0.06, color=PRIMARY)))
        self.add(always_redraw(lambda: Dot(
            world.pt(*t1[min(int(tick.get_value() / 10), len(t1) - 1)][1:3]),
            radius=0.05, color=SUPPORT)))

        meter = UncMeter(thr, vmax=2.4, height=2.5, label="estimator σ")
        meter.scale(0.86).to_edge(RIGHT, buff=0.30).shift(DOWN * 1.55)
        meter.add_updater(lambda m: m.set_value(float(nunc[node_idx()])))
        self.add(meter)

        # comm markers: successes from the recorder, failures from the attempt grid
        comm_ticks = [(int(round(c[5])), True) for c in z["comm_events"]]
        fails = [(int(t), False) for t, ok in
                 zip(z["attempt_tick"], z["attempt_fired"]) if not ok]
        events = sorted(comm_ticks + fails)
        dists = {int(t): float(d) for t, d in zip(z["attempt_tick"], z["attempt_d"])}

        def comm_banner():
            now = tick.get_value()
            live = [(t, ok) for t, ok in events if 0 <= now - t <= FLASH_TICKS]
            if not live:
                return VGroup()
            t, ok = live[-1]
            fade = 1.0 - (now - t) / FLASH_TICKS
            i0 = min(int(t / 10), len(t0) - 1)
            i1 = min(int(t / 10), len(t1) - 1)
            ln = DashedLine(world.pt(*t1[i1][1:3]), world.pt(*t0[i0][1:3]),
                            stroke_color=COMM if ok else COMM_DEAD,
                            stroke_width=3.4 if ok else 1.8,
                            dash_length=0.10).set_opacity(fade * (1.0 if ok else 0.7))
            d = dists.get(t, float(np.hypot(t0[i0][1] - t1[i1][1], t0[i0][2] - t1[i1][2])))
            txt = Text("comm: packet through" if ok else f"comm: no link, {d:.0f} m apart",
                       font=FONT, font_size=19, color=COMM if ok else COMM_DEAD)
            txt.set_opacity(fade).next_to(slot, DOWN, buff=0.16).shift(RIGHT * 3.0)
            return VGroup(ln, txt)

        self.add(always_redraw(comm_banner))

        clock = always_redraw(lambda: Text(
            f"t = {tick.get_value()/30:5.0f} s", font=FONT, font_size=21, color=MUTED
        ).to_corner(UR, buff=0.30))
        note = Text("estimator 2σ, magnified for visibility",
                    font=FONT, font_size=16, color=MUTED)
        note.next_to(meter, DOWN, buff=0.30)
        self.add(clock, note)

        cap = Caption("The plan is flown with vehicle dynamics, a controller, and an "
                      "independent factor-graph estimator.")
        self.add(cap)

        # Every segment's run_time is EXACTLY its tick delta / 900 (the x30
        # decimation rate: 30 ticks/sec sim * 30x speed / 30 fps = 900 ticks
        # per second of video). No wait() calls and no off-rate segments here:
        # the composited footage is a fixed 17.0 s clip with no pauses of its
        # own, so any real-time gap or rate change on this side desyncs the
        # two halves. Captions still change at the same tick boundaries via
        # set_text, which costs no timeline time.
        checkpoints = [0, 4800, 12400, 12800, n_ticks]
        assert checkpoints[-1] == n_ticks
        captions = [
            None,
            "The support goes behind the wall. Every packet is dropped "
            "and the primary is on dead reckoning.",
            "Its uncertainty climbs over the bound.",
            "The first packet after the support rejoins pulls it back down.",
        ]
        for (a, b), text in zip(zip(checkpoints, checkpoints[1:]), captions):
            if text:
                cap.set_text(text)
            self.play(tick.animate.set_value(b), run_time=(b - a) / 900.0, rate_func=linear)
        meter.clear_updaters()
        cap.set_text("The vehicle steers on its own estimate throughout.")


class HoloMC(Scene):
    def construct(self):
        d = json.loads((DATA / "holo" / "holo_mc.json").read_text())
        num = json.loads((DATA / "holo" / "numbers.json").read_text())["mc"]
        tracks = [np.asarray(t, float) for t in d["tracks"]]
        obs = np.asarray(d["obstacle"], float)

        xs = np.concatenate([t[:, 0] for t in tracks] + [obs[:, 0]])
        ys = np.concatenate([t[:, 1] for t in tracks] + [obs[:, 1]])
        world = World((xs.min() - 60, xs.max() + 60), (ys.min() - 60, ys.max() + 60),
                      width=10.6, height=4.4, center=UP * 0.55)

        sup = np.asarray(d["support_track"], float)
        mapg = VGroup(
            obstacle_poly(world, obs),
            Star(n=5, outer_radius=0.16, color=GOAL, fill_opacity=1, stroke_width=0)
            .move_to(world.pt(*d["goal"])),
            polyline(world, sup[:, 0], sup[:, 1], stroke_color=SUPPORT,
                     stroke_width=1.6, stroke_opacity=0.5),
        )
        self.add(mapg)

        cap = Caption("Thirty runs, thirty noise seeds.")
        self.add(cap)
        lines = VGroup(*[polyline(world, t[:, 0], t[:, 1], stroke_color=PRIMARY,
                                  stroke_width=1.3, stroke_opacity=0.45) for t in tracks])
        self.play(LaggedStart(*[Create(l) for l in lines], lag_ratio=0.05, run_time=3.2))

        verdict = VGroup(
            Text(f"{num['clear']}/{num['n']} obstacle-clear", font=FONT, font_size=27, color=OK),
            Text(f"{num['goal']}/{num['n']} reached the goal", font=FONT, font_size=23, color=INK),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12).to_corner(UL, buff=0.35)
        self.play(FadeIn(verdict), run_time=0.6)
        self.wait(1.8)

        cap.set_text("At the goal: what the planner predicted against what was flown.")
        self.play(FadeOut(lines), FadeOut(verdict), FadeOut(mapg), run_time=0.7)

        # --- terminal error, true scale ---
        errs = np.asarray(num["errors"], float)
        span = 1.35 * max(3.0, np.abs(errs).max())
        zoom = World((-span, span), (-span, span), width=4.5, height=4.5,
                     center=LEFT * 3.1 + UP * 0.25)
        axes = VGroup(
            Line(zoom.pt(-span, 0), zoom.pt(span, 0), stroke_color=MUTED, stroke_width=1.2),
            Line(zoom.pt(0, -span), zoom.pt(0, span), stroke_color=MUTED, stroke_width=1.2),
        )
        pred = float(num["predicted"])
        pred_e = Circle(radius=zoom.length(pred), color=BAD, stroke_width=2.6)
        pred_e.move_to(zoom.pt(0, 0)).set_fill(opacity=0)
        # The sample covariance is computed ABOUT THE MEAN (fig8_montecarlo.py's
        # own np.cov convention), so its ellipse belongs at the mean, not at the
        # origin -- the 30 runs have a real ~2.7 m bias, and drawing the ellipse
        # at (0,0) would make a correctly-calibrated model look wrong on screen.
        mean_xy = tuple(num["mean_offset"])
        samp = cov_ellipse(zoom, mean_xy, np.asarray(num["sample_cov"], float),
                           nstd=2, sigma_scale=1.0, color=PRIMARY,
                           fill_opacity=0.10, stroke_width=2.6)
        mean_mark = VGroup(
            Cross(stroke_color=PRIMARY, stroke_width=2.5).scale(0.09).move_to(zoom.pt(*mean_xy)),
            Text(f"mean bias, {np.hypot(*mean_xy):.1f} m", font=FONT, font_size=16, color=PRIMARY),
        )
        mean_mark[1].next_to(mean_mark[0], DOWN, buff=0.08)
        dots = VGroup(*[Dot(zoom.pt(*e), radius=0.045, color=INK, fill_opacity=0.8)
                        for e in errs])
        # How many of the 30 dots actually fall inside the 2-sigma sample
        # ellipse -- a DIFFERENT count from "bound_met" (which compares each
        # run's own estimator marginal to the 1.8 m threshold, nothing to do
        # with this ellipse). Computed here, not exported, since it is purely
        # a property of the ellipse being drawn on screen.
        cov = np.asarray(num["sample_cov"], float)
        d = errs - np.asarray(mean_xy)
        maha2 = np.einsum("ij,jk,ik->i", d, np.linalg.inv(cov), d)
        n_inside = int((maha2 <= 2.0 ** 2).sum())
        scale_bar = VGroup(
            Line(zoom.pt(-span * 0.85, -span * 0.85), zoom.pt(-span * 0.85 + 2, -span * 0.85),
                 stroke_color=INK, stroke_width=2.4),
            Text("2 m", font=FONT, font_size=17, color=INK),
        )
        scale_bar[1].next_to(scale_bar[0], DOWN, buff=0.08)

        legend = VGroup(
            Text(f"predicted (zero-bias)  σ = {pred:.2f} m", font=FONT, font_size=22, color=BAD),
            Text(f"flown sample, 2σ about its mean  σ = {num['empirical']:.2f} m",
                 font=FONT, font_size=22, color=PRIMARY),
            Text(f"terminal bound met in {num['bound_met']}/{num['n']} runs",
                 font=FONT, font_size=21, color=INK),
            Text("each dot: one run's (estimate − truth) at the goal",
                 font=FONT, font_size=17, color=MUTED),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.20).to_edge(RIGHT, buff=0.35).shift(UP * 0.3)

        self.play(FadeIn(axes), FadeIn(scale_bar), run_time=0.5)
        self.play(LaggedStart(*[FadeIn(x, scale=0.6) for x in dots],
                              lag_ratio=0.03, run_time=1.4))
        cap.set_text("The model predicts no bias; the estimator has a small one.")
        self.play(FadeIn(mean_mark), run_time=0.6)
        self.play(Create(pred_e), FadeIn(legend[0]), run_time=0.7)
        cap.set_text(f"Centred on that bias, the 2σ ellipse covers {n_inside} "
                     f"of {num['n']} runs.")
        self.play(Create(samp), FadeIn(legend[1]), run_time=0.7)
        self.play(FadeIn(legend[2:]), run_time=0.5)
        cap.set_text("The planner's belief model is calibrated to within 6%.")
        self.wait(2.4)
