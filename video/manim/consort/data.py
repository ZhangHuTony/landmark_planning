"""Loaders for video/data/*.json produced by video/julia/export_scene.jl.

Every array here came out of the planner. The only arithmetic this module does
is interpolation *along* a logged track, so a ValueTracker can scrub it.
"""
import json
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
DATA = REPO / "video" / "data"


@dataclass
class Track:
    """One agent's path plus the covariance the planner scored along it."""
    label: str
    primary: bool
    ctrls: np.ndarray            # (K,2) B-spline control points
    draw_x: np.ndarray           # dense samples, for drawing the curve
    draw_y: np.ndarray
    draw_arc: np.ndarray
    eval_x: np.ndarray           # the samples eval_continuous scored
    eval_y: np.ndarray
    eval_arc: np.ndarray
    cov: np.ndarray              # (N,3) = [c11, c12, c22], symmetric
    unc: np.ndarray              # (N,) det(Sigma)^(1/4), the planner's own metric
    basis: str = "waypoints"

    @property
    def arc_end(self):
        return float(self.draw_arc[-1])

    def _eval_s(self, arc):
        """Map an arc on the drawn curve onto the eval parameterisation.

        When a run was certified with cont_unc_use_waypoints the eval samples are
        the CONTROL POLYGON, whose total length differs from the spline's, so the
        two are matched by fraction of the way along rather than by metres.
        """
        if self.basis == "controls":
            frac = np.clip(arc / max(self.arc_end, 1e-9), 0.0, 1.0)
            return frac * self.eval_arc[-1]
        return np.clip(arc, self.eval_arc[0], self.eval_arc[-1])

    def pos_at(self, arc):
        a = np.clip(arc, 0.0, self.arc_end)
        return (float(np.interp(a, self.draw_arc, self.draw_x)),
                float(np.interp(a, self.draw_arc, self.draw_y)))

    def heading_at(self, arc, d=8.0):
        x0, y0 = self.pos_at(max(arc - d, 0.0))
        x1, y1 = self.pos_at(min(arc + d, self.arc_end))
        return float(np.arctan2(y1 - y0, x1 - x0))

    def unc_at(self, arc):
        return float(np.interp(self._eval_s(arc), self.eval_arc, self.unc))

    def cov_at(self, arc):
        """Sigma at this arc, held at the last scored sample (no smoothing)."""
        s = self._eval_s(arc)
        i = int(np.searchsorted(self.eval_arc, s, side="right") - 1)
        i = max(0, min(i, len(self.eval_arc) - 1))
        c11, c12, c22 = self.cov[i]
        return np.array([[c11, c12], [c12, c22]])

    def upto(self, arc):
        """Dense points from the start to `arc`, for a growing trail."""
        a = np.clip(arc, 0.0, self.arc_end)
        k = int(np.searchsorted(self.draw_arc, a, side="right"))
        xs = np.concatenate([self.draw_x[:k], [self.pos_at(a)[0]]])
        ys = np.concatenate([self.draw_y[:k], [self.pos_at(a)[1]]])
        return xs, ys


@dataclass
class Scene:
    raw: dict
    agents: list = field(default_factory=list)
    obstacles: list = field(default_factory=list)
    landmarks: list = field(default_factory=list)
    alone: Track = None

    @property
    def bound(self):
        return float(self.raw["cfg"]["unc_radius_threshold"])

    @property
    def primary(self):
        return self.agents[-1]

    @property
    def supports(self):
        return self.agents[:-1]

    @property
    def start(self):
        return np.asarray(self.raw["start"], float)

    @property
    def goal(self):
        return np.asarray(self.raw["goal"], float)

    @property
    def hex_centers(self):
        return np.asarray(self.raw["hex"]["centers"], float)

    @property
    def hex_radius(self):
        return float(self.raw["hex"]["radius"])

    @property
    def comm(self):
        return self.raw["comm"]

    @property
    def checkpoints(self):
        return self.raw["checkpoints"]

    @property
    def landmark_events(self):
        return self.raw["landmark_events"]

    def result(self, key, default=None):
        v = self.raw["results"].get(key, default)
        return default if v is None else v


def _track(a):
    return Track(
        label=a["label"], primary=bool(a["primary"]),
        ctrls=np.asarray(a["ctrls"], float),
        draw_x=np.asarray(a["draw"]["x"], float),
        draw_y=np.asarray(a["draw"]["y"], float),
        draw_arc=np.asarray(a["draw"]["arc"], float),
        eval_x=np.asarray(a["eval"]["x"], float),
        eval_y=np.asarray(a["eval"]["y"], float),
        eval_arc=np.asarray(a["eval"]["arc"], float),
        cov=np.asarray(a["eval"]["cov"], float),
        unc=np.asarray(a["eval"]["unc"], float),
        basis=a["eval"].get("basis", "waypoints"),
    )


def load_scene(path):
    p = Path(path)
    if not p.is_absolute():
        p = DATA / p
    raw = json.loads(p.read_text())
    sc = Scene(raw=raw)
    sc.agents = [_track(a) for a in raw["agents"]]
    sc.obstacles = [np.asarray(o, float) for o in raw["obstacles"]]
    sc.landmarks = raw["landmarks"]
    if raw.get("alone"):
        al = raw["alone"]
        x = np.asarray(al["x"], float); y = np.asarray(al["y"], float)
        arc = np.asarray(al["arc"], float)
        sc.alone = Track(label="primary alone", primary=True, ctrls=sc.agents[-1].ctrls,
                         draw_x=x, draw_y=y, draw_arc=arc,
                         eval_x=x, eval_y=y, eval_arc=arc,
                         cov=np.asarray(al["cov"], float),
                         unc=np.asarray(al["unc"], float))
    return sc
