"""Metres -> Manim scene units, uniformly (no aspect distortion)."""
import numpy as np
from manim import ORIGIN


class World:
    def __init__(self, xlim, ylim, width=11.0, height=5.6, center=ORIGIN, pad=0.04):
        (x0, x1), (y0, y1) = xlim, ylim
        dx, dy = max(x1 - x0, 1e-9), max(y1 - y0, 1e-9)
        x0, x1 = x0 - pad * dx, x1 + pad * dx
        y0, y1 = y0 - pad * dy, y1 + pad * dy
        self.x0, self.x1, self.y0, self.y1 = x0, x1, y0, y1
        self.s = min(width / (x1 - x0), height / (y1 - y0))
        self.center = np.asarray(center, dtype=float)
        self.cx, self.cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)

    @classmethod
    def from_scene(cls, scene, margin=40.0, **kw):
        """Fit everything the scene draws: paths, obstacles, landmarks, start, goal."""
        xs, ys = [], []
        for t in scene.agents:
            xs += [t.draw_x.min(), t.draw_x.max()]
            ys += [t.draw_y.min(), t.draw_y.max()]
        for poly in scene.obstacles:
            xs += [poly[:, 0].min(), poly[:, 0].max()]
            ys += [poly[:, 1].min(), poly[:, 1].max()]
        for lm in scene.landmarks:
            xs.append(lm["x"]); ys.append(lm["y"])
        for p in (scene.start, scene.goal):
            xs.append(p[0]); ys.append(p[1])
        return cls((min(xs) - margin, max(xs) + margin),
                   (min(ys) - margin, max(ys) + margin), **kw)

    def pt(self, x, y):
        return self.center + np.array([(x - self.cx) * self.s, (y - self.cy) * self.s, 0.0])

    def pts(self, xs, ys):
        xs = np.asarray(xs, float); ys = np.asarray(ys, float)
        out = np.zeros((len(xs), 3))
        out[:, 0] = (xs - self.cx) * self.s
        out[:, 1] = (ys - self.cy) * self.s
        return out + self.center

    def length(self, metres):
        return metres * self.s
