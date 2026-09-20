"""Reusable mobjects: lattice, covariance ellipse, uncertainty meter, AUV glyph."""
import textwrap

import numpy as np
from manim import (
    VGroup, VMobject, Polygon, RegularPolygon, Ellipse, Rectangle, Line, Dot, Star,
    Triangle, Text, PI, LEFT, RIGHT, UP, DOWN, ORIGIN, WHITE,
)
from .palette import (
    HEX_FILL, HEX_EDGE, OBSTACLE, INK, MUTED, OK, BAD, LANDMARK, FONT,
)


def obstacle_poly(world, verts, **kw):
    pts = world.pts(verts[:, 0], verts[:, 1])
    kw.setdefault("fill_color", OBSTACLE)
    kw.setdefault("fill_opacity", 0.93)
    kw.setdefault("stroke_color", INK)
    kw.setdefault("stroke_width", 1.2)
    kw.setdefault("z_index", -5)   # over the lattice, under the tracks
    return Polygon(*pts, **kw)


class HexLattice(VGroup):
    """The heading-aware hex graph's route cells, drawn pointy-top.

    Matches src/graph.jl's draw_hex_tiles! (vertices at pi/6 + k*pi/3), so the
    lattice on screen is the one the search actually ran on.
    """

    def __init__(self, world, centers, radius, **kw):
        super().__init__(**kw)
        r = world.length(radius)
        for cx, cy in centers:
            h = RegularPolygon(
                n=6, radius=r, start_angle=PI / 2,
                fill_color=HEX_FILL, fill_opacity=1.0,
                stroke_color=HEX_EDGE, stroke_width=0.9,
            ).move_to(world.pt(cx, cy))
            self.add(h)
        self.set_z_index(-10)


def cov_ellipse(world, mu, cov, nstd=2.0, sigma_scale=1.0, color=INK,
                fill_opacity=0.22, stroke_width=1.0):
    """n-sigma ellipse of a 2x2 covariance, with sigma optionally inflated.

    `sigma_scale` blows up the standard deviation for visibility only (a 1.8 m
    sigma on a 1 km map is sub-pixel). Same convention as Fig. 1, which draws
    2-sigma at sigma x10; state the factor on screen whenever it is not 1.
    """
    cov = np.asarray(cov, float)
    vals, vecs = np.linalg.eigh((cov + cov.T) / 2.0)
    vals = np.clip(vals, 0.0, None)
    order = np.argsort(vals)[::-1]
    vals, vecs = vals[order], vecs[:, order]
    a, b = (nstd * sigma_scale * np.sqrt(vals))
    e = Ellipse(width=2 * world.length(a), height=2 * world.length(b),
                color=color, fill_opacity=fill_opacity, stroke_width=stroke_width)
    e.rotate(float(np.arctan2(vecs[1, 0], vecs[0, 0])))
    e.move_to(world.pt(mu[0], mu[1]))
    return e


class UncMeter(VGroup):
    """Vertical bar: the primary's sigma against the terminal bound u-bar.

    Green while the plan is feasible, red once sigma is over the bound.
    """

    def __init__(self, bound, vmax=None, height=3.0, width=0.46, label="primary σ", **kw):
        super().__init__(**kw)
        self.bound = float(bound)
        self.vmax = float(vmax if vmax is not None else 1.6 * bound)
        self.h, self.w = height, width
        self.frame = Rectangle(width=width, height=height, stroke_color=MUTED,
                               stroke_width=1.6, fill_opacity=0)
        self.bar = Rectangle(width=width, height=1e-3, stroke_width=0,
                             fill_color=OK, fill_opacity=0.95)
        self._seat(0.0)
        y = self._y(self.bound)
        self.bound_line = Line(LEFT * width * 0.78, RIGHT * width * 0.78,
                               stroke_color=BAD, stroke_width=2.2)
        self.bound_line.move_to(self.frame.get_bottom() + UP * y)
        self.bound_txt = Text(f"ū = {self.bound:g} m", font=FONT, font_size=19, color=BAD)
        self.bound_txt.next_to(self.bound_line, RIGHT, buff=0.12)
        self.caption = Text(label, font=FONT, font_size=20, color=INK)
        self.caption.next_to(self.frame, UP, buff=0.14)
        self.readout = Text("0.00 m", font=FONT, font_size=21, color=INK)
        self.readout.next_to(self.frame, DOWN, buff=0.14)
        self.add(self.frame, self.bar, self.bound_line, self.bound_txt,
                 self.caption, self.readout)

    def _y(self, v):
        return float(np.clip(v / self.vmax, 0, 1) * self.h)

    def _seat(self, v):
        h = max(self._y(v), 1e-3)
        self.bar.stretch_to_fit_height(h)
        self.bar.move_to(self.frame.get_bottom() + UP * h / 2)

    def set_value(self, v):
        self._seat(v)
        over = v > self.bound
        self.bar.set_fill(BAD if over else OK)
        new = Text(f"{v:.2f} m", font=FONT, font_size=21, color=BAD if over else INK)
        new.move_to(self.readout, aligned_edge=UP)
        self.readout.become(new)
        return self


class AUVGlyph(VGroup):
    """A torpedo-ish AUV: hull, sail, tail fin.

    The shipped Fig. 1 icon is a bitmap embedded in an Illustrator PDF with no
    vector source in the repo, so this is drawn from scratch. Swap in an
    SVGMobject here if a real vector icon ever appears in manim/assets/.
    """

    def __init__(self, color, size=0.34, **kw):
        super().__init__(**kw)
        L, H = size * 2.1, size * 0.62
        hull = Ellipse(width=L, height=H, fill_color=color, fill_opacity=1.0,
                       stroke_color=color, stroke_width=1.0)
        sail = Rectangle(width=L * 0.22, height=H * 0.62, fill_color=color,
                         fill_opacity=1.0, stroke_width=0)
        sail.next_to(hull.get_center(), UP, buff=0).shift(LEFT * L * 0.02 + DOWN * H * 0.10)
        fin = Triangle(fill_color=color, fill_opacity=1.0, stroke_width=0)
        fin.set(width=H * 0.85).rotate(-PI / 2)
        fin.move_to(hull.get_left() + RIGHT * L * 0.04)
        self.add(fin, hull, sail)
        self.base_angle = 0.0

    def place(self, world, x, y, heading=0.0):
        self.rotate(heading - self.base_angle, about_point=self.get_center())
        self.base_angle = heading
        self.move_to(world.pt(x, y))
        return self


class Caption(VGroup):
    """Bottom caption strip, burned in (the video ships watchable without audio).

    Wraps on word boundaries and then shrinks to fit, so a long line can never
    run off the frame the way an unwrapped Text does.
    """

    def __init__(self, text="", max_chars=62, max_width=12.4, font_size=27, **kw):
        super().__init__(**kw)
        self.max_chars, self.max_width, self.fs = max_chars, max_width, font_size
        self.txt = self._make(text)
        self.add(self.txt)

    def _make(self, text):
        lines = textwrap.wrap(text, self.max_chars) or [""]
        g = VGroup(*[Text(l, font=FONT, font_size=self.fs, color=INK) for l in lines])
        g.arrange(DOWN, buff=0.12)
        if g.width > self.max_width:
            g.scale_to_fit_width(self.max_width)
        g.to_edge(DOWN, buff=0.30)
        return g

    def set_text(self, text):
        self.txt.become(self._make(text))
        return self


class TitleCard(VGroup):
    def __init__(self, title, subtitle=None, **kw):
        super().__init__(**kw)
        t = Text(title, font=FONT, font_size=54, color=INK, weight="BOLD")
        self.add(t)
        if subtitle:
            s = Text(subtitle, font=FONT, font_size=28, color=MUTED)
            s.next_to(t, DOWN, buff=0.35)
            self.add(s)


def landmark_marker(world, x, y, size=0.22):
    tri = Triangle(fill_color=LANDMARK, fill_opacity=1.0,
                   stroke_color=INK, stroke_width=1.0)
    tri.set(width=size * 2).move_to(world.pt(x, y))
    return tri


def polyline(world, xs, ys, **kw):
    m = VMobject(**kw)
    if len(xs) < 2:
        xs = np.asarray([xs[0], xs[0]]) if len(xs) else np.zeros(2)
        ys = np.asarray([ys[0], ys[0]]) if len(ys) else np.zeros(2)
    m.set_points_as_corners(world.pts(xs, ys))
    return m
