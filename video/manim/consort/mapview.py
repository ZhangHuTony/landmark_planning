"""The static part of a map scene: lattice, obstacles, landmarks, start, goal."""
import numpy as np
from manim import VGroup, Dot, Star, Text, UP, DOWN

from .mobjects import HexLattice, obstacle_poly, landmark_marker
from .palette import START, GOAL, INK, FONT


def base_map(world, scene, with_lattice=True):
    g = VGroup()
    lattice = HexLattice(world, scene.hex_centers, scene.hex_radius) if with_lattice else VGroup()
    obstacles = VGroup(*[obstacle_poly(world, o) for o in scene.obstacles])
    lms = VGroup(*[landmark_marker(world, lm["x"], lm["y"]) for lm in scene.landmarks])
    start = Dot(world.pt(*scene.start), radius=0.09, color=START, z_index=5)
    goal = Star(n=5, outer_radius=0.19, color=GOAL, fill_opacity=1.0,
                stroke_width=0, z_index=5).move_to(world.pt(*scene.goal))
    g.add(lattice, obstacles, lms, start, goal)
    g.lattice, g.obstacles, g.landmarks = lattice, obstacles, lms
    g.start, g.goal = start, goal
    return g


def label_at(world, x, y, text, size=19, color=INK, direction=UP, buff=0.14):
    t = Text(text, font=FONT, font_size=size, color=color)
    t.next_to(world.pt(x, y), direction, buff=buff)
    return t
