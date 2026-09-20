"""Shared building blocks for the CONSORT supplementary video.

Nothing in here computes covariance, path length, or comm weight. Every number
shown on screen was produced by the Julia planner (or the HoloOcean sim) and
exported to video/data/; these classes only place it on screen.
"""
from .palette import *          # noqa: F401,F403
from .world import World        # noqa: F401
from .data import load_scene, Track, Scene as SceneData   # noqa: F401
from .mapview import base_map, label_at  # noqa: F401
from .mobjects import (         # noqa: F401
    HexLattice, cov_ellipse, UncMeter, AUVGlyph, Caption, TitleCard, obstacle_poly,
)
