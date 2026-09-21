"""Presentation-only geometry helpers.

Nothing here affects what the planner computed; these only decide how big to
draw an ellipse so the picture reads clearly without visually contradicting
the obstacle test the planner actually ran.

Why this needs a real function and not a single fixed number: Fig. 1's own
print convention draws 2-sigma at sigma-x10 for visibility, which is fine on a
static figure but breaks down at the one point where the primary threads a
genuinely tight gap. Measured on that run: the tightest real clearance to the
obstacle is ~9.4 m, where sigma is ~2.1 m -- a x10 ellipse there has an ~42 m
semi-major axis, ballooning clean past a wall the vehicle's real (tiny) belief
never comes close to. A single global scale can't serve both that moment and
the rest of the route, where sigma is genuinely worth exaggerating to be seen
at all, so the scale is evaluated per point instead.
"""
import numpy as np


def _point_to_poly_dist(px, py, poly):
    poly = np.asarray(poly, float)
    n = len(poly)
    best = np.inf
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        cx, cy = x1 + t * dx, y1 + t * dy
        best = min(best, np.hypot(px - cx, py - cy))
    return best


def make_local_scale(obstacles, default=6.0, floor=1.5, margin=3.0, nstd=2.0):
    """scale_at(x, y, sigma_major) -> a display scale, safe at this point.

    `default` away from any obstacle (still less than the old x10 -- see
    make_local_scale's docstring for why 10 was too aggressive everywhere);
    shrinks smoothly as an obstacle nears, floored at `floor` so an ellipse
    never collapses to invisibility even mid-pinch. The floor means the very
    tightest moment can still show a boundary a couple of metres from the
    wall rather than a hair's breadth -- a presentational choice, since the
    real (unscaled) ellipse there clears with room to spare.
    """
    obstacles = list(obstacles)

    def scale_at(x, y, sigma_major):
        if not obstacles or sigma_major <= 1e-9:
            return default
        d = min(_point_to_poly_dist(x, y, o) for o in obstacles)
        needed = max(0.0, d - margin) / (nstd * sigma_major)
        return float(np.clip(needed, floor, default))

    return scale_at


def sigma_major(cov):
    cov = np.asarray(cov, float)
    vals = np.clip(np.linalg.eigvalsh((cov + cov.T) / 2.0), 0.0, None)
    return float(np.sqrt(vals.max()))
