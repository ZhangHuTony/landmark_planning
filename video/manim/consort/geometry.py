"""Presentation-only geometry helpers.

Nothing here affects what the planner computed; these only decide how the
picture reads clearly without visually contradicting the obstacle test the
planner actually ran.

First attempt (superseded): shrink the ellipse's display scale as it neared
an obstacle. Dropped on request -- a magnification that quietly changes from
one moment to the next reads as "the vehicle got more certain near the wall",
which isn't true; only the exaggeration factor was changing, never the real
sigma. Replaced with the opposite move: the magnification stays ONE fixed
number everywhere, and the obstacle is drawn slightly smaller than the real
one so a full-strength ellipse still clears it. The real, full-size obstacle
is what the planner's own obstacle test used; this is a second, smaller
polygon drawn on top of it for the picture only.
"""
import numpy as np


def _closest_point_on_poly(px, py, poly):
    """Returns (distance, closest_point) from (px,py) to the polygon boundary."""
    poly = np.asarray(poly, float)
    n = len(poly)
    best_d, best_pt = np.inf, poly[0]
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        cx, cy = x1 + t * dx, y1 + t * dy
        d = np.hypot(px - cx, py - cy)
        if d < best_d:
            best_d, best_pt = d, np.array([cx, cy])
    return best_d, best_pt


def sigma_major(cov):
    cov = np.asarray(cov, float)
    vals = np.clip(np.linalg.eigvalsh((cov + cov.T) / 2.0), 0.0, None)
    return float(np.sqrt(vals.max()))


def shrink_obstacle_for_clearance(obstacle, tracks, nstd=2.0, scale=6.0, margin=3.0,
                                  min_factor=0.55, n_samples=80, iters=6):
    """A version of `obstacle` scaled toward its own centroid, just enough
    that a FIXED-scale ellipse (nstd * scale * sigma, real sigma from the
    given tracks) never comes within `margin` metres of it anywhere the
    tracks pass near it.

    `tracks` is a list of (xs, ys, covs) triples, one per agent -- covs is an
    array of 2x2 (or the packed [c11,c12,c22] triples `Track.cov` already
    stores). Scaling toward the centroid is an approximation (a true offset/
    erosion would move every edge in by a fixed distance regardless of shape),
    but for the convex-ish obstacles here it shrinks the near edge by close to
    the intended amount and never distorts the silhouette. `min_factor`
    floors how much it is allowed to shrink, so a pathological case cannot
    collapse the shape to a sliver.

    Iterated rather than solved in one shot: the closest point on the
    boundary can itself move once the polygon shrinks, so a single pass from
    the ORIGINAL polygon under-shrinks slightly (checked: ~1 m short on the
    Fig. 1 obstacle). Each pass re-measures against the current candidate.
    """
    poly = np.asarray(obstacle, float)
    centroid = poly.mean(axis=0)
    x0, x1 = poly[:, 0].min() - 300, poly[:, 0].max() + 300
    y0, y1 = poly[:, 1].min() - 300, poly[:, 1].max() + 300

    packed = []
    for xs, ys, covs in tracks:
        xs, ys, covs = np.asarray(xs), np.asarray(ys), np.asarray(covs)
        idx = np.linspace(0, len(xs) - 1, min(n_samples, len(xs))).astype(int)
        pts, sms = [], []
        for i in idx:
            x, y = xs[i], ys[i]
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            c = covs[i]
            cov = np.array([[c[0], c[1]], [c[1], c[2]]]) if len(c) == 3 else np.asarray(c)
            sm = sigma_major(cov)
            if sm <= 1e-9:
                continue
            pts.append((x, y)); sms.append(sm)
        if pts:
            packed.append((np.array(pts), np.array(sms)))

    factor = 1.0
    cand = poly
    for _ in range(iters):
        worst_shrink = 0.0
        for pts, sms in packed:
            for (x, y), sm in zip(pts, sms):
                needed = nstd * scale * sm + margin
                d, closest = _closest_point_on_poly(x, y, cand)
                if needed <= d:
                    continue
                retreat = needed - d
                R = max(np.linalg.norm(closest - centroid), 1e-6)
                worst_shrink = max(worst_shrink, retreat / R)
        if worst_shrink <= 1e-6:
            break
        factor = max(min_factor, factor * (1.0 - worst_shrink))
        cand = centroid + (poly - centroid) * factor
    return cand, factor
