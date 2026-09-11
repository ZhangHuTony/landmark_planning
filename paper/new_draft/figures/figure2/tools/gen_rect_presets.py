# Emits Julia SCENARIOS entries :ladder_r1..:ladder_r5 (rectangular mazes + scattered shapes).
# Row centres on the 100 m lattice: r(k) = 86.6025 k; gap faces sit 60 m off a row centre.
R = lambda k: 86.6025 * k
def rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
def wall(x0, x1, gaps):
    """full-height wall x∈[x0,x1] with gaps at the given rows (descending)."""
    ga, gb = sorted(gaps, reverse=True)
    blocks = []
    if R(ga) + 60 < 320: blocks.append(rect(x0, x1, R(ga) + 60, 330))
    blocks.append(rect(x0, x1, R(gb) + 60, R(ga) - 60))
    if R(gb) - 60 > -320: blocks.append(rect(x0, x1, -330, R(gb) - 60))
    return blocks
def bar(x0, x1, k):            # a plug filling row k (53 m tall, 60 m off the neighbours)
    return rect(x0, x1, R(k) - 26.6, R(k) + 26.6) if k != 3 and k != -3 else \
           (rect(x0, x1, R(3) - 26.6, 330) if k == 3 else rect(x0, x1, -330, R(-3) + 26.6))
def tri(cx, ytip, ybase, half):  # triangle pointing toward ytip
    return [(cx - half, ybase), (cx + half, ybase), (cx, ytip)]
def hexa(x0, x1, y0, y1, c=25):  # pointed-end hexagon
    ym = (y0 + y1) / 2
    return [(x0, ym), (x0 + c, y0), (x1 - c, y0), (x1, ym), (x1 - c, y1), (x0 + c, y1)]
def house(x0, x1, y0, y1, roof=30):
    return [(x0, y0), (x1, y0), (x1, y1 - roof), ((x0 + x1) / 2, y1), (x0, y1 - roof)]
def house_dn(x0, x1, y0, y1, roof=30):
    return [(x0, y1), (x1, y1), (x1, y0 + roof), ((x0 + x1) / 2, y0), (x0, y0 + roof)]
def diamond(cx, cy, hx, hy):
    return [(cx, cy - hy), (cx + hx, cy), (cx, cy + hy), (cx - hx, cy)]
def trap(x0, x1, y0, y1, inset=20):
    return [(x0, y0), (x1, y0), (x1 - inset, y1), (x0 + inset, y1)]
def mirror(polys):  # flip y, keep vertex order convex-consistent by reversing
    return [[(x, -y) for (x, y) in reversed(p)] for p in polys]

# ---- R1: the shapes_v14 skeleton in rectangles, plus scattered shapes
r1 = wall(200, 350, (1, -1)) + wall(800, 950, (1, -2)) + wall(1200, 1350, (2, -3)) + [
    bar(375, 725, 0), bar(425, 775, -3), bar(975, 1125, 0), bar(1400, 1700, 3),
    house(360, 440, R(3) - 26.6, 330),      # chamber A-B, row +3 west of the pocket (flat side toward the lanes)
    hexa(1000, 1100, -110, -60, 20),        # chamber B-C, row -1
    house(1000, 1200, R(3) - 26.6, 330),    # chamber B-C, row +3
    house_dn(60, 140, -330, R(-3) + 26.6)]  # start pocket, row -3 (flat top toward the lane)
r1_lm = [((1750, -233), (4.0, 3.2)), ((1250, -320), (1.0, 0.8)), ((550, 300), (6.0, 4.8))]
r1_goal = (1700.0, R(2))

# ---- R2: south lane dives -1 -> -3 before wall B and stays on row -3 under a row -2 bar
#      (gaps +1/-1, +1/-3, +2/-3); mid landmark at wall C's -3 gap as in the proven skeleton
r2 = wall(200, 350, (1, -1)) + wall(750, 900, (1, -3)) + wall(1200, 1350, (2, -3)) + [
    bar(375, 440, -3), bar(375, 675, 0), bar(950, 1150, 0), bar(975, 1150, -2),
    bar(1400, 1700, 3),
    house(360, 440, R(3) - 26.6, 330), hexa(1000, 1100, -110, -60, 20),
    house(1000, 1200, R(3) - 26.6, 330), house_dn(60, 140, -330, R(-3) + 26.6)]
r2_lm = [((1750, -233), (4.0, 3.2)), ((1250, -320), (1.0, 0.8)), ((550, 300), (6.0, 4.8))]
r2_goal = (1700.0, R(2))

# ---- R3: staggered wall segments (same gap rows as R1, blocks offset in x)
r3 = [rect(250, 400, R(1) + 60, 330), rect(200, 550, -26.6, 26.6), rect(400, 550, -330, R(-1) - 60),
      rect(800, 950, R(1) + 60, 330), rect(800, 950, R(-2) + 60, R(1) - 60), rect(650, 800, -330, R(-2) - 60),
      rect(1200, 1350, R(2) + 60, 330), rect(1150, 1350, R(-3) + 60, R(2) - 60),
      bar(975, 1125, 0), trap(1400, 1700, R(3) - 26.6, 330),
      house(60, 140, R(3) - 26.6, 330), hexa(1000, 1100, -110, -60, 20),
      house(1000, 1200, R(3) - 26.6, 330), house_dn(60, 140, -330, R(-3) + 26.6)]
r3_lm = r1_lm; r3_goal = r1_goal

# ---- R4: both lanes weave; no pocket landmark, early south landmark instead
#      north +2 -> +1 -> +2 (gaps A +2, B +1, C +2); south -1 -> -2 -> -3
r4 = wall(300, 450, (2, -1)) + wall(750, 900, (1, -2)) + wall(1200, 1350, (2, -3)) + [
    bar(475, 700, 0), bar(500, 700, 3), bar(475, 700, -3), bar(950, 1150, 0),
    house(1000, 1200, R(3) - 26.6, 330), hexa(1000, 1100, -110, -60, 20),
    bar(1400, 1700, 3),
    # three one-cell blocks that remove the 'late lane change at a wall face' variants, which tie
    # with the clean routes at the next node and can shadow them under dominance pruning
    rect(240, 275, 60.0, R(1) + 26.6), rect(700, 725, R(2) - 26.6, R(2) + 26.6), rect(1165, 1195, 60.0, R(1) + 26.6),
    house_dn(60, 140, -330, R(-3) + 26.6)]
r4_lm = [((1750, -233), (4.0, 3.2)), ((1250, -320), (1.0, 0.8)), ((375, -150), (6.0, 4.8))]
r4_goal = (1700.0, R(2))

# ---- R5: R2 mirrored (informative lane north, goal on row -2)
r5 = mirror(r2)
r5_lm = [((x, -y), c) for ((x, y), c) in r2_lm]
r5_goal = (1700.0, -R(2))

def emit(name, polys, lms, goal, comment):
    out = [f"    # ── {name}: {comment}", f"    :{name} => () -> (landmarks = Landmark["]
    for i, ((x, y), (cxx, cyy)) in enumerate(lms):
        tail = "," if i < len(lms) - 1 else "],"
        out.append(f"                              Landmark({x:.1f}, {y:.1f}, [{cxx} 0.0; 0.0 {cyy}]){tail}")
    out.append("                          obstacles = Obstacle[")
    for i, p in enumerate(polys):
        verts = ", ".join(f"({x:.1f},{y:.1f})" for x, y in p)
        tail = "," if i < len(polys) - 1 else "],"
        out.append(f"                              build_obstacle([{verts}]){tail}")
    out.append(f"                          start = (0.0, 0.0), goal = ({goal[0]:.1f}, {goal[1]:.4f})),")
    return "\n".join(out)

hdr = """    # ── Fig. 2 rectangular-maze candidates (2026-09-10) ─────────────────────
    # Five re-drawings of the Fig. 2 ladder scenario as a maze of RECTANGLES
    # (150 m walls with gaps, 53 m row plugs) with a few non-rectangular
    # obstacles scattered in cells no shipped route uses. All on the 100 m
    # lattice of config/mc/sweep_fig2.yaml (rows at multiples of 86.6 m, gap
    # faces 60 m off a row centre, one straight cell before and after every
    # wall for a lane change — see :ladder_maze). Generated by
    # paper/new_draft/figures/figure2/tools/gen_rect_presets.py; landmark order is load-bearing (lms[1] is Σ₀).
"""
body = "\n".join([
    emit("ladder_r1", r1, r1_lm, r1_goal, "the :ladder_shapes skeleton (gaps +1/−1, +1/−2, +2/−3) in rectangles"),
    emit("ladder_r2", r2, r2_lm, r2_goal, "south lane dives −1 → −3 before B and rides row −3 under a row −2 bar (gaps +1/−1, +1/−3, +2/−3)"),
    emit("ladder_r3", r3, r3_lm, r3_goal, "R1 gap rows with the wall blocks staggered in x"),
    emit("ladder_r4", r4, r4_lm, r4_goal, "both lanes weave (+2/−1, +1/−2, +2/−3), wall A at 300; early south landmark, no pocket"),
    emit("ladder_r5", r5, r5_lm, r5_goal, "R2 mirrored: informative lane north, goal on row −2"),
])
import sys
p = "/home/tony-zhang/Research/landmark_planning/src/scenario_generation.jl"
s = open(p).read()
anchor = "                          start = (0.0, 0.0), goal = (2350.0, 86.60254037844386)),\n)\n"
assert s.count(anchor) == 1
s = s.replace(anchor, anchor[:-2] + hdr + body + "\n)\n")
open(p, "w").write(s)
print("presets written:", [n for n in ("ladder_r1","ladder_r2","ladder_r3","ladder_r4","ladder_r5")])
