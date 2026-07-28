# Static-Obstacle Avoidance

How the planner keeps paths out of convex no-go polygons, in **both** stages of
the pipeline. This document covers the **continuous (B-spline refinement)**
stage in detail; the discrete A\* stage is summarized because the continuous
stage reuses its representation verbatim.

- **Discrete stage** — `src/obstacles.jl`: a boolean feasibility filter on the
  A\* search (a segment is admissible only if it clears every obstacle).
- **Continuous stage** — `src/minvo.jl` + edits in
  `planners/hexspline_cl.jl::optimize_continuous`: MINVO-hull chance
  constraints added to the spline optimizer, plus a post-refinement
  re-verification pass.

Both are **pure feasibility**. Obstacles never enter the measurement model and
never modify covariance propagation. Nothing here writes `Σ`; the covariance
math in `src/covariance.jl` is untouched. `Σ` is only *read*.

---

## 1. Problem and scope

A convex obstacle is the intersection of half-planes. With unit outward face
normals `aⱼ` and offsets `bⱼ`, a point `x` is **inside** iff `aⱼ·x ≤ bⱼ` for
every face `j`, and **outside** iff it clears at least one face:
`aⱼ·x ≥ bⱼ` for some `j`.

We want the refined B-spline to stay outside every obstacle, with a
**chance-constrained margin** that accounts for the vehicle's position
uncertainty `Σ` and a per-obstaclPe risk `δ`. The safe (outside) half-space for
a point with covariance `Σ` is

```
aⱼ·x  ≥  bⱼ + r_a + z·√(aⱼᵀ(Σ + Σₒ)aⱼ),     z = Φ⁻¹(1 − δ)
```

where `r_a` is the vehicle radius and `Σₒ` is optional obstacle-location
covariance (default 0). This is the **exact same inflation** used by the
discrete filter — same `δ`, same `z = OBSTACLE_Z`, same `Σₒ`, same general
(anisotropic) projected-variance term `√(aᵀΣa)`.

**In scope:** convex polygons, static obstacles, feasibility only.
**Out of scope (deferred):** obstacles as landmarks, covariance changes,
non-convex decomposition, dynamic/inter-agent avoidance, hard/projected
enforcement (would need a QP solver).

---

## 2. Why MINVO

The optimizer moves **control points**, but collision is about the **curve**.
A cubic B-spline segment is *not* the straight line between control points, so
checking control points (or sampled curve points) is either unsafe or
expensive. Instead we bound the curve by a convex hull that is **linear in the
control points**:

> For each segment `j`, the curve lies inside the convex hull of its **MINVO
> control points** `Q_mv⁽ʲ⁾ = Q_bs⁽ʲ⁾ · M_seg`, where `Q_bs⁽ʲ⁾` is the 2×4
> matrix of that segment's four B-spline control points (columns) and `M_seg`
> is a fixed 4×4 conversion matrix.

MINVO (mit-acl/minvo) gives the *minimum-volume* such enclosing simplex, so the
hull is tight. Because each MINVO vertex is a fixed linear combination of the
segment's control points, "vertex on the safe side of a face" is a **linear
inequality in the optimizer's free variables** — exactly what we need to add to
a gradient-based optimizer cheaply and soundly.

### 2.1 The conversion `M_seg = A_bs · inv(A_mv)`

Convention (matching mit-acl/mader): a degree-3 curve on a segment is
`p(t) = Q · A · τ`, with `τ = [t³, t², t, 1]ᵀ` and `A` a 4×4 whose **row i is
the monomial coefficients of basis function i**. Two bases (`bs`, `mv`)
describing the *same* curve satisfy

```
Q_bs · A_bs · τ = Q_mv · A_mv · τ   ∀τ
⇒  Q_mv = Q_bs · A_bs · inv(A_mv)  =  Q_bs · M_seg,     M_seg = A_bs · inv(A_mv)
```

- **`A_mv`** — the degree-3 MINVO basis matrix `getA_MV(3, [-1,1])`, ported
  verbatim from mit-acl/mader (`mader_types.hpp`, `A_pos_mv_rest`). Its columns
  sum to a partition of unity and it is non-negative on its interval (so the
  curve is a convex combination of the MINVO control points ⇒ inside their
  hull).
- **`A_bs`** — the B-spline basis matrix for that segment. A **clamped** cubic
  spline has non-uniform boundary segments, so the first two and last two
  segments each have a *different* `A_bs` than the uniform interior. Rather than
  hand-transcribe five boundary matrices, we derive `A_bs` **numerically from
  this repo's own Cox-de Boor basis** (see `bs_segment_matrix`), guaranteeing
  consistency with how the repo actually evaluates the spline.

### 2.2 Committed faces (why convexity needs a chosen side)

"Outside a convex obstacle" is a **disjunction** (clear face 1 *or* 2 *or* …) —
non-convex. We cannot hand a disjunction to a gradient optimizer. So each
segment is **committed to one specific face** — the side the discrete A\* seed
already went around (its homotopy). Clearing that one committed face's
half-space is convex, and — crucially — **sound**: if every MINVO vertex
satisfies `aⱼ·v ≥ bⱼ + margin`, then the whole hull (hence the whole curve
segment) is outside the half-space `aⱼ·x ≤ bⱼ` that contains the obstacle, so
the segment is outside the obstacle regardless of the other faces.

The commitment is derived once, from the discrete seed, and held fixed through
refinement (the homotopy cannot flip mid-optimization).

---

## 3. `src/minvo.jl` — functions

Loaded right after `src/obstacles.jl` in `generate_plan.jl`, so it can reuse
`Obstacle`, `OBSTACLES`, `OBSTACLE_Z`, `AGENT_RADIUS`, `norminv`. It calls the
pure B-spline helpers (`bspline_pad_controls`, `bspline_open_uniform_knots`,
`bspline_basis_funs`) from `planners/hexspline_cl.jl` **only at runtime**, so
include order is fine.

### Constants

| Name | Meaning |
|---|---|
| `A_MV` | 4×4 degree-3 MINVO basis matrix (ported from mader). |
| `A_MV_INV` | `inv(A_MV)`, precomputed once. |
| `OBSTACLE_CONTINUOUS` | `Bool` toggle (`config: obstacle_continuous`, default `true`). `false` ⇒ empty plans ⇒ continuous stage is a no-op (discrete filter unaffected). |
| `_MSEG_CACHE` | `Dict{Int → Vector{Matrix}}`: `M_seg` list per padded control count. |

### `bs_segment_matrix(nctrl, degree, seg) → 4×4`

Derives the B-spline basis matrix for segment `seg` **numerically**:

1. Build the clamped open-uniform knot vector (`bspline_open_uniform_knots`) —
   the exact one this repo uses.
2. Take the segment's knot span `[uL, uR]` (find-span index `degree + seg`).
3. Sample the four nonzero basis functions (`bspline_basis_funs`) at four local
   parameters `t ∈ [0,1]` mapped into `[uL, uR]`.
4. Fit each basis function's cubic power coefficients via a 4×4 Vandermonde
   solve. Row `r` of the returned `A_bs` = coefficients `[t³,t²,t,1]` of the
   basis function for control point `seg+r−1`.

Because it reads the repo's actual basis, it is automatically correct for every
segment type (first/second/interior/second-to-last/last). Validated in
`test_minvo.jl` against mader's hardcoded `A_pos_bs_{seg0,rest,last}`.

### `segment_M_matrices(nctrl_padded, degree) → Vector{4×4}`

Returns one `M_seg = A_bs · inv(A_mv)` per segment, memoized in `_MSEG_CACHE`
keyed by padded control count (segment types are fixed by count for a clamped
uniform spline, and the count never changes during optimization). Only a matrix
multiply happens at solve time; the derivation runs once.

### `pad_source_indices(n) → Vector{Int}`

Mirrors `bspline_pad_controls`'s endpoint duplication as an index map: padded
position `k` came from unpadded control `pad_source_indices(n)[k]`. Used to look
up the `Σ` carried on the original (unpadded) control points for a padded
segment. (`n≥4` ⇒ identity; `n=3 → [1,1,2,3,3]`; `n=2 → [1,1,2,2]`;
`n=1 → [1,1,1,1]`.)

### `struct SegFace`

One committed obstacle row-set: `seg` (segment index), `Mseg` (its conversion
matrix), `src4` (the 4 unpadded source control indices, for `Σ` lookup), `obs`
(obstacle index), `face` (committed face index).

### `seg_control_matrix(padded, seg) → 2×4`

The segment's four control points as columns — the `Q_bs` for `Q_mv = Q_bs·Mseg`.

### `build_obstacle_plan(seed_ctrls, degree) → Vector{Vector{SegFace}}`

**The face-commitment step.** For each agent, each segment, each obstacle:
commit the face whose **worst (min-over-the-4-MINVO-vertices) signed clearance
is largest** at the discrete seed geometry — i.e. the side the seed rounded.
Returns per-agent lists of `SegFace`. Empty (no-op) when there are no obstacles
or `OBSTACLE_CONTINUOUS` is off. Run once at setup on the seed control points;
held fixed thereafter.

Why max-min-clearance: a far segment clears one face with large margin ⇒ that
face is committed and the barrier is inactive. A near segment commits the least
violated face; smoothing then pushes it out that side. This self-selects
relevance without a distance threshold and never lets the optimizer cross to
the other side of an obstacle.

### `cov_index(src, nctrl, len) → Int`

Maps an unpadded control index to the covariance-track index. Identity when `Σ`
is carried per control point (`cont_unc_use_waypoints: true`, the default);
proportional fallback when `Σ` is per spline sample.

### `worst_proj_var(ax, ay, obs, covs_a, src4, nctrl) → Float64`

The **Σ decision**: for face normal `(ax,ay)`, the largest projected variance
`aᵀ(Σ+Σₒ)a` over the segment's four control points (re-propagated,
worst-of-segment). Feeds the inflation `z·√(worst_proj_var)`.

### `obstacle_constraint_value(plans, ctrls, covs; mode, barrier_mu) → Float64`

The penalty/barrier value summed over every committed row (per agent, per
`SegFace`, per MINVO vertex). For vertex `v` and committed face `f`,
`slack = a_f·v − (b_f + r_a + z·√(worst_proj_var))`:

- `mode = :smooth` → `CONT_SMOOTH_PENALTY · slack²` when `slack < 0` (feasibility
  recovery).
- `mode = :barrier` → `−barrier_mu·log(slack)` when `slack > 0`, else
  `CONT_BARRIER_HARD_PENALTY · (1e-9 − slack)²`.

This is **identical in shape** to the optimizer's existing uncertainty /
curvature / support-length terms.

### `verify_obstacle_clearance(plans, ctrls, covs; tol) → Vector{(a,seg,obs,face,depth)}`

**The convex feasibility gate.** Re-checks every committed MINVO row at the final
controls + `Σ`; returns the breached rows with their worst violation depth.
`obstacles_clear` (used inside the optimizer's feasibility gates) is
`isempty(verify_obstacle_clearance(...))`. **Sound but conservative:** clearing a
committed face ⇒ the whole MINVO hull (hence the curve segment) is outside that
half-space ⇒ outside the obstacle. It can *over-report*, because the MINVO hull
bounds the curve only loosely — near the clamped spline ends the large boundary-
segment hull can drape over a small nearby obstacle the curve actually rounds,
flagging a "breach" (observed depths up to ~29 m) with no collision. So this
certifies *feasibility of the convex program*, **not** physical collision — for
that use `verify_curve_collisions`.

### `obstacle_penetration(x, y, obs) → Float64`

How deep a point is inside a convex obstacle: `0` if outside (clears some face),
else the min distance to a face (unit normals ⇒ signed clearance). The Σ=0
geometric containment test — "is the mean point physically inside".

### `verify_curve_collisions(ctrls; samples_per_seg, tol) → Vector{(a,obs,depth)}`

**The physically-real collision report (spec E).** Densely samples each agent's
refined clamped B-spline and tests true polygon containment of the **mean** curve
(`obstacle_penetration`, Σ=0). Returns `(agent, obstacle, depth)` for every
obstacle the curve *actually* enters. Unlike `verify_obstacle_clearance` (hull,
conservative), the depth is a real geometric collision — a curve that merely
rounds a nearby obstacle is **not** reported. Needs no committed plan or `Σ`
(pure geometry); reuses the repo's own `bspline_sample_path`. The re-verification
site prints this as the headline verdict, with a one-line note when the
conservative hull flagged rows the curve clears.

### `obstacles_clear(plans, ctrls, covs) → Bool`

`isempty(verify_obstacle_clearance(...))`. Used inside the optimizer's
feasibility gates.

---

## 4. Reused from `src/obstacles.jl` (discrete stage)

The continuous stage does **not** redefine any of this:

| Symbol | Role |
|---|---|
| `struct Obstacle` | `verts`, `A` (m×2 unit outward normals), `b` (offsets, inside = `aⱼ·x ≤ bⱼ`), `Σo`. |
| `norminv(p)` | Acklam `Φ⁻¹` (no `Distributions.jl`). |
| `OBSTACLE_DELTA`, `OBSTACLE_Z` | risk `δ`, `z = Φ⁻¹(1−δ)`. |
| `AGENT_RADIUS` | vehicle radius `r_a`. |
| `OBSTACLES` | parsed obstacle list from config. |

Same representation, same `δ`, same `Σₒ`, same `Φ⁻¹`, same anisotropic
projected-variance form — the discrete and continuous stages cannot diverge.

---

## 5. Integration in `optimize_continuous`

`planners/hexspline_cl.jl`. All edits mirror the existing constraint handling;
covariance is already re-propagated per iterate by `eval_continuous` (which
returns `covs_all` and `ctrl_list`), so the obstacle terms just read them.

```
optimize_continuous(paths, graph, landmarks, num_agents, output_dir; ...)
│
├─ build seed control points  all_agent_wpts               (discrete waypoints)
│
├─ obstacle_plans = build_obstacle_plan(all_agent_wpts, SPLINE_DEGREE)   ← commit faces ONCE
│
├─ init feasibility:  ... && obstacles_clear(obstacle_plans, init_ctrls, init_covs)
│
├─ SMOOTHING phase (only if the initial point is infeasible)
│    smoothing_objective(f):
│        ... + obstacle_constraint_value(obstacle_plans, ctrl_list, covs_all; mode=:smooth)
│    per-iter feasibility:  ... && obstacles_clear(obstacle_plans, sctrls, scovs)
│
├─ BARRIER phase (minimize primary length subject to all constraints)
│    spline_barrier_objective(f, μ):
│        ... + obstacle_constraint_value(obstacle_plans, ctrl_list, covs_all; mode=:barrier, barrier_mu=μ)
│    per-iter feasibility (gates best_feasible_flat):
│        ... && obstacles_clear(obstacle_plans, ctrls2, covs2)
│
├─ select best_feasible_flat (else best_flat) → opt_ctrls, opt_covs
│
└─ RE-VERIFICATION (spec E):
     verify_obstacle_clearance(obstacle_plans, opt_ctrls, opt_covs)
        → print "all MINVO segments clear ✓"  or  the breached rows + depths
```

Key points:

- **Face commitment is done once**, from the discrete seed. Refinement only
  moves control-point positions; segment counts and committed faces are fixed,
  so `SegFace`/`M_seg` indexing stays valid throughout.
- **`Σ` is frozen-per-evaluation, not per-solve.** Each objective call reads the
  `covs_all` `eval_continuous` computed for the current iterate — the same
  treatment the goal-uncertainty barrier already gets. The standoff therefore
  adapts as the spline moves.
- **All agents are constrained** (primary is the last index; supports precede
  it), matching the discrete filter, which also gated every agent.
- **Feasibility gates include obstacles**, so `best_feasible_flat` — the iterate
  the optimizer ultimately returns when one exists — is obstacle-aware.

`generate_plan.jl` gains a single `include("src/minvo.jl")` line (after
`obstacles.jl`). No other entry script loads the planner, so the MC/analysis
scripts are unaffected.

---

## 6. End-to-end flow

```
config obstacles ─► parse_obstacles ─► OBSTACLES (faces aⱼ, bⱼ, Σₒ)     [obstacles.jl]
        │
        ▼
   DISCRETE A*  ── segment_obstacle_free gate ──►  seed path (per agent)  [hexspline_cl.jl]
        │            (waypoints clear every obstacle; defines the homotopy)
        ▼
   build_obstacle_plan(seed, degree)  ─►  committed face per (agent,seg,obstacle)   [minvo.jl]
        │            Q_mv = Q_bs·M_seg,  pick the face the seed rounded
        ▼
   CONTINUOUS refinement (Adam + finite-diff, smoothing → barrier)        [hexspline_cl.jl]
        │   objective += Σ over committed rows of  penalty/barrier(slack),
        │   slack = a_f·v_mv − (b_f + r_a + z·√(worst aᵀ(Σ+Σₒ)a))
        │   feasibility gates require obstacles_clear
        ▼
   verify_obstacle_clearance(final)  ─►  "all clear ✓"  |  breached rows + depths   [minvo.jl]
```

The discrete stage supplies a collision-free **homotopy**; the continuous stage
commits to it and keeps the *smooth* curve on the same side while it shortens
and smooths the path; the re-verification confirms (or flags) the result.

---

## 7. Design decisions

| Decision | Choice | Why |
|---|---|---|
| Face assignment | Derive per-segment from the discrete seed geometry, post-A\* | No change to A\*/`obstacles.jl`; the discrete path already encodes a valid homotopy. |
| `Σ` per segment | Re-propagated per-control-point, worst-of-segment | `eval_continuous` already carries per-control-point `Σ`; worst-case is conservative and consistent with the uncertainty barrier. |
| Agent scope | All agents | Matches the discrete filter, which gated every agent. |
| Enforcement | Log-barrier (barrier phase) + quadratic penalty (smoothing) | The optimizer is hand-rolled Adam with finite-difference gradients — no QP/projection available; this mirrors the existing constraints exactly. |

---

## 8. Validation

`test_minvo.jl` (standalone, `@assert`, no framework) covers:

1. **Enclosure** — sampled curve points lie inside `conv(Q_mv)` for every
   segment; the MINVO hull is tighter than the control-point hull.
2. **Interval / clamped correctness** — numerically-derived `A_bs` matches
   mader's hardcoded `A_pos_bs_{seg0,rest,last}`; boundary ≠ interior;
   `M_seg` matches mader's `M_pos_bs2mv_rest`.
3. **Inflation monotonicity** — larger `Σ` (along the face normal) or smaller
   `δ` tightens the feasible set; cross-normal variance is irrelevant
   (anisotropy).
4. **Reduction** — `Σ = 0` collapses to plain geometry (standoff = `r_a`).
5. **End-to-end re-verification** — a clear seed passes; a control point dragged
   through the obstacle is flagged with a positive depth.

**Teeth A/B** (long wall across the corridor, `x∈[300,700]`, `y∈[-150,150]`):
enforcement **on** drives the primary fully under the wall (`y_min ≈ −179`,
clears, re-verify ✓); **off** (`obstacle_continuous: false`) leaves it cutting
through (`y_min ≈ −146`, inside the band). Same discrete seed both runs — the
difference is purely the continuous constraint.

---

## 9. Known limitation — why "soft" enforcement can still breach

Enforcement is **soft**: the obstacle constraint enters the objective as a
penalty (smoothing phase) and a log-barrier (barrier phase), never as a hard
constraint the solver is guaranteed to satisfy. There is no projection or QP
step that *forces* the iterate back onto the feasible set — the optimizer is
hand-rolled Adam on finite-difference gradients, so all it can do is add a
*force* pushing away from the committed face. Whether the path actually ends up
clear depends on that force winning the tug-of-war against the other terms.

**What it competes against.** The same objective is simultaneously minimizing
primary path length and penalizing curvature above `MAX_CURVATURE`, support-arc
mismatch, and (via the feasibility gates) goal uncertainty. Near a tight corner
all of these pull the *same* segment in conflicting directions:

- **length** pulls the segment straight (toward the obstacle it's trying to
  round),
- **curvature** resists the sharp bend needed to hug the corner (a tighter
  detour means higher curvature, which the curvature term fights),
- the **obstacle barrier** pushes outward, but its gradient only grows as the
  vertex *approaches* the face — and in the barrier phase `μ` is *decaying* by
  design (`CONT_BARRIER_DECAY` per stage) to let length dominate and shorten the
  path.

When the corner is tight enough that the feasible detour also costs a lot of
length/curvature, the decaying barrier can be outweighed and the line search
settles at a point a few metres inside the committed face. The barrier is
`−μ·log(slack)` for `slack>0` and switches to a quadratic
`CONT_BARRIER_HARD_PENALTY·(1e-9−slack)²` once breached, so a breach is *bounded
and penalized*, not unbounded — but with finite penalty weight it is not
*impossible*. It is a soft floor, not a wall.

**How bad, in practice.** The robustness sweep
(`test_obstacle_robustness.jl`, all four scenarios × {5,10,20} obstacles)
stresses exactly this. Discrete A\* found a route in **all 12** runs; the refined
**mean** path was collision-free in **11/12** (every 20-obstacle case included).
The one real breach (`single_10`) is the primary pinned near `y≈0` by length
pressure through an obstacle cluster, **clipping a corner by ≈3 m** into a ~40 m
box — precisely the tug-of-war above. So the residual is real but rare and
shallow; it is **reported, never silently shipped** (§3 re-verification, and the
`ponytail:` note at the re-verification site).

> Note the distinction from a *reporting* artifact. The **gate**
> (`verify_obstacle_clearance`) tests the conservative committed-face MINVO hull,
> which over-reports: near the clamped spline ends the large boundary-segment
> hull drapes over a small nearby obstacle the curve rounds, so the gate printed
> large "breaches" (up to ~29 m) that were **not** collisions. The
> re-verification headline now uses `verify_curve_collisions` (§3) — true
> polygon containment of the sampled mean curve — so a reported depth is a
> genuine collision. The ≈3 m `single_10` case is a real soft-enforcement
> breach; every other sweep "breach" was a hull artifact, confirmed clear by the
> curve test.

**Two related notes:**

- the `straight_cont` planner **bypasses the discrete filter**, so its straight
  seed has no valid homotopy to commit to (segments split to opposite faces);
  re-verification correctly flags the residual. Obstacle avoidance in the
  continuous stage assumes a discrete-stage homotopy.
- Guaranteed clearance is the **hard / projected** upgrade path (project each
  iterate onto `{A q ≤ b}` — or solve a small QP per step — so feasibility is an
  invariant rather than a penalty). It needs a QP solver and is deferred; it is
  the fix if the ~3 m residual is unacceptable for the mission.

---

## 10. Configuration

```yaml
# config/main.yaml
obstacle_delta: 0.05          # risk δ per (agent, obstacle) pair;  z = Φ⁻¹(1−δ)
agent_radius: 0.0             # r_a, inflates every face outward
obstacle_edge_samples: 2      # discrete-stage segment sampling
obstacle_continuous: true     # enforce in the continuous B-spline stage too
obstacles: 200,-150; 260,-150; 260,-100; 200,-100 | 400,60; 460,60; 430,120 @ 4,0,0,4
#          └── convex polygon vertices "x,y" sep ';'  ── polygons sep '|'  ── optional Σₒ after '@'
```

Set `obstacle_continuous: false` to run discrete-only (useful for A/B and
debugging). Absent/empty `obstacles:` ⇒ everything here is a no-op.
