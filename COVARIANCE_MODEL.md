# Covariance Propagation & Update in `planner.jl`

This document explains how agent position covariance is **propagated** (dead-reckoning
growth) and **updated** (landmark + inter-agent fusion) in `planner.jl`, and maps every
step to the equations in the paper
*"Multi-Agent Path Planning in GPS-Denied Environments with Localization Constraints and
Optimality Guarantees"* (`RA_L_2026__Multi_Agent_Planning_with_Localization_Constraints.pdf`),
Section III (Problem Formulation).

All line numbers refer to `planner.jl`.

---

## 0. Notation map (paper ↔ code)

| Paper symbol | Meaning | Code |
|---|---|---|
| $\Sigma_i[k]$ | agent $i$ covariance at step $k$ | `cov` (a `2×2 Matrix{Float64}`) |
| $\Sigma_i^0$ | initial covariance | `init_cov` (seeded from `lms[1].cov`) |
| $F(\theta_i[k])$ | motion Jacobian | identity here — rotation folded into $Q$ (see §2) |
| $Q_i[k]$ | process (dead-reckoning) noise | `growth_covariance(seg, heading)` — L244 |
| $H$ | measurement Jacobian | identity (position-domain measurement) |
| $R$ | measurement noise | per-landmark $S_\text{total}$ in `landmark_info`; `SENSOR_NOISE²·I` for comms |
| $K_i[k]$ | Kalman gain | implicit — update done in **information form** (§3) |
| $w_{ij}(k)$ | tapered Gaussian sensing weight | `p_detect` (L528) and comm `weight` (L786, L961) |
| $\sigma_c$ | taper length scale | `VISIBILITY_SIGMA` (landmarks), `COMM_SIGMA`/`COMM_RADIUS` (comms) |
| $\bar d$ | terminal uncertainty bound | `UNC_RADIUS_THRESHOLD` via `unc_radius` (§5) |
| $v_i, \Delta t$ | constant speed, timestep | absorbed into arc-length `seg`; growth scales with distance |

**Key modeling choice:** all updates use $H = I$ (the measurement and state both live in 2-D
position space), so the Kalman update collapses to the **information-filter** form
$\Sigma^{-1} \leftarrow \Sigma^{-1} + \sum_j R_j^{-1}$, which is algebraically identical to the
Joseph form printed in the paper but cheaper for 2×2 matrices. See §3.

---

## 1. State representation

Each agent carries a single **2×2 position covariance** $\Sigma_i$. There is no orientation or
velocity in the covariance state — the planner optimizes over *mean-state trajectories* and
propagates covariance deterministically for constraint evaluation only (paper §III.B:
*"Covariance is not a decision variable and is propagated deterministically for constraint
evaluation."*).

---

## 2. Dead-reckoning propagation (process noise $Q$)

### Paper (§III.B)

$$\Sigma_i[k+1\mid k] = F(\theta_i[k])\,\Sigma_i[k\mid k-1]\,F(\theta_i[k])^\top + Q_i[k]$$

$$Q_i[k] = \begin{bmatrix} \sigma_\parallel^2[k] + \alpha_\parallel \Delta t & 0 \\ 0 & \sigma_\perp^2[k] + \alpha_\perp \Delta t \end{bmatrix}$$

i.e. anisotropic growth: more uncertainty *along* the direction of travel ($\parallel$) than
*across* it ($\perp$), then rotated into the world frame by heading $\theta$.

### Code

The rotation $F(\theta)$ and the body-frame $Q$ are combined into one rotated noise matrix.
With $F = I$, the propagation is the additive update `cov = cov + growth_covariance(seg, heading)`:

`growth_covariance` — **L244–251**:
```julia
@inline function growth_covariance(distance::Float64, angle::Float64)
    sd2 = (DIR_UNCERTAINTY_PER_METER  * distance)^2   # σ∥²  (along-track)
    sp2 = (PERP_UNCERTAINTY_PER_METER * distance)^2   # σ⊥²  (cross-track)
    c = cos(angle); s = sin(angle)
    diff = sd2 - sp2
    return [c*c*sd2 + s*s*sp2  c*s*diff;          #  R(θ)·diag(σ∥²,σ⊥²)·R(θ)ᵀ
            c*s*diff            s*s*sd2 + c*c*sp2]  #  expanded inline
end
```
This is exactly $R(\theta)\,\mathrm{diag}(\sigma_\parallel^2,\sigma_\perp^2)\,R(\theta)^\top$ — the paper's
$Q_i[k]$ already rotated into the world frame.

**Anisotropy ratio** (constants block):
- `DIR_UNCERTAINTY_PER_METER = 0.05` (L52) → $\sigma_\parallel = 0.05\cdot\text{distance}$
- `PERP_UNCERTAINTY_PER_METER = DIR_UNCERTAINTY_PER_METER / MAJ_MIN_UNC_RATIO` (L54) → $\sigma_\perp$

Because speed is constant, distance $= v\,\Delta t$, so the paper's per-timestep growth
$\sigma_\parallel^2[k]$ is realized as a per-meter growth `(DIR_UNCERTAINTY_PER_METER · seg)²`.
The constant additive rate $\alpha_{\parallel/\perp}\Delta t$ in the paper is **not** present in the
code — growth here is purely distance-proportional (see §6, deviation 1).

### Where it is applied

| Caller | Line | Context |
|---|---|---|
| `propagate_cov_discrete` | **L613** | `cov = cov + growth_covariance(seg, heading)` (waypoint-to-waypoint) |
| `propagate_cov_continuous` | **L659** | same, along B-spline samples |

`seg` and `heading` are computed per segment at L609–610 / L651–656.

---

## 3. Landmark measurement update (information-filter Kalman)

### Paper (§III.C, Measurement Model — Tapered Gaussian Sensing)

$$K_i[k] = \Sigma_i[k\mid k-1]\,H^\top \left(H\,\Sigma_i[k\mid k-1]\,H^\top + R\right)^{-1}$$

$$\Sigma_i[k] = (I - K_i[k]H)\,\Sigma_i[k\mid k-1]\,(I - K_i[k]H)^\top + K_i[k]\,R\,K_i[k]^\top \quad\text{(Joseph form)}$$

### Code (equivalent information form, $H = I$)

With $H = I$, the Joseph update is algebraically equal to

$$\Sigma_i[k] = \Big(\Sigma_i[k\mid k-1]^{-1} + \sum_j R_j^{-1}\Big)^{-1}$$

where the sum runs over all landmarks $j$ visible at the current position. The code implements
exactly this, in three pieces:

**(a) Per-landmark information $R_j^{-1}$** — `landmark_info`, **L523–553**:
```julia
σ_r2 = SENSOR_NOISE^2                          # along line-of-sight (range)
σ_b2 = (SENSOR_NOISE * BEARING_NOISE_RATIO)^2  # perpendicular (bearing)
# S_sensor = R(bearing)·diag(σ_r2, σ_b2)·R(bearing)ᵀ   (rotated into world frame)
...
inv_p = 1.0 / p_detect                         # tapered-Gaussian inflation (see §4)
t11 = (s11 + lm.cov[1,1]) * inv_p              # R_j = (S_sensor + Σ_landmark)/p_detect
...
return (t22*inv_det, -t12*inv_det, t11*inv_det)  # returns inv(R_j) = R_j⁻¹ directly
```
So $R_j = \dfrac{1}{p_\text{detect}}\big(R(\beta)\,\mathrm{diag}(\sigma_r^2,\sigma_b^2)\,R(\beta)^\top + \Sigma_\text{lm}\big)$, with
$\beta$ the bearing to the landmark. The function returns the **inverse** (information) directly
to avoid allocating $R_j$ then inverting.

**(b) Sum the information from all visible landmarks** — `accumulate_landmark_info`, **L559–568**:
```julia
I11 = 0.0; I12 = 0.0; I22 = 0.0
for lm in lms
    unc_radius(lm.cov) < 1e-8 && continue
    info = landmark_info(ax, ay, lm)
    info === nothing && continue          # outside detection range
    I11 += info[1]; I12 += info[2]; I22 += info[3]   #  Σⱼ Rⱼ⁻¹
end
```
Returns $(I_{11}, I_{12}, I_{22}) = \sum_j R_j^{-1}$.

**(c) Apply the update** — `kalman_info_update`, **L575–586**:
```julia
det_c = cov[1,1]*cov[2,2] - cov[1,2]*cov[2,1]
inv_det = 1.0 / det_c
J11 = I11 + cov[2,2]*inv_det        #  J = Σ⁻¹ + Σⱼ Rⱼ⁻¹   (posterior information)
J12 = I12 - cov[1,2]*inv_det        #  (cov[2,2]/det, −cov[1,2]/det, cov[1,1]/det) = Σ⁻¹
J22 = I22 + cov[1,1]*inv_det
...
return [J22*inv_dj  -J12*inv_dj; -J12*inv_dj  J11*inv_dj]   #  Σ_new = J⁻¹
```
This is $\Sigma_\text{new} = (\Sigma^{-1} + \sum_j R_j^{-1})^{-1}$ — the Joseph update for $H=I$.
Returns `nothing` if the prior or posterior is numerically degenerate ($\det < 10^{-20}$).

### Where it is applied

| Caller | Lines | Notes |
|---|---|---|
| `propagate_cov_discrete` | **L619–627** | guards `if I11>0 || I22>0`; bumps `fusion_count` on success |
| `propagate_cov_continuous` | **L662–670** | same math; on degeneracy keeps prior and `continue`s |

Both call `accumulate_landmark_info` then `kalman_info_update` against the *post-propagation*
covariance (so propagate first, then fuse).

---

## 4. Tapered Gaussian sensing weight $w_{ij}$

### Paper (§III.C)

$$w_{ij}(k) = \exp\!\left(-\frac{\lVert x_i[k] - x_j[k]\rVert^2}{2\,\sigma_c^2}\right)$$

*"modulates measurement availability."* The same functional form is used in two distinct roles
in the code:

**(a) Agent → landmark detection probability** — `landmark_info`, **L528**:
```julia
p_detect = exp(-d2 / (2 * VISIBILITY_SIGMA^2))   # σ_c = VISIBILITY_SIGMA = 75.0 (L58)
p_detect < 1e-6 && return nothing                # hard cutoff: landmark not seen
```
Here the weight enters as a **noise inflation** `1/p_detect` (L543): a barely-visible landmark
gets large $R_j$, so it contributes little information. This is the code's realization of
"modulating measurement availability."

**(b) Agent → agent comm weight** — see §5. Two variants:
- exact synchronized eval, **L786**: `weight = exp(-dist^2 / (2 * COMM_SIGMA^2))` ($\sigma_c$ = `COMM_SIGMA` = 50.0, L60)
- in-search approximation, **L961**: `w = exp(-d2 / (2*COMM_RADIUS^2))` ($\sigma_c$ = `COMM_RADIUS` = 300.0, L57)

---

## 5. Inter-agent cooperative fusion

Supports reduce the primary's uncertainty by sharing localization information when nearby —
the cooperative-localization mechanism (paper §II / §III, refs [2], [7]). Implemented as a
**Gaussian-weighted information-filter fusion**:

$$\Sigma_a^{+} = \Big(\Sigma_a^{-1} + w_{ab}\,(\Sigma_b + R)^{-1}\Big)^{-1}, \qquad R = \texttt{SENSOR\_NOISE}^2 I$$

applied bidirectionally (each agent fuses the other). There are **two implementations** that must
stay consistent:

### (a) Exact evaluator — `apply_synchronized_propagation!`

Used for the final/recorded uncertainty. Reached via `evaluate_joint_discrete` →
`evaluate_full_paths`, which is the authoritative scoring used at every A* goal pop
and inside the continuous optimizer.

The function uses an **interleaved single-pass** approach: for each 100 m comm
checkpoint it first propagates each agent (dead-reckoning + landmark fusion via
`propagate_segment!`) up to the nearest waypoint index *from the last fused
covariance*, then applies the bidirectional fusion, then refreshes the running
covariance with the post-fusion value before advancing to the next checkpoint.
This means the reduced covariance after comm event *k* correctly seeds
propagation toward event *k+1*, exactly as real cooperative localization behaves.

```julia
weight = exp(-dist^2 / (2 * COMM_SIGMA^2))              # tapered Gaussian
if weight > COMM_WEIGHT_MIN                              # floor = 1e-4 (L61)
    S_s = all_covs[sender][idx_s] + SENSOR_NOISE^2 * I(2)   # Σ_b + R
    S_r = all_covs[receiver][idx_r] + SENSOR_NOISE^2 * I(2)
    # Receiver fuses sender:
    new_inv_P_r = inv(all_covs[receiver][idx_r]) + weight * inv(S_s)
    all_covs[receiver][idx_r] = inv(new_inv_P_r)
    # Sender fuses receiver (bidirectional):
    new_inv_P_s = inv(all_covs[sender][idx_s]) + weight * inv(S_r)
    all_covs[sender][idx_s] = inv(new_inv_P_s)
end
```
The per-step propagation math is shared with `propagate_cov_discrete` via the helper
`propagate_segment!` to ensure both paths use identical physics.

### (b) In-search approximation — `pairwise_comm`, **L957–972**

A cheaper version used *during* the A* expansion (so the heuristic stays consistent and each
expansion is fast). Same algebra, different taper scale and a per-step trigger:

```julia
d2 = (xa-xb)^2 + (ya-yb)^2
w  = exp(-d2 / (2*COMM_RADIUS^2))            # L961  (σ_c = COMM_RADIUS = 300)
w < 1e-3 && return cov_a, cov_b             # L962  skip negligible fusion
noise = SENSOR_NOISE^2
Ib = inv2(cov_b .+ noise .* I)              # (Σ_b + R)⁻¹
new_a = inv2(inv2(cov_a) .+ w .* Ib)        # Σ_a⁺ = (Σ_a⁻¹ + w·(Σ_b+R)⁻¹)⁻¹
Ia = inv2(cov_a .+ noise .* I)
new_b = inv2(inv2(cov_b) .+ w .* Ia)        # bidirectional
```
Driven by `apply_joint_step_comms` (**L1029–1048**), which only fuses agent pairs whose
arc-distances are within `COMM_INTERVAL_DIST = 5.0` (L501) of each other:
```julia
abs(dists[a] - dists[b]) > COMM_INTERVAL_DIST && continue   # L1039
new_a, new_b = pairwise_comm(updated[a], updated[b], xa, ya, xb, yb)  # L1042
```

> The general two-source information fuse $(\Sigma_a^{-1}+\Sigma_b^{-1})^{-1}$ also exists as
> `fuse_cov` (L262), built on the 2×2 inline inverse `inv2` (L254).

---

## 6. Terminal uncertainty constraint & scalar metric

### Paper (§III.D)

$$\det(\Sigma_i[N]) \le \bar d, \quad \forall i \in \mathcal{I}$$

### Code — `unc_radius` / `unc_det_radius`, **L199–205**:
```julia
@inline function unc_det_radius(cov::Matrix{Float64})
    d = cov[1,1]*cov[2,2] - cov[1,2]*cov[2,1]   # det(Σ)
    return max(d, 1e-18)^(0.25)                 # det(Σ)^(1/4)
end
unc_radius(cov) = unc_det_radius(cov)
```
The code constrains $\det(\Sigma)^{1/4} \le$ `UNC_RADIUS_THRESHOLD` (`= 3.75`, L17), which is the
paper's $\det(\Sigma_i[N]) \le \bar d$ with $\bar d = 3.75^4$. The fourth-root form is used because
for an isotropic covariance $\sigma^2 I$ it equals $\sigma$ (an interpretable "1σ radius" in meters),
and it makes distances and uncertainties comparable in the same units. Checked via
`unc_within_threshold` (L207) with tolerance `UNC_FEAS_TOL`.

This same metric drives the **PSD dominance pruning** `cov_dominates` (L234–241): state $A$
dominates $B$ when $\Sigma_B - \Sigma_A \succeq 0$ (checked by principal minors). The paper states
the weaker scalar version $\det(\Sigma_A) \le \det(\Sigma_B)$ (§IV "sound dominance pruning"); the
code uses the stronger, still-sound, PSD partial order.

---

## 7. End-to-end call graph

```
evaluate_full_paths (L1060)                ← A* goal scoring & continuous optimizer
  └─ evaluate_joint_discrete (L695)
       └─ apply_synchronized_propagation! (L727)
            ├─ propagate_cov_discrete (L592)  ──┐ per-agent, independent
            │    ├─ growth_covariance (L244)    │  §2 propagation
            │    ├─ accumulate_landmark_info (L559)  §3 landmark info
            │    └─ kalman_info_update (L575)        §3 information update
            └─ (inter-agent fusion, L786)            §5(a) cooperative fusion

joint_astar / joint_astar_collect           ← per-expansion (approximate)
  └─ edge_cov_continuous (L824)
       └─ propagate_cov_continuous (L636)    same §2 + §3 along the edge
  └─ apply_joint_step_comms (L1029)
       └─ pairwise_comm (L957)               §5(b) in-search cooperative fusion
```

---

## 8. Notable deviations / approximations (code vs. paper)

1. **Process noise is purely distance-proportional.** The paper's $Q$ has a constant additive
   term $\alpha_{\parallel/\perp}\Delta t$; the code uses only $(\texttt{*_PER_METER}\cdot\text{seg})^2$
   (L245–246). At constant speed the $\sigma^2[k]$ term is captured; the $\alpha\Delta t$ floor is
   dropped.

2. **Update is in information form, not literal Joseph form.** Algebraically identical for
   $H = I$ (§3), but the code never forms $K_i$ explicitly. Numerically equivalent up to the
   degeneracy guards in `kalman_info_update`.

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->
3. **Two comm models.** The exact evaluator fuses at fixed 100 m checkpoints with taper
   `COMM_SIGMA = 50` (§5a); the in-search approximation fuses within a 5 m arc window with taper
   `COMM_RADIUS = 300` (§5b). Search covariance is therefore an estimate — it is recomputed
   exactly via `evaluate_full_paths` whenever a candidate is actually scored. Both models now
   carry the post-fusion covariance forward to subsequent propagation steps (the exact
   evaluator previously froze all waypoints in a first pass and only patched comm-checkpoint
   indices in a second pass, discarding the benefit of earlier comm events).

4. **Tapered Gaussian sensing serves double duty.** The paper presents $w_{ij}$ for inter-agent
   sensing; the code reuses the same form for agent→landmark detection (`p_detect`, as a
   $1/p$ noise inflation) and for agent→agent comm weighting.

5. **Dominance uses the PSD partial order**, which is stronger than (and implies) the scalar
   $\det$ criterion written in the paper §IV.
