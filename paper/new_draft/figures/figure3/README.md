# Fig. 3 — iteration history

Every version of `fig3_length_vs_constraint` we rendered, in order. Numbered
files are the ones that shipped (commit given); `x*` are explorations that never
did. `06_` is what is currently in the paper.

All are 300 dpi previews of a 3.5 in column figure — judge them at that size,
not zoomed.

## Shipped

| file | commit | what changed |
|---|---|---|
| `00_opacity_length-on-y.png` | `6252212f` | Original. Length ratio on y, success rate as line/marker **opacity**. Greedy's 4% line at alpha 0.15 is barely printable. |
| `01_success-on-y_ramp-same-hue.png` | `3947227f` | Axes swapped: success rate on y, length ratio into color. One ramp per planner, its palette hue → a dark shade of the same hue. |
| `02_ramp-cross-hue-41deg.png` | `3b612634` | Same, but each ramp ends on a *different* hue (~41°). Same-hue lightness alone was too quiet on a 1 pt line. |
| `03_ramp-cross-hue-156deg.png` | `ae1249b6` | Rotations widened to ~156°, searched rather than chosen. |
| `04_shared-viridis_on-markers_dashed-lines.png` | `1a8991b3` | Per-planner ramps abandoned. **One** shared ramp on the marker fills; identity by marker shape + dash; lines neutral gray. |
| `05_shared-viridis_on-lines.png` | `39b17928` | Ramp moved from the markers to the **lines** (a 3 pt glyph is mostly outline). Dashes lost — see note below. |
| `06_plasma-r_on-lines_neutral-glyphs.png` | current | Plasma reversed and trimmed, so **dark = long detour**. Larger glyphs, single neutral fill `#dfddd6`. |

## Alternates worth keeping

| file | why it is here |
|---|---|
| `06b_plasma-r_palette-glyphs.png` | Same as `06_` but glyphs take each planner's palette color (white halo to separate them from the line). Restores identity consistency with Figs. 4–6 — at the cost of CL-GBT's yellow and Formation's orange landing *inside* the plasma ramp, where they read as values rather than labels. |
| `x1_plasma-r_untrimmed.png` | Full plasma. Shows why it is trimmed: the short-path end is a near-white yellow that a 1.4 pt line cannot carry on white paper. |
| `x2_plasma-r_charcoal-glyphs.png` | Dark glyph fill instead of light. Disappears into the violet end of the ramp. |
| `x7_shared-viridis_gradient-lines_colored-glyphs.png` | Ramp on both lines *and* marker fills. Redundant, and crossings are hard to trace. |

## Rejected, with the reason

| file | why not |
|---|---|
| `x3_rejected_viridis-magma-pairs-per-method.png` | The viridis / magma / cool-tech / monochromatic pairs, one per planner. Ours (`#00429D`) and Greedy (`#08306B`) are the same navy exactly where most of the data sits. |
| `x4_rejected_same-pairs-reversed.png` | Same pairs, light end first. Greedy becomes an invisible ice blue. Both directions fail: those ramps are built for filled areas, where a very light endpoint is fine. |
| `x5_rejected_wide-rotation-collides.png` | Per-planner rotations pushed as far as they go. Formation and CL-GBT both arrive at magenta. |
| `x6_rejected_shared-viridis-no-identity.png` | One shared ramp, identity by marker only, no dashes and no line-weight hierarchy. The whole left half is dark purple, because every planner sits at 1.1–1.5 there. |
| `x8_rejected_two-color-channels-per-glyph.png` | Palette-colored line + palette-colored marker ring + ramp marker fill. Two color channels on a ~3 pt glyph; illegible at column width. |

## Two things that cost time — don't rediscover them

**A gradient line cannot be dashed.** It is a `LineCollection` of 32 sub-segments
per interval, each shorter than a dash period, so `set_linestyle` renders solid.
Dash patterns and a gradient line are mutually exclusive; every variant picks one.
That is why `04_` has dashes and `05_`/`06_` do not (Ours is held apart by line
weight instead, 2.2 pt vs 1.4).

**Per-planner ramps have a hard ceiling.** 25 colors on a 5-hue budget are not
mutually distinguishable at any rotation — best worst-pair CVD ΔE 3.1, measured
over the full rotation space with the dataviz validator. That ceiling is what
drove the move to a single shared ramp, which has no cross-planner pairs to
separate at all.

Regenerate the current figure with:

    ~/Research/multiagent_base/.venv/bin/python ../make_figs_baseline.py
