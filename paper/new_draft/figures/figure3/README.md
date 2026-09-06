# Fig. 3 — iteration history

Every version of `fig3_length_vs_constraint` we rendered, in order. Numbered
files are the ones that shipped (commit given); `x*` are explorations that never
did. `10_` is what is currently in the paper.

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
| `06_plasma-r_on-lines_neutral-glyphs.png` | `79a02245` | Plasma reversed and trimmed, so **dark = long detour**. Larger glyphs, single neutral fill `#dfddd6`. |
| `07_viridis-trim_green-short.png` | `9d3a5301` | Back to viridis, reversed and cut at 0.74 so it never reaches yellow: **green = at the reference length**, through teal and blue, to dark violet for the worst detours. |
| `08_coral-diamond-primary.png` | `f1b76fef` | Ours takes the diamond (CL-GBT the circle it vacated), a coral `#e8503a` fill instead of the shared neutral, and its glyphs draw on top of everyone's. |
| `09_axes-swapped_constraint-on-color.png` | `f7d6d8c4` | **Both outcomes onto the axes.** Success rate on x, length ratio on y *inverted*, and the sweep variable — constraint level — into the ramp. The figure now reads as a cost/reliability trade with a best corner (top right), which the previous versions had no way to show. |
| `10_discrete-glyphs_texture-lines.png` | current | **Scale moves off the line and into the marker fills, in eight discrete swatches** — one per sweep level, so a fill can be matched back to the key by eye. That frees the line for identity, and plain lines can be dashed where a gradient line could not: shape + dash carry the planner, no second color channel. Ours keeps the coral, now on its line and marker *ring*. |

## Alternates worth keeping

| file | why it is here |
|---|---|
| `06b_plasma-r_palette-glyphs.png` (plasma-era) | Same as `06_` but glyphs take each planner's palette color (white halo to separate them from the line). Restores identity consistency with Figs. 4–6 — at the cost of CL-GBT's yellow and Formation's orange landing *inside* the plasma ramp, where they read as values rather than labels. |
| `x1_plasma-r_untrimmed.png` | Full plasma. Shows why it is trimmed: the short-path end is a near-white yellow that a 1.4 pt line cannot carry on white paper. |
| `x2_plasma-r_charcoal-glyphs.png` | Dark glyph fill instead of light. Disappears into the violet end of the ramp. |
| `x7_shared-viridis_gradient-lines_colored-glyphs.png` | Ramp on both lines *and* marker fills. Redundant, and crossings are hard to trace. |
| `x12_axes-swapped_log-y.png` | `09_` with a log y. It does spread the crowded 1.1–1.4 band, where four of five planners live — but it also flattens Sequential's and CL-GBT's blow-up to 2.0–2.4, which is half the point of the figure. Linear keeps the drama. |

## Rejected, with the reason

| file | why not |
|---|---|
| `x3_rejected_viridis-magma-pairs-per-method.png` | The viridis / magma / cool-tech / monochromatic pairs, one per planner. Ours (`#00429D`) and Greedy (`#08306B`) are the same navy exactly where most of the data sits. |
| `x4_rejected_same-pairs-reversed.png` | Same pairs, light end first. Greedy becomes an invisible ice blue. Both directions fail: those ramps are built for filled areas, where a very light endpoint is fine. |
| `x5_rejected_wide-rotation-collides.png` | Per-planner rotations pushed as far as they go. Formation and CL-GBT both arrive at magenta. |
| `x6_rejected_shared-viridis-no-identity.png` | One shared ramp, identity by marker only, no dashes and no line-weight hierarchy. The whole left half is dark purple, because every planner sits at 1.1–1.5 there. |
| `x8_rejected_two-color-channels-per-glyph.png` | Palette-colored line + palette-colored marker ring + ramp marker fill. Two color channels on a ~3 pt glyph; illegible at column width. |

## Line treatments tried for `10_`

With the scale in the glyphs, the line was free. Four treatments, same figure
otherwise:

| file | verdict |
|---|---|
| `x13_discrete-glyphs_lines-one-color.png` | All lines one gray. Cleanest possible color story — color means level, full stop — but in the top-right pileup, where four planners converge, there is nothing to trace a line by except following a glyph shape. |
| `x14_discrete-glyphs_lines-palette.png` | Lines in the paper palette. Strongest identity and consistent with Figs. 4–6, but the palette's blue (Ours) and green (Sequential) fall *inside* the viridis gamut, so two of five identity colors sit where the reader is being asked to read levels. |
| `x15_discrete-glyphs_texture-no-accent.png` | Per-planner dash, everything gray. Identity without spending any color. Ours is only "the solid, slightly heavier one". |
| `x16_discrete-glyphs_one-color-plus-accent.png` | `x13_` plus the coral accent on Ours. Ours is findable; the other four still are not separable in the pileup. |

`10_` is `x15_` plus the coral accent: dash carries the four baselines, coral
carries Ours, and the fill channel stays entirely the level scale's.

## Colormaps tried for the constraint level (`09_`)

The ramp changed meaning in `09_` — it is the sweep variable now, not the cost —
so the candidates were re-run against the new plot. Viridis (trimmed at 0.74,
green = loose, dark violet = tight) still wins.

| file | why not |
|---|---|
| `x9_axes-swapped_cividis.png` | CVD-optimal by construction, and it still fails here: its midtones are the same gray as the `#dfddd6` marker fill, so the middle levels read as "not highlighted" rather than as a value. Its yellow end is also too pale for a 1.4 pt line. |
| `x10_axes-swapped_magma.png` | The warm end collides with the coral `#e8503a` primary glyph — the one fill chosen specifically to sit *outside* the ramp. Inferno fails the same way. |
| `x11_axes-swapped_blues.png` | Single hue, so lightness carries the whole scale; the loose end goes so faint that Greedy's and Sequential's 100% segments nearly vanish. |

## Two things that cost time — don't rediscover them

**A gradient line cannot be dashed.** It is a `LineCollection` of 32 sub-segments
per interval, each shorter than a dash period, so `set_linestyle` renders solid.
Dash patterns and a gradient line are mutually exclusive; every variant picks one.
That is why `04_` has dashes and `05_`–`09_` do not (Ours is held apart by line
weight instead, 2.2 pt vs 1.4). It stopped binding at `10_`, which paints the
scale into the marker fills and leaves the lines plain — plain lines dash fine.

**Per-planner ramps have a hard ceiling.** 25 colors on a 5-hue budget are not
mutually distinguishable at any rotation — best worst-pair CVD ΔE 3.1, measured
over the full rotation space with the dataviz validator. That ceiling is what
drove the move to a single shared ramp, which has no cross-planner pairs to
separate at all.

## Output formats

`make_figs_baseline.py` writes each figure four ways: **PDF** is what LaTeX
includes, **SVG** and **EPS** both open in Illustrator, **PNG** is a preview.
Prefer the SVG for editing — it is the one that keeps live text.

Two things about the SVG, both deliberate:

- Text is text, not outlines (`svg.fonttype: none`), so labels stay editable.
  Illustrator substitutes only if Nimbus Roman is missing there; the
  font-family falls back through Times New Roman and Liberation Serif.
- The colorbar is a `pcolormesh`, not an `imshow`. `imshow` embeds the bar as a
  raster block, which arrives in Illustrator resolution-locked and uneditable.

And two about the EPS:

- It uses **Type 3** fonts while the PDF uses Type 42. Matplotlib's Type 42
  embedding of Nimbus Roman writes a font Ghostscript rejects outright
  (`invalidfont in definefont`) and the file will not open at all. Type 3
  renders correctly, but text arrives as outlines — hence "prefer the SVG".
  Only the PDF goes into the paper, and it keeps Type 42.
- PostScript has no transparency, so Fig. 4's box fills are **pre-blended onto
  white** (`over_white`) rather than given an alpha. Do not reintroduce
  `set_alpha` there; it comes out opaque in the EPS and stops matching the PDF.

The one rough edge: Fig. 3's gradient lines arrive as a run of short stroked
segments (32 per data interval), not one path each. That is what a per-vertex
color ramp has to be. Group a line's segments before moving it.

Regenerate everything with:

    ~/Research/multiagent_base/.venv/bin/python ../make_figs_baseline.py
