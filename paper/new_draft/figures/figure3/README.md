# Fig. 3 — iteration history

Every version of Fig. 3 we rendered, in order. Numbered
files are the ones that shipped (commit given); `x*` are explorations that never
did. `15a_`/`15b_` are what is currently in the paper.

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
| `10_discrete-glyphs_texture-lines.png` | `1e027ba2` | **Scale moves off the line and into the marker fills, in eight discrete swatches** — one per sweep level, so a fill can be matched back to the key by eye. That frees the line for identity, and plain lines can be dashed where a gradient line could not: shape + dash carry the planner, no second color channel. Ours keeps the coral, now on its line and marker *ring*. |
| `11_transposed_accent-on-line.png` | `39bcf195` | **Axes transposed** — success rate on y, length ratio on x, so the best corner is top left. Length takes x because that is where the data needs the room. The coral accent comes off the marker rings and goes on Ours' line alone, so every glyph keeps the same dark ring and nothing competes with the fills. Line gray darkened 0.55 → 0.35. |
| `12_ylgnbu_dotted-baselines_success-on-y.png` | `ea1fd30d` | Scale switched to **YlGnBu**, dark = the higher constraint number. All four baselines collapse to one **dotted gray**, so they read as a single background population and identity falls entirely to marker shape; Ours becomes **solid black**. Also fixes the reversed key, below. `12b_` is the same figure with the axes swapped — both are generated. |
| `13_gold-diamond_side-key_row-legend.png` | `2159e951` | **Back to the `08_` layout** — constraint level on x, success rate on y, median length ratio painted along the line. Ours' fill goes coral → gold `#c68a00`, the ramp is trimmed a little greener (0.74 → 0.80), the length key stands vertically at the right, and the planner key is one horizontal row under the plot. `12_`/`12b_` stay generated as spares. |
| `14_yellow-diamond_keys-stacked-below.png` | `d5e904d1` | Undoes `13_`'s squish. The side key cost the plot 0.4 in of width, so the length key goes back under the x label and the plot returns to `08_`'s exact 3.073 × 1.696 in; the figure grows to 2.75 in tall instead. Ours' fill goes gold → plain yellow `#ffdd00`. |
| `15a_split_success-rate.png`, `15b_split_length_mean-sd.png` | current | **Split into two plots** (user call, 2026-09-10): constraint level on x for both, success rate on y in (a), **mean ± SD of length / $L_\mathrm{ref}$ over the solved scenarios** on y in (b), planners dodged along x so the bars stay apart. No ramp, no colorbar: identity is back to the paper palette + marker shape, Ours the heavier line drawn on top. (a) carries no error bar on purpose (success rate is a proportion; the user declined a binomial SE). (b)'s y stops at 3.2 so the 1.1–1.5 band where most of the data lives keeps its room; Sequential's 2.49±1.37 / 2.76±1.51 at 40/30% run off the top, and the caption says so. Stems `fig3a_success_vs_constraint`, `fig3b_length_vs_constraint`; `fig3_length_vs_constraint` and `fig3_success_on_*` are deleted (recover from `d5e904d1`). |

## Alternates worth keeping

| file | why it is here |
|---|---|
| `06b_plasma-r_palette-glyphs.png` (plasma-era) | Same as `06_` but glyphs take each planner's palette color (white halo to separate them from the line). Restores identity consistency with Figs. 4–6 — at the cost of CL-GBT's yellow and Formation's orange landing *inside* the plasma ramp, where they read as values rather than labels. |
| `x1_plasma-r_untrimmed.png` | Full plasma. Shows why it is trimmed: the short-path end is a near-white yellow that a 1.4 pt line cannot carry on white paper. |
| `x2_plasma-r_charcoal-glyphs.png` | Dark glyph fill instead of light. Disappears into the violet end of the ramp. |
| `x7_shared-viridis_gradient-lines_colored-glyphs.png` | Ramp on both lines *and* marker fills. Redundant, and crossings are hard to trace. |
| `x17_transposed_x-inverted.png` | `11_` with x inverted, to put the best corner at top right where convention wants it. It jams the 1.1–1.4 cluster — four of five planners — into the right edge, under the key. Best-at-top-left wins on legibility. |
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

## The colormap sweep (`11_`)

`cm_swatch-comparison.png` is the thirteen candidates as eight-swatch strips;
`cm_*.png` are the full figures for the ones worth seeing. Every scale was
scored two ways with a CVD simulation (Machado 2009, severity 1.0, ΔE in OKLab
×100, worst case over normal / protanopia / deuteranopia):

| scale | min pairwise ΔE | ΔE to the coral accent |
|---|---|---|
| cividis | **8.9** | 8.4 |
| magma | 7.5 | 6.1 |
| cubehelix | 7.3 | 2.5 |
| viridis, full | 6.7 | 8.5 |
| **viridis, trimmed — shipped** | 6.6 | 8.5 |
| inferno | 6.0 | 4.7 |
| PuBuGn | 5.8 | 7.0 |
| Blues | 5.0 | 19.0 |
| YlOrRd | 5.0 | 3.0 |
| YlGnBu | 4.5 | 17.4 |
| GnBu | 3.8 | 18.9 |
| plasma | 3.3 | 6.9 |
| turbo | 2.5 | 2.3 |

Reading that table:

- **turbo and plasma are out on the numbers.** Turbo's eight steps collapse to
  ΔE 2.5 under deuteranopia — a rainbow is the worst possible choice for an
  ordered variable seen by a CVD reader.
- **magma, inferno and cubehelix are out on the accent column.** They are warm
  where the coral is warm, so Ours' line reads as one more swatch of the scale.
  You can see it in `cm_magma.png` and `cm_inferno.png`.
- **The ColorBrewer ramps (YlGnBu, GnBu, PuBuGn, Blues) are out on separation.**
  One or two hues over eight steps leaves lightness to do all the work.
- **cividis is the measured winner and still was not shipped.** Its middle
  swatches are literally gray (`#8e8978`, `#6c6e72`), so the 60–70% glyphs read
  as unfilled rather than as a value, and it has no hue variety to help. See
  `cm_cividis.png` and judge for yourself — it is a one-word change.
- **viridis, full range** (`cm_viridis_full.png`) scores a hair above the
  trimmed version because it spans further. The original reason for the trim —
  a 1.4 pt line cannot carry pale yellow — stopped applying at `10_`, when the
  scale moved into filled glyphs. Kept trimmed anyway, per the standing "no
  yellow" call; the full range is there if that call changes.

## Colormaps tried for the constraint level (`09_`)

The ramp changed meaning in `09_` — it is the sweep variable now, not the cost —
so the candidates were re-run against the new plot. Viridis (trimmed at 0.74,
green = loose, dark violet = tight) still wins.

| file | why not |
|---|---|
| `x9_axes-swapped_cividis.png` | CVD-optimal by construction, and it still fails here: its midtones are the same gray as the `#dfddd6` marker fill, so the middle levels read as "not highlighted" rather than as a value. Its yellow end is also too pale for a 1.4 pt line. |
| `x10_axes-swapped_magma.png` | The warm end collides with the coral `#e8503a` primary glyph — the one fill chosen specifically to sit *outside* the ramp. Inferno fails the same way. |
| `x11_axes-swapped_blues.png` | Single hue, so lightness carries the whole scale; the loose end goes so faint that Greedy's and Sequential's 100% segments nearly vanish. |

## The side key does not fit a single column

`13_` stood the length key in the right margin and the plot lost 0.4 in of
width for it. At 3.5 in there is no room: the y label takes ~0.38 in and the
bar plus its ticks and rotated label another ~0.45 in, leaving the plot 2.67 in
against the 3.07 in it has in `08_`. Nor can the figure simply be drawn wider —
`\includegraphics[width=\linewidth]` scales it straight back down and shrinks
every label with it. `14_` puts the key back under the x label and stacks the
planner key beneath that.

## Yellow beats gold here — lightness is what separates

`13_`'s brief was Ours in gold against a greener ramp, and the golds barely
survived it. Plain yellow, asked for next, does better — the ramp's short end
is a **mid-lightness** green (`#7ad151`), so separation from it is mostly a
question of lightness, and the golds sit at exactly the wrong one:

| fill | ΔE to ramp | contrast vs. white |
|---|---|---|
| `#ffff00` pure yellow | 14.9 | 1.07 |
| `#ffe600` | 10.2 | 1.27 |
| **`#ffdd00` — shipped** | **8.0** | **1.35** |
| `#ffe14d` | 8.6 | 1.30 |
| `#c68a00` gold (`13_`) | 7.9 | 2.98 |
| `#e8503a` coral (`08_`) | 7.9 | 3.72 |
| `#f2d024` | 4.4 | 1.52 |
| `#ffc300` | 2.5 | 1.61 |

`#ffdd00` clears the ramp better than either colour it replaced. Its 1.35:1
against white looks alarming and is not, for this mark: the glyph has a 0.15
ring, and on a yellow fill the edge rather than the fill is what holds the
shape. That would not hold for a yellow *line*, which has no edge — the reason
the ramp itself still stops short of yellow.

## Gold fights a greener ramp — pick the deep one

`13_` asked for two things that pull against each other: Ours in gold, and the
ramp shifted toward green. The greener the ramp's short end, the closer it comes
to gold, and the brighter golds lose outright (worst-case CVD ΔE to the ramp,
over normal/protan/deutan, at trim 0.80):

| gold | ΔE to ramp | contrast vs. white |
|---|---|---|
| `#ffc300` | 1.6 | 1.61 |
| `#e8a33d` | 2.1 | 2.16 |
| `#f2b705` | 3.4 | 1.82 |
| `#daa520` goldenrod | 3.8 | 2.24 |
| `#d99b00` | 5.3 | 2.43 |
| `#cf9200` | 6.7 | 2.24 |
| **`#c68a00` — shipped** | **7.9** | **2.98** |
| `#b8860b` darkgoldenrod | 8.8 | 3.25 |
| `#e8503a` coral, for reference | 7.9 | 3.72 |

`#c68a00` matches the coral it replaced exactly on separation (7.9) and is still
unambiguously gold rather than brown. The scores plateau for the deep golds,
because their nearest neighbour on the ramp is no longer the green end — which
is why the ramp could be pushed greener without costing anything here.

Trim `0.80` puts `#7ad151` at the short-path end, against `#58c765` at `0.74`:
a visible step toward green, still short of yellow.

## The key was reversed for three commits — check it if you touch it

`level_bar` draws the eight swatches with `pcolormesh(LVL_BOUNDS, [0,1], C)`.
`LVL_BOUNDS` ascends (25, 35, … 105), so **`C` has to ascend too**. It was
being handed `LVL_PCTS`, which runs 100 → 30, and the axis was then reversed on
top of that — a double flip that cancelled against the ticks but not against the
blocks. The swatch printed under the "100" label was painted with level 30's
color, and vice versa, for the whole of `09_`–`11_`.

Nothing about the data was wrong; every *reading* of those three figures was.
`level_bar` now sorts explicitly and the axis runs 30 → 100 left to right, so
the largest number is at the right end where a reader expects it.

If you change either the scale direction or the bar, verify with:

```python
import matplotlib.colors as mc, make_figs_baseline as M
[mc.to_hex(M.LVL_CMAP(M.LVL_NORM(p))) for p in M.LVL_PCTS]  # glyph colors
```

and check that the swatch under each tick matches the glyph color for that
level.

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

One thing about the SVG, deliberate:

- Text is text, not outlines (`svg.fonttype: none`), so labels stay editable.
  Illustrator substitutes only if Nimbus Roman is missing there; the
  font-family falls back through Times New Roman and Liberation Serif.
  (If a colorbar ever comes back, draw it with `pcolormesh`, not `imshow` —
  `imshow` embeds a raster block that arrives resolution-locked.)

And two about the EPS:

- It uses **Type 3** fonts while the PDF uses Type 42. Matplotlib's Type 42
  embedding of Nimbus Roman writes a font Ghostscript rejects outright
  (`invalidfont in definefont`) and the file will not open at all. Type 3
  renders correctly, but text arrives as outlines — hence "prefer the SVG".
  Only the PDF goes into the paper, and it keeps Type 42.
- PostScript has no transparency, so Fig. 4's box fills are **pre-blended onto
  white** (`over_white`) rather than given an alpha. Do not reintroduce
  `set_alpha` there; it comes out opaque in the EPS and stops matching the PDF.

Since `15_` every line is a single path again (the gradient lines of
`05_`–`14_` were 32 stroked segments per data interval and had to be grouped
before moving).

Regenerate everything with:

    ~/Research/multiagent_base/.venv/bin/python ../make_figs_baseline.py
