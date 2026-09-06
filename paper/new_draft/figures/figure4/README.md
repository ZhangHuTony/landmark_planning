# Fig. 4 — iteration history

Every version of the wall-clock figure we rendered, in order. `01_` is what is
currently in the paper. All are 300 dpi previews — judge them at print size, not
zoomed: `01_` and `03_` are 3.5 in (one column), `02_` and `04_` are 7.16 in
(full text width, so they would need `figure*`).

| file | what it is |
|---|---|
| `00_pooled_box.png` | The original. One box per planner, **all eight constraint levels pooled**. |
| `01_5level_box.png` | Split by level, five levels (100/80/60/40/30), boxes. Column width. **In the paper.** |
| `02_alllevel_box.png` | All eight levels, boxes. Full text width. |
| `03_5level_violin.png` | Five levels, violins. Column width. |
| `04_alllevel_violin.png` | All eight levels, violins. Full text width. |

## What the split is for

Pooling averaged away the one interesting thing in this figure. Four of the five
planners cost essentially the same however tight the bound gets — their boxes
barely move across the levels — while CL-GBT's median roughly triples and its
tail runs to 730 s. In `00_` that shows up only as one planner having a longer
whisker, which reads as noise rather than as a trend.

## Three things to know before editing

**The tight levels barely have data.** Success rates collapse as the bound
tightens: at 30% of $U_\mathrm{ref}$, Greedy solves 2 of 50 scenarios and CL-GBT
3. A box drawn over two runs is a line pretending to be a distribution, so
anything under `MIN_BOX` (5) or `MIN_VIOLIN` (10) is drawn as its **individual
runs instead**, one tick per successful trial. Those little stacks of ticks at
the right-hand end of the plot are not a rendering fault.

Violins get the higher threshold on purpose: a box over five points still
reports real order statistics, but a KDE over five points is mostly kernel.

**The violins are computed on `log10(wall)`, against a linear axis whose ticks
are relabelled in seconds.** They are not `set_yscale("log")`. This matters: a
KDE has to be estimated in the space the reader sees it in, and estimating in
linear space then displaying on a log axis turns every one of these
distributions into a spike at the bottom of the panel. Boxes do not have this
problem — quartiles are order statistics and survive any monotone transform —
so `01_`/`02_` do use a real log axis. The two look identical because the ticks
are chosen to match; they are drawn differently underneath.

**`violinplot` bodies default to `alpha=0.3`.** They are set back to 1.0 and
pre-blended with `over_white` instead, for the same reason as Fig. 4's old
boxes: the PostScript backend has no transparency, so an alpha'd body comes out
opaque in the EPS and stops matching the PDF.

## Which one to ship

`01_` fits a column and drops into the existing `figure` environment. The
eight-level versions need `figure*` and a full-width slot, which costs
placement freedom for three extra levels that mostly repeat what the five
already show. The violins carry more information per panel — you can see that
Ours' distribution is tight with a thin upper tail, where the box only says
"median 18, some outliers" — but at column width the bodies are ~5 pt wide and
the shape is hard to read.
