# Supplementary video

A ~3 minute video for the ICRA submission, built with Manim. Two deliverables
from one timeline:

| | for | limits |
|---|---|---|
| `deliverables/consort_master_1080p.mp4` | the anonymised project page | none |
| `deliverables/ICRA2027_<id>.mp4` | the PaperPlaza attachment | ≤ 180 s, ≤ 20 MB, mp4/H.264, ≥ 480 px, ≥ 20 fps, progressive, no names/logos/URLs |

The attachment limits are from the ICRA 2027 call for papers (checked
2026-09-20). `ffmpeg/check_deliverable.sh` asserts every one of them.

## The rule this pipeline is built around

**Julia computes, Manim replays.** No covariance, path length, spline sample or
comm weight is ever recomputed in Python. Each exporter re-derives its numbers
with the planner's own functions and then refuses to write unless they
reproduce the run's `results.yaml`, so a scene cannot quietly drift from the
paper. The same applies to the simulator side: `export_holo.py` only slices
arrays out of the recorded run logs.

Verified reproductions, all exact:

| run | check |
|---|---|
| Fig. 1 `D_island/tight` | σ_T 1.798839 = `results.yaml`; seed re-run gives 1748 expansions, σ 1.9201, 1200 m |
| Fig. 2 ladder ×4 | 1741 / 1761 / 1822 / 1890 m and σ 9.73 / 8.89 / 6.66 / 3.91, matching Table I |
| benchmark s049 @ 50% ×5 | every `primary_length` byte-identical to `constraint_sweep/baseline_2026-08-17/trials_all.csv` |
| Monte Carlo n=30 | 0/30 collisions, 28/30 under bound, predicted 1.796 vs sample 1.702 |

## Environment

`manimpango` publishes **no Linux wheel** (macOS and Windows only, plus an
sdist), this box has the pango/cairo runtime libraries but not the `-dev`
headers, and there is no sudo to install them. Rather than give up on Manim,
`env.sh` points pkg-config and the linker at **Julia's own JLL artifacts** under
`~/.julia/artifacts`, which ship relocatable headers, `.pc` files and shared
libraries (that is how Julia gets a working Cairo with no root either). Two
small gaps are bridged in `.pcshim/`; both are documented in `env.sh`.

```bash
source video/env.sh          # required before pip, manim, or any scene render
manim checkhealth            # all four checks should pass
```

Recreating the venv from scratch: `python3 -m venv video/.venv && source
video/env.sh && pip install -r video/requirements.txt`.

## Pipeline

```bash
source video/env.sh

# 1. data  (Julia re-derives; Python only slices the sim logs)
julia video/julia/export_scene.jl paper/new_draft/figures/figure1/D_island/tight \
      --alone --out=video/data/fig1/scene.json
julia video/julia/export_seed.jl  paper/new_draft/figures/figure1/D_island/tight \
      --out=video/data/fig1/seed.json
video/julia/run_ladder_exports.sh                 # Fig. 2 rungs 100/70/50/30
SID=s049 video/julia/run_baselines.sh             # re-run 5 planners WITH geometry
SID=s049 video/julia/run_greedy_fallback.sh       # the rung the sweep skipped
SID=s049 video/julia/export_baselines.sh
video/.venv/bin/python video/python/export_sweep.py   # aggregates + headline numbers
video/.venv/bin/python video/python/export_holo.py    # simulator replay + 30 trials

# 2. render  (-ql while iterating, -qh for the master)
cd video/manim && manim -qh scenes/s02_escort.py Escort

# 3. assemble
video/ffmpeg/decimate_closeup.sh      # footage x30, 510 frames
video/ffmpeg/composite_holo.sh        # footage into the slot HoloRun leaves
video/ffmpeg/concat_master.sh         # -> deliverables/consort_master_1080p.mp4
ID=1234 video/ffmpeg/encode_icra.sh   # -> deliverables/ICRA2027_1234.mp4 + checks
```

## Scenes

| file | class | shows | source |
|---|---|---|---|
| `s01_problem.py` | `Problem` | one AUV alone, σ 2.37 > ū 2.0 | `fig1/scene.json` `alone` block |
| `s02_escort.py` | `Escort` | Fig. 1 animated: lattice, divert, blackout, one fusion | `fig1/scene.json` |
| `s03a_astar.py` | — | **not built**: needs the A* expansion tap | — |
| `s03b_refine.py` | `Refine` | lattice seed 1200 m → spline 1100 m | `fig1/seed.json` + `scene.json` |
| `s03c_ladder.py` | `Ladder` | ū 100%→30%, path 1741→1890 m | `ladder/scene_p*.json` |
| `s04_baselines.py` | `Baselines` | what each baseline removes, one scenario | `baselines/s049_p050/*.json` |
| `s05_results.py` | `Results` | success vs length, swept over the ladder | `sweep/levels.json` |
| `s06_holo.py` | `HoloRun` | synced replay, footage slot on the left | `holo/holo_video.npz` |
| `s06_holo.py` | `HoloMC` | 30 runs, predicted vs flown terminal ellipse | `holo/holo_mc.json` |
| `s07_close.py` | `Close` | the three closing numbers | `holo/numbers.json` |

## How scene 6 stays in sync

`closeup_auv0.mp4` was encoded 1:1 from `frames_auv0/<tick>.bmp`, which the
simulator writes in the same loop iteration that records a tick. So:

```
footage frame n  ==  tick n  ==  row n of auv0_track
```

Keeping every 30th frame gives ×30, and `HoloRun` advances 900 ticks per second
of video, so **Manim second s ↔ tick 900s ↔ output frame 30s**. The panel runs
17.0 s and the decimated footage is exactly 510 frames.

One trap: that footage is a *separate render* (seed 1000, made 2026-09-02,
before the estimator fix), **not** `mc0`. Anything frame-locked must read
`video/run_log.npz`; the statistics panels read `mc0..mc29`.

## Still open

- **Scenes 3a and 3b are limited by what the planner logs.** Expansion order and
  the optimizer's iterates exist only inside `joint_astar` and
  `optimize_continuous` and are never returned, so `s03a_astar.py` does not
  exist and `s03b_refine.py` tweens seed → result rather than replaying Adam
  (it says so on screen). Both need the flag-gated logging taps described in the
  plan, which touch planner files and are pending approval.
- The AUV glyph is drawn from primitives; the shipped Fig. 1 icon is a bitmap
  embedded in an Illustrator PDF with no vector source in the repo. Drop an
  `manim/assets/auv.svg` in and swap `AUVGlyph` for an `SVGMobject` if wanted.
- Narration is a script only (`narration/script.md`); record over the picture
  lock, save as `narration/voice.wav`, and `concat_master.sh` will mux it.
- Timings are first-draft. Every scene's `run_time` is a plain number near the
  top of its play calls.
