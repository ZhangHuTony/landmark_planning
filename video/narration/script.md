# Narration

About 420 words, one block per scene. Record against the picture lock and save
as `narration/voice.wav`; `ffmpeg/concat_master.sh` picks it up automatically.
Captions are burned in by the scenes themselves, so the video is watchable
silent and the wording below should match them closely.

Nothing here states a number that is not in `video/data/`. If a scene's numbers
change, re-run `python/export_sweep.py` and re-read this script.

---

## 1. Problem (0:00-0:12)

Underwater there is no GPS. An agent navigating on dead reckoning drifts, and
its uncertainty grows the whole way. It reaches the goal, but its estimate of
where it is has grown past the point where arriving is useful.

## 2. Escort (0:12-0:40)

We give it a support agent. The support has no goal of its own. It is routed
only to keep the primary under a terminal uncertainty bound. Both are searched
on one heading-aware hexagonal lattice. Here the support dives to a landmark
the primary never sees, fixes its own position, and carries that fix back. For
eight hundred metres the two are out of communication range, and the primary
runs on dead reckoning alone. One exchange near the goal is enough to bring it
back under the bound. We optimise the mean paths of both agents and propagate
covariance only to check the constraint.

## 3. Method (0:40-1:15)

The search runs over the joint state of every agent at once, so it decides
which landmark to use and when to relay as one decision. The route it returns
runs through cell centres. Those become the control points of a clamped cubic
B-spline, and the refinement shortens the primary's path while holding the
uncertainty and curvature constraints. Across the benchmark that recovers two
to seven per cent. Tightening the bound changes the shape of the plan, and it
is the primary that pays: on this scenario its path grows from seventeen
hundred and forty metres to eighteen hundred and ninety.

## 4. Baselines (1:15-1:45)

Each baseline keeps our covariance model, our obstacle test and our refinement,
and removes one piece of joint routing. Without lookahead, the support never
reaches a landmark. Locked into a formation, it cannot leave, so the primary
detours itself. Planned in sequence, the primary commits before the support
exists. Sampling the joint space returns the first path it finds, which is
long.

## 5. Results (1:45-2:15)

Across fifty randomised scenarios we sweep the bound from the reference value
down to thirty per cent of it. At half the reference bound we solve eighty per
cent of scenarios against fifty-four for the strongest baseline, at comparable
path length. Where a planner solves almost nothing, its average path length
stops meaning anything, so those marks fade and hollow out.

## 6. Simulation (2:15-2:50)

Every planner here shares one belief model, so the benchmark cannot tell us
whether that model is calibrated. So we fly the plan in a marine simulator with
vehicle dynamics, a controller, and an independently developed factor-graph
estimator, and the vehicle steers on its own estimate. Measurements are ground
truth plus Gaussian noise. Watch the support pass behind the wall: the packets
stop, the primary's uncertainty climbs over the bound, and the first exchange
after it rejoins pulls it back. Over thirty runs there were no collisions, and
the terminal uncertainty we predicted lands within six per cent of the spread
we actually flew.

## 7. Close (2:50-3:00)

Eighty per cent against fifty-four. No collisions in thirty closed-loop runs.
And a belief model that the simulator says we can trust.
