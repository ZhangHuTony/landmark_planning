#!/usr/bin/env python3
"""Extract the HoloOcean closed-loop data the video replays.

Two products, from two different sources, and the distinction matters:

  holo_video.npz   the ONE trial that was rendered to video
                   (results/2026-09-02_thr1.8b/video/). Its run_log.npz is the
                   log of that exact render, so mp4 frame n == tick n == row n
                   of auv0_track. Anything frame-locked to the footage must come
                   from here, NOT from mc0: mc0 is the same seed but was re-flown
                   after the 2026-09-09 estimator fix and differs slightly.

  holo_mc.json     the 30-trial Monte Carlo set (mc0..mc29), post-fix, which is
                   what Fig. 8 and Table IV report.

Only slicing and array bookkeeping happens here. The covariances are the
estimator's own world-frame marginals; the planar block is sigma[3:5, 3:5]
because a GTSAM Pose3 marginal is ordered [Rx Ry Rz Tx Ty Tz].
"""
import json
import re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SET = ROOT / "results" / "2026-09-02_thr1.8b"
OUT = ROOT / "video" / "data" / "holo"
TICKS_PER_SEC = 30          # main.py: FossenInterface takes its dt from this
NODE_STRIDE = 1
TRACK_STRIDE = 10           # 15292 ticks -> ~1530 points, plenty for a trail


def planar(sigma):
    """The xy block of a Pose3 marginal ordered [Rx Ry Rz Tx Ty Tz]."""
    return sigma[:, 3:5, 3:5]


def export_video_run():
    z = np.load(SET / "video" / "run_log.npz")
    out = {}
    for name in ("auv0", "auv1"):
        out[f"{name}_track"] = z[f"{name}_track"][::TRACK_STRIDE].astype(np.float32)
        out[f"{name}_ref"] = z[f"{name}_ref"]
        for k in ("node_tick", "node_arc", "node_unc", "node_est_err"):
            out[f"{name}_{k}"] = z[f"{name}_{k}"][::NODE_STRIDE]
        out[f"{name}_node_est"] = z[f"{name}_node_est"][::NODE_STRIDE, :2]
        out[f"{name}_node_truth"] = z[f"{name}_node_truth"][::NODE_STRIDE, :2]
        out[f"{name}_node_cov"] = planar(z[f"{name}_node_sigma"])[::NODE_STRIDE]

    for k in ("comm_events", "landmark_events", "obstacle0_verts", "landmarks",
              "landmark_cov", "goal_xy", "unc_threshold", "planner_unc",
              "agent_radius", "n_obstacles"):
        out[k] = z[k]
    out["n_ticks"] = np.int64(len(z["auv0_track"]))
    out["ticks_per_sec"] = np.int64(TICKS_PER_SEC)

    # Comm ATTEMPTS, including the ones that never fired. The recorder only
    # stores successes, so the failures -- the whole point of the behind-wall
    # scenario -- would be invisible. An attempt happens every 100 m of the
    # SUPPORT's arc; the tick is looked up in its node table. No model is
    # evaluated here, only geometry that is already in the log.
    arc, tick = z["auv1_node_arc"], z["auv1_node_tick"]
    t0, t1 = z["auv0_track"], z["auv1_track"]
    attempts = []
    fired = {int(round(c[5])) for c in z["comm_events"]}
    for k in range(1, int(arc[-1] // 100) + 1):
        i = int(np.searchsorted(arc, k * 100.0))
        if i >= len(tick):
            break
        tk = int(tick[i])
        j = min(tk, len(t0) - 1)
        d = float(np.hypot(t0[j, 1] - t1[j, 1], t0[j, 2] - t1[j, 2]))
        ok = any(abs(tk - f) <= 200 for f in fired)   # nodes are ~70 ticks apart
        attempts.append({"tick": tk, "arc": float(k * 100.0), "d": d, "fired": bool(ok)})
    out["attempt_tick"] = np.array([a["tick"] for a in attempts], dtype=np.int64)
    out["attempt_d"] = np.array([a["d"] for a in attempts], dtype=np.float64)
    out["attempt_fired"] = np.array([a["fired"] for a in attempts], dtype=bool)

    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / "holo_video.npz", **out)
    print(f"holo_video.npz  ticks={out['n_ticks']} "
          f"({out['n_ticks']/TICKS_PER_SEC:.1f} s sim, "
          f"{out['n_ticks']/TICKS_PER_SEC/30:.1f} s at x30)")
    print(f"  comm fired at ticks {sorted(fired)}")
    print(f"  {sum(a['fired'] for a in attempts)}/{len(attempts)} attempts fired; "
          f"separation {min(a['d'] for a in attempts):.0f}-{max(a['d'] for a in attempts):.0f} m")
    print(f"  sigma peak {z['auv0_node_unc'].max():.3f} m against bound "
          f"{float(z['unc_threshold']):.2f}")
    return out


def export_mc():
    trials = sorted((p for p in SET.glob("mc*") if re.fullmatch(r"mc\d+", p.name)),
                    key=lambda p: int(p.name[2:]))
    tracks, terminal = [], []
    for t in trials:
        z = np.load(t / "run_log.npz")
        tr = z["auv0_node_truth"][:, :2]
        tracks.append(tr.tolist())
        terminal.append({
            "trial": t.name,
            "err": (z["auv0_node_est"][-1, :2] - z["auv0_node_truth"][-1, :2]).tolist(),
            "unc": float(z["auv0_node_unc"][-1]),
        })
    z0 = np.load(trials[0] / "run_log.npz")
    doc = {
        "n": len(trials),
        "threshold": float(z0["unc_threshold"]),
        "predicted": float(z0["planner_unc"]),
        "tracks": tracks,
        "support_track": np.load(trials[0] / "run_log.npz")["auv1_node_truth"][:, :2].tolist(),
        "terminal": terminal,
        "obstacle": z0["obstacle0_verts"].tolist(),
        "landmark": z0["landmarks"].tolist(),
        "goal": z0["goal_xy"].tolist(),
        "ref0": z0["auv0_ref"].tolist(),
        "ref1": z0["auv1_ref"].tolist(),
    }
    (OUT / "holo_mc.json").write_text(json.dumps(doc))
    met = sum(1 for t in terminal if t["unc"] <= doc["threshold"])
    print(f"holo_mc.json    {len(trials)} trials, terminal bound met {met}/{len(trials)}")
    return doc


if __name__ == "__main__":
    export_video_run()
    export_mc()
