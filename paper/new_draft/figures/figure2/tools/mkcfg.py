#!/usr/bin/env python3
"""mkcfg.py <scenario_file> <out_cfg_dir> key=value ...
Copies config/mc/*.yaml (minus sweep_*), sets landmark_scenario: manual, appends the
scenario file's start/goal/landmarks/obstacles lines, and patches key=value overrides
in whichever file defines the key (appending to main.yaml otherwise) — same rule as
run_constraint_sweep.jl's write_run_config."""
import sys, os, re
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../.."))
MC = os.path.join(ROOT, "config", "mc")
scen, out = sys.argv[1], sys.argv[2]
ov = dict(kv.split("=", 1) for kv in sys.argv[3:])
ov.setdefault("landmark_scenario", "manual")
os.makedirs(out, exist_ok=True)
files = sorted(f for f in os.listdir(MC) if f.endswith(".yaml") and not f.startswith("sweep"))
contents = {f: open(os.path.join(MC, f)).read().splitlines() for f in files}
for k, v in ov.items():
    pat = re.compile(r"^\s*" + re.escape(k) + r"\s*:")
    placed = False
    for f in files:
        for i, ln in enumerate(contents[f]):
            if ln.strip().startswith("#") or not pat.match(ln):
                continue
            contents[f][i] = f"{k}: {v}"; placed = True
    if not placed:
        contents["main.yaml"].append(f"{k}: {v}")
scen_lines = [l for l in open(scen).read().splitlines() if l.strip() and not l.strip().startswith("#")]
contents["main.yaml"] += ["# ── manual scenario (fig2 probe) ──"] + scen_lines
for f in files:
    open(os.path.join(out, f), "w").write("\n".join(contents[f]) + "\n")
print(out)
