#!/usr/bin/env python3
"""Parallel grid runner for walksearch: every cell of a cartesian grid of
profile overrides and env knobs runs on its own core; results land in one
CSV sorted by the metric you ask for.

Usage:
  python grid.py --exe PATH --out grid.csv [--workers 12] [--seconds 7]
      [--speed 60] [--extra "--facing 0,1"] [--sort final_along]
      field=v1,v2,v3 ... ENV:OPENMW_RAGDOLL_X=v1,v2 ...
Each key=values argument becomes one grid axis; ENV: keys are set in the
child environment (other OPENMW_RAGDOLL_* variables are stripped).
"""
import argparse, csv, itertools, json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

def run(exe, base_args, profile, env_knobs, timeout=90.0):
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith("OPENMW_RAGDOLL_")}
    env.update(env_knobs)
    cmd = [exe] + base_args + ["%s=%s" % (k, v) for k, v in profile.items()]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=timeout)
        line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ""
        res = json.loads(line) if line.startswith("{") else None
    except Exception:
        res = None
    return cmd, res

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--seconds", default="7")
    ap.add_argument("--speed", default="60")
    ap.add_argument("--start", default="30")
    ap.add_argument("--extra", default="")
    ap.add_argument("--sort", default="final_along")
    ap.add_argument("axes", nargs="*")
    a = ap.parse_args()
    base = ["--seconds", a.seconds, "--speed", a.speed, "--start", a.start] + (a.extra.split() if a.extra else [])
    names, values = [], []
    for axis in a.axes:
        k, v = axis.split("=", 1)
        names.append(k)
        values.append(v.split(","))
    cells = list(itertools.product(*values)) if values else [()]
    def one(cell):
        profile = {n: v for n, v in zip(names, cell) if not n.startswith("ENV:")}
        envk = {n[4:]: v for n, v in zip(names, cell) if n.startswith("ENV:")}
        cmd, res = run(a.exe, base, profile, envk)
        row = dict(zip(names, cell))
        row.update({
            "fall": "" if res is None else ("" if res.get("first_fall") is None else res["first_fall"]),
            "pass": "" if res is None else res.get("pass"),
            "final_along": "" if res is None else res.get("final_along"),
            "mean_speed": "" if res is None else res.get("mean_speed"),
            "max_slide": "" if res is None else res.get("max_slide"),
            "stride": "" if res is None else res.get("mean_landing_fore_aft_separation"),
            "airborne": "" if res is None else res.get("airborne"),
            "violations": "" if res is None else "|".join(v.split(":")[0] for v in res.get("violations", [])),
            "cmd": " ".join(cmd),
        })
        return row
    with ThreadPoolExecutor(a.workers) as pool:
        rows = list(pool.map(one, cells))
    key = a.sort
    def sk(r):
        try: return -float(r[key])
        except (TypeError, ValueError): return 1e9
    rows.sort(key=sk)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    up = sum(1 for r in rows if r["fall"] == "" and r["final_along"] != "")
    print("cells %d  upright %d  written %s" % (len(rows), up, a.out))
    for r in rows[:12]:
        print({k: r[k] for k in names}, "fall", r["fall"], "final", r["final_along"], "slide", r["max_slide"], "stride", r["stride"])

if __name__ == "__main__":
    main()
