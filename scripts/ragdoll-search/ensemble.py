#!/usr/bin/env python3
"""Seeded ensemble scoring for the reference-gait walksearch harness.

One profile (walksearch field=value overrides plus the env-gated solver
knobs) is run over a fixed, seeded sample of the frozen grid below; every
case's command line, raw walksearch JSON and score is written to one JSON.

Frozen grid (docs/ragdoll/history/gait-search-at-scale-2026-09-10.txt):
  speeds     {30, 45, 60, 75}              --speed, --start 30, --seconds 7
  rates      {30, 60, 120}                 --rate (steps stay in 60 Hz units)
  pushes     none | fwd (+Y, travel) | lat (+-X), magnitude {0,1000,1500,2000},
             step uniform in [90, 240] (60 Hz steps; covers both legs'
             stances), body 0 (pelvis), world impulse
  maneuvers  none | stop300 | turn45-300 | turn90-300 | turn90f-300 (faced
             redirect) | strafeR | strafeL (lateral travel from the start)
             | backpedal (negative --speed); strafe/backpedal hold --facing 0,1
             (grid 3, Fable 2026-09-11)
Sampling: each case draws every axis independently and uniformly with
random.Random(seed); magnitude 0 means no push. Common random numbers:
the same (n, seed, rates) gives the same case list for every profile.

Score per case:
  fall (first_fall not null, or harness crash/timeout) -> 0
  any violation  -> 0.5 * (1 - clamp((max_slide - 1.5) / 3.0, 0, 1))
  pass           -> 1 + 0.5 * min(final_along / final_request, 1)
                      - 0.02 * lateral_rms
                    (stop cases: the speed term is 1; pass already means
                     final |v| <= 5)
Usage:
  python ensemble.py --exe PATH [--n 200] [--seed 1] [--rates 30,60,120]
                     [--env OPENMW_RAGDOLL_DS_LIFT=150 ...] [--out f.json]
                     [field=value ...]
"""
import argparse
import math
import json
import os
import random
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SPEEDS = (30, 45, 60, 75)
RATES = (30, 60, 120)
PUSH_MAGNITUDES = (0, 1000, 1500, 2000)
PUSH_DIRECTIONS = ("fwd", "lat")
PUSH_STEP_RANGE = (90, 240)
MANEUVERS = ("none", "stop300", "turn45-300", "turn90-300", "turn90f-300", "strafeR", "strafeL", "backpedal")
# Grid 4 (Fable, 2026-09-11, history/strafe-native-input): strafes whose
# request is modulated like a real AI/animation input - magnitude +-18%
# sine at the walk-cycle period (0.92 s) and heading random-walking within
# +-8 deg of pure lateral. Opt-in (--wander) so grid-3 seeds stay comparable.
WANDER_MANEUVERS = ("strafeRw", "strafeLw")
WANDER_PERIOD_S = 0.92
WANDER_MAG_FRACTION = 0.18
WANDER_HEADING_DEG = 8.0
WANDER_STEP = 4
SECONDS = 7
START = 30
MANEUVER_STEP = 300
SLIDE_GATE = 1.5
SLIDE_SPAN = 3.0
TIMEOUT = 60.0

# Env knobs searched as extra dimensions; every other OPENMW_RAGDOLL_*
# variable is stripped from the child environment so runs are reproducible.
ENV_KNOBS = ("OPENMW_RAGDOLL_DS_PROPULSION", "OPENMW_RAGDOLL_DS_LIFT", "OPENMW_RAGDOLL_DS_PUSH_SHIFT", "OPENMW_RAGDOLL_PUSHOFF",
             # floor/contact probes (2026-09-11, strafe-native-input): pair with mesh_ground=1
             "OPENMW_RAGDOLL_MESH_GRID", "OPENMW_RAGDOLL_MESH_OFFSET", "OPENMW_RAGDOLL_PHYS_MANIFOLD", "OPENMW_RAGDOLL_PHYS_SLOP")


SHOVE2 = [False]  # set by --shove2: every pushed case is pushed twice


def sample_cases(n, seed, rates=RATES, wander=False):
    rng = random.Random(seed)
    maneuvers = MANEUVERS + (WANDER_MANEUVERS if wander else ())
    cases = []
    for i in range(n):
        case = {
            "id": i,
            "speed": rng.choice(SPEEDS),
            "rate": rng.choice(tuple(rates)),
            "push_mag": rng.choice(PUSH_MAGNITUDES),
            "push_dir": rng.choice(PUSH_DIRECTIONS),
            "push_sign": rng.choice((-1, 1)),
            "push_step": rng.randint(*PUSH_STEP_RANGE),
            "maneuver": rng.choice(maneuvers),
        }
        if case["push_mag"] == 0:
            case["push_dir"] = "none"
            case["push_sign"] = 0
            case["push_step"] = 0
        elif SHOVE2[0]:
            case["shove2"] = True
        cases.append(case)
    return cases


NATIVE_BODY = []  # ["--native-body", context, yaml] when --native-body is given


def case_args(case):
    s = case["speed"]
    man = case["maneuver"]
    # backpedal: negative travel from the start (Kimi i36/i47 form)
    speed_arg = -s if man == "backpedal" else s
    args = list(NATIVE_BODY) + ["--seconds", str(SECONDS), "--speed", str(speed_arg), "--start", str(START)]
    if case["rate"] != 60:
        args += ["--rate", str(case["rate"])]
    if case["push_mag"]:
        m = case["push_mag"]
        vec = (0, m, 0) if case["push_dir"] == "fwd" else (case["push_sign"] * m, 0, 0)
        args += ["--impulse", "%d,0,%d,%d,%d" % (case["push_step"], vec[0], vec[1], vec[2])]
        # Combat probe (2026-09-11, --shove2): the same push again 30
        # steps (0.5 s) later. Two hits in a row from behind fell the
        # walker in play while a single one held.
        if case.get("shove2"):
            args += ["--impulse", "%d,0,%d,%d,%d" % (case["push_step"] + 30, vec[0], vec[1], vec[2])]
    # Grid 3 (Fable, 2026-09-11): strafe and backpedal HOLD the facing at
    # +y. Without --facing the harness's requested facing follows travel, so
    # the body rotates toward the travel direction through a sideways state
    # and falls; that measured a turning walk, not a strafe or a backpedal.
    if man in ("strafeR", "strafeL", "backpedal") or man in WANDER_MANEUVERS:
        args += ["--facing", "0,1"]
    if man in WANDER_MANEUVERS:
        side = 1 if man == "strafeRw" else -1
        rng = random.Random(1000003 * case["id"] + 17)
        phase = rng.uniform(0.0, 2.0 * math.pi)
        heading = 0.0
        for step in range(START, SECONDS * 60, WANDER_STEP):
            t = step / 60.0
            mag = s * (1.0 + WANDER_MAG_FRACTION * math.sin(2.0 * math.pi * t / WANDER_PERIOD_S + phase))
            heading = max(-WANDER_HEADING_DEG, min(WANDER_HEADING_DEG, heading + rng.uniform(-2.0, 2.0)))
            a = math.radians(heading)
            args += ["--request", "%d,%.2f,%.2f" % (step, side * mag * math.cos(a), side * mag * math.sin(a))]
    if man == "strafeR":
        args += ["--request", "%d,%d,0" % (START, s)]
    elif man == "strafeL":
        args += ["--request", "%d,%d,0" % (START, -s)]
    elif man == "turn90f-300":
        args += ["--request", "%d,%d,0" % (MANEUVER_STEP, s), "--facing-at", "%d,%d,0" % (MANEUVER_STEP, s)]
    if man == "stop300":
        args += ["--request", "%d,0,0" % MANEUVER_STEP]
    elif man == "turn45-300":
        c = "%.2f" % (s * 0.70710678)
        args += ["--request", "%d,%s,%s" % (MANEUVER_STEP, c, c)]
    elif man == "turn90-300":
        args += ["--request", "%d,%d,0" % (MANEUVER_STEP, s)]
    return args


def fmt(v):
    if isinstance(v, float):
        return "%.6g" % v
    return str(v)


def profile_args(profile):
    return ["%s=%s" % (k, fmt(v)) for k, v in sorted(profile.items())]


def active_env(env_knobs):
    # An explicitly given knob is passed as given: "=0" is a real value for
    # the contact probes (OPENMW_RAGDOLL_PHYS_MANIFOLD=0) and offsets are
    # not numbers (OPENMW_RAGDOLL_MESH_OFFSET=500,0). The old "> 0 only"
    # rule silently dropped both (2026-09-11).
    out = {}
    for k, v in sorted((env_knobs or {}).items()):
        if v is None:
            continue
        try:
            out[k] = float(v)
        except (TypeError, ValueError):
            out[k] = str(v)
    return out


def child_env(env_knobs):
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith("OPENMW_RAGDOLL_")}
    for k, v in active_env(env_knobs).items():
        env[k] = fmt(v)
    return env


def command_line(exe, case, profile, env_knobs):
    envs = ["%s=%s" % (k, fmt(v)) for k, v in active_env(env_knobs).items()]
    return " ".join(envs + [exe] + case_args(case) + profile_args(profile))


def score_result(case, res):
    if res is None or res.get("first_fall") is not None:
        return 0.0
    if res.get("violations"):
        excess = min(max((res.get("max_slide", 0.0) - SLIDE_GATE) / SLIDE_SPAN, 0.0), 1.0)
        return 0.5 * (1.0 - excess)
    req = res.get("final_request", 0.0)
    speed_term = 1.0 if req <= 1e-3 else min(res.get("final_along", 0.0) / req, 1.0)
    return 1.0 + 0.5 * speed_term - 0.02 * res.get("lateral_rms", 0.0)


def outcome(res):
    if res is None:
        return "crash"
    if res.get("first_fall") is not None:
        return "fall"
    return "fail" if res.get("violations") else "pass"


def run_case(exe, case, profile, env_knobs):
    cmd = [os.path.abspath(exe)] + case_args(case) + profile_args(profile)
    t0 = time.time()
    res = None
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=TIMEOUT, env=child_env(env_knobs))
        lines = [l for l in p.stdout.strip().splitlines() if l.startswith("{")]
        res = json.loads(lines[-1]) if lines else None
    except (subprocess.TimeoutExpired, json.JSONDecodeError):
        res = None
    return {
        "case": case,
        "command": command_line(exe, case, profile, env_knobs),
        "result": res,
        "outcome": outcome(res),
        "score": score_result(case, res),
        "seconds": round(time.time() - t0, 3),
    }


def axis_key(case, axis):
    if axis == "push":
        return "none" if case["push_mag"] == 0 else "%s%d" % (case["push_dir"], case["push_mag"])
    return str(case[axis])


def _agg(rs):
    n = len(rs)
    if not n:
        return {"n": 0, "score": 0.0, "fall_rate": 0.0, "pass_rate": 0.0}
    return {
        "n": n,
        "score": sum(r["score"] for r in rs) / n,
        "fall_rate": sum(r["outcome"] in ("fall", "crash") for r in rs) / n,
        "pass_rate": sum(r["outcome"] == "pass" for r in rs) / n,
    }


AXES = ("rate", "speed", "push_mag", "push_dir", "maneuver")


def summarize(records):
    out = _agg(records)
    out["crashes"] = sum(r["outcome"] == "crash" for r in records)
    out["by"] = {}
    for axis in AXES:
        groups = defaultdict(list)
        for r in records:
            groups[axis_key(r["case"], axis)].append(r)
        out["by"][axis] = {k: _agg(v) for k, v in sorted(groups.items())}
    return out


_POOL = None


def pool(workers):
    global _POOL
    if _POOL is None:
        _POOL = ThreadPoolExecutor(max_workers=workers)
    return _POOL


def evaluate_many(exe, candidates, cases, workers=12):
    """candidates: list of (profile, env_knobs). All cases of all candidates
    go into one queue so the pool never idles between candidates."""
    ex = pool(workers)
    futs = [[ex.submit(run_case, exe, c, p, e) for c in cases] for p, e in candidates]
    out = []
    for fl in futs:
        recs = [f.result() for f in fl]
        out.append((summarize(recs), recs))
    return out


def evaluate(exe, profile, env_knobs, cases, workers=12):
    return evaluate_many(exe, [(profile, env_knobs)], cases, workers)[0]


def print_summary(s, label=""):
    print("%sscore %.4f  fall %.3f  pass %.3f  n %d  crashes %d" % (
        label, s["score"], s["fall_rate"], s["pass_rate"], s["n"], s["crashes"]))
    for axis, groups in s["by"].items():
        print("  %-9s " % axis + "  ".join(
            "%s:%.2f/f%.2f/p%.2f" % (k, g["score"], g["fall_rate"], g["pass_rate"]) for k, g in groups.items()))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", required=True)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--rates", default="30,60,120")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--env", action="append", default=[], help="KEY=VALUE, one of " + ", ".join(ENV_KNOBS))
    ap.add_argument("--wander", action="store_true", help="add the grid-4 modulated strafes (strafeRw/strafeLw)")
    ap.add_argument("--shove2", action="store_true", help="every pushed case is pushed again 0.5 s later")
    ap.add_argument("--native-body", nargs=2, metavar=("CONTEXT", "YAML"),
                    help="run on the captured game skeleton + YAML profile (walksearch --native-body)")
    ap.add_argument("--out")
    ap.add_argument("--tracker-task", help="record this raw result for a tracker gate")
    ap.add_argument("overrides", nargs="*", help="walksearch field=value")
    a = ap.parse_args(argv)
    profile = dict(o.split("=", 1) for o in a.overrides)
    env = {}
    for e in a.env:
        k, v = e.split("=", 1)
        if k not in ENV_KNOBS:
            sys.exit("unknown env knob " + k)
        try:
            env[k] = float(v)
        except ValueError:
            env[k] = v
    rates = tuple(int(r) for r in a.rates.split(","))
    if a.native_body:
        NATIVE_BODY[:] = ["--native-body", os.path.abspath(a.native_body[0]), os.path.abspath(a.native_body[1])]
    SHOVE2[0] = bool(a.shove2)
    cases = sample_cases(a.n, a.seed, rates, a.wander)
    t0 = time.time()
    summary, records = evaluate(a.exe, profile, env, cases, a.workers)
    doc = {
        "exe": a.exe, "n": a.n, "seed": a.seed, "rates": list(rates), "profile": profile, "env": env,
        "source_commit": subprocess.run(
            ["git", "-C", str(Path(__file__).resolve().parents[2]), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "command": "python scripts/ragdoll-search/ensemble.py " + " ".join(argv),
        "wall_seconds": round(time.time() - t0, 1), "summary": summary, "cases": records,
        "tracker": {
            "producer": "ensemble.py",
            "total_checks": len(records),
            "failed_checks": sum(record["outcome"] != "pass" for record in records),
        },
    }
    if a.out:
        with open(a.out, "w") as f:
            json.dump(doc, f, indent=1)
    if a.tracker_task:
        if not a.out:
            ap.error("--tracker-task requires --out")
        root = Path(__file__).resolve().parents[2]
        subprocess.run([
            sys.executable, str(root / "tools/tracker/write_result.py"),
            "--root", str(root), "--task", a.tracker_task,
            "--producer", "ensemble_v1", "--artifact", str(Path(a.out).resolve()),
        ], check=True)
    print_summary(summary, "(%.1fs) " % doc["wall_seconds"])
    return doc


if __name__ == "__main__":
    main()
