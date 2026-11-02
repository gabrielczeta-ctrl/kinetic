#!/usr/bin/env python3
"""Offline stress pass for the reference gait (plan: robustness before combat).

Runs the ragdollcore-walksearch driver over perturbation families and prints a
compact table per family. Diagnostic tooling: changes no controller behaviour
and no acceptance gate; every case is judged by the driver's own gates.

Families (pass any subset; default all):
  jitter    every float profile field at -10/-5/+5/+10 percent, one at a
            time, on a short push set; reports fields whose pass count drops
            (knife-edge parameters) and the mass scale sweep
  soak      60 s walks at 30/45/60 u/s plus a 60 s walk with a stop and a
            restart; reports slide, timeouts and final speed
  multipush random sequences of 2-4 shoves (seeded) at random phases,
            directions and magnitudes 500-2000, some during a turn or stop
  turns     the steering sweep (turns at four phases, stops, speed changes)

Usage:
  python scripts/ragdoll-stress/stress.py --exe <path to ragdollcore-walksearch.exe>
      [--families jitter,soak,multipush,turns] [--seed 1] [--sequences 40]
      [--workers 8] [--json out.json]
"""
import argparse
import json
import math
import random
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

SPEED = 60.0
# Short push set for the jitter family: nominal plus eight shoves.
JITTER_CASES = [
    ["--speed", "60", "--start", "30"],
    ["--speed", "45", "--start", "30"],
    ["--speed", "60", "--start", "30", "--impulse", "150,3,1000,0,0"],
    ["--speed", "60", "--start", "30", "--impulse", "150,3,-1000,0,0"],
    ["--speed", "60", "--start", "30", "--impulse", "200,3,0,1000,0"],
    ["--speed", "60", "--start", "30", "--impulse", "200,3,0,-1000,0"],
    ["--speed", "60", "--start", "30", "--impulse", "120,3,1500,0,0"],
    ["--speed", "60", "--start", "30", "--impulse", "260,3,0,1500,0"],
    ["--speed", "60", "--start", "30", "--impulse", "200,3,700,700,0"],
    ["--speed", "60", "--start", "30", "--request", "200,0,0"],
]
FRACTION_FIELDS = {"rise_end", "travel_end", "descent_end", "press_start", "travel_start", "placement_latch",
                   "support_hull_scale", "swing_reach"}
# Fields that are switches or timing gates rather than gains; jittering them
# is not a robustness question.
JITTER_SKIP = {"minimum_swing", "contact_confirmation", "maximum_double_support", "maximum_step",
               "transfer_load", "swing_start_load", "stop_timeout", "stop_speed", "handover",
               # switches, not gains
               "predict_placement", "swing_velocity_feedforward", "stance_hip_rate_hold", "grounds_placement",
               "coordinates_leg_pose", "hold_swing_foot"}


def run_driver(exe, args, overrides):
    command = [exe] + args + [f"{k}={v}" for k, v in overrides.items()]
    try:
        p = subprocess.run(command, capture_output=True, text=True, timeout=600)
        return json.loads(p.stdout.strip().splitlines()[-1])
    except Exception as error:  # driver crash or bad JSON: a failure, never a pass
        return {"pass": False, "first_fall": -1, "violations": [f"driver failed: {error}"], "max_slide": 99.0,
                "final_speed": 0.0, "transfers": [], "timed_out": 0}


def list_fields(exe):
    p = subprocess.run([exe, "--list"], capture_output=True, text=True)
    fields = {}
    for line in p.stdout.splitlines():
        if "=" not in line:
            continue
        name, value = line.strip().split("=", 1)
        try:
            fields[name] = float(value)
        except ValueError:
            pass
    return fields


def summarize(results):
    passes = sum(1 for r in results if r.get("pass"))
    falls = sum(1 for r in results if r.get("first_fall") is not None)
    return passes, falls


def family_jitter(exe, pool, report):
    fields = list_fields(exe)
    base = list(pool.map(lambda a: run_driver(exe, a, {}), JITTER_CASES))
    base_passes, base_falls = summarize(base)
    print(f"jitter: baseline {base_passes}/{len(JITTER_CASES)} passes, {base_falls} falls on the short push set")
    rows = []
    for name, value in sorted(fields.items()):
        if name in JITTER_SKIP or value == 0.0:
            continue
        line = []
        for factor in (0.9, 0.95, 1.05, 1.10):
            # Fractions of the swing (or the hull scale) above one are rejected
            # by the profile validation; that is not a robustness result.
            if name in FRACTION_FIELDS and value * factor > 1.0:
                continue
            results = list(pool.map(lambda a: run_driver(exe, a, {name: f"{value * factor:.6g}"}), JITTER_CASES))
            passes, falls = summarize(results)
            line.append((factor, passes, falls))
        if not line:
            continue
        worst = min(p for _, p, _ in line)
        flag = "KNIFE" if worst <= base_passes - 3 else ("soft" if worst < base_passes else "")
        rows.append((name, value, line, flag))
        cells = " ".join(f"{int(round((f - 1) * 100)):+d}%:{p}/{len(JITTER_CASES)}{'F' * falls}" for f, p, falls in line)
        print(f"  {name:24s} {value:10.4g}  {cells}  {flag}")
    report["jitter"] = {"baseline": [base_passes, base_falls],
                        "fields": {n: {"value": v, "runs": l, "flag": f} for n, v, l, f in rows}}
    # Mass scale: heavier and lighter bodies with the same defaults.
    print("jitter: mass scale")
    mass = {}
    for scale in (0.8, 0.9, 1.1, 1.2):
        results = list(pool.map(lambda a: run_driver(exe, a + ["--mass-scale", str(scale)], {}), JITTER_CASES))
        passes, falls = summarize(results)
        mass[scale] = [passes, falls]
        print(f"  mass x{scale}: {passes}/{len(JITTER_CASES)} passes, {falls} falls")
    report["jitter"]["mass"] = mass


def family_soak(exe, pool, report):
    cases = {
        "soak 30 u/s 60 s": ["--speed", "30", "--start", "30", "--seconds", "60"],
        "soak 45 u/s 60 s": ["--speed", "45", "--start", "30", "--seconds", "60"],
        "soak 60 u/s 60 s": ["--speed", "60", "--start", "30", "--seconds", "60"],
        "soak stop/restart": ["--speed", "60", "--start", "30", "--seconds", "60", "--request", "1500,0,0",
                              "--request", "1800,0,60", "--request", "3000,0,0", "--request", "3300,60,0"],
    }
    results = dict(zip(cases.keys(), pool.map(lambda a: run_driver(exe, a, {}), cases.values())))
    out = {}
    for name, r in results.items():
        out[name] = {k: r.get(k) for k in ("pass", "first_fall", "max_slide", "timed_out", "mean_speed", "final_speed",
                                            "lateral_rms", "emergency_steps")}
        out[name]["transfers"] = len(r.get("transfers", []))
        out[name]["violations"] = r.get("violations", [])[:4]
        print(f"soak: {name:20s} pass={r.get('pass')} fall={r.get('first_fall')} transfers={len(r.get('transfers', []))} "
              f"slide={r.get('max_slide', 0):.2f} timeouts={r.get('timed_out')} mean={r.get('mean_speed', 0):.1f} "
              f"final={r.get('final_speed', 0):.1f} emergency={r.get('emergency_steps')} {r.get('violations', [])[:3]}")
    report["soak"] = out


def family_multipush(exe, pool, report, seed, sequences):
    rng = random.Random(seed)
    cases = []
    for i in range(sequences):
        args = ["--speed", "60", "--start", "30", "--seconds", "9"]
        count = rng.randint(2, 4)
        steps = sorted(rng.sample(range(120, 420, 6), count))
        magnitudes = []
        for step in steps:
            magnitude = rng.choice([500, 800, 1000, 1200, 1500, 2000])
            angle = rng.uniform(0, 2 * math.pi)
            args += ["--impulse", f"{step},3,{magnitude * math.cos(angle):.1f},{magnitude * math.sin(angle):.1f},0"]
            magnitudes.append(magnitude)
        kind = rng.choice(["straight", "straight", "turn", "stop"])
        if kind == "turn":
            angle = rng.choice([90, -90, 180])
            rad = math.radians(angle)
            args += ["--request", f"{rng.choice(steps) - 30},{-SPEED * math.sin(rad):.2f},{SPEED * math.cos(rad):.2f}"]
        elif kind == "stop":
            args += ["--request", f"{rng.choice(steps) - 20},0,0"]
        cases.append((i, kind, steps, magnitudes, args))
    results = list(pool.map(lambda c: run_driver(exe, c[4], {}), cases))
    falls_by_max = {}
    out = []
    for (i, kind, steps, magnitudes, args), r in zip(cases, results):
        fell = r.get("first_fall") is not None
        falls_by_max.setdefault(max(magnitudes), [0, 0])
        falls_by_max[max(magnitudes)][1] += 1
        if fell:
            falls_by_max[max(magnitudes)][0] += 1
        out.append({"sequence": i, "kind": kind, "steps": steps, "magnitudes": magnitudes, "fall": r.get("first_fall"),
                    "pass": r.get("pass"), "max_slide": r.get("max_slide"), "violations": r.get("violations", [])[:3]})
        mark = "FALL" if fell else ("pass" if r.get("pass") else "gate")
        print(f"multipush: #{i:02d} {kind:8s} steps={steps} mags={magnitudes} -> {mark} "
              f"(slide {r.get('max_slide', 0):.2f}) {r.get('violations', [])[:2] if not r.get('pass') else ''}")
    passes, falls = summarize(results)
    print(f"multipush: {passes}/{len(cases)} passes, {falls} falls; falls by largest shove in the sequence: "
          + ", ".join(f"{m}: {f}/{n}" for m, (f, n) in sorted(falls_by_max.items())))
    report["multipush"] = {"seed": seed, "passes": passes, "falls": falls, "falls_by_max": falls_by_max, "cases": out}


def family_turns(exe, pool, report):
    cases = []
    for angle in (45, 90, -90, 135, 180):
        rad = math.radians(angle)
        vx, vy = -SPEED * math.sin(rad), SPEED * math.cos(rad)
        for st in (200, 208, 216, 224):
            cases.append((f"turn{angle:+d}", ["--speed", "60", "--start", "30", "--seconds", "8", "--request",
                                              f"{st},{vx:.3f},{vy:.3f}"]))
    for st in (150, 200, 250, 300):
        cases.append(("stop", ["--speed", "60", "--start", "30", "--seconds", "8", "--request", f"{st},0,0"]))
    for st in (200, 224):
        cases.append(("slow30", ["--speed", "60", "--start", "30", "--request", f"{st},0,30"]))
        cases.append(("fast75", ["--speed", "45", "--start", "30", "--request", f"{st},0,75"]))
    results = list(pool.map(lambda c: run_driver(exe, c[1], {}), cases))
    grid = {}
    for (name, _), r in zip(cases, results):
        fell = r.get("first_fall") is not None
        grid.setdefault(name, []).append("F" if fell else ("." if r.get("pass") else "s"))
    passes, falls = summarize(results)
    print(f"turns: {passes}/{len(cases)} passes, {falls} falls")
    for name, marks in grid.items():
        print(f"  {name:8s} {''.join(marks)}")
    report["turns"] = {"passes": passes, "falls": falls, "grid": {k: "".join(v) for k, v in grid.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to ragdollcore-walksearch.exe")
    parser.add_argument("--families", default="jitter,soak,multipush,turns")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sequences", type=int, default=40)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--json", default="")
    options = parser.parse_args()
    families = [f.strip() for f in options.families.split(",") if f.strip()]
    report = {}
    with ThreadPoolExecutor(max_workers=options.workers) as pool:
        if "turns" in families:
            family_turns(options.exe, pool, report)
        if "soak" in families:
            family_soak(options.exe, pool, report)
        if "multipush" in families:
            family_multipush(options.exe, pool, report, options.seed, options.sequences)
        if "jitter" in families:
            family_jitter(options.exe, pool, report)
    if options.json:
        with open(options.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=1)
        print(f"report written to {options.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
