#!/usr/bin/env python3
"""CMA-ES gait-profile search over the frozen ensemble (ensemble.py).

Re-running on a new binary is one command:
  python scripts/ragdoll-search/search.py --exe <walksearch.exe> --out <dir>

Search space: the 17 profile fields of the task spec, bounded 0.5x-2x of
the default and clamped to the profile validity rules (referencegait.cpp
isValid: lateral_request_fraction in (0,1], maximum_width >= minimum_width,
minimum_swing <= swing_duration <= maximum_step, double_support <=
maximum_double_support), plus the three env-gated solver knobs present at
cc32da17d6 (DS_PROPULSION, DS_LIFT, DS_PUSH_SHIFT) and stance_frequency
(default 0 = spring off), which are "zero-allowed" dimensions: the lowest
30% of their normalized range decodes to exactly 0, the rest linearly to
(0, hi].

Noise handling: every generation draws a fresh case sample (seed
search_seed + generation, N cases) shared by all candidates of that
generation (common random numbers), and the default profile is evaluated
on the same sample as a control. CMA-ES ranks only within a generation;
the cross-generation ranking used to pick the top 5 is the score delta
against that generation's default. Held-out: top 5 + default + sw022 on
seed --holdout-seed (never a search seed) with N --holdout-n.
Every evaluation is one CSV row (evaluations.csv).
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ensemble  # noqa: E402

# name, kind, lo, hi, default, zero_allowed
PARAMS = [
    # Strafe search (stage 2): grid-3 profile fields around their optimum plus
    # the two strafe gates. INITIAL_STANCE_BY_TRAVEL is held at 1.
    ("swing_duration", "field", 0.18, 0.34, 0.23878, False),
    ("double_support", "field", 0.02, 0.12, 0.021027, False),
    ("lateral_offset", "field", 1.5, 6.0, 2.9941, False),
    ("minimum_width", "field", 6.0, 18.0, 12.284, False),
    ("maximum_width", "field", 20.0, 60.0, 29.627, False),
    ("velocity_gain", "field", 0.2, 1.5, 0.31851, False),
    ("forward_error_gain", "field", 0.3, 1.8, 0.65695, False),
    ("transfer_velocity_gain", "field", 2.0, 12.0, 4.8692, False),
    ("maximum_acceleration", "field", 75.0, 250.0, 150.97, False),
    ("swing_frequency", "field", 6.0, 20.0, 11.401, False),
    ("stance_frequency", "field", 0.0, 12.0, 0.0, True),
    ("stance_knee_flexion", "field", 0.2, 0.8, 0.49418, False),
    ("loaded_ankle_frequency", "field", 1.0, 6.0, 3.2477, False),
    ("emergency_margin", "field", 4.0, 20.0, 12.75, False),
    ("turn_rate", "field", 0.2, 1.0, 0.47724, False),
    ("stop_deceleration", "field", 30.0, 120.0, 51.459, False),
    ("lateral_request_fraction", "field", 0.5, 1.0, 0.90978, False),
    ("OPENMW_RAGDOLL_PELVIS_TWIST", "env", 30.0, 90.0, 60.0, False),
    ("OPENMW_RAGDOLL_INITIAL_STANCE_BY_TRAVEL", "env", 1.0, 1.001, 1.0, False),
]
ZERO_ZONE = 0.3
ZERO_X0 = 0.15
NAMES = [p[0] for p in PARAMS]


def x_default():
    x = []
    for name, kind, lo, hi, d, zero in PARAMS:
        x.append(ZERO_X0 if zero and d == 0.0 else (d - lo) / (hi - lo))
    return np.array(x)


def decode(x):
    """normalized vector -> (profile overrides, env knobs, flat values)."""
    values = {}
    for xi, (name, kind, lo, hi, d, zero) in zip(x, PARAMS):
        xi = float(min(max(xi, 0.0), 1.0))
        if zero:
            v = 0.0 if xi <= ZERO_ZONE else hi * (xi - ZERO_ZONE) / (1.0 - ZERO_ZONE)
        else:
            v = lo + xi * (hi - lo)
        values[name] = float("%.5g" % v)
    values["maximum_width"] = max(values["maximum_width"], values["minimum_width"])
    profile = {n: values[n] for n, k, *_ in PARAMS if k == "field"}
    env = {n: values[n] for n, k, *_ in PARAMS if k == "env" and values[n] > 0.0}
    return profile, env, values


def default_candidate():
    return {}, {}, {n: p[4] for n, p in zip(NAMES, PARAMS)}


def diff_profile(values):
    """Only the fields that differ from the default (pasteable, minimal)."""
    prof, env = {}, {}
    for name, kind, lo, hi, d, zero in PARAMS:
        v = values[name]
        if abs(v - d) <= 1e-9 * max(1.0, abs(d)):
            continue
        if kind == "field":
            prof[name] = v
        elif v > 0.0:
            env[name] = v
    return prof, env


CSV_HEAD = ["eval", "gen", "role", "wall", "case_seed", "n", "score", "fall_rate", "pass_rate", "crashes",
            "default_score", "delta_vs_default"] + ["rate%d" % r for r in ensemble.RATES] + \
           ["m_" + m for m in ensemble.MANEUVERS] + NAMES


def csv_row(ev, gen, role, wall, seed, summ, dscore, values):
    by = summ["by"]
    row = [ev, gen, role, round(wall, 1), seed, summ["n"], round(summ["score"], 5), round(summ["fall_rate"], 4),
           round(summ["pass_rate"], 4), summ["crashes"], round(dscore, 5), round(summ["score"] - dscore, 5)]
    row += [round(by["rate"].get(str(r), {"score": float("nan")})["score"], 4) for r in ensemble.RATES]
    row += [round(by["maneuver"].get(m, {"score": float("nan")})["score"], 4) for m in ensemble.MANEUVERS]
    row += [values[n] for n in NAMES]
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--rates", default="60")
    ap.add_argument("--search-seed", type=int, default=1000)
    ap.add_argument("--holdout-seed", type=int, default=777)
    ap.add_argument("--holdout-n", type=int, default=400)
    ap.add_argument("--popsize", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.2)
    ap.add_argument("--max-evals", type=int, default=3000)
    ap.add_argument("--minutes", type=float, default=100.0)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--cma-seed", type=int, default=12345)
    ap.add_argument("--skip-holdout", action="store_true")
    ap.add_argument("--shove2", action="store_true", help="score with every pushed case pushed twice (ensemble --shove2)")
    ap.add_argument("--native-body", nargs=2, metavar=("CONTEXT", "YAML"),
                    help="score on the captured game skeleton + YAML profile (walksearch --native-body)")
    a = ap.parse_args()
    if a.native_body:
        ensemble.NATIVE_BODY[:] = ["--native-body", os.path.abspath(a.native_body[0]), os.path.abspath(a.native_body[1])]
    ensemble.SHOVE2[0] = bool(a.shove2)
    import cma

    os.makedirs(a.out, exist_ok=True)
    rates = tuple(int(r) for r in a.rates.split(","))
    csv_path = os.path.join(a.out, "evaluations.csv")
    fresh = not os.path.exists(csv_path)
    fh = open(csv_path, "a", newline="")
    w = csv.writer(fh)
    if fresh:
        w.writerow(CSV_HEAD)
    with open(os.path.join(a.out, "search-command.txt"), "w") as f:
        f.write("python scripts/ragdoll-search/search.py " + " ".join(sys.argv[1:]) + "\n")

    es = cma.CMAEvolutionStrategy(list(x_default()), a.sigma, {
        "bounds": [0.0, 1.0], "popsize": a.popsize, "seed": a.cma_seed, "verbose": -9})
    t0 = time.time()
    ev = 0
    gen = 0
    history = []  # (delta, score, gen, values)
    while ev < a.max_evals and (time.time() - t0) < a.minutes * 60 and not es.stop():
        xs = es.ask()
        decoded = [decode(x) for x in xs]
        seed = a.search_seed + gen
        cases = ensemble.sample_cases(a.n, seed, rates)
        dprof, denv, dvals = default_candidate()
        results = ensemble.evaluate_many(a.exe, [(dprof, denv)] + [(p, e) for p, e, _ in decoded], cases, a.workers)
        dsum = results[0][0]
        wall = time.time() - t0
        w.writerow(csv_row(ev, gen, "control-default", wall, seed, dsum, dsum["score"], dvals))
        fits = []
        for (p, e, vals), (summ, _recs) in zip(decoded, results[1:]):
            ev += 1
            w.writerow(csv_row(ev, gen, "candidate", wall, seed, summ, dsum["score"], vals))
            delta = summ["score"] - dsum["score"]
            history.append((delta, summ["score"], gen, vals, summ["fall_rate"]))
            fits.append(-summ["score"])
        fh.flush()
        es.tell(xs, fits)
        best = max(history, key=lambda h: h[0])
        print("gen %3d evals %5d  %.0fs  default %.3f  gen-best %.3f (fall %.2f)  best-delta %+.3f@g%d  sigma %.3f" % (
            gen, ev, wall, dsum["score"], -min(fits), min(r[0]["fall_rate"] for r in results[1:]),
            best[0], best[2], es.sigma), flush=True)
        state = {
            "gen": gen, "evals": ev, "wall": wall, "sigma": es.sigma,
            "mean": dict(zip(NAMES, [float(v) for v in es.mean])),
            "mean_decoded": decode(es.mean)[2],
            "stds": dict(zip(NAMES, [float(v) for v in es.stds])),
            "top": [{"delta": h[0], "score": h[1], "gen": h[2], "fall": h[4], "values": h[3]}
                    for h in sorted(history, key=lambda h: -h[0])[:10]],
        }
        with open(os.path.join(a.out, "state.json"), "w") as f:
            json.dump(state, f, indent=1)
        gen += 1
    fh.close()
    print("search done: %d evals, %d generations, %.0fs, stop=%s" % (ev, gen, time.time() - t0, es.stop()))

    # Top-5 distinct candidates by delta against the same-sample default;
    # the CMA mean joins as a sixth contender (it is the search's estimate).
    top, seen = [], set()
    for h in sorted(history, key=lambda h: -h[0]):
        key = tuple(round(h[3][n], 4) for n in NAMES)
        if key in seen:
            continue
        seen.add(key)
        top.append(h)
        if len(top) == 5:
            break
    with open(os.path.join(a.out, "top5.json"), "w") as f:
        json.dump([{"delta": h[0], "score": h[1], "gen": h[2], "values": h[3]} for h in top], f, indent=1)
    if a.skip_holdout:
        return
    holdout(a.exe, a.out, [h[3] for h in top] + [decode(es.mean)[2]], a.holdout_n, a.holdout_seed, rates, a.workers)


def holdout(exe, out, value_sets, n, seed, rates, workers):
    cases = ensemble.sample_cases(n, seed, rates)
    contenders = [("default", {}, {}), ("sw022", {"swing_duration": 0.22}, {})]
    for i, vals in enumerate(value_sets):
        prof, env = diff_profile(vals)
        contenders.append(("top%d" % (i + 1) if i < 5 else "cma-mean", prof, env))
    results = ensemble.evaluate_many(exe, [(p, e) for _, p, e in contenders], cases, workers)
    table = []
    for (label, prof, env), (summ, recs) in zip(contenders, results):
        doc = {"label": label, "exe": exe, "n": n, "seed": seed, "rates": list(rates), "profile": prof, "env": env,
               "summary": summ, "cases": recs}
        with open(os.path.join(out, "holdout-%s.json" % label), "w") as f:
            json.dump(doc, f, indent=1)
        table.append((label, summ, prof, env))
        ensemble.print_summary(summ, "%-9s " % label)
    with open(os.path.join(out, "holdout-summary.json"), "w") as f:
        json.dump([{"label": l, "score": s["score"], "fall_rate": s["fall_rate"], "pass_rate": s["pass_rate"],
                    "by": s["by"], "profile": p, "env": e} for l, s, p, e in table], f, indent=1)


if __name__ == "__main__":
    main()
