"""Sentinel against a baseline build run the same night (2026-10-04).

The sentinel's bars are "no worse than the 2026-09-29 build, with the
run-to-run spread"; by 10-04 the walking body had moved on and build 37 - the
last build the sentinel passed - failed its walk stage in 3 of 3 runs and its
combat stage in 1 of 3. A single sentinel run then gates on noise, not on the
change. This compares N runs of a candidate with N runs of a baseline, each
stage on its own:

- a case regresses when the candidate FAILs it (failed, and failed again) in
  more runs than the baseline beyond run-to-run noise: more than
  max(1, 1.645 * sqrt(2 n p (1 - p))) runs, n the runs each, p the pooled
  FAIL rate (one-sided 95%; 2026-10-05: with 12 walk runs a case failing
  about half the time - 6 vs 9 - counted as worse at the fixed margin of 1,
  and build 45's own later runs failed it 4 of 4);
- a stage regresses when its FAILs over all runs exceed the baseline's by
  more than one;
- duels must pass in every candidate run (absolute, as before).

usage: python tools/body/sentinel_compare.py --base b37c1,b37c2,... --cand b45c1,...
(names of sentinel runs; reads runtime-SENT-NAME-sentinel.txt in the dev root)
"""
import argparse
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]  # the workspace (source-footfall/tools/body/..)


def read(name):
    """{stage: (Counter of FAIL cases, runs counted)} for one sentinel run."""
    text = (ROOT / ('runtime-SENT-%s-sentinel.txt' % name)).read_text()
    out, stage = {}, None
    for line in text.splitlines():
        m = re.match(r'== (\w+):', line)
        if m:
            stage = m.group(1)
            out[stage] = Counter()
            continue
        m = re.match(r'\s+FAIL\s+(.*)$', line)
        if m and stage:
            out[stage][m.group(1).strip()] += 1
    return out


def collect(names):
    fails, runs = defaultdict(Counter), Counter()
    for name in names:
        for stage, counter in read(name).items():
            runs[stage] += 1
            fails[stage].update(counter)
    return fails, runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True)
    ap.add_argument('--cand', required=True)
    args = ap.parse_args()
    base, base_runs = collect(args.base.split(','))
    cand, cand_runs = collect(args.cand.split(','))
    ok = True
    for stage in sorted(set(base_runs) | set(cand_runs)):
        b, c = base[stage], cand[stage]
        print('== %s: baseline %d run(s), candidate %d run(s)' % (stage, base_runs[stage], cand_runs[stage]))
        if base_runs[stage] != cand_runs[stage]:
            print('  NOTE  unequal run counts')
        n = min(base_runs[stage], cand_runs[stage]) or 1
        for case in sorted(set(b) | set(c)):
            p = (b[case] + c[case]) / (base_runs[stage] + cand_runs[stage] or 1)
            margin = max(1.0, 1.645 * math.sqrt(2 * n * p * (1 - p)))
            worse = c[case] - b[case] > margin
            ok &= not worse
            print('  %-5s %-90s base %d  cand %d' % ('WORSE' if worse else 'same', case, b[case], c[case]))
        total_b, total_c = sum(b.values()), sum(c.values())
        stage_worse = total_c - total_b > 1
        if stage == 'duel' and total_c > 0:
            stage_worse = True
        ok &= not stage_worse
        print('  %s stage FAILs: baseline %d, candidate %d' % ('WORSE' if stage_worse else 'ok   ', total_b, total_c))
    print('SENTINEL VS BASELINE ' + ('PASS' if ok else 'FAIL'))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
