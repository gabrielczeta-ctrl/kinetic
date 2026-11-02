"""Why each foot lifted, by speed band (STEPPING audit 2026-11-01, item 3).

Reads the library's lift trace (OPENMW_RAGDOLL_FOOTFALL_TRACE=1, on in the
play launcher; `npc` traces the NPCs instead):
  RAGDOLL_FOOTFALL_LIFT side=.. due=.. crossed=.. overrun=.. trail=.. speed=..
  RAGDOLL_FOOTFALL_CAPTURE side=.. speed=..        (WALK-02 capture steps)
  RAGDOLL_FOOTFALL_HELPER_SWING side=..            (a balance helper's step)
  RAGDOLL_FOOTFALL_LAND side=.. miss=.. contact=.. (where a swing landed
                                                    against its target)

Categories of a lift: clock (on the gait clock's beat), late (due, waited
for the other foot), overrun (a stance leg stretched past its limit, off the
clock), late_overrun (both), trail (stair trail), capture, helper.
Bands: walk 15-140 u/s, jog 140-250, run 250+ (the audit's split).

Used by play_metrics.py (play) and move_report.py (tests), so a change is
judged by the same numbers in both.

usage: python tools/body/lift_reasons.py LOG [LOG ...]
"""
import re
import statistics
import sys
from collections import Counter, defaultdict

BANDS = (('walk', 15.0, 140.0), ('jog', 140.0, 250.0), ('run', 250.0, 1e9))
LIFT = re.compile(r'RAGDOLL_FOOTFALL_LIFT side=\d due=(\d) crossed=(\d) overrun=(\d) trail=(\d)'
                  r'.* speed=([\d.]+)')
CAPTURE = re.compile(r'RAGDOLL_FOOTFALL_CAPTURE .*speed=([\d.]+)')
LAND = re.compile(r'RAGDOLL_FOOTFALL_LAND side=\d .*miss=([\d.]+).*speed=([\d.]+)')


def band(speed):
    for name, low, high in BANDS:
        if low <= speed < high:
            return name
    return None


def category(due, crossed, overrun, trail):
    if trail:
        return 'trail'
    if crossed and not overrun:
        return 'clock'
    if due and overrun:
        return 'late_overrun'
    if due:
        return 'late'
    return 'overrun'


def count(lines, actor=None):
    """Counters per band of lift categories, and landing misses per band."""
    lifts = defaultdict(Counter)
    misses = defaultdict(list)
    for line in lines:
        if 'RAGDOLL_FOOTFALL_' not in line or (actor and ('actor=' + actor) not in line):
            continue
        m = LIFT.search(line)
        if m:
            b = band(float(m.group(5)))
            if b:
                lifts[b][category(*(m.group(i) == '1' for i in range(1, 5)))] += 1
            continue
        m = CAPTURE.search(line)
        if m:
            b = band(float(m.group(1)))
            if b:
                lifts[b]['capture'] += 1
            continue
        if 'RAGDOLL_FOOTFALL_HELPER_SWING' in line:
            lifts['any']['helper'] += 1
            continue
        m = LAND.search(line)
        if m:
            b = band(float(m.group(2)))
            if b:
                misses[b].append(float(m.group(1)))
    return lifts, misses


def summary(lines, actor=None):
    """Flat numbers: lift_clock_walk (share on the clock), lift_overrun_walk,
    lift_late_walk, lift_n_walk, ... per band; land_miss_walk (median units)."""
    lifts, misses = count(lines, actor)
    out = {}
    for name, _, _ in BANDS:
        c = lifts.get(name)
        n = sum(c.values()) if c else 0
        out['lift_n_' + name] = n
        for key in ('clock', 'late', 'overrun'):
            share = (c[key] + (c['late_overrun'] if key != 'clock' else 0)) / n if n else None
            out['lift_%s_%s' % (key, name)] = share
        out['land_miss_' + name] = statistics.median(misses[name]) if misses.get(name) else None
    return out


def main():
    for path in sys.argv[1:]:
        lines = open(path, encoding='utf-8', errors='replace').read().splitlines()
        lifts, misses = count(lines)
        print(path)
        for name, _, _ in BANDS:
            c = lifts.get(name, Counter())
            n = sum(c.values())
            parts = '  '.join('%s %d (%.0f%%)' % (k, v, 100.0 * v / n) for k, v in c.most_common()) if n else '-'
            miss = misses.get(name)
            print('  %-4s n=%-5d %s%s' % (name, n, parts,
                                         '  land miss p50 %.1f u' % statistics.median(miss) if miss else ''))
        if lifts.get('any'):
            print('  helper swings %d' % lifts['any']['helper'])


if __name__ == '__main__':
    main()
