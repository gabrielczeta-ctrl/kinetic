"""Push table (roadmap step 3, M0): how each push is caught.

usage: push_report.py RUN_DIR [RUN_DIR ...]
For every test_play01_walking case with 'push' or 'knockdown' in its name,
from the impact on: fell (and when), drift (largest horizontal centre-of-mass
move in the first 2 s off where the walk target carried it), recovery time (centre-of-mass
velocity back within 30 u/s of the target and the torso within 12 degrees of
upright for 0.3 s), and the balance helper (soft root): its mean force over
the first second against the second before the impact, its peak force, its
lowest strength and the share of steps it saturated; slide (a foot's
travel while it stays down, first 2 s; slide2 = with both feet down). The legs do more of the
catching when the helper's extra force is small and recovery is still fast.
"""
import gzip
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

FALL_MODES = {'falling', 'fallen', 'gettingup', 'getting_up', 'blendback', 'blend_back'}


def records(run):
    for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def length(v, horizontal=False):
    if not isinstance(v, list) or len(v) < 2:
        return None
    return math.hypot(v[0], v[1]) if horizontal or len(v) < 3 else math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def summarise(run):
    tests = defaultdict(lambda: {'rows': [], 'impact': None})
    for r in records(run):
        test = r.get('test')
        if not test or not ('push' in test.lower() or 'knockdown' in test):
            continue
        if r.get('kind') == 'impact':
            tests[test]['impact_pending'] = True
            continue
        state = r.get('state') or r
        if not isinstance(state, dict) or not isinstance(state.get('controller'), dict):
            continue
        t = r.get('simulation_time', 0.0)
        if tests[test].pop('impact_pending', False):
            tests[test]['impact'] = t
        tests[test]['rows'].append((t, state))
    out = {}
    for test, data in tests.items():
        t0 = data['impact']
        if t0 is None:
            continue
        rows = data['rows']
        before = [s for t, s in rows if t0 - 1.0 <= t < t0]
        after = [(t, s) for t, s in rows if t >= t0]
        if not after:
            continue
        start = after[0][1]
        com0 = start.get('centerOfMass')
        expected = list(com0[:2]) if isinstance(com0, list) else None
        fell, fell_t, drift, recovered, calm = False, None, 0.0, None, 0.0
        previous_t = t0
        for t, s in after:
            c = s['controller']
            if str(c.get('mode', '')).lower() in FALL_MODES and not fell:
                fell, fell_t = True, t - t0
            # Where the body was headed: the walk target integrated from the
            # impact on (the cases stop walking part-way).
            target = c.get('targetHorizontalVelocity')
            if expected is not None and isinstance(target, list):
                expected[0] += target[0] * (t - previous_t)
                expected[1] += target[1] * (t - previous_t)
            com = s.get('centerOfMass')
            if isinstance(com, list) and expected is not None and t - t0 <= 2.0:
                drift = max(drift, math.hypot(com[0] - expected[0], com[1] - expected[1]))
            v, target = c.get('centerOfMassVelocity'), c.get('targetHorizontalVelocity')
            up = c.get('torsoUpDot')
            steady = (isinstance(v, list) and isinstance(target, list) and isinstance(up, (int, float))
                      and math.hypot(v[0] - target[0], v[1] - target[1]) < 30.0
                      and up > math.cos(math.radians(12.0)))
            calm = calm + (t - previous_t) if steady else 0.0
            if recovered is None and calm >= 0.3 and t - t0 > 0.1:
                recovered = t - t0 - 0.3
            previous_t = t

        def force(rows_):
            values = [length((s.get('softRoot') or {}).get('force')) for s in rows_]
            values = [v for v in values if v is not None]
            return values

        # Steps in the 2 s after the impact: a foot that lifts more than 2
        # units and lands more than 8 units from where it last stood still
        # (sliding feet do not count; walking cases count the walk's steps).
        steps = 0
        for foot in ('left_foot', 'right_foot'):
            anchor, previous, lifted = None, None, False
            for t, s in after:
                if t - t0 > 2.0:
                    break
                body = (s.get('bodies') or {}).get(foot) or {}
                p = body.get('position')
                if not isinstance(p, list):
                    continue
                if previous is not None and t > previous[0]:
                    speed = math.hypot(p[0] - previous[1][0], p[1] - previous[1][1]) / (t - previous[0])
                    if anchor is None and speed < 15.0:
                        anchor, lifted = p, False
                    elif anchor is not None:
                        lifted = lifted or p[2] - anchor[2] > 2.0
                        if speed < 15.0 and math.hypot(p[0] - anchor[0], p[1] - anchor[1]) > 8.0:
                            steps += lifted
                            anchor, lifted = p, False
                previous = (t, p)
        # Skating: horizontal travel of a foot that stays down (within 2
        # units of its height at the impact) in the 2 s after it.
        slide = 0.0
        for foot in ('left_foot', 'right_foot'):
            z0, previous = None, None
            for t, s in after:
                if t - t0 > 2.0:
                    break
                p = ((s.get('bodies') or {}).get(foot) or {}).get('position')
                if not isinstance(p, list):
                    continue
                if z0 is None:
                    z0 = p[2]
                if previous is not None and p[2] - z0 < 2.0 and previous[2] - z0 < 2.0:
                    slide += math.hypot(p[0] - previous[0], p[1] - previous[1])
                previous = p
        # The same slide split by support (PGC request 2026-09-30): a foot
        # sliding while the other is up (single) or down too (double).
        slide_single = slide_double = 0.0
        z0s, prev = {}, {}
        for t, s in after:
            if t - t0 > 2.0:
                break
            pos = {f: ((s.get('bodies') or {}).get(f) or {}).get('position') for f in ('left_foot', 'right_foot')}
            if not all(isinstance(v, list) for v in pos.values()):
                continue
            for f, v in pos.items():
                z0s.setdefault(f, v[2])
            down = {f: v[2] - z0s[f] < 2.0 for f, v in pos.items()}
            for f, v in pos.items():
                if f in prev and down[f] and prev[f][2] - z0s[f] < 2.0:
                    d = math.hypot(v[0] - prev[f][0], v[1] - prev[f][1])
                    if all(down.values()):
                        slide_double += d
                    else:
                        slide_single += d
            prev = pos
        # Hover (user bookmark 5): the longest stretch within 3 s of the push
        # spent between standing and falling (bracing or stumbling).
        hover, run_start = 0.0, None
        for t, s in after:
            if t - t0 > 3.0:
                break
            m = str(((s.get('controller') or {}).get('mode')) or '').lower()
            if m in ('bracing', 'stumbling'):
                run_start = t if run_start is None else run_start
                hover = max(hover, t - run_start)
            else:
                run_start = None
        pre = force(before)
        first = force([s for t, s in after if t - t0 <= 1.0])
        strength = [(s.get('softRoot') or {}).get('strength') for t, s in after]
        strength = [v for v in strength if isinstance(v, (int, float))]
        saturated = [(s.get('softRoot') or {}).get('saturated') for t, s in after if t - t0 <= 1.0]
        out[test] = {
            'fell': fell, 'fell_t': fell_t, 'drift': drift, 'recover': recovered, 'steps': steps, 'slide': slide, 'slide_single': slide_single, 'hover': hover, 'slide_double': slide_double,
            'helper_pre': sum(pre) / len(pre) if pre else float('nan'),
            'helper_1s': sum(first) / len(first) if first else float('nan'),
            'helper_peak': max(first) if first else float('nan'),
            'strength_min': min(strength) if strength else float('nan'),
            'saturated': sum(1 for x in saturated if x) / len(saturated) if saturated else float('nan'),
        }
    return out


def main():
    runs = [summarise(run) for run in sys.argv[1:]]
    names = sorted({name for run in runs for name in run})
    print('%-44s %-4s %5s %6s %7s %5s %9s %9s %9s %6s %5s %6s %6s' % (
        'case', 'run', 'fell', 'drift', 'recover', 'steps', 'helper-1', 'helper+1s', 'peak', 'minstr', 'sat', 'slide',
        'slide2'))
    for name in names:
        for index, run in enumerate(runs):
            row = run.get(name)
            if not row:
                continue
            fell = ('%.1f' % row['fell_t']) if row['fell'] else '-'
            recover = ('%.2f' % row['recover']) if row['recover'] is not None else 'never'
            print('%-44s %-4d %5s %6.1f %7s %5d %9.0f %9.0f %9.0f %6.2f %5.2f %6.1f %6.1f' % (
                name.replace('PLAY-01 ', '')[:44], index, fell, row['drift'], recover, row['steps'], row['helper_pre'],
                row['helper_1s'], row['helper_peak'], row['strength_min'], row['saturated'], row['slide'],
                row['slide_double']))


if __name__ == '__main__':
    main()
