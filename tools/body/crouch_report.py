"""Crouch (sneak) quality per case: the physical body against the clip
(targetPosition), for CROUCH-01 / SNEAK-01 / look crouch.

usage: crouch_report.py [--filter REGEX] [--json] RUN_DIR [RUN_DIR ...]

Per case, over its sneaking segments (after the first 0.6 s of the crouch):
  depth   clip pelvis drop / body pelvis drop below the idle standing height
  pelv    body pelvis minus clip pelvis height, mean (negative = sinks)
  chest   the same for spine2 (the torso height of the walking bar)
  head    the same for the head
  pitch   torso forward pitch (pelvis to spine2), body minus clip, mean deg
  knee    knee bend (thigh-calf-foot), body minus clip, mean deg (+ = deeper)
  hands   hand offset from the chest, body against clip, mean units
  slide   horizontal speed of a foot while its contact flag is set, mean u/s
  steps   foot lifts (5 u above its last low) per second, standing crouch only
  speed   centre of mass speed / commanded speed while sneaking (moving segs)
  down    s from the crouch request until the pelvis is within 3 u of the clip
  up      s from the stand request until the pelvis is within 3 u of the clip
  over    most the pelvis rose above the clip on standing up (units)
  lean    torsoUpDot minimum while crouched (1 = upright)
  sunk    longest time both feet stayed over 4 u under their targets (s; the
          whole leg sunk into the floor, seen once after a sideways sneak)
  fall    the segment it fell in (- = none)
"""
import gzip
import json
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

FALL = {'falling', 'fallen', 'gettingup', 'getting_up', 'blendback', 'blend_back'}


def load(run):
    rows, sneaking = defaultdict(list), defaultdict(dict)
    for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8', errors='replace') as stream:
            for line in stream:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                test = r.get('test')
                if not test:
                    continue
                if r.get('kind') == 'segment_request':
                    c = r.get('command') or {}
                    sneaking[test][c.get('segment')] = c.get('sneak') is True
                    sneaking[test].setdefault('_moving', {})[c.get('segment')] = bool(
                        c.get('movement') or c.get('sideMovement'))
                elif r.get('kind') == 'frame' and 'bodies' in (r.get('state') or {}):
                    rows[test].append(r)
    return rows, sneaking


def P(r, body, key='position'):
    return r['state']['bodies'][body][key]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def angle(a, b):
    la, lb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    if la < 1e-6 or lb < 1e-6:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (la * lb)))))


def pitch(r, key):
    v = sub(P(r, 'spine2', key), P(r, 'pelvis', key))
    return math.degrees(math.atan2(math.hypot(v[0], v[1]), v[2]))


def knee(r, side, key):
    t, c, f = (P(r, '%s_%s' % (side, b), key) for b in ('thigh', 'calf', 'foot'))
    return angle(sub(c, t), sub(f, c))


def mean(xs):
    return statistics.fmean(xs) if xs else float('nan')


def case_metrics(rows, sneak):
    moving = sneak.get('_moving', {})
    mode = [str(((r['state'].get('controller') or {}).get('mode', ''))).lower() for r in rows]
    out = {'fall': next((r.get('segment') for r, m in zip(rows, mode) if m in FALL), '-')}
    idle = [r for r in rows if r.get('segment') == 'idle' and r['elapsed'] > 1.0] or rows[:30]
    idle_body = mean([P(r, 'pelvis')[2] - r['player_position'][2] for r in idle])
    idle_clip = mean([P(r, 'pelvis', 'targetPosition')[2] - r['player_position'][2] for r in idle])
    seg_start, crouched, first_crouch, first_stand = {}, [], None, None
    previous = None
    for i, r in enumerate(rows):
        seg = r.get('segment')
        if seg != previous:
            seg_start[i] = r['elapsed']
            if sneak.get(seg) and first_crouch is None:
                first_crouch = i
            if previous is not None and sneak.get(previous) and not sneak.get(seg) and first_stand is None:
                first_stand = i
            previous = seg
    t_crouch = rows[first_crouch]['elapsed'] if first_crouch is not None else None
    for i, r in enumerate(rows):
        if sneak.get(r.get('segment')) and t_crouch is not None and r['elapsed'] - t_crouch >= 0.6 \
                and (first_stand is None or i < first_stand) and mode[i] not in FALL:
            crouched.append((i, r))
    if not crouched:
        return out
    C = [r for _, r in crouched]
    z = lambda r, b, k='position': P(r, b, k)[2]  # noqa: E731
    out['depth'] = '%.0f/%.0f' % (idle_clip - mean([z(r, 'pelvis', 'targetPosition') - r['player_position'][2] for r in C]),
                                  idle_body - mean([z(r, 'pelvis') - r['player_position'][2] for r in C]))
    for name, body in (('pelv', 'pelvis'), ('chest', 'spine2'), ('head', 'head')):
        out[name] = mean([z(r, body) - z(r, body, 'targetPosition') for r in C])
    out['pitch'] = mean([pitch(r, 'position') - pitch(r, 'targetPosition') for r in C])
    out['knee'] = mean([knee(r, s, 'position') - knee(r, s, 'targetPosition') for r in C for s in ('left', 'right')])
    out['hands'] = mean([math.dist(sub(P(r, h), P(r, 'spine2')), sub(P(r, h, 'targetPosition'), P(r, 'spine2', 'targetPosition')))
                         for r in C for h in ('left_hand', 'right_hand')])
    slides = []
    for (i, r) in crouched:
        if i == 0:
            continue
        q, dt = rows[i - 1], r['elapsed'] - rows[i - 1]['elapsed']
        if dt <= 0:
            continue
        support = r['state'].get('support') or {}
        for side, flag in (('left', 'leftFoot'), ('right', 'rightFoot')):
            if support.get(flag) and (q['state'].get('support') or {}).get(flag):
                a, b = P(r, side + '_foot'), P(q, side + '_foot')
                slides.append(math.hypot(a[0] - b[0], a[1] - b[1]) / dt)
    out['slide'] = mean(slides)
    still = [(i, r) for i, r in crouched if not moving.get(r.get('segment'))
             and not ('turn' in (r.get('segment') or ''))]
    if len(still) > 30:
        lifts = 0
        for side in ('left', 'right'):
            low, up = None, False
            for _, r in still:
                fz = P(r, side + '_foot')[2] - r['player_position'][2]
                low = fz if low is None or (not up and fz < low) else low
                if not up and fz > low + 5.0:
                    lifts, up = lifts + 1, True
                elif up and fz < low + 2.0:
                    up, low = False, fz
        out['steps'] = lifts / max(1e-3, still[-1][1]['elapsed'] - still[0][1]['elapsed'])
    speeds = []
    for _, r in crouched:
        if moving.get(r.get('segment')):
            want = (r['state'].get('controller') or {}).get('rawTargetHorizontalVelocity') or [0, 0, 0]
            have = (r['state'].get('controller') or {}).get('centerOfMassVelocity') or [0, 0, 0]
            w = math.hypot(want[0], want[1])
            if w > 10:
                speeds.append(math.hypot(have[0], have[1]) / w)
    out['speed'] = mean(speeds)
    run_start, longest = None, 0.0
    for r in rows:
        sunk = all(P(r, s + '_foot', 'targetPosition')[2] - P(r, s + '_foot')[2] > 4.0 for s in ('left', 'right'))
        if sunk and run_start is None:
            run_start = r['elapsed']
        elif not sunk and run_start is not None:
            longest, run_start = max(longest, r['elapsed'] - run_start), None
    if run_start is not None:
        longest = max(longest, rows[-1]['elapsed'] - run_start)
    out['sunk'] = longest
    ups = [((r['state'].get('controller') or {}).get('torsoUpDot')) for r in C]
    ups = [u for u in ups if isinstance(u, (int, float))]
    out['lean'] = min(ups) if ups else float('nan')

    def settle(start):
        for i in range(start, len(rows)):
            r = rows[i]
            if abs(z(r, 'pelvis') - z(r, 'pelvis', 'targetPosition')) < 3.0:
                # held for 0.2 s
                j = i
                while j < len(rows) and rows[j]['elapsed'] - r['elapsed'] < 0.2 \
                        and abs(z(rows[j], 'pelvis') - z(rows[j], 'pelvis', 'targetPosition')) < 3.0:
                    j += 1
                if j >= len(rows) or rows[j]['elapsed'] - r['elapsed'] >= 0.2:
                    return r['elapsed'] - rows[start]['elapsed']
        return float('nan')
    if first_crouch is not None:
        out['down'] = settle(first_crouch)
    if first_stand is not None:
        out['up'] = settle(first_stand)
        window = [r for r in rows[first_stand:] if r['elapsed'] - rows[first_stand]['elapsed'] < 2.0]
        out['over'] = max(z(r, 'pelvis') - z(r, 'pelvis', 'targetPosition') for r in window) if window else float('nan')
    return out


COLUMNS = ('depth', 'pelv', 'chest', 'head', 'pitch', 'knee', 'hands', 'slide', 'steps', 'speed',
           'down', 'up', 'over', 'lean', 'sunk', 'fall')


def fmt(v):
    if isinstance(v, float):
        return 'nan' if math.isnan(v) else ('%.2f' % v if abs(v) < 2 else '%.1f' % v)
    return str(v)


def main():
    args = sys.argv[1:]
    pattern, as_json = None, '--json' in args
    args = [a for a in args if a != '--json']
    if '--filter' in args:
        i = args.index('--filter')
        pattern = re.compile(args[i + 1])
        del args[i:i + 2]
    if not args:
        sys.exit(__doc__)
    results = {}
    for run in args:
        rows, sneaking = load(run)
        for test in sorted(rows):
            if not any(sneaking[test].get(s) for s in sneaking[test] if s != '_moving'):
                continue
            if pattern and not pattern.search(test):
                continue
            results.setdefault(test, []).append((run, case_metrics(rows[test], sneaking[test])))
    if as_json:
        print(json.dumps({t: [m for _, m in v] for t, v in results.items()}, default=str))
        return
    print('%-46s ' % 'case' + ' '.join('%7s' % c for c in COLUMNS))
    for test, runs in results.items():
        for run, m in runs:
            label = test.replace('PLAY-01 ', '')[:46]
            print('%-46s ' % label + ' '.join('%7s' % fmt(m.get(c, '')) for c in COLUMNS))


if __name__ == '__main__':
    main()
