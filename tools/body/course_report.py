"""Test course scores (PLAN.md, definition of done): the physical body against
the animation, per loadout and course segment.

usage: python course_report.py RUN_DIR [--csv OUT]

The target positions in the diagnostics are the drive pose, which for the
pelvis and chest is the animation (the V ghost) to within half a unit. Each
segment is scored after its first second (transition segments over all of
it). Columns:
  sink    physical minus animation height of the chest (spine2): median, p10
  bob     chest height swing, physical / animation (peak to peak)
  stepT   step time variation (coefficient of variation, %), from foot height
  stepTc  the same from contact onsets (running reads far truer; not a bar yet)
  stepL   step length variation (%)
  side    sideways landing spread (sd of each foot's line, units)
  cross   landings on the wrong side of the other foot
  scuff   foot contacts under 0.1 s per second
Steps are landings detected from the foot's height, not from contact.
  hand    hand error against the animation in the chest frame, p90 (units)
  shake   hand jitter: RMS of the error's second difference (units/frame^2)
  gallop  slower / faster of the left-to-right and right-to-left landing intervals
  push    how far behind the pelvis a foot is when it lifts (units)
  both    share of time both feet are down
Pass bars: sink >= -2 walk / -3 run (median) and >= -4 / -6 (p10); stepT and
stepL < 5; side < 2 walk / 3 run; cross 0; scuff < 0.3; hand < 5 (weapon
hands are held by the weapon controller and not scored); shake < 1.0;
gallop < 1.15; running: push > 25, both < 8%.
"""
import glob
import gzip
import json
import sys
from collections import defaultdict

import numpy as np

TRANSITIONS = ('walk to run', 'run to walk', 'stop', 'stand')


def rows_by_case(run):
    by = defaultdict(list)
    for p in sorted(glob.glob(run + '/shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(p, 'rt') as f:
            for line in f:
                r = json.loads(line)
                seg = r.get('segment') or ''
                if r.get('kind') == 'frame' and seg.startswith('c ') and 'bodies' in r.get('state', {}):
                    loadout = r['test'].split(' ')[1]
                    by[(loadout, seg[2:])].append(r)
    return by


def P(b, n, k='position'):
    return np.array(b[n][k])


def R(b, n, k='rotation'):
    return np.array([b[n][k]['x'], b[n][k]['y'], b[n][k]['z']])


def score(rows, seg):
    t0 = rows[0]['elapsed']
    steady = rows if seg in TRANSITIONS else [r for r in rows if r['elapsed'] - t0 >= 1.0]
    if len(steady) < 20:
        return None
    running = 'run' in seg and seg != 'run to walk'
    B = [r['state']['bodies'] for r in steady]
    sink = np.array([P(b, 'spine2')[2] - P(b, 'spine2', 'targetPosition')[2] for b in B])
    z = np.array([P(b, 'spine2')[2] for b in B])
    tz = np.array([P(b, 'spine2', 'targetPosition')[2] for b in B])
    bob = (np.percentile(z, 95) - np.percentile(z, 5)) / max(np.percentile(tz, 95) - np.percentile(tz, 5), 0.5)

    # Steps: touchdowns (support onsets held >= 0.1 s) along the travel line.
    el = np.array([r['elapsed'] for r in steady])
    pel = np.array([P(b, 'pelvis')[:2] for b in B])
    travel = pel[-1] - pel[0]
    moving = np.linalg.norm(travel) > 20
    along = travel / np.linalg.norm(travel) if moving else np.array([0.0, 1.0])
    left = np.array([-along[1], along[0]])
    touch = []
    scuffs = 0
    for foot, key in (('left_foot', 'leftFoot'), ('right_foot', 'rightFoot')):
        # Scuffs from contact: touches shorter than 0.1 s.
        sup = [bool(r['state']['support'][key]) for r in steady]
        i = 0
        while i < len(sup):
            if sup[i]:
                j = i
                while j < len(sup) and sup[j]:
                    j += 1
                if el[j - 1] - el[i] < 0.1:
                    scuffs += 1
                i = j
            else:
                i += 1
        # Steps from the foot's height: a landing is the foot coming back down
        # under 1.5 units after rising above 2.5 over its stance level, so a
        # flickering contact is not a step.
        zf = np.array([P(b, foot)[2] for b in B])
        ground = np.percentile(zf, 10)
        lifted = False
        for i in range(len(zf)):
            h = zf[i] - ground
            if not lifted and h > 2.5:
                lifted = True
            elif lifted and h < 1.5:
                lifted = False
                touch.append((el[i], foot, P(B[i], foot)[:2]))
    # Gallop and push-off (the user's words, 2026-09-27: "it looks like it's
    # galloping", "the leg doesn't go back"): the left-to-right against the
    # right-to-left landing interval, how far behind the pelvis a foot is when
    # it lifts, and the share of time both feet are down.
    lifts = []
    downs = {}
    for foot in ('left_foot', 'right_foot'):
        zf = np.array([P(b, foot)[2] for b in B])
        ground = np.percentile(zf, 10)
        up = zf[0] - ground > 2.5
        for i in range(1, len(zf)):
            h = zf[i] - ground
            if up and h < 1.5:
                up = False
            elif not up and h > 2.5:
                up = True
                lifts.append(-(P(B[i], foot) - P(B[i], 'pelvis'))[:2] @ along)
        downs[foot] = (zf - ground) < 1.5
    both_down = float(np.mean(downs['left_foot'] & downs['right_foot']))
    pushoff = float(np.median(lifts)) if lifts else np.nan
    touch.sort(key=lambda t: t[0])
    # Step timing from contact (2026-09-29): onsets held >= 0.08 s, a foot's
    # re-touch before the other foot lands dropped. Running, the height rule
    # above missed and split landings (a foot touching 0.1 s after its swing
    # ended, bouncing), reading ~40% where contact timing spreads ~12%.
    onsets = []
    for key in ('leftFoot', 'rightFoot'):
        sup = [bool(r['state']['support'][key]) for r in steady]
        i = 0
        while i < len(sup):
            if sup[i] and (i == 0 or not sup[i - 1]):
                j = i
                while j < len(sup) and sup[j]:
                    j += 1
                if i > 0 and el[j - 1] - el[i] >= 0.08:
                    onsets.append((el[i], key))
                i = j
            else:
                i += 1
    onsets.sort()
    alternating = [o for k, o in enumerate(onsets) if k == 0 or o[1] != onsets[k - 1][1]]
    stepTc = np.nan
    if moving and len(alternating) >= 4:
        gaps = np.diff([o[0] for o in alternating])
        stepTc = 100 * gaps.std() / gaps.mean()
    stepT = stepL = side = gallop = np.nan
    cross = 0
    if moving and len(touch) >= 4:
        times = np.diff([t[0] for t in touch])
        stepT = 100 * times.std() / times.mean()
        lengths = [abs((b[2] - a[2]) @ along) for a, b in zip(touch, touch[1:]) if a[1] != b[1]]
        if len(lengths) >= 3:
            stepL = 100 * np.std(lengths) / np.mean(lengths)
        # Sideways: each foot's landings against a line fitted along the travel.
        spreads = []
        for foot in ('left_foot', 'right_foot'):
            pts = np.array([t[2] for t in touch if t[1] == foot])
            if len(pts) >= 3:
                s = pts @ left
                a = pts @ along
                fit = np.polyfit(a, s, 1)
                spreads.append((s - np.polyval(fit, a)).std())
        side = max(spreads) if spreads else np.nan
        lr = [b[0] - a[0] for a, b in zip(touch, touch[1:]) if a[1] == 'left_foot' and b[1] == 'right_foot']
        rl = [b[0] - a[0] for a, b in zip(touch, touch[1:]) if a[1] == 'right_foot' and b[1] == 'left_foot']
        if lr and rl:
            gallop = max(np.median(lr), np.median(rl)) / max(min(np.median(lr), np.median(rl)), 1e-3)
        for a, b in zip(touch, touch[1:]):
            if a[1] != b[1]:
                sign = 1 if b[1] == 'left_foot' else -1
                if sign * ((b[2] - a[2]) @ left) < 0:
                    cross += 1

    # Hands in the chest frame against the animation.
    hands = {}
    for h in ('left_hand', 'right_hand'):
        e = []
        for b in B:
            Ra, Rt = R(b, 'spine2'), R(b, 'spine2', 'targetRotation')
            e.append(Ra @ (P(b, h) - P(b, 'spine2')) - Rt @ (P(b, h, 'targetPosition') - P(b, 'spine2', 'targetPosition')))
        e = np.array(e)
        err = np.percentile(np.linalg.norm(e, axis=1), 90)
        # Jitter only: the error's second difference (a smooth lag has almost
        # none; a hand buzzing from frame to frame has a lot).
        shake = np.sqrt(np.mean(np.sum(np.diff(e, n=2, axis=0) ** 2, axis=1)))
        hands[h] = (err, shake)
    dur = el[-1] - el[0]
    return dict(sink=np.median(sink), sink10=np.percentile(sink, 10), bob=bob, stepT=stepT, stepTc=stepTc, stepL=stepL,
                side=side, cross=cross, gallop=gallop, pushoff=pushoff, both=both_down, scuff=scuffs / max(dur, 0.1), hands=hands, running=running, moving=moving)


def main():
    run = sys.argv[1]
    by = rows_by_case(run)
    order = ['stand', 'walk', 'run', 'walk again', 'strafe left', 'strafe right', 'backpedal', 'walk to run',
             'run again', 'run strafe left', 'run strafe right', 'run turn', 'run to walk', 'stop']
    loadouts = sorted({k[0] for k in by})
    fails_total = 0
    table = []
    print('%-9s %-16s %11s %5s %6s %6s %6s %5s %5s %5s %11s %11s %6s %5s %5s  fails' % (
        'loadout', 'segment', 'sink p50/10', 'bob', 'stepT', 'stepTc', 'stepL', 'side', 'cross', 'scuff', 'L hand/shk', 'R hand/shk',
        'gallop', 'push', 'both'))
    for lo in loadouts:
        weapon = lo not in ('fists', 'unarmed')
        for seg in order:
            if (lo, seg) not in by:
                continue
            s = score(by[(lo, seg)], seg)
            if not s:
                continue
            f = []
            bar = 3 if s['running'] else 2
            if s['sink'] < -bar:
                f.append('sink')
            if s['sink10'] < -2 * bar:
                f.append('sink10')
            if s['moving'] and seg not in TRANSITIONS:
                if not s['stepT'] < 5:
                    f.append('stepT')
                if not s['stepL'] < 5:
                    f.append('stepL')
                if not s['side'] < bar:
                    f.append('side')
                if s['cross']:
                    f.append('cross')
            if s['moving'] and seg not in TRANSITIONS:
                if not s['gallop'] < 1.15:
                    f.append('gallop')
                if s['running'] and not s['pushoff'] > 25:
                    f.append('push')
                if s['running'] and not s['both'] < 0.08:
                    f.append('both')
            if s['scuff'] >= 0.3:
                f.append('scuff')
            for h, (err, shk) in s['hands'].items():
                if weapon and h == 'right_hand':
                    continue
                if err >= 5:
                    f.append(h[0].upper() + 'hand')
                if shk >= 1.0:
                    f.append(h[0].upper() + 'shake')
            fails_total += len(f)
            row = {k: (bool(v) if isinstance(v, (bool, np.bool_)) else float(v) if isinstance(v, (int, float, np.floating)) else v) for k, v in s.items() if k != 'hands'}
            row.update(loadout=lo, segment=seg, fails=f, lhand=float(s['hands']['left_hand'][0]), rhand=float(s['hands']['right_hand'][0]))
            table.append(row)
            L, Rr = s['hands']['left_hand'], s['hands']['right_hand']
            print('%-9s %-16s %5.1f/%5.1f %5.1f %6.1f %6.1f %6.1f %5.1f %5d %5.2f %5.1f/%4.2f %5.1f/%4.2f %6.2f %5.1f %4.0f%%  %s' % (
                lo, seg, s['sink'], s['sink10'], s['bob'], s['stepT'], s['stepTc'], s['stepL'], s['side'], s['cross'], s['scuff'],
                L[0], L[1], Rr[0], Rr[1], s['gallop'], s['pushoff'], 100 * s['both'], ' '.join(f)))
        print()
    print('total fails:', fails_total)
    with open(run + '/course.json', 'w') as out:
        json.dump(table, out, indent=1)


if __name__ == '__main__':
    main()
