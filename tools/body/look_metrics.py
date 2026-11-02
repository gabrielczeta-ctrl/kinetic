"""Goal checks for the LOOK cases: height against vanilla, feet/shins
clipping, feet keeping up.

usage: look_metrics.py RUN_DIR [RUN_DIR ...]

Per look case (moving segments 'steady'/'travel'):
  pelvis   median physical pelvis height; vs vanilla = against the vanilla
           loadout's same case (the animation without the ragdoll), units
  feet     closest distance between the two foot bodies, and the share of
           moving frames under 7 units (touching/clipping)
  shins    the same for the calves (under 6 units)
  trail    95th percentile of how far a planted foot is behind the pelvis
           sideways-or-back against the travel (legs not keeping up), units
  track    median |pelvis speed - commanded body speed| (units/s)
"""
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

MOVING = ('steady', 'travel')


def frames_by_case(run):
    cases = defaultdict(list)
    for p in sorted(Path(run).rglob('*.player-diagnostics.jsonl.gz')):
        with gzip.open(p, 'rt') as f:
            for line in f:
                r = json.loads(line)
                if r.get('kind') == 'frame' and ' look ' in r.get('test', '') and 'bodies' in r.get('state', {}):
                    cases[r['test']].append(r)
    return cases


def metrics(fr):
    B = lambda r, n: np.array(r['state']['bodies'][n]['position'])
    moving = [r for r in fr if r.get('segment') in MOVING]
    if len(moving) < 10:
        return None
    pel = np.array([B(r, 'pelvis') for r in moving])
    lf = np.array([B(r, 'left_foot') for r in moving])
    rf = np.array([B(r, 'right_foot') for r in moving])
    lc = np.array([B(r, 'left_calf') for r in moving])
    rc = np.array([B(r, 'right_calf') for r in moving])
    feet = np.linalg.norm(lf - rf, axis=1)
    shins = np.linalg.norm(lc - rc, axis=1)
    el = np.array([r['elapsed'] for r in moving])
    vel = np.diff(pel[:, :2], axis=0) / np.maximum(np.diff(el), 1e-3)[:, None]
    trail = []
    for i in range(1, len(moving)):
        v = vel[i - 1]
        s = np.linalg.norm(v)
        if s < 20:
            continue
        d = v / s
        for foot, key in ((lf, 'leftFoot'), (rf, 'rightFoot')):
            if moving[i]['state']['support'][key]:
                trail.append(-(foot[i, :2] - pel[i, :2]) @ d)
    speed = np.linalg.norm(vel, axis=1)
    return dict(pelvis=float(np.median(pel[:, 2])), feet_min=float(feet.min()), feet_touch=float((feet < 7).mean()),
                shins_min=float(shins.min()), shins_touch=float((shins < 6).mean()),
                trail=float(np.percentile(trail, 95)) if trail else float('nan'),
                speed=float(np.median(speed)))


def main(runs):
    for run in runs:
        cases = frames_by_case(run)
        res = {name: metrics(fr) for name, fr in cases.items()}
        vanilla = {re.sub(r'^PLAY-01 vanilla ', '', k): v for k, v in res.items() if ' vanilla ' in k and v}
        print('==', run)
        print('%-34s %7s %8s | %6s %6s | %6s %6s | %6s | %6s' % ('case', 'pelvis', 'vs van', 'feet', 'touch', 'shins',
                                                                'touch', 'trail', 'speed'))
        for name in sorted(res):
            m = res[name]
            if not m or ' vanilla ' in name:
                continue
            key = re.sub(r'^PLAY-01 \w+ ', '', name)
            van = vanilla.get(key)
            diff = m['pelvis'] - van['pelvis'] if van else float('nan')
            print('%-34s %7.1f %+8.1f | %6.1f %5.0f%% | %6.1f %5.0f%% | %6.1f | %6.0f' % (
                name[8:], m['pelvis'], diff, m['feet_min'], 100 * m['feet_touch'], m['shins_min'],
                100 * m['shins_touch'], m['trail'], m['speed']))


if __name__ == '__main__':
    main(sys.argv[1:])
