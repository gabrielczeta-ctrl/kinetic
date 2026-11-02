"""G8 hit-speed tuning (spec 8.2): peak tip speed of the user's recorded drags.

    tune_speed.py <session dir> [...]

For every held drag (press to release) in the manual recordings, the peak
physical tip speed. Prints per weapon the count, median and p90 of the
per-drag peaks, and the class maxima. The gate uses 0.8 x the p90 of the
strongest weapon of each class: light (fists, dagger), medium (longsword,
spear, staff), heavy (claymore, warhammer, battle axe).
"""
import collections
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates as G

CLASSES = {'light': ['fists', 'iron dagger'], 'medium': ['iron longsword', 'iron spear', 'wooden staff'],
           'heavy': ['iron claymore', 'iron warhammer', 'iron battle axe']}


def tip(t):
    w, h = G.body(t, 'weapon'), G.body(t, 'right_hand')
    if 'tip' in t and t.get('tip_valid', [0])[0]:
        return t['tip']
    if w and t.get('weapon_length'):
        return [w['physical'][i] + G.qrot(w['physical_q'], t['weapon_axis'])[i] * t['weapon_length'] * 0.5
                for i in range(3)]
    return h['physical'] if h else None


def main(paths):
    peaks = collections.defaultdict(list)
    for path in paths:
        ticks = {}
        for chunk in sorted(glob.glob(os.path.join(path, 'manual-recording', 'ticks-*.jsonl'))):
            for line in open(chunk, encoding='utf-8'):
                if line.startswith('{"kind":"tick"'):
                    try:
                        d = json.loads(line)
                    except ValueError:
                        continue
                    if d['bodies']:
                        ticks[d['tick']] = d
        ks = sorted(ticks)
        run, weapon = [], None
        for a, b in zip(ks, ks[1:]):
            ta, tb = ticks[a], ticks[b]
            if b == a + 1 and ta['held'] and tb['held']:
                pa, pb = tip(ta), tip(tb)
                if pa and pb:
                    run.append(math.dist(pa, pb) * 60)
                    weapon = tb.get('weapon') or 'fists'
            elif run:
                if len(run) > 10:
                    peaks[weapon].append(max(run))
                run = []
    out = {}
    for w, p in sorted(peaks.items()):
        p.sort()
        out[w] = dict(n=len(p), median=round(p[len(p) // 2]), p90=round(p[int(0.9 * (len(p) - 1))]))
        print('%-16s drags %3d  median peak %5d  p90 peak %5d u/s' % (w, len(p), out[w]['median'], out[w]['p90']))
    for c, ws in CLASSES.items():
        best = max((out[w]['p90'] for w in ws if w in out), default=None)
        print('%-7s strongest p90 %s -> gate %s' % (c, best, best and round(0.8 * best)))
    return out


if __name__ == '__main__':
    main(sys.argv[1:])
