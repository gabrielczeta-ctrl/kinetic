"""Stairs descent posture (user bookmark 10-01: walking down stairs "the body
tries to be upright and feet get shot forwards"; "we need to be able to lower
our pelvis in order to mantain balance to catch a longer step").

usage: python tools/body/stair_stance.py RUN_DIR [RUN_DIR ...]

Per stairs case (the runs' player diagnostics, as behaviour_report.py), with
one foot planted: the centre of mass's height over the planted foot (the
stance) descending (centre of mass > 10 u/s down, over the stairs) against the
same on level ground, the stance knee's bend (thigh -> calf -> foot, degrees)
and how far ahead the swing foot is of the centre of mass along the travel
when it lands (u). A body that stays upright descending shows the stance
height as high as on the flat and a straight stance knee.
"""
import gzip, json, math, sys
from collections import defaultdict
from pathlib import Path

STAIRS = ('stairs', 'STAIRS-01', 'STAIRJUMP-01')


def records(run):
    for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def pos(state, name):
    p = ((state.get('bodies') or {}).get(name) or {}).get('position')
    return p if isinstance(p, list) and len(p) >= 3 else None


def angle(a, b, c):
    u = [a[i] - b[i] for i in range(3)]
    v = [c[i] - b[i] for i in range(3)]
    nu = math.sqrt(sum(x * x for x in u)) or 1.0
    nv = math.sqrt(sum(x * x for x in v)) or 1.0
    d = max(-1.0, min(1.0, sum(u[i] * v[i] for i in range(3)) / (nu * nv)))
    return 180.0 - math.degrees(math.acos(d))  # 0 straight


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else float('nan')


def main():
    rows = defaultdict(lambda: defaultdict(list))
    for run in sys.argv[1:]:
        for r in records(run):
            test = r.get('test') or ''
            if not any(p in test for p in STAIRS) or r.get('kind') != 'frame':
                continue
            s = r.get('state')
            if not isinstance(s, dict) or not isinstance(s.get('controller'), dict):
                continue
            com = s.get('centerOfMass')
            v = s['controller'].get('centerOfMassVelocity')
            sup = s.get('support') or {}
            if not isinstance(com, list) or not isinstance(v, list):
                continue
            feet = [(k, pos(s, f), pos(s, c), pos(s, t)) for k, f, c, t in
                    (('leftFoot', 'left_foot', 'left_calf', 'left_thigh'),
                     ('rightFoot', 'right_foot', 'right_calf', 'right_thigh'))]
            planted = [f for f in feet if sup.get(f[0]) and f[1] and f[2] and f[3]]
            if len(planted) != 1:
                continue
            _, foot, calf, thigh = planted[0]
            height = com[2] - foot[2]
            knee = angle(thigh, calf, foot)
            key = 'down' if v[2] < -10.0 else ('level' if abs(v[2]) < 3.0 else None)
            if key is None:
                continue
            rows[test][key + '_h'].append(height)
            rows[test][key + '_k'].append(knee)
    print('%-44s %8s %8s %8s %8s %6s' % ('case', 'h down', 'h level', 'knee dn', 'knee lv', 'n dn'))
    for test, d in sorted(rows.items()):
        print('%-44s %8.1f %8.1f %8.1f %8.1f %6d' % (test[-44:], med(d['down_h']), med(d['level_h']),
              med(d['down_k']), med(d['level_k']), len(d['down_h'])))


if __name__ == '__main__':
    main()
