"""Summarise test_play01_walking runs: falls, lean, sway, speed tracking.

usage: walk_report.py RUN_DIR [RUN_DIR ...]   (compares runs side by side)
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


def horizontal(v):
    return (v[0], v[1]) if isinstance(v, list) and len(v) >= 2 else None


def summarise(run):
    tests = defaultdict(lambda: dict(n=0, fell=False, fall_t=None, up_min=1.0, lean=[], track=[], speed=[],
                                     target=[], height=[], t0=None, impact_com=None, last_com=None, swing=[], opp_spin=[], opp_tilt=[], drift=[], last_pos=None, last_t=None, chest=[]))
    for r in records(run):
        c = (r.get('state') or r).get('controller') if isinstance(r.get('state') or r, dict) else None
        test = r.get('test')
        if r.get('kind') == 'impact' and test:
            tests[test]['impact_pending'] = True
        if not test or not isinstance(c, dict) or 'mode' not in c:
            continue
        s = tests[test]
        s['n'] += 1
        com = (r.get('state') or r).get('centerOfMass') if isinstance(r.get('state') or r, dict) else None
        if isinstance(com, list):
            if s.pop('impact_pending', False):
                s['impact_com'] = com
                s['impact_t'] = r.get('simulation_time', 0.0)
            s['last_com'] = com
        t = r.get('simulation_time', 0.0)
        if s['t0'] is None:
            s['t0'] = t
        mode = str(c.get('mode', '')).lower()
        if mode in FALL_MODES and not s['fell']:
            s['fell'], s['fall_t'] = True, t - s['t0']
        up = c.get('torsoUpDot')
        if isinstance(up, (int, float)):
            s['up_min'] = min(s['up_min'], up)
            s['lean'].append(math.degrees(math.acos(max(-1.0, min(1.0, up)))))
        v, target = horizontal(c.get('centerOfMassVelocity')), horizontal(c.get('targetHorizontalVelocity'))
        if v and target and not s['fell']:
            tspeed = math.hypot(*target)
            s['speed'].append(math.hypot(*v))
            s['target'].append(tspeed)
            if tspeed > 5:
                s['track'].append(math.hypot(v[0] - target[0], v[1] - target[1]))
        bodies = (r.get('state') or {}).get('bodies') if isinstance(r.get('state'), dict) else None
        pelvis = bodies.get('pelvis') if isinstance(bodies, dict) else None
        if pelvis and isinstance(pelvis.get('position'), list) and isinstance(pelvis.get('targetPosition'), list)                 and not s['fell'] and s.get('impact_com') is None:
            p, q = pelvis['position'], pelvis['targetPosition']
            s['swing'].append(math.hypot(p[0] - q[0], p[1] - q[1]))
        # Capsule motion with no movement input (runaway check): skip the first
        # 1.5 s after an impact, when stumble steps move the capsule on purpose.
        pos, inp = r.get('player_position'), r.get('input') or {}
        if isinstance(pos, list) and s['last_pos'] is not None and t > s['last_t']:
            idle = abs(inp.get('movement', 0) or 0) < 0.1 and abs(inp.get('side_movement', 0) or 0) < 0.1
            since = t - s['impact_t'] if s.get('impact_t') is not None else 99
            if idle and since > 1.5:
                s['drift'].append(math.hypot(pos[0] - s['last_pos'][0], pos[1] - s['last_pos'][1]) / (t - s['last_t']))
        if isinstance(pos, list):
            s['last_pos'], s['last_t'] = pos, t
        opp = r.get('opponent')
        if isinstance(opp, dict) and isinstance(opp.get('angularVelocity'), list):
            s['opp_spin'].append(math.degrees(math.sqrt(sum(x * x for x in opp['angularVelocity']))))
            if isinstance(opp.get('torsoUpDot'), (int, float)):
                s['opp_tilt'].append(math.degrees(math.acos(max(-1.0, min(1.0, opp['torsoUpDot'])))))
        chest = bodies.get('spine2') if isinstance(bodies, dict) else None
        if chest and isinstance(chest.get('position'), list) and isinstance(chest.get('targetPosition'), list)                 and not s['fell'] and s.get('impact_com') is None:
            s['chest'].append(math.dist(chest['position'], chest['targetPosition']))
        h = c.get('pelvisHeight')
        if isinstance(h, (int, float)) and h > 0 and not s['fell']:
            s['height'].append(h)
    return tests


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


def std(xs):
    if len(xs) < 2:
        return float('nan')
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def row(s):
    lean = sorted(s['lean'])
    p90 = lean[int(0.9 * (len(lean) - 1))] if lean else float('nan')
    fall = f"FALL@{s['fall_t']:.1f}s" if s['fell'] else 'ok'
    moved = ''
    if s.get('impact_com') and s.get('last_com'):
        moved = f" | pushed {math.hypot(s['last_com'][0] - s['impact_com'][0], s['last_com'][1] - s['impact_com'][1]):5.1f} u"
    return (f"{fall:>10} lean p90 {p90:5.1f} max {max(lean) if lean else float('nan'):5.1f} "
            f"| track {mean(s['track']):5.1f} u/s (tgt {mean([x for x in s['target'] if x > 5]):5.1f}) "
            f"| hip-height sd {std(s['height']):4.1f}"
            f" | idle drift max {max(s['drift']) if s['drift'] else 0:5.1f} u/s"
            f" | chest off rms {math.sqrt(mean([x * x for x in s['chest']])) if s['chest'] else float('nan'):4.1f}"
            f" | swing rms {math.sqrt(mean([x * x for x in s['swing']])) if s['swing'] else float('nan'):4.1f}"
            f" max {max(s['swing']) if s['swing'] else float('nan'):4.1f}" + moved
            + (f" | opponent spin p90 {sorted(s['opp_spin'])[int(0.9 * (len(s['opp_spin']) - 1))]:5.0f} max {max(s['opp_spin']):5.0f} deg/s"
               f" tilt max {max(s['opp_tilt']) if s['opp_tilt'] else float('nan'):4.1f}" if s['opp_spin'] else ''))


def main(runs):
    summaries = [(run, summarise(run)) for run in runs]
    names = sorted(set().union(*[set(s) for _, s in summaries]))
    for name in names:
        print(name)
        for run, s in summaries:
            print(f"  {Path(run).name:>22}: " + (row(s[name]) if name in s else 'missing'))
    for run, s in summaries:
        falls = sum(1 for x in s.values() if x['fell'])
        lean = [v for x in s.values() for v in x['lean']]
        track = [v for x in s.values() for v in x['track']]
        swing = [v for x in s.values() for v in x['swing']]
        chest = [v for x in s.values() for v in x['chest']]
        print(f"CHEST {Path(run).name}: chest-vs-animation offset rms {math.sqrt(mean([v * v for v in chest])) if chest else float('nan'):.1f} u")
        print(f"TOTAL {Path(run).name}: {len(s)} tests, {falls} falls, mean lean {mean(lean):.1f} deg, "
              f"swing rms {math.sqrt(mean([v * v for v in swing])) if swing else float('nan'):.1f} u, "
              f"mean tracking error {mean(track):.1f} u/s")


if __name__ == '__main__':
    main(sys.argv[1:])
