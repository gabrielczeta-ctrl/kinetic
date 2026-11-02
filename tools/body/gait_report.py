"""Gait quality per case, physical body against the clip (targetPosition).

usage: gait_report.py [--filter REGEX] [--cases] RUN_DIR [RUN_DIR ...]

Per foot touchdown (foot centre drops below floor + 2.5 after being above
floor + 5), on flat travel segments only:
  lead    foot minus clip foot along the travel direction (negative = trails)
  side    foot minus clip foot along the facing's left axis, signed toward the
          foot's own side (negative = drifted inward)
  xsw     clip swings during which the physical swing foot passes the pelvis
          midline (sideways, facing frame)
  cross   touchdowns with the feet crossed: (left - right) along the left axis
          below 0 (the clip never crosses)
  slide   horizontal speed of a planted foot (both feet down or the other foot
          lifted), mean u/s
  stall   share of steady travel time (after 1 s) with the centre of mass below
          40% of the commanded speed
  sway    pelvis lateral velocity RMS, physical / clip
  bob     pelvis height standard deviation, physical / clip
  cad     touchdowns per second, physical / clip
  pslide  horizontal speed of a foot while its contact flag is set, mean/median u/s
          (independent of the motor target)
  pstep   physical foot travel from lift to touchdown, mean units
Stops (segments named stop*): the most negative velocity along the previous
travel direction (rebound) and the time until the speed stays below 10 u/s.
"""
import gzip
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

TRAVEL = re.compile(r'^(travel|forward|start|restart|run|new heading|armed|steady|run .*|strafe|diagonal)$')
FALL = {'falling', 'fallen', 'gettingup', 'getting_up', 'blendback', 'blend_back'}


def frames(run):
    for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get('kind') == 'frame' and r.get('test'):
                    yield r


def h(v):
    return (v[0], v[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def std(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


class Case:
    def __init__(self):
        self.rows = []


def analyse(rows):
    out = defaultdict(list)
    rest, clip_floor = {}, {}
    idle = [r for r in rows if r['segment'] in ('idle', 'setup')][-30:] or rows[:30]
    for side in ('left_foot', 'right_foot'):
        zs = sorted(r['b'][side]['z'] for r in idle)
        rest[side] = zs[len(zs) // 2] if zs else 0
        tz = [r['b'][side]['tz'] for r in rows if r['travel']]
        clip_floor[side] = min(tz) if tz else 0
    # physical touchdowns (cadence) and clip touchdowns (lead, side, crossing)
    for key in ('phys', 'clip'):
        state = {s: 'down' for s in rest}
        inward = {s: 0 for s in rest}
        lift_at = {}
        inward_clip = {s: 0 for s in rest}
        count = 0
        travel_time = 0.0
        for r in rows:
            if not r['travel'] or r['fell']:
                continue
            travel_time += r['dt']
            for s in rest:
                if key == 'phys':
                    z, up, down = r['b'][s]['z'], rest[s] + 4, rest[s] + 2
                else:
                    z, up, down = r['b'][s]['tz'], clip_floor[s] + 5, clip_floor[s] + 1.5
                if state[s] == 'down' and z > up:
                    state[s] = 'up'
                    lift_at[s] = r['b'][s]['p']
                    inward[s] = 0
                    inward_clip[s] = 0
                elif state[s] == 'up' and key == 'clip' and z >= down:
                    sign = 1 if s == 'left_foot' else -1
                    if sign * dot(sub(r['b'][s]['p'], r['pel']), r['left']) < 0:
                        inward[s] = 1
                    if sign * dot(sub(r['b'][s]['t'], r['tpel']), r['left']) < 0:
                        inward_clip[s] = 1
                elif state[s] == 'up' and z < down:
                    state[s] = 'down'
                    count += 1
                    if key == 'phys' and lift_at.get(s) is not None:
                        out['pstep'].append(math.hypot(*sub(r['b'][s]['p'], lift_at[s])))
                    if key == 'clip':
                        out['xswing'].append(inward[s])
                        out['xswing_clip'].append(inward_clip[s])
                        b = r['b'][s]
                        d = sub(b['p'], b['t'])
                        out['lead'].append(dot(d, r['dir']))
                        sign = 1 if s == 'left_foot' else -1
                        out['side'].append(sign * dot(d, r['left']))
                        sep = dot(sub(r['b']['left_foot']['p'], r['b']['right_foot']['p']), r['left'])
                        out['cross'].append(1 if sep < 0 else 0)
        out['cad_' + key] = [count / travel_time] if travel_time > 1 else []
    for r in rows:
        if r['travel'] and not r['fell']:
            out['sink'].append(1 if min(r['b'][s]['z'] - rest[s] for s in rest) < -4 else 0)
    # contact slide: physical foot speed while its contact flag is set
    for r in rows:
        if r['travel'] and not r['fell']:
            for s in rest:
                if r['contact'].get(s):
                    out['pslide'].append(r['b'][s]['speed'])
    # plant slide: phys foot low and slow-moving clip foot (clip stance)
    for r in rows:
        if not r['travel'] or r['fell']:
            continue
        for s in rest:
            b = r['b'][s]
            if b['z'] < rest[s] + 2 and b['tspeed'] < 20:
                out['slide'].append(b['speed'])
    steady = [r for r in rows if r['travel'] and not r['fell'] and r['seg_t'] > 1.0 and r['cmd'] > 5]
    if steady:
        out['stall'] = [sum(1 for r in steady if r['speed'] < 0.4 * r['cmd']) / len(steady)]
        out['sway_phys'] = [rms([r['lat'] for r in steady])]
        out['sway_clip'] = [rms([r['tlat'] for r in steady])]
        out['bob_phys'] = [std([r['pz'] for r in steady])]
        out['bob_clip'] = [std([r['tpz'] for r in steady])]
    # stops
    prev_dir = None
    stop_rows = defaultdict(list)
    for r in rows:
        if r['travel'] and r['cmd'] > 5:
            prev_dir = r['dir']
        if r['segment'].startswith('stop') and prev_dir is not None and not r['fell']:
            stop_rows[r['segment']].append((r['seg_t'], dot(r['v'], prev_dir), math.hypot(*r['v'])))
    for seg, rs in stop_rows.items():
        out['rebound'].append(min(v for _, v, _ in rs))
        settle = 0.0
        for t, _, sp in rs:
            if sp >= 10:
                settle = t
        out['settle'].append(settle)
    return out


def load(run, pattern):
    cases = defaultdict(list)
    seg_start = {}
    for r in frames(run):
        test = r['test']
        if pattern and not re.search(pattern, test):
            continue
        st = r.get('state') or {}
        bodies = st.get('bodies') or {}
        c = st.get('controller') or {}
        if not all(k in bodies for k in ('left_foot', 'right_foot', 'pelvis')):
            continue
        seg = r.get('segment') or ''
        t = r.get('simulation_time', 0.0)
        if seg_start.get(test, (None,))[0] != seg:
            seg_start[test] = (seg, t)
        yaw = (r.get('input') or {}).get('gameplay_yaw', r.get('gameplay_yaw', 0.0)) or 0.0
        fwd = (math.sin(yaw), math.cos(yaw))
        left = (-fwd[1], fwd[0])
        tv = c.get('targetHorizontalVelocity') or [0, 0, 0]
        cmd = math.hypot(tv[0], tv[1])
        direction = (tv[0] / cmd, tv[1] / cmd) if cmd > 1 else fwd
        v = c.get('centerOfMassVelocity') or [0, 0, 0]
        pel = bodies['pelvis']
        prev = cases[test][-1] if cases[test] else None
        dt = t - prev['t'] if prev else 1 / 60
        row = dict(t=t, dt=dt if 0 < dt < 0.2 else 1 / 60, segment=seg, seg_t=t - seg_start[test][1],
                   travel=bool(TRAVEL.match(seg)) and 'stairs' not in test and 'jump' not in test,
                   fell=str(c.get('mode', '')).lower() in FALL, dir=direction, left=left, cmd=cmd,
                   v=h(v), speed=math.hypot(v[0], v[1]),
                   pz=pel['position'][2], tpz=pel['targetPosition'][2], pel=h(pel['position']), tpel=h(pel['targetPosition']), b={})
        pv = h(pel['linearVelocity'])
        row['lat'] = dot(pv, left) - dot(h(tv), left)
        if prev:
            tp = h(pel['targetPosition'])
            ptp = h(prev['b_tpel'])
            row['tlat'] = dot(sub(tp, ptp), left) / row['dt'] - dot(h(tv), left)
        else:
            row['tlat'] = 0.0
        row['b_tpel'] = pel['targetPosition']
        support = st.get('support') or {}
        row['contact'] = {'left_foot': bool(support.get('leftFoot')), 'right_foot': bool(support.get('rightFoot'))}
        for s in ('left_foot', 'right_foot'):
            b = bodies[s]
            p, tpos = b['position'], b['targetPosition']
            e = dict(p=h(p), t=h(tpos), z=p[2], tz=tpos[2], speed=math.hypot(*h(b['linearVelocity'])))
            if prev:
                e['tspeed'] = math.hypot(*sub(h(tpos), prev['b'][s]['t'])) / row['dt']
            else:
                e['tspeed'] = 0.0
            row['b'][s] = e
        if row['fell']:
            cases[test + '#fell'] = []
        cases[test].append(row)
    return {k: v for k, v in cases.items() if not k.endswith('#fell')}, {k[:-5] for k in cases if k.endswith('#fell')}


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


GROUPS = [('unarmed walk', r'^PLAY-01 unarmed (forward|backward|strafe|diagonal|start stop|moving turn|reversal)'),
          ('armed walk', r'^PLAY-01 warhammer (forward|backward|strafe|diagonal|start stop|moving turn|reversal)$'),
          ('armed fwd45', r'^PLAY-01 warhammer forward (cycle|activation|qualification)'),
          ('sheathed walk', r'^PLAY-01 sheathed'),
          ('unarmed run', r'^PLAY-01 unarmed (run straight|run turn|run strafe)'),
          ('armed run', r'^PLAY-01 warhammer (run straight|run turn|run strafe)')]


def summary(stats):
    def pct(xs, q):
        xs = sorted(xs)
        return xs[int(q * (len(xs) - 1))] if xs else float('nan')
    s = stats
    return ('td %3d sink %4.1f%% lead %6.1f side %5.1f cross %4.0f%% xsw %3.0f/%2.0f%% slide %5.1f stall %4.1f%% sway %4.1f/%4.1f bob %4.1f/%4.1f '
            'cad %3.1f/%3.1f rebound %6.1f settle %4.2f pslide %5.1f/%4.1f pstep %5.1f' % (
                len(s['lead']), 100 * mean(s['sink']), mean(s['lead']), mean(s['side']), 100 * mean(s['cross']) if s['cross'] else float('nan'), 100 * mean(s['xswing']) if s['xswing'] else float('nan'), 100 * mean(s['xswing_clip']) if s['xswing_clip'] else float('nan'),
                mean(s['slide']), 100 * mean(s['stall']), mean(s['sway_phys']), mean(s['sway_clip']),
                mean(s['bob_phys']), mean(s['bob_clip']), mean(s['cad_phys']), mean(s['cad_clip']),
                pct(s['rebound'], 0.5), pct(s['settle'], 0.5), mean(s['pslide']), pct(s['pslide'], 0.5), mean(s['pstep'])))


def main(argv):
    pattern, show_cases = None, False
    runs = []
    it = iter(argv)
    for a in it:
        if a == '--filter':
            pattern = next(it)
        elif a == '--cases':
            show_cases = True
        else:
            runs.append(a)
    for run in runs:
        cases, fell = load(run, pattern)
        print('==', run, '(%d cases, %d with falls excluded)' % (len(cases) + len(fell), len(fell)))
        per_case = {name: analyse(rows) for name, rows in cases.items()}
        for gname, gre in GROUPS:
            agg = defaultdict(list)
            for name, st in per_case.items():
                if re.search(gre, name):
                    for k, v in st.items():
                        agg[k] += v
            if agg:
                print('%-14s %s' % (gname, summary(agg)))
        if show_cases:
            for name in sorted(per_case):
                print('  %-60s %s' % (name[8:68], summary(per_case[name])))


if __name__ == '__main__':
    main(sys.argv[1:])
