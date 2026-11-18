"""Steps around direction changes, read from the leg trace (K2_TURN_STEP).

usage: turn_report.py LOG|RUN_DIR [...] [--json OUT] [--actor NAME]

Reads RAGDOLL_FOOTFALL (OPENMW_RAGDOLL_FOOTFALL_TRACE=1), RAGDOLL_LOAD and the
RAGDOLL_FOOTFALL_TURN_STEP / _TURN_CUT / _LIFT events of one actor (the
player) from a game log, a .log.gz, or every shard log of a run directory.
One trace row pair is one physics step (1/60 s).

Definitions (the 11-16 21:11 bookmark's, findings-bookmarks-2026-11-16-2111):
  change   within 0.6 s of the command turning more than 30 deg in 0.15 s, or
           of a start from rest; steady = no such change for 0.6 s. Command
           over 30 u/s only.
  lift     the highest the physical foot rose over the straight line from
           its pin to its target in a swing; goal = the same for the swing
           goal Footfall asked for; reached = lift / goal.
  skim     a swing whose lift stayed under 3 u.
  slide    a planted foot's speed (u/s); rootH the root's horizontal force
           (body weights); crossed the share of moving steps with the left
           foot right of the right one across the facing.
--json writes the swings and the per-step series (for a showcase page).
"""
import gzip
import json
import math
import re
import sys
from pathlib import Path

FF = re.compile(
    r'RAGDOLL_FOOTFALL side=(\d) swing=(\d) phase=([\d.-]+) foot=([-\d.]+),([-\d.]+),([-\d.]+) '
    r'pin=([-\d.]+),([-\d.]+),([-\d.]+) target=([-\d.]+),([-\d.]+),([-\d.]+) goal=([-\d.]+),([-\d.]+),([-\d.]+) '
    r'pelvis=([-\d.]+),([-\d.]+) speed=([\d.]+).*? yaw=([-\d.]+) clock=([\d.]+) cmd=([-\d.]+),([-\d.]+)')
LOAD = re.compile(r'RAGDOLL_LOAD .*?left=([-\d.]+) right=([-\d.]+) rootV=([-\d.]+) rootH=([-\d.]+) '
                  r'slipL=([-\d.]+) slipR=([-\d.]+) pelvisH=([-\d.]+)')


def lines_of(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8', errors='replace') as handle:
        yield from handle


def logs_of(arg):
    path = Path(arg)
    if path.is_file():
        return [path]
    found = sorted(path.glob('shard-*/*.log.gz'))
    return [f for f in found if 'player-diagnostics' not in f.name]


def angle(a, b):
    return abs(math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])))


def median(values):
    values = sorted(values)
    return values[len(values) // 2] if values else float('nan')


def read(paths, actor):
    """Steps (both feet's rows, the command, the load) and the event counts."""
    tag = 'actor="%s"' % actor
    steps, events, pending, load = [], {}, None, None
    for path in paths:
        for line in lines_of(path):
            if 'RAGDOLL_' not in line or tag not in line:
                continue
            if 'RAGDOLL_FOOTFALL side=' in line:
                m = FF.search(line)
                if not m:
                    continue
                g = m.groups()
                row = dict(swing=g[1] == '1', phase=float(g[2]), foot=tuple(map(float, g[3:6])),
                           pin=tuple(map(float, g[6:9])), target=tuple(map(float, g[9:12])),
                           goal=tuple(map(float, g[12:15])), pelvis=(float(g[15]), float(g[16])),
                           yaw=float(g[18]), cmd=(float(g[20]), float(g[21])))
                if g[0] == '0':
                    pending = row
                elif pending is not None:
                    steps.append(dict(feet=(pending, row), cmd=pending['cmd'], yaw=pending['yaw'], load=load,
                                      index=len(steps)))
                    pending, load = None, None
            elif 'RAGDOLL_LOAD ' in line:
                m = LOAD.search(line)
                if m:
                    load = tuple(map(float, m.groups()))
            else:
                m = re.search(r'RAGDOLL_FOOTFALL_(TURN_STEP \w+|TURN_CUT|LIFT)\b(.*)', line)
                if m:
                    kind = m.group(1)
                    if kind == 'TURN_CUT' and 'kept=1' in m.group(2):
                        kind = 'TURN_CUT kept'
                    events[kind] = events.get(kind, 0) + 1
    return steps, events


def analyse(steps):
    dt = 1.0 / 60.0
    history, last_change = [], -1e9
    for step in steps:
        t = step['index'] * dt
        cmd = step['cmd']
        speed = math.hypot(*cmd)
        history.append((t, cmd, speed))
        while history and t - history[0][0] > 0.15:
            history.pop(0)
        old = history[0]
        if speed <= 30 or old[2] <= 30 or angle(old[1], cmd) > 30:
            last_change = t
        step['t'], step['speed'], step['since'] = t, speed, t - last_change
        yaw = step['yaw']
        step['rel'] = angle((math.sin(yaw), math.cos(yaw)), cmd) if speed > 30 else None
    swings, open_swings = [], {}
    for step in steps:
        for side in (0, 1):
            foot = step['feet'][side]
            if foot['swing']:
                phase = max(0.0, min(1.0, foot['phase']))
                base = foot['pin'][2] + (foot['target'][2] - foot['pin'][2]) * phase
                lift, goal = foot['foot'][2] - base, foot['goal'][2] - base
                foot['over'], foot['goal_over'] = lift, goal
                swing = open_swings.get(side)
                if swing is None:
                    swing = open_swings[side] = dict(side=side, t0=step['t'], lift=lift, goal=goal,
                                                     since=step['since'], speed=step['speed'], rel=step['rel'],
                                                     start=foot['foot'][:2])
                swing['lift'], swing['goal'] = max(swing['lift'], lift), max(swing['goal'], goal)
                swing['since'] = min(swing['since'], step['since'])
                swing['end'], swing['t1'] = foot['foot'][:2], step['t']
            else:
                foot['over'] = foot['goal_over'] = 0.0
                if side in open_swings:
                    swing = open_swings.pop(side)
                    swing['time'] = swing['t1'] - swing['t0'] + dt
                    swing['travel'] = math.dist(swing['start'], swing['end'])
                    swings.append(swing)
    return [s for s in swings if s['speed'] > 30]


def row(name, group):
    if len(group) < 3:
        return None
    goal = median([s['goal'] for s in group])
    lift = median([s['lift'] for s in group])
    return dict(name=name, n=len(group), lift=lift, goal=goal,
                reached=median([s['lift'] / max(0.1, s['goal']) for s in group]),
                skim=100.0 * sum(s['lift'] < 3 for s in group) / len(group),
                time=median([s['time'] for s in group]), travel=median([s['travel'] for s in group]))


def table(swings):
    rows = [row('steady', [s for s in swings if s['since'] >= 0.6]),
            row('change', [s for s in swings if s['since'] < 0.6])]
    change = [s for s in swings if s['since'] < 0.6]
    for low, high, name in ((0, 22.5, 'forward'), (22.5, 67.5, 'diag fwd'), (67.5, 112.5, 'sideways'),
                            (112.5, 157.5, 'diag back'), (157.5, 181, 'backward')):
        rows.append(row('  ' + name, [s for s in change if s['rel'] is not None and low <= s['rel'] < high]))
    for low, high in ((30, 140), (140, 300), (300, 5000)):
        rows.append(row('  %d-%d u/s' % (low, high), [s for s in change if low <= s['speed'] < high]))
    for low, high in ((0, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 0.7)):
        rows.append(row('  swing %.1f-%.1f s' % (low, high), [s for s in change if low <= s['time'] < high]))
    return [r for r in rows if r]


def body(steps):
    """Measures over the steps, by condition."""
    out = {}
    for name in ('change', 'steady'):
        chosen = [s for s in steps if s['speed'] > 30 and ((s['since'] < 0.6) == (name == 'change'))]
        slide, crossed, root, pelvis, legs = [], 0, [], [], []
        previous = None
        for step in chosen:
            if previous is not None and step['index'] == previous['index'] + 1:
                for side in (0, 1):
                    a, b = previous['feet'][side], step['feet'][side]
                    if not a['swing'] and not b['swing']:
                        slide.append(math.dist(a['foot'][:2], b['foot'][:2]) * 60.0)
            previous = step
            yaw = step['yaw']
            right = (math.cos(yaw), -math.sin(yaw))
            left_foot, right_foot = step['feet'][0]['foot'], step['feet'][1]['foot']
            across = (left_foot[0] - right_foot[0]) * right[0] + (left_foot[1] - right_foot[1]) * right[1]
            crossed += across > 0
            if step['load']:
                legs.append(step['load'][0] + step['load'][1])
                root.append(step['load'][3])
                pelvis.append(step['load'][6])
        if len(chosen) < 30:
            continue
        pelvis.sort()
        out[name] = dict(seconds=len(chosen) / 60.0, slide=median(slide),
                         slide_over_40=100.0 * sum(v > 40 for v in slide) / max(1, len(slide)),
                         crossed=100.0 * crossed / len(chosen), root_h=median(root), legs=median(legs),
                         pelvis_low=pelvis[len(pelvis) // 20] if pelvis else float('nan'),
                         pelvis=median(pelvis))
    return out


def series(steps):
    """Per-step series from the first moving step: for a page's figures."""
    first = next((s['index'] for s in steps if s['speed'] > 30), 0)
    out = []
    for step in steps[max(0, first - 30):]:
        feet = step['feet']
        out.append([round(step['t'], 3), round(step['cmd'][0], 1), round(step['cmd'][1], 1), round(step['yaw'], 3),
                    int(feet[0]['swing']), round(feet[0]['over'], 2), round(feet[0]['goal_over'], 2),
                    int(feet[1]['swing']), round(feet[1]['over'], 2), round(feet[1]['goal_over'], 2),
                    round(step['since'], 2),
                    round(feet[0]['foot'][0], 1), round(feet[0]['foot'][1], 1),
                    round(feet[1]['foot'][0], 1), round(feet[1]['foot'][1], 1),
                    round(feet[0]['pelvis'][0], 1), round(feet[0]['pelvis'][1], 1)])
    return out


def main():
    args = sys.argv[1:]
    out_json, actor = None, 'player'
    for flag in ('--json', '--actor'):
        if flag in args:
            i = args.index(flag)
            if flag == '--json':
                out_json = args[i + 1]
            else:
                actor = args[i + 1]
            del args[i:i + 2]
    if not args:
        sys.exit(__doc__)
    result = {}
    for arg in args:
        paths = logs_of(arg)
        steps, events = read(paths, actor)
        if not steps:
            print('%s: no leg trace (OPENMW_RAGDOLL_FOOTFALL_TRACE=1)' % arg)
            continue
        swings = analyse(steps)
        rows, measures = table(swings), body(steps)
        print('%s: %d steps, %d swings moving' % (arg, len(steps), len(swings)))
        print('  %-20s %5s %6s %6s %8s %6s %8s %7s' % ('group', 'n', 'lift', 'goal', 'reached', 'skim', 'swing s', 'travel'))
        for r in rows:
            print('  %-20s %5d %6.1f %6.1f %8.2f %5.0f%% %8.2f %7.0f' % (
                r['name'], r['n'], r['lift'], r['goal'], r['reached'], r['skim'], r['time'], r['travel']))
        for name, m in measures.items():
            print('  %-7s %5.1f s: planted slide %4.1f u/s (%2.0f%% over 40), crossed %4.1f%%, rootH %.2f, legs %.2f, '
                  'pelvis %.1f (low %.1f)' % (name, m['seconds'], m['slide'], m['slide_over_40'], m['crossed'],
                                               m['root_h'], m['legs'], m['pelvis'], m['pelvis_low']))
        print('  events: %s' % ', '.join('%s %d' % kv for kv in sorted(events.items())))
        result[arg] = dict(rows=rows, body=measures, events=events,
                           swings=[dict(t0=round(s['t0'], 3), time=round(s['time'], 3), lift=round(s['lift'], 2),
                                        goal=round(s['goal'], 2), since=round(s['since'], 2), side=s['side'],
                                        speed=round(s['speed'], 1), rel=None if s['rel'] is None else round(s['rel'], 1))
                                   for s in swings],
                           series=series(steps))
    if out_json:
        Path(out_json).write_text(json.dumps(result, separators=(',', ':')))


if __name__ == '__main__':
    main()
