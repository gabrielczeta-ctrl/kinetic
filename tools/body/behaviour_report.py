"""PGC-03 behaviour report (2026-10-11): STAGGER-01, LEDGE-01, STAIRJUMP-01,
SLOPE-01 (and any walking case with a blow or impact).

usage: behaviour_report.py RUN_DIR [RUN_DIR ...] [--json OUT]

Per case, from its anchor - the blow/impact, else the first frame of the
segment named off / jump / down / stand / push / hit - over the next 4 s:
  fell     any fall mode (falling, fallen, getting up)
  steps    feet that leave the ground and land more than 8 u from where they left
  skid     travel of feet while in ground contact (sliding)
  slide    upright slide: the centre of mass's travel beyond the walk target
           while both feet are down and the torso is within 20 deg of upright
  steady   time until the centre of mass is within 30 u/s of the walk target
           and the torso within 12 deg of upright for 0.3 s (- = never)
  tilt     the largest torso tilt (deg)
  rise     the centre of mass's height at the end against the anchor (a
           mantle onto an 80-unit ledge rises ~80)
DROP-01 (the drop roll, PGC-03 C8b) over 7 s, in a second table, from the
touchdown (the first frame after the anchor where the centre of mass's
vertical speed rises above -100 u/s after having been below -300):
  td       the touchdown, s after the anchor (steady - td = steady from it)
  inv      the torso went past upside down (torsoUpDot under -0.3)
  rolled   the chest faced down (torsoFacingUpDot <= -0.6) and then up
           (>= +0.6) within 1.2 s: a forward roll ("tilt" reads ~180 on one)
  headv    the head's largest speed while it is below z 20 (u/s)
  jolt     the pelvis's largest upward change of vertical speed over 3
           frames, in g (627 u/s^2)
  travel   the centre of mass's horizontal travel from the touchdown on
  roll     the player's RAGDOLL_ROLL end results (openmw.log, by TEST_START)
           with the roll's time in s and the speed it left with (result:t:vh)
  knee/hip ACRO-01: the tuck, the largest knee fold (thigh -> calf -> foot
           centres) and hip fold (trunk against the thigh: 0 standing)
           in the 1.5 s from the touchdown, degrees; the reference rolls
           fold 131-142 / 147-173 (joint angles, so not equal measures)
  headz    the head's lowest centre height in that time (the floor is z 0)
The get-up table (ACRO-01 and knockdown cases, from the impact to the end):
  fallen   s to the first Fallen mode
  lie      s Fallen before the get-up starts
  rise     s from the get-up's start to Standing
  total    s from the impact to Standing (- = never)
  refall   falls after the first get-up started
  speed    the get-up clip speeds (RAGDOLL_GETUP_START, openmw.log)
With several runs, each case's values are listed per run, then the median.
'stairs up and back' and STAIRS-01 are read over the whole case (their
anchors - climb, stop - would cut the descent out of a 4 s window).

The stairs table (PGC-03 C9b; the stairs cases over the whole case, the
fixture's geometry below; the footfall trace from the shards' openmw.log,
so the runs need OPENMW_RAGDOLL_FOOTFALL_TRACE=1):
  falls    any fall mode
  edge%    feet's first contacts on the stairs (off the ground 0.08 s or
           more before) whose centre lies within 8 u of a riser line
  late/sw  stair swings (pin or target on the stairs) whose target height
           moved more than 4 u at phase 0.3 or later / stair swings
  bothOff  seconds with neither foot in ground contact
  float    p90 of the centre of mass's height over the fixture's ground
           while it descends (> 10 u/s down) over the stairs
  flat     the same off the stairs (any vertical speed)
  ovr%     RAGDOLL_FOOTFALL_LIFT lines with overrun=1; trail = with trail=1
  hold%    swing trace frames at phase > 1 (the 0.2 s airborne hold)
  reach    RAGDOLL_LANDING_REACH lines
  stair    RAGDOLL_FOOTFALL_STAIR swing lines (down = step-down ones),
           lock = lock_reject lines, probe = their probe_miss sum
  drop     RAGDOLL_STAIR_DROP lines (on and off)
Rates pool the runs; the other columns are the runs' median.
"""
import gzip
import json
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

FALL_MODES = {'falling', 'fallen', 'gettingup', 'getting_up', 'blendback', 'blend_back'}
ANCHORS = ('hit', 'push', 'off', 'jump', 'down', 'stand', 'stop', 'climb', 'crawl')
WINDOW = 4.0
# Read over the whole case, not WINDOW from the anchor.
WHOLE = ('stairs up and back', 'STAIRS-01')
STAIRS = ('stairs up and back', 'STAIRS-01', 'STAIRJUMP-01', 'stairs drop off')
# The fixtures' sides, widened by about the capsule's radius: STAIRS-01 climbs
# with the centre of mass at y 734-750, along the strafe staircase's side.
SIDE = 24.0


def strafe_ground(x, y):
    """The original staircase off the corridor: treads 32 deep rising 16 along
    +X from x = 875 to the top at 1099-1131 (z 128), down again to 1355;
    y 747-1003."""
    if not (747.0 - SIDE <= y <= 1003.0 + SIDE) or not (875.0 <= x < 1355.0):
        return 0.0
    n = int((x - 875.0) // 32.0)
    return 16.0 * (n + 1 if n <= 7 else 15 - n)


def tall_ground(x, y):
    """STAIRJUMP-01 (tools/make-ragdoll-test-features.py): a top at 192 over
    x -3200..-2900, then 7 treads 40 deep, 24 down each along -X; y
    -2000..-1600."""
    if not (-2000.0 - SIDE <= y <= -1600.0 + SIDE) or not (-3480.0 <= x < -2900.0):
        return 0.0
    if x >= -3200.0:
        return 192.0
    return 168.0 - 24.0 * int((-3200.0 - x) // 40.0)


# Per fixture: its ground, its riser lines (x) and the stairs' extent (the
# treads between the first and last riser, y across).
FIXTURES = {
    'strafe': (strafe_ground, [875.0 + 32.0 * k for k in range(16)], (875.0, 1355.0), (747.0, 1003.0)),
    'tall': (tall_ground, [-3200.0 - 40.0 * k for k in range(8)], (-3480.0, -3200.0), (-2000.0, -1600.0)),
}


def fixture(test):
    return FIXTURES['tall' if 'STAIRJUMP-01' in test else 'strafe']


def on_stairs(fx, x, y, margin=8.0):
    _, _, (x0, x1), (y0, y1) = fx
    return y0 - SIDE <= y <= y1 + SIDE and x0 - margin <= x <= x1 + margin


def p90(values):
    if not values:
        return None
    values = sorted(values)
    return values[int(0.9 * (len(values) - 1))]
DROP = 'DROP-01'
GETUP = 'ACRO-01'
DROP_WINDOW = 7.0
GRAVITY = 627.2


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


def roll_results(run):
    """The player's RAGDOLL_ROLL end results per case (the last TEST_START
    before each line names the case)."""
    by = defaultdict(list)
    for log in sorted(Path(run).glob('shard-*/test_play01_walking/openmw.log')):
        case = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'TEST_START\t\d+\t(.*)', line)
            if m:
                case = m.group(1).strip()
            elif case and 'RAGDOLL_ROLL end' in line and re.search(r'actor="?player"?\s', line):
                r = re.search(r'result=(\w+)', line)
                t = re.search(r' t=([-\d.]+)', line)
                if r:
                    vh = re.search(r' vh=([-\d.]+)', line)
                    by[case].append(r.group(1) + (':%.2f' % float(t.group(1)) if t else '')
                                    + (':%.0f' % float(vh.group(1)) if vh else ''))
    return by


def getup_speeds(run):
    """The player's RAGDOLL_GETUP_START clip speeds per case."""
    by = defaultdict(list)
    for log in sorted(Path(run).glob('shard-*/test_play01_walking/openmw.log')):
        case = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'TEST_START\t\d+\t(.*)', line)
            if m:
                case = m.group(1).strip()
            elif case and 'RAGDOLL_GETUP_START' in line and re.search(r'actor="?player"?\s', line):
                v = re.search(r'speed=([-\d.]+)', line)
                if v:
                    by[case].append(float(v.group(1)))
    return by


def angle(a, b):
    la, lb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    if la < 1e-6 or lb < 1e-6:
        return None
    c = sum(x * y for x, y in zip(a, b)) / (la * lb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def tuck_metrics(after, td):
    """ACRO-01: the largest knee and hip fold and the lowest head in the
    1.5 s from the touchdown (body centres)."""
    knee = hip = 0.0
    headz = None
    for t, s in after:
        if td is None or t < td or t > td + 1.5:
            continue
        pelvis, chest, head = pos(s, 'pelvis'), pos(s, 'spine2'), pos(s, 'head')
        if head is not None:
            headz = head[2] if headz is None else min(headz, head[2])
        for side in ('left', 'right'):
            thigh, calf, foot = pos(s, side + '_thigh'), pos(s, side + '_calf'), pos(s, side + '_foot')
            if thigh and calf and foot:
                a = angle(sub(calf, thigh), sub(foot, calf))
                knee = max(knee, a or 0.0)
            if pelvis and chest and thigh:
                a = angle(sub(chest, pelvis), sub(thigh, pelvis))
                if a is not None:
                    hip = max(hip, 180.0 - a)
    return {'knee': knee, 'hip': hip, 'headz': headz}


def getup_metrics(after):
    """From the impact: Fallen, the get-up's start, Standing after it, and
    falls after the first get-up started."""
    t0 = after[0][0]
    fallen = getup = stand = None
    refall, previous = 0, None
    for t, s in after:
        mode = str(s['controller'].get('mode', '')).lower().replace('_', '')
        if mode == 'fallen' and fallen is None:
            fallen = t
        if mode == 'gettingup' and getup is None:
            getup = t
        if getup is not None and stand is None and mode == 'standing':
            stand = t
        if getup is not None and mode == 'falling' and previous != 'falling':
            refall += 1
        previous = mode

    def rel(x, y):
        return None if x is None or y is None else x - y
    return {'g_fallen': rel(fallen, t0), 'g_lie': rel(getup, fallen), 'g_rise': rel(stand, getup),
            'g_total': rel(stand, t0), 'g_refall': refall}


def drop_metrics(after):
    """The drop roll's columns over (t, state) frames from the anchor."""
    td, below = None, False
    for t, s in after:
        v = s['controller'].get('centerOfMassVelocity')
        if not isinstance(v, list) or len(v) < 3:
            continue
        if v[2] < -300.0:
            below = True
        elif below and v[2] > -100.0:
            td = t
            break
    ups = [s['controller'].get('torsoUpDot') for _, s in after]
    ups = [u for u in ups if isinstance(u, (int, float))]
    inv = bool(ups) and min(ups) < -0.3
    rolled, down_at = False, None
    for t, s in after:
        f = s['controller'].get('torsoFacingUpDot')
        if not isinstance(f, (int, float)):
            continue
        if f <= -0.6:
            down_at = t if down_at is None else down_at
        elif f >= 0.6 and down_at is not None:
            if t - down_at <= 1.2:
                rolled = True
                break
            down_at = None
    headv, prev = 0.0, None
    for t, s in after:
        h = pos(s, 'head')
        if h is None:
            continue
        if prev is not None and t > prev[0] and h[2] < 20.0:
            headv = max(headv, math.dist(h[:3], prev[1][:3]) / (t - prev[0]))
        prev = (t, h)
    pel = [(t, p[2]) for t, s in after for p in [pos(s, 'pelvis')] if p is not None]
    vz = [(pel[i][0], (pel[i][1] - pel[i - 1][1]) / (pel[i][0] - pel[i - 1][0]))
          for i in range(1, len(pel)) if pel[i][0] > pel[i - 1][0]]
    jolt = 0.0
    for i in range(len(vz) - 3):
        dt = vz[i + 3][0] - vz[i][0]
        if dt > 0:
            jolt = max(jolt, (vz[i + 3][1] - vz[i][1]) / dt / GRAVITY)
    travel = None
    if td is not None:
        coms = [s.get('centerOfMass') for t, s in after if t >= td]
        coms = [c for c in coms if isinstance(c, list) and len(c) >= 2]
        if coms:
            travel = math.hypot(coms[-1][0] - coms[0][0], coms[-1][1] - coms[0][1])
    return dict({'td': None if td is None else td - after[0][0], 'inv': inv, 'rolled': rolled, 'headv': headv,
                 'jolt': jolt, 'travel': travel}, **tuck_metrics(after, td))


def summarise(run, prefixes):
    tests = defaultdict(lambda: {'frames': [], 'impact': None})
    for r in records(run):
        test = r.get('test') or ''
        if prefixes and not any(p in test for p in prefixes):
            continue
        if r.get('kind') == 'impact' and tests[test]['impact'] is None:
            tests[test]['impact'] = r.get('simulation_time')
        if r.get('kind') == 'frame' and isinstance(r.get('state'), dict) and isinstance(
                r['state'].get('controller'), dict):
            tests[test]['frames'].append((r.get('simulation_time', 0.0), r.get('segment') or '', r['state']))
    out = {}
    for test, data in tests.items():
        frames = data['frames']
        if not frames:
            continue
        whole = any(p in test for p in WHOLE)
        t0 = frames[0][0] if whole else data['impact']
        if t0 is None:
            for t, seg, _ in frames:
                if seg in ANCHORS:
                    t0 = t
                    break
        if t0 is None:
            continue
        end = frames[-1][0] if whole else t0 + (DROP_WINDOW if DROP in test else WINDOW)
        after = [(t, s) for t, seg, s in frames if t0 <= t <= end]
        if len(after) < 3:
            continue
        fell = any(str(s['controller'].get('mode', '')).lower() in FALL_MODES for _, s in after)
        tilt = 0.0
        steady, calm, prev_t = None, 0.0, t0
        slide = 0.0
        prev_com, prev_tt = None, None
        foot_hist = {'left_foot': [], 'right_foot': []}
        for t, s in after:
            c = s['controller']
            up = c.get('torsoUpDot')
            if isinstance(up, (int, float)):
                tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, up)))))
            v, target = c.get('centerOfMassVelocity'), c.get('targetHorizontalVelocity')
            ok = (isinstance(v, list) and isinstance(target, list) and isinstance(up, (int, float))
                  and math.hypot(v[0] - target[0], v[1] - target[1]) < 30.0 and up > math.cos(math.radians(12)))
            calm = calm + (t - prev_t) if ok else 0.0
            if steady is None and calm >= 0.3:
                steady = t - t0 - 0.3
            sup = s.get('support') or {}
            down = bool(sup.get('leftFoot')) and bool(sup.get('rightFoot'))
            com = s.get('centerOfMass')
            if isinstance(com, list) and prev_com is not None and isinstance(target, list):
                dt = t - prev_tt
                dx = com[0] - prev_com[0] - target[0] * dt
                dy = com[1] - prev_com[1] - target[1] * dt
                if down and isinstance(up, (int, float)) and up > math.cos(math.radians(20)):
                    slide += math.hypot(dx, dy)
            prev_com, prev_tt = com if isinstance(com, list) else prev_com, t
            prev_t = t
        # Steps: a foot leaves the ground and lands again more than 8 u from
        # where it left. Skid: a foot's travel while it stays in contact.
        steps, skid = 0, 0.0
        for foot, key in (('left_foot', 'leftFoot'), ('right_foot', 'rightFoot')):
            left_at, previous, was = None, None, None
            for t, s in after:
                p = pos(s, foot)
                contact = bool((s.get('support') or {}).get(key))
                if p is None:
                    continue
                if was is not None:
                    if was and contact and previous is not None:
                        skid += math.hypot(p[0] - previous[0], p[1] - previous[1])
                    if was and not contact:
                        left_at = previous
                    if not was and contact and left_at is not None:
                        if math.hypot(p[0] - left_at[0], p[1] - left_at[1]) > 8.0:
                            steps += 1
                        left_at = None
                was, previous = contact, p
        z0 = (after[0][1].get('centerOfMass') or [0, 0, 0])[2]
        z1 = (frames[-1][2].get('centerOfMass') or [0, 0, 0])[2]
        out[test] = {'fell': fell, 'steps': steps, 'skid': skid, 'slide': slide,
                     'steady': steady, 'tilt': tilt, 'over': z1 < z0 - 40.0, 'rise': z1 - z0}
        if DROP in test:
            out[test].update(drop_metrics(after))
        if GETUP in test or 'knockdown' in test:
            out[test].update(getup_metrics([(t, s) for t, seg, s in frames if t >= t0]))
    if any(GETUP in test or 'knockdown' in test for test in out):
        speeds = getup_speeds(run)
        for test in out:
            found = [v for name, v in speeds.items() if name == test or test.endswith(name) or name.endswith(test)]
            out[test]['g_speed'] = found[0] if found else []
    if any(DROP in test for test in out):
        rolls = roll_results(run)
        for test in out:
            if DROP in test:
                found = [v for name, v in rolls.items() if name == test or test.endswith(name) or name.endswith(test)]
                out[test]['roll'] = ','.join(found[0]) if found and found[0] else '-'
    return out


def stair_frames(run):
    """Per stairs case: the falls, first contacts, both-feet-off time and the
    centre of mass's float over the fixture's ground, over the whole case."""
    tests = defaultdict(list)
    for r in records(run):
        test = r.get('test') or ''
        if not any(p in test for p in STAIRS):
            continue
        if r.get('kind') == 'frame' and isinstance(r.get('state'), dict) and isinstance(
                r['state'].get('controller'), dict):
            tests[test].append((r.get('simulation_time', 0.0), r['state']))
    out = {}
    for test, frames in tests.items():
        if len(frames) < 3:
            continue
        frames.sort(key=lambda f: f[0])
        fx = fixture(test)
        ground, risers, (x0, x1), _ = fx
        fell = any(str(s['controller'].get('mode', '')).lower() in FALL_MODES for _, s in frames)
        contacts, edges = 0, 0
        for foot, key in (('left_foot', 'leftFoot'), ('right_foot', 'rightFoot')):
            was, off_since = None, None
            for t, s in frames:
                p = pos(s, foot)
                if p is None:
                    continue
                contact = bool((s.get('support') or {}).get(key))
                if not contact and (was or was is None):
                    off_since = t
                if contact and was is False and off_since is not None and t - off_since >= 0.08 \
                        and on_stairs(fx, p[0], p[1]):
                    contacts += 1
                    if min(abs(p[0] - x) for x in risers) < 8.0:
                        edges += 1
                was = contact
        both_off, floats, flats = 0.0, [], []
        for (ta, sa), (tb, sb) in zip(frames, frames[1:]):
            sup = sb.get('support') or {}
            if not sup.get('leftFoot') and not sup.get('rightFoot'):
                both_off += tb - ta
            com = sb.get('centerOfMass')
            v = sb['controller'].get('centerOfMassVelocity')
            if not isinstance(com, list) or len(com) < 3:
                continue
            height = com[2] - ground(com[0], com[1])
            if on_stairs(fx, com[0], com[1], 0.0):
                if isinstance(v, list) and len(v) >= 3 and v[2] < -10.0:
                    floats.append(height)
            elif not (x0 - 8.0 <= com[0] <= x1 + 8.0 and ground(com[0], com[1]) > 0.0):
                flats.append(height)
        out[test] = {'fell': fell, 'contacts': contacts, 'edges': edges, 'bothOff': both_off,
                     'float': p90(floats), 'flat': p90(flats)}
    return out


def logs(run):
    paths = sorted(Path(run).glob('shard-*/*/openmw.log'))
    return paths or sorted(Path(run).glob('shard-*/*.log.gz'))


def stair_log(run):
    """Per stairs case, from the footfall trace between its TEST_START and the
    next: swings, late retargets, airborne holds, lifts, landing reaches and
    the stair lines."""
    out = {}
    for path in logs(run):
        opener = gzip.open if path.suffix == '.gz' else open
        data, fx, feet = None, None, None
        with opener(path, 'rt', encoding='utf-8', errors='replace') as stream:
            for line in stream:
                if '\tTEST_START\t' in line:
                    name = line.rstrip('\r\n').split('\t')[-1]
                    data = None
                    if any(p in name for p in STAIRS):
                        data = out[name] = defaultdict(int)
                        fx = fixture(name)
                        feet = [{'swing': False}, {'swing': False}]
                    continue
                if data is None:
                    continue
                i = line.find('RAGDOLL_')
                if i < 0:
                    continue
                body = line[i:].rstrip('\r\n')
                tag = body.split(' ', 1)[0]
                if tag not in ('RAGDOLL_FOOTFALL', 'RAGDOLL_FOOTFALL_LIFT', 'RAGDOLL_LANDING_REACH',
                               'RAGDOLL_FOOTFALL_STAIR', 'RAGDOLL_STAIR_DROP'):
                    continue
                kv = dict(tok.split('=', 1) for tok in body.split()[1:] if '=' in tok)
                if 'player' not in kv.get('actor', 'player'):
                    continue
                if tag == 'RAGDOLL_FOOTFALL':
                    try:
                        side = int(kv['side'])
                        phase = float(kv['phase'])
                        pin = [float(v) for v in kv['pin'].split(',')]
                        target = [float(v) for v in kv['target'].split(',')]
                    except (KeyError, ValueError, IndexError):
                        continue
                    foot = feet[side]
                    if kv.get('swing') == '1':
                        data['swingFrames'] += 1
                        if phase > 1.0:
                            data['holdFrames'] += 1
                        if not foot['swing']:
                            feet[side] = foot = {'swing': True, 'z': target[2], 'late': False,
                                                 'stair': on_stairs(fx, pin[0], pin[1]) or on_stairs(
                                                     fx, target[0], target[1])}
                        else:
                            if abs(target[2] - foot['z']) > 4.0 and phase >= 0.3:
                                foot['late'] = True
                            foot['z'] = target[2]
                            foot['stair'] = foot['stair'] or on_stairs(fx, target[0], target[1])
                    elif foot['swing']:
                        if foot['stair']:
                            data['swings'] += 1
                            data['late'] += foot['late']
                        feet[side] = {'swing': False}
                elif tag == 'RAGDOLL_FOOTFALL_LIFT':
                    data['lifts'] += 1
                    data['overrun'] += kv.get('overrun') == '1'
                    data['trail'] += kv.get('trail') == '1'
                elif tag == 'RAGDOLL_LANDING_REACH':
                    data['reach'] += 1
                elif tag == 'RAGDOLL_FOOTFALL_STAIR':
                    if 'lock_reject' in body:
                        data['lock'] += 1
                    else:
                        data['stair'] += 1
                        data['down'] += kv.get('down') == '1'
                        try:
                            data['probe'] += int(kv.get('probe_miss', '0'))
                        except ValueError:
                            pass
                else:
                    data['drop'] += 1
    return out


def stairs_table(runs_dirs):
    frames = [stair_frames(r) for r in runs_dirs]
    traces = [stair_log(r) for r in runs_dirs]
    names = sorted({n for r in frames for n in r} | {n for r in traces for n in r})
    if not names:
        return {}
    print()
    print('%-34s %5s %6s %7s %7s %5s %5s %5s %5s %5s %5s %5s %4s %4s %5s %4s' % (
        'stairs case (whole)', 'falls', 'edge%', 'late/sw', 'bothOff', 'float', 'flat', 'ovr%', 'trail', 'hold%',
        'reach', 'stair', 'down', 'lock', 'probe', 'drop'))
    table = {}
    for name in names:
        rows = [r[name] for r in frames if name in r]
        logs_ = [r[name] for r in traces if name in r]
        med = lambda xs: statistics.median(xs) if xs else None
        fmt = lambda v, f: (f % v) if v is not None else '-'
        pct = lambda a, b: (100.0 * a / b) if b else None
        falls = sum(1 for x in rows if x['fell'])
        edge = pct(sum(x['edges'] for x in rows), sum(x['contacts'] for x in rows))
        late = med([x['late'] for x in logs_])
        swings = med([x['swings'] for x in logs_])
        both = med([x['bothOff'] for x in rows])
        flo = med([x['float'] for x in rows if x['float'] is not None])
        flat = med([x['flat'] for x in rows if x['flat'] is not None])
        ovr = pct(sum(x['overrun'] for x in logs_), sum(x['lifts'] for x in logs_))
        hold = pct(sum(x['holdFrames'] for x in logs_), sum(x['swingFrames'] for x in logs_))
        count = lambda k: med([x[k] for x in logs_])
        print('%-34s %2d/%-2d %6s %7s %7s %5s %5s %5s %5s %5s %5s %5s %4s %4s %5s %4s' % (
            name.replace('PLAY-01 unarmed ', '')[:34], falls, len(rows), fmt(edge, '%.0f'),
            ('%.0f/%.0f' % (late, swings)) if swings is not None else '-', fmt(both, '%.2f'), fmt(flo, '%.0f'),
            fmt(flat, '%.0f'), fmt(ovr, '%.0f'), fmt(count('trail'), '%.0f'), fmt(hold, '%.1f'),
            fmt(count('reach'), '%.0f'), fmt(count('stair'), '%.0f'), fmt(count('down'), '%.0f'),
            fmt(count('lock'), '%.0f'), fmt(count('probe'), '%.0f'), fmt(count('drop'), '%.0f')))
        table[name] = {'frames': rows, 'trace': [dict(x) for x in logs_]}
    return table


def main():
    args = sys.argv[1:]
    json_out = None
    if '--json' in args:
        i = args.index('--json')
        json_out = args[i + 1]
        del args[i:i + 2]
    prefixes = ('STAGGER-01', 'LEDGE-01', 'STAIRJUMP-01', 'SLOPE-01', 'TEETER-01', 'WALL-01', 'MANTLE-01', 'CRAWL-01',
                'stairs up and back', 'STAIRS-01', DROP, GETUP, 'knockdown')
    runs = [summarise(r, prefixes) for r in args]
    names = sorted({n for r in runs for n in r})
    print('%-46s %5s %6s %7s %7s %7s %6s %5s %5s' % ('case', 'falls', 'steps', 'skid', 'slide', 'steady', 'tilt', 'over',
                                                    'rise'))
    table = {}
    for name in names:
        rows = [r[name] for r in runs if name in r]
        falls = sum(1 for x in rows if x['fell'])
        med = lambda k: statistics.median([x[k] for x in rows])
        steadies = [x['steady'] for x in rows if x['steady'] is not None]
        steady = '%.2f' % statistics.median(steadies) if steadies and len(steadies) * 2 > len(rows) else '-'
        print('%-46s %2d/%-2d %6.1f %7.0f %7.0f %7s %6.0f %5d %5.0f' % (
            name.replace('PLAY-01 unarmed ', '')[:46], falls, len(rows), med('steps'), med('skid'), med('slide'),
            steady, med('tilt'), sum(1 for x in rows if x.get('over')), med('rise')))
        table[name] = {'runs': rows}
    drops = [n for n in names if DROP in n]
    if drops:
        print()
        print('%-46s %5s %3s %6s %6s %5s %6s %5s %5s %5s  %s' % ('drop case', 'td', 'inv', 'rolled', 'headv', 'jolt',
                                                                 'travel', 'knee', 'hip', 'headz', 'roll:t:vh (per run)'))
        for name in drops:
            rows = [r[name] for r in runs if name in r]
            tds = [x['td'] for x in rows if x.get('td') is not None]
            travels = [x['travel'] for x in rows if x.get('travel') is not None]
            heads = [x['headz'] for x in rows if x.get('headz') is not None]
            print('%-46s %5s %3d %6d %6.0f %5.1f %6s %5.0f %5.0f %5s  %s' % (
                name.replace('PLAY-01 unarmed ', '')[:46],
                '%.2f' % statistics.median(tds) if tds else '-',
                sum(1 for x in rows if x.get('inv')), sum(1 for x in rows if x.get('rolled')),
                statistics.median([x.get('headv', 0.0) for x in rows]),
                statistics.median([x.get('jolt', 0.0) for x in rows]),
                '%.0f' % statistics.median(travels) if travels else '-',
                statistics.median([x.get('knee', 0.0) for x in rows]),
                statistics.median([x.get('hip', 0.0) for x in rows]),
                '%.0f' % statistics.median(heads) if heads else '-',
                ' '.join(x.get('roll', '-') for x in rows)))
    getups = [n for n in names if GETUP in n or 'knockdown' in n]
    if getups:
        print()
        print('%-46s %6s %5s %5s %6s %6s  %s' % ('get-up case', 'fallen', 'lie', 'rise', 'total', 'refall',
                                               'total@speed (per run)'))

        def fmt(v):
            return '-' if v is None else '%.2f' % v
        for name in getups:
            rows = [r[name] for r in runs if name in r]

            def m(k):
                vs = [x[k] for x in rows if x.get(k) is not None]
                return statistics.median(vs) if vs else None
            print('%-46s %6s %5s %5s %6s %6d  %s' % (
                name.replace('PLAY-01 unarmed ', '')[:46], fmt(m('g_fallen')), fmt(m('g_lie')), fmt(m('g_rise')),
                fmt(m('g_total')), sum(x.get('g_refall', 0) for x in rows),
                ' '.join('%s@%s' % (fmt(x.get('g_total')), ','.join('%.2f' % v for v in x.get('g_speed', [])) or '-')
                         for x in rows)))
    stairs = stairs_table(args)
    if json_out:
        Path(json_out).write_text(json.dumps(dict(table, _stairs=stairs), indent=1))


if __name__ == '__main__':
    main()
