"""Play metrics: the user's movement and combat notes as numbers, from manual
play recordings (test review 10-30: the user's sessions are the oracle the
scripted tests miss).

usage: python tools/body/play_metrics.py RECORDING_DIR|LOG [...] [--bookmarks] [--no-history]

A RECORDING_DIR is a manual hub recording (runtime-GAME-manual-*); a LOG is
a game log without a recording (the playthrough package's config\openmw.log),
read for its RAGDOLL_PLAY_METRICS lines (the running numbers, once a
minute) and the log measures. Each result is appended to
play-metrics-history.jsonl in the dev root with the engine's sha256, and
printed beside the median of the previous builds' sessions.

Per recording (the player only; tick rows of manual-recording/ticks-*.jsonl
and logs/openmw-run.log):
  run_float      share of running steps with no foot pinned (pin=00)
  long_flights   flights of 0.25 s or more while running
  step_cv        coefficient of variation of the time between touchdowns
                 (walk / run; the definition of done wants under 5%)
  stride_cv      the same for the touchdown-to-touchdown distance
  foot_split     mean forward offset left minus right foot from the pelvis
                 while running (a foot that "stays forward" shows here), units
  arm_ratio      left / right hand swing (fore-aft range) while running
                 sheathed or with the guard down, 1 = even
  turn_s         median seconds until the pelvis moves at 80 u/s along a
                 reversed movement request (the body, not the capsule)
  jump_keep      share of horizontal speed kept 0.3 s after a running jump
  landing_reach  RAGDOLL_LANDING_REACH lines per minute
  death_slide    RAGDOLL_DEATH_SLIDE horizontal, median / max
  down_blocked   blows a body down or getting up would have landed
                 (RAGDOLL_DOWN_NO_STRIKE, one line per attacker and second)
  pay            median share of the band a landed player blow paid
  lift_*         why the player's feet lifted, by speed band (lift_reasons.py:
                 lift_clock_walk = share on the gait clock 15-140 u/s, ...;
                 land_miss_* = median landing miss, units)
--bookmarks also prints the same numbers in the 4 s before each bookmark.
"""
import glob
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lift_reasons  # noqa: E402

STEP = 1.0 / 60.0


def ticks(directory):
    chunks = glob.glob(os.path.join(directory, 'manual-recording', 'ticks-*.jsonl'))
    chunks.sort(key=lambda p: int(re.search(r'ticks-(\d+)', p).group(1)))
    for chunk in chunks:
        with open(chunk, encoding='utf-8', errors='replace') as f:
            for line in f:
                if '"kind":"tick"' not in line and '"bookmark' not in line:
                    continue
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def body(row, name):
    for b in row.get('bodies') or []:
        if b['name'] == name:
            return b['physical']
    return None


def cv(values):
    values = [v for v in values if v > 0]
    if len(values) < 4:
        return None
    mean = statistics.mean(values)
    return statistics.pstdev(values) / mean if mean > 0 else None


def metrics(rows, log_lines=None, minutes=None):
    out = {}
    running = flights = 0
    flight = 0
    long_flights = 0
    touch = {0: [], 1: []}  # (tick, pos, running)
    last_pin = None
    splits = []
    hands = {'l': [], 'r': []}
    reversals = []
    pending_reversal = None
    last_move = None
    jumps = []
    prev_pos = None
    prev_pelvis = None
    for r in rows:
        if r.get('kind') != 'tick':
            continue
        b = r.get('body') or ''
        pin = re.search(r'pin=(\d)(\d)', b)
        move = r.get('move') or [0, 0, 0]
        moving = abs(move[0]) + abs(move[1]) > 0
        is_run = moving and move[2] == 1
        yaw = r.get('actor_yaw') or 0.0
        fwd = (math.sin(yaw), math.cos(yaw))
        right = (math.cos(yaw), -math.sin(yaw))
        pelvis = body(r, 'pelvis')
        lf, rf = body(r, 'left_foot'), body(r, 'right_foot')
        if pin and r.get('drive') is not None:
            pins = (pin.group(1) == '1', pin.group(2) == '1')
            if is_run:
                running += 1
                if not any(pins):
                    flights += 1
                    flight += 1
                else:
                    if flight >= 15:
                        long_flights += 1
                    flight = 0
            if last_pin is not None:
                for side, foot in ((0, lf), (1, rf)):
                    if pins[side] and not last_pin[side] and foot:
                        touch[side].append((r['tick'], foot, is_run, moving))
            last_pin = pins
        if is_run and pelvis and lf and rf and r.get('draw') in (0, 1):
            def ahead(p):
                return (p[0] - pelvis[0]) * fwd[0] + (p[1] - pelvis[1]) * fwd[1]
            splits.append(ahead(lf) - ahead(rf))
            lh, rh = body(r, 'left_hand'), body(r, 'right_hand')
            if lh and rh and (r.get('hsc') or {}).get('stage', 0) == 0:
                hands['l'].append(ahead(lh))
                hands['r'].append(ahead(rh))
        # A reversal of the keys: the travel direction (from actor_pos) must
        # come round past 90 deg toward the new request.
        pos = r.get('actor_pos')
        if moving and last_move and (move[0] * last_move[0] + move[1] * last_move[1]) < 0:
            want = (move[0] * right[0] + move[1] * fwd[0], move[0] * right[1] + move[1] * fwd[1])
            n = math.hypot(*want) or 1.0
            pending_reversal = (r['tick'], (want[0] / n, want[1] / n))
        if pending_reversal and pelvis:
            t0, want = pending_reversal
            if r['tick'] - t0 > 120:
                reversals.append(2.0)
                pending_reversal = None
            elif prev_pelvis and r['tick'] - prev_pelvis[0] == 1:
                vx = (pelvis[0] - prev_pelvis[1][0]) / STEP
                vy = (pelvis[1] - prev_pelvis[1][1]) / STEP
                if vx * want[0] + vy * want[1] >= 80.0:
                    reversals.append((r['tick'] - t0) * STEP)
                    pending_reversal = None
        prev_pelvis = (r['tick'], pelvis) if pelvis else None
        if moving:
            last_move = move
        # A running jump: the actor rises 20+ units within 0.25 s.
        prev_pos = pos
        if pos:
            jumps.append((r['tick'], pos, is_run))
    out['run_float'] = flights / running if running else None
    out['long_flights'] = long_flights
    for kind, want_run in (('walk', False), ('run', True)):
        gaps, strides = [], []
        for side in (0, 1):
            seq = [t for t in touch[side] if t[2] == want_run and t[3]]
            for a, c in zip(seq, seq[1:]):
                dt = (c[0] - a[0]) * STEP
                if 0.15 < dt < 1.5:
                    gaps.append(dt)
                    stride = math.hypot(c[1][0] - a[1][0], c[1][1] - a[1][1])
                    if stride < 150:  # not across a teleport
                        strides.append(stride)
        out['step_cv_' + kind] = cv(gaps)
        out['stride_cv_' + kind] = cv(strides)
    out['foot_split'] = statistics.mean(splits) if len(splits) > 30 else None
    if len(hands['l']) > 60:
        rng = lambda v: sorted(v)[int(len(v) * 0.95)] - sorted(v)[int(len(v) * 0.05)]
        out['arm_ratio'] = rng(hands['l']) / max(rng(hands['r']), 1e-3)
    else:
        out['arm_ratio'] = None
    out['turn_s'] = statistics.median(reversals) if reversals else None
    out['turns'] = len(reversals)
    # Running jumps: the speed kept 0.3 s after the take-off.
    keeps = []
    for i in range(10, len(jumps) - 30):
        t, p, run = jumps[i]
        if not run:
            continue
        before = jumps[i - 10][1]
        if p[2] - before[2] > 20 and jumps[i - 1][1][2] - jumps[i - 10][1][2] <= 20:
            v0 = math.hypot(p[0] - before[0], p[1] - before[1]) / (10 * STEP)
            a, c = jumps[i + 8][1], jumps[i + 18][1]
            v1 = math.hypot(c[0] - a[0], c[1] - a[1]) / (10 * STEP)
            if v0 > 100:
                keeps.append(v1 / v0)
    out['jump_keep'] = statistics.median(keeps) if keeps else None
    if log_lines is not None:
        reaches = sum(1 for l in log_lines if 'RAGDOLL_LANDING_REACH actor="player"' in l)
        out['landing_reach_per_min'] = reaches / minutes if minutes else None
        slides = [float(m.group(1)) for l in log_lines
                  for m in [re.search(r'RAGDOLL_DEATH_SLIDE .*horizontal=([\d.]+)', l)] if m]
        out['death_slide'] = (statistics.median(slides), max(slides)) if slides else None
        pays = []
        for l in log_lines:
            m = re.search(r'RAGDOLL_DAMAGE attacker="player".* quality=([\d.]+).* scale=([\d.]+)', l)
            if m:
                pays.append(float(m.group(1)) * float(m.group(2)))
        out['pay'] = statistics.median(pays) if pays else None
        out['blows'] = len(pays)
        out['down_blocked'] = sum(1 for l in log_lines if 'RAGDOLL_DOWN_NO_STRIKE' in l)
        out.update(lift_reasons.summary(log_lines))
        runs = [dict(kv.split('=') for kv in l.split('RAGDOLL_PLAY_METRICS ', 1)[1].split())
                for l in log_lines if 'RAGDOLL_PLAY_METRICS ' in l]
        if runs and out.get('run_float') is None:
            total = sum(float(x['run_s']) for x in runs)
            out['run_float'] = sum(float(x['air']) * float(x['run_s']) for x in runs) / total if total else None
            out['long_flights'] = sum(int(x['long_flights']) for x in runs)
    return out


def show(label, m):
    def f(v):
        if v is None:
            return '-'
        if isinstance(v, tuple):
            return '/'.join('%.0f' % x for x in v)
        return '%.2f' % v if isinstance(v, float) else str(v)
    print(label)
    print('  ' + '  '.join('%s=%s' % (k, f(v)) for k, v in m.items()))


HISTORY = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))), 'play-metrics-history.jsonl')


def engine_of(directory):
    manifest = os.path.join(directory, 'manual-manifest.json')
    if os.path.exists(manifest):
        try:
            return json.load(open(manifest, encoding='utf-8-sig')).get('engine_sha256', '?')[:16]
        except ValueError:
            pass
    return '?'


def compare(name, engine, m):
    """Append to the history; print the previous two builds' medians beside."""
    old = []
    if os.path.exists(HISTORY):
        for line in open(HISTORY, encoding='utf-8'):
            try:
                old.append(json.loads(line))
            except ValueError:
                pass
    old = [o for o in old if o.get('name') != name]
    keys = [k for k, v in m.items() if isinstance(v, (int, float)) and not isinstance(v, bool)]
    builds = []
    for o in old:
        if o['engine'] not in builds:
            builds.append(o['engine'])
    for engine_before in [b for b in builds if b != engine][-2:]:
        rows = [o['m'] for o in old if o['engine'] == engine_before]
        med = {k: statistics.median([r[k] for r in rows if isinstance(r.get(k), (int, float))])
               for k in keys if any(isinstance(r.get(k), (int, float)) for r in rows)}
        print('  before (%s, %d sessions): %s' % (engine_before, len(rows),
              '  '.join('%s=%.2f' % (k, v) for k, v in med.items())))
    with open(HISTORY, 'a', encoding='utf-8') as f:
        f.write(json.dumps({'name': name, 'engine': engine, 'm': {k: m[k] for k in keys}}) + '\n')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        sys.exit(__doc__)
    for directory in args:
        if os.path.isfile(directory):
            lines = open(directory, encoding='utf-8', errors='replace').read().splitlines()
            name = os.path.abspath(directory)
            m = metrics([], lines, None)
            show(name, m)
            if '--no-history' not in sys.argv:
                compare(name, 'package', m)
            continue
        rows = list(ticks(directory))
        # The hub's recordings keep the log under logs\, the playthrough's
        # at the top of the recording.
        log = os.path.join(directory, 'logs', 'openmw-run.log')
        if not os.path.exists(log):
            log = os.path.join(directory, 'openmw.log')
        lines = open(log, encoding='utf-8', errors='replace').read().splitlines() if os.path.exists(log) else []
        tick_rows = [r for r in rows if r.get('kind') == 'tick']
        minutes = len(tick_rows) * STEP / 60.0
        name = os.path.basename(directory.rstrip('/' + chr(92)))
        m = metrics(rows, lines, minutes)
        show('%s (%.1f min, engine %s)' % (name, minutes, engine_of(directory)), m)
        if '--no-history' not in sys.argv and tick_rows:
            compare(name, engine_of(directory), m)
        if '--bookmarks' in sys.argv:
            notes = {r.get('id'): r.get('reason', '') for r in rows if r.get('kind') == 'bookmark_note'}
            for r in rows:
                if r.get('kind') != 'bookmark':
                    continue
                window = [x for x in tick_rows if r['tick'] - 240 <= x['tick'] <= r['tick']]
                show('  bookmark %s tick %s: %s' % (r.get('id'), r['tick'], notes.get(r.get('id'), '')[:70]),
                     metrics(window))


if __name__ == '__main__':
    main()
