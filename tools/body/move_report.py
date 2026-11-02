"""Movement measures per test case (test review 10-30: the user's movement
notes as scripted numbers; play_metrics.py measures the same in play).

usage: python tools/body/move_report.py RUN_DIR [RUN_DIR ...]

Per case, from the player diagnostics and the log:
  air        share of running frames with no foot supported ("both feet
             suspended", run_flight.py)
  long/min   flights of 0.25 s or more per running minute
  step_cv    touchdown-to-touchdown time variation while running
  turn_s     slowest reversal: seconds until the centre of mass moves at half
             the new request's speed along it (turn_time.py)
  reaches    RAGDOLL_LANDING_REACH lines of the player (flat ground: 0)
  lift_*     why the player's feet lifted per speed band (lift_reasons.py, the
             same numbers play_metrics.py reads in play; needs
             OPENMW_RAGDOLL_FOOTFALL_TRACE=1, on with --play)
"""
import glob
import gzip
import json
import math
import statistics
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lift_reasons  # noqa: E402

RUN_SEGMENTS = ('c run', 'c run again', 'run')


def case_rows(out):
    by = defaultdict(list)
    for path in sorted(glob.glob(str(out) + '/shard-*/*.player-diagnostics.jsonl.gz')):
        for line in gzip.open(path, 'rt', errors='replace'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get('kind') == 'frame' and 'state' in r:
                by[r.get('test', '?')].append(r)
    return by


def case_log_lines(out):
    """The game log's lines per test (between its [ RUN ] markers)."""
    lines = defaultdict(list)
    for path in glob.glob(str(out) + '/shard-*/**/*.log', recursive=True):
        test = None
        for line in open(path, encoding='utf-8', errors='replace'):
            if '[ RUN      ] ' in line:
                test = line.split('[ RUN      ] ', 1)[1].strip()
            elif test and 'RAGDOLL_' in line:
                lines[test].append(line)
    return lines


def landing_reaches(lines):
    return {test: sum(1 for l in ls if 'RAGDOLL_LANDING_REACH actor="player"' in l) for test, ls in lines.items()}


def measure(rows):
    m = {}
    air = frames = longs = 0
    run_len, last, touch, i = 0, None, {0: [], 1: []}, 0
    for r in rows:
        seg = r.get('segment') or ''
        support = r['state'].get('support')
        if not support or not any(seg.endswith(s) for s in RUN_SEGMENTS):
            run_len, last = 0, None
            continue
        i += 1
        feet = (bool(support.get('leftFoot')), bool(support.get('rightFoot')))
        frames += 1
        if not any(feet):
            air += 1
            run_len += 1
        else:
            if run_len >= 15:
                longs += 1
            run_len = 0
        if last:
            for side in (0, 1):
                if feet[side] and not last[side]:
                    touch[side].append(i)
        last = feet
    if frames >= 60:
        m['air'] = air / frames
        m['long/min'] = longs / (frames / 3600.0)
        gaps = [(b - a) / 60.0 for side in (0, 1) for a, b in zip(touch[side], touch[side][1:]) if 9 < b - a < 90]
        if len(gaps) > 4:
            m['step_cv'] = statistics.pstdev(gaps) / statistics.mean(gaps)
    turns, last_raw, pending = [], None, None
    for i, r in enumerate(rows):
        c = r['state'].get('controller') or {}
        raw = c.get('rawTargetHorizontalVelocity') or [0, 0, 0]
        v = c.get('centerOfMassVelocity') or [0, 0, 0]
        speed = math.hypot(raw[0], raw[1])
        if speed > 40 and last_raw and math.hypot(last_raw[0], last_raw[1]) > 40 \
                and raw[0] * last_raw[0] + raw[1] * last_raw[1] < 0 and not pending:
            pending = (i, (raw[0] / speed, raw[1] / speed), speed)
        if pending:
            i0, d, s = pending
            if v[0] * d[0] + v[1] * d[1] >= 0.5 * s:
                turns.append((i - i0) / 60.0)
                pending = None
            elif i - i0 > 120:
                turns.append(2.0)
                pending = None
        if speed > 40:
            last_raw = raw
    if turns:
        m['turn_s'] = max(turns)
    return m


def per_case(out):
    lines = case_log_lines(out)
    reaches = landing_reaches(lines)
    result = {}
    for test, rows in case_rows(out).items():
        m = measure(rows)
        m['reaches'] = reaches.get(test, 0)
        lifts = lift_reasons.summary(lines.get(test, []))
        m.update({k: v for k, v in lifts.items() if v is not None and (not k.startswith('lift_n_') or v)})
        result[test] = m
    return result


if __name__ == '__main__':
    for run in sys.argv[1:]:
        for test, m in sorted(per_case(run).items()):
            print('%-50s %s' % (test[:50], '  '.join('%s=%.2f' % (k, v) for k, v in m.items())))
