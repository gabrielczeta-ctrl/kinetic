"""How fast the body follows a change of the movement request (user 10-30:
"less drifting ... change direction faster"): after every turn of the raw
movement request by more than 90 deg, the seconds until the centre of mass
moves at half the new request's speed along it, and the centre of mass's
sideways drift (units) meanwhile.

usage: python tools/body/turn_time.py RUN_DIR [RUN_DIR ...]
"""
import glob
import gzip
import json
import math
import statistics
import sys

for run in sys.argv[1:]:
    times, drifts, misses = [], [], 0
    for path in sorted(glob.glob(run + '/shard-*/*.player-diagnostics.jsonl.gz')):
        by = {}
        for line in gzip.open(path, 'rt', errors='replace'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get('kind') == 'frame' and 'controller' in r.get('state', {}):
                by.setdefault(r['test'], []).append(r)
        for rows in by.values():
            last, pending = None, None
            for i, r in enumerate(rows):
                c = r['state']['controller']
                raw = c.get('rawTargetHorizontalVelocity') or [0, 0, 0]
                v = c.get('centerOfMassVelocity') or [0, 0, 0]
                speed = math.hypot(raw[0], raw[1])
                if speed > 40 and last and math.hypot(last[0], last[1]) > 40 \
                        and (raw[0] * last[0] + raw[1] * last[1]) < 0 and not pending:
                    pending = (i, (raw[0] / speed, raw[1] / speed), speed, 0.0)
                if pending:
                    i0, d, s, side = pending
                    along = v[0] * d[0] + v[1] * d[1]
                    across = abs(-v[0] * d[1] + v[1] * d[0]) / 60.0
                    pending = (i0, d, s, side + across)
                    if along >= 0.5 * s:
                        times.append((i - i0) / 60.0)
                        drifts.append(pending[3])
                        pending = None
                    elif i - i0 > 120:
                        misses += 1
                        pending = None
                if speed > 40:
                    last = raw
    print('%s turns=%d median=%.2fs p90=%.2fs drift=%.1fu not_within_2s=%d' % (
        run, len(times), statistics.median(times) if times else float('nan'),
        sorted(times)[int(len(times) * 0.9)] if times else float('nan'),
        statistics.median(drifts) if drifts else float('nan'), misses))
