"""Hand tremor and idle-step check per case and segment (TORCH-01, IDLE-01).

hand jitter: RMS of the left/right hand's acceleration relative to the chest
(second difference of its chest-relative position, u/s^2) - a tremor shows as
a high value with the hand going nowhere; also the share of frames where that
relative velocity flips direction.
idle steps: RAGDOLL_FOOTFALL_SETTLE / HELPER_SWING / catch rows in the case's
log per segment, when the run had OPENMW_RAGDOLL_FOOTFALL_TRACE=1.

usage: hand_jitter.py RUN_DIR [RUN_DIR ...]
"""
import gzip
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def body_position(bodies, name):
    b = bodies.get(name)
    if not b:
        return None
    p = b.get('position') or b.get('centerOfMass')
    return p if isinstance(p, list) and len(p) == 3 else None


def main(runs):
    for run in runs:
        print(run)
        for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
            frames = defaultdict(list)
            with gzip.open(path, 'rt', encoding='utf-8') as stream:
                for line in stream:
                    r = json.loads(line)
                    if r.get('kind') != 'frame':
                        continue
                    bodies = (r.get('state') or {}).get('bodies') or {}
                    chest = body_position(bodies, 'spine2') or body_position(bodies, 'pelvis')
                    t = r.get('simulation_time')
                    if chest is None or t is None:
                        continue
                    row = {'t': t, 'seg': r.get('segment'), 'test': r.get('test')}
                    for side in ('left_hand', 'right_hand'):
                        hand = body_position(bodies, side)
                        if hand is not None:
                            row[side] = sub(hand, chest)
                    frames[r.get('test')].append(row)
            for test, rows in frames.items():
                stats = defaultdict(lambda: defaultdict(list))
                for a, b, c in zip(rows, rows[1:], rows[2:]):
                    dt1, dt2 = b['t'] - a['t'], c['t'] - b['t']
                    if not (0.002 < dt1 < 0.1 and 0.002 < dt2 < 0.1) or b['seg'] != c['seg']:
                        continue
                    for side in ('left_hand', 'right_hand'):
                        if side not in a or side not in b or side not in c:
                            continue
                        v1 = [x / dt1 for x in sub(b[side], a[side])]
                        v2 = [x / dt2 for x in sub(c[side], b[side])]
                        acc = norm(sub(v2, v1)) / (0.5 * (dt1 + dt2))
                        s = stats[b['seg']][side]
                        s.append((acc, sum(x * y for x, y in zip(v1, v2)) < 0 and norm(v2) > 5, norm(v2)))
                for seg, sides in stats.items():
                    parts = []
                    for side, values in sorted(sides.items()):
                        rms = math.sqrt(sum(v[0] ** 2 for v in values) / len(values))
                        flips = sum(v[1] for v in values) / len(values)
                        speed = sum(v[2] for v in values) / len(values)
                        parts.append('%s acc_rms=%7.0f flips=%4.0f%% speed=%5.1f' % (side[0].upper(), rms, 100 * flips, speed))
                    print('  %-42s %-8s %s' % (test, seg, '  '.join(parts)))
        # Step rows from the runner's logs.
        counts = defaultdict(lambda: defaultdict(int))
        for log in sorted(Path(run).glob('shard-*/*.log.gz')):
            test = None
            for line in gzip.open(log, 'rt', errors='replace'):
                m = re.search(r'TEST_START	\d+	(.+)', line)
                if m:
                    test = m.group(1).strip()
                for tag in ('RAGDOLL_FOOTFALL_SETTLE', 'RAGDOLL_FOOTFALL_HELPER_SWING', 'RAGDOLL_PGC_CATCH',
                            'RAGDOLL_FOOTFALL_LIFT'):
                    if tag in line:
                        reason = re.search(r'reason=(\w+)', line)
                        counts[test or str(log)][tag[8:] + (':' + reason.group(1) if reason else '')] += 1
        for test, tags in counts.items():
            print('  steps %-40s %s' % (test, ' '.join('%s=%d' % kv for kv in sorted(tags.items()))))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
