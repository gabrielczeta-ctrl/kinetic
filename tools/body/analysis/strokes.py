"""Player weapon strokes in manual recordings (roadmap 2b, committed swings).

usage: strokes.py RECORDING_DIR [...]   (runtime-GAME-manual-*/manual-recording)
A stroke is a run of ticks in one tip mode other than guard (1 drag, 2 wind-up,
3 strike) with the same weapon. Per stroke: mouse speed (counts/s, peak over
3 ticks), commanded tip speed and physical tip speed (units/s, peak over 3
ticks), and the physical/command ratio. Summary per weapon and mode: how fast
the user moved the mouse and how much of it reached the blade.
"""
import glob
import json
import math
import statistics
import sys
from collections import defaultdict

MODES = {1: 'drag', 2: 'windup', 3: 'strike'}


def ticks(directory):
    for path in sorted(glob.glob(directory + '/ticks-*.jsonl'), key=lambda p: int(p.rsplit('-', 1)[1].split('.')[0])):
        for line in open(path, encoding='utf-8'):
            row = json.loads(line)
            if row.get('kind') == 'tick' and row.get('tip_mode', -1) >= 0:
                yield row


def peak3(values):
    if len(values) < 3:
        return max(values, default=0.0)
    return max(sum(values[i:i + 3]) / 3 for i in range(len(values) - 2))


def strokes(directory):
    # Only the player runs the tip controller; its actor id differs per session.
    current, last = None, None
    for row in ticks(directory):
        mode = row.get('tip_mode', -1)
        key = (mode, row.get('weapon', ''))
        if current and key != current['key']:
            yield current
            current = None
        if mode in MODES and row.get('tip_valid', [0, 0])[0]:
            if not current:
                current = {'key': key, 'mouse': [], 'cmd': [], 'phys': [], 'ticks': 0}
            dt = row.get('dt') or 1 / 60
            if last and last.get('tip_valid', [0, 0])[0]:
                current['phys'].append(math.dist(row['tip'], last['tip']) / dt)
                current['cmd'].append(math.dist(row['tip_cmd'], last['tip_cmd']) / dt)
                current['mouse'].append(math.hypot(row['mouse'][0], row['mouse'][1]) / dt)
            current['ticks'] += 1
        last = row
    if current:
        yield current


def main():
    table = defaultdict(list)
    for directory in sys.argv[1:]:
        for s in strokes(directory):
            if s['ticks'] < 4:
                continue
            table[s['key']].append((peak3(s['mouse']), peak3(s['cmd']), peak3(s['phys']), s['ticks'] / 60))
    for (mode, weapon), rows in sorted(table.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        mouse = [r[0] for r in rows]
        cmd = [r[1] for r in rows]
        phys = [r[2] for r in rows]
        ratio = [p / c for _, c, p, _ in rows if c > 50]
        fast = sorted(rows, key=lambda r: -r[0])[: max(1, len(rows) // 4)]
        print(f'{weapon or "fists":18s} {MODES[mode]:7s} n={len(rows):4d} '
              f'mouse p50={statistics.median(mouse):6.0f} p90={sorted(mouse)[int(0.9 * (len(mouse) - 1))]:6.0f} | '
              f'cmd p50={statistics.median(cmd):5.0f} p90={sorted(cmd)[int(0.9 * (len(cmd) - 1))]:5.0f} | '
              f'phys p50={statistics.median(phys):5.0f} p90={sorted(phys)[int(0.9 * (len(phys) - 1))]:5.0f} | '
              f'phys/cmd {statistics.median(ratio) if ratio else 0:.2f} | '
              f'fastest quarter mouse {statistics.median(r[0] for r in fast):6.0f} cmd {statistics.median(r[1] for r in fast):5.0f} '
              f'phys {statistics.median(r[2] for r in fast):5.0f}')


if __name__ == '__main__':
    main()
