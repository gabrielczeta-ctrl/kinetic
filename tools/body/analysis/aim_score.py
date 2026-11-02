"""Aim accuracy in manual recordings (aiming review 2026-09-30).

usage: aim_score.py RECORDING_DIR [...]   (runtime-GAME-manual-*/manual-recording)

Drag starts: over the first 0.2 s of each drag (tip mode 1 from a press), the
cosine between the physical tip's travel on the screen plane and the mouse's
(mouse y is down). 1 = the blade went where the mouse went.

Clicks: each wind-up (tip mode 2) and its strike (3). The blade is the segment
from the right hand to the tip, taken from 0.4 of the way out; its sweet spot
is 3/4 of the way. Per click:
  ray   closest the sweet spot came to the eye ray (the crosshair line), units
  point closest the blade came to the aim point (the crosshair at its target
        depth, as the wind-up began), units
Summaries per weapon: medians and 90th percentiles.
"""
import glob
import json
import math
import statistics
import sys
from collections import defaultdict


def ticks(directory):
    for path in sorted(glob.glob(directory + '/ticks-*.jsonl'), key=lambda p: int(p.rsplit('-', 1)[1].split('.')[0])):
        for line in open(path, encoding='utf-8'):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:  # a recording still being written
                continue
            if row.get('kind') == 'tick' and row.get('tip_mode', -1) >= 0:
                yield row


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a):
    length = math.sqrt(dot(a, a))
    return [c / length for c in a] if length > 1e-6 else [0.0, 0.0, 0.0]


def hand(row):
    for body in row.get('bodies', []):
        if body.get('name') == 'right_hand' and body.get('valid', [0, 0, 0])[2]:
            return body['physical']
    return None


def blade_miss(h, tip, point, start=0.4):
    axis = sub(tip, h)
    length2 = dot(axis, axis)
    if length2 < 1:
        return None
    along = min(1.0, max(start, dot(sub(point, h), axis) / length2))
    return math.dist([h[i] + axis[i] * along for i in range(3)], point)


def ray_distance(point, origin, direction):
    d = norm(direction)
    offset = sub(point, origin)
    along = max(0.0, dot(offset, d))
    return math.dist(point, [origin[i] + d[i] * along for i in range(3)])


def score(directory, drags, clicks):
    rows = list(ticks(directory))
    i = 0
    while i < len(rows):
        row = rows[i]
        previous = rows[i - 1] if i else None
        mode = row['tip_mode']
        started = previous is None or previous['tip_mode'] != mode
        weapon = row.get('weapon') or 'fists'
        if mode == 1 and started and row.get('tip_valid', [0])[0]:
            # A drag: the first 0.2 s (or until it ends).
            start = row['tip']
            mouse = [0.0, 0.0]
            elapsed, j = 0.0, i
            while j < len(rows) and rows[j]['tip_mode'] == 1 and elapsed < 0.2:
                mouse[0] += rows[j]['mouse'][0]
                mouse[1] += rows[j]['mouse'][1]
                elapsed += rows[j].get('dt') or 1 / 60
                j += 1
            end = rows[j - 1]
            if math.hypot(*mouse) > 20 and end.get('tip_valid', [0])[0]:
                forward = norm(end['eye_dir'])
                right = norm(cross(forward, [0, 0, 1]))
                up = cross(right, forward)
                moved = sub(end['tip'], start)
                screen = (dot(moved, right), dot(moved, up))
                want = norm([mouse[0], -mouse[1], 0])
                length = math.hypot(*screen)
                cosine = (screen[0] * want[0] + screen[1] * want[1]) / length if length > 0.5 else 0.0
                drags[weapon].append(cosine)
        if mode == 2 and started:
            aim = row['aim'] if row.get('aim_valid') else None
            best_ray, best_point = math.inf, math.inf
            j = i
            after = 0.0
            while j < len(rows) and (rows[j]['tip_mode'] in (2, 3) or after < 0.2):
                r = rows[j]
                if r['tip_mode'] not in (2, 3):
                    after += r.get('dt') or 1 / 60
                h = hand(r)
                if r['tip_mode'] != 2 and h and r.get('tip_valid', [0])[0]:
                    tip = r['tip']
                    sweet = [h[k] + (tip[k] - h[k]) * 0.75 for k in range(3)]
                    best_ray = min(best_ray, ray_distance(sweet, r['eye'], r['eye_dir']))
                    if aim:
                        miss = blade_miss(h, tip, aim)
                        if miss is not None:
                            best_point = min(best_point, miss)
                j += 1
            if best_ray < math.inf:
                clicks[weapon].append((best_ray, best_point))
            i = max(i + 1, j - 1)
            continue
        i += 1


def p(values, q):
    values = sorted(v for v in values if v < math.inf)
    return values[int(q * (len(values) - 1))] if values else float('nan')


def main():
    drags, clicks = defaultdict(list), defaultdict(list)
    for directory in sys.argv[1:]:
        score(directory, drags, clicks)
    print('drag starts (cosine of blade travel to mouse, first 0.2 s)')
    for weapon, values in sorted(drags.items()):
        print(f'  {weapon:18s} n={len(values):4d} mean={statistics.mean(values):5.2f} '
              f'with the mouse (>0.5) {sum(v > 0.5 for v in values) / len(values):4.0%}')
    print('clicks (units: sweet spot to crosshair line, blade to aim point)')
    for weapon, values in sorted(clicks.items()):
        ray = [v[0] for v in values]
        point = [v[1] for v in values]
        print(f'  {weapon:18s} n={len(values):4d} ray p50={p(ray, 0.5):5.1f} p90={p(ray, 0.9):5.1f} | '
              f'point p50={p(point, 0.5):5.1f} p90={p(point, 0.9):5.1f}')


if __name__ == '__main__':
    main()
