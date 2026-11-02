"""Body drift per case and segment (user 10-30 notes 3/7: "less drifting",
"too floaty"): how far the physical centre of mass strays from the
character's position (the capsule the game moves), horizontally.

  lag     mean / p95 horizontal distance, centre of mass to capsule (units)
  over    on stop segments: the largest distance the centre of mass travelled
          past the capsule along the last travel direction (overshoot)
  settle  on stop segments: seconds until the distance stays under 4 units
  back    on stop segments: how far the pelvis moved back after its furthest
          point along the travel (the rebound, user 10-31 "slingshots back")
  fback   on stop segments: the largest backward move of a foot once the
          pelvis has passed its furthest point (a step back)
  cam     on stop segments: the largest change of the camera's speed along the
          travel from one frame to the next (u/s; the user's 10-31 stop
          "tweak" was a 9 unit forward pop in one frame, 370-660)

usage: drift_report.py RUN_DIR [RUN_DIR ...]
"""
import gzip
import json
import math
import sys
from collections import defaultdict
from pathlib import Path


def main(runs):
    for run in runs:
        print(run)
        rows = defaultdict(list)
        for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
            for line in gzip.open(path, 'rt', errors='replace'):
                r = json.loads(line)
                if r.get('kind') != 'frame':
                    continue
                com = (r.get('state') or {}).get('centerOfMass')
                pos = r.get('player_position')
                if not isinstance(com, list) or not isinstance(pos, list):
                    continue
                bodies = (r.get('state') or {}).get('bodies') or {}
                parts = [(bodies.get(k) or {}).get('position') for k in ('pelvis', 'left_foot', 'right_foot')]
                parts.append((r.get('input') or {}).get('camera_position'))
                rows[r.get('test')].append((r.get('simulation_time'), r.get('segment'), com, pos, parts))
        for test, frames in sorted(rows.items()):
            by = defaultdict(list)
            for f in frames:
                by[f[1]].append(f)
            parts = []
            for seg, fs in by.items():
                d = sorted(math.hypot(c[0] - p[0], c[1] - p[1]) for _, _, c, p, _ in fs)
                text = '%s lag=%.1f/%.1f' % (seg, sum(d) / len(d), d[int(0.95 * (len(d) - 1))])
                if 'stop' in (seg or ''):
                    # Travel direction: the capsule's motion over the frames before the stop.
                    first = fs[0]
                    before = [f for f in frames if f[0] is not None and first[0] is not None
                              and first[0] - 0.4 <= f[0] < first[0]]
                    if len(before) >= 2:
                        dx = before[-1][3][0] - before[0][3][0]
                        dy = before[-1][3][1] - before[0][3][1]
                        n = math.hypot(dx, dy)
                        if n > 1e-3:
                            over = max((c[0] - p[0]) * dx / n + (c[1] - p[1]) * dy / n for _, _, c, p, _ in fs)
                            settle = 0.0
                            for t, _, c, p, _ in fs:
                                if math.hypot(c[0] - p[0], c[1] - p[1]) >= 4.0:
                                    settle = t - first[0]
                            text += ' over=%.1f settle=%.2f' % (over, settle)
                            along = lambda v: v[0] * dx / n + v[1] * dy / n
                            if all(isinstance(v, list) for f in fs for v in f[4][:3]):
                                pelvis = [along(f[4][0]) for f in fs]
                                peak = pelvis.index(max(pelvis))
                                back = max(pelvis[peak] - min(pelvis[peak:]), 0.0)
                                fback = 0.0
                                for foot in (1, 2):
                                    xs = [along(f[4][foot]) for f in fs][peak:]
                                    fback = max([fback] + [max(xs[:i + 1]) - xs[i] for i in range(len(xs))])
                                text += ' back=%.1f fback=%.1f' % (back, fback)
                            if all(isinstance(f[4][3], list) for f in fs):
                                cam, prev, worst = [(f[0], along(f[4][3])) for f in fs], None, 0.0
                                for (t0, a0), (t1, a1) in zip(cam, cam[1:]):
                                    if t1 > t0:
                                        v = (a1 - a0) / (t1 - t0)
                                        if prev is not None:
                                            worst = max(worst, abs(v - prev))
                                        prev = v
                                text += ' cam=%.0f' % worst
                parts.append(text)
            print('  %-40s %s' % (test, ' | '.join(parts)))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
