"""Third-person body shake as drawn (RAGDOLL_RENDER_JITTER rows, a run with
OPENMW_RAGDOLL_RENDER_JITTER=1): per rendered frame, the drawn pelvis
relative to the camera. A body that moves smoothly with the camera has a
relative path that is straight frame to frame; shake is the per-frame
deviation from the midpoint of its neighbours (units).

usage: render_jitter.py RUN_DIR|LOG [...]
"""
import gzip
import math
import re
import statistics
import sys
from pathlib import Path

ROW = re.compile(r'RAGDOLL_RENDER_JITTER dt=([\d.]+) alpha=([\d.]+) pelvis=([-\d.]+),([-\d.]+),([-\d.]+) '
                 r'camera=([-\d.]+),([-\d.]+),([-\d.]+) actor=([-\d.]+),([-\d.]+),([-\d.]+)')


NPC = re.compile(r'RAGDOLL_RENDER_JITTER_NPC dt=([\d.]+) pelvis=([-\d.]+),([-\d.]+),([-\d.]+)')


def lines(target):
    p = Path(target)
    files = sorted(p.glob('shard-*/*.log.gz')) if p.is_dir() else [p]
    for f in files:
        opener = gzip.open if str(f).endswith('.gz') else open
        with opener(f, 'rt', errors='replace') as stream:
            yield from stream


def main(targets):
    for target in targets:
        rows = []
        for line in lines(target):
            m = ROW.search(line)
            if m:
                v = [float(x) for x in m.groups()]
                rows.append((v[0], v[1], (v[2] - v[5], v[3] - v[6], v[4] - v[7]), (v[8], v[9], v[10])))
        dev, moving = [], []
        for a, b, c in zip(rows, rows[1:], rows[2:]):
            mid = [(x + y) / 2 for x, y in zip(a[2], c[2])]
            d = math.dist(b[2], mid)
            dev.append(d)
            if b[0] > 0 and math.dist(a[3], c[3]) / max(1e-6, a[0] + b[0]) > 30:
                moving.append(d)
        npc = []
        for line in lines(target):
            m = NPC.search(line)
            if m:
                npc.append([float(x) for x in m.groups()])
        if npc:
            # World smoothness: deviation from the neighbours' midpoint, and the
            # speed, frame to frame (a moving NPC is fine; a flicker is not).
            npcDev = [math.dist(b[1:], [(x + y) / 2 for x, y in zip(a[1:], c[1:])])
                      for a, b, c in zip(npc, npc[1:], npc[2:])]
            npcQ = lambda xs, f: sorted(xs)[int(f * (len(xs) - 1))]
            print('%s: npc frames=%d shake p50=%.2f p95=%.2f' % (target, len(npc), npcQ(npcDev, 0.5),
                                                                npcQ(npcDev, 0.95)))
        if not rows:
            print('%s: no RAGDOLL_RENDER_JITTER rows' % target)
            continue
        dts = sorted(r[0] for r in rows if r[0] > 0)
        fps = 1.0 / dts[len(dts) // 2] if dts else 0
        q = lambda xs, f: sorted(xs)[int(f * (len(xs) - 1))] if xs else float('nan')
        print('%s: frames=%d fps~%.0f shake all p50=%.2f p95=%.2f | moving n=%d p50=%.2f p95=%.2f' % (
            target, len(rows), fps, q(dev, 0.5), q(dev, 0.95), len(moving), q(moving, 0.5), q(moving, 0.95)))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
