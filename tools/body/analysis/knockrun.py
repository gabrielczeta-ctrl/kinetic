"""KNOCKRUN-01: after a get-up, running. Per segment ('run', 'run back'):
share of frames in each controller mode, pelvis speed, and foot lifts (a
foot rising 3+ units above its lowest height in the segment counts a step
when it comes back down).
usage: python tools/body/analysis/knockrun.py RUNTIME_DIR ..."""
import math, sys
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentinel
for root in sys.argv[1:]:
    for test, rows in sorted(sentinel.frames(Path(root)).items()):
        if 'KNOCKRUN' not in test:
            continue
        for seg in ('recover', 'run', 'run back'):
            rs = [r for r in rows if r.get('segment') == seg and 'bodies' in (r.get('state') or {})]
            if not rs:
                continue
            modes = Counter(sentinel.mode(r) for r in rs)
            speed = sum(math.hypot(*r['state']['bodies']['pelvis']['linearVelocity'][:2]) for r in rs) / len(rs)
            steps = 0
            for foot in ('left_foot', 'right_foot'):
                zs = [r['state']['bodies'][foot]['position'][2] for r in rs]
                low, up = min(zs), False
                for z in zs:
                    if not up and z > low + 3:
                        up = True
                    elif up and z < low + 1.5:
                        up = False
                        steps += 1
            n = len(rs)
            print('%-38s %-8s %s speed=%.0f steps=%d' % (test.replace('PLAY-01 ', ''), seg,
                  ' '.join('%s=%.2f' % (k, v / n) for k, v in modes.most_common(4)), speed, steps))
