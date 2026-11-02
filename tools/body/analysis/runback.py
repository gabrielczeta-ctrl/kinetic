"""RUNBACK-01 skate score: in the running segment (after its first second),
the mean speed of the slower foot (in a real run the standing foot is
near 0 while it stands; a skating body drags it along), the mean
pelvis speed, and the torso tilt p90 (the "torso flies around").

usage: python tools/body/analysis/runback.py RUNTIME_DIR ...
"""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentinel
for root in sys.argv[1:]:
    for test, rows in sorted(sentinel.frames(Path(root)).items()):
        if 'RUNBACK-01' not in test:
            continue
        seg = [r for r in rows if r.get('segment') in ('run back', 'run fwd') and 'bodies' in r.get('state', {})]
        if not seg:
            continue
        t0 = seg[0]['elapsed']
        seg = [r for r in seg if r['elapsed'] >= t0 + 1.0]
        both, speeds, tilts = 0, [], []
        for r in seg:
            b = r['state']['bodies']
            v = lambda n: math.hypot(*b[n]['linearVelocity'][:2])  # noqa: E731
            both += min(v('left_foot'), v('right_foot'))
            speeds.append(v('pelvis'))
            up = r['state']['controller'].get('torsoUpDot')
            if isinstance(up, (int, float)):
                tilts.append(math.degrees(math.acos(max(-1, min(1, up)))))
        tilts.sort()
        print('%-50s slower_foot_speed=%.0f pelvis_speed=%.0f torso_tilt_p90=%.0f' % (
            test.replace('PLAY-01 ', ''), both / len(seg), sum(speeds) / len(speeds), tilts[int(0.9 * (len(tilts) - 1))]))
