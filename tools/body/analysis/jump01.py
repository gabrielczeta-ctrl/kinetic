"""JUMP-01 timeline: per ledge-jump case, the controller mode, torso tilt,
pelvis height and vertical speed around the flight and landing.

usage: python tools/body/analysis/jump01.py RUNTIME_DIR [case substring]
"""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentinel
out = Path(sys.argv[1]); want = sys.argv[2] if len(sys.argv) > 2 else 'JUMP-01'
for test, rows in sorted(sentinel.frames(out).items()):
    if want not in test:
        continue
    print('==', test)
    last = None
    for r in rows:
        seg = r.get('segment')
        if seg not in ('jump off', 'flight', 'stop', 'run off'):
            continue
        c = (r.get('state') or {}).get('controller') or {}
        b = ((r.get('state') or {}).get('bodies') or {}).get('pelvis') or {}
        up = c.get('torsoUpDot')
        tilt = math.degrees(math.acos(max(-1, min(1, up)))) if isinstance(up, (int, float)) else -1
        z = (b.get('position') or [0, 0, 0])[2]; vz = (b.get('linearVelocity') or [0, 0, 0])[2]
        key = (c.get('mode'), int(tilt // 10))
        if key != last:
            print('  %5.2f %-9s %-10s tilt=%4.0f z=%6.1f vz=%7.1f' % (r.get('elapsed', 0), seg, c.get('mode'), tilt, z, vz))
            last = key
        if r.get('elapsed', 0) > 0 and c.get('mode') == 'fallen':
            break
