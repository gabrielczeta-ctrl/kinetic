"""Hands' forward offset from the chest (units, + in front) over a walking
case, every half second: the claymore walk carry left both hands behind the
back (user bookmark 6).
usage: python tools/body/analysis/hands.py RUNTIME_DIR [case substring]"""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentinel
want = sys.argv[2] if len(sys.argv) > 2 else ''
for test, rows in sorted(sentinel.frames(Path(sys.argv[1])).items()):
    if want not in test:
        continue
    print('==', test)
    last = None
    for r in rows:
        b = (r.get('state') or {}).get('bodies') or {}
        if 'right_hand' not in b or 'spine2' not in b:
            continue
        yaw = r.get('gameplay_yaw', 0.0)
        fw = (math.sin(yaw), math.cos(yaw))
        def f(n):
            p, t = b[n]['position'], b['spine2']['position']
            return (p[0] - t[0]) * fw[0] + (p[1] - t[1]) * fw[1]
        key = (r.get('segment'), int(r['elapsed'] * 2))
        if key != last:
            last = key
            print('  %5.1f %-10s right=%6.1f left=%6.1f' % (r['elapsed'], r.get('segment'), f('right_hand'), f('left_hand')))
