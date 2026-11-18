"""HUD-02 weight arrow: what the arrow under the reticle would show through
each case - the capture point off the feet (state.hud.weightRight/Forward)
as a share of the tipping edge (weightEdge), its way (0 = forward, 90 =
right) and the HUD-01 tip, per segment and at the peak.

usage: python tools/body/analysis/weight_cue.py RUNTIME_DIR [case substring]

The reticle shows the arrow from a share of 0.12 (ARROW_FROM) and warms it to
amber from 0.45; off the ground (edge 0) it hides.
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentinel  # noqa: E402

out = Path(sys.argv[1])
want = sys.argv[2] if len(sys.argv) > 2 else ''
for test, rows in sorted(sentinel.frames(out).items()):
    if want not in test:
        continue
    by = defaultdict(list)
    peak = None
    for r in rows:
        hud = (r.get('state') or {}).get('hud') or {}
        edge = hud.get('weightEdge') or 0
        if not edge:
            by[r.get('segment')].append(None)
            continue
        x, y = hud.get('weightRight') or 0, hud.get('weightForward') or 0
        share = math.hypot(x, y) / edge
        way = math.degrees(math.atan2(x, y))
        by[r.get('segment')].append((share, way, hud.get('balanceTip') or 0))
        if peak is None or share > peak[0]:
            peak = (share, way, r.get('elapsed', 0), r.get('segment'))
    print('==', test)
    for seg, vals in by.items():
        got = [v for v in vals if v]
        air = len(vals) - len(got)
        if not got:
            print('  %-14s air (hidden) %d frames' % (seg, air))
            continue
        shares = sorted(v[0] for v in got)
        shown = sum(1 for v in got if v[0] > 0.12) / len(vals)
        print('  %-14s share median %.2f p90 %.2f max %.2f  shown %3.0f%%  amber %3.0f%%  hidden(air) %d' % (
            seg, shares[len(shares) // 2], shares[int(len(shares) * 0.9)], shares[-1], 100 * shown,
            100 * sum(1 for v in got if v[0] > 0.45) / len(vals), air))
    if peak:
        print('  peak share %.2f way %4.0f deg at %.2f s (%s)' % peak)
