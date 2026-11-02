"""WRIST-01 report: per weapon, the command-to-blade gap and wrist overshoot
by command azimuth (slow sweeps), and the fast strokes.

usage: python tools/body/analysis/wrist01.py RUNTIME_DIR [RUNTIME_DIR ...]
"""
import glob, re, sys
from collections import defaultdict

ROW = re.compile(r'WRIST-01S (.+?) seg=(\S+) az=(\S+) el=(\S+) twist=(\S+) gap=(\S+) out_r=(\S+) out_l=(\S+) blade_az=(\S+) blade_el=(\S+)')
data = defaultdict(list)
for root in sys.argv[1:]:
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        for line in open(log, encoding='utf-8', errors='replace'):
            m = ROW.search(line)
            if m:
                w, seg = m.group(1), m.group(2)
                v = [float(x) for x in m.groups()[2:]]
                data[w].append((seg, *v))
for w in sorted(data):
    rows = data[w]
    slow = [r for r in rows if r[0].split('-')[-1] in ('right', 'left', 'back')]
    fast = [r for r in rows if r[0].startswith('fast')]
    def stats(rs):
        if not rs:
            return 'n=0'
        gaps = sorted(r[4] for r in rs)
        outs = sorted(max(r[5], r[6]) for r in rs)
        return 'n=%d gap mean %.0f p90 %.0f | wrist p90 %.1f max %.1f over20 %.3f | twist max %.0f' % (
            len(rs), sum(gaps) / len(gaps), gaps[int(0.9 * (len(gaps) - 1))],
            outs[int(0.9 * (len(outs) - 1))], outs[-1], sum(o > 20 for o in outs) / len(outs),
            max(abs(r[3]) for r in rs))
    print('%s\n  slow: %s\n  fast: %s' % (w, stats(slow), stats(fast)))
    bins = defaultdict(list)
    for r in slow:
        bins[int((r[1] + 200) // 20) * 20 - 200].append(r[4])
    print('  gap by command az: ' + ' '.join('%d:%.0f' % (b, sum(g) / len(g)) for b, g in sorted(bins.items()) if len(g) >= 3))
