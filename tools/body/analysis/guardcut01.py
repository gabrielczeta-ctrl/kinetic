"""GUARDCUT-01 report (build 63 play, 2026-10-07): per weapon and line, what
each cut at a raised guard did - damage, the RAGDOLL_DEFENSE outcomes of the
player's strikes (hit / block / parry, with the cover distance when
OPENMW_RAGDOLL_GUARD_COVER is on) and the blade clashes (RAGDOLL_PARRY).

usage: python tools/body/analysis/guardcut01.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

for root in sys.argv[1:]:
    rows = collections.defaultdict(list)
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'GUARDCUT01_BEGIN weapon=(.+?) line=(\S+) rep=', line)
            if m:
                current = {'key': (m.group(1), m.group(2)), 'out': [], 'clash': 0}
                continue
            e = re.search(r'GUARDCUT01_END .* damage=([\d.\-]+)', line)
            if e and current:
                rows[current['key']].append((float(e.group(1)), current['out'], current['clash']))
                current = None
                continue
            if not current:
                continue
            d = re.search(r'RAGDOLL_DEFENSE outcome=(\w+) guard=(\d)(?: cover=(\S+))?.* ai=0', line)
            if d:
                cover = d.group(3)
                current['out'].append(d.group(1) + ('' if cover in (None, '-1') else '@%.0f' % float(cover)))
            if 'RAGDOLL_PARRY kind=blade attacker="player"' in line:
                current['clash'] += 1
    print(root)
    total = collections.Counter()
    for (weapon, line), v in sorted(rows.items()):
        for damage, outs, clash in v:
            total['cuts'] += 1
            total['damaging'] += damage > 0
            total['clash'] += clash > 0
            total['guard-blocked'] += any(o.startswith(('block', 'parry')) for o in outs) and damage <= 0
        print('  %-15s %-13s damage %s outcomes %s clashes %s' % (weapon, line,
            [round(d, 1) for d, _, _ in v], [o[:3] for _, o, _ in v], [c for _, _, c in v]))
    print('  total', dict(total))
