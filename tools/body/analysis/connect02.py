"""CONNECT-02 report (2026-10-07): point-blank cut series with the
reactions on. Per weapon and distance (`near`, blade lengths past the
arm's reach) the blows (RAGDOLL_IMPACT by the player) of each of the six
cuts and the foe's chest gap at the cut's start. Every cut should strike.

usage: python tools/body/analysis/connect02.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

BEGIN = re.compile(r'CONNECT02_BEGIN weapon=(.+?) near=([\d.]+) cut=(\d+) gap=([-\d.]+)')
for root in sys.argv[1:]:
    rows = collections.defaultdict(list)
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = BEGIN.search(line)
            if m:
                current = [m.group(1), m.group(2), float(m.group(4)), 0]
                continue
            if 'CONNECT02_END' in line and current:
                rows[(current[0], current[1])].append((current[3], current[2]))
                current = None
                continue
            if current and 'RAGDOLL_IMPACT attacker="player"' in line:
                current[3] += 1
    print(root)
    total = struck = 0
    for key in sorted(rows):
        v = rows[key]
        total += len(v)
        struck += sum(1 for b, _ in v if b > 0)
        print('  %-15s near %s  struck %d/%d  blows %s  gap %s' % (key[0], key[1], sum(1 for b, _ in v if b > 0),
              len(v), [b for b, _ in v], [round(g) for _, g in v]))
    print('  total struck %d/%d' % (struck, total))
