"""GUARD-02 report (2026-10-08): the right-click guard. Part 1: per zone the
guard state, the zone read and the hand against the chest (body frame:
right, forward, up); the tap. Part 2: per guard zone, the player's overhead
chops met by the blade (RAGDOLL_PARRY kind=blade) or landing on the body
(RAGDOLL_IMPACT by the player) between GUARD02_CHOP_BEGIN/END.

usage: python tools/body/analysis/guard02.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

for root in sys.argv[1:]:
    print(root)
    chops = collections.defaultdict(lambda: [0, 0, 0])
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            if 'GUARD02 zone=' in line or 'GUARD02 tap' in line or 'GUARD02 released' in line:
                print('  ' + line.split('GUARD02 ', 1)[1].strip())
                continue
            m = re.search(r'GUARD02_CHOP_BEGIN zone=(\w+)', line)
            if m:
                current = m.group(1)
                chops[current][0] += 1
                continue
            if 'GUARD02_CHOP_END' in line:
                current = None
                continue
            if current and 'RAGDOLL_PARRY kind=blade' in line:
                chops[current][1] += 1
            if current and 'RAGDOLL_IMPACT attacker="player"' in line:
                chops[current][2] += 1
    for zone, (n, blade, body) in chops.items():
        print('  chop vs %-5s guard: %d chops, blade contacts %d, body impacts %d' % (zone, n, blade, body))
