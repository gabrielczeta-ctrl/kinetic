"""Blows per test in duel runs: count, median approach speed and final damage
per weapon, from RAGDOLL_DAMAGE lines between a test's TEST_START and its end.

usage: python tools/body/analysis/duel_blows.py 'TEST PREFIX' RUNTIME_DIR ...
"""
import glob, re, statistics, sys
from collections import defaultdict
prefix = sys.argv[1]
for root in sys.argv[2:]:
    per = defaultdict(lambda: ([], []))
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        inside = False
        for line in open(log, encoding='utf-8', errors='replace'):
            if 'TEST_START' in line:
                inside = prefix in line
            elif inside and 'RAGDOLL_DAMAGE' in line:
                w = re.search(r'weapon=("[^"]*"|\S+)', line).group(1)
                sp = float(re.search(r' speed=(\S+)', line).group(1))
                fi = float(re.search(r' final=(\S+)', line).group(1))
                per[w][0].append(sp); per[w][1].append(fi)
    print(root, ' | '.join('%s n=%d speed=%.0f final=%.1f' % (w, len(v[0]), statistics.median(v[0]), statistics.median(v[1]))
                           for w, v in sorted(per.items())))
