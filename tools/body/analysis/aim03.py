"""AIM-03 report (off-centre foe, 2026-10-06): hits per weapon, mode, spot and
angle from the AIM03 lines of one or more runs.

usage: python tools/body/analysis/aim03.py RUNTIME_DIR ...
"""
import glob, re, sys, collections
ROW = re.compile(r'AIM03 weapon=(.+?) mode=(\w+) spot=([\d.]+) angle=(-?\d+) hit=(\d)')
for root in sys.argv[1:]:
    t = collections.defaultdict(lambda: [0, 0])
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        for line in open(log, encoding='utf-8', errors='replace'):
            m = ROW.search(line)
            if m:
                w, mode, spot, angle, hit = m.groups()
                for key in ((w, mode, 'all'), (w, mode, 'spot ' + spot), (w, mode, 'angle ' + str(abs(int(angle))))):
                    t[key][0] += int(hit); t[key][1] += 1
    print(root)
    for key in sorted(t):
        print('  %-16s %-6s %-11s %d/%d' % (key[0], key[1], key[2], *t[key]))
