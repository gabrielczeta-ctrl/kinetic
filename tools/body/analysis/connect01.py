"""CONNECT-01 report (2026-10-06): per weapon and mode, the player's blows
(RAGDOLL_IMPACT) and their damage (RAGDOLL_DAMAGE final) between the
CONNECT01_BEGIN/END lines. "rest" should strike once, "chain" twice.

usage: python tools/body/analysis/connect01.py RUNTIME_DIR ...
"""
import glob, re, sys, collections
BEGIN = re.compile(r'CONNECT01_BEGIN weapon=(.+?) mode=(\w+) rep=(\d+)')
for root in sys.argv[1:]:
    rows = collections.defaultdict(list)
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = BEGIN.search(line)
            if m:
                current = [m.group(1), m.group(2), 0, 0.0]; continue
            if 'CONNECT01_END' in line and current:
                rows[(current[0], current[1])].append((current[2], current[3])); current = None; continue
            if current and 'RAGDOLL_IMPACT attacker="player"' in line:
                current[2] += 1
            d = re.search(r'RAGDOLL_DAMAGE attacker="player".* final=([\d.]+)', line)
            if current and d:
                current[3] += float(d.group(1))
    print(root)
    for key in sorted(rows):
        v = rows[key]
        print('  %-15s %-6s blows %s  damage %s' % (key[0], key[1], [b for b, _ in v], [round(x, 1) for _, x in v]))
