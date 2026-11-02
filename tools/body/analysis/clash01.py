"""CLASH-01 report (C1, 2026-10-06): per weapon, the damage each chop dealt
into a staff block, and its clashes and push-throughs.

usage: python tools/body/analysis/clash01.py RUNTIME_DIR ...
"""
import glob, re, sys, collections
for root in sys.argv[1:]:
    rows = collections.defaultdict(list)
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'CLASH01_BEGIN weapon=(.+?) rep=', line)
            if m:
                current = [m.group(1), 0, 0]; continue
            e = re.search(r'CLASH01_END weapon=(.+?) rep=\d+ damage=([\d.\-]+)', line)
            if e and current:
                rows[current[0]].append((float(e.group(2)), current[1], current[2])); current = None; continue
            if current and 'RAGDOLL_PARRY kind=blade attacker="player"' in line: current[1] += 1
            if current and 'RAGDOLL_CLASH_WEIGHT attacker="player"' in line: current[2] += 1
    print(root)
    for weapon, v in sorted(rows.items()):
        print('  %-15s damage %s clashes %s push %s' % (weapon, [round(d, 1) for d, _, _ in v], [c for _, c, _ in v], [p for _, _, p in v]))
