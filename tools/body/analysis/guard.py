"""GUARD-01 report: blows that got through to the player (RAGDOLL_DAMAGE with
victim=player between 'GUARD-01 start' and 'GUARD-01 end'), clashes with the
guard (RAGDOLL_PARRY_ROLES), and the summary line.

usage: python tools/body/analysis/guard.py RUNTIME_DIR ...
"""
import glob, re, sys
for root in sys.argv[1:]:
    blows, damage, clashes, summary = 0, 0.0, 0, ''
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        inside = False
        for line in open(log, encoding='utf-8', errors='replace'):
            if 'GUARD-01 start' in line: inside = True
            elif 'GUARD-01 end' in line: inside = False
            elif 'GUARD-01 summary' in line: summary = line.split('GUARD-01 summary')[1].strip()
            elif inside and 'RAGDOLL_DAMAGE' in line and 'victim="player"' in line:
                blows += 1
                damage += float(re.search(r' final=(\S+)', line).group(1))
            elif inside and 'RAGDOLL_PARRY_ROLES' in line:
                clashes += 1
    print('%s blows_through=%d damage=%.1f clashes=%d %s' % (root, blows, damage, clashes, summary))
