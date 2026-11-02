"""NPCHIT-01 report (10-06): per opponent, the blows on the standing player in
30 s (RAGDOLL_DAMAGE victim="player"), how many bit the hit roll, and the damage.

usage: python tools/body/analysis/npchit01.py RUNTIME_DIR ...
"""
import glob, re, sys
for root in sys.argv[1:]:
    print(root)
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'NPCHIT01_BEGIN opponent=(.+?) weapon', line)
            if m: current = [m.group(1), 0, 0, 0.0, 0, 0]; continue
            if 'NPCHIT01_END' in line and current:
                print('  %-20s blows %d  bit %d/%d rolls  damage %.1f  contacts %d' % (current[0], current[1], current[4], current[5], current[3], current[2]))
                current = None; continue
            if not current: continue
            if re.search(r'RAGDOLL_DAMAGE attacker=\S+ victim="player"', line):
                current[1] += 1; current[3] += float(re.search(r'final=([\d.]+)', line).group(1))
            if 'RAGDOLL_DEFENSE' in line and 'ai=1' in line: current[2] += 1
            h = re.search(r'RAGDOLL_HIT_ROLL attacker=\S+ victim="player" chance=\S+ bites=(\d)', line)
            if h: current[5] += 1; current[4] += int(h.group(1))
