"""DMG-04 damage report: every landed physical blow against Morrowind's band.

usage: damage_report.py RUNDIR [RUNDIR...]
Reads RAGDOLL_DAMAGE lines (OPENMW_RAGDOLL_DAMAGE_TRACE=1) from each case's
openmw.log and prints, per case and attacker/weapon: blows, approach speed
(median, range), swing quality, authored damage (band x quality x bonus x
region, before strength/armor) as a share of the band maximum, final damage.
A full swing should sit near 1.0 of the maximum (up to 1.15 with the bonus).
"""
import glob
import re
import statistics
import sys
from collections import defaultdict


def cases(run):
    for log in sorted(glob.glob(run + '/shard-*/*/openmw.log') + glob.glob(run + '/*/openmw.log')):
        case = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'TEST_START\t\d+\t(.*)', line)
            if m:
                case = m.group(1).strip()
            elif 'RAGDOLL_DAMAGE ' in line:
                yield case or '(manual)', dict(re.findall(r'(\w+)=(\S+)', line))


def main():
    table = defaultdict(list)
    for run in sys.argv[1:]:
        for case, row in cases(run):
            key = (case, row.get('attacker', '?').strip('"')[:20], row.get('weapon', '?').strip('"'))
            table[key].append(row)
    for (case, attacker, weapon), rows in table.items():
        speed = [float(r['speed']) for r in rows]
        quality = [float(r['quality']) for r in rows]
        final = [float(r['final']) for r in rows]
        band = rows[0].get('band', '')
        top = max((int(b) for r in rows for b in r.get('band', '0-0').split('-')), default=0)
        authored = [float(r['authored']) / top for r in rows] if top else []
        print(f'{case[:48]:48s} {attacker:20s} {weapon:16s} n={len(rows):3d} '
              f'speed={statistics.median(speed):6.0f} [{min(speed):4.0f}-{max(speed):5.0f}] '
              f'q={statistics.median(quality):.2f} band={band:6s} '
              f'authored/max={statistics.mean(authored) if authored else 0:.2f} '
              f'final={statistics.mean(final):6.2f} total={sum(final):7.1f}')


if __name__ == '__main__':
    main()
