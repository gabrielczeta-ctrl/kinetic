"""Committed-swing A/B per test case (roadmap 2b).

usage: swingab.py RUNDIR [RUNDIR...]   (runs with OPENMW_RAGDOLL_SWING_TRACE=1
and OPENMW_RAGDOLL_DAMAGE_TRACE=1). Per case and run: strokes (committed
ones marked by modes bit 16), median command/physical peak speed of the
strokes, then blows: median approach speed at contact, quality, total damage.
"""
import glob
import re
import statistics
import sys
from collections import defaultdict


def rows(run):
    for log in sorted(glob.glob(run + '/shard-*/*/openmw.log')):
        case = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'TEST_START\t\d+\t(.*)', line)
            if m:
                case = m.group(1).strip()
            elif 'RAGDOLL_STROKE ' in line or 'RAGDOLL_DAMAGE ' in line:
                kind = 'stroke' if 'RAGDOLL_STROKE ' in line else 'blow'
                yield case, kind, dict(re.findall(r'(\w+)=(\S+)', line))


def med(values):
    return statistics.median(values) if values else float('nan')


def main():
    table = defaultdict(lambda: defaultdict(lambda: {'stroke': [], 'blow': []}))
    for run in sys.argv[1:]:
        for case, kind, row in rows(run):
            if case and ('drag' in case or 'guided' in case):
                table[case][run][kind].append(row)
    for case in sorted(table):
        print(case.replace('FIX-08 repeated native player hits with ', ''))
        for run, data in table[case].items():
            strokes = [s for s in data['stroke'] if float(s['command_peak']) > 300]
            committed = sum(int(s['modes']) & 16 != 0 for s in strokes)
            blows = data['blow']
            print(f'   {run[-14:]:14s} strokes={len(strokes):2d} committed={committed:2d} '
                  f'cmd={med([float(s["command_peak"]) for s in strokes]):6.0f} '
                  f'phys={med([float(s["physical_peak"]) for s in strokes]):6.0f} | '
                  f'blows={len(blows):2d} contact={med([float(b["speed"]) for b in blows]):5.0f} '
                  f'q={med([float(b["quality"]) for b in blows]):.2f} '
                  f'damage={sum(float(b["final"]) for b in blows):6.1f}')


if __name__ == '__main__':
    main()
