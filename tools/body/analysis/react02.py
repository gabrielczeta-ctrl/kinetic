"""REACT-02 report (RXN-02, 2026-10-07): the region x strength matrix of
blows. One row per cell (mean over the runs given): head, chest and struck
part turn (deg), chest tilt peak, pelvis travel (u), steps, falls, settle
time (s), head peak spin (rad/s), first motion of the struck part, first
foot-off and landing (s), direction error (deg, standing).

Human bands (from human reaction measurements): head
spin light < 10, solid 10-20, heavy 20-30 rad/s; first motion < 0.017 s;
foot-off 0.15-0.32 s, landing 0.30-0.52 s; steps light 0, solid 0-1, heavy
1-2; settle < 1.5 s after solid; direction error < 30 deg.

usage: python tools/body/analysis/react02.py RUNTIME_DIR ... [--csv OUT]
"""
import collections
import glob
import re
import sys

PARTS = ['none', 'head', 'spine2', 'right_clavicle', 'right_upper_arm', 'left_thigh', 'left_calf']
STRENGTHS = ['none', 'light', 'solid', 'heavy']
COLUMNS = [('head_turn', 'head', '%6.1f'), ('chest_turn', 'chest', '%6.1f'), ('part_turn', 'part', '%6.1f'),
           ('tilt_max', 'tilt', '%5.1f'), ('travel', 'trav', '%5.1f'), ('steps', 'steps', '%5.1f'),
           ('fell', 'falls', '%5d'), ('settle', 'settle', '%6.2f'), ('head_spin', 'spin', '%5.1f'),
           ('part_t', 'move_t', '%6.3f'), ('foot_off', 'off_t', '%5.2f'), ('foot_land', 'land_t', '%6.2f'),
           ('dir_err', 'dir', '%4.0f'), ('arm_turn', 'arms', '%5.1f'), ('drop', 'drop', '%5.1f')]


def number(value):
    if value in ('true', 'false'):
        return 1.0 if value == 'true' else 0.0
    try:
        return float(value)
    except ValueError:
        return None


def main(args):
    csv = None
    if '--csv' in args:
        csv = args[args.index('--csv') + 1]
        args = [a for a in args if a not in ('--csv', csv)]
    cells = collections.defaultdict(list)
    for root in args:
        for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
            for line in open(log, encoding='utf-8', errors='replace'):
                if 'REACT02 cell=' not in line:
                    continue
                fields = dict(re.findall(r'(\w+)=(\S+)', line.split('REACT02 ', 1)[1]))
                side = 'fight' if fields.get('fight') == 'true' else 'guard' if fields.get('guard') == 'true' else (
                    'side' if fields['side'] == 'true' else 'front')
                key = (fields['part'], fields['strength'], fields['moving'] == 'true', side)
                cells[key].append({k: number(v) for k, v in fields.items()})
    if not cells:
        print('no REACT02 rows')
        return
    header = '%-16s %-6s %-5s %-5s %2s ' % ('part', 'str', 'state', 'side', 'n') + ' '.join(
        '%6s' % c[1] for c in COLUMNS)
    print(header)
    out = []
    for moving in (False, True):
        for side in ('front', 'side', 'guard', 'fight'):
            for part in PARTS:
                for strength in STRENGTHS:
                    rows = cells.get((part, strength, moving, side))
                    if not rows:
                        continue
                    values = []
                    for name, _, _ in COLUMNS:
                        got = [r.get(name) for r in rows if r.get(name) is not None]
                        if name == 'fell':
                            values.append(int(sum(got)))
                        else:
                            values.append(sum(got) / len(got) if got else None)
                    cells_text = []
                    for (name, _, fmt), value in zip(COLUMNS, values):
                        cells_text.append(('%6s' % '-') if value is None else (fmt % value).rjust(6))
                    print('%-16s %-6s %-5s %-5s %2d ' % (part, strength, 'walk' if moving else 'stand', side, len(rows))
                          + ' '.join(cells_text))
                    out.append([part, strength, 'walk' if moving else 'stand', side, len(rows)] + values)
    if csv:
        with open(csv, 'w') as f:
            f.write('part,strength,state,side,n,' + ','.join(c[0] for c in COLUMNS) + '\n')
            for row in out:
                f.write(','.join('' if v is None else str(v) for v in row) + '\n')


if __name__ == '__main__':
    main(sys.argv[1:])
