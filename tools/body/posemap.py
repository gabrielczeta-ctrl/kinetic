"""Build a weapon pose map from POSEMAP-01 capture lines (alpha 0.9).

usage: posemap.py RUN_DIR WEAPON OUT.cfg

Each captured cell (azimuth, elevation of the commanded tip, degrees) becomes
one line of the map, in the actor's frame (x right, y forward, z up; units
from the actor's position, which is its feet):
  cell AZ EL grip=x,y,z blade=x,y,z pole=x,y,z twist=DEG out=WRIST
grip: the weapon hand; blade: the blade direction; pole: which way the weapon
arm's elbow points (its offset from the shoulder-hand line); twist: the
controller's chest twist command (+ = turned right). The player faces +y and has yaw 0 in the test,
so world axes are the actor's axes.
"""
import math
import re
import sys
from pathlib import Path


def vec(text):
    return [float(v) for v in text.split(',')]


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(a):
    length = math.sqrt(dot(a, a))
    return [v / length for v in a] if length > 1e-6 else [0.0, 0.0, 0.0]


def parse(line):
    fields = dict(re.findall(r'(\w+)=(\S+)', line))
    bodies = {}
    for name, value in re.findall(r'(\w+_?\w*)=([-\d.,]+)', line):
        parts = value.split(',')
        if len(parts) == 9:
            v = [float(p) for p in parts]
            bodies[name] = {'x': v[0:3], 'y': v[3:6], 'p': v[6:9]}
    return fields, bodies


def yaw_of(axis):
    return math.degrees(math.atan2(axis[1], axis[0]))


def rot_z(v, deg):
    a = math.radians(deg)
    return [v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a), v[2]]


def rot_x(v, deg):
    a = math.radians(deg)
    return [v[0], v[1] * math.cos(a) - v[2] * math.sin(a), v[1] * math.sin(a) + v[2] * math.cos(a)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def add_windups(cells):
    """Wind-up cells past the captured grid (user 2026-10-02: "dragging up
    should really make that sword come back and wind up a hit ... the wrists
    and torso bend/rotate"). Side columns at +-100: the blade turned 60
    degrees back behind that shoulder, the hands drawn toward it, the chest
    coiled 35 degrees away. Top rows at 105 and 140: the blade turned 45
    degrees more over the head each row, the hands raised and drawn back, the
    elbows up."""
    # The old solve cannot steer cleanly near the top (POSEMAP-01: commands
    # at 70 degrees flipped between +50 and -50 azimuth), so captured rows
    # from 70 up are dropped and built from the 40-degree row instead.
    for key in [k for k in cells if k[1] >= 70]:
        del cells[key]
    azs = sorted({az for az, _ in cells})
    top = max(el for _, el in cells)
    for az in azs:
        hand, blade, pole, twist, out = cells[(az, top)]
        cells[(az, 70)] = (add(hand, [0.0, -4.0, 10.0]), norm(rot_x(blade, 30.0)), pole, twist, -1.0)
    els = sorted({el for _, el in cells})
    for el in els:
        for src, dst, turn, shift, coil in ((azs[-1], 100, -60.0, [6.0, -16.0, 4.0], 35.0),
                                            (azs[0], -100, 60.0, [-6.0, -16.0, 4.0], -35.0)):
            hand, blade, pole, twist, out = cells[(src, el)]
            pole = norm(add(rot_z(pole, turn), [0.0, 0.0, 0.5]))
            cells[(dst, el)] = (add(hand, shift), norm(rot_z(blade, turn)), pole, twist + coil, -1.0)
    azs = sorted({az for az, _ in cells})
    for src, dst, lift in ((els[-1], 105, [0.0, -10.0, 12.0]), (105, 140, [0.0, -6.0, 4.0])):
        for az in azs:
            hand, blade, pole, twist, out = cells[(az, src)]
            pole = norm(add(rot_x(pole, 30.0), [0.0, 0.0, 0.4]))
            cells[(az, dst)] = (add(hand, lift), norm(rot_x(blade, 45.0)), pole, twist, -1.0)


def main():
    run, weapon, out = sys.argv[1], sys.argv[2], sys.argv[3]
    cells = {}
    for path in Path(run).rglob('*.log'):
        for line in open(path, encoding='utf-8', errors='replace'):
            if 'POSEMAP-01 weapon=' not in line or ('weapon=' + weapon) not in line.replace('weapon=' + weapon, 'weapon=' + weapon):
                continue
            if weapon not in line:
                continue
            fields, bodies = parse(line[line.index('POSEMAP-01'):])
            if 'actor' not in fields or 'right_hand' not in bodies:
                continue
            actor = vec(fields['actor'])
            hand = sub(bodies['right_hand']['p'], actor)
            shoulder = sub(bodies['right_upper_arm']['p'], actor)
            fore = sub(bodies['right_forearm']['p'], actor)
            line_dir = norm(sub(hand, shoulder))
            off = sub(fore, shoulder)
            along = dot(off, line_dir)
            pole = norm([off[i] - line_dir[i] * along for i in range(3)])
            # The controller's own chest twist command (degrees, + = turned
            # right); the bodies' local axes are not aligned alike.
            twist = float(fields.get('twist', '0'))
            out_r = float(fields.get('out_r', '-1'))
            cells[(int(fields['az']), int(fields['el']))] = (hand, vec(fields['blade_dir']), pole, twist, out_r)
    if not cells:
        sys.exit('no POSEMAP-01 cells for ' + weapon)
    add_windups(cells)
    # One elbow rule instead of the captured poles (physical snapshots, noisy):
    # down and out in the guards, blending to up and out over the head.
    for (az, el), (hand, blade, pole, twist, out_r) in list(cells.items()):
        t = min(1.0, max(0.0, (el - 40.0) / 100.0))
        guard, wind = norm([0.45, 0.0, -0.9]), norm([0.8, -0.2, 0.55])
        cells[(az, el)] = (hand, blade, norm([guard[i] * (1.0 - t) + wind[i] * t for i in range(3)]), twist, out_r)
    with open(out, 'w', encoding='utf-8') as stream:
        stream.write('# Pose map for %s, from POSEMAP-01 (tools/body/posemap.py). Actor frame:\n' % weapon)
        stream.write('# x right, y forward, z up, units from the feet. cell AZ EL grip blade pole twist out\n')
        for (az, el), (hand, blade, pole, twist, out_r) in sorted(cells.items(), key=lambda c: (c[0][1], c[0][0])):
            stream.write('cell %d %d grip=%.2f,%.2f,%.2f blade=%.4f,%.4f,%.4f pole=%.4f,%.4f,%.4f twist=%.1f out=%.1f\n'
                         % (az, el, *hand, *blade, *pole, twist, out_r))
    print('%d cells -> %s' % (len(cells), out))


if __name__ == '__main__':
    main()
