#!/usr/bin/env python3
"""Behaviour fixtures on the ragdoll test floor (PGC-03, 2026-10-11).

Rewrites the second drawable of scripts/data/ragdoll_manual/meshes/
ragdoll_test_floor.osgt: the original staircase (its first 192 vertices, kept
as they are) plus the PGC-03 fixtures, all in the -X half of the floor, clear
of the older cases around the origin:

- LEDGE-01 platforms, 400 x 600, heights 40 / 80 / 120, centred at
  x = -3400 / -2700 / -2000, y = 0.
- SLOPE-01 ramps rising along +X to a 200-unit plateau, 400 wide, at
  30 / 40 / 50 degrees, centred on y = 1200 / 2000 / 2800, the ramp foot at
  x = -3600.
- STAIRJUMP-01 a tall staircase: a 300-long top at 192 units, then 8 steps
  of 24 down along -X (40 deep; the eighth is the floor), centred on y = -1800, the top's +X edge at
  x = -2900.
- WALL-01 a wall 200 high along Y at x = -1500 (y -300..300).
- DROP-01 towers (the drop roll, PGC-03 C8b), 300 x 400, heights
  300 / 500 / 800, centred at x = -3400 / -2700 / -2000, y = -2650 (the -Y
  edge at y = -2850, open floor beyond it to y = -4096).

Per-vertex normals (flat per triangle) so the films read the shapes.
Idempotent: run it again after changing the fixtures.
"""
import math
import re
from pathlib import Path

MESH = Path(__file__).resolve().parents[1] / 'scripts/data/ragdoll_manual/meshes/ragdoll_test_floor.osgt'
ORIGINAL = 192


def box(x0, x1, y0, y1, z0, z1):
    """Triangles of a closed box (outward winding)."""
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quads = [(4, 5, 6, 7), (3, 2, 1, 0), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    tris = []
    for a, b, c, d in quads:
        tris += [(p[a], p[b], p[c]), (p[a], p[c], p[d])]
    return tris


def wedge(x0, run, y0, y1, height):
    """A ramp rising along +X from (x0, 0) to (x0 + run, height)."""
    x1 = x0 + run
    a, b, c, d = (x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)
    e, f = (x1, y0, height), (x1, y1, height)
    return [
        (a, e, f), (a, f, d),  # the slope
        (d, c, b), (d, b, a),  # bottom
        (b, c, f), (b, f, e),  # the high end
        (a, b, e),  # side y0
        (d, f, c),  # side y1
    ]


def normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / n, ny / n, nz / n


def fixtures():
    tris = []
    for x, h in ((-3400, 40), (-2700, 80), (-2000, 120)):
        tris += box(x - 200, x + 200, -300, 300, 0, h)
    for y, angle in ((1200, 30), (2000, 40), (2800, 50)):
        height = 200.0
        run = height / math.tan(math.radians(angle))
        tris += wedge(-3600, run, y - 200, y + 200, height)
        tris += box(-3600 + run, -3600 + run + 300, y - 200, y + 200, 0, height)
    # WALL-01: a wall 200 high, 20 thick, along Y at x = -1500.
    tris += box(-1500, -1480, -300, 300, 0, 200)
    top = 192
    tris += box(-3200, -2900, -2000, -1600, 0, top)
    for i in range(7):
        h = top - 24 * (i + 1)
        x1 = -3200 - 40 * i
        tris += box(x1 - 40, x1, -2000, -1600, 0, h)
    # DROP-01: towers to run off (a 500 drop lands at ~790 u/s, past the
    # hard-landing release; 300 stays under the roll's threshold).
    for x, h in ((-3400, 300), (-2700, 500), (-2000, 800)):
        tris += box(x - 150, x + 150, -2850, -2450, 0, h)
    return tris


def main():
    text = MESH.read_text()  # newline translation: CRLF in, written back below
    # The second drawable's vertex vector (after "UniqueID 22").
    m = re.search(r'(UniqueID 22\s*\n\s*Binding BIND_PER_VERTEX\s*\n\s*vector )(\d+)( \{\n)(.*?)(\n\s*\}\n)', text, re.S)
    if not m:
        raise SystemExit('staircase vertex array not found')
    lines = [l for l in m.group(4).split('\n') if l.strip()]
    original = lines[:ORIGINAL]
    indent = re.match(r'\s*', original[0]).group(0)
    stair = []
    for i in range(0, ORIGINAL, 3):
        stair.append(tuple(tuple(float(v) for v in original[i + k].split()) for k in range(3)))
    tris = stair + fixtures()
    verts = [v for t in tris for v in t]
    fmt = lambda v: '%g %g %g' % v
    body = '\n'.join(indent + fmt(v) for v in verts)
    text = text[:m.start()] + m.group(1) + str(len(verts)) + m.group(3) + body + m.group(5) + text[m.end():]
    # Draw count.
    text = re.sub(r'(UniqueID 21\s*\n\s*Mode TRIANGLES\s*\n\s*First 0\s*\n\s*Count )\d+', r'\g<1>%d' % len(verts), text)
    # Normals per vertex (the second drawable's NormalArray, UniqueID 23).
    normals = []
    for t in tris:
        normals += [normal(t)] * 3
    nbody = '\n'.join(indent + fmt(n) for n in normals)
    text = re.sub(r'(UniqueID 23\s*\n\s*Binding )BIND_\w+(\s*\n\s*vector )\d+ \{[^}]*\}',
                  lambda mm: mm.group(1) + 'BIND_PER_VERTEX' + mm.group(2) + '%d {\n%s\n%s}' % (
                      len(normals), nbody, indent[:-2]), text)
    MESH.write_text(text, newline=chr(13) + chr(10))
    print(f'{MESH}: {len(verts)} vertices ({len(verts) - ORIGINAL} new)')


if __name__ == '__main__':
    main()
