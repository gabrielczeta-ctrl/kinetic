"""Compare a manual session's movement input with where the capsule and the
physical body actually went, and measure body-vs-animation oscillation.

usage: session_travel.py RUNTIME_DIR [--csv OUT.csv]
"""
import json
import math
import sys
from pathlib import Path


def load(run, actor):
    ticks = []
    for chunk in sorted(Path(run, 'manual-recording').glob('ticks-*.jsonl')):
        for line in open(chunk, encoding='utf-8'):
            r = json.loads(line)
            if r.get('kind') == 'tick' and r.get('actor') == actor:
                ticks.append(r)
    return ticks


def body(r, name):
    for b in r.get('bodies') or []:
        if b.get('name') == name:
            return b
    return None


def main(run, csv=None):
    actors = sorted({json.loads(l).get('actor') for c in Path(run, 'manual-recording').glob('ticks-*.jsonl')
                     for l in open(c, encoding='utf-8') if '"kind":"tick"' in l})
    for actor in actors:
        print(f'=== actor {actor}')
        report(load(run, actor), csv and f'{csv}.{actor}.csv')


def report(ticks, csv=None):
    names = sorted({b['name'] for r in ticks for b in (r.get('bodies') or [])})
    print(f"{len(ticks)} ticks, bodies: {', '.join(names)}")
    rows = []
    for a, b in zip(ticks, ticks[1:]):
        dt = b.get('dt') or 1 / 60
        pa, pb = a['actor_pos'], b['actor_pos']
        capsule_v = (math.hypot(pb[0] - pa[0], pb[1] - pa[1]) / dt)
        ta, tb = body(a, 'torso'), body(b, 'torso')
        phys_v = anim_off = None
        if ta and tb:
            phys_v = math.hypot(tb['physical'][0] - ta['physical'][0], tb['physical'][1] - ta['physical'][1]) / dt
            anim_off = (tb['physical'][0] - tb['animation'][0], tb['physical'][1] - tb['animation'][1],
                        tb['physical'][2] - tb['animation'][2])
        move = b.get('move') or [0, 0, 0]
        rows.append(dict(tick=b['tick'], move=move, moving=abs(move[0]) > 0.1 or abs(move[1]) > 0.1,
                         yaw=b.get('actor_yaw'), capsule_v=capsule_v, phys_v=phys_v, off=anim_off))
    # Segments of constant input.
    segments = []
    for r in rows:
        key = (round(r['move'][0], 1), round(r['move'][1], 1), r['move'][2] if len(r['move']) > 2 else 0)
        if not segments or segments[-1]['key'] != key:
            segments.append(dict(key=key, rows=[]))
        segments[-1]['rows'].append(r)
    print(f"\n{'input (side,fwd,run)':>22} {'ticks':>5} {'capsule u/s':>11} {'body u/s':>9} "
          f"{'offset rms':>10} {'offset max':>10} {'osc Hz':>7} {'yaw rate':>8}")
    for s in segments:
        rs = s['rows']
        if len(rs) < 20:
            continue
        cv = sum(r['capsule_v'] for r in rs) / len(rs)
        pv = [r['phys_v'] for r in rs if r['phys_v'] is not None]
        offs = [r['off'] for r in rs if r['off'] is not None]
        horiz = [math.hypot(o[0], o[1]) for o in offs]
        mean_x = sum(o[0] for o in offs) / len(offs) if offs else 0
        mean_y = sum(o[1] for o in offs) / len(offs) if offs else 0
        # Oscillation: zero crossings of the offset around its mean (x and y).
        crossings = 0
        for axis, m in ((0, mean_x), (1, mean_y)):
            prev = None
            for o in offs:
                v = o[axis] - m
                if prev is not None and (v > 0) != (prev > 0) and abs(v - prev) > 0.5:
                    crossings += 1
                prev = v
        seconds = len(rs) / 60
        osc = crossings / 2 / 2 / seconds if seconds else 0
        yaws = [r['yaw'] for r in rs if r['yaw'] is not None]
        yaw_rate = 0.0
        if len(yaws) > 1:
            d = [math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(yaws, yaws[1:])]
            yaw_rate = math.degrees(sum(abs(x) for x in d) / seconds)
        rms = math.sqrt(sum(h * h for h in horiz) / len(horiz)) if horiz else float('nan')
        print(f"{str(s['key']):>22} {len(rs):5d} {cv:11.1f} {sum(pv) / len(pv) if pv else float('nan'):9.1f} "
              f"{rms:10.1f} {max(horiz) if horiz else float('nan'):10.1f} {osc:7.2f} {yaw_rate:8.0f}")
    if csv:
        with open(csv, 'w', encoding='utf-8') as out:
            out.write('tick,move_side,move_fwd,run,yaw,capsule_v,body_v,off_x,off_y,off_z\n')
            for r in rows:
                o = r['off'] or (float('nan'),) * 3
                out.write(f"{r['tick']},{r['move'][0]},{r['move'][1]},{r['move'][2] if len(r['move']) > 2 else 0},"
                          f"{r['yaw']},{r['capsule_v']:.2f},{r['phys_v'] if r['phys_v'] is not None else ''},"
                          f"{o[0]:.2f},{o[1]:.2f},{o[2]:.2f}\n")


if __name__ == '__main__':
    args = sys.argv[1:]
    csv = None
    if '--csv' in args:
        i = args.index('--csv')
        csv = args[i + 1]
        del args[i:i + 2]
    main(args[0], csv)
