"""The mouse against the weapon: one play recording as data for the mouse
viewer (tools/body/mouse_view.html), to compare what the mouse asked with
what hand control did and what the arm did (user 2026-10-04: "a visualizer
to know what the mouse is doing ... compare the expected with what is going
on").

Per physics step: the mouse (counts, up positive) and its button; hand
control's stance point, stage, line, route position and chain/snap state
(recorded since build 46; older recordings get the stance circle replayed
from the mouse, approximately, as tools/body/stance_sim.py); the commanded and
physical grip, tip and blade, the right elbow and shoulder, the left hand
and the wrists past their limits. Positions are in the actor's frame (x
right, y forward, z up) from the torso. Bookmarks and their notes come along.

usage: python tools/body/mouse_view.py RECORDING_DIR [--out FILE.json]
       [--html FILE.html] [--from TICK --to TICK] [--around BOOKMARK --window S]
--html writes the viewer with the data inside (opens offline).
"""
import argparse
import glob
import json
import math
from pathlib import Path

VIEWER = Path(__file__).with_name('mouse_view.html')
# The stance knobs (passiveragdollentry.inl, build 45 defaults).
KNOBS = {'radius': 300, 'commit': 1500, 'cut': 3000, 'cut_speed': 1500, 'snap_speed': 3500, 'run_on': 2500}


def rows(directory):
    paths = sorted(glob.glob(directory + '/ticks-*.jsonl'), key=lambda p: int(p.rsplit('-', 1)[1].split('.')[0]))
    for path in paths:
        for line in open(path, encoding='utf-8'):
            try:
                yield json.loads(line)
            except ValueError:
                continue  # a session cut off mid-line


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v] if n > 1e-6 else None


class StanceSim:
    """Hand control's stance circle replayed from the mouse (stance_sim.py):
    for recordings without the recorded stance."""

    def __init__(self):
        self.px = self.py = self.vx = self.vy = 0.0
        self.stage, self.pend = 0, 0.0

    def press(self):
        self.stage, self.pend = 0, 0.0
        if math.hypot(self.px, self.py) > KNOBS['radius']:
            self.px = self.py = 0.0

    def step(self, dx, dy, dt):
        R, C = KNOBS['radius'], KNOBS['commit']
        k = min(1.0, dt / 0.03)
        self.vx += (dx / dt - self.vx) * k
        self.vy += (dy / dt - self.vy) * k
        if self.stage == 2:
            return
        px, py = self.px, self.py
        r = math.hypot(px, py)
        if r > R:
            ex, ey = px / r, py / r
            dr = dx * ex + dy * ey
            tx, ty = dx - ex * dr, dy - ey * dr
            tl = math.hypot(tx, ty)
            inward = -(self.vx * ex + self.vy * ey)
            if dr < 0 and tl <= abs(dr):
                self.pend += -dr
            elif dr > 0:
                self.pend = 0
            if inward > KNOBS['cut_speed'] and dr < 0 and r > R + 0.2 * C:
                self.stage = 2
                return
            if dr < 0 and tl <= abs(dr) and self.pend < 120:
                pass
            elif tl <= abs(dr):
                px += dx
                py += dy
            else:
                need = r + dr - R
                if need <= 0:
                    px, py = ex * (r + dr) + tx, ey * (r + dr) + ty
                elif tl <= need:
                    rn = min(r + dr - tl, R + C)
                    px, py = ex * rn, ey * rn
                else:
                    left = tl - need
                    px, py = ex * R + tx / tl * left, ey * R + ty / tl * left
        else:
            if (math.hypot(self.vx, self.vy) > KNOBS['snap_speed'] and r > 0.3 * R
                    and self.vx * px + self.vy * py < 0 and self.stage == 1):
                self.stage = 2
                return
            px += dx
            py += dy
        rn = math.hypot(px, py)
        if rn > R + C:
            px, py = px * (R + C) / rn, py * (R + C) / rn
            rn = R + C
        if not (self.stage == 0 and rn < 25):
            self.stage = 1
        self.px, self.py = px, py


def convert(directory, first=None, last=None):
    inputs, ticks, bookmarks, notes = {}, [], {}, {}
    for row in rows(directory):
        kind = row.get('kind')
        if kind == 'input':
            e = inputs.setdefault(row['after_tick'], [0.0, 0.0, 0, 0])
            if not row.get('gui'):
                e[0] += row['dx']
                e[1] -= row['dy']
            e[2] = 1 if row['buttons'] & 1 else 0
            e[3] = 1 if row.get('gui') else e[3]
        elif kind == 'tick':
            if (first is None or row['tick'] >= first) and (last is None or row['tick'] <= last):
                ticks.append(row)
        elif kind == 'bookmark':
            bookmarks[row['id']] = row['tick']
        elif kind == 'bookmark_note':
            notes[row['id']] = row['reason']
    cols = {k: [] for k in ('t', 'mx', 'my', 'lmb', 'gui', 'yaw', 'wpn', 'phase', 'attack', 'hp', 'hc', 'ep', 'sp',
                            'lh', 'tp', 'tc', 'bp', 'bc', 'pole', 'wr', 'wl', 'sup', 'drv', 'hsc', 'sim')}
    weapons, sim, held, has_hsc = [], StanceSim(), 0, False
    r1 = lambda v: [round(c, 1) for c in v] if v is not None else None
    r3 = lambda v: [round(c, 3) for c in v] if v is not None else None
    for row in ticks:
        tick = row['tick']
        bodies = {b['name']: b for b in row.get('bodies', [])}
        torso = bodies.get('torso')
        yaw = row.get('actor_yaw', 0.0)
        fwd = (math.sin(yaw), math.cos(yaw), 0.0)
        right = (math.cos(yaw), -math.sin(yaw), 0.0)
        origin = torso['physical'] if torso and torso['valid'][2] else row.get('actor_pos', [0, 0, 0])

        def local(p, direction=False):
            if p is None:
                return None
            d = p if direction else [p[i] - origin[i] for i in range(3)]
            return [d[0] * right[0] + d[1] * right[1], d[0] * fwd[0] + d[1] * fwd[1], d[2]]

        def phys(name):
            b = bodies.get(name)
            return b['physical'] if b and b['valid'][2] else None

        def command(name):
            b = bodies.get(name)
            return b['command'] if b and b['valid'][1] else None

        dx, dy, lmb, gui = inputs.get(tick - 1, (0.0, 0.0, held, 0))
        if lmb and not held:
            sim.press()
        held = lmb
        dt = row.get('dt') or 1 / 60
        if lmb:
            sim.step(dx, dy, dt)
        else:
            sim.vx = sim.vy = 0.0
            sim.stage = 0
        weapon = row.get('weapon', '')
        if weapon not in weapons:
            weapons.append(weapon)
        hsc = row.get('hsc')
        has_hsc |= bool(hsc)
        hand_p = phys('right_hand')
        tip_p = row['tip'] if row.get('tip_valid', [0, 0])[0] else None
        tip_c = row['tip_cmd'] if row.get('tip_valid', [0, 0])[1] else None
        hand_c = hsc['grip'] if hsc and hsc.get('on') else command('right_hand')
        blade_p = unit([tip_p[i] - hand_p[i] for i in range(3)]) if tip_p and hand_p else None
        if hsc and hsc.get('on'):
            blade_c = unit(hsc['blade'])
        else:
            blade_c = unit([tip_c[i] - hand_c[i] for i in range(3)]) if tip_c and hand_c else None
        cols['t'].append(tick)
        cols['mx'].append(round(dx))
        cols['my'].append(round(dy))
        cols['lmb'].append(lmb)
        cols['gui'].append(gui)
        cols['yaw'].append(round(yaw, 4))
        cols['wpn'].append(weapons.index(weapon))
        cols['phase'].append(row.get('phase', 0))
        cols['attack'].append(row.get('attack', 0))
        cols['hp'].append(r1(local(hand_p)))
        cols['hc'].append(r1(local(hand_c)))
        cols['ep'].append(r1(local(phys('right_elbow'))))
        cols['sp'].append(r1(local(phys('right_shoulder'))))
        cols['lh'].append(r1(local(phys('left_hand'))))
        cols['tp'].append(r1(local(tip_p)))
        cols['tc'].append(r1(local(tip_c)))
        cols['bp'].append(r3(local(blade_p, True)))
        cols['bc'].append(r3(local(blade_c, True)))
        cols['pole'].append(r3(local(hsc['pole'], True)) if hsc and 'pole' in hsc else None)
        we = row.get('wrist_excess', [-1, -1])
        cols['wl'].append(round(we[0], 1))
        cols['wr'].append(round(we[1], 1))
        cols['sup'].append(row.get('support', '').split(':')[0])
        # the weapon drive, the tip controller or hand control owns the arm
        cols['drv'].append(1 if row.get('drive') or row.get('tip_mode', -1) >= 0 or (hsc and hsc.get('on')) else 0)
        if hsc:
            keep = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in hsc.items()
                    if k not in ('grip', 'blade', 'pole')}
            cols['hsc'].append(keep)
        else:
            cols['hsc'].append(None)
        cols['sim'].append([round(sim.px), round(sim.py), sim.stage])
    marks = [{'id': i, 'tick': t, 'note': notes.get(i, '')} for i, t in sorted(bookmarks.items())]
    return {'source': str(directory), 'knobs': KNOBS, 'weapons': weapons, 'has_hsc': has_hsc,
            'bookmarks': marks, 'cols': cols}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directory')
    ap.add_argument('--out')
    ap.add_argument('--html')
    ap.add_argument('--from', dest='first', type=int)
    ap.add_argument('--to', dest='last', type=int)
    ap.add_argument('--around', type=int, help='bookmark id')
    ap.add_argument('--window', type=float, default=8.0, help='seconds before and after --around')
    args = ap.parse_args()
    directory = args.directory.rstrip('/\\')
    if not glob.glob(directory + '/ticks-*.jsonl'):
        directory += '/manual-recording'
    first, last = args.first, args.last
    if args.around is not None:
        tick = next(r['tick'] for r in rows(directory) if r.get('kind') == 'bookmark' and r['id'] == args.around)
        first, last = tick - int(args.window * 60), tick + int(args.window * 60 / 4)
    data = convert(directory, first, last)
    text = json.dumps(data, separators=(',', ':'))
    if args.out:
        Path(args.out).write_text(text, encoding='utf-8')
    if args.html:
        page = VIEWER.read_text(encoding='utf-8')
        page = page.replace('/*DATA*/null', text.replace('</', '<\\/'))
        Path(args.html).write_text(page, encoding='utf-8')
    print('%d steps, %d bookmarks, hand control recorded: %s, %.1f MB'
          % (len(data['cols']['t']), len(data['bookmarks']), data['has_hsc'], len(text) / 1e6))


if __name__ == '__main__':
    main()
