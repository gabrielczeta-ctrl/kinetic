"""Walking measures read from the game log, per actor (MOVE-01 S0, 11-03).

Reads the library's per-step leg trace and the stuck fallback:
  RAGDOLL_FOOTFALL side=.. swing=.. phase=.. foot=x,y,z pin=x,y,z target=x,y,z
      [goal=x,y,z] ... actor=".."     (OPENMW_RAGDOLL_FOOTFALL_TRACE=1 traces
                                       the player, =npc the NPCs, =all both)
  RAGDOLL_FOOTFALL_LAND side=.. miss=.. ... actor=".."
  RAGDOLL_NPC_STUCK actor=.. vanilla=1 / powered=1   (an NPC stuck on geometry
                                                      walks the vanilla capsule)
  RAGDOLL_NPC_STEEP actor=.. vanilla=1               (steep ground, the same)
  RAGDOLL_LOAD left=.. right=.. rootV=.. rootH=.. slipL=.. slipR=.. pelvisH=.. errZ=..
      contacts=feet/withImpulse other=.. otherBody=.. actor=..
                                                     (who carries the weight,
                                                      in body weights; 11-03)

Per actor group (the player, the NPCs):
  lift        median clearance of a swing: the highest the physical foot rose
              above the straight line from its pin to its target, units
  goal_lift   the same for the swing goal Footfall asked for (when the trace
              has goal=; the gap to `lift` is what the motors did not reach)
  land_miss   median distance a landing ended from its target, units
  swings      swings measured
  slide       walking (40-140 u/s) stance feet: median speed of the planted
              foot, u/s (a planted foot should not move), and slide_share the
              share of stance frames over 10 u/s
  legs, rootV, rootH   walking: median weight on both feet, the root's
              vertical and horizontal force, in body weights (RAGDOLL_LOAD)
  cslip       walking: median contact slip (u/s) of a foot carrying a third of
              the weight or more (RAGDOLL_LOAD slipL/slipR)
  hidden      walking: 1 - legs - rootV, the weight held by neither the feet
              nor the soft root's spring: the kinematic pelvis through the hip
              joints (over a stride the vertical forces sum to one weight)
  pelvisH     walking: median pelvis height over the lower foot (RAGDOLL_LOAD
              pelvisH=, 11-03)
  stance, double   walking: share of foot-frames planted, and of frames with
              both feet planted
  crossed     moving (15 u/s or more): share of frames with the left foot right
              of the right one across the body's facing (RAGDOLL_FOOTFALL yaw=,
              11-03; Morrowind yaw: forward is (sin, cos), right (cos, -sin))
  step_cv     walking: coefficient of variation of the times between
              successive lifts (either foot), in runs of steady walking
Fallback (NPCs): count of vanilla hand-overs (and how many came from steep
ground), distinct NPCs, median and longest closed hold (seconds from
vanilla=1 to powered=1), and holds still open when the log ends.

Used by play_metrics.py (play) and move_report.py (tests).

usage: python tools/body/walk_traces.py LOG [LOG ...]
"""
import math
import re
import statistics
import sys
from collections import defaultdict

STEP_LINE = re.compile(r'RAGDOLL_FOOTFALL side=(\d) swing=(\d) phase=([\d.-]+) foot=([-\d.]+),([-\d.]+),([-\d.]+)'
                       r' pin=([-\d.]+),([-\d.]+),([-\d.]+) target=([-\d.]+),([-\d.]+),([-\d.]+)'
                       r'(?: goal=([-\d.]+),([-\d.]+),([-\d.]+))?')
LAND = re.compile(r'RAGDOLL_FOOTFALL_LAND side=\d miss=([\d.]+)')
ACTOR = re.compile(r'actor=(?:"([^"]*)"|(\S+))\s*$')
FALLBACK = re.compile(r'RAGDOLL_NPC_(STUCK|STEEP) actor="?([^"]+?)"? (vanilla|powered)=1')
SPEED = re.compile(r'pelvis=[-\d.]+,[-\d.]+ speed=([\d.]+)')
LOAD_LINE = re.compile(r'RAGDOLL_LOAD left=([-\d.]+) right=([-\d.]+) rootV=([-\d.]+) rootH=([-\d.]+)'
                       r'(?: slipL=[-\d.]+ slipR=[-\d.]+ pelvisH=([-\d.]+))?(?: errZ=([-\d.]+))?')
SLIP = re.compile(r'slipL=([-\d.]+) slipR=([-\d.]+)')
WALK_BAND = (40.0, 140.0)
YAW = re.compile(r' yaw=([-\d.]+)')
CLOCK = re.compile(r'^\[(\d+):(\d+):([\d.]+)')


def actor_of(line):
    m = ACTOR.search(line.rstrip())
    return (m.group(1) if m.group(1) is not None else m.group(2)) if m else '?'


def seconds(line):
    m = CLOCK.match(line)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


def swings(lines):
    """{actor: [(lift, goal_lift or None)]} from the per-step trace."""
    open_swing = {}
    out = defaultdict(list)
    for line in lines:
        if 'RAGDOLL_FOOTFALL side=' not in line:
            continue
        m = STEP_LINE.search(line)
        if not m:
            continue
        key = (actor_of(line), m.group(1))
        swinging = m.group(2) == '1'
        if swinging:
            phase = max(0.0, min(1.0, float(m.group(3))))
            pin_z, target_z = float(m.group(9)), float(m.group(12))
            base = pin_z + (target_z - pin_z) * phase
            lift = float(m.group(6)) - base
            goal = float(m.group(15)) - base if m.group(15) else None
            best = open_swing.get(key)
            if best is None:
                open_swing[key] = [lift, goal]
            else:
                best[0] = max(best[0], lift)
                if goal is not None:
                    best[1] = goal if best[1] is None else max(best[1], goal)
        elif key in open_swing:
            out[key[0]].append(tuple(open_swing.pop(key)))
    return out


def stance_and_load(lines):
    """{group: {'slide': [...], 'legs': [...], 'rootV': [...], 'rootH': [...]}} while walking."""
    out = {g: defaultdict(list) for g in ('player', 'npc')}
    last = {}
    speed_of = {}
    for line in lines:
        if 'RAGDOLL_LOAD ' in line:
            m = LOAD_LINE.search(line)
            actor = actor_of(line)
            v = speed_of.get(actor)
            if m and v is not None and WALK_BAND[0] <= v < WALK_BAND[1]:
                d = out['player' if actor == 'player' else 'npc']
                d['legs'].append(float(m.group(1)) + float(m.group(2)))
                d['rootV'].append(float(m.group(3)))
                d['rootH'].append(float(m.group(4)))
                d['hidden'].append(1.0 - float(m.group(1)) - float(m.group(2)) - float(m.group(3)))
                if m.group(5) is not None and float(m.group(5)) >= 0.0:
                    d['pelvisH'].append(float(m.group(5)))
                if m.group(6) is not None:
                    d['errZ'].append(float(m.group(6)))
                # Contact slip of a foot carrying at least a third of the weight
                # (the heel-to-toe roll moves the foot's centre, not its contacts).
                sl = SLIP.search(line)
                if sl:
                    for load, slip in ((float(m.group(1)), float(sl.group(1))), (float(m.group(2)), float(sl.group(2)))):
                        if load >= 0.33:
                            d['cslip'].append(slip)
            continue
        if 'RAGDOLL_FOOTFALL side=' not in line:
            continue
        m = STEP_LINE.search(line)
        sp = SPEED.search(line)
        t = seconds(line)
        if not m or not sp or t is None:
            continue
        actor = actor_of(line)
        speed = float(sp.group(1))
        speed_of[actor] = speed
        key = (actor, m.group(1))
        foot = (float(m.group(4)), float(m.group(5)))
        planted = m.group(2) == '0'
        prev = last.get(key)
        if planted and prev and prev[3] and WALK_BAND[0] <= speed < WALK_BAND[1]:
            dt = t - prev[0]
            if 0.005 < dt < 0.05:
                out['player' if actor == 'player' else 'npc']['slide'].append(
                    ((foot[0] - prev[1]) ** 2 + (foot[1] - prev[2]) ** 2) ** 0.5 / dt)
        if WALK_BAND[0] <= speed < WALK_BAND[1]:
            group = out['player' if actor == 'player' else 'npc']
            group['stance'].append(1.0 if planted else 0.0)
            other = last.get((actor, '1' if m.group(1) == '0' else '0'))
            if m.group(1) == '1' and other and abs(other[0] - t) < 0.005:
                group['double'].append(1.0 if planted and other[3] else 0.0)
            # A lift: this foot planted on its last line, swinging now.
            if not planted and prev and prev[3]:
                lifts = group.setdefault('_lifts', {}).setdefault(actor, [])
                lifts.append(t)
        elif not (WALK_BAND[0] <= speed < WALK_BAND[1]):
            # Leaving the band ends a steady run of lifts.
            for group in out.values():
                if '_lifts' in group and actor in group['_lifts'] and group['_lifts'][actor]:
                    group.setdefault('_runs', []).append(group['_lifts'][actor])
                    group['_lifts'][actor] = []
        last[key] = (t, foot[0], foot[1], planted)
        # Crossed feet: on the right foot's line, against the left's last.
        yw = YAW.search(line)
        if yw and m.group(1) == '1' and speed >= 15.0:
            left = last.get((actor, '0'))
            if left and abs(left[0] - t) < 0.005:
                yaw = float(yw.group(1))
                rx, ry = math.cos(yaw), -math.sin(yaw)
                across = (foot[0] - left[1]) * rx + (foot[1] - left[2]) * ry
                out['player' if actor == 'player' else 'npc']['crossed'].append(1.0 if across < 0.0 else 0.0)
    for group in out.values():
        runs = group.pop('_runs', []) + [v for v in group.pop('_lifts', {}).values() if v]
        for run in runs:
            gaps = [b - a for a, b in zip(run, run[1:]) if 0.1 < b - a < 1.5]
            if len(gaps) >= 6:
                group['step_cv'].append(statistics.pstdev(gaps) / statistics.mean(gaps))
    return out


def summary(lines):
    """Flat numbers: lift_player, goal_lift_player, land_miss_player,
    swings_player and the same with _npc; fallback_n, fallback_npcs,
    fallback_hold_p50, fallback_hold_max."""
    out = {}
    groups = {'player': [], 'npc': []}
    for actor, values in swings(lines).items():
        groups['player' if actor == 'player' else 'npc'].extend(values)
    misses = {'player': [], 'npc': []}
    for line in lines:
        if 'RAGDOLL_FOOTFALL_LAND' in line:
            m = LAND.search(line)
            if m:
                misses['player' if actor_of(line) == 'player' else 'npc'].append(float(m.group(1)))
    for group in ('player', 'npc'):
        lifts = [v[0] for v in groups[group]]
        goals = [v[1] for v in groups[group] if v[1] is not None]
        out['swings_' + group] = len(lifts)
        out['lift_' + group] = statistics.median(lifts) if lifts else None
        out['goal_lift_' + group] = statistics.median(goals) if goals else None
        out['land_miss_' + group] = statistics.median(misses[group]) if misses[group] else None
    for group, d in stance_and_load(lines).items():
        slide = d['slide']
        out['slide_' + group] = statistics.median(slide) if slide else None
        out['slide_share_' + group] = sum(v > 10.0 for v in slide) / len(slide) if slide else None
        for name in ('legs', 'rootV', 'rootH', 'hidden', 'pelvisH', 'errZ', 'cslip'):
            out[name + '_' + group] = statistics.median(d[name]) if d[name] else None
        # Means too: over a stride the vertical forces sum to one weight,
        # while each median sits apart (a foot's load swings within a step).
        for name in ('legs', 'rootV', 'rootH'):
            out[name + '_mean_' + group] = statistics.mean(d[name]) if d[name] else None
        for name in ('stance', 'double', 'crossed'):
            out[name + '_' + group] = statistics.mean(d[name]) if d[name] else None
        out['step_cv_' + group] = statistics.median(d['step_cv']) if d['step_cv'] else None
    held = {}
    holds = []
    npcs = set()
    count = steep = 0
    for line in lines:
        t = seconds(line)
        m = FALLBACK.search(line)
        if not m:
            continue
        actor = m.group(2)
        if m.group(1) == 'STEEP':
            steep += 1  # always followed by its own STUCK line
        elif m.group(3) == 'vanilla':
            count += 1
            npcs.add(actor)
            held.setdefault(actor, t)
        elif actor in held:
            start = held.pop(actor)
            if start is not None and t is not None:
                holds.append((t - start) % 86400)
    out['fallback_n'] = count
    out['fallback_steep'] = steep
    out['fallback_npcs'] = len(npcs)
    out['fallback_hold_p50'] = statistics.median(holds) if holds else None
    out['fallback_hold_max'] = max(holds) if holds else None
    # Still on the vanilla capsule when the log ends (steep ground never re-powers).
    out['fallback_open'] = len(held)
    return out


def main():
    for path in sys.argv[1:]:
        lines = open(path, encoding='utf-8', errors='replace').read().splitlines()
        print(path)
        print('  ' + '  '.join('%s=%s' % (k, '-' if v is None else ('%.2f' % v if isinstance(v, float) else v))
                               for k, v in summary(lines).items()))


if __name__ == '__main__':
    main()
