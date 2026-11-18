"""The legs and the root per physics step, by the slope of the ground walked, for each traced actor.

usage: python tools/body/slope_report.py <openmw.log | *.log.gz | run or session dir> [--actor TEXT] [--min-seconds N]

Reads the traces OPENMW_RAGDOLL_FOOTFALL_TRACE writes (1 = the player, npc = the NPCs, all = both):
RAGDOLL_LEG_DRIVE, RAGDOLL_LOAD, RAGDOLL_FOOTFALL, RAGDOLL_FOOTFALL_LAND, RAGDOLL_LEGS_LEAD.
Per slope of the path (the pelvis target's height over the pelvis' way, 0.6 s back), moving upright:
  sag     the pelvis under its target (u), median / p90
  lift    the root's vertical force, body weights (mean);  legs  the feet's load, body weights (mean)
  none    steps with no foot loaded
  float   a pinned foot over the floor under it (u), p90
  brace   steps in the controller's Bracing state
  lead    steps with the legs-lead push on
  trail   how far the feet's middle trails the pelvis along the travel (u), median / p90, and the share of
          steps over 30 u (the chest ahead, the legs behind)
  miss    landings over 15 u from their target
"""
import collections
import glob
import gzip
import math
import os
import re
import sys

NUM = r"(-?[\d.]+)"
TIME = re.compile(r"^\[(\d\d):(\d\d):(\d\d\.\d+) ")
ACTOR = re.compile(r'actor=(?:"([^"]*)"|(\S+))')
ORDER = ["flat", "up 5-12", "up 12-25", "up 25+", "down 5-12", "down 12-25", "down 25+"]


def logs(path):
    if os.path.isfile(path):
        return [path]
    found = glob.glob(os.path.join(path, "openmw.log")) + glob.glob(os.path.join(path, "**", "*.log.gz"), recursive=True)
    return sorted(found)


def opened(path):
    return gzip.open(path, "rt", errors="replace") if path.endswith(".gz") else open(path, errors="replace")


def field(key, line):
    m = re.search(" " + key + "=" + NUM, line)
    return float(m.group(1)) if m else None


def actor_of(line):
    m = ACTOR.search(line)
    return (m.group(1) or m.group(2)) if m else None


def bucket(angle):
    if angle is None:
        return None
    if angle > 25:
        return "up 25+"
    if angle > 12:
        return "up 12-25"
    if angle > 5:
        return "up 5-12"
    if angle > -5:
        return "flat"
    if angle > -12:
        return "down 5-12"
    if angle > -25:
        return "down 12-25"
    return "down 25+"


def quantile(values, p):
    values = sorted(v for v in values if v is not None)
    return values[int(p * (len(values) - 1))] if values else float("nan")


def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else float("nan")


def read(path):
    """-> {actor: {"steps": [...], "lands": [...], "toggles": [...]}}"""
    actors = collections.defaultdict(lambda: {"steps": [], "lands": [], "toggles": [], "lead": False, "cur": None})
    previous, day = None, 0
    with opened(path) as handle:
        for line in handle:
            if "RAGDOLL_" not in line:
                continue
            m = TIME.match(line)
            if not m:
                continue
            t = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) + day
            if previous is not None and t < previous - 40000:
                day += 86400
                t += 86400
            previous = t
            if "RAGDOLL_LEG_DRIVE side=" in line:
                who = actors[actor_of(line)]
                if " side=0" in line:
                    who["cur"] = {"t": t, "mode": field("mode", line), "pz": field("pelvisZ", line),
                        "ptz": field("pelvisTargetZ", line), "pin": [field("pinned", line), None],
                        "fz": [field("footZ", line), None], "fl": [field("floorZ", line), None], "lead": who["lead"]}
                    who["steps"].append(who["cur"])
                elif who["cur"] is not None:
                    who["cur"]["pin"][1] = field("pinned", line)
                    who["cur"]["fz"][1] = field("footZ", line)
                    who["cur"]["fl"][1] = field("floorZ", line)
            elif "RAGDOLL_LOAD " in line:
                cur = actors[actor_of(line)]["cur"]
                if cur is not None:
                    for key in ("rootV", "left", "right", "slipL", "slipR"):
                        cur[key] = field(key, line)
            elif "RAGDOLL_FOOTFALL side=" in line:
                cur = actors[actor_of(line)]["cur"]
                if cur is None:
                    continue
                side = 0 if " side=0" in line else 1
                foot = re.search(" foot=" + NUM + "," + NUM, line)
                if foot:
                    cur.setdefault("feet", [None, None])[side] = (float(foot.group(1)), float(foot.group(2)))
                if side == 0:
                    cur["speed"] = field("speed", line)
                    pelvis = re.search(" pelvis=" + NUM + "," + NUM, line)
                    command = re.search(" cmd=" + NUM + "," + NUM, line)
                    if pelvis:
                        cur["x"], cur["y"] = float(pelvis.group(1)), float(pelvis.group(2))
                    if command:
                        cur["cmd"] = (float(command.group(1)), float(command.group(2)))
            elif "RAGDOLL_LEGS_LEAD actor=" in line:
                who = actors[actor_of(line)]
                who["lead"] = " on=1" in line
                who["toggles"].append(t)
            elif "RAGDOLL_FOOTFALL_LAND" in line:
                actors[actor_of(line)]["lands"].append((t, field("miss", line)))
    return actors


def slopes(steps):
    j = 0
    for i, step in enumerate(steps):
        step["slope"] = None
        if "x" not in step or step["ptz"] is None:
            continue
        while j < i and step["t"] - steps[j]["t"] > 0.6:
            j += 1
        k = j
        while k < i and ("x" not in steps[k] or steps[k]["ptz"] is None):
            k += 1
        back = steps[k]
        if k == i or step["t"] - back["t"] < 0.3:
            continue
        way = math.hypot(step["x"] - back["x"], step["y"] - back["y"])
        rise = step["ptz"] - back["ptz"]
        if way > 400 or abs(rise) > 250 or way < 8:
            continue
        step["slope"] = math.degrees(math.atan2(rise, way))


def trail(step):
    feet = step.get("feet")
    command = step.get("cmd")
    if not feet or feet[0] is None or feet[1] is None or not command or "x" not in step:
        return None
    speed = math.hypot(*command)
    if speed < 30:
        return None
    mx, my = (feet[0][0] + feet[1][0]) * 0.5, (feet[0][1] + feet[1][1]) * 0.5
    return ((step["x"] - mx) * command[0] + (step["y"] - my) * command[1]) / speed


def report(name, who, minimum):
    steps = who["steps"]
    slopes(steps)
    groups = collections.defaultdict(list)
    for step in steps:
        if step.get("speed") is None or step["speed"] < 30 or step["mode"] not in (0.0, 1.0):
            continue
        key = bucket(step["slope"])
        if key:
            groups[key].append(step)
    moving = sum(len(v) for v in groups.values()) / 60.0
    if moving < minimum:
        return
    toggles = sorted(who["toggles"])
    quick = sum(1 for a, b in zip(toggles, toggles[1:]) if b - a < 0.3)
    print("%s: %.1f s moving upright, legs-lead switches %d (%d within 0.3 s)" % (name, moving, len(toggles), quick))
    print("  %-10s %7s  %11s  %5s %5s %5s %6s %6s %5s  %16s" % (
        "slope", "seconds", "sag p50/p90", "lift", "legs", "none", "float", "brace", "lead", "trail p50/p90/>30"))
    for key in ORDER:
        rows = groups[key]
        if len(rows) < 30:
            continue
        sag = [r["ptz"] - r["pz"] for r in rows if r["ptz"] is not None and r["pz"] is not None]
        floats = [r["fz"][s] - r["fl"][s] for r in rows for s in (0, 1)
            if r["pin"][s] == 1.0 and r["fz"][s] is not None and r["fl"][s] is not None]
        loaded = [r for r in rows if r.get("left") is not None]
        trails = [v for v in (trail(r) for r in rows) if v is not None]
        print("  %-10s %7.1f  %5.1f /%5.1f  %5.2f %5.2f %4.0f%% %6.1f %5.0f%% %4.0f%%  %5.1f /%5.1f /%3.0f%%" % (
            key, len(rows) / 60.0, quantile(sag, 0.5), quantile(sag, 0.9), mean([r.get("rootV") for r in loaded]),
            mean([(r["left"] or 0) + (r["right"] or 0) for r in loaded]),
            100.0 * sum(1 for r in loaded if (r["left"] or 0) < 0.05 and (r["right"] or 0) < 0.05) / max(1, len(loaded)),
            quantile(floats, 0.9), 100.0 * sum(1 for r in rows if r["mode"] == 1.0) / len(rows),
            100.0 * sum(1 for r in rows if r["lead"]) / len(rows), quantile(trails, 0.5), quantile(trails, 0.9),
            100.0 * sum(1 for v in trails if v > 30) / max(1, len(trails))))
    index = 0
    missed = collections.defaultdict(list)
    for t, miss in who["lands"]:
        while index < len(steps) - 1 and steps[index]["t"] < t:
            index += 1
        key = bucket(steps[index]["slope"]) if steps else None
        if key and miss is not None:
            missed[key].append(miss)
    parts = ["%s %.0f%% of %d" % (key, 100.0 * sum(1 for v in missed[key] if v > 15) / len(missed[key]), len(missed[key]))
        for key in ORDER if len(missed[key]) >= 8]
    if parts:
        print("  landings over 15 u off: " + ", ".join(parts))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        return 2
    only = next((sys.argv[i + 1] for i, a in enumerate(sys.argv[:-1]) if a == "--actor"), None)
    minimum = float(next((sys.argv[i + 1] for i, a in enumerate(sys.argv[:-1]) if a == "--min-seconds"), 2))
    args = [a for a in args if a != only and a != str(minimum)] or args[:1]
    for path in logs(args[0]):
        actors = read(path)
        shown = False
        for name in sorted(k for k in actors if k):
            if only and only.lower() not in name.lower():
                continue
            if not shown:
                print(path)
                shown = True
            report(name, actors[name], minimum)
    return 0


if __name__ == "__main__":
    sys.exit(main())
