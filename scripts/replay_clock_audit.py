#!/usr/bin/env python3
"""Audit executed-step coverage and catch-up loss in an isolated replay trace.

This does not infer simulation-clock endpoints or walking acceptance. The
ten-step cap is the native PhysicsSystem::stepSimulation policy, not a gate.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re


def audit(lines, actor_kind):
    frames = []
    observed = {}
    actors = set()
    player = "1" if actor_kind == "player" else "0"
    for line in lines:
        if "RAGDOLL_REPLAY_FRAME " in line:
            fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
            frames.append((int(fields["frame"]), int(fields["step"])))
        elif "RAGDOLL_STEP phase=observed " in line:
            fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
            if fields.get("player") == player:
                actors.add(fields["actor"])
                step = int(fields["step"])
                if step in observed:
                    raise ValueError("duplicate observed step; isolated actor required")
                observed[step] = fields
    if len(actors) != 1 or not frames:
        raise ValueError("one isolated actor and one nonempty recording required")
    if any(frame != i for i, (frame, _) in enumerate(frames)):
        raise ValueError("nonconsecutive recording indices")
    missing = [step for _, step in frames if step not in observed]
    gaps = [frames[i][1] for i in range(1, len(frames))
        if frames[i][1] != frames[i - 1][1] + 1]
    if missing or gaps:
        raise ValueError(f"executed-step coverage incomplete: missing={missing}, gaps={gaps}")
    renders = defaultdict(list)
    for _, step in frames:
        row = observed[step]
        renders[int(row["render"])].append(row)
    capped = []
    physics_duration = 0.0
    for render, rows in renders.items():
        for row in rows:
            dt = float(row["physics_dt"])
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError("invalid physics timestep")
            physics_duration += dt
        last = rows[-1]
        # substep is necessary: a recording may begin within a render batch.
        dt = float(last["physics_dt"])
        backlog = float(last["backlog"])
        if int(last["substep"]) == 10 and backlog >= dt:
            dropped = math.floor(backlog / dt)
            capped.append({"render": render, "last_step": int(last["step"]),
                "render_dt": float(last["render_dt"]),
                "backlog_before_discard": backlog,
                "discarded_whole_steps": dropped,
                "discarded_seconds": dropped * dt})
    return {"actor_kind": actor_kind, "recorded_frames": len(frames),
        "first_step": frames[0][1], "last_step": frames[-1][1],
        "executed_step_coverage_complete": True,
        "recorded_physics_seconds": physics_duration,
        "catchup_cap_events": capped,
        "discarded_seconds": sum(row["discarded_seconds"] for row in capped),
        "scope": "executed steps and native catch-up policy; no wall-clock or walking acceptance claim"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--actor-kind", choices=("player", "npc"), default="player")
    args = parser.parse_args()
    with args.log.open(encoding="utf-8") as stream:
        report = audit(stream, args.actor_kind)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
