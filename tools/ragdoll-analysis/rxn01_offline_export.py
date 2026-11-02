#!/usr/bin/env python3
"""Export RXN-01 offline traces into the offline viewer's CSV format.

The existing tools/offline-viewer serves runtime-*/**/*-bodies.csv traces and
their matching metrics CSV. This exporter runs (or reads) the RXN-01 trace
producers and writes:

  <workspace>/runtime-viewer-rxn01/rxn01-duel-bodies.csv
  <workspace>/runtime-viewer-rxn01/rxn01-duel.csv
  <workspace>/runtime-viewer-rxn01/rxn01-walker-bodies.csv
  <workspace>/runtime-viewer-rxn01/rxn01-walker.csv
  <workspace>/runtime-viewer-rxn01/rxn01-reactions-bodies.csv
  <workspace>/runtime-viewer-rxn01/rxn01-reactions.csv

Duel bodies are exported with the second fighter offset by one body count, and
index.html draws the offset connections; metrics carry health, authority,
stability, swing and hit state so the existing plot and table work unchanged.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

BEHAVIOR_CODE = {
    "none": 0,
    "face_guard": 1,
    "arm_cradle": 2,
    "limp": 3,
    "doubled_over": 4,
    "stumble_flail": 5,
}


def write_bodies(path: Path, frames: list[dict], actor_pairs: list[tuple[str, int]]) -> None:
    """actor_pairs: (frameKey, bodyOffset) in draw order."""
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step", "body", "x", "y", "z"])
        for step, frame in enumerate(frames):
            for key, offset in actor_pairs:
                for body, point in enumerate(frame[key]["bodies"]):
                    writer.writerow([step, offset + body, f"{point[0]:.4f}",
                                     f"{point[1]:.4f}", f"{point[2]:.4f}"])


def write_metrics(path: Path, columns: list[str], rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for row in rows:
            writer.writerow([row.get(column, "") for column in columns])


def run_trace(exe: Path, env_name: str, env_path: Path, gtest_filter: str, log_path: Path) -> None:
    import os
    env = dict(os.environ)
    env[env_name] = str(env_path)
    run = subprocess.run([str(exe), f"--gtest_filter={gtest_filter}",
                          "--gtest_also_run_disabled_tests", "--gtest_brief=1"],
                         capture_output=True, text=True, env=env)
    log_path.write_text(run.stdout + run.stderr, encoding="utf-8", newline="\n")
    if run.returncode != 0 or not env_path.is_file():
        raise SystemExit(f"trace producer failed; see {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    out = (args.out or (root / "runtime-viewer-rxn01")).resolve()
    out.mkdir(parents=True, exist_ok=True)

    duel_trace = out / "rxn01-duel.json"
    viewer_trace = out / "rxn01-viewer.json"
    run_trace(args.exe, "OPENMW_RAGDOLL_DUEL_VIEWER", duel_trace,
              "RagdollCoreDuelScenarioTest.DISABLED_writesDuelViewerTrace", out / "duel-run.log")
    run_trace(args.exe, "OPENMW_RAGDOLL_REACTION_VIEWER", viewer_trace,
              "RagdollCoreReactionViewerTest.DISABLED_writesOfflineViewerTrace", out / "reaction-run.log")

    duel = json.loads(duel_trace.read_text(encoding="utf-8"))
    duel_bodies = len(duel["bodyNames"])
    duel_frames = duel["frames"]
    write_bodies(out / "rxn01-duel-bodies.csv", duel_frames, [("left", 0), ("right", duel_bodies)])
    rows = []
    for step, frame in enumerate(duel_frames):
        row = {"step": step, "time": frame["t"]}
        for side in ("left", "right"):
            actor = frame[side]
            row[f"{side}_health"] = actor["health"]
            row[f"{side}_alive"] = 1 if actor["alive"] else 0
            row[f"{side}_swinging"] = actor["swinging"]
            row[f"{side}_hit"] = actor["hit"]
            row[f"{side}_behavior"] = BEHAVIOR_CODE.get(actor["behavior"], -1)
        rows.append(row)
    write_metrics(out / "rxn01-duel.csv",
                  ["step", "time", "left_health", "right_health", "left_alive", "right_alive",
                   "left_swinging", "right_swinging", "left_hit", "right_hit",
                   "left_behavior", "right_behavior"], rows)

    viewer = json.loads(viewer_trace.read_text(encoding="utf-8"))
    reaction_frames = viewer["reactions"]["frames"]
    with (out / "rxn01-reactions-bodies.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step", "body", "x", "y", "z"])
        for step, frame in enumerate(reaction_frames):
            for body, point in enumerate(frame["bodies"]):
                writer.writerow([step, body, f"{point[0]:.4f}", f"{point[1]:.4f}", f"{point[2]:.4f}"])
    rows = []
    for step, frame in enumerate(reaction_frames):
        rows.append({
            "step": step,
            "time": frame["t"],
            "authority": frame["authority"],
            "stability": frame["stability"],
            "speed": frame["speed"],
            "intensity": frame["intensity"],
            "behavior": BEHAVIOR_CODE.get(frame["behavior"], -1),
            "phase": frame["phase"],
            "cover_face": 1 if frame["cover"] else 0,
            "cradle_left": 1 if frame["cradleL"] else 0,
            "cradle_right": 1 if frame["cradleR"] else 0,
        })
    write_metrics(out / "rxn01-reactions.csv",
                  ["step", "time", "authority", "stability", "speed", "intensity", "behavior",
                   "phase", "cover_face", "cradle_left", "cradle_right"], rows)

    with (out / "rxn01-walker-bodies.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step", "body", "x", "y", "z"])
        for step, frame in enumerate(viewer["walker"]["frames"]):
            for body, point in enumerate(frame["bodies"]):
                writer.writerow([step, body, f"{point[0]:.4f}", f"{point[1]:.4f}",
                                 f"{point[2]:.4f}"])
    rows = []
    for step, frame in enumerate(viewer["walker"]["frames"]):
        rows.append({"step": step, "time": frame["t"], "stage": frame["stage"],
                     "speed": frame["speed"], "left_supported": frame["left"],
                     "right_supported": frame["right"]})
    write_metrics(out / "rxn01-walker.csv",
                  ["step", "time", "stage", "speed", "left_supported", "right_supported"], rows)

    print(f"exported RXN-01 viewer traces under {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
