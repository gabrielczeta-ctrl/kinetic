#!/usr/bin/env python3
"""Batch captured-body qualification with one review summary.

Candidates share the same screening and hold-out cases. A cheap tiled-floor
screen removes only catastrophic regressions; survivors run one 300+ case
hold-out on box, mesh, tiled, and tiled-with-wandering-input conditions.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import calibration
import ensemble


CONDITIONS = {
    "box": {"profile": {}, "env": {"OPENMW_RAGDOLL_PHYS_MANIFOLD": 0}, "wander": False},
    "mesh": {"profile": {"mesh_ground": 1}, "env": {"OPENMW_RAGDOLL_PHYS_MANIFOLD": 0}, "wander": False},
    "tiled": {"profile": {"mesh_ground": 1}, "env": {
        "OPENMW_RAGDOLL_MESH_GRID": 64, "OPENMW_RAGDOLL_PHYS_MANIFOLD": 0,
    }, "wander": False},
    "wander": {"profile": {"mesh_ground": 1}, "env": {
        "OPENMW_RAGDOLL_MESH_GRID": 64, "OPENMW_RAGDOLL_PHYS_MANIFOLD": 0,
    }, "wander": True},
}


def load_candidates(path: Path | None) -> list[dict[str, Any]]:
    if path is None:
        return []
    doc = json.loads(path.read_text(encoding="utf-8-sig"))
    rows = doc.get("candidates") if isinstance(doc, dict) else None
    if not isinstance(rows, list):
        raise ValueError("candidate file must contain a candidates list")
    labels: set[str] = set()
    out = []
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("label"), str):
            raise ValueError("each candidate needs a string label")
        label = row["label"].strip()
        if not label or label == "default" or label in labels:
            raise ValueError(f"invalid or duplicate candidate label: {label!r}")
        profile = row.get("profile", {})
        env = row.get("env", {})
        if not isinstance(profile, dict) or not isinstance(env, dict):
            raise ValueError(f"candidate {label} profile and env must be objects")
        unknown = set(env) - set(ensemble.ENV_KNOBS)
        if unknown:
            raise ValueError(f"candidate {label} has unknown env knobs: {', '.join(sorted(unknown))}")
        labels.add(label)
        out.append({"label": label, "profile": profile, "env": env})
    return out


def merged_candidate(candidate: dict[str, Any], condition: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    profile = {**candidate["profile"], **condition["profile"]}
    env = {**candidate["env"], **condition["env"]}
    return profile, env


def run_batch(
    exe: Path,
    candidates: list[dict[str, Any]],
    condition_name: str,
    n: int,
    seed: int,
    workers: int,
) -> list[dict[str, Any]]:
    condition = CONDITIONS[condition_name]
    cases = ensemble.sample_cases(n, seed, rates=(60,), wander=condition["wander"])
    inputs = [merged_candidate(candidate, condition) for candidate in candidates]
    evaluated = ensemble.evaluate_many(str(exe), inputs, cases, workers)
    return [
        {"label": candidate["label"], "condition": condition_name, "summary": summary}
        for candidate, (summary, _records) in zip(candidates, evaluated)
    ]


def screen_survivors(rows: list[dict[str, Any]]) -> tuple[list[str], list[dict[str, Any]]]:
    default = next(row for row in rows if row["label"] == "default")["summary"]
    survivors = ["default"]
    decisions = []
    for row in rows:
        if row["label"] == "default":
            continue
        summary = row["summary"]
        catastrophic = summary["crashes"] > 0 or summary["fall_rate"] > max(0.25, default["fall_rate"] + 0.10)
        decisions.append({
            "label": row["label"],
            "survived": not catastrophic,
            "reason": "catastrophic tiled regression" if catastrophic else "advance to hold-out",
            "fall_rate": summary["fall_rate"],
            "default_fall_rate": default["fall_rate"],
            "crashes": summary["crashes"],
        })
        if not catastrophic:
            survivors.append(row["label"])
    return survivors, decisions


def summary_text(report: dict[str, Any]) -> str:
    lines = [
        f"source {report['source_commit']}",
        f"screen n={report['screen']['n']} seed={report['screen']['seed']}",
        f"holdout n={report['contract']['sample_n_per_condition']} seed={report['contract']['holdout_seed']}",
        "",
        "SCREEN",
    ]
    for row in report["screen"]["results"]:
        s = row["summary"]
        lines.append(f"{row['label']:<18} fall={s['fall_rate']:.3f} pass={s['pass_rate']:.3f} crashes={s['crashes']}")
    lines.extend(["", "PROMOTION"])
    for row in report["promotion_results"]:
        s = row["summary"]
        lines.append(
            f"{row['label']:<18} {row['condition']:<7} fall={s['fall_rate']:.3f} "
            f"pass={s['pass_rate']:.3f} score={s['score']:.3f} crashes={s['crashes']}"
        )
    lines.extend(["", "Review this summary first; open qualification.json only for a failing axis."])
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = calibration.repository_root()
    default_context, default_profile = calibration.canonical_paths(root)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--context", type=Path, default=default_context)
    parser.add_argument("--profile", type=Path, default=default_profile)
    parser.add_argument("--search-seed", type=int, action="append", required=True)
    parser.add_argument("--holdout-seed", type=int, default=777)
    parser.add_argument("--n", type=int, default=300)
    parser.add_argument("--screen-n", type=int, default=48)
    parser.add_argument("--workers", type=int, default=max(1, min(12, os.cpu_count() or 1)))
    parser.add_argument("--tracker-task")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    try:
        metadata = calibration.qualification_metadata(
            args.context, args.profile, args.n, args.search_seed, args.holdout_seed, root=root,
        )
        contract = calibration.require_contract(metadata)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if not args.exe.is_file():
        parser.error(f"walksearch executable does not exist: {args.exe}")
    candidates = [{"label": "default", "profile": {}, "env": {}}] + load_candidates(args.candidates)
    ensemble.NATIVE_BODY[:] = ["--native-body", str(args.context.resolve()), str(args.profile.resolve())]
    ensemble.SHOVE2[0] = False
    started = time.time()
    screen = run_batch(args.exe.resolve(), candidates, "tiled", args.screen_n, args.search_seed[0], args.workers)
    survivor_labels, decisions = screen_survivors(screen)
    survivors = [candidate for candidate in candidates if candidate["label"] in survivor_labels]
    promotion = []
    for condition in calibration.PROMOTION_CONDITIONS:
        promotion.extend(run_batch(
            args.exe.resolve(), survivors, condition, args.n, args.holdout_seed, args.workers,
        ))
    source_commit = calibration.git_head(root)
    report = {
        "schema_version": 1,
        "generated": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "source_commit": source_commit,
        "command": "python scripts/ragdoll-search/qualify.py " + " ".join(sys.argv[1:] if argv is None else argv),
        "exe": str(args.exe.resolve()),
        "contract": metadata,
        "contract_checks": contract,
        "screen": {"n": args.screen_n, "seed": args.search_seed[0], "results": screen, "decisions": decisions},
        "promotion_results": promotion,
        "wall_seconds": round(time.time() - started, 1),
        "tracker": {
            "producer": "scripts/ragdoll-search/qualify.py",
            "total_checks": len(contract),
            "failed_checks": sum(not passed for passed in contract.values()),
        },
    }
    artifact = args.out / "qualification.json"
    artifact.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    (args.out / "SUMMARY.txt").write_text(summary_text(report), encoding="utf-8", newline="\n")
    print(summary_text(report), end="")
    if args.tracker_task:
        subprocess.run([
            sys.executable, str(root / "tools/tracker/write_result.py"),
            "--root", str(root), "--task", args.tracker_task,
            "--producer", "qualification_v1", "--artifact", str(artifact),
        ], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
