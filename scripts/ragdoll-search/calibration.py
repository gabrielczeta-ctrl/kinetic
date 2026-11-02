#!/usr/bin/env python3
"""Canonical captured-body calibration metadata and contract checks."""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any


BODY_ID = "context-symP"
PROFILE_ID = "openmw_humanoid_v1"
PRIMARY_FLOOR = "tiled"
PHYSICS_HZ = 60
MINIMUM_ENSEMBLE_N = 300
PROMOTION_CONDITIONS = ("box", "mesh", "tiled", "wander")


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def canonical_paths(root: Path | None = None) -> tuple[Path, Path]:
    base = (root or repository_root()).resolve()
    return (
        base / "files/data/ragdoll/calibration/context-symP.creation.txt",
        base / "files/data/ragdoll/humanoid-v1.yaml",
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_profile_id(path: Path) -> str:
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        match = re.match(r"^id:\s*([^#\s]+)", line)
        if match:
            return match.group(1).strip("'\"")
    raise ValueError(f"profile has no id: {path}")


def read_context_identity(path: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    if len(lines) < 7:
        raise ValueError(f"captured-body fixture is truncated: {path}")
    header = shlex.split(lines[0], posix=True)
    if len(header) != 4:
        raise ValueError(f"captured-body header is malformed: {path}")
    try:
        version, profile_hash, warmup = int(header[0]), int(header[2]), int(header[3])
        bone_count = int(lines[6].strip())
    except ValueError as error:
        raise ValueError(f"captured-body header is malformed: {path}") from error
    if len(lines) < 7 + bone_count:
        raise ValueError(f"captured-body skeleton is truncated: {path}")
    bones = []
    for line in lines[7:7 + bone_count]:
        fields = shlex.split(line, posix=True)
        if len(fields) != 9:
            raise ValueError(f"captured-body bone row is malformed: {path}")
        bones.append(fields[0])
    return {
        "format_version": version,
        "profile_id": header[1],
        "profile_hash": profile_hash,
        "warmup_frames": warmup,
        "bone_count": bone_count,
        "bone_names": bones,
    }


def fixture_identity(context: Path, profile: Path, root: Path | None = None) -> dict[str, Any]:
    canonical_context, canonical_profile = canonical_paths(root)
    context = context.resolve()
    profile = profile.resolve()
    context_header = read_context_identity(context)
    profile_id = read_profile_id(profile)
    return {
        "body_id": BODY_ID if sha256_file(context) == sha256_file(canonical_context) else "unregistered",
        "context_path": str(context),
        "context_sha256": sha256_file(context),
        "canonical_context_sha256": sha256_file(canonical_context),
        "profile_id": profile_id,
        "context_profile_id": context_header["profile_id"],
        "profile_sha256": sha256_file(profile),
        "canonical_profile_sha256": sha256_file(canonical_profile),
        "bone_count": context_header["bone_count"],
    }


def qualification_metadata(
    context: Path,
    profile: Path,
    sample_n: int,
    search_seeds: list[int],
    holdout_seed: int,
    conditions: list[str] | tuple[str, ...] = PROMOTION_CONDITIONS,
    root: Path | None = None,
) -> dict[str, Any]:
    return {
        "fixture": fixture_identity(context, profile, root),
        "primary_floor": PRIMARY_FLOOR,
        "manifold_reduction": False,
        "physics_hz": PHYSICS_HZ,
        "sample_n_per_condition": sample_n,
        "search_seeds": list(search_seeds),
        "holdout_seed": holdout_seed,
        "holdout_disjoint": bool(search_seeds) and holdout_seed not in search_seeds,
        "promotion_conditions": list(conditions),
    }


def contract_checks(metadata: dict[str, Any]) -> dict[str, bool]:
    fixture = metadata.get("fixture") if isinstance(metadata.get("fixture"), dict) else {}
    search_seeds = metadata.get("search_seeds")
    conditions = metadata.get("promotion_conditions")
    return {
        "registered_body": fixture.get("body_id") == BODY_ID,
        "context_hash": bool(fixture.get("context_sha256"))
        and fixture.get("context_sha256") == fixture.get("canonical_context_sha256"),
        "profile_id": fixture.get("profile_id") == PROFILE_ID
        and fixture.get("context_profile_id") == PROFILE_ID,
        "profile_hash": bool(fixture.get("profile_sha256"))
        and fixture.get("profile_sha256") == fixture.get("canonical_profile_sha256"),
        "captured_skeleton": fixture.get("bone_count") == 20,
        "primary_floor": metadata.get("primary_floor") == PRIMARY_FLOOR,
        "manifold_reduction_off": metadata.get("manifold_reduction") is False,
        "physics_rate": metadata.get("physics_hz") == PHYSICS_HZ,
        "ensemble_size": isinstance(metadata.get("sample_n_per_condition"), int)
        and not isinstance(metadata.get("sample_n_per_condition"), bool)
        and metadata["sample_n_per_condition"] >= MINIMUM_ENSEMBLE_N,
        "search_seeds_recorded": isinstance(search_seeds, list)
        and bool(search_seeds)
        and all(isinstance(seed, int) and not isinstance(seed, bool) for seed in search_seeds),
        "holdout_disjoint": metadata.get("holdout_disjoint") is True
        and isinstance(metadata.get("holdout_seed"), int)
        and isinstance(search_seeds, list)
        and metadata.get("holdout_seed") not in search_seeds,
        "promotion_matrix": isinstance(conditions, list)
        and len(conditions) == len(PROMOTION_CONDITIONS)
        and set(conditions) == set(PROMOTION_CONDITIONS),
    }


def require_contract(metadata: dict[str, Any]) -> dict[str, bool]:
    checks = contract_checks(metadata)
    failures = [name for name, passed in checks.items() if not passed]
    if failures:
        raise ValueError("qualification rejected by calibration contract: " + ", ".join(failures))
    return checks


def git_head(root: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout.strip()


def self_check(root: Path) -> dict[str, Any]:
    context, profile = canonical_paths(root)
    valid = qualification_metadata(context, profile, 300, [778], 777, root=root)
    valid_checks = contract_checks(valid)
    mutations = {
        "wrong_body": ("fixture.body_id", "context-symA"),
        "wrong_profile": ("fixture.profile_id", "openmw_humanoid_v0"),
        "wrong_floor": ("primary_floor", "box"),
        "manifold_on": ("manifold_reduction", True),
        "wrong_rate": ("physics_hz", 120),
        "undersized_ensemble": ("sample_n_per_condition", 299),
        "overlapping_holdout": ("holdout_seed", 778),
        "missing_promotion_condition": ("promotion_conditions", ["box", "mesh", "tiled"]),
    }
    negative_controls: dict[str, dict[str, Any]] = {}
    rejected: dict[str, bool] = {}
    for name, (path, value) in mutations.items():
        changed = copy.deepcopy(valid)
        target: dict[str, Any] = changed
        parts = path.split(".")
        for part in parts[:-1]:
            target = target[part]
        target[parts[-1]] = value
        if name == "overlapping_holdout":
            changed["holdout_disjoint"] = False
        negative_controls[name] = changed
        rejected[name] = not all(contract_checks(changed).values())
    checks = {
        "canonical_fixture_exists": context.is_file() and profile.is_file(),
        "canonical_fixture_valid": all(valid_checks.values()),
        **{f"rejects_{name}": value for name, value in rejected.items()},
    }
    return {
        "schema_version": 1,
        "generated": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "source_commit": git_head(root),
        "contract": valid,
        "checks": checks,
        "negative_controls": negative_controls,
        "tracker": {
            "producer": "scripts/ragdoll-search/calibration.py",
            "total_checks": len(checks),
            "failed_checks": sum(not passed for passed in checks.values()),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=repository_root())
    parser.add_argument("--out", type=Path)
    parser.add_argument("--tracker-task")
    args = parser.parse_args()
    root = args.root.resolve()
    report = self_check(root)
    output = args.out or root / "docs/ragdoll/evidence/raw" / f"calibration-{report['source_commit'][:12]}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(output)
    if report["tracker"]["failed_checks"]:
        return 1
    if args.tracker_task:
        writer = root / "tools/tracker/write_result.py"
        subprocess.run([
            sys.executable, str(writer), "--root", str(root), "--task", args.tracker_task,
            "--producer", "calibration_contract_v1", "--artifact", str(output), "--no-render",
        ], check=True)
        return subprocess.run(
            [sys.executable, str(root / "tools/tracker/render.py"), "--root", str(root)], check=False
        ).returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
