#!/usr/bin/env python3
"""Compare DUEL-A01's validated clock with its accelerated scripted preflight."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any


TEST = "DUEL-A01 animated attacks recover and resume"
FILTER = "^DUEL%-A01 animated attacks recover and resume$"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_run(directory: Path) -> dict[str, Any]:
    summary = json.loads((directory / "result-summary.json").read_text(encoding="utf-8"))
    manifest = json.loads((directory / "run-manifest.json").read_text(encoding="utf-8"))
    telemetry: list[dict[str, Any]] = []
    for path in sorted((directory / "test-results").glob("*.telemetry.jsonl")):
        telemetry.extend(json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line)
    cycles = sorted(
        (row for row in telemetry if row.get("scenario") == "duel_cycle"),
        key=lambda row: row["cycle"],
    )
    summaries = [row for row in telemetry if row.get("scenario") == "duel_summary"]
    suite = summary.get("suites", [{}])[0]
    case = suite.get("cases", [{}])[0]
    return {
        "directory": str(directory),
        "summary": summary,
        "manifest": manifest,
        "cycles": cycles,
        "duel_summary": summaries[0] if len(summaries) == 1 else {},
        "case_duration_ms": case.get("duration_ms"),
    }


def cycle_signature(run: dict[str, Any]) -> list[dict[str, Any]]:
    keys = (
        "cycle", "reactions", "opponent_resumed", "saw_falling", "saw_fallen",
        "saw_getting_up", "saw_blend_back",
    )
    return [{key: row.get(key) for key in keys} for row in run["cycles"]]


def summary_signature(run: dict[str, Any]) -> dict[str, Any]:
    row = run["duel_summary"]
    keys = (
        "cycles_requested", "cycles_completed", "reactions", "opponent_strikes",
        "soft_locks", "final_mode",
    )
    return {key: row.get(key) for key in keys}


def source_hashes_match(manifest: dict[str, Any]) -> bool:
    hashes = manifest.get("source_sha256", {})
    return bool(hashes) and all(Path(path).is_file() and sha256(Path(path)) == digest
                                for path, digest in hashes.items())


def build_report(root: Path, k1_directory: Path, k4_directory: Path, build_manifest_path: Path) -> dict[str, Any]:
    source_commit = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True, encoding="utf-8",
    ).strip()
    build = json.loads(build_manifest_path.read_text(encoding="utf-8"))
    k1 = load_run(k1_directory)
    k4 = load_run(k4_directory)
    k1_case = k1["summary"].get("suites", [{}])[0].get("cases", [{}])[0]
    k4_case = k4["summary"].get("suites", [{}])[0].get("cases", [{}])[0]
    duration_ratio = float(k4["case_duration_ms"]) / float(k1["case_duration_ms"])
    expected_summary = {
        "cycles_requested": 3,
        "cycles_completed": 3,
        "reactions": 3,
        "opponent_strikes": 3,
        "soft_locks": 0,
        "final_mode": "standing",
    }
    checks = {
        "source_commit_matches": all(
            run["summary"].get("source_commit") == source_commit for run in (k1, k4)
        ),
        "same_engine": k1["manifest"].get("engine_sha256") == k4["manifest"].get("engine_sha256"),
        "canonical_engine": k1["manifest"].get("engine_sha256", "").lower()
        == str(build.get("openmw_sha256", "")).lower(),
        "source_hashes_match": source_hashes_match(k1["manifest"])
        and source_hashes_match(k4["manifest"]),
        "exact_filters": all(
            run["manifest"].get("arguments", {}).get("test_filter") == FILTER for run in (k1, k4)
        ),
        "physics_rate": all(run["manifest"].get("physics_fps_environment") == "60"
                            for run in (k1, k4)),
        "clock_selection": k1["manifest"].get("ragdoll_environment", {}).get(
            "OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER") == "1"
        and k4["manifest"].get("ragdoll_environment", {}).get(
            "OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER") == "4",
        "both_cases_pass": k1_case.get("name") == k4_case.get("name") == TEST
        and k1_case.get("status") == k4_case.get("status") == "passed",
        "three_complete_cycles": len(k1["cycles"]) == len(k4["cycles"]) == 3,
        "cycle_signatures_match": cycle_signature(k1) == cycle_signature(k4),
        "summary_signatures_match": summary_signature(k1) == summary_signature(k4) == expected_summary,
        "accelerated_case_ratio": duration_ratio < 0.4,
    }
    return {
        "schema_version": 1,
        "producer": "scripts/duel-preflight/duel_preflight.py",
        "source_commit": source_commit,
        "build_manifest": str(build_manifest_path),
        "build_manifest_sha256": sha256(build_manifest_path),
        "case_duration_ms": {"k1": k1["case_duration_ms"], "k4": k4["case_duration_ms"]},
        "accelerated_case_ratio": duration_ratio,
        "signatures": {"k1": cycle_signature(k1), "k4": cycle_signature(k4)},
        "summary_signatures": {"k1": summary_signature(k1), "k4": summary_signature(k4)},
        "checks": checks,
        "failed_checks": [name for name, passed in checks.items() if not passed],
        "runs": {"k1": k1["directory"], "k4": k4["directory"]},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--k1", required=True, type=Path)
    parser.add_argument("--k4", required=True, type=Path)
    parser.add_argument("--build-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    report = build_report(
        args.root.resolve(), args.k1.resolve(), args.k4.resolve(), args.build_manifest.resolve(),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(f"DUEL-A01 preflight: {len(report['checks'])} checks, {len(report['failed_checks'])} failures")
    return 1 if report["failed_checks"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
