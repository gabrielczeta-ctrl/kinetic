#!/usr/bin/env python3
"""Run one exact native test as isolated, parallel, identified repeat trials."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any


def file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def native_runtime_environment(root: Path, engine: Path, source: dict[str, str] | None = None) -> dict[str, str]:
    environment = dict(os.environ if source is None else source)
    version = json.loads((root / "docs/ragdoll/version-manifest.json").read_text(encoding="utf-8-sig"))
    bundle = version["dependencies"]["windows_bundle"]
    dependency = (root.parent / bundle["relative_path"]).resolve()
    installed = dependency / "installed" / bundle["triplet"]
    search = [installed / "bin/Release", installed / "bin"]
    existing = environment.get("PATH", "")
    environment["PATH"] = os.pathsep.join([*(str(path) for path in search if path.is_dir()), existing])
    environment["OSG_LIBRARY_PATH"] = str(engine.resolve().parent)
    return environment


def validate_documents(
    summary: dict[str, Any], manifest: dict[str, Any], source_commit: str,
    engine_hash: str, case_name: str, test_filter: str,
) -> tuple[dict[str, bool], str | None, float | None]:
    suites = summary.get("suites")
    suite = suites[0] if isinstance(suites, list) and len(suites) == 1 else {}
    cases = suite.get("cases")
    case = cases[0] if isinstance(cases, list) and len(cases) == 1 else {}
    arguments = manifest.get("arguments") if isinstance(manifest.get("arguments"), dict) else {}
    checks = {
        "source_commit_matches": summary.get("source_commit") == source_commit,
        "engine_hash_matches": manifest.get("engine_sha256") == engine_hash,
        "one_suite": isinstance(suites, list) and len(suites) == 1,
        "one_exact_case": isinstance(cases, list) and len(cases) == 1 and case.get("name") == case_name,
        "matched_count": suite.get("matched_tests") == 1,
        "complete_verdict": case.get("status") in ("passed", "failed"),
        "exact_filter": arguments.get("test_filter") == test_filter,
        "executed_not_listed": arguments.get("list_tests") is False,
        "source_hashes_recorded": isinstance(manifest.get("source_sha256"), dict)
        and bool(manifest.get("source_sha256")),
    }
    return checks, case.get("status"), case.get("duration_ms")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    parser.add_argument("--omw", required=True, type=Path)
    parser.add_argument("--workdir", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--data", action="append", default=[], type=Path)
    parser.add_argument("--ini", type=Path)
    parser.add_argument("--suite-timeout", type=float, default=300)
    parser.add_argument("--render-fps", type=int, choices=(30, 60, 120), default=60)
    parser.add_argument("--physics-threads", type=int, choices=range(65))
    parser.add_argument("--test-filter", required=True)
    parser.add_argument("--case-name", required=True)
    parser.add_argument("--repeat-trials", required=True, type=int, choices=range(1, 101))
    parser.add_argument("--parallel", required=True, type=int, choices=range(1, 17))
    parser.add_argument("--test-step-multiplier", type=int, choices=range(1, 17))
    parser.add_argument("--source-dependency", action="append", default=[], type=Path)
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    script = Path(__file__).resolve()
    root = script.parents[1]
    integration = script.with_name("integration_tests.py")
    source_commit = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True,
    ).strip()
    engine = args.omw.resolve()
    if not engine.is_file() or not args.suite.resolve().is_dir() or not args.config.resolve().is_dir():
        raise ValueError("engine, suite, and config must exist")
    output = args.workdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    engine_hash = file_hash(engine)
    started = time.perf_counter()

    def launch(trial_id: int) -> dict[str, Any]:
        directory = output / f"trial-{trial_id:03d}"
        directory.mkdir()
        selection = directory / "selected-tests.json"
        selection.write_text(json.dumps([args.case_name]) + "\n", encoding="utf-8", newline="\n")
        command = [
            sys.executable, str(integration), str(args.suite.resolve()),
            "--omw", str(engine), "--workdir", str(directory),
            "--config", str(args.config.resolve()), "--render_fps", str(args.render_fps),
            "--suite_timeout", str(args.suite_timeout), "--test_filter", args.test_filter,
            "--case_names", str(selection),
        ]
        for path in args.data:
            command += ["--data", str(path.resolve())]
        if args.ini:
            command += ["--ini", str(args.ini.resolve())]
        if args.physics_threads is not None:
            command += ["--physics_threads", str(args.physics_threads)]
        environment = {key: value for key, value in native_runtime_environment(root, engine).items()
                       if not key.startswith("OPENMW_RAGDOLL_")}
        environment.update({
            "SDL_WINDOW_NO_ACTIVATION_WHEN_SHOWN": "1",
            "SDL_HINT_FORCE_RAISEWINDOW": "0",
            "OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR": str(directory / "test-results/replays"),
        })
        if args.test_step_multiplier:
            environment["OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER"] = str(args.test_step_multiplier)
            environment["OPENMW_RAGDOLL_TEST_RENDER_FPS"] = str(args.render_fps)
        Path(environment["OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR"]).mkdir(parents=True)
        trial_started = time.perf_counter()
        with (directory / "launcher.log").open("w", encoding="utf-8") as log:
            process = subprocess.run(
                command, cwd=directory, env=environment, stdout=log, stderr=subprocess.STDOUT,
            )
        summary_path = directory / "result-summary.json"
        manifest_path = directory / "run-manifest.json"
        errors: list[str] = []
        summary = manifest = None
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            errors.append(f"missing or malformed result: {error}")
        checks: dict[str, bool] = {}
        verdict = None
        duration = None
        if summary is not None and manifest is not None:
            checks, verdict, duration = validate_documents(
                summary, manifest, source_commit, engine_hash, args.case_name, args.test_filter,
            )
            for name, passed in checks.items():
                if not passed:
                    errors.append(name)
            for path, digest in manifest.get("source_sha256", {}).items():
                if not Path(path).is_file() or file_hash(Path(path)) != digest:
                    errors.append(f"source hash mismatch: {path}")
        if process.returncode != 0 and verdict != "failed":
            errors.append(f"process exit {process.returncode}")
        telemetry = sorted((directory / "test-results").glob("*.telemetry.jsonl"))
        return {
            "trial_id": trial_id,
            "directory": str(directory),
            "exit_code": process.returncode,
            "wall_seconds": round(time.perf_counter() - trial_started, 3),
            "verdict": verdict,
            "duration_ms": duration,
            "checks": checks,
            "errors": errors,
            "summary_sha256": file_hash(summary_path) if summary_path.is_file() else None,
            "manifest_sha256": file_hash(manifest_path) if manifest_path.is_file() else None,
            "telemetry": [{"path": str(path), "sha256": file_hash(path)} for path in telemetry],
        }

    worker_count = min(args.parallel, args.repeat_trials)
    with ThreadPoolExecutor(max_workers=worker_count) as pool:
        trials = list(pool.map(launch, range(1, args.repeat_trials + 1)))
    errors = [f"trial-{trial['trial_id']:03d}: {error}"
              for trial in trials for error in trial["errors"]]
    checks = {
        "exact_trial_count": len(trials) == args.repeat_trials,
        "identified_trials": [trial["trial_id"] for trial in trials]
        == list(range(1, args.repeat_trials + 1)),
        "isolated_directories": len({trial["directory"] for trial in trials}) == args.repeat_trials,
        "complete_summaries": all(trial["summary_sha256"] and trial["manifest_sha256"] for trial in trials),
        "complete_verdicts": all(trial["verdict"] in ("passed", "failed") for trial in trials),
        "no_infrastructure_errors": not errors,
    }
    source_files = [script, integration]
    source_files += sorted(path for path in args.suite.resolve().glob("*") if path.is_file())
    source_files += [path.resolve() for path in args.source_dependency]
    def json_value(value: Any) -> Any:
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, list):
            return [json_value(item) for item in value]
        return value

    outer_manifest = {
        "source_commit": source_commit,
        "engine": str(engine),
        "engine_sha256": engine_hash,
        "arguments": {key: json_value(value) for key, value in vars(args).items()},
        "physics_fps_environment": os.environ.get("OPENMW_PHYSICS_FPS"),
        "source_sha256": {str(path): file_hash(path) for path in source_files},
    }
    summary = {
        "schema_version": 1,
        "generated": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "source_commit": source_commit,
        "manifest": "run-manifest.json",
        "case_name": args.case_name,
        "test_filter": args.test_filter,
        "repeat_trials": args.repeat_trials,
        "parallel": {"requested": args.parallel, "actual": worker_count,
                     "wall_seconds": round(time.perf_counter() - started, 3)},
        "test_step_multiplier": args.test_step_multiplier,
        "trials": trials,
        "checks": checks,
        "errors": errors,
        "tracker": {"producer": "native_smoke_repeat.py", "total_checks": len(checks),
                    "failed_checks": sum(not value for value in checks.values()) + len(errors)},
    }
    (output / "run-manifest.json").write_text(
        json.dumps(outer_manifest, indent=2) + "\n", encoding="utf-8", newline="\n",
    )
    (output / "result-summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8", newline="\n",
    )
    print(f"Repeated native smoke: {args.repeat_trials} trials, {worker_count} workers, "
          f"{len(errors)} infrastructure errors, {summary['parallel']['wall_seconds']:.3f}s")
    for error in errors:
        print(error, file=sys.stderr)
    failed_verdicts = sum(trial["verdict"] != "passed" for trial in trials)
    return int(bool(errors) or not all(checks.values()) or failed_verdicts > 0)


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
