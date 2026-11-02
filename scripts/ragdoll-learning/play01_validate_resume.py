"""Bounded process-restart control for a prepared production residual allocation."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from play01_runner import atomic_json, sha256
from play01_watchdog import kill_child_tree


def run_bounded(command, cwd, timeout=360):
    """Bound the supervisor and its own descendants, including on outer failure."""
    isolation = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW}
                 if os.name == "nt" else {"start_new_session": True})
    process = subprocess.Popen(command, cwd=cwd, **isolation)
    try:
        process.communicate(timeout=timeout)
    except BaseException:
        # Killing only the supervisor would orphan the training runner. Its
        # PID is ours from Popen, so target precisely that descendant tree.
        kill_child_tree(process)
        raise
    if process.returncode:
        raise subprocess.CalledProcessError(process.returncode, command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--exe", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    out = Path(args.out).resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit("restart validation output must be empty")
    out.mkdir(parents=True, exist_ok=True)
    document = copy.deepcopy(json.loads(Path(args.manifest).read_text(encoding="utf-8")))
    document["run_id"] = "PLAY-01-production-residual-process-restart-control"
    document["target_usable_optimizer_transitions"] = 12960
    document["maximum_total_physics_steps"] = 80000
    document["maximum_cumulative_wall_seconds"] = 300
    document["stage_and_command_scope"]["command_speed_schedule_units_per_second"] = [
        {"through_usable_optimizer_transitions": 12960, "speed": 45.0}]
    document["checkpoint_interval_and_retention"]["evaluation_milestones_usable_transitions"] = [6480, 12960]
    document["reserved_evaluation_steps_wall_and_storage"]["physics_steps"] = 30600
    document["stop_and_continuation_rules"]["evaluation_guard"]["minimum_usable_transitions"] = 500000
    document["compute_allocation_and_authorization_reference"] = "User requested overnight preparation; this is the bounded process-restart integration control only (80k physics/300s per comparison arm)."
    manifest = out / "validation-manifest.json"
    atomic_json(manifest, document)
    runner = Path(args.source_root).resolve() / "scripts/ragdoll-learning/play01_runner.py"
    watchdog = runner.with_name("play01_watchdog.py")
    common = ["--manifest", str(manifest), "--source-root", str(Path(args.source_root).resolve()),
              "--exe", str(Path(args.exe).resolve()), "--library", str(Path(args.library).resolve()),
              "--control-mode", "production-residual", "--command-speed", "45", "--tracker-task", "PLAY-01"]
    commands = []
    for name, mode, stop in (("uninterrupted", "fresh", None), ("resumed", "fresh", 2), ("resumed", "exact-resume", None)):
        command = [sys.executable, str(runner), "train", *common, "--out", str(out / name), "--checkpoint-mode", mode]
        if stop is not None:
            command += ["--stop-after-updates", str(stop)]
        supervised = [sys.executable, str(watchdog), "--manifest", str(manifest),
                      "--out", str(out / name), "--", *command]
        commands.append(supervised)
        run_bounded(supervised, cwd=args.source_root)
    import torch
    records = []
    for name in ("uninterrupted", "resumed"):
        state = json.loads((out / name / "train-state.json").read_text(encoding="utf-8"))
        checkpoint = Path(state["latest_checkpoint"]["path"])
        if sha256(checkpoint) != state["latest_checkpoint"]["sha256"]:
            raise RuntimeError("validation checkpoint hash mismatch")
        value = torch.load(checkpoint, weights_only=False)
        digest = hashlib.sha256(b"".join(tensor.numpy().tobytes() for tensor in value["policy"].values())).hexdigest()
        records.append({"directory": str(out / name), "parameter_sha256": digest,
                        "checkpoint": state["latest_checkpoint"], "training_count": state["training_count"],
                        "milestones": state["completed_evaluation_milestones"], "numpy_rngs": value["numpy_rngs"],
                        "torch_rng_sha256": hashlib.sha256(value["torch_rng"].numpy().tobytes()).hexdigest(),
                        "counters": {k: v for k, v in state["counters"].items() if k != "worker_reset_seconds"}})
    fields = ("parameter_sha256", "training_count", "milestones", "numpy_rngs", "torch_rng_sha256", "counters")
    checks = {key: records[0][key] == records[1][key] for key in fields}
    result = {"schema": "play01_process_restart_validation_v1", "passed": all(checks.values()),
              "checks": checks, "arms": records, "commands": commands,
              "qualification_holdout_opened": False, "allocated_overnight_training_launched": False}
    atomic_json(out / "process-restart-result.json", result)
    print(json.dumps({"passed": result["passed"], "checks": checks}), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
