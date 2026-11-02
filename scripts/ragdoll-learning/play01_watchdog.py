"""Supervise one declared PLAY-01 training process and its bounded work units.

The runner owns its output lock and all checkpoint/physics identities. This
outer process independently bounds wall time, including interpreter startup,
and leaves interrupted work reservations intact for conservative exact resume.
It never launches training merely by being imported or inspected.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any
import uuid


RUNNER = Path(__file__).resolve().with_name("play01_runner.py")


def read_object(path: Path, *, optional: bool = False) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        if optional:
            return None
        raise
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain an object")
    return value


def finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (positive and value == 0)):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return float(value)


def integer(value: Any, name: str, *, positive: bool = False) -> int:
    if type(value) is not int or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be a {'positive' if positive else 'nonnegative'} integer")
    return value


def option_value(arguments: list[str], option: str) -> str:
    matches = []
    for index, value in enumerate(arguments):
        if value == option:
            if index + 1 == len(arguments) or arguments[index + 1].startswith("--"):
                raise ValueError(f"missing value for child {option}")
            matches.append(arguments[index + 1])
        elif value.startswith(option + "="):
            matches.append(value.split("=", 1)[1])
    if len(matches) != 1 or not matches[0]:
        raise ValueError(f"child must declare {option} exactly once")
    return matches[0]


def validate_command(command: list[str], manifest: Path, out: Path) -> tuple[list[str], str]:
    """Require an explicit Python executable and this checkout's train entry."""
    command = list(command)
    if command and command[0] == "--":
        command.pop(0)
    if len(command) < 3:
        raise ValueError("expected -- PYTHON_PATH PLAY01_RUNNER_PATH train [arguments]")
    executable = Path(command[0]).resolve()
    if (not executable.is_file()
            or not re.fullmatch(r"python(?:w|[0-9.]*)?(?:\.exe)?", executable.name, re.IGNORECASE)):
        raise ValueError("child must start with a real Python executable path")
    index = 1
    while index < len(command) and command[index] in ("-u", "-B"):
        index += 1
    if (index + 1 >= len(command) or Path(command[index]).resolve() != RUNNER
            or command[index + 1] != "train" or not RUNNER.is_file()):
        raise ValueError("child must invoke this checkout's play01_runner.py train")
    arguments = command[index + 2:]
    if "--" in arguments:
        raise ValueError("unexpected child argument separator")
    if Path(option_value(arguments, "--manifest")).resolve() != manifest.resolve():
        raise ValueError("child --manifest does not match watchdog --manifest")
    if Path(option_value(arguments, "--out")).resolve() != out.resolve():
        raise ValueError("child --out does not match watchdog --out")
    mode = option_value(arguments, "--checkpoint-mode")
    if mode not in ("fresh", "transfer", "exact-resume"):
        raise ValueError("unsupported checkpoint mode")
    command[0], command[index] = str(executable), str(RUNNER)
    return command, mode


def pending_work(out: Path, state: dict[str, Any] | None = None) -> dict[str, Any] | None:
    """Read without creating, deleting or claiming the runner's ledger."""
    pending = read_object(out / "pending-work.json", optional=True)
    return resolve_pending(pending, state)


def resolve_pending(pending: dict[str, Any] | None, state: dict[str, Any] | None) -> dict[str, Any] | None:
    if pending is None:
        return None
    if pending.get("schema") != "play01_pending_work_v1" or not pending.get("reservation_id"):
        raise ValueError("invalid pending-work identity")
    base = integer(pending.get("base_checkpoint_generation"), "pending base generation")
    current = integer((state or {}).get("checkpoint_generation", 0), "checkpoint generation")
    integer(pending.get("physics_upper_bound"), "pending physics bound")
    integer(pending.get("owner_pid"), "pending owner PID", positive=True)
    finite_number(pending.get("wall_upper_bound"), "pending wall bound", positive=True)
    finite_number(pending.get("started_monotonic_seconds"), "pending monotonic start")
    if current < base:
        raise ValueError("pending work is newer than the committed state")
    return None if current > base else pending


@dataclass(frozen=True)
class LaunchPlan:
    command: list[str]
    manifest_path: Path
    out: Path
    manifest_sha256: str
    run_id: str
    maximum_wall_seconds: float
    previous_wall_seconds: float
    reserved_unknown_wall_seconds: float
    remaining_wall_seconds: float
    reserved_unknown_physics_steps: int
    previous_pending_id: str | None


def prepare_launch(manifest_path: Path, out: Path, command: list[str]) -> LaunchPlan:
    manifest_path, out = manifest_path.resolve(), out.resolve()
    child, mode = validate_command(command, manifest_path, out)
    manifest = read_object(manifest_path)
    maximum_wall = finite_number(manifest.get("maximum_cumulative_wall_seconds"), "manifest wall cap", positive=True)
    maximum_physics = integer(manifest.get("maximum_total_physics_steps"), "manifest physics cap", positive=True)
    sample_target = integer(manifest.get("target_usable_optimizer_transitions"), "manifest sample target", positive=True)
    manifest_sha = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    run_id = manifest.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        raise ValueError("manifest requires a run_id")
    state = read_object(out / "train-state.json", optional=True)
    if (mode == "exact-resume") != (state is not None):
        raise ValueError("exact-resume requires committed state; fresh/transfer require no committed state")
    previous_wall = 0.0
    physics = samples = recovered_physics = 0
    if state is not None:
        if state.get("run_id") != run_id or state.get("manifest_sha256") != manifest_sha:
            raise ValueError("committed state manifest identity mismatch")
        previous_wall = finite_number(state.get("cumulative_wall_seconds"), "committed wall counter")
        counters = state.get("counters")
        if not isinstance(counters, dict):
            raise ValueError("committed state requires counters")
        physics = integer(counters.get("total_simulated_physics_steps"), "committed physics counter")
        samples = integer(counters.get("training_usable_optimizer_transitions"), "committed usable counter")
        recovery = state.get("recovery_reservations", {})
        recovered_physics = integer(recovery.get("unknown_physics_steps_reserved", 0), "recovered physics reservation")
        # Recovered wall reservations already belong to cumulative_wall_seconds.
        finite_number(recovery.get("wall_seconds_reserved", 0), "recovered wall reservation")
    pending = pending_work(out, state)
    if pending is not None and state is None:
        raise ValueError("pending work without a committed state requires investigation")
    pending_wall = 0.0 if pending is None else float(pending["wall_upper_bound"])
    pending_physics = 0 if pending is None else pending["physics_upper_bound"]
    remaining = maximum_wall - previous_wall - pending_wall
    if remaining <= 0:
        raise ValueError("cumulative wall allocation is exhausted")
    if physics + recovered_physics + pending_physics >= maximum_physics:
        raise ValueError("cumulative physics allocation is exhausted")
    if samples >= sample_target:
        raise ValueError("usable sample target is already reached")
    return LaunchPlan(child, manifest_path, out, manifest_sha, run_id, maximum_wall, previous_wall,
                      pending_wall, remaining, recovered_physics + pending_physics,
                      None if pending is None else pending["reservation_id"])


def kill_child_tree(process: subprocess.Popen) -> None:
    """Target only the directly launched child and its own descendants."""
    if process.poll() is not None:
        return
    if os.name == "nt":
        result = subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                                capture_output=True, text=True, timeout=15,
                                creationflags=subprocess.CREATE_NO_WINDOW)
        if result.returncode and process.poll() is None:
            raise RuntimeError(f"could not terminate child tree {process.pid}: {result.stderr.strip()}")
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait(timeout=15)


def write_result(out: Path, result: dict[str, Any]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    target = out / "watchdog-result.json"
    temporary = out / f".watchdog-result-{uuid.uuid4().hex}.pending"
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(result, stream, indent=1, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary.is_file():
            temporary.unlink()


def supervise(plan: LaunchPlan, *, popen=None, clock=None, sleep=None, terminate=None) -> int:
    popen = subprocess.Popen if popen is None else popen
    clock = time.monotonic if clock is None else clock
    sleep = time.sleep if sleep is None else sleep
    terminate = kill_child_tree if terminate is None else terminate
    started = clock()
    deadline = started + plan.remaining_wall_seconds
    kwargs = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW}
              if os.name == "nt" else {"start_new_session": True})
    process = popen(plan.command, **kwargs)
    interrupted = False
    original_handlers = {}

    def request_interruption(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    for signum in (signal.SIGINT, getattr(signal, "SIGTERM", None), getattr(signal, "SIGBREAK", None)):
        if signum is not None:
            original_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_interruption)
    reason = "child_exited"
    status = None
    error = None
    active_pending = None
    stopped_at = None
    try:
        while True:
            status = process.poll()
            if status is not None:
                break
            now = clock()
            if interrupted:
                reason, status = "watchdog_interrupted", 130
                break
            if now >= deadline:
                reason, status = "cumulative_wall_deadline", 124
                break
            # Read the ledger before the state pointer: a completed newer
            # pointer covers an old ledger even if its unlink was interrupted.
            pending = read_object(plan.out / "pending-work.json", optional=True)
            state = read_object(plan.out / "train-state.json", optional=True)
            active_pending = None
            work_deadline = deadline
            if pending is not None:
                pending = resolve_pending(pending, state)
                if (pending is not None and pending["owner_pid"] == process.pid
                        and pending["reservation_id"] != plan.previous_pending_id):
                    active_pending = pending
                    began = float(pending["started_monotonic_seconds"])
                    if began > now + 1.0:
                        raise ValueError("child pending-work start is ahead of the monotonic clock")
                    work_deadline = began + float(pending["wall_upper_bound"])
                    if now >= work_deadline:
                        reason, status = "pending_work_deadline", 124
                        break
            sleep(max(0.0, min(1.0, deadline - now, work_deadline - now)))
    except Exception as exception:
        reason, status, error = "watchdog_error", 125, str(exception)
    finally:
        stopped_at = clock()
        try:
            if process.poll() is None:
                terminate(process)
        except Exception as exception:
            reason, status, error = "child_termination_failed", 125, str(exception)
        for signum, handler in original_handlers.items():
            signal.signal(signum, handler)
        # No file is written by the supervisor before the runner's fresh-output
        # check. Committed checkpoints and pending ledgers are never rewritten.
        write_result(plan.out, {
            "schema": "play01_watchdog_result_v1", "run_id": plan.run_id,
            "manifest_path": str(plan.manifest_path), "manifest_sha256": plan.manifest_sha256,
            "child_pid": process.pid, "child_command": plan.command,
            "reason": reason, "exit_code": status, "child_exit_code": process.poll(), "error": error,
            "maximum_cumulative_wall_seconds": plan.maximum_wall_seconds,
            "prior_committed_wall_seconds": plan.previous_wall_seconds,
            "prior_pending_wall_seconds_reserved": plan.reserved_unknown_wall_seconds,
            "remaining_wall_seconds_at_launch": plan.remaining_wall_seconds,
            "execution_elapsed_seconds": stopped_at - started,
            "termination_and_reporting_elapsed_seconds": clock() - stopped_at,
            "interrupted_work": active_pending,
            "pending_ledger_preserved": (plan.out / "pending-work.json").is_file(),
            "qualification_holdouts_opened_by_watchdog": False,
        })
    return int(status)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    try:
        plan = prepare_launch(args.manifest, args.out, args.command)
        print(json.dumps({"kind": "watchdog_launch", "run_id": plan.run_id,
                          "remaining_wall_seconds": plan.remaining_wall_seconds,
                          "out": str(plan.out)}), flush=True)
        return supervise(plan)
    except (ValueError, OSError, json.JSONDecodeError) as error:
        print(f"PLAY-01 watchdog refused launch: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
