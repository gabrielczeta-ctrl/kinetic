"""Durable update-boundary checkpoints and conservative in-flight budget records.

Only ``train-state.json`` selects the committed checkpoint. New checkpoint
files never overwrite an older generation. A pending work record survives
interruption until its reservation is covered by a newer committed state.
"""

from __future__ import annotations

import copy
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
from typing import Any, Callable
import uuid


STATE_FILE = "train-state.json"
PENDING_FILE = "pending-work.json"
LOCK_FILE = ".run-lock"
_CHECKPOINT_NAME = re.compile(r"checkpoint-state-([1-9][0-9]*)\.pt(?:\.pending)?\Z")


def _directory(out: Path) -> Path:
    directory = Path(out).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    return directory


@contextmanager
def exclusive_run(out: Path):
    """Hold an OS-released-on-exit mutex for one output directory.

    The small lock file remains after release. Unlinking a lock inode can let
    an existing waiter and a new opener acquire different files simultaneously.
    Its recorded PID is informational; the OS lock determines ownership.
    Fresh-directory checks should ignore only this exact file.
    """
    directory = _directory(out)
    path = directory / LOCK_FILE
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    acquired = False
    try:
        if os.fstat(descriptor).st_size == 0:
            os.write(descriptor, b"\0")
        os.lseek(descriptor, 0, os.SEEK_SET)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise RuntimeError(f"training output already has an active owner: {directory}") from error
        acquired = True
        owner = {"pid": os.getpid(), "lock_path": str(path)}
        os.lseek(descriptor, 1, os.SEEK_SET)
        os.write(descriptor, json.dumps(owner).encode("utf-8"))
        os.ftruncate(descriptor, os.lseek(descriptor, 0, os.SEEK_CUR))
        os.fsync(descriptor)
        yield owner
    finally:
        if acquired:
            os.lseek(descriptor, 0, os.SEEK_SET)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(descriptor, msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain an object")
    return value


def _generation(state: dict[str, Any] | None) -> int:
    value = 0 if state is None else state.get("checkpoint_generation", 0)
    if type(value) is not int or value < 0:
        raise ValueError("checkpoint_generation must be a nonnegative integer")
    return value


def _sync_directory(directory: Path) -> None:
    # Windows does not support opening directories this way. File fsync and
    # atomic replace are still used there for both the checkpoint and pointer.
    if os.name == "nt":
        return
    descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.pending")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, indent=1, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _sync_directory(path.parent)
    finally:
        if temporary.is_file():
            temporary.unlink()


def next_generation(out: Path) -> int:
    """Advance past committed generations and orphaned interrupted writes."""
    directory = _directory(out)
    maximum = _generation(_read_json(directory / STATE_FILE))
    for path in directory.iterdir():
        match = _CHECKPOINT_NAME.fullmatch(path.name)
        if match:
            maximum = max(maximum, int(match.group(1)))
    return maximum + 1


def begin_work(out: Path, physics_upper_bound: int, wall_upper_bound: float) -> dict[str, Any]:
    """Durably reserve one batch/evaluation before starting it.

    The caller must commit all completed work before beginning another unit.
    Bounds represent charged uncertainty after interruption, never measured
    simulated steps or measured elapsed time.
    """
    if type(physics_upper_bound) is not int or physics_upper_bound < 0:
        raise ValueError("physics_upper_bound must be a nonnegative integer")
    if not math.isfinite(wall_upper_bound) or wall_upper_bound <= 0:
        raise ValueError("wall_upper_bound must be finite and positive")
    directory = _directory(out)
    if recover_pending(directory) is not None:
        raise RuntimeError("pending work must be accounted and committed before new simulation")
    state = _read_json(directory / STATE_FILE)
    record = {
        "schema": "play01_pending_work_v1",
        "reservation_id": uuid.uuid4().hex,
        "owner_pid": os.getpid(),
        "started_monotonic_seconds": time.monotonic(),
        "base_checkpoint_generation": _generation(state),
        "physics_upper_bound": physics_upper_bound,
        "wall_upper_bound": float(wall_upper_bound),
    }
    _atomic_json(directory / PENDING_FILE, record)
    return record


def recover_pending(out: Path) -> dict[str, Any] | None:
    """Return unresolved work without deleting its durable reservation.

    A newer state pointer already covers a reservation even if interruption
    happened immediately before removing its ledger file.
    """
    directory = _directory(out)
    pending = _read_json(directory / PENDING_FILE)
    if pending is None:
        return None
    if pending.get("schema") != "play01_pending_work_v1":
        raise ValueError("unrecognized pending-work schema")
    base = pending.get("base_checkpoint_generation")
    physics = pending.get("physics_upper_bound")
    wall = pending.get("wall_upper_bound")
    if (type(base) is not int or base < 0 or type(physics) is not int or physics < 0
            or not isinstance(wall, (int, float)) or isinstance(wall, bool)
            or not math.isfinite(wall) or wall <= 0 or not pending.get("reservation_id")):
        raise ValueError("invalid pending-work reservation")
    current = _generation(_read_json(directory / STATE_FILE))
    if current < base:
        raise ValueError("pending-work base is newer than the committed state")
    return None if current > base else pending


def commit_generation(out: Path, generation: int,
                      save_callback: Callable[[Path], dict[str, Any]],
                      state: dict[str, Any]) -> dict[str, Any]:
    """Save a new checkpoint then atomically select it; retain two generations.

    ``save_callback(path)`` must close its written file and return checkpoint
    metadata including ``path`` and ``sha256``. Other metadata is preserved.
    The returned state includes the final checkpoint path, so callers must use
    that path for restore rather than a mutable ``checkpoint-latest.pt`` alias.
    """
    directory = _directory(out)
    previous = _read_json(directory / STATE_FILE)
    if type(generation) is not int or generation <= _generation(previous):
        raise ValueError("checkpoint generation must increase")
    checkpoint = directory / f"checkpoint-state-{generation}.pt"
    temporary = checkpoint.with_name(checkpoint.name + ".pending")
    if checkpoint.exists() or temporary.exists():
        raise FileExistsError("checkpoint generation already exists; use next_generation")
    # A failed callback leaves an identifiable orphan, never replacing the
    # checkpoint referenced by the current committed state.
    saved = dict(save_callback(temporary))
    if not temporary.is_file() or temporary.is_symlink():
        raise ValueError("checkpoint callback did not create a regular local file")
    if Path(saved.get("path", "")).resolve() != temporary:
        raise ValueError("checkpoint callback metadata path mismatch")
    digest = hashlib.sha256()
    with temporary.open("r+b") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
        os.fsync(stream.fileno())
    if saved.get("sha256") != digest.hexdigest():
        raise ValueError("checkpoint callback metadata hash mismatch")
    os.replace(temporary, checkpoint)
    _sync_directory(directory)
    saved["path"] = str(checkpoint)
    old_generations = [] if previous is None else previous.get("committed_checkpoint_generations", [_generation(previous)])
    if (not isinstance(old_generations, list)
            or any(type(value) is not int or value < 1 or value >= generation for value in old_generations)):
        raise ValueError("invalid committed checkpoint history")
    retained = sorted(set(old_generations + [generation]))[-2:]
    committed = copy.deepcopy(state)
    committed.update(checkpoint_generation=generation, committed_checkpoint_generations=retained,
                     latest_checkpoint=saved)
    _atomic_json(directory / STATE_FILE, committed)
    # No deletion occurs until the new pointer is durably written. Every
    # cleanup target is a fixed child filename selected from committed history.
    pending = directory / PENDING_FILE
    if pending.is_file():
        pending.unlink()
    for obsolete in set(old_generations) - set(retained):
        old_path = directory / f"checkpoint-state-{obsolete}.pt"
        if old_path.is_file() and not old_path.is_symlink():
            old_path.unlink()
    _sync_directory(directory)
    return committed
