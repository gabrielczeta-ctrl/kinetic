"""Python client for ``ragdollcore-walksearch --play01-worker``.

Framing: little-endian uint32 tag, uint32 byte length, payload.
tag 1 = UTF-8 JSON (worker -> client), tag 2 = float32 array (both ways).
See apps/ragdollcore_walksearch/play01worker.cpp for the message layouts.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import threading
import queue
from typing import Any

import numpy as np

TAG_JSON = 1
TAG_FLOATS = 2
MAX_FRAME_BYTES = 16 * 1024 * 1024


def worker_creation_flags() -> int:
    """Keep native workers outside the parent's Windows Ctrl+C group."""
    if os.name != "nt":
        return 0
    return int(subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW)


class WorkerError(RuntimeError):
    pass


class Worker:
    def __init__(self, executable: Path, context: Path, profile: Path, floor: str = "tiled",
                 stderr_path: Path | None = None, env: dict[str, str] | None = None,
                 frame_timeout_seconds: float = 30.0, handshake_timeout_seconds: float = 30.0):
        for name, value in (("frame_timeout_seconds", frame_timeout_seconds),
                            ("handshake_timeout_seconds", handshake_timeout_seconds)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.executable = Path(executable)
        self.frame_timeout_seconds = float(frame_timeout_seconds)
        self.handshake_timeout_seconds = float(handshake_timeout_seconds)
        self._handshake_complete = False
        self._closed = False
        self._frames: queue.Queue[tuple[int, bytes] | WorkerError] = queue.Queue()
        self.stderr_file = open(stderr_path, "w", encoding="utf-8") if stderr_path else subprocess.DEVNULL
        command = [str(self.executable), "--play01-worker", str(context), str(profile), "--floor", floor]
        self.command = command
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr_file,
                                            bufsize=0, env=env, creationflags=worker_creation_flags())
        except BaseException:
            if self.stderr_file is not subprocess.DEVNULL:
                self.stderr_file.close()
            raise
        # One persistent reader per native process: no thread creation per
        # action. Waiting for a complete frame bounds partial-header/payload
        # stalls as well as a worker that emits no response at all.
        self._reader = threading.Thread(target=self._read_frames, name=f"play01-worker-reader-{self.process.pid}", daemon=True)
        self._reader.start()
        try:
            self.manifest = self.read_json()
            if self.manifest.get("kind") != "manifest":
                raise WorkerError(f"unexpected handshake: {self.manifest}")
            self.observation_size = int(self.manifest["observation_size"])
            self.action_count = int(self.manifest["action_count"])
            self.body_count = len(self.manifest["bodies"])
            self.layout: dict[str, tuple[int, int]] = {}
            offset = 0
            for name, count in self.manifest["observation_layout"]:
                self.layout[name] = (offset, offset + int(count))
                offset += int(count)
            self.episodes = 0
            self.physics_steps = 0
            self.remaining_actions = 0
            self._handshake_complete = True
        except BaseException:
            self._close(force=True)
            raise

    # ------------------------------------------------------------ framing

    def _read_exact(self, size: int) -> bytes:
        chunks = []
        remaining = size
        while remaining:
            chunk = self.process.stdout.read(remaining)
            if not chunk:
                raise WorkerError("worker closed its output (see its stderr log)")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _read_frames(self) -> None:
        try:
            while not self._closed:
                header = self._read_exact(8)
                tag, length = struct.unpack("<II", header)
                if tag not in (TAG_JSON, TAG_FLOATS):
                    raise WorkerError(f"unsupported frame tag {tag}")
                if length > MAX_FRAME_BYTES:
                    raise WorkerError(f"worker frame length {length} exceeds limit {MAX_FRAME_BYTES}")
                self._frames.put((tag, self._read_exact(length)))
        except Exception as error:
            self._frames.put(error if isinstance(error, WorkerError) else WorkerError(f"worker output read failed: {error}"))

    def _read_frame(self) -> tuple[int, bytes]:
        if self._closed:
            raise WorkerError("worker is closed")
        timeout = self.frame_timeout_seconds if self._handshake_complete else self.handshake_timeout_seconds
        try:
            result = self._frames.get(timeout=timeout)
        except queue.Empty:
            phase = "frame" if self._handshake_complete else "handshake"
            self._close(force=True)
            raise WorkerError(f"worker {phase} timed out after {timeout:g} seconds; process terminated (see its stderr log)") from None
        if isinstance(result, WorkerError):
            self._close(force=True)
            raise result
        return result

    def read_json(self) -> dict[str, Any]:
        tag, payload = self._read_frame()
        if tag != TAG_JSON:
            raise WorkerError(f"expected JSON frame, got tag {tag}")
        return json.loads(payload.decode("utf-8"))

    def read_floats(self) -> np.ndarray:
        tag, payload = self._read_frame()
        if tag == TAG_JSON:
            message = json.loads(payload.decode("utf-8"))
            raise WorkerError(f"worker message while expecting floats: {message}")
        if tag != TAG_FLOATS:
            raise WorkerError(f"expected float frame, got tag {tag}")
        return np.frombuffer(payload, dtype="<f4").copy()

    def read_floats_or_json(self) -> tuple[np.ndarray | None, dict[str, Any] | None]:
        tag, payload = self._read_frame()
        if tag == TAG_JSON:
            return None, json.loads(payload.decode("utf-8"))
        return np.frombuffer(payload, dtype="<f4").copy(), None

    def write_floats(self, values: np.ndarray) -> None:
        data = np.ascontiguousarray(values, dtype="<f4").tobytes()
        self.process.stdin.write(struct.pack("<II", TAG_FLOATS, len(data)) + data)
        self.process.stdin.flush()

    # ------------------------------------------------------------ episodes

    @staticmethod
    def episode_header(direct: bool, steps: int, commands: list[tuple[int, float, float, float, float]],
                       impulse: tuple[int, int, float, float, float] | None = None, motors_off_step: int | None = None) -> np.ndarray:
        values = [1.0 if direct else 0.0, float(steps)]
        if impulse is None:
            values += [-1.0, 0.0, 0.0, 0.0, 0.0]
        else:
            values += [float(impulse[0]), float(impulse[1]), float(impulse[2]), float(impulse[3]), float(impulse[4])]
        values.append(-1.0 if motors_off_step is None else float(motors_off_step))
        values.append(float(len(commands)))
        for command in commands:
            values += [float(v) for v in command]
        return np.asarray(values, dtype=np.float32)

    def start_episode(self, header: np.ndarray) -> np.ndarray:
        """Send the episode header and return the step-0 observation.

        The worker emits one observation per physics step plus one final
        observation before the end record, so exactly ``steps`` actions are
        expected; ``send_action`` returns the end record with the last one.
        """
        self.write_floats(header)
        self.episodes += 1
        self.remaining_actions = int(header[1])
        result = self.read_floats_or_json()
        if result[1] is not None:
            raise WorkerError(f"episode rejected: {result[1]}")
        return result[0]

    def send_action(self, values: np.ndarray) -> tuple[np.ndarray | None, dict[str, Any] | None]:
        """Send an action.

        Returns (observation, None) mid-episode, (final_observation, end)
        after the last action, or (None, error) when the worker rejected the
        action before its physics solve.
        """
        if self.remaining_actions <= 0:
            raise WorkerError("no action expected: the episode has ended")
        self.write_floats(values)
        observation, message = self.read_floats_or_json()
        if message is not None:
            self.remaining_actions = 0
            return None, message
        self.physics_steps += 1
        self.remaining_actions -= 1
        if self.remaining_actions == 0:
            end = self.read_json()
            return observation, end
        return observation, None

    def block(self, observation: np.ndarray, name: str) -> np.ndarray:
        lo, hi = self.layout[name]
        return observation[lo:hi]

    def close(self) -> None:
        self._close(force=False)

    def _close(self, force: bool) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self.process.poll() is None:
                if force:
                    self.process.kill()
                else:
                    self.process.stdin.close()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=5)
        finally:
            # Terminating the native process releases the reader's pipe read.
            self._reader.join(timeout=1)
            for pipe in (self.process.stdin, self.process.stdout):
                if pipe is not None:
                    pipe.close()
            if self.stderr_file is not subprocess.DEVNULL:
                self.stderr_file.close()

    def identity(self) -> dict[str, Any]:
        return {"executable": str(self.executable), "executable_sha256": sha256(self.executable), "command": self.command,
                "protocol": self.manifest["protocol"], "profile_id": self.manifest["profile_id"],
                "creation_profile_hash": self.manifest["creation_profile_hash"], "floor": self.manifest["floor"],
                "mesh_grid_env": self.manifest["mesh_grid_env"], "manifold_reduction_env": self.manifest["manifold_reduction_env"],
                "io_deadlines_seconds": {"handshake": self.handshake_timeout_seconds, "frame": self.frame_timeout_seconds}}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()
