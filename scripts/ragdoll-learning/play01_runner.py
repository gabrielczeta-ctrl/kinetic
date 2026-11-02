#!/usr/bin/env python3
"""PLAY-01 staged runner: readiness controls, integration smoke, resumable training.

Implements specification PLAY-01-reference-conditioned-motor-residual-v5
(scripts/ragdoll-learning/pilot_spec.json) on the captured game body through
``ragdollcore-walksearch --play01-worker``. Nothing here writes a tracker
verdict; every artifact is a hashed diagnostic record.

Subcommands
  readiness  pre-update controls (identity, zero-action, authority, replay,
             motor-off, malformed rejection, physical authority, coverage)
  smoke      readiness + bounded optimizer updates + throughput + checkpoint/
             resume control + baselines on the selection controls, under the
             specification's 1,000,000-step / 1200-second integration cap
  preflight  validate an allocated manifest/checkpoint without starting physics
  train      resumable allocated training from a training_run_manifest
  propose    size a training allocation from measured smoke throughput
"""

from __future__ import annotations

import argparse
import atexit
import concurrent.futures
import copy
import datetime as dt
import gzip
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import signal
import shutil
import subprocess
import sys
import time
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reference_motion as motion  # noqa: E402
from play01_worker_client import Worker, WorkerError, sha256  # noqa: E402
from reference_generator import ReferenceGenerator, GENERATOR_VERSION  # noqa: E402

RUNNER_VERSION = "play01-runner-v22"
TORSO_LANDMARK = "torso"
TORSO_BODY_NAME = "spine2"


_PARALLEL_RUNNER = None
_PARALLEL_POLICY = None


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256_json(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=1, allow_nan=False, default=str)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def physical_selection_key(summary: dict[str, Any]) -> tuple:
    """Select survival first; do not trade a fall for several minor violations."""
    violations = summary["violation_counts"]
    return (summary["falls"], violations.get("nonfoot_support", 0), sum(violations.values()),
            summary["mean_commanded_velocity_rmse_walk"], summary["mean_joint_acceleration_rms"],
            summary["mean_support_slip_sum"])


def evaluation_guard(history: list[dict[str, Any]], baseline: dict[str, Any],
                     rules: dict[str, Any], usable: int) -> str | None:
    if not rules or usable < int(rules["minimum_usable_transitions"]):
        return None
    window = int(rules["consecutive_evaluations"])
    if len(history) < window:
        return None
    recent = history[-window:]
    if all(row["summary"]["falls"] > baseline["falls"] + int(rules["additional_falls_limit"])
           or row["summary"]["mean_achieved_along_command_speed_walk"] <= 0 for row in recent):
        return "persistent physical regression on selection evaluations"
    if usable >= int(rules["plateau_minimum_usable_transitions"]):
        improved = any(row["summary"]["falls"] <= baseline["falls"]
                       and row["summary"]["mean_commanded_velocity_rmse_walk"] <=
                       baseline["mean_commanded_velocity_rmse_walk"] * (1 - float(rules["minimum_relative_rmse_improvement"]))
                       and all(row["summary"]["violation_counts"].get(k, 0) <= value
                               for k, value in baseline["violation_counts"].items())
                       for row in history)
        if not improved:
            return "minimum exposure reached without baseline-relative physical improvement"
    return None


def gae_returns(rewards: np.ndarray, values: np.ndarray, continuation: np.ndarray,
                gamma: float, lam: float) -> tuple[np.ndarray, np.ndarray]:
    """GAE with the continuation flag attached to the current transition."""
    rewards = np.asarray(rewards, dtype=np.float32)
    values = np.asarray(values, dtype=np.float32)
    continuation = np.asarray(continuation, dtype=np.float32)
    if rewards.shape != values.shape or rewards.shape != continuation.shape:
        raise ValueError("GAE inputs must have identical shapes")
    advantages = np.zeros_like(rewards)
    carried = 0.0
    for index in reversed(range(len(rewards))):
        next_value = values[index + 1] if index + 1 < len(values) else 0.0
        keep_going = continuation[index]
        delta = rewards[index] + gamma * next_value * keep_going - values[index]
        carried = delta + gamma * lam * keep_going * carried
        advantages[index] = carried
    return advantages, advantages + values


def terminal_reward_with_lost_command_opportunity(step: int, command_stop_step: int,
                                                  discount: float, base_penalty: float = -3.0,
                                                  command_reward_weight: float = 0.55) -> float:
    """Charge only the bounded discounted command reward lost after a failure."""
    remaining = max(0, command_stop_step - step - 1)
    if remaining == 0:
        return base_penalty
    if not 0.0 < discount <= 1.0:
        raise ValueError("discount must be in (0, 1]")
    discounted_steps = (float(remaining) if discount == 1.0
                        else (1.0 - discount ** remaining) / (1.0 - discount))
    return base_penalty - command_reward_weight * discounted_steps


def policy_controls_step(step: int, warmup_steps: int) -> bool:
    """The learned residual must not alter state during excluded warmup."""
    return step >= warmup_steps


def should_end_rollout(purpose: str, stop_on_terminal: bool, terminal: bool, worker_end_received: bool) -> bool:
    """Only training may end early; evaluation/diagnostics consume the declared trajectory."""
    return purpose == "training" and stop_on_terminal and terminal and not worker_end_received


def is_physical_early_end(end: dict[str, Any] | None) -> bool:
    """Whether an observation-less worker END carries terminal physical evidence."""
    return bool(end and end.get("kind") == "end"
                and (end.get("fall_step") is not None or int(end.get("nonfoot_support_frames", 0)) > 0))


def validate_checkpoint_state(state: dict[str, Any], fixture: "Fixture", mode: str,
                              expected_contract: dict[str, Any] | None = None,
                              allow_repair_transition: bool = False) -> None:
    """Validate a deserialized checkpoint without mutating policy or RNG state."""
    if mode not in ("transfer", "exact-resume"):
        raise ValueError("checkpoint validation mode must be transfer or exact-resume")
    for key in ("identity", "policy", "optimizer", "torch_generator", "torch_rng", "numpy_rngs", "counters", "normalization", "generator_identity", "extra"):
        if key not in state:
            raise RuntimeError(f"checkpoint lacks required field {key}")
    common = ("executable_sha256", "context_sha256", "profile_sha256", "library_sha256",
              "generator_source_sha256", "spec_sha256", "generator_version")
    mismatched = [key for key in common if state["identity"].get(key) != fixture.identity.get(key)]
    if mode == "exact-resume" and not allow_repair_transition:
        if state["identity"].get("runner_version") != fixture.identity["runner_version"]:
            mismatched.append("runner_version")
        if state["identity"].get("git", {}).get("commit") != fixture.identity["git"]["commit"]:
            mismatched.append("source_commit")
    if state.get("generator_identity") != fixture.make_generator().identity():
        mismatched.append("generator_identity")
    if mismatched:
        raise RuntimeError(f"checkpoint {mode} identity mismatch: " + ", ".join(mismatched))
    if state["identity"].get("control_contract") != fixture.identity.get("control_contract"):
        raise RuntimeError(f"checkpoint {mode} control contract mismatch")
    if expected_contract is not None and state.get("extra", {}).get("run_contract") != expected_contract:
        raise RuntimeError("checkpoint run/case contract mismatch")
    if mode == "exact-resume":
        if state.get("normalization") != {"running_normalizer": None, "fixed_scales": True}:
            raise RuntimeError("checkpoint normalization contract mismatch")
        required_counters = set(Runner.counter_snapshot_fields()) | {"cumulative_wall_seconds"}
        missing = sorted(required_counters - set(state["counters"]))
        if missing:
            raise RuntimeError("checkpoint lacks cumulative counters: " + ", ".join(missing))


def validate_repair_migration(migration: dict[str, Any], state: dict[str, Any], fixture: "Fixture",
                              checkpoint_sha256: str, previous_manifest_sha256: str) -> None:
    """Authorize a narrow exact-state restore across a recorded runner repair."""
    expected = {
        "from_source_commit": state["identity"].get("git", {}).get("commit"),
        "from_runner_version": state["identity"].get("runner_version"),
        "from_checkpoint_sha256": checkpoint_sha256,
        "from_manifest_sha256": previous_manifest_sha256,
        "to_source_commit": fixture.identity["git"]["commit"],
        "to_runner_version": fixture.identity["runner_version"],
        "treatment": "restore_policy_optimizer_rng_normalization_counters_and_schedule",
    }
    missing = [key for key, value in expected.items() if migration.get(key) != value]
    if missing:
        raise RuntimeError("exact-resume repair migration mismatch: " + ", ".join(missing))
    if not migration.get("reason") or not migration.get("authorization_reference"):
        raise RuntimeError("exact-resume repair migration lacks reason or authorization")
    if expected["from_source_commit"] == expected["to_source_commit"]:
        raise RuntimeError("exact-resume repair migration does not cross a source revision")


def episode_seed(run_seed: int, case: dict[str, Any], label: str) -> int:
    """A schedule-independent policy seed for one complete episode."""
    case_id = str(case.get("id", case_hash(case)))
    digest = hashlib.sha256(f"{run_seed}|{case_id}|{label}".encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "little") & ((1 << 63) - 1)


def policy_to_numpy(policy) -> dict[str, np.ndarray]:
    return {name: value.detach().cpu().numpy().copy() for name, value in policy.state_dict().items()}


def _parallel_close() -> None:
    global _PARALLEL_RUNNER
    if _PARALLEL_RUNNER is not None:
        _PARALLEL_RUNNER.close()
        _PARALLEL_RUNNER = None


def _parallel_initialize(fixture_args: dict[str, Any], output_root: str) -> None:
    """Create one persistent physics worker and episode engine per child process."""
    global _PARALLEL_RUNNER, _PARALLEL_POLICY
    import torch
    # On Windows, Ctrl+C is delivered to the whole console process group. The
    # parent owns graceful-stop/checkpoint policy; children must finish their
    # assigned episode group so its physics and RNG state can be accounted.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    torch.set_num_threads(1)
    fixture = Fixture(argparse.Namespace(**fixture_args))
    child_out = Path(output_root) / f"process-{os.getpid()}"
    _PARALLEL_RUNNER = Runner(fixture, child_out)
    _PARALLEL_RUNNER.ensure_workers(1)
    _PARALLEL_POLICY = build_policy(_PARALLEL_RUNNER.builder.dimension, _PARALLEL_RUNNER.builder.action_count, fixture.spec)
    atexit.register(_parallel_close)


def _parallel_episode_group(policy_state: dict[str, np.ndarray] | None, jobs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Run a fixed job group serially inside one persistent child process."""
    global _PARALLEL_RUNNER, _PARALLEL_POLICY
    import torch
    if _PARALLEL_RUNNER is None:
        raise RuntimeError("parallel rollout process was not initialized")
    policy = None
    if policy_state is not None:
        _PARALLEL_POLICY.load_state_dict({name: torch.from_numpy(value) for name, value in policy_state.items()})
        policy = _PARALLEL_POLICY
    results = []
    for job in jobs:
        generator = torch.Generator().manual_seed(int(job["episode_seed"]))
        result = _PARALLEL_RUNNER.run_episode(
            _PARALLEL_RUNNER.workers[0], job["controller"], job["case"], policy=policy,
            stochastic=job["stochastic"], replay=job.get("replay"),
            journal=job["journal"], label=job["label"],
             steps=job.get("steps"), torch_generator=generator, purpose=job["purpose"],
             stop_on_terminal=job["stop_on_terminal"],
             collect_teacher=bool(job.get("collect_teacher", False)),
             capture_visual=bool(job.get("capture_visual", False)))
        if result["requires_worker_reset"]:
            reset_started = time.monotonic()
            _PARALLEL_RUNNER.replace_worker(0)
            reset_seconds = time.monotonic() - reset_started
            result["accounting"]["worker_resets"] = 1
            result["accounting"]["worker_reset_seconds"] = reset_seconds
            _PARALLEL_RUNNER.worker_resets += 1
            _PARALLEL_RUNNER.worker_reset_seconds += reset_seconds
        result["parallel_job_index"] = job["index"]
        if job["compact"]:
            for key in ("journal", "terms", "offsets", "delivered", "rotations", "utilization"):
                result.pop(key, None)
        results.append(result)
    return results


def git_identity(source_root: Path) -> dict[str, Any]:
    def run(*args: str) -> str:
        return subprocess.run(["git", *args], cwd=source_root, text=True, capture_output=True, check=True).stdout.strip()
    return {"source_root": str(source_root), "commit": run("rev-parse", "HEAD"), "branch": run("branch", "--show-current"),
            "dirty_files": [line for line in run("status", "--porcelain").splitlines() if line.strip()]}


# ------------------------------------------------------------------ math


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    return motion.rotate(q, v)


def quat_conj(q: np.ndarray) -> np.ndarray:
    return motion.conjugate(q)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return motion.multiply(a, b)


def yaw_of_forward(forward_xy: np.ndarray) -> float:
    return math.atan2(-forward_xy[0], forward_xy[1])


def rotate_xy(v: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    out = np.array(v, dtype=float)
    x, y = out[..., 0].copy(), out[..., 1].copy()
    out[..., 0] = c * x - s * y
    out[..., 1] = s * x + c * y
    return out


def six_d(q: np.ndarray) -> np.ndarray:
    """First two columns of the rotation matrix for (..., 4) xyzw quaternions."""
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    col0 = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)], axis=-1)
    col1 = np.stack([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)], axis=-1)
    return np.concatenate([col0, col1], axis=-1)


def project_residual_action(action: np.ndarray, layout: list[dict[str, Any]]) -> np.ndarray:
    """Project every residual joint just inside the worker's closed unit bound.

    The worker converts a unit-radius action to a maximum-angle float vector.
    Keeping a small float32 margin prevents that multiplication and the engine's
    subsequent squared-length check from disagreeing at the exact boundary.
    """
    projected = np.asarray(action, dtype=np.float32).copy()
    inward_radius = np.float32(1.0 - 16.0 * np.finfo(np.float32).eps)
    for joint in layout:
        first = int(joint["first_scalar"])
        dimensions = int(joint["dimensions"])
        values = projected[first:first + dimensions]
        length = float(np.linalg.norm(values))
        if length > float(inward_radius):
            values *= inward_radius / np.float32(length)
    return projected


def teacher_action_from_local_targets(target_local: np.ndarray, reference_local: np.ndarray,
                                      layout: list[dict[str, Any]], maximum_offset: float = 0.5
                                      ) -> tuple[np.ndarray, dict[str, Any]]:
    """Invert a driven parent-local pose into the existing residual action basis."""
    target_local = np.asarray(target_local, dtype=float)
    reference_local = np.asarray(reference_local, dtype=float)
    if target_local.shape != reference_local.shape or target_local.ndim != 2 or target_local.shape[1] != 4:
        raise ValueError("teacher/reference local rotations must have matching Bx4 shapes")
    requested = np.zeros(sum(int(joint["dimensions"]) for joint in layout), dtype=np.float32)
    requested_norms = []
    off_axis_errors = []
    for joint in layout:
        body = int(joint["body"])
        first = int(joint["first_scalar"])
        dimensions = int(joint["dimensions"])
        delta = motion.rotation_vector(quat_mul(target_local[body], quat_conj(reference_local[body])))
        if dimensions == 3:
            requested[first:first + 3] = delta / maximum_offset
            requested_norms.append(float(np.linalg.norm(requested[first:first + 3])))
            off_axis_errors.append(0.0)
        elif dimensions == 1:
            axis = np.asarray(joint["hinge_axis_parent_local"], dtype=float)
            axis /= max(float(np.linalg.norm(axis)), 1e-12)
            along = float(delta @ axis)
            requested[first] = along / maximum_offset
            requested_norms.append(abs(float(requested[first])))
            off_axis_errors.append(float(np.linalg.norm(delta - along * axis)))
        else:
            raise ValueError("teacher action layout has unsupported joint dimension")
    projected = project_residual_action(np.clip(requested, -1.0, 1.0), layout)
    clipped = [value > 1.0 + 1e-6 for value in requested_norms]
    return projected, {
        "joint_count": len(layout), "clipped_joint_count": int(sum(clipped)),
        "clipped_joint_fraction": float(np.mean(clipped)) if clipped else 0.0,
        "maximum_requested_joint_norm": max(requested_norms, default=0.0),
        "maximum_hinge_off_axis_error_radians": max(off_axis_errors, default=0.0),
    }


def teacher_filter_proposal(desired_applied: np.ndarray, previous_applied: np.ndarray,
                            layout: list[dict[str, Any]], previous_weight: float = 0.8
                            ) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Invert the frozen residual low-pass filter and report unreachable change."""
    proposed_weight = 1.0 - previous_weight
    if not 0.0 < proposed_weight <= 1.0:
        raise ValueError("teacher filter weights are invalid")
    desired = np.asarray(desired_applied, dtype=np.float32)
    previous = np.asarray(previous_applied, dtype=np.float32)
    raw = (desired - previous_weight * previous) / proposed_weight
    proposal = project_residual_action(np.clip(raw, -1.0, 1.0), layout)
    delivered = previous_weight * previous + proposed_weight * proposal
    error = delivered - desired
    return proposal, delivered.astype(np.float32), {
        "maximum_delivery_error": float(np.max(np.abs(error))),
        "rms_delivery_error": float(np.sqrt(np.mean(error ** 2))),
        "proposal_clipped_fraction": float(np.mean(np.abs(raw) > 1.0 + 1e-6)),
    }


# ----------------------------------------------------------------- config


class Fixture:
    """Paths, hashes and derived constants shared by every stage."""

    def __init__(self, args: argparse.Namespace):
        self.source_root = Path(args.source_root).resolve()
        self.exe = Path(args.exe).resolve()
        self.context = (self.source_root / "files/data/ragdoll/calibration/context-symP.creation.txt").resolve()
        self.profile = (self.source_root / "files/data/ragdoll/humanoid-v1.yaml").resolve()
        self.spec_path = (self.source_root / "scripts/ragdoll-learning/pilot_spec.json").resolve()
        self.library_path = Path(args.library).resolve()
        self.floor = args.floor
        self.workers = int(args.workers)
        self.control_mode = getattr(args, "control_mode", "direct")
        if self.control_mode not in ("direct", "production-residual"):
            raise ValueError("unknown learned controller mode")
        self.policy_controller = self.control_mode
        self.spec = json.loads(self.spec_path.read_text(encoding="utf-8"))
        self.library = json.loads(self.library_path.read_text(encoding="utf-8"))
        if self.library.get("sha256") != sha256_json({k: v for k, v in self.library.items() if k != "sha256"}):
            raise ValueError("motion library content hash mismatch")
        maximum_workers = int(self.spec["optimizer"]["maximum_persistent_cpu_workers"])
        if self.workers < 1 or self.workers > maximum_workers:
            raise ValueError(f"workers must be between 1 and the specification cap of {maximum_workers}")
        vanilla = self.library.get("measured_vanilla_forward_speed") or {}
        self.bound_speed = float(args.bound_speed if args.bound_speed is not None else vanilla.get("speed_units_per_second"))
        self.command_speed = float(args.command_speed if args.command_speed is not None else self.bound_speed)
        protocol = self.spec["reference_dataset"]["command_protocol"]
        self.episode_steps = int(protocol["episode_physics_steps"])
        self.warmup_steps = int(protocol["warmup_steps"])
        self.start_blend_steps = int(protocol["start_blend_steps"])
        self.walk_steps = int(protocol["walk_steps"])
        self.stop_blend_steps = int(protocol["stop_blend_steps"])
        self.dt = 1.0 / 60.0
        self.identity = {
            "runner_version": RUNNER_VERSION, "generator_version": GENERATOR_VERSION,
            "specification_id": self.spec["specification_id"], "spec_sha256": sha256(self.spec_path),
            "executable": str(self.exe), "executable_sha256": sha256(self.exe),
            "context_sha256": sha256(self.context), "profile_sha256": sha256(self.profile),
            "library_path": str(self.library_path), "library_sha256": self.library["sha256"],
            "generator_source_sha256": sha256(HERE / "reference_generator.py"),
            "bound_speed": self.bound_speed, "command_speed": self.command_speed, "floor": self.floor,
            "git": git_identity(self.source_root), "python": platform.python_version(), "platform": platform.platform(),
        }
        self.identity["control_mode"] = self.control_mode
        self.identity["implementation_sha256"] = sha256_json({
            name: sha256(HERE / name) for name in (
                "play01_runner.py", "play01_worker_client.py", "play01_run_guard.py", "reference_generator.py", "pilot_spec.json")})
        self.identity["control_contract"] = {
            "controller": self.control_mode,
            "maximum_rotation_offset_radians": 0.1 if self.control_mode == "production-residual" else 0.5,
            "production_nonpose_authority": self.control_mode == "production-residual",
            "filter": self.spec["actions"]["filter"],
        }
        try:
            import torch
            self.identity["torch"] = torch.__version__
        except ImportError:
            self.identity["torch"] = None

    def make_generator(self) -> ReferenceGenerator:
        return ReferenceGenerator(self.library, self.bound_speed, dt=self.dt, start_blend_steps=self.start_blend_steps,
                                  stop_blend_steps=self.stop_blend_steps)

    def commands(self, speed: float | None = None) -> list[tuple[int, float, float, float, float]]:
        speed = self.command_speed if speed is None else speed
        start = self.warmup_steps
        stop = self.warmup_steps + self.start_blend_steps + self.walk_steps
        return [(0, 0.0, 0.0, 0.0, 0.0), (start, 0.0, speed, 0.0, 1.0), (stop, 0.0, 0.0, 0.0, 1.0)]


# ----------------------------------------------------------- observation


class ObservationBuilder:
    def __init__(self, fixture: Fixture, manifest: dict[str, Any]):
        self.fixture = fixture
        self.manifest = manifest
        self.bodies = manifest["bodies"]
        self.names = [b["name"] for b in self.bodies]
        self.parents = [int(b["parent"]) for b in self.bodies]
        self.children = np.asarray([i for i, p in enumerate(self.parents) if p >= 0], dtype=int)
        self.parent_of_children = np.asarray([self.parents[i] for i in self.children], dtype=int)
        self.joints = [i for i, b in enumerate(self.bodies) if b["constraint"] and b["constraint"]["maximum_torque"] > 0]
        self.actions = manifest["actions"]
        self.action_count = int(manifest["action_count"])
        self.landmarks = manifest["landmarks"]
        self.pelvis = int(self.landmarks["pelvis"])
        self.ends = {k: int(self.landmarks[k]) for k in ("left_hand", "right_hand", "left_foot", "right_foot")}
        offsets = np.asarray(fixture.library["creation_local_offsets"], dtype=float)
        index = {n: i for i, n in enumerate(self.names)}
        self.length_scale = float(np.mean([np.linalg.norm(offsets[index["left_calf"]]) + np.linalg.norm(offsets[index["left_foot"]]),
                                           np.linalg.norm(offsets[index["right_calf"]]) + np.linalg.norm(offsets[index["right_foot"]])]))
        self.pelvis_local_forward = np.asarray(fixture.library["forward_axis_mapping"]["pelvis_local_forward"], dtype=float)
        bind_pelvis = np.asarray(fixture.library["creation_bind_rotations_xyzw"][self.pelvis], dtype=float)
        self.pelvis_local_up = quat_rotate(quat_conj(bind_pelvis), np.array([0.0, 0.0, 1.0]))
        self.linear_scale = 60.0
        self.angular_scale = 5.0
        layout = {}
        offset = 0
        for name, count in manifest["observation_layout"]:
            layout[name] = (offset, offset + int(count))
            offset += int(count)
        self.layout = layout
        self.dimension = (len(self.bodies) * 3 + len(self.joints) * 6 + len(self.bodies) * 6 + 3 + 1 + 2 + 2 + 4
                          + 2 * (len(self.joints) * 9) + 2 * 12 + self.action_count)
        self.blocks = [
            ("body_position_heading_frame", len(self.bodies) * 3), ("joint_rotation_6d", len(self.joints) * 6),
            ("body_velocity_heading_frame", len(self.bodies) * 6), ("pelvis_up_and_height", 4),
            ("foot_support_flags", 2), ("phase_sin_cos", 2), ("command_and_facing_heading_frame", 4),
            ("reference_snapshot_0", len(self.joints) * 9), ("reference_snapshot_6", len(self.joints) * 9),
            ("reference_end_effectors_0_and_6", 24), ("previous_action", self.action_count),
        ]
        assert sum(c for _, c in self.blocks) == self.dimension

    def block(self, raw: np.ndarray, name: str) -> np.ndarray:
        lo, hi = self.layout[name]
        return raw[lo:hi]

    def physical(self, raw: np.ndarray) -> dict[str, Any]:
        """Decode the worker's raw observation into named arrays."""
        b = len(self.bodies)
        positions = self.block(raw, "body_world_position").reshape(b, 3).astype(float)
        rotations = self.block(raw, "body_world_rotation_xyzw").reshape(b, 4).astype(float)
        rotations = rotations / np.maximum(np.linalg.norm(rotations, axis=-1, keepdims=True), 1e-9)
        linear = self.block(raw, "body_linear_velocity").reshape(b, 3).astype(float)
        angular = self.block(raw, "body_angular_velocity").reshape(b, 3).astype(float)
        forward = quat_rotate(rotations[self.pelvis], self.pelvis_local_forward)
        yaw = yaw_of_forward(forward[:2]) if np.hypot(forward[0], forward[1]) > 1e-6 else 0.0
        local = rotations.copy()
        local_velocity = np.zeros((b, 3))
        children = self.children
        parent_conj = quat_conj(rotations[self.parent_of_children])
        local[children] = quat_mul(parent_conj, rotations[children])
        local_velocity[children] = quat_rotate(parent_conj, angular[children] - angular[self.parent_of_children])
        driven = self.block(raw, "driven_motor_local_rotation_xyzw").reshape(b, 4).astype(float)
        driven_norm = np.linalg.norm(driven, axis=-1, keepdims=True)
        driven_local = (driven / np.maximum(driven_norm, 1e-9)
                        if bool(np.all(driven_norm > 0.5)) else None)
        return {
            "valid": bool(raw[0] > 0.5), "step": int(raw[1]), "positions": positions, "rotations": rotations,
            "linear": linear, "angular": angular, "local": local, "local_velocity": local_velocity, "yaw": yaw,
            "com": self.block(raw, "com_position").astype(float), "com_velocity": self.block(raw, "com_velocity").astype(float),
            "pelvis_up": float(raw[self.layout["pelvis_up_dot"][0]]), "pelvis_height": float(raw[self.layout["pelvis_height"][0]]),
            "mode": int(raw[self.layout["controller_mode"][0]]), "motors_applied": bool(raw[self.layout["motors_applied"][0]] > 0.5),
            "left_supported": bool(raw[self.layout["left_foot_supported"][0]] > 0.5),
            "right_supported": bool(raw[self.layout["right_foot_supported"][0]] > 0.5),
            "nonfoot_support": bool(raw[self.layout["nonfoot_support"][0]] > 0.5),
            "free_cap": bool(raw[self.layout["free_velocity_limit_touched"][0]] > 0.5),
            "solver_cap": bool(raw[self.layout["solver_velocity_limit_touched"][0]] > 0.5),
            "utilization": self.block(raw, "motor_utilization").astype(float),
            "saturated": bool(raw[self.layout["any_motor_saturated"][0]] > 0.5),
            "delivered": self.block(raw, "delivered_motor_target").reshape(b, 6).astype(float),
            "driven_motor_local": driven_local,
            "applied_offsets": self.block(raw, "applied_offset_parent_local").reshape(b, 3).astype(float),
            "net_force": float(raw[self.layout["applied_net_force_magnitude"][0]]),
            "pelvis_force": float(raw[self.layout["applied_pelvis_force_magnitude"][0]]),
            "pelvis_torque": float(raw[self.layout["applied_pelvis_torque_magnitude"][0]]),
            "foot_tangential": (float(raw[self.layout["left_foot_contact_tangential_speed_mean"][0]]),
                                float(raw[self.layout["right_foot_contact_tangential_speed_mean"][0]])),
        }

    def vector(self, phys: dict[str, Any], generator: ReferenceGenerator, reference: dict[str, Any],
               command: np.ndarray, facing: np.ndarray, previous_action: np.ndarray) -> np.ndarray:
        yaw = phys["yaw"]
        rel = phys["positions"] - phys["positions"][self.pelvis]
        rel_h = rotate_xy(rel, -yaw) / self.length_scale
        joints6 = six_d(phys["local"][self.joints]).reshape(-1)
        lin = rotate_xy(phys["linear"], -yaw) / self.linear_scale
        ang = rotate_xy(phys["angular"], -yaw) / self.angular_scale
        up = quat_rotate(phys["rotations"][self.pelvis], self.pelvis_local_up)
        pelvis_block = np.array([up[0], up[1], up[2], phys["pelvis_height"] / self.length_scale])
        feet = np.array([1.0 if phys["left_supported"] else 0.0, 1.0 if phys["right_supported"] else 0.0])
        phase = np.array([math.sin(2 * math.pi * reference["phase"]), math.cos(2 * math.pi * reference["phase"])])
        cmd_h = rotate_xy(np.asarray(command[:2], dtype=float), -yaw) / self.linear_scale
        face = np.asarray(facing[:2], dtype=float)
        if np.linalg.norm(face) < 1e-6:
            face = np.asarray(command[:2], dtype=float)
        if np.linalg.norm(face) < 1e-6:
            face_h = np.array([0.0, 1.0])
        else:
            face_h = rotate_xy(face / np.linalg.norm(face), -yaw)
        snapshots = []
        ends = []
        # Offset 0 is the reference just generated for this step; offset 6 is a
        # pure lookahead of the same clock. Velocities are finite differences
        # of the parent-local rotations (no extra kinematics pass).
        local0 = reference["local_rotations"][self.joints]
        velocity0 = reference["local_angular_velocities"][self.joints]
        snapshots.append(np.concatenate([six_d(local0).reshape(-1), (velocity0 / self.angular_scale).reshape(-1)]))
        for key in ("left_hand", "right_hand", "left_foot", "right_foot"):
            ends.append(np.asarray(reference["root_relative_end_effectors"][key], dtype=float) / self.length_scale)
        snap6 = generator.reference_snapshot(6)
        local5, _ = generator.snapshot_local(5)
        velocity6 = motion.rotation_vector(quat_mul(snap6["local_rotations"], quat_conj(local5)))[self.joints] / self.fixture.dt
        snapshots.append(np.concatenate([six_d(snap6["local_rotations"][self.joints]).reshape(-1), (velocity6 / self.angular_scale).reshape(-1)]))
        for key in ("left_hand", "right_hand", "left_foot", "right_foot"):
            ends.append(np.asarray(snap6["end_effectors"][key], dtype=float) / self.length_scale)
        vector = np.concatenate([rel_h.reshape(-1), joints6, lin.reshape(-1), ang.reshape(-1), pelvis_block, feet, phase,
                                 cmd_h, face_h, snapshots[0], snapshots[1], np.concatenate(ends), previous_action])
        if vector.shape[0] != self.dimension or not np.isfinite(vector).all():
            raise RuntimeError("observation dimension or finiteness violated")
        return vector.astype(np.float32)


# ----------------------------------------------------------------- reward


class RewardModel:
    def __init__(self, builder: ObservationBuilder, fixture: Fixture):
        self.b = builder
        self.dt = fixture.dt

    def terms(self, before: dict[str, Any], after: dict[str, Any], reference: dict[str, Any], command: np.ndarray,
              action: np.ndarray, previous_action: np.ndarray) -> dict[str, float]:
        b = self.b
        joints = b.joints
        confidence = reference.get("style_confidence", {name: 0.0 for name in b.names})
        weights = np.asarray([confidence[b.names[j]] for j in joints], dtype=float)
        total = max(float(weights.sum()), 1e-9)
        style_pose = float(weights.mean())
        ref_local = reference["local_rotations"]
        pose_error = motion.rotation_vector(quat_mul(after["local"][joints], quat_conj(ref_local[joints])))
        pose = float(np.sum(weights * np.sum(pose_error ** 2, axis=-1)) / total)
        vel_error = (after["local_velocity"][joints] - reference["local_angular_velocities"][joints]) / 5.0
        velocity = float(np.sum(weights * np.sum(vel_error ** 2, axis=-1)) / total)
        yaw = after["yaw"]
        ee_weighted = 0.0
        ee_total = 0.0
        ee_confidence = []
        for key in ("left_hand", "right_hand", "left_foot", "right_foot"):
            measured = rotate_xy(after["positions"][b.ends[key]] - after["positions"][b.pelvis], -yaw)
            weight = float(confidence[b.names[b.ends[key]]])
            ee_confidence.append(weight)
            ee_weighted += weight * float(np.sum(((measured - np.asarray(reference["root_relative_end_effectors"][key])) / b.length_scale) ** 2))
            ee_total += weight
        ee = ee_weighted / max(ee_total, 1e-9)
        style_ends = float(np.mean(ee_confidence))
        ref_height = float(reference["model_positions"][b.pelvis][2])
        com_height = float(((after["pelvis_height"] - ref_height) / b.length_scale) ** 2)
        cmd = np.asarray(command[:2], dtype=float)
        command_velocity = float(np.sum((after["com_velocity"][:2] - cmd) ** 2) / 3600.0)
        command_norm = float(np.linalg.norm(cmd))
        achieved_along_command_speed = (float(after["com_velocity"][:2] @ (cmd / command_norm))
                                        if command_norm > 1e-6 else 0.0)
        slip = 0.0
        for key, flag in (("left_foot", "left_supported"), ("right_foot", "right_supported")):
            if before["valid"] and before[flag] and after[flag]:
                d = after["positions"][b.ends[key]][:2] - before["positions"][b.ends[key]][:2]
                slip += float(np.sum(d ** 2) / 2.25)
        action_size = float(np.mean(action ** 2))
        action_change = float(np.mean((action - previous_action) ** 2))
        if before["valid"]:
            accel = (after["local_velocity"][joints] - before["local_velocity"][joints]) / self.dt / 100.0
            joint_acceleration = float(np.mean(np.sum(accel ** 2, axis=-1)))
        else:
            joint_acceleration = 0.0
        no_support = not (after["left_supported"] or after["right_supported"])
        caps = after["free_cap"] or after["solver_cap"]
        terminal = after["mode"] in (3, 4) or after["nonfoot_support"]
        reward = (0.25 * style_pose * math.exp(-2 * pose) + 0.05 * style_pose * math.exp(-velocity)
                  + 0.10 * style_ends * math.exp(-40 * ee) + 0.05 * math.exp(-10 * com_height)
                  + 0.55 * math.exp(-2.5 * command_velocity) - 0.25 * min(10.0, slip) - 0.01 * action_size
                  - 0.02 * action_change - 0.02 * min(10.0, joint_acceleration) - 0.50 * float(after["saturated"])
                  - 2.0 * float(no_support) - 4.0 * float(caps))
        if terminal:
            reward = -3.0
        return {"reward": reward, "pose": pose, "velocity": velocity, "end_effectors": ee, "com_height": com_height,
                "command_velocity": command_velocity, "support_slip": slip, "action_size": action_size,
                "action_change": action_change, "joint_acceleration": joint_acceleration, "saturated": float(after["saturated"]),
                "no_support": float(no_support), "caps": float(caps), "terminal": float(terminal),
                "achieved_along_command_speed": achieved_along_command_speed,
                "style_pose_confidence": style_pose, "style_end_confidence": style_ends}


# ----------------------------------------------------------------- policy


def build_policy(observation_dim: int, action_dim: int, spec: dict[str, Any]):
    import torch
    from torch import nn
    hidden = spec["optimizer"]["actor_hidden_sizes"]

    class Policy(nn.Module):
        def __init__(self):
            super().__init__()

            def net(output):
                layers, width = [], observation_dim
                for h in hidden:
                    layers += [nn.Linear(width, h), nn.Tanh()]
                    width = h
                layers.append(nn.Linear(width, output))
                return nn.Sequential(*layers)
            self.actor, self.critic = net(action_dim), net(1)
            nn.init.zeros_(self.actor[-1].weight)
            nn.init.zeros_(self.actor[-1].bias)
            self.log_std = nn.Parameter(torch.full((action_dim,), math.log(spec["optimizer"]["initial_gaussian_std"])))

        def distribution(self, observations):
            lo, hi = spec["optimizer"]["log_std_bounds"]
            return torch.distributions.Normal(self.actor(observations), self.log_std.clamp(lo, hi).exp())

    return Policy()


def behavior_clone_actor(policy, observations: np.ndarray, proposals: np.ndarray, *, epochs: int,
                         minibatch_size: int, learning_rate: float, seed: int):
    """Fit only the actor mean to bounded teacher proposals; critic/RNG stay explicit."""
    import torch
    if observations.ndim != 2 or proposals.ndim != 2 or len(observations) != len(proposals) or not len(observations):
        raise ValueError("behavior-cloning arrays must be non-empty aligned matrices")
    if epochs < 1 or minibatch_size < 1 or learning_rate <= 0.0:
        raise ValueError("behavior-cloning optimizer settings must be positive")
    states = torch.from_numpy(np.asarray(observations, dtype=np.float32))
    targets = torch.from_numpy(np.asarray(proposals, dtype=np.float32))
    optimizer = torch.optim.Adam(policy.actor.parameters(), lr=learning_rate)
    generator = torch.Generator().manual_seed(seed)
    with torch.no_grad():
        initial_loss = float(torch.mean((torch.tanh(policy.actor(states)) - targets) ** 2))
    epoch_losses = []
    for _ in range(epochs):
        permutation = torch.randperm(len(states), generator=generator)
        losses = []
        for start in range(0, len(states), minibatch_size):
            indices = permutation[start:start + minibatch_size]
            prediction = torch.tanh(policy.actor(states[indices]))
            loss = torch.mean((prediction - targets[indices]) ** 2)
            if not torch.isfinite(loss):
                raise RuntimeError("non-finite behavior-cloning loss")
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.actor.parameters(), 1.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        epoch_losses.append(float(np.mean(losses)))
    with torch.no_grad():
        final_loss = float(torch.mean((torch.tanh(policy.actor(states)) - targets) ** 2))
    return optimizer, generator, {"samples": len(states), "epochs": epochs,
                                  "initial_mse": initial_loss, "final_mse": final_loss,
                                  "epoch_mse": epoch_losses}


# ------------------------------------------------------------------ cases


def make_case(rng: np.random.Generator, perturbed: bool, torso: int) -> dict[str, Any]:
    if not perturbed:
        return {"perturbed": False, "impulse": None}
    angle = float(rng.uniform(0.0, 2 * math.pi))
    magnitude = float(rng.uniform(50.0, 400.0))
    step = int(rng.integers(180, 421))
    return {"perturbed": True, "impulse": {"step": step, "body": torso, "angle": angle, "magnitude": magnitude,
                                           "vector": [magnitude * math.cos(angle), magnitude * math.sin(angle), 0.0]}}


def training_episode_is_perturbed(index: int, perturbations_enabled: bool) -> bool:
    """Keep the alternating schedule explicit while allowing a frozen startup stage."""
    return bool(perturbations_enabled and index % 2 == 1)


def resolve_episode_steps(case: dict[str, Any], requested_steps: int | None, full_episode_steps: int) -> int:
    """Resolve an explicit call override or the case-bound training horizon."""
    resolved = int(case.get("episode_steps", full_episode_steps) if requested_steps is None else requested_steps)
    if resolved < 1 or resolved > full_episode_steps:
        raise ValueError(f"episode step limit {resolved} is outside [1, {full_episode_steps}]")
    return resolved


def validate_startup_horizon_curriculum(document: dict[str, Any] | None, episode_steps: int) -> list[dict[str, Any]]:
    """Validate an optional monotone, bounded startup-horizon curriculum."""
    if document is None:
        return []
    stages = document.get("stages") if isinstance(document, dict) else None
    if not isinstance(stages, list) or not stages:
        raise ValueError("startup_horizon_curriculum requires non-empty stages")
    horizons = []
    for index, stage in enumerate(stages):
        if not isinstance(stage, dict):
            raise ValueError(f"startup horizon stage {index} is not an object")
        horizon = int(stage.get("episode_steps", 0))
        minimum = int(stage.get("minimum_stage_usable_transitions", 0))
        maximum = int(stage.get("maximum_stage_usable_transitions", 0))
        positive_required = stage.get("require_positive_signed_speed", False)
        if horizon < 1 or horizon > episode_steps or minimum < 1 or maximum < minimum:
            raise ValueError(f"startup horizon stage {index} has invalid horizon or sample bounds")
        if not isinstance(positive_required, bool):
            raise ValueError(f"startup horizon stage {index} positive-speed gate must be boolean")
        horizons.append(horizon)
    if horizons != sorted(set(horizons)) or horizons[-1] != episode_steps:
        raise ValueError("startup horizon stages must increase uniquely and end at the full episode horizon")
    if int(document.get("probe_episodes", 0)) < 1 or int(document.get("probe_interval_usable_transitions", 0)) < 1:
        raise ValueError("startup horizon curriculum requires positive probe count and interval")
    if not bool(stages[-1].get("require_positive_signed_speed", False)):
        raise ValueError("the full-horizon startup stage must require positive signed speed")
    return stages


def validate_teacher_bootstrap_config(document: dict[str, Any] | None) -> dict[str, Any]:
    """Validate the bounded production-teacher behavior-cloning contract."""
    if not isinstance(document, dict):
        raise ValueError("teacher_bootstrap stage requires a configuration object")
    if document.get("source_controller") != "production_walker":
        raise ValueError("teacher bootstrap requires the physically verified production walker")
    case_ids = document.get("teacher_case_ids")
    if not isinstance(case_ids, list) or not case_ids or len(case_ids) != len(set(case_ids)):
        raise ValueError("teacher bootstrap case ids must be non-empty and unique")
    for field in ("epochs", "minibatch_size"):
        if int(document.get(field, 0)) < 1:
            raise ValueError(f"teacher bootstrap {field} must be positive")
    if float(document.get("learning_rate", 0.0)) <= 0.0:
        raise ValueError("teacher bootstrap learning_rate must be positive")
    survival = float(document.get("minimum_teacher_survival_fraction", -1.0))
    if not 0.0 <= survival <= 1.0 or not bool(document.get("require_positive_teacher_signed_speed", False)):
        raise ValueError("teacher bootstrap requires bounded survival and positive-speed teacher gates")
    coverage = float(document.get("minimum_teacher_pose_coverage_fraction", -1.0))
    if not 0.0 <= coverage <= 1.0:
        raise ValueError("teacher bootstrap requires bounded pose coverage")
    if document.get("student_success_gate") != "positive signed speed and fewer full-length falls than fresh zero residual":
        raise ValueError("teacher bootstrap student success gate is not frozen")
    return document


def validate_teacher_ablation_config(document: dict[str, Any] | None) -> dict[str, Any]:
    """Validate the frozen production-versus-pose-only causal comparison."""
    if not isinstance(document, dict):
        raise ValueError("teacher_ablation stage requires a configuration object")
    if document.get("source_controller") != "production_walker":
        raise ValueError("teacher ablation requires the production walker")
    case_ids = document.get("case_ids")
    if not isinstance(case_ids, list) or not case_ids or len(case_ids) != len(set(case_ids)):
        raise ValueError("teacher ablation case ids must be non-empty and unique")
    if document.get("variants") != ["production_full", "production_pose_only_oracle", "direct_zero_residual"]:
        raise ValueError("teacher ablation variants are not frozen")
    if document.get("pose_only_success_gate") != "positive signed speed and fewer full-length falls than direct zero residual":
        raise ValueError("teacher ablation success gate is not frozen")
    return document


def case_hash(case: dict[str, Any]) -> str:
    return sha256_json(case)[:16]


def resolved_case_documents(spec: dict[str, Any], torso: int) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    selection_rng = np.random.Generator(np.random.PCG64(int(spec["selection_seed"])))
    holdout_rng = np.random.Generator(np.random.PCG64(int(spec["holdout_seed"])))
    selection = [make_case(selection_rng, True, torso) for _ in range(int(spec["selection_cases"]))]
    holdout = [make_case(holdout_rng, True, torso) for _ in range(int(spec["holdout_cases"]))]
    for index, case in enumerate(selection):
        case["id"] = f"selection-{index}"
    for index, case in enumerate(holdout):
        case["id"] = f"holdout-{index}"
    hashes = [case_hash(case) for case in selection] + [case_hash(case) for case in holdout]
    if len(set(hashes)) != len(hashes):
        raise RuntimeError("selection/holdout cases are not disjoint")
    doc = {"selection_seed": spec["selection_seed"], "holdout_seed": spec["holdout_seed"], "selection": selection,
           "holdout_manifest_sha256": sha256_json(holdout), "holdout_cases_count": len(holdout),
           "holdout_status": "identity generated and hashed only; cases are not returned to callers or simulated"}
    del holdout
    return selection, doc


# ----------------------------------------------------------------- runner


class Runner:
    def __init__(self, fixture: Fixture, out: Path, wall_start: float | None = None):
        self.fixture = fixture
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "journals").mkdir(exist_ok=True)
        self.wall_start = time.monotonic() if wall_start is None else wall_start
        self.events = open(self.out / "events.jsonl", "a", encoding="utf-8", buffering=1)
        self.physics_steps = 0
        self.policy_decisions = 0
        self.updates = 0
        self.episodes = 0
        self.training_physics_steps = 0
        self.training_usable_transitions = 0
        self.training_warmup_steps = 0
        self.training_unusable_steps = 0
        self.evaluation_physics_steps = 0
        self.diagnostic_physics_steps = 0
        self.worker_resets = 0
        self.worker_reset_seconds = 0.0
        self.workers: list[Worker] = []
        self.manifest: dict[str, Any] | None = None
        self.builder: ObservationBuilder | None = None
        self.reward: RewardModel | None = None
        self.worker_index = 0
        self.rollout_pool: concurrent.futures.ProcessPoolExecutor | None = None

    # ------------------------------------------------------------ infra

    def elapsed(self) -> float:
        return time.monotonic() - self.wall_start

    def event(self, kind: str, **values: Any) -> None:
        record = {"kind": kind, "utc": utc_now(), "physics_steps": self.physics_steps, "updates": self.updates,
                  "episodes": self.episodes, "elapsed_seconds": round(self.elapsed(), 3), **values}
        self.events.write(json.dumps(record, allow_nan=False, default=str) + "\n")
        if kind not in ("update", "rollout_episode"):
            print(json.dumps({k: record[k] for k in ("kind", "physics_steps", "updates", "elapsed_seconds")} | {k: v for k, v in values.items() if isinstance(v, (int, float, str, bool)) and k != "kind"}, default=str), flush=True)

    def counter_snapshot(self) -> dict[str, Any]:
        return {
            "total_simulated_physics_steps": self.physics_steps,
            "training_simulated_physics_steps": self.training_physics_steps,
            "training_usable_optimizer_transitions": self.training_usable_transitions,
            "training_warmup_physics_steps": self.training_warmup_steps,
            "training_unusable_physics_steps": self.training_unusable_steps,
            "evaluation_physics_steps": self.evaluation_physics_steps,
            "diagnostic_physics_steps": self.diagnostic_physics_steps,
            "policy_decisions": self.policy_decisions, "updates": self.updates,
            "episodes": self.episodes, "worker_resets": self.worker_resets,
            "worker_reset_seconds": self.worker_reset_seconds,
        }

    @staticmethod
    def counter_snapshot_fields() -> tuple[str, ...]:
        return ("total_simulated_physics_steps", "training_simulated_physics_steps",
                "training_usable_optimizer_transitions", "training_warmup_physics_steps",
                "training_unusable_physics_steps", "evaluation_physics_steps",
                "diagnostic_physics_steps", "policy_decisions", "updates", "episodes",
                "worker_resets", "worker_reset_seconds")

    def account(self, accounting: dict[str, Any]) -> None:
        frames = int(accounting["simulated_physics_steps"])
        purpose = accounting["purpose"]
        self.physics_steps += frames
        self.episodes += 1
        self.policy_decisions += int(accounting.get("policy_decisions", 0))
        if purpose == "training":
            self.training_physics_steps += frames
            self.training_usable_transitions += int(accounting["usable_optimizer_transitions"])
            self.training_warmup_steps += int(accounting["warmup_physics_steps"])
            self.training_unusable_steps += int(accounting["unusable_physics_steps"])
        elif purpose == "evaluation":
            self.evaluation_physics_steps += frames
        else:
            self.diagnostic_physics_steps += frames
        self.worker_resets += int(accounting.get("worker_resets", 0))
        self.worker_reset_seconds += float(accounting.get("worker_reset_seconds", 0.0))

    def spawn_worker(self, index: int) -> Worker:
        env = {k: v for k, v in os.environ.items() if not k.startswith("OPENMW_RAGDOLL_")}
        worker = Worker(self.fixture.exe, self.fixture.context, self.fixture.profile, self.fixture.floor,
                        stderr_path=self.out / f"worker-{index}-stderr.log", env=env)
        if self.fixture.control_mode == "production-residual" and not math.isclose(
                float(worker.manifest.get("maximum_production_residual_offset_radians", -1)), 0.1, rel_tol=1e-6):
            worker.close()
            raise WorkerError("executable does not declare the required production residual bound")
        if ([body["name"] for body in worker.manifest["bodies"]] != self.fixture.library["body_names"]
                or [int(body["parent"]) for body in worker.manifest["bodies"]] != self.fixture.library["parent_indices"]):
            worker.close()
            raise WorkerError("worker and reference body mapping mismatch")
        if self.manifest is None:
            self.manifest = worker.manifest
            self.builder = ObservationBuilder(self.fixture, worker.manifest)
            self.reward = RewardModel(self.builder, self.fixture)
            manifest_doc = {"worker_manifest": worker.manifest, "worker_identity": worker.identity(),
                            "observation_blocks": self.builder.blocks, "observation_dimension": self.builder.dimension,
                            "action_dimension": self.builder.action_count, "length_scale": self.builder.length_scale,
                            "style_confidence": self.fixture.make_generator().identity()["style_confidence"],
                            "normalization": {"linear_velocity": self.builder.linear_scale, "angular_velocity": self.builder.angular_scale,
                                              "length": self.builder.length_scale, "running_normalizer": None}}
            (self.out / "resolved-joint-manifest.json").write_text(json.dumps(manifest_doc, indent=1, allow_nan=False) + "\n", encoding="utf-8")
        elif worker.manifest != self.manifest:
            worker.close()
            raise RuntimeError("worker manifests differ between processes")
        return worker

    def ensure_workers(self, count: int) -> None:
        while len(self.workers) < count:
            self.workers.append(self.spawn_worker(len(self.workers)))

    def replace_worker(self, index: int) -> None:
        try:
            self.workers[index].close()
        except Exception:
            pass
        self.workers[index] = self.spawn_worker(index)

    def ensure_rollout_pool(self) -> None:
        if self.rollout_pool is not None:
            return
        if self.manifest is None:
            self.ensure_workers(1)
        # The spec caps persistent physics processes at fixture.workers. Close
        # the readiness worker before replacing it with the process pool.
        for worker in self.workers:
            worker.close()
        self.workers = []
        fixture_args = {
            "source_root": str(self.fixture.source_root), "exe": str(self.fixture.exe),
            "library": str(self.fixture.library_path), "floor": self.fixture.floor,
            "workers": 1, "bound_speed": self.fixture.bound_speed,
            "command_speed": self.fixture.command_speed, "tracker_task": "PLAY-01",
            "control_mode": self.fixture.control_mode,
        }
        worker_root = self.out / "rollout-workers"
        worker_root.mkdir(exist_ok=True)
        context = multiprocessing.get_context("spawn")
        self.rollout_pool = concurrent.futures.ProcessPoolExecutor(
            max_workers=self.fixture.workers, mp_context=context,
            initializer=_parallel_initialize, initargs=(fixture_args, str(worker_root)))

    def parallel_episodes(self, policy, jobs: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Dispatch complete episodes while preserving case order and RNG identity."""
        if not jobs:
            return []
        self.ensure_rollout_pool()
        groups = [[] for _ in range(min(self.fixture.workers, len(jobs)))]
        run_seed = int(self.fixture.spec["optimizer"]["seed"])
        for index, job in enumerate(jobs):
            item = {**job, "index": index,
                    "episode_seed": episode_seed(run_seed, job["case"], job["seed_label"])}
            groups[index % len(groups)].append(item)
        state = None if policy is None else policy_to_numpy(policy)
        futures = [self.rollout_pool.submit(_parallel_episode_group, state, group) for group in groups]
        results = [result for future in futures for result in future.result()]
        results.sort(key=lambda result: result.pop("parallel_job_index"))
        for result in results:
            self.account(result["accounting"])
        return results

    def close(self) -> None:
        if self.rollout_pool is not None:
            self.rollout_pool.shutdown(wait=True, cancel_futures=True)
            self.rollout_pool = None
        for worker in self.workers:
            try:
                worker.close()
            except Exception:
                pass
        self.workers = []
        self.events.close()

    # ---------------------------------------------------------- episodes

    def episode_header(self, controller: str, case: dict[str, Any], steps: int | None = None,
                       motors_off_step: int | None = None, speed: float | None = None) -> np.ndarray:
        steps = resolve_episode_steps(case, steps, self.fixture.episode_steps)
        impulse = None
        if case.get("impulse"):
            i = case["impulse"]
            impulse = (i["step"], i["body"], *i["vector"])
        code = {"reference": 0.0, "direct": 1.0, "seam": 2.0, "production-residual": 3.0}[controller]
        header = Worker.episode_header(False, steps, self.fixture.commands(speed), impulse, motors_off_step)
        header[0] = code
        return header

    def command_at(self, step: int, speed: float | None = None) -> tuple[np.ndarray, np.ndarray]:
        commands = self.fixture.commands(speed)
        current = commands[0]
        for command in commands:
            if command[0] <= step:
                current = command
        return np.array([current[1], current[2]]), np.array([current[3], current[4]])

    def run_episode(self, worker: Worker, controller: str, case: dict[str, Any], policy=None, stochastic: bool = False,
                    replay: dict[str, Any] | None = None, journal: bool = False, label: str = "", motors_off_step: int | None = None,
                    pulses: list[tuple[int, int, int, float]] | None = None, steps: int | None = None,
                    torch_generator=None, purpose: str = "diagnostic", stop_on_terminal: bool = False,
                    collect_teacher: bool = False, capture_visual: bool = False) -> dict[str, Any]:
        """One complete episode on one worker; returns metrics, transitions and journal.

        controller: "reference" (production walker), "direct" or "seam".
        pulses: list of (scalar index, start step, end step, value) forced actions.
        replay: {"actions": (T, A), "poses": (T, B, 7)} bypassing generator and policy.
        """
        import torch
        b = self.builder
        steps = resolve_episode_steps(case, steps, self.fixture.episode_steps)
        episode_speed = float(case.get("command_speed", self.fixture.command_speed))
        header = self.episode_header(controller, case, steps, motors_off_step, speed=episode_speed)
        raw = worker.start_episode(header)
        generator = self.fixture.make_generator()
        previous_action = np.zeros(b.action_count, dtype=np.float32)
        transitions: dict[str, list[Any]] = {"obs": [], "latent": [], "logp": [], "value": [], "reward": [], "applied": [], "continuation": []}
        teacher_samples: dict[str, Any] = {"obs": [], "proposal": [], "desired_applied": [],
                                           "target_diagnostics": [], "filter_diagnostics": [],
                                           "eligible_frames": 0, "missing_driven_pose_steps": [],
                                           "oracle_actions": [], "oracle_poses": [],
                                           "oracle_target_diagnostics": []}
        journal_rows: list[dict[str, Any]] = []
        visual_frames: list[dict[str, Any]] = []
        terms_log: list[dict[str, float]] = []
        phys_before = b.physical(raw)
        terminated_at: int | None = None
        end: dict[str, Any] | None = None
        physical_authority_ok = True
        offsets_log = []
        delivered_log = []
        rotations_log = []
        utilization_log = []
        speed_log: list[float] = []
        max_stance_slide = 0.0
        airborne_frames = nonfoot_frames = free_cap_frames = solver_cap_frames = 0
        max_applied_force = 0.0
        telemetry = hashlib.sha256(raw.tobytes())
        episode_policy_decisions = 0
        requires_worker_reset = False
        unexpected_worker_end = False
        for step in range(steps):
            command, facing = self.command_at(step, episode_speed)
            if controller == "reference":
                teacher_reference = None
                teacher_observation = None
                if collect_teacher:
                    measured_along = 0.0
                    if phys_before["valid"] and np.linalg.norm(command) > 1e-6:
                        measured_along = float(phys_before["com_velocity"][:2]
                                               @ (command / np.linalg.norm(command)))
                    teacher_reference = generator.step(command, facing, measured_along)
                    if phys_before["valid"] and policy_controls_step(step, self.fixture.warmup_steps):
                        teacher_observation = b.vector(
                            phys_before, generator, teacher_reference, command, facing, previous_action)
                raw, end = worker.send_action(np.array([step], dtype=np.float32))
                if end is not None and end.get("kind") == "error":
                    raise WorkerError(end["message"])
                if raw is None:
                    raise WorkerError(f"worker ended full-length {purpose} trajectory without an observation")
                phys_after = b.physical(raw)
                telemetry.update(raw.tobytes())
                oracle_desired = None
                if teacher_reference is not None:
                    teacher_samples["oracle_poses"].append(np.concatenate([
                        teacher_reference["model_positions"], teacher_reference["model_rotations"]
                    ], axis=-1).astype(np.float32))
                    if phys_after["driven_motor_local"] is None:
                        oracle_desired = np.zeros(b.action_count, dtype=np.float32)
                    else:
                        oracle_desired, oracle_diagnostics = teacher_action_from_local_targets(
                            phys_after["driven_motor_local"], teacher_reference["local_rotations"], b.actions)
                        teacher_samples["oracle_target_diagnostics"].append(oracle_diagnostics)
                    teacher_samples["oracle_actions"].append(oracle_desired)
                if teacher_observation is not None:
                    teacher_samples["eligible_frames"] += 1
                    if phys_after["driven_motor_local"] is None:
                        # Full diagnostics continue after terminal states, but
                        # the stopped production controller has no pose to copy.
                        teacher_samples["missing_driven_pose_steps"].append(step)
                    else:
                        desired = oracle_desired
                        target_diagnostics = teacher_samples["oracle_target_diagnostics"][-1]
                        proposal, delivered, filter_diagnostics = teacher_filter_proposal(
                            desired, previous_action, b.actions)
                        teacher_samples["obs"].append(teacher_observation)
                        teacher_samples["proposal"].append(proposal)
                        teacher_samples["desired_applied"].append(desired)
                        teacher_samples["target_diagnostics"].append(target_diagnostics)
                        teacher_samples["filter_diagnostics"].append(filter_diagnostics)
                        previous_action = delivered
                if teacher_reference is not None:
                    effective_command = command * float(teacher_reference["walk_weight"])
                    terms = self.reward.terms(
                        phys_before, phys_after, teacher_reference, effective_command,
                        previous_action, previous_action)
                else:
                    terms = self.reward.terms(phys_before, phys_after, self._reference_stub(phys_after),
                                              command, previous_action, previous_action)
                terms_log.append(terms)
                if terms["terminal"] and terminated_at is None:
                    terminated_at = step
                if capture_visual:
                    visual_frames.append(self.visual_frame(step, phys_after, command, teacher_reference))
                phys_before = phys_after
                if end is not None:
                    break
                continue
            # Direct / seam controllers: reference from the generator, residual from the policy.
            measured_along = 0.0
            if phys_before["valid"] and np.linalg.norm(command) > 1e-6:
                measured_along = float(phys_before["com_velocity"][:2] @ (command / np.linalg.norm(command)))
            if replay is not None:
                pose = np.asarray(replay["poses"][step], dtype=np.float32)
                applied = np.asarray(replay["actions"][step], dtype=np.float32)
                # Replay still advances the deterministic generator for
                # command/style metrics; only its recorded pose and action own
                # physics input. This keeps exact action replay unchanged.
                reference = generator.step(command, facing, measured_along)
                observation = None
                latent = logp = value = None
            else:
                reference = generator.step(command, facing, measured_along)
                pose = np.concatenate([reference["model_positions"], reference["model_rotations"]], axis=-1).astype(np.float32)
                if phys_before["valid"] and policy is not None and policy_controls_step(step, self.fixture.warmup_steps):
                    observation = b.vector(phys_before, generator, reference, command, facing, previous_action)
                    with torch.no_grad():
                        distribution = policy.distribution(torch.from_numpy(observation)[None])
                        if not stochastic:
                            latent = distribution.mean
                        elif torch_generator is None:
                            latent = distribution.sample()
                        else:
                            latent = distribution.mean + distribution.stddev * torch.randn(distribution.mean.shape, generator=torch_generator)
                        logp = distribution.log_prob(latent).sum(-1)
                        value = policy.critic(torch.from_numpy(observation)[None]).squeeze(-1)
                    proposed = project_residual_action(torch.tanh(latent)[0].numpy(), b.actions)
                    episode_policy_decisions += 1
                elif phys_before["valid"]:
                    observation = None
                    latent = logp = value = None
                    proposed = np.zeros(b.action_count, dtype=np.float32)
                else:
                    observation = None
                    latent = logp = value = None
                    proposed = np.zeros(b.action_count, dtype=np.float32)
                applied = (0.8 * previous_action + 0.2 * proposed).astype(np.float32)
                if not phys_before["valid"]:
                    applied = np.zeros(b.action_count, dtype=np.float32)
                if pulses:
                    for scalar, start, stop, value_ in pulses:
                        if start <= step < stop:
                            applied[scalar] = value_
                    applied = project_residual_action(np.clip(applied, -1.0, 1.0), b.actions)
            motors_enabled = 1.0 if motors_off_step is None or step < motors_off_step else 0.0
            message = np.concatenate([np.array([step, motors_enabled], dtype=np.float32), applied, pose.reshape(-1)]).astype(np.float32)
            raw, end = worker.send_action(message)
            if end is not None and end.get("kind") == "error":
                raise WorkerError(end["message"])
            if raw is None:
                if purpose == "training" and stop_on_terminal and is_physical_early_end(end):
                    terminated_at = int(end["fall_step"] if end.get("fall_step") is not None else max(0, int(end["frames"]) - 1))
                    requires_worker_reset = True
                    unexpected_worker_end = True
                    break
                raise WorkerError(
                    "worker returned no observation: "
                    + ("no end record" if end is None else json.dumps(end, sort_keys=True, allow_nan=False)))
            phys_after = b.physical(raw)
            telemetry.update(raw.tobytes())
            if controller != "production-residual" and (phys_after["net_force"] != 0.0 or phys_after["pelvis_force"] != 0.0 or phys_after["pelvis_torque"] != 0.0):
                physical_authority_ok = False
            if motors_off_step is not None and step >= motors_off_step:
                if phys_after["net_force"] != 0.0 or phys_after["pelvis_force"] != 0.0 or phys_after["pelvis_torque"] != 0.0:
                    physical_authority_ok = False
            if reference is not None:
                effective_command = command * float(reference["walk_weight"])
                terms = self.reward.terms(phys_before, phys_after, reference, effective_command, applied, previous_action)
            else:
                terms = {"reward": 0.0, "terminal": float(phys_after["mode"] in (3, 4) or phys_after["nonfoot_support"])}
            terms_log.append(terms)
            if terms["terminal"] and terminated_at is None:
                terminated_at = step
            if purpose == "training" and terms["terminal"]:
                command_stop_step = min(steps, self.fixture.warmup_steps + self.fixture.start_blend_steps + self.fixture.walk_steps)
                terms["reward"] = terminal_reward_with_lost_command_opportunity(
                    step, command_stop_step, float(self.fixture.spec["optimizer"]["discount"]))
                terms["terminal_lost_command_opportunity"] = -3.0 - terms["reward"]
            if observation is not None and latent is not None and step >= self.fixture.warmup_steps:
                transitions["obs"].append(observation)
                transitions["latent"].append(latent[0].numpy())
                transitions["logp"].append(float(logp))
                transitions["value"].append(float(value))
                transitions["reward"].append(terms["reward"])
                transitions["applied"].append(applied)
                transitions["continuation"].append(0.0 if terms["terminal"] else 1.0)
            if journal:
                row = {"step": step, "applied_action": applied.tolist(), "motors_enabled": motors_enabled,
                       "reference_pose": pose.tolist(), "generator": None if reference is None else {
                           "phase": reference["phase"], "rate": reference["rate"], "walk_weight": reference["walk_weight"],
                           "family_weights": reference["family_weights"], "coverage": reference["coverage"], "reference_yaw": reference["reference_yaw"]}}
                journal_rows.append(row)
            offsets_log.append(phys_after["applied_offsets"].copy())
            delivered_log.append(phys_after["delivered"].copy())
            rotations_log.append(phys_after["rotations"].copy())
            utilization_log.append(phys_after["utilization"].copy())
            command_norm = float(np.linalg.norm(command))
            speed_log.append(float(phys_after["com_velocity"][:2] @ (command / command_norm)) if command_norm > 1e-6 else 0.0)
            for key, flag in (("left_foot", "left_supported"), ("right_foot", "right_supported")):
                if phys_before["valid"] and phys_before[flag] and phys_after[flag]:
                    max_stance_slide = max(max_stance_slide, float(np.linalg.norm(phys_after["positions"][b.ends[key]][:2] - phys_before["positions"][b.ends[key]][:2])))
            airborne_frames += int(not (phys_after["left_supported"] or phys_after["right_supported"]))
            nonfoot_frames += int(phys_after["nonfoot_support"])
            free_cap_frames += int(phys_after["free_cap"])
            solver_cap_frames += int(phys_after["solver_cap"])
            max_applied_force = max(max_applied_force, phys_after["net_force"], phys_after["pelvis_force"], phys_after["pelvis_torque"])
            if capture_visual:
                visual_frames.append(self.visual_frame(step, phys_after, command, reference))
            previous_action = applied
            phys_before = phys_after
            if should_end_rollout(purpose, stop_on_terminal, bool(terms["terminal"]), end is not None):
                requires_worker_reset = True
                break
            if end is not None:
                break
        if end is None:
            if not requires_worker_reset:
                raise WorkerError("episode ended without an end record")
            actual_steps = len(terms_log)
            end = {
                "kind": "end", "partial": True, "frames": actual_steps,
                "fall_step": terminated_at if phys_before["mode"] in (3, 4) else None,
                "telemetry_hash": "partial:" + telemetry.hexdigest(),
                "max_stance_foot_slide": max_stance_slide,
                "mean_achieved_speed": float(np.mean(speed_log)) if speed_log else 0.0,
                "final_half_second_speed": float(np.mean(speed_log[-30:])) if speed_log else 0.0,
                "airborne_frames": airborne_frames, "nonfoot_support_frames": nonfoot_frames,
                "free_velocity_limited_frames": free_cap_frames, "solver_velocity_limited_frames": solver_cap_frames,
                "max_applied_net_force": max_applied_force,
            }
        actual_steps = int(end["frames"])
        accounting = {
            "purpose": purpose, "simulated_physics_steps": actual_steps,
            "usable_optimizer_transitions": len(transitions["obs"]) if purpose == "training" else 0,
            "warmup_physics_steps": min(actual_steps, self.fixture.warmup_steps) if purpose == "training" else 0,
            "unusable_physics_steps": max(0, actual_steps - len(transitions["obs"]) - min(actual_steps, self.fixture.warmup_steps)) if purpose == "training" else 0,
            "policy_decisions": episode_policy_decisions,
            "worker_resets": 0, "worker_reset_seconds": 0.0,
        }
        self.account(accounting)
        metrics = self.episode_metrics(end, terms_log, terminated_at, physical_authority_ok, controller)
        return {"end": end, "metrics": metrics, "transitions": transitions, "journal": journal_rows, "terms": terms_log,
                "physical_observation_sha256": telemetry.hexdigest(),
                "offsets": np.asarray(offsets_log), "delivered": np.asarray(delivered_log), "rotations": np.asarray(rotations_log),
                "utilization": np.asarray(utilization_log),
                "label": label, "case": case, "accounting": accounting,
                "teacher_samples": teacher_samples,
                "visual_frames": visual_frames,
                "requires_worker_reset": requires_worker_reset, "unexpected_worker_end": unexpected_worker_end}

    @staticmethod
    def visual_frame(step: int, phys: dict[str, Any], command: np.ndarray,
                     reference: dict[str, Any] | None) -> dict[str, Any]:
        """Return the small, JSON-safe physical subset needed by the motion viewer."""
        return {
            "step": int(step),
            "positions": np.round(np.asarray(phys["positions"], dtype=float), 4).tolist(),
            "com_velocity": np.round(np.asarray(phys["com_velocity"], dtype=float), 4).tolist(),
            "command": np.round(np.asarray(command, dtype=float), 4).tolist(),
            "mode": int(phys["mode"]),
            "left_supported": bool(phys["left_supported"]),
            "right_supported": bool(phys["right_supported"]),
            "nonfoot_support": bool(phys["nonfoot_support"]),
            "phase": None if reference is None else round(float(reference["phase"]), 6),
            "walk_weight": None if reference is None else round(float(reference["walk_weight"]), 6),
        }

    def _reference_stub(self, phys: dict[str, Any]) -> dict[str, Any]:
        b = self.builder
        return {"local_rotations": phys["local"], "local_angular_velocities": phys["local_velocity"],
                "root_relative_end_effectors": {k: rotate_xy(phys["positions"][b.ends[k]] - phys["positions"][b.pelvis], -phys["yaw"]).tolist() for k in b.ends},
                "model_positions": np.array([[0, 0, phys["pelvis_height"]]] * len(b.names)), "phase": 0.0}

    def episode_metrics(self, end: dict[str, Any], terms: list[dict[str, float]], terminated_at: int | None,
                        authority_ok: bool, controller: str) -> dict[str, Any]:
        f = self.fixture
        walk_lo = f.warmup_steps + f.start_blend_steps
        walk_hi = walk_lo + f.walk_steps
        cmd_err = [t.get("command_velocity", 0.0) for t in terms[walk_lo:walk_hi]]
        speed_rmse = math.sqrt(np.mean(cmd_err)) * 60.0 if cmd_err else None
        signed_speed = [t.get("achieved_along_command_speed", 0.0) for t in terms[walk_lo:walk_hi]]
        accel = [t.get("joint_acceleration", 0.0) for t in terms]
        slip = [t.get("support_slip", 0.0) for t in terms]
        return {
            "controller": controller, "frames": end["frames"], "fall_step": end["fall_step"], "terminated_at": terminated_at,
            "telemetry_hash": end["telemetry_hash"], "reward_sum": float(sum(t.get("reward", 0.0) for t in terms)),
            "commanded_velocity_rmse_walk": speed_rmse, "joint_acceleration_rms": float(math.sqrt(np.mean(accel))) * 100.0 if accel else None,
            "mean_achieved_along_command_speed_walk": float(np.mean(signed_speed)) if signed_speed else None,
            "support_slip_sum": float(sum(slip)), "max_stance_foot_slide": end["max_stance_foot_slide"],
            "mean_achieved_speed": end["mean_achieved_speed"], "final_half_second_speed": end["final_half_second_speed"],
            "airborne_frames": end["airborne_frames"], "nonfoot_support_frames": end["nonfoot_support_frames"],
            "free_velocity_limited_frames": end["free_velocity_limited_frames"], "solver_velocity_limited_frames": end["solver_velocity_limited_frames"],
            "saturated_steps": int(sum(t.get("saturated", 0.0) for t in terms)), "max_applied_net_force": end["max_applied_net_force"],
            "physical_authority_ok": authority_ok, "pose_error_mean": float(np.mean([t.get("pose", 0.0) for t in terms])) if terms else None,
            "violations": {
                "falling_or_fallen": end["fall_step"] is not None, "nonfoot_support": end["nonfoot_support_frames"] > 0,
                "solver_or_free_velocity_cap": (end["free_velocity_limited_frames"] + end["solver_velocity_limited_frames"]) > 0,
                "stance_slide_above_1.5": end["max_stance_foot_slide"] > 1.5,
                "walking_speed_error_above_20_percent": bool(speed_rmse > 0.2 * f.command_speed) if speed_rmse is not None else None,
                "motor_limit_exceeded": int(sum(t.get("saturated", 0.0) for t in terms)) > 0,
                "root_or_other_nonjoint_actuation": (not authority_ok) if controller not in ("reference", "production-residual") else None,
            },
        }

    # ---------------------------------------------------------- journals

    def write_journal(self, name: str, result: dict[str, Any]) -> dict[str, Any]:
        path = self.out / "journals" / f"{name}.jsonl.gz"
        with gzip.open(path, "wt", encoding="utf-8") as stream:
            stream.write(json.dumps({"kind": "header", "label": result["label"], "case": result["case"], "end": result["end"],
                                     "identity": self.fixture.identity}, allow_nan=False) + "\n")
            for row, offsets, delivered in zip(result["journal"], result["offsets"], result["delivered"]):
                stream.write(json.dumps({**row, "applied_offsets": offsets.tolist(), "delivered_targets": delivered.tolist()}, allow_nan=False) + "\n")
        return {"path": str(path), "sha256": sha256(path), "rows": len(result["journal"])}

    @staticmethod
    def replay_payload(result: dict[str, Any]) -> dict[str, Any]:
        return {"actions": np.asarray([row["applied_action"] for row in result["journal"]], dtype=np.float32),
                "poses": np.asarray([row["reference_pose"] for row in result["journal"]], dtype=np.float32)}

    # ---------------------------------------------------------- readiness

    def readiness(self, policy=None) -> dict[str, Any]:
        self.ensure_workers(1)
        w = self.workers[0]
        b = self.builder
        f = self.fixture
        controlled = f.policy_controller
        torso = int(self.manifest["landmarks"][TORSO_LANDMARK])
        control_case = {"perturbed": False, "impulse": None, "id": "unperturbed-control"}
        report: dict[str, Any] = {"generated_utc": utc_now(), "identity": f.identity, "requirements": {}}
        req = report["requirements"]
        started = self.physics_steps

        # current_fixture_identity
        req["current_fixture_identity"] = {"passed": all(f.identity[k] for k in ("executable_sha256", "context_sha256", "profile_sha256", "library_sha256", "spec_sha256")),
                                           "worker_identity": w.identity(), "dirty_files": f.identity["git"]["dirty_files"]}

        # zero_action_identity: seam (empty offsets) vs direct (explicit zeros), repeated.
        seam = self.run_episode(w, "reference" if controlled == "production-residual" else "seam", control_case, journal=True, label="zero-seam")
        direct = self.run_episode(w, controlled, control_case, journal=True, label="zero-direct")
        direct_repeat = self.run_episode(w, controlled, control_case, journal=True, label="zero-direct-repeat")
        req["zero_action_identity"] = {
            "passed": seam["physical_observation_sha256"] == direct["physical_observation_sha256"] == direct_repeat["physical_observation_sha256"],
            "observation_hashes": [r["physical_observation_sha256"] for r in (seam, direct, direct_repeat)],
            "seam_hash": seam["end"]["telemetry_hash"], "direct_hash": direct["end"]["telemetry_hash"], "repeat_hash": direct_repeat["end"]["telemetry_hash"],
            "zero_residual_tracker_metrics": direct["metrics"], "journals": {"seam": self.write_journal("zero-seam", seam), "direct": self.write_journal("zero-direct", direct)},
        }
        self.event("readiness", requirement="zero_action_identity", passed=req["zero_action_identity"]["passed"])

        # nonzero_action_authority: paired pulses per scalar on short episodes.
        production_mode = controlled == "production-residual"
        pulse_steps = 180 if production_mode else 90
        positive = (60, 100, 0.6) if production_mode else (20, 40, 0.6)
        negative = (115, 155, -0.6) if production_mode else (55, 75, -0.6)
        pos_window = slice(positive[0] + 10, positive[1])
        neg_window = slice(negative[0] + 10, negative[1])
        response_window = slice(positive[0] + 10, negative[0])
        zero_short = self.run_episode(w, controlled, control_case, label="authority-zero", steps=pulse_steps)
        axes = []
        for action in self.manifest["actions"]:
            body = action["body"]
            for k in range(action["dimensions"]):
                scalar = action["first_scalar"] + k
                pulsed = self.run_episode(w, controlled, control_case, label=f"authority-{scalar}", steps=pulse_steps,
                                          pulses=[(scalar, *positive), (scalar, *negative)])
                d0 = zero_short["delivered"][pos_window, body]
                d1 = pulsed["delivered"][pos_window, body]
                d2 = pulsed["delivered"][neg_window, body]
                target_change = float(np.max(np.abs(d1 - d0)))
                target_change_negative = float(np.max(np.abs(d2 - zero_short["delivered"][neg_window, body])))
                q_pulsed = pulsed["rotations"][response_window, body]
                q_zero = zero_short["rotations"][response_window, body]
                physical = float(np.max(np.linalg.norm(motion.rotation_vector(quat_mul(q_pulsed, quat_conj(q_zero))), axis=-1)))
                differs = pulsed["end"]["telemetry_hash"] != zero_short["end"]["telemetry_hash"]
                target_utilization = np.concatenate([
                    pulsed["utilization"][positive[0]:positive[1], body], pulsed["utilization"][negative[0]:negative[1], body]
                ])
                target_utilization_peak = float(np.max(target_utilization))
                target_saturated_fraction = float(np.mean(target_utilization >= 1.0))
                target_fully_saturated = bool(np.all(target_utilization >= 1.0))
                axes.append({"scalar": scalar, "body": body, "name": b.names[body], "dimension": k,
                             "delivered_target_change_positive": target_change, "delivered_target_change_negative": target_change_negative,
                             "physical_rotation_response_radians": physical, "telemetry_differs": differs,
                             "target_motor_utilization_peak": target_utilization_peak,
                             "target_motor_saturated_fraction": target_saturated_fraction,
                             "target_motor_fully_saturated": target_fully_saturated,
                             "controllable": bool(target_change > 1e-4 and target_change_negative > 1e-4 and differs and physical > 1e-4)})
        groups = {}
        for axis in axes:
            key = "hinge" if self.manifest["bodies"][axis["body"]]["constraint"]["type"] == "hinge" else "swing_twist"
            groups.setdefault(key, []).append(axis)
        req["nonzero_action_authority"] = {
            "passed": all(a["controllable"] for a in axes) and all(any(a["controllable"] and not a["target_motor_fully_saturated"] for a in g) for g in groups.values()),
            "axes": axes, "uncontrollable": [a["scalar"] for a in axes if not a["controllable"]],
            "pulse_protocol": {"steps": pulse_steps, "positive": list(positive), "negative": list(negative)},
        }
        self.event("readiness", requirement="nonzero_action_authority", passed=req["nonzero_action_authority"]["passed"],
                   uncontrollable=len(req["nonzero_action_authority"]["uncontrollable"]))

        # exact_action_replay: stochastic zero-initialized policy, journal, replay twice.
        import torch
        rng = torch.Generator().manual_seed(int(f.spec["optimizer"]["seed"]))
        if policy is None:
            torch.manual_seed(int(f.spec["optimizer"]["seed"]))
            policy = build_policy(b.dimension, b.action_count, f.spec)
        sampled = self.run_episode(w, controlled, control_case, policy=policy, stochastic=True, torch_generator=rng, journal=True, label="replay-source")
        payload = self.replay_payload(sampled)
        replay1 = self.run_episode(w, controlled, control_case, replay=payload, journal=True, label="replay-1")
        replay2 = self.run_episode(w, controlled, control_case, replay=payload, journal=True, label="replay-2")
        req["exact_action_replay"] = {
            "passed": sampled["end"]["telemetry_hash"] == replay1["end"]["telemetry_hash"] == replay2["end"]["telemetry_hash"],
            "source_hash": sampled["end"]["telemetry_hash"], "replay_hashes": [replay1["end"]["telemetry_hash"], replay2["end"]["telemetry_hash"]],
            "nonzero_actions": bool(np.abs(payload["actions"]).max() > 0), "journal": self.write_journal("replay-source", sampled),
        }
        self.event("readiness", requirement="exact_action_replay", passed=req["exact_action_replay"]["passed"])

        # motor_off_authority: disable motors mid-episode with a nonzero policy.
        off_step = 150
        off = self.run_episode(w, controlled, control_case, policy=policy, stochastic=True, torch_generator=rng, motors_off_step=off_step, label="motor-off")
        after_off = off["delivered"][off_step + 1:]
        util_after = [t.get("saturated", 0.0) for t in off["terms"][off_step + 1:]]
        req["motor_off_authority"] = {
            "passed": bool(np.all(after_off == 0.0)) and off["metrics"]["physical_authority_ok"],
            "delivered_targets_after_off_all_zero": bool(np.all(after_off == 0.0)), "off_step": off_step,
            "physical_authority_ok": off["metrics"]["physical_authority_ok"], "fall_after_release": off["end"]["fall_step"],
        }
        self.event("readiness", requirement="motor_off_authority", passed=req["motor_off_authority"]["passed"])

        # malformed_action_rejection: each malformed message must be rejected before its solve.
        rejections = []
        creation = np.array([bb["creation_translation"] + bb["creation_rotation_xyzw"] for bb in self.manifest["bodies"]], dtype=np.float32).reshape(-1)
        good = lambda step: np.concatenate([np.array([step, 1.0], dtype=np.float32), np.zeros(b.action_count, dtype=np.float32), creation])
        cases_bad = {
            "non_finite": lambda step: (lambda m: (m.__setitem__(2, np.nan), m)[1])(good(step)),
            "out_of_range": lambda step: (lambda m: (m.__setitem__(3, 1.5), m)[1])(good(step)),
            "wrong_dimension": lambda step: good(step)[:-1],
            "stale_step": lambda step: good(step + 1),
            "invalid_motor_flag": lambda step: (lambda m: (m.__setitem__(1, 0.5), m)[1])(good(step)),
            "denormalized_reference": lambda step: (lambda m: (m.__setitem__(slice(2 + b.action_count + 3, 2 + b.action_count + 7), [0, 0, 0, 3.0]), m)[1])(good(step)),
        }
        for name, make in cases_bad.items():
            try:
                worker = self.workers[0]
                header = self.episode_header(controlled, control_case, 30)
                worker.start_episode(header)
                _, msg = worker.send_action(good(0))
                self.physics_steps += 1
                _, msg = worker.send_action(make(1))
                rejected = msg is not None and msg.get("kind") == "error"
                rejections.append({"case": name, "rejected_before_solve": rejected, "message": None if msg is None else msg.get("message")})
            except WorkerError as error:
                rejections.append({"case": name, "rejected_before_solve": True, "message": str(error)})
            finally:
                self.replace_worker(0)
        w = self.workers[0]
        req["malformed_action_rejection"] = {"passed": all(r["rejected_before_solve"] for r in rejections), "cases": rejections,
                                             "root_and_passive": "the action vector has no root or passive entries by construction; offsets for them are always zero"}
        self.event("readiness", requirement="malformed_action_rejection", passed=req["malformed_action_rejection"]["passed"])

        # physical_authority: every direct episode reported zero root/pelvis actuation.
        req["physical_authority"] = {"passed": all(r["metrics"]["physical_authority_ok"] and r["end"]["max_applied_net_force"] == 0 for r in (seam, direct, direct_repeat, sampled, replay1, replay2, off)),
                                     "max_applied_net_force": max(r["end"]["max_applied_net_force"] for r in (seam, direct, sampled, off))}
        if controlled == "production-residual":
            req["physical_authority"] = {
                "passed": req["zero_action_identity"]["passed"] and req["motor_off_authority"]["passed"],
                "meaning": "Production retains existing non-pose actuation; zero correction matches all observed forces/targets; motor-off clears actuation. The policy only supplies bounded joint offsets.",
                "production_max_applied_net_force": seam["end"]["max_applied_net_force"],
                "zero_residual_max_applied_net_force": direct["end"]["max_applied_net_force"],
            }
        # stage_motion_coverage
        coverage = f.library["coverage"]
        req["stage_motion_coverage"] = {"passed": coverage["idle"]["status"] == "content_ready" and coverage["forward"]["status"] == "content_ready",
                                        "stage": "integration_and_forward", "coverage": {k: v["status"] for k, v in coverage.items()}}
        report["physics_steps_used"] = self.physics_steps - started
        report["passed"] = all(v["passed"] for v in req.values())
        (self.out / "readiness-raw.json").write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
        self.event("readiness_complete", passed=report["passed"], physics_steps_used=report["physics_steps_used"])
        return report

    # ------------------------------------------------------------- PPO

    def make_optimizer(self, policy):
        import torch
        return torch.optim.Adam(policy.parameters(), lr=float(self.fixture.spec["optimizer"]["adam_learning_rate"]))

    def rollout_batch(self, policy, cases: list[dict[str, Any]], torch_generator, label: str = "train") -> dict[str, Any]:
        """Run stochastic episodes in persistent processes with per-episode RNGs."""
        jobs = [{"controller": self.fixture.policy_controller, "case": case, "stochastic": True, "journal": False,
                 "compact": True, "label": f"{label}-{case.get('id', i)}", "seed_label": label,
                 "purpose": "training", "stop_on_terminal": True}
                for i, case in enumerate(cases)]
        results = self.parallel_episodes(policy, jobs)
        for case, result in zip(cases, results):
            self.event("rollout_episode", label=label, case=case_hash(case), reward=result["metrics"]["reward_sum"],
                       fall_step=result["end"]["fall_step"], speed_rmse=result["metrics"]["commanded_velocity_rmse_walk"],
                       achieved_along_speed=result["metrics"]["mean_achieved_along_command_speed_walk"],
                       terminated_at=result["metrics"]["terminated_at"],
                       actual_physics_steps=result["accounting"]["simulated_physics_steps"],
                       unexpected_worker_end=result["unexpected_worker_end"])
        return {"results": results}

    def update(self, policy, optimizer, batch: dict[str, Any]) -> dict[str, Any]:
        import torch
        from torch import nn
        spec = self.fixture.spec["optimizer"]
        gamma, lam = float(spec["discount"]), float(spec["gae_lambda"])
        states, actions, logps, targets, advantages = [], [], [], [], []
        for result in batch["results"]:
            t = result["transitions"]
            if not t["obs"]:
                continue
            rewards = np.asarray(t["reward"], dtype=np.float32)
            values = np.asarray(t["value"], dtype=np.float32)
            continuation = np.asarray(t["continuation"], dtype=np.float32)
            adv, target = gae_returns(rewards, values, continuation, gamma, lam)
            states.append(np.asarray(t["obs"], dtype=np.float32))
            actions.append(np.asarray(t["latent"], dtype=np.float32))
            logps.append(np.asarray(t["logp"], dtype=np.float32))
            targets.append(target)
            advantages.append(adv)
        if not states:
            raise RuntimeError("empty batch")
        states = torch.from_numpy(np.concatenate(states))
        actions = torch.from_numpy(np.concatenate(actions))
        old_logp = torch.from_numpy(np.concatenate(logps))
        targets = torch.from_numpy(np.concatenate(targets))
        adv = torch.from_numpy(np.concatenate(advantages))
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        losses = []
        for _ in range(int(spec["epochs_per_update"])):
            permutation = torch.randperm(len(adv))
            for batch_index in permutation.split(int(spec["minibatch_size"])):
                distribution = policy.distribution(states[batch_index])
                logp = distribution.log_prob(actions[batch_index]).sum(-1)
                ratio = (logp - old_logp[batch_index]).exp()
                clip = float(spec["ratio_clip"])
                actor_loss = -torch.minimum(ratio * adv[batch_index], ratio.clamp(1 - clip, 1 + clip) * adv[batch_index]).mean()
                critic_loss = (policy.critic(states[batch_index]).squeeze(-1) - targets[batch_index]).square().mean()
                loss = actor_loss + float(spec["value_loss_weight"]) * critic_loss
                if not torch.isfinite(loss):
                    raise RuntimeError("non-finite loss")
                optimizer.zero_grad()
                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(policy.parameters(), float(spec["gradient_norm_limit"]))
                if not torch.isfinite(grad_norm):
                    raise RuntimeError("non-finite gradient")
                optimizer.step()
                losses.append((float(actor_loss.detach()), float(critic_loss.detach()), float(grad_norm.detach())))
        self.updates += 1
        parameter_hash = hashlib.sha256(b"".join(p.detach().numpy().tobytes() for p in policy.parameters())).hexdigest()
        speed_values = [float(r["metrics"]["commanded_velocity_rmse_walk"]) for r in batch["results"]
                        if r["metrics"]["commanded_velocity_rmse_walk"] is not None]
        signed_speed_values = [float(r["metrics"]["mean_achieved_along_command_speed_walk"]) for r in batch["results"]
                               if r["metrics"]["mean_achieved_along_command_speed_walk"] is not None]
        record = {"transitions": int(len(adv)), "actor_loss": losses[-1][0], "critic_loss": losses[-1][1], "grad_norm": losses[-1][2],
                  "mean_reward": float(np.mean([r["metrics"]["reward_sum"] for r in batch["results"]])),
                  "mean_speed_rmse": float(np.mean(speed_values)) if speed_values else None,
                  "mean_achieved_along_speed": float(np.mean(signed_speed_values)) if signed_speed_values else None,
                  "falls": int(sum(r["end"]["fall_step"] is not None for r in batch["results"])), "parameter_sha256": parameter_hash,
                  "log_std_mean": float(policy.log_std.detach().mean())}
        self.event("update", **record)
        return record

    # ------------------------------------------------------ checkpoints

    def save_checkpoint(self, path: Path, policy, optimizer, torch_generator, rngs: dict[str, np.random.Generator],
                        extra: dict[str, Any]) -> dict[str, Any]:
        import torch
        state = {
            "runner_version": RUNNER_VERSION, "identity": self.fixture.identity, "policy": policy.state_dict(),
            "optimizer": optimizer.state_dict(), "torch_generator": torch_generator.get_state(), "torch_rng": torch.get_rng_state(),
            "numpy_rngs": {k: v.bit_generator.state for k, v in rngs.items()},
            "counters": {**self.counter_snapshot(), "cumulative_wall_seconds": extra.get("cumulative_wall_seconds", self.elapsed())},
            "normalization": {"running_normalizer": None, "fixed_scales": True},
            "generator_identity": self.fixture.make_generator().identity(), "extra": extra,
        }
        temporary = path.with_name(path.name + ".tmp")
        with temporary.open("wb") as stream:
            torch.save(state, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        return {"path": str(path), "sha256": sha256(path), "counters": state["counters"]}

    def load_checkpoint(self, path: Path, policy, optimizer, torch_generator, rngs: dict[str, np.random.Generator],
                        mode: str = "exact-resume", apply_counters: bool = True,
                        expected_contract: dict[str, Any] | None = None,
                        repair_migration: dict[str, Any] | None = None,
                        previous_manifest_sha256: str | None = None) -> dict[str, Any]:
        import torch
        state = torch.load(path, weights_only=False)
        if repair_migration is not None:
            if mode != "exact-resume" or previous_manifest_sha256 is None:
                raise RuntimeError("repair migration is valid only for exact-resume with prior manifest identity")
            validate_repair_migration(repair_migration, state, self.fixture, sha256(path), previous_manifest_sha256)
        validate_checkpoint_state(state, self.fixture, mode, expected_contract, allow_repair_transition=repair_migration is not None)
        policy.load_state_dict(state["policy"])
        if mode == "exact-resume":
            optimizer.load_state_dict(state["optimizer"])
            torch_generator.set_state(state["torch_generator"])
            torch.set_rng_state(state["torch_rng"])
            for k, v in state["numpy_rngs"].items():
                rngs[k].bit_generator.state = v
            c = state["counters"]
            if apply_counters:
                counter_fields = {
                    "physics_steps": "total_simulated_physics_steps", "training_physics_steps": "training_simulated_physics_steps",
                    "training_usable_transitions": "training_usable_optimizer_transitions", "training_warmup_steps": "training_warmup_physics_steps",
                    "training_unusable_steps": "training_unusable_physics_steps", "evaluation_physics_steps": "evaluation_physics_steps",
                    "diagnostic_physics_steps": "diagnostic_physics_steps", "policy_decisions": "policy_decisions",
                    "updates": "updates", "episodes": "episodes", "worker_resets": "worker_resets",
                    "worker_reset_seconds": "worker_reset_seconds",
                }
                for attribute, key in counter_fields.items():
                    if key not in c:
                        raise RuntimeError(f"checkpoint lacks cumulative counter {key}")
                    setattr(self, attribute, c[key])
        return state

    # ----------------------------------------------------------- stages

    def selection_cases(self) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        torso = int(self.manifest["landmarks"][TORSO_LANDMARK])
        return resolved_case_documents(self.fixture.spec["case_generation"], torso)

    def evaluate(self, label: str, controller: str, cases: list[dict[str, Any]], policy=None, journal: bool = False) -> dict[str, Any]:
        jobs = [{"controller": controller, "case": case, "stochastic": False, "journal": journal,
                 "compact": not journal, "label": f"{label}-{case.get('id', i)}", "seed_label": label,
                 "purpose": "evaluation", "stop_on_terminal": False}
                for i, case in enumerate(cases)]
        episode_results = self.parallel_episodes(policy, jobs)
        results = []
        for i, (case, result) in enumerate(zip(cases, episode_results)):
            entry = {"case": case, "metrics": result["metrics"]}
            if journal:
                entry["journal"] = self.write_journal(f"{label}-{case.get('id', i)}", result)
            results.append(entry)
        metrics = [r["metrics"] for r in results]
        summary = {
            "label": label, "controller": controller, "cases": len(results),
            "falls": int(sum(m["fall_step"] is not None for m in metrics)),
            "mean_commanded_velocity_rmse_walk": float(np.nanmean([m["commanded_velocity_rmse_walk"] for m in metrics])),
            "mean_joint_acceleration_rms": float(np.nanmean([m["joint_acceleration_rms"] for m in metrics])),
            "mean_support_slip_sum": float(np.mean([m["support_slip_sum"] for m in metrics])),
            "mean_reward": float(np.mean([m["reward_sum"] for m in metrics])),
            "mean_achieved_speed": float(np.mean([m["mean_achieved_speed"] for m in metrics])),
            "mean_achieved_along_command_speed_walk": float(np.mean([m["mean_achieved_along_command_speed_walk"] for m in metrics])),
            "violation_counts": {k: int(sum(bool(m["violations"][k]) for m in metrics if m["violations"][k] is not None)) for k in metrics[0]["violations"]},
            "per_case": results,
        }
        self.event("evaluation", label=label, controller=controller, falls=summary["falls"], speed_rmse=summary["mean_commanded_velocity_rmse_walk"],
                   achieved_along_speed=summary["mean_achieved_along_command_speed_walk"],
                   mean_reward=summary["mean_reward"])
        return summary

    def startup_probe(self, policy, horizon: int, count: int, command_speed: float,
                      require_positive_signed_speed: bool, stage_index: int) -> dict[str, Any]:
        """Repeat a deterministic short-horizon completion probe without replacing full evaluations."""
        cases = [{"perturbed": False, "impulse": None, "id": f"startup-stage-{stage_index}-probe-{i}",
                  "episode_steps": horizon, "command_speed": command_speed} for i in range(count)]
        jobs = [{"controller": self.fixture.policy_controller, "case": case, "stochastic": False, "journal": False,
                 "compact": True, "label": case["id"], "seed_label": f"startup-stage-{stage_index}",
                 "purpose": "evaluation", "stop_on_terminal": False, "steps": horizon}
                for case in cases]
        results = self.parallel_episodes(policy, jobs)
        completed = sum(result["metrics"]["terminated_at"] is None for result in results)
        signed = [result["metrics"]["mean_achieved_along_command_speed_walk"] for result in results
                  if result["metrics"]["mean_achieved_along_command_speed_walk"] is not None]
        mean_signed = float(np.mean(signed)) if signed else None
        passed = completed == count and (not require_positive_signed_speed
                                         or (mean_signed is not None and mean_signed > 0.0))
        record = {"stage_index": stage_index, "episode_steps": horizon, "probe_episodes": count,
                  "command_speed_units_per_second": command_speed,
                  "completed": completed, "mean_signed_speed": mean_signed,
                  "positive_signed_speed_required": require_positive_signed_speed, "passed": passed}
        self.event("startup_probe", **record)
        return record


# ----------------------------------------------------------------- stages


def validate_checkpoint_mode(mode: str, policy: dict[str, Any], state_exists: bool) -> None:
    """Reject every implicit fresh/transfer/resume interpretation."""
    if mode not in ("fresh", "transfer", "exact-resume"):
        raise ValueError("checkpoint mode must be fresh, transfer, or exact-resume")
    initial = policy.get("initial_mode")
    if mode in ("fresh", "transfer") and initial != mode:
        raise ValueError(f"manifest initial_mode is {initial!r}, not requested {mode!r}")
    checkpoint = policy.get("checkpoint")
    if mode == "fresh":
        if state_exists or checkpoint is not None:
            raise ValueError("fresh mode requires an empty run and no parent checkpoint")
        expected = {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"}
    elif mode == "transfer":
        if state_exists or not isinstance(checkpoint, dict) or not checkpoint.get("path") or not checkpoint.get("sha256"):
            raise ValueError("transfer mode requires an empty run and a declared checkpoint path/hash")
        expected = {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"}
    else:
        if not state_exists:
            raise ValueError("exact-resume mode requires existing run state and checkpoint")
        expected = {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"}
    treatment = policy.get("treatment", {}) if mode != "exact-resume" else policy.get("exact_resume_treatment", {})
    mismatch = [key for key, value in expected.items() if treatment.get(key) != value]
    if mismatch:
        raise ValueError(f"{mode} checkpoint treatment mismatch: " + ", ".join(mismatch))


def verify_forward_review(review_binding: dict[str, Any], manifest_dir: Path, fixture: Fixture) -> dict[str, Any]:
    path = Path(review_binding.get("path", ""))
    if not path.is_absolute():
        path = (manifest_dir / path).resolve()
    if not path.is_file() or sha256(path) != review_binding.get("sha256"):
        raise ValueError("forward reference review path/hash mismatch")
    review = json.loads(path.read_text(encoding="utf-8"))
    expected = {
        "generator_source_sha256": fixture.identity["generator_source_sha256"],
        "library_file_sha256": sha256(fixture.library_path),
    }
    if any(review.get(key) != value for key, value in expected.items()):
        raise ValueError("forward reference review generator/library identity mismatch")
    if review.get("generator", {}).get("library_sha256") != fixture.identity["library_sha256"]:
        raise ValueError("forward reference review internal library identity mismatch")
    generator_identity = review.get("generator", {})
    if (generator_identity.get("generator_version") != GENERATOR_VERSION
            or float(generator_identity.get("bound_speed", float("nan"))) != fixture.bound_speed
            or float(generator_identity.get("dt", float("nan"))) != fixture.dt):
        raise ValueError("forward reference review generator context mismatch")
    scope = review.get("reviewed_scope", {})
    if scope.get("commands") != ["idle", "start", "forward", "stop"] or scope.get("speeds_units_per_second") != [45.0, 60.0]:
        raise ValueError("forward reference review coverage mismatch")
    if review.get("review_status") != "reviewed_for_forward_reference_contract" or not review.get("numeric_contract_passed"):
        raise ValueError("forward reference review is incomplete")
    return {"path": str(path), "sha256": sha256(path), "review_status": review["review_status"]}


def training_run_contract(run_manifest: dict[str, Any], cases_doc: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": run_manifest["run_id"],
        "selection_manifest_sha256": sha256_json(cases_doc["selection"]),
        "holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
        "curriculum_sha256": sha256_json({
            "command_speed_schedule_units_per_second": run_manifest["stage_and_command_scope"].get(
                "command_speed_schedule_units_per_second", []),
            "startup_horizon_curriculum": run_manifest["stage_and_command_scope"].get(
                "startup_horizon_curriculum"),
        }),
        "rollout_settings_sha256": sha256_json(run_manifest["resolved_rollout_batch_and_optimizer_settings"]),
        "evaluation_schedule_sha256": sha256_json(run_manifest["selection_evaluation_schedule"]),
        "checkpoint_schedule_sha256": sha256_json(run_manifest["checkpoint_interval_and_retention"]),
        "control_contract": run_manifest["stage_and_command_scope"].get("control_contract"),
        "limits_sha256": sha256_json({key: run_manifest[key] for key in (
            "target_usable_optimizer_transitions", "maximum_total_physics_steps",
            "maximum_cumulative_wall_seconds", "maximum_storage_bytes", "stop_and_continuation_rules")}),
    }


def load_training_manifest(args: argparse.Namespace, require_authorization: bool) -> tuple[Path, dict[str, Any], Fixture, dict[str, Any], bool]:
    """Validate every file-backed manifest identity without starting physics."""
    manifest_path = Path(args.manifest).resolve()
    run_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    spec_path = Path(args.source_root).resolve() / "scripts/ragdoll-learning/pilot_spec.json"
    spec_doc = json.loads(spec_path.read_text(encoding="utf-8"))
    required = list(spec_doc["training_run_manifest"]["required_fields"])
    missing = [key for key in required if key not in run_manifest]
    if missing:
        raise ValueError("training_run_manifest missing fields: " + ", ".join(missing))
    authorization = run_manifest["compute_allocation_and_authorization_reference"]
    authorized = bool(authorization and str(authorization).strip().lower() not in ("pending", "required", "none"))
    if require_authorization and not authorized:
        raise ValueError("training_run_manifest has no compute allocation authorization reference")
    args.workers = int(run_manifest["worker_count"])
    fixture = Fixture(args)
    if fixture.identity["git"]["dirty_files"]:
        raise ValueError("training source worktree is dirty: " + ", ".join(fixture.identity["git"]["dirty_files"]))
    bindings = run_manifest["specification_and_source_hashes"]
    declared_control = run_manifest["stage_and_command_scope"].get("control_contract")
    if declared_control != fixture.identity["control_contract"] and not (declared_control is None and fixture.control_mode == "direct"):
        raise ValueError("training manifest control contract mismatch; explicitly select the declared --control-mode")
    for key in ("target_usable_optimizer_transitions", "maximum_total_physics_steps", "maximum_cumulative_wall_seconds", "maximum_storage_bytes"):
        diagnostic_zero = key == "target_usable_optimizer_transitions" and run_manifest["stage_and_command_scope"].get("stage") in ("teacher_bootstrap", "teacher_ablation") and run_manifest[key] == 0
        if not diagnostic_zero and (not isinstance(run_manifest[key], (int, float)) or not math.isfinite(run_manifest[key]) or run_manifest[key] <= 0):
            raise ValueError(f"invalid positive allocation limit {key}")
    for parent, key in (("checkpoint_interval_and_retention", "interval_updates"),
                        ("checkpoint_interval_and_retention", "retain_milestone_every_updates"),
                        ("selection_evaluation_schedule", "interval_updates")):
        if int(run_manifest[parent].get(key, 0)) < 1:
            raise ValueError(f"{parent}.{key} must be positive")
    expected = {"specification_id": fixture.identity["specification_id"], "spec_sha256": fixture.identity["spec_sha256"],
                "source_commit": fixture.identity["git"]["commit"], "engine_binary_sha256": fixture.identity["executable_sha256"],
                "runner_version": RUNNER_VERSION}
    mismatched = [key for key, value in expected.items() if bindings.get(key) != value]
    if mismatched:
        raise ValueError("training_run_manifest identity mismatch: " + ", ".join(mismatched))
    body = run_manifest["body_profile_and_motion_generator_hashes"]
    body_expected = {"context_sha256": fixture.identity["context_sha256"], "profile_sha256": fixture.identity["profile_sha256"],
                     "library_sha256": fixture.identity["library_sha256"], "generator_version": GENERATOR_VERSION,
                     "generator_source_sha256": fixture.identity["generator_source_sha256"]}
    body_mismatch = [key for key, value in body_expected.items() if body.get(key) != value]
    if body_mismatch:
        raise ValueError("training_run_manifest body/generator identity mismatch: " + ", ".join(body_mismatch))
    coverage_path = fixture.library_path.parent / "content-coverage.json"
    if not coverage_path.is_file() or sha256(coverage_path) != run_manifest["content_coverage_manifest_sha256"]:
        raise ValueError("training_run_manifest content coverage identity mismatch")
    rollout = run_manifest["resolved_rollout_batch_and_optimizer_settings"]
    if rollout.get("optimizer") != fixture.spec["optimizer"] or int(rollout.get("episode_physics_steps", -1)) != fixture.episode_steps:
        raise ValueError("training_run_manifest rollout/optimizer contract mismatch")
    if not isinstance(rollout.get("training_perturbations_enabled"), bool):
        raise ValueError("training_run_manifest must explicitly select training perturbation exposure")
    if int(rollout.get("episodes_per_batch", 0)) < 1:
        raise ValueError("episodes_per_batch must be positive")
    if fixture.control_mode == "production-residual":
        target = int(run_manifest["target_usable_optimizer_transitions"])
        schedule = run_manifest["stage_and_command_scope"].get("command_speed_schedule_units_per_second", [])
        boundaries = [int(row["through_usable_optimizer_transitions"]) for row in schedule]
        milestones = run_manifest["checkpoint_interval_and_retention"].get("evaluation_milestones_usable_transitions", [])
        if not boundaries or boundaries != sorted(set(boundaries)) or boundaries[-1] != target:
            raise ValueError("production residual curriculum must increase to the sample target")
        if not milestones or milestones != sorted(set(milestones)) or milestones[-1] != target or milestones[0] < 1:
            raise ValueError("production residual evaluation milestones must increase to the sample target")
        if any(float(row["speed"]) not in (45.0, 60.0) for row in schedule):
            raise ValueError("production residual commands lack reviewed speed coverage")
    startup_stages = validate_startup_horizon_curriculum(
        run_manifest["stage_and_command_scope"].get("startup_horizon_curriculum"), fixture.episode_steps)
    if startup_stages:
        maximum_curriculum_samples = sum(int(stage["maximum_stage_usable_transitions"])
                                         for stage in startup_stages)
        if maximum_curriculum_samples > int(run_manifest["target_usable_optimizer_transitions"]):
            raise ValueError("startup horizon maximum stage budgets exceed the usable-transition target")
    if run_manifest["stage_and_command_scope"].get("stage") == "teacher_bootstrap":
        validate_teacher_bootstrap_config(run_manifest["stage_and_command_scope"].get("teacher_bootstrap"))
    if run_manifest["stage_and_command_scope"].get("stage") == "teacher_ablation":
        validate_teacher_ablation_config(run_manifest["stage_and_command_scope"].get("teacher_ablation"))
        teacher_binding = run_manifest.get("teacher_bootstrap_result", {})
        teacher_path = Path(teacher_binding.get("path", ""))
        if not teacher_path.is_absolute():
            teacher_path = (manifest_path.parent / teacher_path).resolve()
        if not teacher_path.is_file() or sha256(teacher_path) != teacher_binding.get("sha256"):
            raise ValueError("teacher ablation bootstrap-result path/hash mismatch")
        teacher_result = json.loads(teacher_path.read_text(encoding="utf-8"))
        if (teacher_result.get("schema") != "play01_teacher_bootstrap_result_v1"
                or teacher_result.get("qualification_holdout_opened") is not False
                or not teacher_result.get("teacher_physical", {}).get("gate_passed")
                or not teacher_result.get("target_representability", {}).get("gate_passed")):
            raise ValueError("teacher ablation bootstrap result failed its source gates")
    review_identity = verify_forward_review(run_manifest["forward_reference_review"], manifest_path.parent, fixture)
    if fixture.control_mode == "production-residual":
        binding = run_manifest.get("integration_evidence", {})
        path = Path(binding.get("path", ""))
        if not path.is_file() or sha256(path) != binding.get("sha256"):
            raise ValueError("production residual integration evidence path/hash mismatch")
        evidence = json.loads(path.read_text(encoding="utf-8"))
        for key in ("implementation_sha256", "executable_sha256", "context_sha256", "profile_sha256",
                    "library_sha256", "generator_source_sha256", "spec_sha256", "control_contract",
                    "floor", "command_speed", "bound_speed"):
            if evidence.get("identity", {}).get(key) != fixture.identity[key]:
                raise ValueError(f"production residual integration identity mismatch: {key}")
        if not evidence.get("ready_for_allocation") or evidence.get("qualification_holdout_opened") is not False:
            raise ValueError("production residual integration evidence is incomplete")
    return manifest_path, run_manifest, fixture, review_identity, authorized


def stage_smoke(args: argparse.Namespace) -> int:
    import torch
    fixture = Fixture(args)
    if not args.reference_review:
        raise SystemExit("smoke requires --reference-review bound to the reviewed 45/60 forward previews")
    review_path = Path(args.reference_review).resolve()
    try:
        review_identity = verify_forward_review({"path": str(review_path), "sha256": sha256(review_path)}, review_path.parent, fixture)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
    out = Path(args.out).resolve()
    if out.exists() and any(out.iterdir()) and not args.allow_existing:
        raise SystemExit("output directory must be empty")
    runner = Runner(fixture, out)
    spec = fixture.spec
    budget = spec["budget"]
    max_steps = int(args.max_steps if args.max_steps else budget["maximum_total_physics_steps"])
    max_seconds = float(args.max_seconds if args.max_seconds else budget["maximum_wall_seconds"])
    reserve = int(budget["reserved_evaluation_physics_steps"])
    torch.set_num_threads(int(spec["optimizer"]["torch_threads"]))
    manifest: dict[str, Any] = {"run_id": out.name, "stage": "integration_smoke", "started_utc": utc_now(), "identity": fixture.identity,
                                "caps": {"maximum_total_physics_steps": max_steps, "maximum_wall_seconds": max_seconds, "reserved_evaluation_physics_steps": reserve},
                                 "workers": fixture.workers, "command_speed": fixture.command_speed, "bound_speed": fixture.bound_speed,
                                 "training_perturbations_enabled": False,
                                "forward_reference_review": review_identity,
                                "episode_protocol": {"steps": fixture.episode_steps, "warmup": fixture.warmup_steps, "start_blend": fixture.start_blend_steps,
                                                     "walk": fixture.walk_steps, "stop_blend": fixture.stop_blend_steps, "commands": fixture.commands()}}
    (out / "run-manifest.json").write_text(json.dumps(manifest, indent=1, allow_nan=False, default=str) + "\n", encoding="utf-8")
    shutil.copy(fixture.library_path.parent / "content-coverage.json", out / "content-coverage.json") if (fixture.library_path.parent / "content-coverage.json").exists() else None
    runner.event("start", stage="integration_smoke", max_steps=max_steps, max_seconds=max_seconds)
    stop_reason = None
    curves: list[dict[str, Any]] = []
    physical: dict[str, Any] = {}
    readiness = None
    checkpoint_control = None
    throughput: dict[str, Any] = {}
    try:
        readiness = runner.readiness()
        if not readiness["passed"]:
            stop_reason = "readiness failed: " + ", ".join(k for k, v in readiness["requirements"].items() if not v["passed"])
            raise StopIteration
        b = runner.builder
        torch.manual_seed(int(spec["optimizer"]["seed"]))
        policy = build_policy(b.dimension, b.action_count, spec)
        optimizer = runner.make_optimizer(policy)
        torch_generator = torch.Generator().manual_seed(int(spec["optimizer"]["seed"]))
        rngs = {"training": np.random.Generator(np.random.PCG64(int(spec["case_generation"]["training_seed"])))}
        selection, cases_doc = runner.selection_cases()
        torso = int(runner.manifest["landmarks"][TORSO_LANDMARK])
        control = {"perturbed": False, "impulse": None, "id": "unperturbed-control"}
        eval_cases = selection + [control]
        training_cases: list[dict[str, Any]] = []

        # Baselines on the selection controls (production walker and zero-residual tracker).
        physical["production_walker"] = runner.evaluate("baseline-production", "reference", eval_cases)
        physical["zero_residual_tracker"] = runner.evaluate("baseline-zero-residual", fixture.policy_controller, eval_cases, policy=None)
        physical["initial_policy"] = runner.evaluate("policy-initial", fixture.policy_controller, eval_cases, policy=policy)

        # Bounded optimizer updates with throughput measurement.
        episodes_per_batch = int(args.episodes_per_batch)
        batch_steps = episodes_per_batch * fixture.episode_steps
        rollout_seconds, update_seconds, batch_count = 0.0, 0.0, 0
        first_terminal_steps: list[int] = []
        eval_steps = len(eval_cases) * fixture.episode_steps
        smoke_updates_target = int(args.smoke_updates) if args.smoke_updates else None
        while True:
            remaining_steps = max_steps - runner.physics_steps
            remaining_seconds = max_seconds - runner.elapsed()
            needed = batch_steps + reserve + eval_steps + 8 * batch_steps  # batch + reserve + final evaluation + checkpoint control
            per_second = (runner.physics_steps / max(runner.elapsed(), 1e-6))
            if remaining_steps < needed or remaining_seconds < (needed / max(per_second, 1.0)) + 60:
                stop_reason = "integration cap reached (reserve kept for controls)"
                break
            if smoke_updates_target is not None and batch_count >= smoke_updates_target:
                stop_reason = "requested smoke update count reached"
                break
            batch_cases = []
            for i in range(episodes_per_batch):
                case = make_case(rngs["training"], perturbed=False, torso=torso)
                case["id"] = f"train-{len(training_cases) + i}"
                batch_cases.append(case)
            training_cases.extend(batch_cases)
            t0 = time.monotonic()
            batch = runner.rollout_batch(policy, batch_cases, torch_generator)
            t1 = time.monotonic()
            record = runner.update(policy, optimizer, batch)
            t2 = time.monotonic()
            rollout_seconds += t1 - t0
            update_seconds += t2 - t1
            batch_count += 1
            first_terminal_steps.extend(int(r["metrics"]["terminated_at"]) for r in batch["results"] if r["metrics"]["terminated_at"] is not None)
            curves.append({"update": runner.updates, "counters": runner.counter_snapshot(), "elapsed": runner.elapsed(), **record,
                           "rollout_seconds": t1 - t0, "update_seconds": t2 - t1})
        throughput = {
            "counter_definitions": {
                "total_simulated_physics_steps": "every worker action advanced, including diagnostics, evaluation and training warmup",
                "training_usable_optimizer_transitions": "valid stochastic-policy transitions at or after the 60-step warmup, including the first terminal transition",
                "training_warmup_physics_steps": "training steps before the command starts; excluded from optimizer samples",
                "evaluation_physics_steps": "full-length selection/control trajectories, including time after a terminal diagnostic state",
                "worker_reset_seconds": "wall time to terminate and replace a worker after an early-ended training episode"},
            "batches": batch_count, "episodes_per_batch": episodes_per_batch, "declared_batch_physics_step_ceiling": batch_steps,
            "measured_counters": runner.counter_snapshot(),
            "rollout_physics_steps_per_second": runner.training_physics_steps / rollout_seconds if rollout_seconds else None,
            "rollout_usable_transitions_per_second": runner.training_usable_transitions / rollout_seconds if rollout_seconds else None,
            "update_seconds_mean": update_seconds / batch_count if batch_count else None,
            "complete_update_usable_transitions_per_second": runner.training_usable_transitions / (rollout_seconds + update_seconds) if (rollout_seconds + update_seconds) else None,
            "workers": fixture.workers, "policy_decisions": runner.policy_decisions,
            "first_terminal_timing": {"count": len(first_terminal_steps), "minimum_step": min(first_terminal_steps) if first_terminal_steps else None,
                                      "mean_step": float(np.mean(first_terminal_steps)) if first_terminal_steps else None,
                                      "maximum_step": max(first_terminal_steps) if first_terminal_steps else None},
            "overall_steps_per_second_including_controls": runner.physics_steps / max(runner.elapsed(), 1e-6),
        }
        runner.event("throughput", **{k: v for k, v in throughput.items() if isinstance(v, (int, float))})

        # Final policy evaluation on the selection controls.
        physical["smoke_policy"] = runner.evaluate("policy-smoke", fixture.policy_controller, eval_cases, policy=policy, journal=True)
        checkpoint = runner.save_checkpoint(out / "checkpoint-smoke.pt", policy, optimizer, torch_generator, rngs,
                                            {"cumulative_wall_seconds": runner.elapsed(), "training_cases": [case_hash(c) for c in training_cases]})

        # checkpoint_resume_identity: uninterrupted control vs save/restart/resume at an update boundary.
        checkpoint_control = checkpoint_resume_control(runner, fixture, spec, torso, episodes_per_batch, out)
        storage_bytes = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
        throughput["storage_bytes_total"] = storage_bytes
        throughput["storage_bytes_per_physics_step"] = storage_bytes / max(runner.physics_steps, 1)
        (out / "resolved-cases.json").write_text(json.dumps({**cases_doc, "training_cases": training_cases}, indent=1, allow_nan=False) + "\n", encoding="utf-8")
        (out / "weights-and-resume-state-hashes.json").write_text(json.dumps({"smoke_checkpoint": checkpoint, "checkpoint_control": checkpoint_control}, indent=1, default=str) + "\n", encoding="utf-8")
        if stop_reason is None:
            stop_reason = "complete"
    except StopIteration:
        pass
    except Exception as error:  # record and re-raise so the failure is visible
        stop_reason = f"{type(error).__name__}: {error}"
        runner.event("failure", error=stop_reason)
        write_smoke_artifacts(runner, out, fixture, readiness, physical, curves, throughput, checkpoint_control, stop_reason)
        runner.close()
        raise
    write_smoke_artifacts(runner, out, fixture, readiness, physical, curves, throughput, checkpoint_control, stop_reason)
    runner.event("stopped", reason=stop_reason)
    runner.close()
    return 0


def checkpoint_resume_control(runner: Runner, fixture: Fixture, spec: dict[str, Any], torso: int, episodes_per_batch: int, out: Path) -> dict[str, Any]:
    """Uninterrupted K updates versus K/2, checkpoint, fresh instance, resume K/2."""
    import torch
    b = runner.builder
    k = 4

    def fresh():
        torch.manual_seed(int(spec["optimizer"]["seed"]) + 7)
        policy = build_policy(b.dimension, b.action_count, spec)
        optimizer = runner.make_optimizer(policy)
        generator = torch.Generator().manual_seed(int(spec["optimizer"]["seed"]) + 7)
        rngs = {"training": np.random.Generator(np.random.PCG64(int(spec["case_generation"]["training_seed"]) + 7))}
        return policy, optimizer, generator, rngs

    def batch_cases(rngs, index):
        cases = []
        for i in range(episodes_per_batch):
            case = make_case(rngs["training"], perturbed=False, torso=torso)
            case["id"] = f"control-{index}-{i}"
            cases.append(case)
        return cases

    def parameter_hash(policy):
        return hashlib.sha256(b"".join(p.detach().numpy().tobytes() for p in policy.parameters())).hexdigest()

    # Uninterrupted control.
    policy, optimizer, generator, rngs = fresh()
    hashes_a = []
    for index in range(k):
        batch = runner.rollout_batch(policy, batch_cases(rngs, index), generator, label="resume-control")
        runner.update(policy, optimizer, batch)
        hashes_a.append(parameter_hash(policy))
    # Interrupted: half, checkpoint, new objects, resume.
    policy, optimizer, generator, rngs = fresh()
    hashes_b = []
    for index in range(k // 2):
        batch = runner.rollout_batch(policy, batch_cases(rngs, index), generator, label="resume-control")
        runner.update(policy, optimizer, batch)
        hashes_b.append(parameter_hash(policy))
    saved_counters = runner.counter_snapshot()
    checkpoint_path = out / "checkpoint-resume-control.pt"
    saved = runner.save_checkpoint(checkpoint_path, policy, optimizer, generator, rngs, {"resume_index": k // 2})
    policy2, optimizer2, generator2, rngs2 = fresh()
    # The live smoke accounting keeps running; the restored counters are
    # compared against the saved ones instead of being applied here.
    state = runner.load_checkpoint(checkpoint_path, policy2, optimizer2, generator2, rngs2, apply_counters=False)
    restored_counters = {k: state["counters"][k] for k in saved_counters}
    for index in range(k // 2, k):
        batch = runner.rollout_batch(policy2, batch_cases(rngs2, index), generator2, label="resume-control")
        runner.update(policy2, optimizer2, batch)
        hashes_b.append(parameter_hash(policy2))
    return {"passed": hashes_a == hashes_b and saved_counters == restored_counters, "uninterrupted_parameter_hashes": hashes_a,
            "resumed_parameter_hashes": hashes_b, "checkpoint": saved, "counters_restored": saved_counters == restored_counters,
            "boundary": "optimizer update boundary; mid-episode physics state is not restored"}


def write_smoke_artifacts(runner: Runner, out: Path, fixture: Fixture, readiness, physical, curves, throughput, checkpoint_control, stop_reason) -> None:
    (out / "learning-curves.json").write_text(json.dumps({"updates": curves, "interpretation": "integration observations only; no learning verdict"}, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    (out / "physical-metrics.json").write_text(json.dumps(physical, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    motion_errors = {}
    for name, summary in physical.items():
        if "per_case" not in summary:
            continue
        values = [c["metrics"].get("pose_error_mean") for c in summary["per_case"]]
        finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
        motion_errors[name] = {"mean_pose_error": float(np.mean(finite)) if finite else None}
    (out / "motion-errors.json").write_text(json.dumps(motion_errors, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    if fixture.control_mode == "production-residual":
        proposal = {"schema": "play01_production_residual_measurement_v1", "measured": throughput,
                    "integration_used_steps": runner.physics_steps, "integration_elapsed_seconds": runner.elapsed(),
                    "launch_status": "prepare_new_measured_allocation", "recommended": None}
        initial = physical.get("initial_policy", {})
        integration_ready = (bool(readiness and readiness["passed"]) and bool(checkpoint_control and checkpoint_control["passed"])
                             and bool(curves) and initial.get("falls", 17) <= 3
                             and initial.get("mean_achieved_along_command_speed_walk", 0) > 0
                             and any(row.get("mean_achieved_along_speed", 0) is not None and row["mean_achieved_along_speed"] > 0 for row in curves)
                             and runner.physics_steps <= int(fixture.spec["budget"]["maximum_total_physics_steps"])
                             and runner.elapsed() < float(fixture.spec["budget"]["maximum_wall_seconds"]))
        atomic_json(out / "integration-evidence.json", {
            "schema": "play01_production_residual_integration_v1", "identity": fixture.identity,
            "ready_for_allocation": integration_ready, "qualification_holdout_opened": False,
            "throughput": throughput, "source_smoke_path": str(out),
            "readiness": {key: val["passed"] for key, val in readiness["requirements"].items()} if readiness else {},
            "checkpoint_resume_control": checkpoint_control, "finite_optimizer_updates": len(curves),
            "initial_physical": initial, "final_physical": physical.get("smoke_policy"),
            "forward_reference_review": json.loads((out / "run-manifest.json").read_text(encoding="utf-8"))["forward_reference_review"],
            "total_physics_steps": runner.physics_steps, "wall_seconds": runner.elapsed()})
    else:
        proposal = propose_allocation(fixture, throughput, runner.physics_steps, runner.elapsed(), out)
    (out / "throughput-and-budget-proposal.json").write_text(json.dumps(proposal, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    lines = [f"PLAY-01 integration smoke ({RUNNER_VERSION}) — {utc_now()}", f"stop reason: {stop_reason}",
             f"counters: {json.dumps(runner.counter_snapshot(), sort_keys=True)}  elapsed: {runner.elapsed():.0f} s",
             f"source: {fixture.identity['git']['commit']} ({fixture.identity['git']['branch']})  exe: {fixture.identity['executable_sha256'][:16]}",
             f"library: {fixture.identity['library_sha256'][:16]}  spec: {fixture.identity['spec_sha256'][:16]}  command speed: {fixture.command_speed:.2f}  bound speed: {fixture.bound_speed:.2f}", ""]
    if readiness:
        lines.append("readiness: " + ", ".join(f"{k}={'pass' if v['passed'] else 'FAIL'}" for k, v in readiness["requirements"].items()))
    if throughput:
        lines.append(f"throughput: rollout {throughput.get('rollout_usable_transitions_per_second')} usable transitions/s, complete-update {throughput.get('complete_update_usable_transitions_per_second')} usable transitions/s, update {throughput.get('update_seconds_mean')} s, workers {throughput.get('workers')}")
        lines.append(f"first terminal timing: {throughput.get('first_terminal_timing')}")
    if checkpoint_control:
        lines.append(f"checkpoint/resume identity: {'pass' if checkpoint_control['passed'] else 'FAIL'}")
    for name, summary in physical.items():
        lines.append(f"{name}: falls {summary['falls']}/{summary['cases']}, speed RMSE {summary['mean_commanded_velocity_rmse_walk']:.2f}, "
                     f"joint accel RMS {summary['mean_joint_acceleration_rms']:.2f}, slip {summary['mean_support_slip_sum']:.2f}, reward {summary['mean_reward']:.2f}, "
                     f"achieved speed {summary['mean_achieved_speed']:.2f}, violations {summary['violation_counts']}")
    if curves:
        lines.append(f"learning curve: first reward {curves[0]['mean_reward']:.2f} -> last {curves[-1]['mean_reward']:.2f} over {len(curves)} updates (observation only)")
    lines.append("")
    lines.append("This smoke records no tracker verdict. Absence of improvement at this cap is inconclusive by specification.")
    (out / "SUMMARY.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def propose_allocation(fixture: Fixture, throughput: dict[str, Any], used_steps: int, elapsed: float,
                       evidence_dir: Path | None = None) -> dict[str, Any]:
    usable_sps = throughput.get("complete_update_usable_transitions_per_second") or 0.0
    measured_workers = int(throughput.get("workers") or fixture.workers)
    per_worker = usable_sps / max(measured_workers, 1)
    measured_counters = throughput.get("measured_counters", {})
    training_physics = int(measured_counters.get("training_simulated_physics_steps", 0))
    usable = int(measured_counters.get("training_usable_optimizer_transitions", 0))
    physics_per_usable = training_physics / max(usable, 1)
    maximum_workers = int(fixture.spec["optimizer"]["maximum_persistent_cpu_workers"])
    worker_options = sorted(set(range(1, maximum_workers + 1)))
    candidates = []
    for target in (100_000, 175_000, 250_000):
        for w in worker_options:
            # Complete-update throughput already includes optimizer work. Keep a
            # further ten percent reserve for scheduled evaluation/checkpoint I/O.
            projected = per_worker * w * 0.9
            hours = target / max(projected, 1e-6) / 3600
            candidates.append({"target_usable_optimizer_transitions": target, "workers": w,
                               "projected_usable_transitions_per_second": projected, "projected_hours": hours,
                               "projected_training_physics_steps": math.ceil(target * physics_per_usable),
                               "checkpoint_interval_updates": 25})
    recommended_workers = maximum_workers
    projected_sps = per_worker * recommended_workers * 0.9
    target_usable = 250_000
    recommended_hours = target_usable / max(projected_sps, 1e-6) / 3600
    evidence_dir = Path(evidence_dir).resolve() if evidence_dir is not None else None
    cases, measurement_manifest = {}, {}
    measurement_identity = fixture.identity
    parent_checkpoint = None
    if evidence_dir is not None:
        cases_path = evidence_dir / "resolved-cases.json"
        manifest_path = evidence_dir / "run-manifest.json"
        weights_path = evidence_dir / "weights-and-resume-state-hashes.json"
        if cases_path.exists():
            cases = json.loads(cases_path.read_text(encoding="utf-8"))
        if manifest_path.exists():
            measurement_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            measurement_identity = measurement_manifest["identity"]
        if weights_path.exists():
            parent_checkpoint = json.loads(weights_path.read_text(encoding="utf-8"))["smoke_checkpoint"]
    coverage_path = fixture.library_path.parent / "content-coverage.json"
    selection_hash = sha256_json(cases.get("selection", [])) if cases.get("selection") else None
    holdout_hash = cases.get("holdout_manifest_sha256")
    evaluation_reserve = 320_000
    projected_training_physics = math.ceil(target_usable * physics_per_usable)
    total_physics_cap = max(800_000, projected_training_physics + evaluation_reserve)
    review = measurement_manifest.get("forward_reference_review")
    startup_curriculum = {
        "probe_episodes": 3,
        "probe_interval_usable_transitions": 10_000,
        "stages": [
            {"episode_steps": 120, "minimum_stage_usable_transitions": 10_000,
             "maximum_stage_usable_transitions": 30_000, "require_positive_signed_speed": False},
            {"episode_steps": 132, "minimum_stage_usable_transitions": 10_000,
             "maximum_stage_usable_transitions": 30_000, "require_positive_signed_speed": False},
            {"episode_steps": 144, "minimum_stage_usable_transitions": 10_000,
             "maximum_stage_usable_transitions": 40_000, "require_positive_signed_speed": False},
            {"episode_steps": 180, "minimum_stage_usable_transitions": 15_000,
             "maximum_stage_usable_transitions": 40_000, "require_positive_signed_speed": True},
            {"episode_steps": 240, "minimum_stage_usable_transitions": 15_000,
             "maximum_stage_usable_transitions": 50_000, "require_positive_signed_speed": True},
            {"episode_steps": 600, "minimum_stage_usable_transitions": 20_000,
             "maximum_stage_usable_transitions": 50_000, "require_positive_signed_speed": True},
        ],
    }
    proposed_manifest = {
        "run_id": "PLAY-01-startup-horizon-250k-proposed",
        "stage_and_command_scope": {
            "stage": "bounded_startup_horizon_experiment", "commands": ["idle", "forward", "start", "stop"],
            "command_speed_schedule_units_per_second": [
                {"through_usable_optimizer_transitions": target_usable, "speed": 45.0}],
            "startup_horizon_curriculum": startup_curriculum,
            "qualification_claim": "none; bounded startup signal and full selection evaluation only"},
        "specification_and_source_hashes": {
            "specification_id": fixture.identity["specification_id"], "spec_sha256": fixture.identity["spec_sha256"],
            "source_commit": fixture.identity["git"]["commit"], "runner_version": RUNNER_VERSION,
            "engine_binary_sha256": fixture.identity["executable_sha256"]},
        "body_profile_and_motion_generator_hashes": {
            "context_sha256": fixture.identity["context_sha256"], "profile_sha256": fixture.identity["profile_sha256"],
            "library_sha256": fixture.identity["library_sha256"], "generator_version": GENERATOR_VERSION,
            "generator_source_sha256": fixture.identity["generator_source_sha256"]},
        "content_coverage_manifest_sha256": sha256(coverage_path) if coverage_path.exists() else None,
        "forward_reference_review": review,
        "parent_checkpoint_and_transfer_policy": {
            "initial_mode": "fresh", "checkpoint": None,
            "treatment": {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"},
            "exact_resume_treatment": {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"},
            "contract": "transfer consumes only verified policy parameters; exact-resume consumes the run's hashed latest checkpoint and restores all cumulative state"},
        "compute_allocation_and_authorization_reference": "User authorized the bounded staged-horizon experiment in the Codex task on 2026-09-14",
        "measured_steps_and_updates_per_second": {
            "measurement_source_commit": measurement_identity["git"]["commit"],
            "measurement_runner_version": measurement_identity["runner_version"],
            "workers": measured_workers, "complete_update_usable_transitions_per_second": usable_sps,
            "training_physics_steps_per_usable_transition": physics_per_usable,
            "projected_allocated_usable_transitions_per_second_after_overhead": projected_sps},
        "measurement_protocol_and_overhead": {
            "integration_physics_steps": used_steps, "integration_elapsed_seconds": elapsed,
            "episodes_per_batch": throughput.get("episodes_per_batch"), "evaluation_and_checkpoint_reserve_fraction": 0.10,
            "counter_definitions": throughput.get("counter_definitions"), "measured_counters": measured_counters},
        "target_usable_optimizer_transitions": target_usable,
        "maximum_total_physics_steps": total_physics_cap,
        "maximum_cumulative_wall_seconds": 1200,
        "maximum_storage_bytes": 1_000_000_000,
        "worker_count": recommended_workers,
        "resolved_rollout_batch_and_optimizer_settings": {
            "episodes_per_batch": 6, "episode_physics_steps": fixture.episode_steps,
            "training_perturbations_enabled": False,
            "optimizer": fixture.spec["optimizer"]},
        "checkpoint_interval_and_retention": {
            "interval_updates": 25, "retain_latest": True, "retain_best": True,
            "retain_milestone_every_updates": 100,
            "evaluation_milestones_usable_transitions": [50_000, 100_000, 150_000, 200_000, 250_000]},
        "selection_evaluation_schedule": {
            "interval_updates": 250, "cases": 16, "repeated_unperturbed_control": True,
            "command_speed_units_per_second": 45.0},
        "reserved_evaluation_steps_wall_and_storage": {
            "physics_steps": evaluation_reserve, "wall_seconds": math.ceil(evaluation_reserve / max(projected_sps, 1e-6)),
            "storage_bytes": 100_000_000},
        "curriculum_and_case_manifest_hashes": {
            "selection_manifest_sha256": selection_hash,
            "training_case_rng": "PCG64 state in checkpoint; unperturbed staged-horizon startup experiment",
            "command_speed_schedule": "250k usable optimizer transitions maximum at 45 u/s; stages 120/132/144/180/240/600 steps; 60 u/s remains reviewed but outside this experiment"},
        "fresh_holdout_manifest_sha256": holdout_hash,
        "budget_rationale": "This is a new bounded 250k-usable-transition staged-horizon experiment, not an extension of the unsuccessful 1M run or a reinterpretation of the rejected 60M proposal. Measured early-terminal overhead projects %d training physics steps; %d additional physics steps are reserved for deterministic stage probes and full selection evaluations. The independent hard caps are %d total physics steps and 1200 cumulative execution seconds." % (projected_training_physics, evaluation_reserve, total_physics_cap),
        "stop_and_continuation_rules": {
            "stop": ["any control or integrity failure", "non-finite value", "physics/wall/storage cap", "evaluation reserve cannot fit"],
            "plateau_review": "stop on the first failed stage maximum or successful full-horizon positive-speed gate; save and compare to the existing setup, never auto-extend",
            "holdout": "remain unopened until a separately frozen qualification decision"},
    }
    return {
        "schema": "play01_training_allocation_proposal_v4", "measured": throughput, "integration_used_steps": used_steps, "integration_elapsed_seconds": elapsed,
        "sample_budget_rationale": "The 250k maximum bounds a staged startup test after the unsuccessful 1M configuration. Stage gates can stop substantially earlier. This experiment tests repeatable survival and positive forward motion; it is not a convergence or qualification allocation and may not be silently extended.",
        "recommended": {"target_usable_optimizer_transitions": target_usable, "maximum_total_physics_steps": total_physics_cap,
                        "projected_training_physics_steps": projected_training_physics, "reserved_evaluation_physics_steps": evaluation_reserve,
                        "workers": recommended_workers, "projected_usable_transitions_per_second": projected_sps, "projected_hours": recommended_hours,
                        "note": "uses three persistent workers, a separate 320k-physics evaluation reserve, and independent 800k-physics/1200-second hard caps"},
        "candidates": candidates,
        "required_before_launch": ["training_run_manifest with compute authorization reference", "fresh disjoint holdout manifest", "checkpoint interval and retention",
                                   "plateau/continuation rule fixed before the run"],
        "launch_status": "authorized_bounded_experiment",
        "proposed_training_run_manifest": proposed_manifest,
        "launch_command_template": f"python scripts/ragdoll-learning/play01_runner.py train --checkpoint-mode fresh --manifest <training-run-manifest.json> --out <run dir> --exe {fixture.exe} --library {fixture.library_path} --source-root {fixture.source_root}",
    }


def stage_teacher_propose(args: argparse.Namespace) -> int:
    """Freeze an authorized, bounded production-teacher bootstrap manifest."""
    fixture = Fixture(args)
    if fixture.identity["git"]["dirty_files"]:
        raise SystemExit("teacher proposal requires a clean source worktree")
    if not args.reference_review:
        raise SystemExit("teacher proposal requires --reference-review")
    review_path = Path(args.reference_review).resolve()
    review = verify_forward_review(
        {"path": str(review_path), "sha256": sha256(review_path)}, review_path.parent, fixture)
    torso = fixture.library["body_names"].index(TORSO_BODY_NAME)
    selection, cases_doc = resolved_case_documents(fixture.spec["case_generation"], torso)
    teacher_case_ids = [case["id"] for case in selection[:12]] + ["unperturbed-control"]
    coverage_path = fixture.library_path.parent / "content-coverage.json"
    manifest = {
        "run_id": "PLAY-01-production-teacher-bootstrap-v1",
        "stage_and_command_scope": {
            "stage": "teacher_bootstrap", "commands": ["idle", "forward", "start", "stop"],
            "command_speed_schedule_units_per_second": [],
            "teacher_bootstrap": {
                "source_controller": "production_walker", "teacher_case_ids": teacher_case_ids,
                "teacher_case_manifest_sha256": sha256_json(teacher_case_ids),
                "epochs": 40, "minibatch_size": 512, "learning_rate": 0.0003,
                "minimum_teacher_survival_fraction": 0.8,
                "minimum_teacher_pose_coverage_fraction": 0.8,
                "require_positive_teacher_signed_speed": True,
                "student_success_gate": "positive signed speed and fewer full-length falls than fresh zero residual",
                "target_mapping": "invert production driven parent-local motor pose into the unchanged 49-action/0.5-radian residual basis, then invert the frozen 0.8/0.2 action filter",
            },
            "qualification_claim": "none; teacher representability and direct-controller handoff only",
        },
        "specification_and_source_hashes": {
            "specification_id": fixture.identity["specification_id"], "spec_sha256": fixture.identity["spec_sha256"],
            "source_commit": fixture.identity["git"]["commit"], "runner_version": RUNNER_VERSION,
            "engine_binary_sha256": fixture.identity["executable_sha256"],
        },
        "body_profile_and_motion_generator_hashes": {
            "context_sha256": fixture.identity["context_sha256"], "profile_sha256": fixture.identity["profile_sha256"],
            "library_sha256": fixture.identity["library_sha256"], "generator_version": GENERATOR_VERSION,
            "generator_source_sha256": fixture.identity["generator_source_sha256"],
        },
        "content_coverage_manifest_sha256": sha256(coverage_path),
        "forward_reference_review": {"path": review["path"], "sha256": review["sha256"]},
        "parent_checkpoint_and_transfer_policy": {
            "initial_mode": "fresh", "checkpoint": None,
            "treatment": {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"},
            "exact_resume_treatment": {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"},
            "contract": "fresh behavior clone; emitted checkpoint may be consumed only by a later explicit policy-only transfer manifest",
        },
        "compute_allocation_and_authorization_reference": "User authorized the bounded production-teacher bootstrap in the Codex task on 2026-09-14",
        "measured_steps_and_updates_per_second": {
            "measurement_source": "PLAY-01 staged-horizon v18", "workers": 3,
            "complete_update_usable_transitions_per_second": 252.70,
            "overall_physics_steps_per_second": 760.53,
        },
        "measurement_protocol_and_overhead": {
            "teacher_collection_episodes": len(teacher_case_ids), "full_student_evaluations": 2,
            "teacher_optimizer_updates": "40 fixed supervised epochs; no PPO transitions or updates",
        },
        "target_usable_optimizer_transitions": 0,
        "maximum_total_physics_steps": 40_000,
        "maximum_cumulative_wall_seconds": 600,
        "maximum_storage_bytes": 500_000_000,
        "worker_count": 3,
        "resolved_rollout_batch_and_optimizer_settings": {
            "episodes_per_batch": 6, "episode_physics_steps": fixture.episode_steps,
            "training_perturbations_enabled": False, "optimizer": fixture.spec["optimizer"],
        },
        "checkpoint_interval_and_retention": {
            "interval_updates": 0, "retain_latest": True, "retain_best": False,
            "retain_milestone_every_updates": 0, "evaluation_milestones_usable_transitions": [],
        },
        "selection_evaluation_schedule": {
            "interval_updates": 0, "cases": 16, "repeated_unperturbed_control": True,
            "command_speed_units_per_second": 45.0,
        },
        "reserved_evaluation_steps_wall_and_storage": {
            "physics_steps": 20_400, "wall_seconds": 120, "storage_bytes": 100_000_000,
        },
        "curriculum_and_case_manifest_hashes": {
            "selection_manifest_sha256": sha256_json(selection),
            "training_case_rng": "none; fixed opened selection/control teacher cases",
            "command_speed_schedule": "teacher and both student evaluations fixed at 45 units/s",
        },
        "fresh_holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
        "budget_rationale": "13 full production-teacher episodes plus fresh and cloned 17-case full evaluations require 28,200 physics steps; 40,000 steps and 600 seconds leave explicit failure/I/O margin. No PPO allocation is authorized.",
        "stop_and_continuation_rules": {
            "stop": ["teacher physical gate fails", "identity/control failure", "physics/wall/storage cap", "non-finite supervised update"],
            "plateau_review": "stop after the single frozen 40-epoch clone and full evaluation; never tune or launch PPO automatically",
            "holdout": "remain unopened",
        },
    }
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(output), "sha256": sha256(output),
                      "physics_cap": manifest["maximum_total_physics_steps"],
                      "wall_cap": manifest["maximum_cumulative_wall_seconds"]}))
    return 0


def stage_teacher_ablation_propose(args: argparse.Namespace) -> int:
    """Freeze the bounded full-production versus pose-only replay comparison."""
    fixture = Fixture(args)
    if fixture.identity["git"]["dirty_files"]:
        raise SystemExit("teacher ablation proposal requires a clean source worktree")
    if not args.reference_review or not args.teacher_result:
        raise SystemExit("teacher ablation proposal requires --reference-review and --teacher-result")
    review_path = Path(args.reference_review).resolve()
    review = verify_forward_review(
        {"path": str(review_path), "sha256": sha256(review_path)}, review_path.parent, fixture)
    teacher_result_path = Path(args.teacher_result).resolve()
    teacher_result = json.loads(teacher_result_path.read_text(encoding="utf-8"))
    if (teacher_result.get("schema") != "play01_teacher_bootstrap_result_v1"
            or teacher_result.get("qualification_holdout_opened") is not False
            or not teacher_result.get("teacher_physical", {}).get("gate_passed")
            or not teacher_result.get("target_representability", {}).get("gate_passed")):
        raise SystemExit("teacher result is not an intact passed teacher/coverage measurement")
    torso = fixture.library["body_names"].index(TORSO_BODY_NAME)
    selection, cases_doc = resolved_case_documents(fixture.spec["case_generation"], torso)
    case_ids = [case["id"] for case in selection] + ["unperturbed-control"]
    coverage_path = fixture.library_path.parent / "content-coverage.json"
    manifest = {
        "run_id": "PLAY-01-production-pose-ablation-v1",
        "stage_and_command_scope": {
            "stage": "teacher_ablation", "commands": ["idle", "forward", "start", "stop"],
            "command_speed_schedule_units_per_second": [],
            "teacher_ablation": {
                "source_controller": "production_walker", "case_ids": case_ids,
                "case_manifest_sha256": sha256_json(case_ids),
                "variants": ["production_full", "production_pose_only_oracle", "direct_zero_residual"],
                "pose_only_success_gate": "positive signed speed and fewer full-length falls than direct zero residual",
                "oracle_mapping": "replay the production driven pose through the unchanged direct seam, using the reviewed generator pose plus the per-frame clipped 49-action inverse; no production support/load/force/world-target actuation",
            },
            "qualification_claim": "none; causal actuation representability diagnostic only",
        },
        "specification_and_source_hashes": {
            "specification_id": fixture.identity["specification_id"], "spec_sha256": fixture.identity["spec_sha256"],
            "source_commit": fixture.identity["git"]["commit"], "runner_version": RUNNER_VERSION,
            "engine_binary_sha256": fixture.identity["executable_sha256"],
        },
        "body_profile_and_motion_generator_hashes": {
            "context_sha256": fixture.identity["context_sha256"], "profile_sha256": fixture.identity["profile_sha256"],
            "library_sha256": fixture.identity["library_sha256"], "generator_version": GENERATOR_VERSION,
            "generator_source_sha256": fixture.identity["generator_source_sha256"],
        },
        "content_coverage_manifest_sha256": sha256(coverage_path),
        "forward_reference_review": {"path": review["path"], "sha256": review["sha256"]},
        "parent_checkpoint_and_transfer_policy": {
            "initial_mode": "fresh", "checkpoint": None,
            "treatment": {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"},
            "exact_resume_treatment": {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"},
            "contract": "diagnostic has no learned checkpoint or resume path",
        },
        "compute_allocation_and_authorization_reference": "User authorized the proposed bounded production-actuation diagnostic by saying continue in the Codex task on 2026-09-14",
        "teacher_bootstrap_result": {"path": str(teacher_result_path), "sha256": sha256(teacher_result_path)},
        "measured_steps_and_updates_per_second": teacher_result["throughput"],
        "measurement_protocol_and_overhead": {
            "source": "completed v19 production-teacher bootstrap",
            "comparison_episodes": 3 * len(case_ids), "optimizer_updates": 0,
        },
        "target_usable_optimizer_transitions": 0,
        "maximum_total_physics_steps": 50_000,
        "maximum_cumulative_wall_seconds": 600,
        "maximum_storage_bytes": 500_000_000,
        "worker_count": 3,
        "resolved_rollout_batch_and_optimizer_settings": {
            "episodes_per_batch": 6, "episode_physics_steps": fixture.episode_steps,
            "training_perturbations_enabled": False, "optimizer": fixture.spec["optimizer"],
        },
        "checkpoint_interval_and_retention": {
            "interval_updates": 0, "retain_latest": False, "retain_best": False,
            "retain_milestone_every_updates": 0, "evaluation_milestones_usable_transitions": [],
        },
        "selection_evaluation_schedule": {
            "interval_updates": 0, "cases": 16, "repeated_unperturbed_control": True,
            "command_speed_units_per_second": 45.0,
        },
        "reserved_evaluation_steps_wall_and_storage": {
            "physics_steps": 30_600, "wall_seconds": 120, "storage_bytes": 100_000_000,
        },
        "curriculum_and_case_manifest_hashes": {
            "selection_manifest_sha256": sha256_json(selection),
            "training_case_rng": "none; fixed opened selection/control diagnostic cases",
            "command_speed_schedule": "all three full-length variants fixed at 45 units/s",
        },
        "fresh_holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
        "budget_rationale": "Three full 17-case comparisons require 30,600 physics steps. The independent 50,000-step and 600-second caps leave failure and I/O margin. The sample target is zero because this diagnostic performs no optimization.",
        "stop_and_continuation_rules": {
            "stop": ["identity/control failure", "physics/wall/storage cap", "non-finite telemetry"],
            "plateau_review": "not applicable; run each frozen variant once and stop",
            "holdout": "remain unopened",
        },
    }
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(output), "sha256": sha256(output),
                      "physics_cap": manifest["maximum_total_physics_steps"],
                      "wall_cap": manifest["maximum_cumulative_wall_seconds"]}))
    return 0


def stage_train(args: argparse.Namespace) -> int:
    from play01_run_guard import exclusive_run
    with exclusive_run(Path(args.out)):
        return _stage_train(args)


def _stage_train(args: argparse.Namespace) -> int:
    """Resumable allocated training from a training_run_manifest."""
    import torch
    from play01_run_guard import begin_work, commit_generation, next_generation, recover_pending
    try:
        manifest_path, run_manifest, fixture, review_identity, _ = load_training_manifest(args, require_authorization=True)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.stop_after_updates is not None and int(args.stop_after_updates) < 1:
        raise SystemExit("--stop-after-updates must be positive")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    spec = fixture.spec
    torch.set_num_threads(int(spec["optimizer"]["torch_threads"]))
    state_path = out / "train-state.json"
    previous = json.loads(state_path.read_text(encoding="utf-8")) if state_path.exists() else None
    checkpoint_policy = run_manifest["parent_checkpoint_and_transfer_policy"]
    try:
        validate_checkpoint_mode(args.checkpoint_mode, checkpoint_policy, previous is not None)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.checkpoint_mode in ("fresh", "transfer") and any(path.name != ".run-lock" for path in out.iterdir()):
        raise SystemExit(f"{args.checkpoint_mode} mode requires an empty output directory")
    manifest_sha = sha256(manifest_path)
    repair_migration = run_manifest.get("exact_resume_repair_migration")
    if previous and previous.get("manifest_sha256") != manifest_sha and repair_migration is None:
        raise SystemExit("exact-resume manifest hash mismatch")
    cumulative_wall = previous["cumulative_wall_seconds"] if previous else 0.0
    recovery = dict(previous.get("recovery_reservations", {"unknown_physics_steps_reserved": 0, "wall_seconds_reserved": 0.0})) if previous else {"unknown_physics_steps_reserved": 0, "wall_seconds_reserved": 0.0}
    pending_recovery = recover_pending(out)
    runner = Runner(fixture, out)
    runner.event("start", stage="training", run_id=run_manifest["run_id"], checkpoint_mode=args.checkpoint_mode)
    runner.ensure_workers(fixture.workers)
    b = runner.builder
    torch.manual_seed(int(spec["optimizer"]["seed"]))
    policy = build_policy(b.dimension, b.action_count, spec)
    optimizer = runner.make_optimizer(policy)
    torch_generator = torch.Generator().manual_seed(int(spec["optimizer"]["seed"]))
    rngs = {"training": np.random.Generator(np.random.PCG64(int(spec["case_generation"]["training_seed"])))}
    selection, cases_doc = runner.selection_cases()
    if cases_doc["holdout_manifest_sha256"] != run_manifest["fresh_holdout_manifest_sha256"]:
        raise SystemExit("fresh holdout manifest identity mismatch; no simulation started")
    declared_cases = run_manifest["curriculum_and_case_manifest_hashes"]
    if declared_cases.get("selection_manifest_sha256") != sha256_json(selection):
        raise SystemExit("selection case manifest identity mismatch; no simulation started")
    run_contract = training_run_contract(run_manifest, cases_doc)
    torso = int(runner.manifest["landmarks"][TORSO_LANDMARK])
    control = {"perturbed": False, "impulse": None, "id": "unperturbed-control"}
    eval_cases = selection + [control]
    rollout_settings = run_manifest["resolved_rollout_batch_and_optimizer_settings"]
    episodes_per_batch = int(rollout_settings["episodes_per_batch"])
    checkpoint_settings = run_manifest["checkpoint_interval_and_retention"]
    checkpoint_interval = int(checkpoint_settings["interval_updates"])
    milestone_interval = int(checkpoint_settings["retain_milestone_every_updates"])
    selection_interval = int(run_manifest["selection_evaluation_schedule"]["interval_updates"])
    curriculum = run_manifest["stage_and_command_scope"].get("command_speed_schedule_units_per_second") or []
    if not curriculum:
        curriculum = [{"through_usable_optimizer_transitions": int(run_manifest["target_usable_optimizer_transitions"]), "speed": fixture.command_speed}]

    def curriculum_speed(completed_training_steps: int) -> float:
        for stage in curriculum:
            if completed_training_steps < int(stage["through_usable_optimizer_transitions"]):
                return float(stage["speed"])
        return float(curriculum[-1]["speed"])
    startup_document = run_manifest["stage_and_command_scope"].get("startup_horizon_curriculum")
    startup_stages = validate_startup_horizon_curriculum(startup_document, fixture.episode_steps)
    training_count = 0
    best = None
    completed_evaluation_milestones = list(previous.get("completed_evaluation_milestones", [])) if previous else []
    startup_state = (dict(previous.get("startup_curriculum_state", {})) if previous else {})
    if startup_stages and not startup_state:
        startup_state = {"stage_index": 0, "stage_start_usable_transitions": 0,
                         "last_probe_usable_transitions": 0, "probe_history": [], "completed": False}
    latest = out / "checkpoint-latest.pt"
    if args.checkpoint_mode == "transfer":
        parent = checkpoint_policy["checkpoint"]
        parent_path = Path(parent["path"])
        if not parent_path.is_absolute():
            parent_path = (manifest_path.parent / parent_path).resolve()
        if not parent_path.is_file() or sha256(parent_path) != parent["sha256"]:
            raise SystemExit("declared transfer checkpoint path/hash mismatch")
        runner.load_checkpoint(parent_path, policy, optimizer, torch_generator, rngs, mode="transfer", apply_counters=False)
        runner.event("transferred", checkpoint=str(parent_path), checkpoint_sha256=parent["sha256"],
                     treatment="policy only; optimizer/RNG/counters initialized; fixed normalization")
    elif args.checkpoint_mode == "exact-resume":
        latest = Path(previous.get("latest_checkpoint", {}).get("path", latest))
        if not latest.is_file() or sha256(latest) != previous.get("latest_checkpoint", {}).get("sha256"):
            raise SystemExit("exact-resume latest checkpoint path/hash mismatch")
        state = runner.load_checkpoint(latest, policy, optimizer, torch_generator, rngs, mode="exact-resume", expected_contract=run_contract,
                                       repair_migration=repair_migration,
                                       previous_manifest_sha256=previous["manifest_sha256"] if repair_migration else None)
        training_count = previous["training_count"]
        best = previous.get("best")
        if state["counters"]["cumulative_wall_seconds"] != previous["cumulative_wall_seconds"]:
            raise SystemExit("exact-resume cumulative wall counter mismatch")
        if {key: state["counters"][key] for key in Runner.counter_snapshot_fields()} != previous["counters"]:
            raise SystemExit("exact-resume sidecar counter mismatch")
        for key in ("training_count", "best", "evaluation_guard_state", "recovery_reservations"):
            if state.get("extra", {}).get(key) != previous.get(key):
                raise SystemExit(f"exact-resume sidecar {key} mismatch")
        if state.get("extra", {}).get("completed_evaluation_milestones", []) != completed_evaluation_milestones:
            raise SystemExit("exact-resume pending evaluation schedule mismatch")
        if state.get("extra", {}).get("startup_curriculum_state", {}) != startup_state:
            raise SystemExit("exact-resume startup curriculum state mismatch")
        runner.event("repair_resumed" if repair_migration else "resumed", checkpoint=str(latest), physics_steps=runner.physics_steps,
                     updates=runner.updates, cumulative_wall_seconds=cumulative_wall,
                     repair_reason=None if repair_migration is None else repair_migration["reason"])
    session_start = 0.0  # Include worker startup and checkpoint validation in the execution budget.
    if pending_recovery:
        recovery["unknown_physics_steps_reserved"] += int(pending_recovery["physics_upper_bound"])
        recovery["wall_seconds_reserved"] += float(pending_recovery["wall_upper_bound"])
        cumulative_wall += float(pending_recovery["wall_upper_bound"])
        runner.event("uncertain_work_recovered", **pending_recovery)
    stop_reason = None
    curves_path = out / "learning-curves.json"
    curves = json.loads(curves_path.read_text(encoding="utf-8")).get("updates", []) if previous and curves_path.exists() else []
    if previous:
        restored_tail = state.get("extra", {}).get("curve_tail", [])
        curves = list({row["update"]: row for row in curves + restored_tail}.values())
        curves = sorted((row for row in curves if row["update"] <= runner.updates), key=lambda row: row["update"])
    max_steps = int(run_manifest["maximum_total_physics_steps"])
    max_wall = float(run_manifest["maximum_cumulative_wall_seconds"])
    max_storage = int(run_manifest["maximum_storage_bytes"])
    batch_steps = episodes_per_batch * fixture.episode_steps
    eval_steps = len(eval_cases) * fixture.episode_steps
    startup_probe_steps = (int(startup_document["probe_episodes"]) * fixture.episode_steps
                           if startup_document else 0)
    guard_state = (previous.get("evaluation_guard_state", {}) if previous else {}) or {"baseline": None, "history": []}
    guard_rules = run_manifest["stop_and_continuation_rules"].get("evaluation_guard", {})
    work_wall_reserve = float(run_manifest["reserved_evaluation_steps_wall_and_storage"].get("work_unit_wall_reserve_seconds", 120.0))
    stop_requested = False
    original_signal_handlers: dict[int, Any] = {}

    def request_graceful_stop(signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    for requested_signal in (signal.SIGINT, getattr(signal, "SIGTERM", None)):
        if requested_signal is not None:
            original_signal_handlers[requested_signal] = signal.getsignal(requested_signal)
            signal.signal(requested_signal, request_graceful_stop)

    def persist(reason: str | None) -> None:
        nonlocal latest
        cumulative_wall_now = cumulative_wall + (runner.elapsed() - session_start)
        extra = {"cumulative_wall_seconds": cumulative_wall_now, "training_count": training_count,
                 "run_contract": run_contract, "forward_reference_review": review_identity,
                 "completed_evaluation_milestones": completed_evaluation_milestones,
                 "startup_curriculum_state": startup_state, "best": best,
                 "evaluation_guard_state": guard_state, "recovery_reservations": recovery,
                 "curve_tail": curves[-checkpoint_interval:]}
        state = {"run_id": run_manifest["run_id"], "cumulative_wall_seconds": cumulative_wall_now, "calendar_last_utc": utc_now(), "training_count": training_count,
                 "manifest_sha256": manifest_sha, "counters": runner.counter_snapshot(), "best": best,
                 "run_contract": run_contract, "completed_evaluation_milestones": completed_evaluation_milestones,
                 "startup_curriculum_state": startup_state,
                 "evaluation_guard_state": guard_state, "recovery_reservations": recovery,
                 "stop_reason": reason}
        committed = commit_generation(out, next_generation(out),
            lambda path: runner.save_checkpoint(path, policy, optimizer, torch_generator, rngs, extra), state)
        latest = Path(committed["latest_checkpoint"]["path"])
        if reason is not None or runner.updates % checkpoint_interval == 0:
            atomic_json(out / "learning-curves.json", {"updates": curves})

    def evaluate_selection(label: str) -> dict[str, Any]:
        scheduled = [{**case, "command_speed": float(run_manifest["selection_evaluation_schedule"]["command_speed_units_per_second"])} for case in eval_cases]
        summary = runner.evaluate(label, fixture.policy_controller, scheduled, policy=policy)
        atomic_json(out / f"evaluation-update-{runner.updates}.json", summary)
        return summary

    try:
        persist(None)
        if guard_rules and guard_state["baseline"] is None:
            if (runner.physics_steps + recovery["unknown_physics_steps_reserved"] + eval_steps > max_steps
                    or cumulative_wall + runner.elapsed() + work_wall_reserve > max_wall):
                raise RuntimeError("baseline evaluation cannot fit remaining cumulative allocation")
            begin_work(out, eval_steps, work_wall_reserve)
            baseline = evaluate_selection("baseline-initial-policy")
            guard_state["baseline"] = baseline
            best = {"update": runner.updates, "key": list(physical_selection_key(baseline)),
                    "summary": {k: v for k, v in baseline.items() if k != "per_case"}}
            saved_initial = runner.save_checkpoint(out / "checkpoint-initial.pt", policy, optimizer, torch_generator, rngs,
                {"run_contract": run_contract, "forward_reference_review": review_identity, "evaluation": best["summary"]})
            best["checkpoint"] = saved_initial
            persist(None)
        while True:
            storage = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
            cumulative_now = cumulative_wall + (runner.elapsed() - session_start)
            if stop_requested:
                stop_reason = "interrupted by signal at completed update boundary"
                break
            # A due startup gate performs both its deterministic short probe and
            # the normal full-length selection evaluation. Reserve their worst
            # case before beginning a batch so the hard physics cap cannot be
            # crossed by a stage transition.
            if runner.physics_steps + recovery["unknown_physics_steps_reserved"] + batch_steps + eval_steps + startup_probe_steps > max_steps:
                stop_reason = "maximum_total_physics_steps reached"
                break
            if cumulative_now + work_wall_reserve > max_wall:
                stop_reason = "maximum_cumulative_wall_seconds reached"
                break
            if storage + 20_000_000 > max_storage:
                stop_reason = "maximum_storage_bytes reached"
                break
            if runner.training_usable_transitions >= int(run_manifest["target_usable_optimizer_transitions"]):
                stop_reason = "target_usable_optimizer_transitions reached"
                break
            if args.stop_after_updates is not None and runner.updates >= int(args.stop_after_updates):
                stop_reason = "requested deterministic update-boundary interruption"
                break
            begin_work(out, batch_steps + eval_steps + startup_probe_steps, work_wall_reserve)
            startup_stage = (startup_stages[int(startup_state["stage_index"])]
                             if startup_stages and not startup_state.get("completed") else None)
            training_horizon = int(startup_stage["episode_steps"]) if startup_stage else fixture.episode_steps
            batch_cases = []
            for i in range(episodes_per_batch):
                perturbations_enabled = bool(rollout_settings["training_perturbations_enabled"])
                case = make_case(rngs["training"],
                                 perturbed=training_episode_is_perturbed(training_count + i, perturbations_enabled),
                                 torso=torso)
                case["id"] = f"train-{training_count + i}"
                case["command_speed"] = curriculum_speed(runner.training_usable_transitions)
                case["episode_steps"] = training_horizon
                batch_cases.append(case)
            training_count += episodes_per_batch
            t0 = time.monotonic()
            batch = runner.rollout_batch(policy, batch_cases, torch_generator)
            t1 = time.monotonic()
            record = runner.update(policy, optimizer, batch)
            curves.append({"update": runner.updates, "counters": runner.counter_snapshot(), "elapsed": runner.elapsed(), **record, "rollout_seconds": t1 - t0,
                           "update_seconds": time.monotonic() - t1,
                           "startup_stage_index": None if startup_stage is None else int(startup_state["stage_index"]),
                           "training_horizon_steps": training_horizon})
            startup_probe_record = None
            startup_stage_stop = False
            if startup_stage is not None:
                stage_used = runner.training_usable_transitions - int(startup_state["stage_start_usable_transitions"])
                probe_interval = int(startup_document["probe_interval_usable_transitions"])
                probe_due = (stage_used >= int(startup_stage["minimum_stage_usable_transitions"])
                             and runner.training_usable_transitions - int(startup_state["last_probe_usable_transitions"]) >= probe_interval)
                if probe_due:
                    startup_probe_record = runner.startup_probe(
                        policy, training_horizon, int(startup_document["probe_episodes"]),
                        curriculum_speed(runner.training_usable_transitions),
                        bool(startup_stage.get("require_positive_signed_speed", False)),
                        int(startup_state["stage_index"]))
                    startup_probe_record["usable_optimizer_transitions"] = runner.training_usable_transitions
                    startup_state["probe_history"].append(startup_probe_record)
                    startup_state["last_probe_usable_transitions"] = runner.training_usable_transitions
                    if startup_probe_record["passed"]:
                        if int(startup_state["stage_index"]) + 1 < len(startup_stages):
                            startup_state["stage_index"] = int(startup_state["stage_index"]) + 1
                            startup_state["stage_start_usable_transitions"] = runner.training_usable_transitions
                            runner.event("startup_stage_advanced", stage_index=int(startup_state["stage_index"]),
                                         episode_steps=int(startup_stages[int(startup_state["stage_index"])]["episode_steps"]))
                        else:
                            startup_state["completed"] = True
                            stop_reason = "startup curriculum success gate reached"
                            startup_stage_stop = True
                    elif stage_used >= int(startup_stage["maximum_stage_usable_transitions"]):
                        stop_reason = f"startup curriculum stage {startup_state['stage_index']} failed deterministic gate"
                        startup_stage_stop = True
            milestones = [int(value) for value in checkpoint_settings.get("evaluation_milestones_usable_transitions", [])]
            due_milestones = [value for value in milestones
                              if value <= runner.training_usable_transitions and value not in completed_evaluation_milestones]
            if runner.updates % selection_interval == 0 or due_milestones or startup_probe_record is not None:
                selection_speed = float(run_manifest["selection_evaluation_schedule"]["command_speed_units_per_second"])
                scheduled_eval_cases = [{**case, "command_speed": selection_speed} for case in eval_cases]
                suffix = (f"startup-stage-{startup_probe_record['stage_index']}" if startup_probe_record is not None
                          else (f"milestone-{due_milestones[-1]}" if due_milestones else f"update-{runner.updates}"))
                summary = evaluate_selection(f"selection-{suffix}")
                completed_evaluation_milestones.extend(due_milestones)
                key = physical_selection_key(summary)
                if best is None or key < tuple(best["key"]):
                    best = {"update": runner.updates, "key": list(key), "summary": {k: v for k, v in summary.items() if k != "per_case"}}
                    best["checkpoint"] = runner.save_checkpoint(out / f"checkpoint-best-update-{runner.updates}.pt", policy, optimizer, torch_generator, rngs,
                        {"run_contract": run_contract, "forward_reference_review": review_identity, "evaluation": best["summary"]})
                    runner.event("selection", update=runner.updates, key=list(key))
                guard_state["history"].append({"update": runner.updates, "usable_transitions": runner.training_usable_transitions,
                                              "summary": {k: v for k, v in summary.items() if k != "per_case"}})
                if guard_rules:
                    stop_reason = stop_reason or evaluation_guard(guard_state["history"], guard_state["baseline"], guard_rules, runner.training_usable_transitions)
                    startup_stage_stop = startup_stage_stop or stop_reason is not None
                runner.save_checkpoint(out / f"checkpoint-evaluation-{runner.updates}.pt", policy, optimizer, torch_generator, rngs,
                    {"run_contract": run_contract, "forward_reference_review": review_identity, "evaluation": guard_state["history"][-1]})
            persist(None)
            if startup_stage_stop:
                break
            if runner.updates % checkpoint_interval == 0:
                if runner.updates % milestone_interval == 0:
                    shutil.copy(latest, out / f"checkpoint-update-{runner.updates}.pt")
    except Exception as error:
        stop_reason = f"{type(error).__name__}: {error}"
        runner.event("failure", error=stop_reason)
        atomic_json(out / "failure.json", {"error": stop_reason, "utc": utc_now(), "pending_work": recover_pending(out)})
        runner.close()
        raise
    finally:
        for requested_signal, handler in original_signal_handlers.items():
            signal.signal(requested_signal, handler)
    persist(stop_reason)
    summary_lines = [f"PLAY-01 allocated run ({RUNNER_VERSION})", f"stop: {stop_reason}",
                     f"source: {fixture.identity['git']['commit']}", f"executable sha256: {fixture.identity['executable_sha256']}",
                     f"checkpoint: {latest}", f"checkpoint sha256: {sha256(latest)}",
                     f"counters: {json.dumps(runner.counter_snapshot(), sort_keys=True)}",
                     f"recovery reservations: {json.dumps(recovery, sort_keys=True)}",
                     f"cumulative wall seconds: {cumulative_wall + runner.elapsed():.3f}",
                     f"last evaluated update: {guard_state['history'][-1]['update'] if guard_state['history'] else 0}",
                     f"best evaluated checkpoint: {best.get('checkpoint') if best else None}",
                     "qualification holdouts opened: false", "PLAY-01 remains open."]
    (out / "SUMMARY.txt").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    runner.event("stopped", reason=stop_reason, cumulative_wall_seconds=cumulative_wall + (runner.elapsed() - session_start))
    runner.close()
    return 0


def stage_teacher_bootstrap(args: argparse.Namespace) -> int:
    """Collect a physical production teacher, behavior-clone once, and evaluate the direct seam."""
    import torch
    try:
        manifest_path, run_manifest, fixture, review_identity, _ = load_training_manifest(
            args, require_authorization=True)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if run_manifest["stage_and_command_scope"].get("stage") != "teacher_bootstrap":
        raise SystemExit("teacher-bootstrap requires a teacher_bootstrap manifest")
    teacher = validate_teacher_bootstrap_config(
        run_manifest["stage_and_command_scope"].get("teacher_bootstrap"))
    try:
        validate_checkpoint_mode(args.checkpoint_mode, run_manifest["parent_checkpoint_and_transfer_policy"], False)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.checkpoint_mode != "fresh":
        raise SystemExit("teacher-bootstrap currently requires explicit fresh mode")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise SystemExit("fresh teacher-bootstrap output directory must be empty")
    shutil.copy(manifest_path, out / "training-run-manifest.json")
    runner = Runner(fixture, out)
    maximum_steps = int(run_manifest["maximum_total_physics_steps"])
    maximum_wall = float(run_manifest["maximum_cumulative_wall_seconds"])
    maximum_storage = int(run_manifest["maximum_storage_bytes"])
    report: dict[str, Any] = {
        "schema": "play01_teacher_bootstrap_result_v1", "tracker_task": "PLAY-01",
        "started_utc": utc_now(), "identity": fixture.identity,
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "forward_reference_review": review_identity, "qualification_holdout_opened": False,
    }
    try:
        # Load and verify the native body/action manifest in the parent before
        # resolving cases. Starting the rollout pool will close this handshake
        # worker and replace it with the declared persistent child processes.
        runner.ensure_workers(1)
        selection, cases_doc = runner.selection_cases()
        if cases_doc["holdout_manifest_sha256"] != run_manifest["fresh_holdout_manifest_sha256"]:
            raise RuntimeError("teacher bootstrap holdout identity mismatch")
        control = {"perturbed": False, "impulse": None, "id": "unperturbed-control",
                   "command_speed": 45.0}
        all_cases = [{**case, "command_speed": 45.0} for case in selection] + [control]
        case_by_id = {case["id"]: case for case in all_cases}
        teacher_cases = [case_by_id[case_id] for case_id in teacher["teacher_case_ids"]]
        if sha256_json(teacher["teacher_case_ids"]) != teacher["teacher_case_manifest_sha256"]:
            raise RuntimeError("teacher case manifest identity mismatch")
        required_steps = (len(teacher_cases) + 2 * len(all_cases)) * fixture.episode_steps
        if required_steps > maximum_steps:
            raise RuntimeError("teacher bootstrap cannot fit its frozen physics budget")
        (out / "resolved-cases.json").write_text(json.dumps({
            "selection": selection, "teacher_case_ids": teacher["teacher_case_ids"],
            "teacher_case_manifest_sha256": teacher["teacher_case_manifest_sha256"],
            "holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
            "holdout_status": cases_doc["holdout_status"],
        }, indent=1, allow_nan=False) + "\n", encoding="utf-8")

        observations, proposals = [], []
        teacher_metrics = []
        target_diagnostics, filter_diagnostics = [], []
        eligible_teacher_frames = 0
        missing_pose_records = []
        jobs = [{"controller": "reference", "case": case, "stochastic": False,
                 "journal": False, "compact": True, "label": f"teacher-{case['id']}",
                 "seed_label": "production-teacher", "purpose": "diagnostic",
                 "stop_on_terminal": False, "collect_teacher": True}
                for case in teacher_cases]
        collection_started = time.monotonic()
        teacher_results = runner.parallel_episodes(None, jobs)
        collection_elapsed = time.monotonic() - collection_started
        for result in teacher_results:
            samples = result["teacher_samples"]
            observations.extend(samples["obs"])
            proposals.extend(samples["proposal"])
            target_diagnostics.extend(samples["target_diagnostics"])
            filter_diagnostics.extend(samples["filter_diagnostics"])
            eligible_teacher_frames += int(samples["eligible_frames"])
            if samples["missing_driven_pose_steps"]:
                missing_pose_records.append({"case_id": result["case"]["id"],
                                             "steps": samples["missing_driven_pose_steps"]})
            teacher_metrics.append(result["metrics"])
        report["throughput"] = {
            "usable_teacher_pose_samples": len(observations),
            "teacher_collection_physics_steps": sum(
                int(result["accounting"]["simulated_physics_steps"]) for result in teacher_results),
            "teacher_collection_wall_seconds": collection_elapsed,
            "usable_teacher_pose_samples_per_second": len(observations) / max(collection_elapsed, 1e-9),
            "definition": "valid post-warmup production pose targets divided by parallel collection wall time; supervised samples are not PPO optimizer transitions",
        }
        teacher_signed = float(np.mean([metric["mean_achieved_along_command_speed_walk"]
                                        for metric in teacher_metrics]))
        teacher_survival = float(np.mean([metric["fall_step"] is None for metric in teacher_metrics]))
        teacher_summary = {
            "cases": len(teacher_metrics),
            "falls": int(sum(metric["fall_step"] is not None for metric in teacher_metrics)),
            "survival_fraction": teacher_survival,
            "mean_signed_along_command_speed": teacher_signed,
            "mean_commanded_velocity_rmse": float(np.mean(
                [metric["commanded_velocity_rmse_walk"] for metric in teacher_metrics])),
            "case_metrics": teacher_metrics,
        }
        physical_gate = (teacher_survival >= float(teacher["minimum_teacher_survival_fraction"])
                         and (not teacher["require_positive_teacher_signed_speed"] or teacher_signed > 0.0))
        pose_coverage = len(observations) / max(1, eligible_teacher_frames)
        representability_gate = (bool(observations)
                                 and pose_coverage >= float(teacher["minimum_teacher_pose_coverage_fraction"]))
        teacher_gate = physical_gate and representability_gate
        report["teacher_physical"] = {**teacher_summary, "gate_passed": physical_gate}
        report["target_representability"] = {
            "samples": len(observations),
            "eligible_post_warmup_frames": eligible_teacher_frames,
            "pose_coverage_fraction": pose_coverage,
            "minimum_pose_coverage_fraction": teacher["minimum_teacher_pose_coverage_fraction"],
            "missing_driven_pose_frames": eligible_teacher_frames - len(observations),
            "missing_driven_pose_records": missing_pose_records,
            "gate_passed": representability_gate,
            "mean_clipped_joint_fraction": float(np.mean(
                [entry["clipped_joint_fraction"] for entry in target_diagnostics])) if target_diagnostics else 0.0,
            "frames_with_any_clipped_joint_fraction": float(np.mean(
                [entry["clipped_joint_count"] > 0 for entry in target_diagnostics])) if target_diagnostics else 0.0,
            "maximum_requested_joint_norm": max(
                (entry["maximum_requested_joint_norm"] for entry in target_diagnostics), default=0.0),
            "maximum_hinge_off_axis_error_radians": max(
                (entry["maximum_hinge_off_axis_error_radians"] for entry in target_diagnostics), default=0.0),
            "mean_filter_proposal_clipped_fraction": float(np.mean(
                [entry["proposal_clipped_fraction"] for entry in filter_diagnostics])) if filter_diagnostics else 0.0,
            "mean_filter_delivery_rms_error": float(np.mean(
                [entry["rms_delivery_error"] for entry in filter_diagnostics])) if filter_diagnostics else 0.0,
            "maximum_filter_delivery_error": max(
                (entry["maximum_delivery_error"] for entry in filter_diagnostics), default=0.0),
            "scope": "joint-pose component only; production support loads, force pairs, released axes and world targets are not residual-policy actions",
        }
        if not teacher_gate:
            report["stop_reason"] = ("production teacher failed its frozen physical gate"
                                     if not physical_gate else
                                     "production teacher failed its frozen pose-coverage gate")
            raise StopIteration

        torch.set_num_threads(int(fixture.spec["optimizer"]["torch_threads"]))
        torch.manual_seed(int(fixture.spec["optimizer"]["seed"]))
        policy = build_policy(runner.builder.dimension, runner.builder.action_count, fixture.spec)
        initial = runner.evaluate("fresh-zero-residual", "direct", all_cases, policy=policy)
        optimizer, torch_generator, cloning = behavior_clone_actor(
            policy, np.asarray(observations, dtype=np.float32), np.asarray(proposals, dtype=np.float32),
            epochs=int(teacher["epochs"]), minibatch_size=int(teacher["minibatch_size"]),
            learning_rate=float(teacher["learning_rate"]), seed=int(fixture.spec["optimizer"]["seed"]))
        student = runner.evaluate("teacher-cloned-direct", "direct", all_cases, policy=policy)
        report["fresh_zero_residual"] = initial
        report["behavior_cloning"] = cloning
        report["student_full_evaluation"] = student
        student_gate = (student["mean_achieved_along_command_speed_walk"] > 0.0
                        and student["falls"] < initial["falls"])
        report["student_success_gate_passed"] = student_gate
        report["stop_reason"] = ("teacher bootstrap student gate passed"
                                 if student_gate else "teacher bootstrap student gate failed")
        rngs = {"teacher_cases": np.random.Generator(
            np.random.PCG64(int(fixture.spec["case_generation"]["training_seed"])))}
        checkpoint = runner.save_checkpoint(
            out / "checkpoint-teacher-bootstrap.pt", policy, optimizer, torch_generator, rngs,
            {"cumulative_wall_seconds": runner.elapsed(),
             "run_contract": training_run_contract(run_manifest, cases_doc),
             "forward_reference_review": review_identity,
             "teacher_bootstrap": teacher, "teacher_result_gate_passed": student_gate})
        report["checkpoint"] = checkpoint
    except StopIteration:
        pass
    finally:
        report["finished_utc"] = utc_now()
        report["counters"] = runner.counter_snapshot()
        report["elapsed_seconds"] = runner.elapsed()
        report.setdefault("throughput", {})["whole_experiment_physics_steps_per_second"] = (
            runner.physics_steps / max(report["elapsed_seconds"], 1e-9))
        report["caps"] = {"maximum_total_physics_steps": maximum_steps,
                          "maximum_cumulative_wall_seconds": maximum_wall,
                          "maximum_storage_bytes": maximum_storage}
        report_path = out / "teacher-bootstrap-report.json"
        report_path.write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
        summary = [
            f"PLAY-01 production-teacher bootstrap ({RUNNER_VERSION})",
            f"stop reason: {report.get('stop_reason', 'unexpected failure')}",
            f"source: {fixture.identity['git']['commit']}",
            f"executable sha256: {fixture.identity['executable_sha256']}",
            f"manifest sha256: {sha256(manifest_path)}",
            f"counters: {json.dumps(runner.counter_snapshot(), sort_keys=True)}",
            f"elapsed seconds: {runner.elapsed():.3f}",
        ]
        if "teacher_physical" in report:
            value = report["teacher_physical"]
            summary.append(f"teacher: falls {value['falls']}/{value['cases']}, survival {value['survival_fraction']:.3f}, signed speed {value['mean_signed_along_command_speed']:.3f}, gate {value['gate_passed']}")
        if report.get("throughput", {}).get("usable_teacher_pose_samples_per_second") is not None:
            value = report["throughput"]
            summary.append(f"throughput: {value['usable_teacher_pose_samples']} usable teacher samples / {value['teacher_collection_wall_seconds']:.3f} collection seconds = {value['usable_teacher_pose_samples_per_second']:.2f} samples/s; whole experiment {value['whole_experiment_physics_steps_per_second']:.2f} physics steps/s")
        if "target_representability" in report:
            value = report["target_representability"]
            summary.append(f"pose target: coverage {value['pose_coverage_fraction']:.3f}, mean clipped joints {value['mean_clipped_joint_fraction']:.3f}, frames clipped {value['frames_with_any_clipped_joint_fraction']:.3f}, filter RMS error {value['mean_filter_delivery_rms_error']:.3f}, gate {value['gate_passed']}")
        if "student_full_evaluation" in report:
            initial = report["fresh_zero_residual"]
            student = report["student_full_evaluation"]
            summary.append(f"fresh: falls {initial['falls']}/{initial['cases']}, signed speed {initial['mean_achieved_along_command_speed_walk']:.3f}, RMSE {initial['mean_commanded_velocity_rmse_walk']:.3f}")
            summary.append(f"student: falls {student['falls']}/{student['cases']}, signed speed {student['mean_achieved_along_command_speed_walk']:.3f}, RMSE {student['mean_commanded_velocity_rmse_walk']:.3f}, gate {report['student_success_gate_passed']}")
        summary += ["qualification holdouts opened: false", "No PPO allocation was launched."]
        (out / "SUMMARY.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
        report["storage_bytes"] = sum(path.stat().st_size for path in out.rglob("*") if path.is_file())
        report_path.write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
        runner.close()
    if runner.physics_steps > maximum_steps or report["elapsed_seconds"] > maximum_wall or report["storage_bytes"] > maximum_storage:
        raise RuntimeError("teacher bootstrap exceeded a frozen cap")
    return 0


def stage_teacher_ablation(args: argparse.Namespace) -> int:
    """Compare full production actuation with its clipped pose-only direct replay."""
    try:
        manifest_path, run_manifest, fixture, review_identity, _ = load_training_manifest(
            args, require_authorization=True)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if run_manifest["stage_and_command_scope"].get("stage") != "teacher_ablation":
        raise SystemExit("teacher-ablation requires a teacher_ablation manifest")
    ablation = validate_teacher_ablation_config(
        run_manifest["stage_and_command_scope"].get("teacher_ablation"))
    try:
        validate_checkpoint_mode(args.checkpoint_mode,
                                 run_manifest["parent_checkpoint_and_transfer_policy"], False)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.checkpoint_mode != "fresh":
        raise SystemExit("teacher-ablation requires explicit fresh mode")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise SystemExit("fresh teacher-ablation output directory must be empty")
    shutil.copy(manifest_path, out / "diagnostic-run-manifest.json")
    runner = Runner(fixture, out)
    maximum_steps = int(run_manifest["maximum_total_physics_steps"])
    maximum_wall = float(run_manifest["maximum_cumulative_wall_seconds"])
    maximum_storage = int(run_manifest["maximum_storage_bytes"])
    report: dict[str, Any] = {
        "schema": "play01_teacher_ablation_result_v1", "tracker_task": "PLAY-01",
        "started_utc": utc_now(), "identity": fixture.identity,
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "forward_reference_review": review_identity, "qualification_holdout_opened": False,
    }

    def summarize(label: str, controller: str, results: list[dict[str, Any]]) -> dict[str, Any]:
        metrics = [result["metrics"] for result in results]
        return {
            "label": label, "controller": controller, "cases": len(metrics),
            "falls": int(sum(metric["fall_step"] is not None for metric in metrics)),
            "mean_commanded_velocity_rmse_walk": float(np.mean(
                [metric["commanded_velocity_rmse_walk"] for metric in metrics])),
            "mean_achieved_along_command_speed_walk": float(np.mean(
                [metric["mean_achieved_along_command_speed_walk"] for metric in metrics])),
            "mean_joint_acceleration_rms": float(np.mean(
                [metric["joint_acceleration_rms"] for metric in metrics])),
            "mean_support_slip_sum": float(np.mean(
                [metric["support_slip_sum"] for metric in metrics])),
            "violation_counts": {key: int(sum(bool(metric["violations"][key]) for metric in metrics
                                                   if metric["violations"][key] is not None))
                                 for key in metrics[0]["violations"]},
            "per_case": [{"case": result["case"], "metrics": result["metrics"]}
                         for result in results],
        }

    try:
        runner.ensure_workers(1)
        selection, cases_doc = runner.selection_cases()
        if cases_doc["holdout_manifest_sha256"] != run_manifest["fresh_holdout_manifest_sha256"]:
            raise RuntimeError("teacher ablation holdout identity mismatch")
        control = {"perturbed": False, "impulse": None, "id": "unperturbed-control",
                   "command_speed": 45.0}
        all_cases = [{**case, "command_speed": 45.0} for case in selection] + [control]
        case_by_id = {case["id"]: case for case in all_cases}
        cases = [case_by_id[case_id] for case_id in ablation["case_ids"]]
        if sha256_json(ablation["case_ids"]) != ablation["case_manifest_sha256"]:
            raise RuntimeError("teacher ablation case manifest identity mismatch")
        required_steps = 3 * len(cases) * fixture.episode_steps
        if required_steps > maximum_steps:
            raise RuntimeError("teacher ablation cannot fit its frozen physics budget")
        (out / "resolved-cases.json").write_text(json.dumps({
            "selection": selection, "diagnostic_case_ids": ablation["case_ids"],
            "diagnostic_case_manifest_sha256": ablation["case_manifest_sha256"],
            "holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
            "holdout_status": cases_doc["holdout_status"],
        }, indent=1, allow_nan=False) + "\n", encoding="utf-8")

        production_jobs = [{"controller": "reference", "case": case, "stochastic": False,
                            "journal": False, "compact": True, "label": f"production-{case['id']}",
                            "seed_label": "production-pose-ablation", "purpose": "diagnostic",
                            "stop_on_terminal": False, "collect_teacher": True}
                           for case in cases]
        started = time.monotonic()
        production_results = runner.parallel_episodes(None, production_jobs)
        production_wall = time.monotonic() - started
        oracle_jobs = []
        oracle_diagnostics = []
        missing_pose_frames = 0
        for case, result in zip(cases, production_results):
            samples = result["teacher_samples"]
            if len(samples["oracle_actions"]) != fixture.episode_steps or len(samples["oracle_poses"]) != fixture.episode_steps:
                raise RuntimeError("production pose oracle does not cover the full diagnostic trajectory")
            oracle_diagnostics.extend(samples["oracle_target_diagnostics"])
            missing_pose_frames += len(samples["missing_driven_pose_steps"])
            oracle_jobs.append({
                "controller": "direct", "case": case, "stochastic": False,
                "replay": {"actions": np.asarray(samples["oracle_actions"], dtype=np.float32),
                           "poses": np.asarray(samples["oracle_poses"], dtype=np.float32)},
                "journal": False, "compact": True, "label": f"pose-only-{case['id']}",
                "seed_label": "production-pose-only-oracle", "purpose": "evaluation",
                "stop_on_terminal": False,
            })
        started = time.monotonic()
        oracle_results = runner.parallel_episodes(None, oracle_jobs)
        oracle_wall = time.monotonic() - started
        zero = runner.evaluate("direct-zero-residual", "direct", cases, policy=None)
        production = summarize("production-full", "reference", production_results)
        oracle = summarize("production-pose-only-oracle", "direct", oracle_results)
        gate = (oracle["mean_achieved_along_command_speed_walk"] > 0.0
                and oracle["falls"] < zero["falls"])
        report.update({
            "production_full": production,
            "production_pose_only_oracle": oracle,
            "direct_zero_residual": zero,
            "pose_only_success_gate_passed": gate,
            "representability": {
                "total_frames": len(cases) * fixture.episode_steps,
                "frames_with_driven_pose": len(oracle_diagnostics),
                "missing_post_terminal_pose_frames": missing_pose_frames,
                "frames_with_any_clipped_joint_fraction": float(np.mean(
                    [entry["clipped_joint_count"] > 0 for entry in oracle_diagnostics])),
                "mean_clipped_joint_fraction": float(np.mean(
                    [entry["clipped_joint_fraction"] for entry in oracle_diagnostics])),
                "maximum_requested_joint_norm": max(
                    entry["maximum_requested_joint_norm"] for entry in oracle_diagnostics),
            },
            "timing": {"production_collection_wall_seconds": production_wall,
                       "pose_only_replay_wall_seconds": oracle_wall},
            "causal_interpretation": (
                "the bounded production pose component retained useful positive forward motion"
                if gate else
                "the bounded production pose component alone did not retain useful positive forward motion; non-pose production actuation and/or a different action seam is required"),
            "stop_reason": "teacher ablation completed",
        })
    finally:
        report["finished_utc"] = utc_now()
        report["counters"] = runner.counter_snapshot()
        report["elapsed_seconds"] = runner.elapsed()
        report["caps"] = {"maximum_total_physics_steps": maximum_steps,
                          "maximum_cumulative_wall_seconds": maximum_wall,
                          "maximum_storage_bytes": maximum_storage}
        report_path = out / "teacher-ablation-report.json"
        report_path.write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
        summary = [
            f"PLAY-01 production pose-only ablation ({RUNNER_VERSION})",
            f"stop reason: {report.get('stop_reason', 'unexpected failure')}",
            f"source: {fixture.identity['git']['commit']}",
            f"executable sha256: {fixture.identity['executable_sha256']}",
            f"manifest sha256: {sha256(manifest_path)}",
            f"counters: {json.dumps(runner.counter_snapshot(), sort_keys=True)}",
            f"elapsed seconds: {runner.elapsed():.3f}",
        ]
        for key in ("production_full", "production_pose_only_oracle", "direct_zero_residual"):
            if key in report:
                value = report[key]
                summary.append(f"{key}: falls {value['falls']}/{value['cases']}, signed speed {value['mean_achieved_along_command_speed_walk']:.3f}, RMSE {value['mean_commanded_velocity_rmse_walk']:.3f}")
        if "pose_only_success_gate_passed" in report:
            summary.append(f"pose-only success gate: {report['pose_only_success_gate_passed']}")
            summary.append(f"interpretation: {report['causal_interpretation']}")
        summary += ["qualification holdouts opened: false", "No optimization or allocated training was launched."]
        (out / "SUMMARY.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
        report["storage_bytes"] = sum(path.stat().st_size for path in out.rglob("*") if path.is_file())
        report_path.write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
        runner.close()
    if (runner.physics_steps > maximum_steps or report["elapsed_seconds"] > maximum_wall
            or report["storage_bytes"] > maximum_storage):
        raise RuntimeError("teacher ablation exceeded a frozen cap")
    return 0


def stage_preflight(args: argparse.Namespace) -> int:
    """Validate an allocated-run manifest and checkpoint without starting physics."""
    import torch
    try:
        manifest_path, run_manifest, fixture, review_identity, authorized = load_training_manifest(args, require_authorization=False)
        training_out = Path(args.training_out).resolve()
        state_path = training_out / "train-state.json"
        previous = json.loads(state_path.read_text(encoding="utf-8")) if state_path.is_file() else None
        checkpoint_policy = run_manifest["parent_checkpoint_and_transfer_policy"]
        validate_checkpoint_mode(args.checkpoint_mode, checkpoint_policy, previous is not None)
        if args.checkpoint_mode in ("fresh", "transfer") and training_out.exists() and any(path.name != ".run-lock" for path in training_out.iterdir()):
            raise ValueError(f"{args.checkpoint_mode} mode requires an empty training output directory")
        manifest_sha = sha256(manifest_path)
        repair_migration = run_manifest.get("exact_resume_repair_migration")
        if previous and previous.get("manifest_sha256") != manifest_sha and repair_migration is None:
            raise ValueError("exact-resume manifest hash mismatch")
        try:
            torso = fixture.library["body_names"].index(TORSO_BODY_NAME)
        except ValueError as error:
            raise ValueError(f"motion library lacks frozen torso body {TORSO_BODY_NAME!r} needed for case identity") from error
        selection, cases_doc = resolved_case_documents(fixture.spec["case_generation"], torso)
        if cases_doc["holdout_manifest_sha256"] != run_manifest["fresh_holdout_manifest_sha256"]:
            raise ValueError("fresh holdout manifest identity mismatch")
        if run_manifest["curriculum_and_case_manifest_hashes"].get("selection_manifest_sha256") != sha256_json(selection):
            raise ValueError("selection case manifest identity mismatch")
        target = int(run_manifest["target_usable_optimizer_transitions"])
        maximum_steps = int(run_manifest["maximum_total_physics_steps"])
        reserved_steps = int(run_manifest["reserved_evaluation_steps_wall_and_storage"]["physics_steps"])
        stage = run_manifest["stage_and_command_scope"].get("stage")
        diagnostic_stage = stage in ("teacher_bootstrap", "teacher_ablation")
        if (target < 0 or (target == 0 and not diagnostic_stage) or maximum_steps <= reserved_steps
                or int(run_manifest["maximum_cumulative_wall_seconds"]) <= 0):
            raise ValueError("allocation counters/caps are not positive or leave no training physics budget")
        curriculum = run_manifest["stage_and_command_scope"].get("command_speed_schedule_units_per_second", [])
        curriculum_limits = [int(stage["through_usable_optimizer_transitions"]) for stage in curriculum]
        if (not diagnostic_stage
                and (not curriculum_limits or curriculum_limits != sorted(set(curriculum_limits))
                     or curriculum_limits[-1] != target)):
            raise ValueError("curriculum usable-transition boundaries are incomplete or non-monotonic")
        startup_stages = validate_startup_horizon_curriculum(
            run_manifest["stage_and_command_scope"].get("startup_horizon_curriculum"), fixture.episode_steps)
        probe_steps = (int(run_manifest["stage_and_command_scope"]["startup_horizon_curriculum"]["probe_episodes"])
                       * fixture.episode_steps if startup_stages else 0)
        selection_steps = (len(selection) + 1) * fixture.episode_steps
        if reserved_steps < probe_steps + selection_steps:
            raise ValueError("evaluation reserve cannot fit one startup probe plus full selection evaluation")
        if stage == "teacher_bootstrap":
            teacher = validate_teacher_bootstrap_config(
                run_manifest["stage_and_command_scope"].get("teacher_bootstrap"))
            valid_ids = {case["id"] for case in selection} | {"unperturbed-control"}
            if not set(teacher["teacher_case_ids"]).issubset(valid_ids):
                raise ValueError("teacher bootstrap names a case outside the opened selection/control set")
            required_steps = (len(teacher["teacher_case_ids"]) + 2 * (len(selection) + 1)) * fixture.episode_steps
            if required_steps > maximum_steps:
                raise ValueError("teacher bootstrap collection and two full evaluations exceed the physics cap")
        if stage == "teacher_ablation":
            ablation = validate_teacher_ablation_config(
                run_manifest["stage_and_command_scope"].get("teacher_ablation"))
            valid_ids = {case["id"] for case in selection} | {"unperturbed-control"}
            if not set(ablation["case_ids"]).issubset(valid_ids):
                raise ValueError("teacher ablation names a case outside the opened selection/control set")
            required_steps = 3 * len(ablation["case_ids"]) * fixture.episode_steps
            if required_steps > maximum_steps or reserved_steps < required_steps:
                raise ValueError("teacher ablation comparison does not fit the physics reserve/cap")
        run_contract = training_run_contract(run_manifest, cases_doc)
        checkpoint_identity = None
        if args.checkpoint_mode == "transfer":
            parent = checkpoint_policy["checkpoint"]
            checkpoint_path = Path(parent["path"])
            if not checkpoint_path.is_absolute():
                checkpoint_path = (manifest_path.parent / checkpoint_path).resolve()
            if not checkpoint_path.is_file() or sha256(checkpoint_path) != parent["sha256"]:
                raise ValueError("declared transfer checkpoint path/hash mismatch")
            state = torch.load(checkpoint_path, weights_only=False)
            validate_checkpoint_state(state, fixture, "transfer")
            checkpoint_identity = {"path": str(checkpoint_path), "sha256": parent["sha256"], "mode": "policy_only_transfer"}
        elif args.checkpoint_mode == "exact-resume":
            latest = Path(previous.get("latest_checkpoint", {}).get("path", training_out / "checkpoint-latest.pt"))
            if not latest.is_file() or sha256(latest) != previous.get("latest_checkpoint", {}).get("sha256"):
                raise ValueError("exact-resume latest checkpoint path/hash mismatch")
            state = torch.load(latest, weights_only=False)
            if repair_migration is not None:
                validate_repair_migration(repair_migration, state, fixture, sha256(latest), previous["manifest_sha256"])
            validate_checkpoint_state(state, fixture, "exact-resume", run_contract,
                                      allow_repair_transition=repair_migration is not None)
            if state["counters"]["cumulative_wall_seconds"] != previous["cumulative_wall_seconds"]:
                raise ValueError("exact-resume cumulative wall counter mismatch")
            if state.get("extra", {}).get("completed_evaluation_milestones", []) != previous.get("completed_evaluation_milestones", []):
                raise ValueError("exact-resume pending evaluation schedule mismatch")
            if state.get("extra", {}).get("startup_curriculum_state", {}) != previous.get("startup_curriculum_state", {}):
                raise ValueError("exact-resume startup curriculum state mismatch")
            checkpoint_identity = {"path": str(latest), "sha256": sha256(latest), "mode": "exact_resume",
                                   "counters": state["counters"]}
        report = {
            "schema": "play01_training_preflight_v1", "generated_utc": utc_now(), "tracker_task": "PLAY-01",
            "status": "ready" if authorized else "ready_except_compute_authorization",
            "compute_authorized": authorized, "checkpoint_mode": args.checkpoint_mode,
            "manifest": {"path": str(manifest_path), "sha256": manifest_sha, "run_id": run_manifest["run_id"]},
            "source_and_fixture_identity": fixture.identity, "forward_reference_review": review_identity,
            "case_contract": run_contract, "checkpoint": checkpoint_identity,
            "startup_horizon_curriculum": run_manifest["stage_and_command_scope"].get("startup_horizon_curriculum"),
            "exact_resume_repair_migration": repair_migration,
            "limits": {"target_usable_optimizer_transitions": target, "maximum_total_physics_steps": maximum_steps,
                       "reserved_evaluation_physics_steps": reserved_steps,
                       "maximum_cumulative_wall_seconds": int(run_manifest["maximum_cumulative_wall_seconds"]),
                       "maximum_storage_bytes": int(run_manifest["maximum_storage_bytes"])},
            "simulation_physics_steps": 0, "qualification_holdout_opened": False,
        }
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        raise SystemExit(f"training preflight failed: {error}") from error
    output = Path(args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=1, allow_nan=False, default=str) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "report": str(output), "simulation_physics_steps": 0}))
    return 0


def stage_propose(args: argparse.Namespace) -> int:
    fixture = Fixture(args)
    smoke = Path(args.smoke).resolve()
    measured = json.loads((smoke / "throughput-and-budget-proposal.json").read_text(encoding="utf-8"))
    throughput = measured["measured"]
    proposal = propose_allocation(fixture, throughput, int(measured["integration_used_steps"]),
                                  float(measured["integration_elapsed_seconds"]), smoke)
    output_path = Path(args.out).resolve()
    manifest_path = output_path.with_name(output_path.stem + "-manifest.json")
    manifest_path.write_text(json.dumps(proposal["proposed_training_run_manifest"], indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    proposal["proposed_training_run_manifest_path"] = str(manifest_path)
    proposal["reproducible_launch_command"] = (
        f"python scripts/ragdoll-learning/play01_runner.py train --checkpoint-mode {proposal['proposed_training_run_manifest']['parent_checkpoint_and_transfer_policy']['initial_mode']} --manifest {manifest_path} "
        f"--out {output_path.parent / 'train-PLAY-01-startup-horizon-250k'} --exe {fixture.exe} "
        f"--library {fixture.library_path} --source-root {fixture.source_root} --workers 3 --tracker-task PLAY-01")
    output_path.write_text(json.dumps(proposal, indent=1, allow_nan=False, default=float) + "\n", encoding="utf-8")
    print(json.dumps(proposal["recommended"]))
    return 0


def stage_readiness(args: argparse.Namespace) -> int:
    fixture = Fixture(args)
    out = Path(args.out).resolve()
    runner = Runner(fixture, out)
    try:
        report = runner.readiness()
    finally:
        runner.close()
    print(json.dumps({k: v["passed"] for k, v in report["requirements"].items()}))
    return 0 if report["passed"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="stage", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--source-root", required=True)
        p.add_argument("--exe", required=True)
        p.add_argument("--library", required=True)
        p.add_argument("--out", required=True)
        p.add_argument("--floor", default="tiled", choices=["tiled", "mesh", "box"])
        p.add_argument("--workers", type=int, default=3)
        p.add_argument("--control-mode", choices=["direct", "production-residual"], default="direct")
        p.add_argument("--bound-speed", type=float, default=None)
        p.add_argument("--command-speed", type=float, default=None, help="commanded walking speed; defaults to the bound reference speed per specification")
        p.add_argument("--episodes-per-batch", type=int, default=6)
        p.add_argument("--reference-review", default=None)
        p.add_argument("--tracker-task", choices=["PLAY-01"], required=True)

    r = sub.add_parser("readiness")
    common(r)
    s = sub.add_parser("smoke")
    common(s)
    s.add_argument("--max-steps", type=int, default=None)
    s.add_argument("--max-seconds", type=float, default=None)
    s.add_argument("--smoke-updates", type=int, default=None, help="stop the optimizer phase after this many updates (budget permitting)")
    s.add_argument("--allow-existing", action="store_true")
    t = sub.add_parser("train")
    common(t)
    t.add_argument("--manifest", required=True)
    t.add_argument("--checkpoint-mode", required=True, choices=["fresh", "transfer", "exact-resume"])
    t.add_argument("--stop-after-updates", type=int, default=None,
                   help="test interruption at the next completed update boundary")
    v = sub.add_parser("preflight")
    common(v)
    v.add_argument("--manifest", required=True)
    v.add_argument("--checkpoint-mode", required=True, choices=["fresh", "transfer", "exact-resume"])
    v.add_argument("--training-out", required=True, help="training directory whose empty/resume state is being validated")
    p = sub.add_parser("propose")
    common(p)
    p.add_argument("--smoke", required=True)
    tp = sub.add_parser("teacher-propose")
    common(tp)
    tap = sub.add_parser("teacher-ablation-propose")
    common(tap)
    tap.add_argument("--teacher-result", required=True)
    tb = sub.add_parser("teacher-bootstrap")
    common(tb)
    tb.add_argument("--manifest", required=True)
    tb.add_argument("--checkpoint-mode", required=True, choices=["fresh", "transfer", "exact-resume"])
    ta = sub.add_parser("teacher-ablation")
    common(ta)
    ta.add_argument("--manifest", required=True)
    ta.add_argument("--checkpoint-mode", required=True, choices=["fresh", "transfer", "exact-resume"])
    args = parser.parse_args()
    return {"readiness": stage_readiness, "smoke": stage_smoke, "train": stage_train,
            "preflight": stage_preflight, "propose": stage_propose,
            "teacher-propose": stage_teacher_propose,
            "teacher-bootstrap": stage_teacher_bootstrap,
            "teacher-ablation-propose": stage_teacher_ablation_propose,
            "teacher-ablation": stage_teacher_ablation}[args.stage](args)


if __name__ == "__main__":
    raise SystemExit(main())
