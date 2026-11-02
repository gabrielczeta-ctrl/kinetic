#!/usr/bin/env python3
"""PLAY-01 command-conditioned reference generator (deterministic, journaled).

Inputs per step: requested horizontal travel velocity, desired facing, the
previous phase/transition state and the permitted measured feedback (COM speed
along the command). Output: coherent parent-local joint rotations on the fixed
captured geometry, their finite-difference angular velocities, the model-space
pose used as the motor reference, root-relative end effectors in the reference
heading frame, phase/blend/rate state and a coverage identity.

Design (recorded, versioned as ``GENERATOR_VERSION``):
  * Clip families come from the motion library: idle plus one loop per travel
    family (forward, backward, strafe_left, strafe_right). The family is chosen
    by the travel direction expressed in the FACING frame, so travel and facing
    stay independent; a diagonal travel direction blends the two adjacent
    families with angular weights (nlerp of parent-local rotations, FK after).
  * Walking phase: one shared bounded clock. target_rate =
    clamp(max(measured_along_command_speed, 0) / bound_speed, 0.5, 1.5),
    rate = 0.8 * rate + 0.2 * target_rate, phase advances by rate * dt /
    clip_duration while the walk weight is positive. Start phase is 0.
  * Start/stop: smoothstep blend weight between idle and the walk over the
    protocol's blend durations. Reversal/direction changes crossfade family
    weights over ``transition_steps`` with phase preserved.
  * Facing: the reference root yaw slews toward the requested facing at a
    bounded rate. Turning in place has no clip: the generator reports the
    command as unsupported coverage while holding the idle/walk reference.
  * Out-of-coverage commands are reported explicitly, never relabelled.

The generator proves nothing about balance. It is the same object for the
zero-residual tracker and the learned variant.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reference_motion as motion  # noqa: E402
import motion_library as library_module  # noqa: E402

GENERATOR_VERSION = "play01-reference-generator-v2"
FAMILY_ANGLES = {"forward": 0.0, "strafe_left": math.pi / 2, "backward": math.pi, "strafe_right": -math.pi / 2}


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def nlerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    dot = np.sum(a * b, axis=-1, keepdims=True)
    b = np.where(dot < 0, -b, b)
    q = (1 - t) * a + t * b
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def sample_loop(clip: dict[str, Any], phase: float) -> tuple[np.ndarray, np.ndarray]:
    """Parent-local rotations and root position at a fractional phase (nlerp)."""
    rotations = clip["_rotations"]
    roots = clip["_roots"]
    count = len(rotations)
    x = (phase % 1.0) * count
    i = int(math.floor(x)) % count
    j = (i + 1) % count
    t = x - math.floor(x)
    return nlerp(rotations[i], rotations[j], t), (1 - t) * roots[i] + t * roots[j]


class ReferenceGenerator:
    def __init__(self, library: dict[str, Any], bound_speed: float, dt: float = 1.0 / 60.0,
                 start_blend_steps: int = 60, stop_blend_steps: int = 120, transition_steps: int = 30,
                 facing_rate: float = 1.0, idle_rest_steps: int = 0):
        self.library = library
        self.names = library["body_names"]
        self.parents = library["parent_indices"]
        self.dt = dt
        self.bound_speed = float(bound_speed)
        self.start_blend_steps = start_blend_steps
        self.stop_blend_steps = stop_blend_steps
        self.transition_steps = transition_steps
        self.facing_rate = facing_rate
        self.skeleton = {
            "parent_indices": self.parents,
            "local_offsets": np.asarray(library["creation_local_offsets"], dtype=float),
            "_levels": library_module.tree_levels(self.parents),
        }
        index = {name: i for i, name in enumerate(self.names)}
        self.pelvis = index["pelvis"]
        self.ends = {k: index[k] for k in ("left_hand", "right_hand", "left_foot", "right_foot")}
        self.clips: dict[str, dict[str, Any]] = {}
        for name, clip in library["clips"].items():
            entry = dict(clip)
            entry["_rotations"] = motion.normalize_quaternions(np.asarray(clip["parent_local_rotations_xyzw"], dtype=float))
            entry["_roots"] = np.asarray(clip["root_positions"], dtype=float)
            self.clips[name] = entry
        if "idle" not in self.clips:
            raise ValueError("library has no idle clip")
        self.idle_rotations = self.clips["idle"]["_rotations"].mean(axis=0)
        self.idle_rotations /= np.linalg.norm(self.idle_rotations, axis=-1, keepdims=True)
        self.idle_root = self.clips["idle"]["_roots"].mean(axis=0)
        forward = np.asarray(library["forward_axis_mapping"]["model_forward"], dtype=float)
        self.model_forward = forward / np.linalg.norm(forward)
        self.clip_confidence = {
            clip_name: {body: float(clip.get("style_confidence", {}).get(body, 0.0)) for body in self.names}
            for clip_name, clip in self.clips.items()
        }
        self.reset()

    # ------------------------------------------------------------ state

    def reset(self) -> None:
        self.phase = 0.0
        self.rate = 1.0
        self.walk_weight = 0.0
        self.walk_target = 0.0
        self.family_weights = {name: 0.0 for name in FAMILY_ANGLES}
        self.family_weights["forward"] = 1.0
        self.reference_yaw = 0.0
        self.previous_local: np.ndarray | None = None
        self.step_index = 0
        self.coverage = "idle"

    def state(self) -> dict[str, Any]:
        return {
            "phase": self.phase, "rate": self.rate, "walk_weight": self.walk_weight,
            "family_weights": dict(self.family_weights), "reference_yaw": self.reference_yaw,
            "step_index": self.step_index, "coverage": self.coverage,
            "previous_local": None if self.previous_local is None else self.previous_local.tolist(),
        }

    def load_state(self, state: dict[str, Any]) -> None:
        self.phase = float(state["phase"])
        self.rate = float(state["rate"])
        self.walk_weight = float(state["walk_weight"])
        self.family_weights = {k: float(v) for k, v in state["family_weights"].items()}
        self.reference_yaw = float(state["reference_yaw"])
        self.step_index = int(state["step_index"])
        self.coverage = str(state["coverage"])
        self.previous_local = None if state["previous_local"] is None else np.asarray(state["previous_local"], dtype=float)

    def identity(self) -> dict[str, Any]:
        return {
            "generator_version": GENERATOR_VERSION, "library_sha256": self.library["sha256"],
            "bound_speed": self.bound_speed, "dt": self.dt, "start_blend_steps": self.start_blend_steps,
            "stop_blend_steps": self.stop_blend_steps, "transition_steps": self.transition_steps,
            "facing_rate": self.facing_rate, "clips": sorted(self.clips),
            "style_confidence": {"composition": "weights of active pose clips only", "per_clip": self.clip_confidence},
            "phase_adaptation": "rate = 0.8*rate + 0.2*clamp(max(v_along,0)/bound_speed, 0.5, 1.5); phase += rate*dt/duration while walking",
        }

    # -------------------------------------------------------- generation

    def family_targets(self, travel_body: np.ndarray) -> tuple[dict[str, float], str]:
        """Angular blend weights over the adjacent travel families."""
        speed = float(np.linalg.norm(travel_body))
        if speed < 1e-6:
            return {name: 0.0 for name in FAMILY_ANGLES}, "idle"
        angle = math.atan2(-travel_body[0], travel_body[1])  # +Y forward, +X right: left strafe = +pi/2
        ordered = sorted(FAMILY_ANGLES.items(), key=lambda kv: kv[1])
        weights = {name: 0.0 for name in FAMILY_ANGLES}
        best = None
        for i, (name_a, angle_a) in enumerate(ordered):
            name_b, angle_b = ordered[(i + 1) % len(ordered)]
            span = (angle_b - angle_a) % (2 * math.pi)
            offset = (angle - angle_a) % (2 * math.pi)
            if offset <= span + 1e-9:
                t = offset / span
                weights[name_a] = 1 - t
                weights[name_b] = t
                best = (name_a, name_b, t)
                break
        if best is None:
            weights["forward"] = 1.0
            return weights, "forward"
        name_a, name_b, t = best
        if t < 0.02:
            return {k: (1.0 if k == name_a else 0.0) for k in weights}, name_a
        if t > 0.98:
            return {k: (1.0 if k == name_b else 0.0) for k in weights}, name_b
        return weights, f"blend:{name_a}+{name_b}"

    def step(self, travel_velocity: np.ndarray, facing: np.ndarray, measured_along_command_speed: float,
             yaw_rate: float = 0.0) -> dict[str, Any]:
        travel = np.asarray(travel_velocity, dtype=float)[:2]
        facing = np.asarray(facing, dtype=float)[:2]
        speed = float(np.linalg.norm(travel))
        # Facing: requested direction in model axes; zero means face travel.
        if np.linalg.norm(facing) > 1e-6:
            target_yaw = math.atan2(-facing[0], facing[1])
        elif speed > 1e-6:
            target_yaw = math.atan2(-travel[0], travel[1])
        else:
            target_yaw = self.reference_yaw
        delta = (target_yaw - self.reference_yaw + math.pi) % (2 * math.pi) - math.pi
        limit = self.facing_rate * self.dt
        self.reference_yaw += max(-limit, min(limit, delta))
        yaw_q = yaw_quaternion(self.reference_yaw)
        # Travel expressed in the facing (body) frame.
        c, s = math.cos(-self.reference_yaw), math.sin(-self.reference_yaw)
        travel_body = np.array([c * travel[0] - s * travel[1], s * travel[0] + c * travel[1]])
        targets, coverage = self.family_targets(travel_body)
        unsupported = []
        for name, weight in targets.items():
            if weight > 0 and name not in self.clips:
                unsupported.append(name)
        if abs(yaw_rate) > 1e-6 and speed < 1e-6:
            unsupported.append("turn_in_place")
        if unsupported:
            coverage = "unsupported:" + "+".join(sorted(unsupported))
            targets = {k: (v if k in self.clips else 0.0) for k, v in targets.items()}
            total = sum(targets.values())
            if total > 0:
                targets = {k: v / total for k, v in targets.items()}
            else:
                targets["forward"] = 1.0 if "forward" in self.clips else 0.0
        # Transition state: walk weight and family crossfade.
        self.walk_target = 1.0 if speed > 1e-6 and any(targets.values()) else 0.0
        blend_steps = self.start_blend_steps if self.walk_target > self.walk_weight else self.stop_blend_steps
        self.walk_weight += max(-1.0 / max(blend_steps, 1), min(1.0 / max(blend_steps, 1), self.walk_target - self.walk_weight))
        walk_weight = smoothstep(self.walk_weight)
        if any(targets.values()):
            for name in self.family_weights:
                target = targets.get(name, 0.0)
                self.family_weights[name] += max(-1.0 / self.transition_steps, min(1.0 / self.transition_steps, target - self.family_weights[name]))
            total = sum(self.family_weights.values())
            if total > 0:
                for name in self.family_weights:
                    self.family_weights[name] /= total
        # Phase clock (shared, bounded).
        if walk_weight > 0:
            target_rate = min(1.5, max(0.5, max(measured_along_command_speed, 0.0) / self.bound_speed))
            self.rate = 0.8 * self.rate + 0.2 * target_rate
            duration = sum(w * self.clips[n]["duration_seconds"] for n, w in self.family_weights.items() if w > 0 and n in self.clips)
            self.phase = (self.phase + self.rate * self.dt / max(duration, 1e-6)) % 1.0
        else:
            self.rate = 1.0
            self.phase = 0.0
        # Compose the walk reference from the family weights, then blend with idle.
        walk_local = self._walk_local(self.phase)
        walk_root = None
        for name, weight in self.family_weights.items():
            if weight <= 0 or name not in self.clips:
                continue
            _, root = sample_loop(self.clips[name], self.phase)
            walk_root = root * weight if walk_root is None else walk_root + root * weight
        if walk_root is None:
            walk_root = self.idle_root
        local = nlerp(self.idle_rotations, walk_local, walk_weight)
        root = (1 - walk_weight) * self.idle_root + walk_weight * walk_root
        confidence = self._active_confidence(walk_weight)
        # Root: reference heading applied to the pelvis rotation.
        local = local.copy()
        local[self.pelvis] = motion.multiply(yaw_q, local[self.pelvis])
        local /= np.linalg.norm(local, axis=-1, keepdims=True)
        positions, rotations = library_module.forward_kinematics(local[None], root[None], self.skeleton)
        positions, rotations = positions[0], rotations[0]
        if self.previous_local is None:
            velocities = np.zeros((len(self.names), 3))
        else:
            velocities = motion.rotation_vector(motion.multiply(local, motion.conjugate(self.previous_local))) / self.dt
        self.previous_local = local
        heading_inverse = motion.conjugate(yaw_q)
        end_effectors = {k: motion.rotate(heading_inverse, positions[i] - positions[self.pelvis]).tolist() for k, i in self.ends.items()}
        self.coverage = coverage
        self.step_index += 1
        return {
            "local_rotations": local, "local_angular_velocities": velocities,
            "model_positions": positions, "model_rotations": rotations,
            "root_relative_end_effectors": end_effectors, "reference_yaw": self.reference_yaw,
            "phase": self.phase, "rate": self.rate, "walk_weight": walk_weight,
            "family_weights": dict(self.family_weights), "coverage": coverage,
            "travel_body": travel_body.tolist(), "style_confidence": confidence,
        }

    def _active_confidence(self, walk_weight: float) -> dict[str, float]:
        """Propagate confidence through exactly the clips used by the pose blend.

        Missing and inactive clips contribute zero; in particular, a high-
        confidence channel in an unrelated clip can never restore a channel
        excluded by the active clip family.
        """
        idle = self.clip_confidence["idle"]
        active = [(name, weight) for name, weight in self.family_weights.items()
                  if weight > 0.0 and name in self.clip_confidence]
        active_total = sum(weight for _, weight in active)
        result = {}
        for body in self.names:
            walking = (sum(weight * self.clip_confidence[name][body] for name, weight in active) / active_total
                       if active_total > 0.0 else 0.0)
            result[body] = (1.0 - walk_weight) * idle[body] + walk_weight * walking
        return result

    def _walk_local(self, phase: float) -> np.ndarray:
        walk_local = None
        for name, w in self.family_weights.items():
            if w <= 0 or name not in self.clips:
                continue
            local, _ = sample_loop(self.clips[name], phase)
            if walk_local is None:
                walk_local, accumulated = local, w
            else:
                accumulated += w
                walk_local = nlerp(walk_local, local, w / accumulated)
        return self.idle_rotations if walk_local is None else walk_local

    def _lookahead_phase(self, offset_steps: int) -> float:
        duration = sum(w * self.clips[n]["duration_seconds"] for n, w in self.family_weights.items() if w > 0 and n in self.clips)
        if smoothstep(self.walk_weight) > 0 and duration > 0:
            return (self.phase + offset_steps * self.rate * self.dt / duration) % 1.0
        return self.phase

    def snapshot_local(self, offset_steps: int) -> tuple[np.ndarray, float]:
        """Parent-local reference rotations at the clock advanced by offset steps (pure)."""
        phase = self._lookahead_phase(offset_steps)
        local = nlerp(self.idle_rotations, self._walk_local(phase), smoothstep(self.walk_weight)).copy()
        local[self.pelvis] = motion.multiply(yaw_quaternion(self.reference_yaw), local[self.pelvis])
        local /= np.linalg.norm(local, axis=-1, keepdims=True)
        return local, phase

    def reference_snapshot(self, offset_steps: int) -> dict[str, Any]:
        """Lookahead reference at the current clock advanced by offset steps.

        Uses the current rate and weights without mutating state (prerecorded
        lookahead in the observation sense; it never reads future physics).
        """
        local, phase = self.snapshot_local(offset_steps)
        positions, _ = library_module.forward_kinematics(local[None], self.idle_root[None], self.skeleton)
        heading_inverse = motion.conjugate(yaw_quaternion(self.reference_yaw))
        ends = {k: motion.rotate(heading_inverse, positions[0][i] - positions[0][self.pelvis]) for k, i in self.ends.items()}
        return {"local_rotations": local, "phase": phase, "end_effectors": ends}


def load_library(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def preview(generator: ReferenceGenerator, script: list[dict[str, Any]], out_dir: Path, name: str) -> dict[str, Any]:
    """Run a command script, write a viewer-compatible bodies CSV and a PNG strip."""
    out_dir.mkdir(parents=True, exist_ok=True)
    generator.reset()
    rows, coverage_trace, phases = [], [], []
    step = 0
    frames = []
    locals_trace, velocities_trace, rates, walk_weights, active_trace, confidence_trace = [], [], [], [], [], []
    for segment in script:
        for _ in range(int(segment["steps"])):
            result = generator.step(np.asarray(segment["travel"], dtype=float), np.asarray(segment.get("facing", [0, 0]), dtype=float),
                                    float(segment.get("measured_speed", np.linalg.norm(segment["travel"]))), float(segment.get("yaw_rate", 0.0)))
            positions = result["model_positions"]
            for body in range(len(generator.names)):
                rows.append((step, body, *positions[body]))
            coverage_trace.append(result["coverage"])
            phases.append(result["phase"])
            frames.append(positions.copy())
            locals_trace.append(result["local_rotations"].copy())
            velocities_trace.append(result["local_angular_velocities"].copy())
            rates.append(float(result["rate"]))
            walk_weights.append(float(result["walk_weight"]))
            active_trace.append(float(np.linalg.norm(segment["travel"])) > 1e-6)
            confidence_trace.append(list(result["style_confidence"].values()))
            step += 1
    csv_path = out_dir / f"{name}-bodies.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        stream.write("step,body,x,y,z,vx,vy,vz,wx,wy,wz,qx,qy,qz,qw\n")
        for row in rows:
            stream.write("%d,%d,%.6f,%.6f,%.6f,0,0,0,0,0,0,0,0,0,1\n" % row)
    png_path = None
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        picks = np.linspace(0, len(frames) - 1, num=min(12, len(frames)), dtype=int)
        fig, axes = plt.subplots(2, len(picks), figsize=(1.6 * len(picks), 5.5))
        parents = generator.parents
        for column, frame_index in enumerate(picks):
            p = frames[frame_index]
            for row, (a, b, title) in enumerate(((1, 2, "side (y,z)"), (0, 2, "front (x,z)"))):
                ax = axes[row][column]
                for body, parent in enumerate(parents):
                    if parent >= 0:
                        ax.plot([p[parent][a], p[body][a]], [p[parent][b], p[body][b]], "k-", linewidth=1)
                ax.set_aspect("equal")
                ax.set_xlim(p[generator.pelvis][a] - 60, p[generator.pelvis][a] + 60)
                ax.set_ylim(-5, 140)
                ax.set_xticks([])
                ax.set_yticks([])
                if row == 0:
                    ax.set_title(f"t={frame_index / 60:.2f}s\n{coverage_trace[frame_index][:14]}", fontsize=7)
        fig.suptitle(f"{name}: reference generator preview (fixed captured geometry, no physics)", fontsize=9)
        png_path = out_dir / f"{name}.png"
        fig.savefig(png_path, dpi=110)
        plt.close(fig)
    except Exception as error:  # preview image is optional
        png_path = None
        print("preview image skipped:", error, file=sys.stderr)
    counts: dict[str, int] = {}
    for c in coverage_trace:
        counts[c] = counts.get(c, 0) + 1
    rotations = np.asarray(locals_trace)
    velocities = np.asarray(velocities_trace)
    positions = np.asarray(frames)
    quaternion_norm_error = float(np.max(np.abs(np.linalg.norm(rotations, axis=-1) - 1.0)))
    length_errors = []
    offsets = np.asarray(generator.library["creation_local_offsets"], dtype=float)
    for body, parent in enumerate(generator.parents):
        if parent >= 0:
            length_errors.append(np.abs(np.linalg.norm(positions[:, body] - positions[:, parent], axis=-1) - np.linalg.norm(offsets[body])))
    bone_length_error = float(np.max(length_errors)) if length_errors else 0.0
    reconstructed = motion.rotation_vector(motion.multiply(rotations[1:], motion.conjugate(rotations[:-1]))) / generator.dt
    velocity_error = float(np.max(np.abs(reconstructed - velocities[1:]))) if len(rotations) > 1 else 0.0
    active_indices = [i for i, active in enumerate(active_trace) if active]
    inactive_after = [i for i in range(1, len(active_trace)) if not active_trace[i] and active_trace[i - 1]]
    first_active = active_indices[0] if active_indices else None
    stop_index = inactive_after[0] if inactive_after else None
    start_weights = walk_weights[first_active:stop_index] if first_active is not None else []
    stop_weights = walk_weights[stop_index:] if stop_index is not None else []
    phase_deltas = [((phases[i] - phases[i - 1]) % 1.0) for i in active_indices if i > 0]
    finite = bool(np.isfinite(rotations).all() and np.isfinite(velocities).all() and np.isfinite(positions).all()
                  and np.isfinite(rates).all() and np.isfinite(confidence_trace).all())
    checks = {
        "finite": finite,
        "quaternion_norm_within_1e-6": quaternion_norm_error <= 1e-6,
        "captured_geometry_within_1e-6": bone_length_error <= 1e-6,
        "reference_velocity_matches_finite_difference_1e-6": velocity_error <= 1e-6,
        "phase_advances_while_active": bool(phase_deltas and min(phase_deltas) > 0.0),
        "rate_within_declared_bounds": bool(rates and min(rates) >= 0.5 - 1e-9 and max(rates) <= 1.5 + 1e-9),
        "start_weight_monotone": all(b >= a - 1e-12 for a, b in zip(start_weights, start_weights[1:])),
        "stop_weight_monotone": all(b <= a + 1e-12 for a, b in zip(stop_weights, stop_weights[1:])),
        "idle_start_forward_stop_covered": set(counts).issubset({"idle", "forward"}) and "idle" in counts and "forward" in counts,
        "confidence_bounded": bool(confidence_trace and np.min(confidence_trace) >= 0.0 and np.max(confidence_trace) <= 1.0),
    }
    diagnostics = {
        "maximum_quaternion_norm_error": quaternion_norm_error,
        "maximum_bone_length_error": bone_length_error,
        "maximum_reference_velocity_reconstruction_error": velocity_error,
        "rate_minimum": min(rates) if rates else None, "rate_maximum": max(rates) if rates else None,
        "phase_delta_minimum_active": min(phase_deltas) if phase_deltas else None,
        "phase_delta_maximum_active": max(phase_deltas) if phase_deltas else None,
        "walk_weight_first_active": walk_weights[first_active] if first_active is not None else None,
        "walk_weight_end_start": walk_weights[stop_index - 1] if stop_index else None,
        "walk_weight_end_stop": walk_weights[-1] if walk_weights else None,
    }
    return {"name": name, "steps": step, "bodies_csv": str(csv_path), "png": None if png_path is None else str(png_path),
            "coverage_counts": counts, "phase_final": phases[-1] if phases else None,
            "bodies_csv_sha256": motion.sha256(csv_path), "checks": checks, "diagnostics": diagnostics,
            "contract_passed": all(checks.values())}


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bound-speed", type=float, default=None, help="reference forward speed; defaults to the library's measured vanilla speed")
    parser.add_argument("--visual-review-note", default=None, help="record the completed human/agent visual inspection note")
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    library = load_library(args.library)
    bound = args.bound_speed if args.bound_speed is not None else library["measured_vanilla_forward_speed"]["speed_units_per_second"]
    generator = ReferenceGenerator(library, bound)
    scripts = {f"forward-{int(speed)}-idle-start-forward-stop": [
        {"travel": [0, 0], "steps": 60},
        {"travel": [0, speed], "facing": [0, 1], "steps": 360, "measured_speed": speed},
        {"travel": [0, 0], "facing": [0, 1], "steps": 120},
    ] for speed in (45.0, 60.0)}
    previews = [preview(generator, script, args.output, name) for name, script in scripts.items()]
    numeric_passed = all(preview_["contract_passed"] for preview_ in previews)
    review = {"schema": "play01_forward_reference_review_v2", "tracker_task": "PLAY-01", "generator": generator.identity(),
              "generator_source_sha256": motion.sha256(Path(__file__)), "library_file_sha256": motion.sha256(args.library),
              "reviewed_scope": {"commands": ["idle", "start", "forward", "stop"], "speeds_units_per_second": [45.0, 60.0]},
              "previews": previews,
              "numeric_contract_passed": numeric_passed,
              "visual_review": {"completed": args.visual_review_note is not None, "note": args.visual_review_note},
              "review_status": "reviewed_for_forward_reference_contract" if numeric_passed and args.visual_review_note else "visual_review_pending",
              "scope_limit": "reference geometry/phase/rate/velocity/transitions only; no physics or balance claim; turns remain out of scope"}
    (args.output / "generator-review.json").write_text(json.dumps(review, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"task": args.tracker_task, "generator": GENERATOR_VERSION, "bound_speed": bound,
                      "previews": [{k: p[k] for k in ("name", "steps", "coverage_counts", "png")} for p in previews]}, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
