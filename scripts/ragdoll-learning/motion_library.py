#!/usr/bin/env python3
"""PLAY-01 motion library: native animation channels on fixed captured geometry.

This module turns the animation-target channel recorded in native fixed-step
replays (one per ordinary-input case) into coherent, hashed reference clips
for the tracking experiment. It is content preparation and diagnostics; it
launches no physics, trains nothing and records no tracker verdict.

Pipeline per case:
  1. Decode the replay with ``ragdollcore-walksearch --dump-animation`` (the
     powered actor's animation target channel, model space, 60 Hz).
  2. Keep the authored parent-local ROTATIONS. Discard the exported leg
     TRANSLATIONS, which the native adapter mirror-averages, and rebuild every
     bone position by forward kinematics through the captured creation
     skeleton's fixed bone offsets (context-symP). Report how far the export
     deviated from rigid geometry for every bone.
  3. Cut clips by the harness protocol segments, measure the rotational
     period in the steady travel window, pick a kinematic phase anchor and
     check the loop seam.
  4. Check every joint against the profile's derived constraint frames and
     limits (approximate cone check), derive a frozen per-joint style
     confidence and record missing coverage.

Vanilla clips are optional style material. They carry no physical contact
truth, prove no dynamic feasibility and do not define acceptable balance.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reference_motion as motion  # noqa: E402

SAMPLE_HZ = 60.0
FAMILIES = ("idle", "forward", "backward", "strafe_left", "strafe_right")
# Harness case name -> (command family, diagonal/turn note)
CASE_FAMILIES = {
    "PLAY-01 unarmed no input": "idle",
    "PLAY-01 unarmed forward": "forward",
    "PLAY-01 unarmed backward": "backward",
    "PLAY-01 unarmed strafe left": "strafe_left",
    "PLAY-01 unarmed strafe right": "strafe_right",
    "PLAY-01 unarmed diagonal left": "forward_left",
    "PLAY-01 unarmed diagonal right": "forward_right",
    "PLAY-01 unarmed moving turn": "moving_turn",
    "PLAY-01 unarmed reversal": "reversal",
    "PLAY-01 unarmed start stop": "start_stop",
}
REQUIRED_COVERAGE = [
    "idle", "forward", "backward", "strafe_left", "strafe_right",
    "forward_left", "forward_right", "backward_left", "backward_right",
    "turn_in_place_left", "turn_in_place_right", "moving_turn_left", "moving_turn_right",
    "start", "stop", "reversal", "changes_between_travel_directions",
]


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8"))


# --------------------------------------------------------------------- skeleton


def read_creation_skeleton(context_path: Path) -> dict[str, Any]:
    """Header and 20-bone creation skeleton from a replay_context creation file."""
    lines = context_path.read_text(encoding="utf-8-sig").splitlines()
    header = shlex.split(lines[0], posix=True)
    version, profile_id, profile_hash, warmup = int(header[0]), header[1], int(header[2]), int(header[3])
    total_mass, uniform_scale, ground_height = (float(v) for v in lines[1].split())
    bone_count = int(lines[6].strip())
    names, parents, translations, rotations = [], [], [], []
    for line in lines[7:7 + bone_count]:
        fields = shlex.split(line, posix=True)
        names.append(fields[0])
        parents.append(int(fields[1]))
        translations.append([float(v) for v in fields[2:5]])
        rotations.append([float(v) for v in fields[5:9]])
    t = np.asarray(translations, dtype=float)
    q = motion.normalize_quaternions(np.asarray(rotations, dtype=float))
    offsets = t.copy()
    for child, parent in enumerate(parents):
        if parent >= 0:
            offsets[child] = motion.rotate(motion.conjugate(q[parent]), t[child] - t[parent])
    return {
        "path": str(context_path), "sha256": motion.sha256(context_path), "format_version": version,
        "profile_id": profile_id, "profile_hash": profile_hash, "warmup_frames": warmup,
        "total_mass": total_mass, "uniform_scale": uniform_scale, "ground_height": ground_height,
        "body_names": names, "parent_indices": parents, "bind_translations": t, "bind_rotations": q,
        "local_offsets": offsets,
    }


def tree_levels(parents: list[int]) -> list[np.ndarray]:
    """Body indices grouped by depth so kinematics runs one vector op per level."""
    depth = [0] * len(parents)
    for body, parent in enumerate(parents):
        depth[body] = 0 if parent < 0 else depth[parent] + 1
    levels = []
    for d in range(1, max(depth) + 1):
        levels.append(np.asarray([i for i, value in enumerate(depth) if value == d], dtype=int))
    return levels


def forward_kinematics(local_rotations: np.ndarray, root_positions: np.ndarray, skeleton: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Model-space positions/rotations from parent-local rotations on fixed offsets.

    local_rotations: (F, B, 4) xyzw, root entry is the model-space root rotation.
    root_positions: (F, 3) model-space root translation.
    """
    parents = np.asarray(skeleton["parent_indices"], dtype=int)
    offsets = np.asarray(skeleton["local_offsets"], dtype=float)
    levels = skeleton.get("_levels")
    if levels is None:
        levels = tree_levels(list(parents))
        skeleton["_levels"] = levels
    frames, bodies = local_rotations.shape[:2]
    positions = np.zeros((frames, bodies, 3))
    rotations = np.zeros((frames, bodies, 4))
    root = int(np.flatnonzero(parents < 0)[0])
    rotations[:, root] = local_rotations[:, root]
    positions[:, root] = root_positions
    for level in levels:
        parent_index = parents[level]
        parent_rotation = rotations[:, parent_index]
        rotations[:, level] = motion.multiply(parent_rotation, local_rotations[:, level])
        positions[:, level] = positions[:, parent_index] + motion.rotate(parent_rotation, offsets[level])
    return positions, motion.normalize_quaternions(rotations)


def parent_local_rotations(model_rotations: np.ndarray, parents: list[int]) -> np.ndarray:
    local = model_rotations.copy()
    for child, parent in enumerate(parents):
        if parent >= 0:
            local[:, child] = motion.multiply(motion.conjugate(model_rotations[:, parent]), model_rotations[:, child])
    return motion.continuous_quaternions(local)


# ------------------------------------------------------------------ constraints


def derive_constraint_frames(profile: dict[str, Any], skeleton: dict[str, Any]) -> list[dict[str, Any] | None]:
    """Mirror builder.cpp createConstraintFrame (derive_frame) per profile body.

    Frames are unit quaternions mapping constraint axes (x = twist, y = plane,
    z = normal) into the parent (frame1) and child (frame2) bone axes.
    """
    names = skeleton["body_names"]
    index = {name: i for i, name in enumerate(names)}
    t = skeleton["bind_translations"]
    q = skeleton["bind_rotations"]
    result: list[dict[str, Any] | None] = []
    for body in profile["bodies"]:
        constraint = body.get("constraint")
        if not constraint:
            result.append(None)
            continue
        child = index[body["bone"]]
        parent = index[next(b for b in profile["bodies"] if b["name"] == body["parent"])["bone"]]
        if not constraint.get("derive_frame", False):
            raise ValueError("only derived constraint frames are supported by this audit")
        axis_bone = constraint.get("frame_axis_bone")
        model_offset = t[child] - t[parent]
        if axis_bone:
            axis_child = index[next(b for b in profile["bodies"] if b["name"] == axis_bone)["bone"]]
            model_axis = t[axis_child] - t[child]
        else:
            model_axis = model_offset
        twist = model_axis / np.linalg.norm(model_axis)
        reference = np.array([0.0, 0.0, 1.0]) if abs(twist[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        plane = np.cross(twist, reference)
        plane /= np.linalg.norm(plane)
        normal = np.cross(twist, plane)
        # Rotation matrix with columns twist, plane, normal expressed in model
        # axes, then re-expressed in each bone's local axes.
        basis = np.stack([twist, plane, normal], axis=1)
        parent_inverse = motion.conjugate(q[parent])
        child_inverse = motion.conjugate(q[child])
        frame1 = matrix_to_quaternion(rotation_matrix(parent_inverse) @ basis)
        frame2 = matrix_to_quaternion(rotation_matrix(child_inverse) @ basis)
        hinge_axis_parent_local = motion.rotate(parent_inverse, np.cross(plane, twist))
        result.append({
            "body": child, "parent": parent, "type": constraint.get("type", "swing_twist"),
            "frame1": frame1, "frame2": frame2,
            "hinge_axis_parent_local": hinge_axis_parent_local / np.linalg.norm(hinge_axis_parent_local),
            "limits": {k: constraint.get(k) for k in ("normal_half_cone_angle", "plane_half_cone_angle", "twist_min_angle",
                                                        "twist_max_angle", "limit_min_angle", "limit_max_angle")},
            "maximum_torque": float(constraint["motor"]["maximum_torque"]),
        })
    return result


def rotation_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(m: np.ndarray) -> np.ndarray:
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def joint_limit_audit(local_q: np.ndarray, frames: list[dict[str, Any] | None], bind_local: np.ndarray) -> list[dict[str, Any]]:
    """Approximate per-joint limit occupancy of a clip. Diagnostic only.

    Swing-twist: constraint-space rotation q_c = conj(frame1) * q_rel * frame2,
    decomposed into twist about x and swing about y (plane) / z (normal).
    Jolt uses an elliptic cone; the report gives the ellipse occupancy
    ((sy/plane)^2 + (sz/normal)^2, <= 1 inside) and the twist range.
    Hinge: signed angle of q_rel relative to the bind relative rotation about
    the derived hinge axis.
    """
    report = []
    for frame in frames:
        if frame is None:
            continue
        body = frame["body"]
        q_rel = local_q[:, body]
        if frame["type"] == "hinge":
            delta = motion.multiply(q_rel, motion.conjugate(bind_local[body]))
            vectors = motion.rotation_vector(delta)
            angle = vectors @ frame["hinge_axis_parent_local"]
            off_axis = np.linalg.norm(vectors - np.outer(angle, frame["hinge_axis_parent_local"]), axis=-1)
            lo, hi = frame["limits"]["limit_min_angle"], frame["limits"]["limit_max_angle"]
            inside = (angle >= lo - 1e-3) & (angle <= hi + 1e-3)
            report.append({"body": body, "type": "hinge", "angle_min": float(angle.min()), "angle_max": float(angle.max()),
                           "limit_min": lo, "limit_max": hi, "inside_fraction": float(inside.mean()),
                           "off_axis_rotation_max_radians": float(off_axis.max())})
            continue
        q_c = motion.multiply(motion.multiply(motion.conjugate(frame["frame1"]), q_rel), frame["frame2"])
        q_c = np.where(q_c[..., 3:] < 0, -q_c, q_c)
        # Twist about x: normalize (x, 0, 0, w); swing = q_c * conj(twist).
        twist = np.zeros_like(q_c)
        twist[:, 0], twist[:, 3] = q_c[:, 0], q_c[:, 3]
        norm = np.linalg.norm(twist, axis=-1, keepdims=True)
        twist = np.where(norm > 1e-9, twist / np.maximum(norm, 1e-9), np.array([0, 0, 0, 1.0]))
        swing = motion.multiply(q_c, motion.conjugate(twist))
        twist_angle = 2 * np.arctan2(twist[:, 0], twist[:, 3])
        swing_vec = motion.rotation_vector(swing)
        plane_angle, normal_angle = swing_vec[:, 1], swing_vec[:, 2]
        limits = frame["limits"]
        plane_limit, normal_limit = limits["plane_half_cone_angle"], limits["normal_half_cone_angle"]
        occupancy = (plane_angle / plane_limit) ** 2 + (normal_angle / normal_limit) ** 2
        twist_inside = (twist_angle >= limits["twist_min_angle"] - 1e-3) & (twist_angle <= limits["twist_max_angle"] + 1e-3)
        report.append({
            "body": body, "type": "swing_twist", "twist_min": float(twist_angle.min()), "twist_max": float(twist_angle.max()),
            "twist_limits": [limits["twist_min_angle"], limits["twist_max_angle"]],
            "twist_inside_fraction": float(twist_inside.mean()),
            "swing_ellipse_occupancy_max": float(occupancy.max()), "swing_ellipse_occupancy_mean": float(occupancy.mean()),
            "swing_inside_fraction": float((occupancy <= 1.0 + 1e-3).mean()),
            "plane_angle_max_abs": float(np.abs(plane_angle).max()), "normal_angle_max_abs": float(np.abs(normal_angle).max()),
        })
    return report


# ----------------------------------------------------------------------- clips


def dump_case(exe: Path, replay: Path, csv_path: Path) -> dict[str, Any]:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    exe_hash = motion.sha256(exe)
    command = [str(exe), "--dump-animation", str(replay), str(csv_path)]
    completed = subprocess.run(command, text=True, capture_output=True, check=True)
    if motion.sha256(exe) != exe_hash:
        raise RuntimeError("decoder changed during conversion")
    return {"command": command, "stdout": completed.stdout.strip(), "decoder_exe_sha256": exe_hash,
            "replay_sha256": motion.sha256(replay), "csv_sha256": motion.sha256(csv_path)}


def period_search(local_q: np.ndarray, joints: list[int], window: tuple[int, int], lags: range) -> dict[str, Any]:
    start, end = window
    curve = []
    for lag in lags:
        a = local_q[start:end - lag, joints]
        b = local_q[start + lag:end, joints]
        distance = np.linalg.norm(motion.rotation_vector(motion.multiply(a, motion.conjugate(b))), axis=-1)
        curve.append((lag, float(np.sqrt(np.mean(distance ** 2)))))
    values = np.array([v for _, v in curve])
    minima = [curve[i] for i in range(1, len(curve) - 1) if values[i] < values[i - 1] and values[i] <= values[i + 1]]
    minima.sort(key=lambda item: item[1])
    return {"window": [start, end], "lags": [lag for lag, _ in curve], "rms_radians": [v for _, v in curve], "local_minima": minima[:6]}


def kinematic_phase_anchor(positions: np.ndarray, rotations: np.ndarray, skeleton: dict[str, Any], pelvis: int,
                           left_foot: int, forward_model: np.ndarray, window: tuple[int, int], period: int) -> dict[str, Any]:
    """Frame in the window where the left foot is furthest ahead of the pelvis.

    A kinematic proxy for left heel strike; the exported channel carries no
    contact annotation, so this is not a contact label.
    """
    start, end = window
    heading = forward_model / np.linalg.norm(forward_model)
    relative = (positions[start:end, left_foot] - positions[start:end, pelvis]) @ heading
    best = int(np.argmax(relative[: max(period, 1)]))
    return {"frame": start + best, "left_foot_forward_offset": float(relative[best]),
            "definition": "left foot furthest along the heading relative to the pelvis in the first period of the window; kinematic proxy, not contact"}


def cut_cycle(local_q: np.ndarray, root_positions: np.ndarray, start: int, count: int) -> dict[str, np.ndarray]:
    return {"local_rotations": local_q[start:start + count].copy(), "root_positions": root_positions[start:start + count].copy()}


def seam_report(local_q: np.ndarray, positions: np.ndarray) -> dict[str, float]:
    dt_ = 1.0 / SAMPLE_HZ
    seam = motion.rotation_vector(motion.multiply(local_q[0], motion.conjugate(local_q[-1])))
    interior = motion.rotation_vector(motion.multiply(local_q[1:], motion.conjugate(local_q[:-1])))
    return {
        "seam_joint_rotation_step_max_radians": float(np.linalg.norm(seam, axis=-1).max()),
        "interior_joint_rotation_step_max_radians": float(np.linalg.norm(interior, axis=-1).max()),
        "seam_joint_angular_speed_max": float(np.linalg.norm(seam, axis=-1).max() / dt_),
        "seam_root_relative_position_step_max": float(np.linalg.norm((positions[0] - positions[0, :1]) - (positions[-1] - positions[-1, :1]), axis=-1).max()),
    }


def style_confidence(audit: list[dict[str, Any]], geometry_deviation: dict[str, float], names: list[str]) -> dict[str, float]:
    """Frozen per-joint confidence in [0, 1] from limit occupancy and rigidity.

    1.0: >= 90 percent of frames inside the derived limits.
    0.5: 50-90 percent inside (the seam clamps targets to the limits, so a
         channel that leaves them part of the time is kept at half weight).
    0.0: below 50 percent inside (channel excluded from style terms).
    The rule is deterministic and recorded; it is not a quality judgement of
    the original animation.
    """
    result = {}
    for entry in audit:
        name = names[entry["body"]]
        inside = entry["inside_fraction"] if entry["type"] == "hinge" else min(entry["twist_inside_fraction"], entry["swing_inside_fraction"])
        result[name] = 1.0 if inside >= 0.90 else 0.5 if inside >= 0.50 else 0.0
    return result


# ------------------------------------------------------------------ pipeline


def case_frame_ranges(anchor: dict[str, Any]) -> dict[str, tuple[int, int]]:
    """Inclusive-exclusive frame ranges from the harness protocol segments."""
    segments = sorted(anchor["protocol"]["segments"].items(), key=lambda kv: int(kv[0]))
    ranges = {}
    cursor = 0
    for _, segment in segments:
        count = int(round(float(segment["duration"]) * SAMPLE_HZ))
        ranges[segment["segment"]] = (cursor, cursor + count)
        cursor += count
    return ranges


def load_case_summary(run_dir: Path) -> dict[str, Any]:
    summaries = list(run_dir.glob("*.player-diagnostics.summary.json"))
    if len(summaries) != 1:
        raise FileNotFoundError(f"expected one diagnostics summary in {run_dir}")
    return json.loads(summaries[0].read_text(encoding="utf-8-sig"))


def read_anchor(run_dir: Path, case: str) -> dict[str, Any]:
    import gzip
    path = next(run_dir.glob("*.player-diagnostics.jsonl.gz"))
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("kind") == "anchor" and record.get("test") == case:
                return record
    raise KeyError(case)


def measure_vanilla_forward_speed(run_dir: Path, case: str = "PLAY-01 vanilla forward") -> dict[str, Any]:
    """Horizontal travel speed of the ordinary animated actor during 'travel'."""
    import gzip
    path = next(run_dir.glob("*.player-diagnostics.jsonl.gz"))
    positions, times = [], []
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("kind") == "frame" and record.get("test") == case and record.get("segment") == "travel":
                positions.append(record["player_position"][:2])
                times.append(record["simulation_time"])
    if len(positions) < 2:
        raise ValueError("no vanilla travel frames")
    p = np.asarray(positions, dtype=float)
    t = np.asarray(times, dtype=float)
    # Use the steady interior (drop the first and last 10 percent) so the
    # start/stop blends do not bias the cruise speed.
    lo, hi = int(len(p) * 0.1), int(len(p) * 0.9)
    displacement = np.linalg.norm(p[hi] - p[lo])
    return {"case": case, "frames": len(p), "interior_frames": [lo, hi],
            "speed_units_per_second": float(displacement / (t[hi] - t[lo])),
            "whole_segment_speed": float(np.linalg.norm(p[-1] - p[0]) / (t[-1] - t[0])),
            "diagnostics_sha256": motion.sha256(path)}


def build_library(args: argparse.Namespace) -> dict[str, Any]:
    run_dir = Path(args.run_dir)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    skeleton = read_creation_skeleton(Path(args.context))
    profile = yaml.safe_load(Path(args.profile).read_text(encoding="utf-8"))
    profile_names = [b["name"] for b in profile["bodies"]]
    if profile_names != skeleton["body_names"]:
        raise ValueError("profile body order differs from the creation skeleton")
    names = skeleton["body_names"]
    parents = skeleton["parent_indices"]
    index = {name: i for i, name in enumerate(names)}
    pelvis, left_foot, right_foot = index["pelvis"], index["left_foot"], index["right_foot"]
    frames = derive_constraint_frames(profile, skeleton)
    bind_local = parent_local_rotations(skeleton["bind_rotations"][None], parents)[0]
    # Actor forward in model space is +Y (walksearch requests (0, v, 0));
    # pelvis-local forward is recorded so the heading frame never assumes it.
    forward_model = np.array([0.0, 1.0, 0.0])
    pelvis_local_forward = motion.rotate(motion.conjugate(skeleton["bind_rotations"][pelvis]), forward_model)

    summary = load_case_summary(run_dir)
    exe = Path(args.exe)
    clips: dict[str, Any] = {}
    cases: dict[str, Any] = {}
    for case, family in CASE_FAMILIES.items():
        entry = summary["tests"].get(case)
        if not entry or not entry.get("replay", {}).get("present"):
            cases[case] = {"family": family, "status": "replay missing"}
            continue
        replay = Path(entry["replay"]["output_path"])
        csv_path = out / "csv" / (family + ".csv")
        dumped = dump_case(exe, replay, csv_path)
        data = motion.read_animation_csv(csv_path, len(names))
        model_q = data["rotations"]
        local_q = parent_local_rotations(model_q, parents)
        root_positions = data["positions"][:, pelvis]
        positions, rotations = forward_kinematics(local_q, root_positions, skeleton)
        deviation = np.linalg.norm(positions - data["positions"], axis=-1)
        geometry = {names[b]: float(deviation[:, b].max()) for b in range(len(names))}
        anchor = read_anchor(run_dir, case)
        ranges = case_frame_ranges(anchor)
        case_record: dict[str, Any] = {
            "family": family, "replay": str(replay), "frames": int(len(model_q)), "protocol_ranges": ranges,
            "decode": dumped, "export_geometry_deviation_max_units": geometry,
            "export_geometry_note": "distance between exported bone origins and fixed-offset forward kinematics of the same rotations; legs are mirror-averaged by the native adapter",
        }
        clip_specs = []
        if family == "idle":
            lo, hi = ranges["idle"]
            clip_specs.append(("idle", lo + 60, min(hi, lo + 60 + 120)))  # skip settle, keep two seconds
        elif family in ("forward", "backward", "strafe_left", "strafe_right"):
            travel = ranges["travel"]
            steady = (travel[0] + 30, travel[1] - 20)
            leg_joints = [index[n] for n in ("left_thigh", "left_calf", "left_foot", "right_thigh", "right_calf", "right_foot")]
            periods = period_search(local_q, leg_joints, steady, range(30, 150))
            if not periods["local_minima"]:
                case_record["status"] = "no rotational period found"
                cases[case] = case_record
                continue
            # Prefer the shortest lag whose recurrence is within twice the best
            # minimum: a strafe's two half-steps recur at both L and 2L.
            best_rms = periods["local_minima"][0][1]
            period = int(min(lag for lag, rms in periods["local_minima"] if rms <= 2.0 * best_rms))
            periods["selection_rule"] = "shortest local minimum with rms <= 2 * best rms"
            case_record["period_search"] = periods
            case_record["period_frames"] = period
            anchor_frame = kinematic_phase_anchor(positions, rotations, skeleton, pelvis, left_foot, forward_model, (steady[0] + period, steady[1]), period)
            case_record["phase_anchor"] = anchor_frame
            clip_specs.append((family, anchor_frame["frame"], anchor_frame["frame"] + period))
            # Start/stop transitions from the same capture, as authored blends
            # between idle and the walk: keep them for review, not for tracking.
            case_record["transition_windows"] = {"start": [travel[0], travel[0] + 90], "stop": [ranges["stop"][0], ranges["stop"][1]]}
        else:
            case_record["status"] = "reviewed for coverage only"
        for clip_name, start, end in clip_specs:
            cycle = cut_cycle(local_q, root_positions, start, end - start)
            cycle_positions, cycle_rotations = forward_kinematics(cycle["local_rotations"], cycle["root_positions"], skeleton)
            audit = joint_limit_audit(cycle["local_rotations"], frames, bind_local)
            velocities = motion.angular_velocity(cycle["local_rotations"], 1.0 / SAMPLE_HZ)
            clip = {
                "name": clip_name, "case": case, "source_frames": [start, end], "frame_count": end - start,
                "duration_seconds": (end - start) / SAMPLE_HZ, "loop": clip_name != "idle",
                "parent_local_rotations_xyzw": cycle["local_rotations"].tolist(),
                "parent_local_angular_velocities": velocities.tolist(),
                "root_positions": cycle["root_positions"].tolist(),
                "model_positions": cycle_positions.tolist(), "model_rotations_xyzw": cycle_rotations.tolist(),
                "seam": seam_report(cycle["local_rotations"], cycle_positions) if clip_name != "idle" else None,
                "joint_limit_audit": audit,
                "style_confidence": style_confidence(audit, geometry, names),
                "csv_sha256": dumped["csv_sha256"], "replay_sha256": dumped["replay_sha256"],
            }
            clip["sha256"] = sha256_json({k: v for k, v in clip.items() if k not in ("sha256",)})
            clips[clip_name] = clip
        cases[case] = case_record

    # Diagonal, turn and reversal captures: record what vanilla actually played
    # by comparing their travel-window rotations with the forward clip family.
    coverage = {}
    forward_clip = clips.get("forward")
    for case, record in cases.items():
        family = record["family"]
        if family in ("forward_left", "forward_right", "moving_turn", "reversal", "start_stop") and "replay" in record:
            csv_path = out / "csv" / (family + ".csv")
            data = motion.read_animation_csv(csv_path, len(names))
            local_q = parent_local_rotations(data["rotations"], parents)
            travel = record["protocol_ranges"].get("travel") or next(iter(record["protocol_ranges"].values()))
            lo, hi = travel[0] + 60, travel[1] - 20
            if forward_clip is not None and hi > lo:
                fq = np.asarray(forward_clip["parent_local_rotations_xyzw"])
                leg_joints = [index[n] for n in ("left_thigh", "left_calf", "right_thigh", "right_calf")]
                best = None
                for offset in range(len(fq)):
                    tiled = np.concatenate([fq[offset:], fq[:offset]])
                    reps = int(np.ceil((hi - lo) / len(fq))) + 1
                    seq = np.concatenate([tiled] * reps)[: hi - lo]
                    d = np.linalg.norm(motion.rotation_vector(motion.multiply(local_q[lo:hi, leg_joints], motion.conjugate(seq[:, leg_joints]))), axis=-1)
                    rms = float(np.sqrt(np.mean(d ** 2)))
                    if best is None or rms < best[1]:
                        best = (offset, rms)
                record["forward_clip_match"] = {"best_phase_offset": best[0], "leg_rotation_rms_radians": best[1],
                                                "interpretation": "small values mean vanilla reused the forward walk for this input"}
    for family in REQUIRED_COVERAGE:
        if family in clips:
            coverage[family] = {"status": "content_ready", "source": "native capture of the vanilla clip", "clip": family}
        elif family in ("forward_left", "forward_right"):
            match = next((r.get("forward_clip_match") for r in cases.values() if r["family"] == family), None)
            coverage[family] = {"status": "synthesis_required", "source": "vanilla plays the forward walk for diagonal input (character.cpp strafing rule); blend of forward and strafe clips proposed",
                                "vanilla_capture_match": match}
        elif family in ("backward_left", "backward_right"):
            coverage[family] = {"status": "synthesis_required", "source": "blend of backward and strafe clips proposed; no vanilla capture exists"}
        elif family in ("turn_in_place_left", "turn_in_place_right"):
            coverage[family] = {"status": "missing_content", "source": "vanilla has turnleft/turnright clips; no harness case captured them. Acquisition plan: add a turn-in-place case to test_play01_walking and re-run the native capture."}
        elif family in ("moving_turn_left", "moving_turn_right"):
            coverage[family] = {"status": "synthesis_required", "source": "vanilla plays the forward walk while yawing; generator slews the reference heading at the requested yaw rate"}
        elif family in ("start", "stop"):
            coverage[family] = {"status": "content_ready", "source": "smoothstep blend between idle and the walk clip per the command protocol; authored start/stop windows retained for review"}
        elif family in ("reversal", "changes_between_travel_directions"):
            coverage[family] = {"status": "synthesis_required", "source": "generator crossfades between clip families with phase preserved; unreviewed"}
        else:
            coverage[family] = {"status": "missing_content", "source": "no capture"}

    vanilla_dir = Path(args.vanilla_run_dir) if args.vanilla_run_dir else run_dir
    vanilla = measure_vanilla_forward_speed(vanilla_dir) if not args.skip_vanilla else None
    if vanilla is not None:
        vanilla["run_dir"] = str(vanilla_dir)
        vanilla["run_manifest_sha256"] = motion.sha256(vanilla_dir / "run-manifest.json")
    library = {
        "schema": "play01_motion_library_v1", "tracker_task": "PLAY-01",
        "scope": "content preparation and diagnostics; no physics, training or tracker verdict",
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "run_dir": str(run_dir), "run_manifest_sha256": motion.sha256(run_dir / "run-manifest.json"),
        "decoder_exe": str(exe), "decoder_exe_sha256": motion.sha256(exe),
        "creation_context": {k: v for k, v in skeleton.items() if k not in ("bind_translations", "bind_rotations", "local_offsets")},
        "creation_bind_translations": skeleton["bind_translations"].tolist(),
        "creation_bind_rotations_xyzw": skeleton["bind_rotations"].tolist(),
        "creation_local_offsets": skeleton["local_offsets"].tolist(),
        "profile_path": str(Path(args.profile)), "profile_sha256": motion.sha256(Path(args.profile)),
        "body_names": names, "parent_indices": parents,
        "forward_axis_mapping": {"model_forward": forward_model.tolist(), "pelvis_local_forward": pelvis_local_forward.tolist(),
                                 "note": "actor forward is model +Y (walksearch request convention); heading is measured from the pelvis rotation applied to pelvis_local_forward"},
        "constraint_frames": [None if f is None else {**f, "frame1": f["frame1"].tolist(), "frame2": f["frame2"].tolist(),
                                                       "hinge_axis_parent_local": f["hinge_axis_parent_local"].tolist()} for f in frames],
        "sample_hz": SAMPLE_HZ, "cases": cases, "clips": clips, "coverage": coverage,
        "measured_vanilla_forward_speed": vanilla,
        "usage_rights": "Vanilla Morrowind animations captured through the user's own installed game data; used as optional style reference within this private development workspace. No redistribution of the source assets.",
        "physical_contacts": None,
        "contact_status": "not provided; animation foot height is not physical contact evidence",
    }
    library["sha256"] = sha256_json({k: v for k, v in library.items() if k != "sha256"})
    (out / "library.json").write_text(json.dumps(library, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    coverage_doc = {
        "schema": "play01_content_coverage_v1", "tracker_task": "PLAY-01", "library_sha256": library["sha256"],
        "required_command_coverage": REQUIRED_COVERAGE, "coverage": coverage,
        "stage_evidence": {"content_ready": [k for k, v in coverage.items() if v["status"] == "content_ready"],
                           "synthesis_required": [k for k, v in coverage.items() if v["status"] == "synthesis_required"],
                           "missing_content": [k for k, v in coverage.items() if v["status"] == "missing_content"],
                           "generator_reviewed": [], "commands_trained": [], "commands_qualified": []},
    }
    (out / "content-coverage.json").write_text(json.dumps(coverage_doc, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    return library


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="native integration run directory with the ten-case replays")
    parser.add_argument("--exe", required=True, help="ragdollcore-walksearch executable for --dump-animation")
    parser.add_argument("--context", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--vanilla-run-dir", help="native run directory holding the PLAY-01 vanilla forward case (defaults to --run-dir)")
    parser.add_argument("--skip-vanilla", action="store_true")
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    library = build_library(args)
    print(json.dumps({"task": args.tracker_task, "clips": sorted(library["clips"]), "library_sha256": library["sha256"],
                      "coverage": {k: v["status"] for k, v in library["coverage"].items()},
                      "vanilla_forward_speed": library["measured_vanilla_forward_speed"]}, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
