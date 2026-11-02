#!/usr/bin/env python3
"""Prepare PLAY-01 reference motion from ragdollcore-walksearch --dump-animation.

This is diagnostic data preparation, not a tracker gate or a trained controller.
Vanilla clips are optional motion/style references, not accepted natural motion
or evidence of dynamic feasibility. Missing transitions need separate references.
CSV transforms are model-space bone frames (not rigid-body COM or world frames).
Quaternions use xyzw, lengths retain OpenMW model units, angles use radians.
The profile gives articulated parents; metadata.body_names gives CSV index order.

Metadata schema 1 requires clip_name, source_hz, source_commit (40 hex or null
when unknown), source_replay, source_replay_sha256, source_csv_sha256,
profile_sha256, and body_names. Paths are relative to the metadata file.
Hashes are checked; the CSV-to-replay relationship and source revision remain
caller declarations. Never manufacture a revision for a historical recording.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
from typing import Any

import numpy as np
import yaml


OUTPUT_HZ = 60.0
TRANSFORM_FIELDS = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def conjugate(q: np.ndarray) -> np.ndarray:
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of xyzw quaternions (component form; numpy.cross is slow for small arrays)."""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack((aw * bx + bw * ax + (ay * bz - az * by),
                     aw * by + bw * ay + (az * bx - ax * bz),
                     aw * bz + bw * az + (ax * by - ay * bx),
                     aw * bw - (ax * bx + ay * by + az * bz)), axis=-1)


def rotate(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate vectors by xyzw quaternions: v + w*t + q x t with t = 2 q x v."""
    qx, qy, qz, qw = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    vx, vy, vz = vector[..., 0], vector[..., 1], vector[..., 2]
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return np.stack((vx + qw * tx + (qy * tz - qz * ty),
                     vy + qw * ty + (qz * tx - qx * tz),
                     vz + qw * tz + (qx * ty - qy * tx)), axis=-1)


def normalize_quaternions(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if q.shape[-1] != 4 or not np.isfinite(q).all() or np.any(np.abs(norm - 1) > 1e-3):
        raise ValueError("rotations must be finite unit xyzw quaternions (roundoff tolerance 0.001)")
    return q / norm


def continuous_quaternions(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternions(q).copy()
    for frame in range(1, len(q)):
        flip = np.sum(q[frame - 1] * q[frame], axis=-1) < 0
        q[frame] = np.where(flip[..., None], -q[frame], q[frame])
    return q


def rotation_vector(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternions(q)
    q = np.where(q[..., 3:] < 0, -q, q)
    length = np.linalg.norm(q[..., :3], axis=-1, keepdims=True)
    angle = 2 * np.arctan2(length, np.clip(q[..., 3:], 0, 1))
    factor = np.divide(angle, length, out=np.full_like(length, 2.0), where=length > 1e-12)
    return q[..., :3] * factor


def angular_velocity(q: np.ndarray, dt: float) -> np.ndarray:
    """Shortest finite differences: spatial angular velocity in q's parent frame.

    Interior samples use the two adjacent frames; endpoints use one-sided
    differences. There is no assumed loop seam or fabricated endpoint sample.
    """
    if len(q) < 2 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("angular velocity needs two frames and positive dt")
    velocity = np.empty(q.shape[:-1] + (3,))
    velocity[0] = rotation_vector(multiply(q[1], conjugate(q[0]))) / dt
    velocity[-1] = rotation_vector(multiply(q[-1], conjugate(q[-2]))) / dt
    if len(q) > 2:
        velocity[1:-1] = rotation_vector(multiply(q[2:], conjugate(q[:-2]))) / (2 * dt)
    return velocity


def load_body_map(profile_path: Path, names: list[str]) -> dict[str, Any]:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or not isinstance(profile.get("bodies"), list):
        raise ValueError("profile must contain bodies")
    bodies = profile["bodies"]
    if not all(isinstance(body, dict) and isinstance(body.get("name"), str) for body in bodies):
        raise ValueError("invalid profile body")
    by_name = {body["name"]: body for body in bodies}
    if (not isinstance(names, list) or not names or not all(isinstance(name, str) for name in names)
            or len(set(names)) != len(names) or len(by_name) != len(bodies) or set(names) != set(by_name)):
        raise ValueError("body_names must uniquely map every profile body to its CSV index")
    indices = {name: index for index, name in enumerate(names)}
    parents = []
    for name in names:
        parent = by_name[name].get("parent")
        if parent is not None and parent not in indices:
            raise ValueError(f"unknown parent for {name}")
        parents.append(-1 if parent is None else indices[parent])
    roots = [i for i, parent in enumerate(parents) if parent == -1]
    if len(roots) != 1:
        raise ValueError("profile must have one articulated root")
    for index in range(len(names)):
        seen: set[int] = set()
        while index != -1:
            if index in seen:
                raise ValueError("cyclic articulated parents")
            seen.add(index)
            index = parents[index]
    landmarks = profile.get("landmarks", {})
    ends = {name: indices[landmarks[name]] for name in ("left_hand", "right_hand", "left_foot", "right_foot")
            if name in landmarks and landmarks[name] in indices}
    if len(ends) != 4:
        raise ValueError("profile must identify both hands and both feet")
    return {"body_names": names, "parent_indices": parents, "root_index": roots[0],
            "end_effector_indices": ends, "profile_id": profile.get("id")}


def read_animation_csv(path: Path, body_count: int) -> dict[str, np.ndarray]:
    rows: dict[tuple[int, int], list[float]] = {}
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"frame", "body", *TRANSFORM_FIELDS, "req_x", "req_y"}
        if (reader.fieldnames is None or len(reader.fieldnames) != len(set(reader.fieldnames))
                or not required.issubset(reader.fieldnames)):
            raise ValueError("missing or duplicate dump-animation CSV columns")
        for line, row in enumerate(reader, 2):
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("ragged row")
                frame, body = int(row["frame"]), int(row["body"])
                values = [float(row[field]) for field in (*TRANSFORM_FIELDS, "req_x", "req_y")]
                if frame < 0 or not 0 <= body < body_count or not np.isfinite(values).all():
                    raise ValueError("invalid index or nonfinite transform/request")
                if (frame, body) in rows:
                    raise ValueError("duplicate frame/body")
                rows[frame, body] = values
            except (ValueError, TypeError) as error:
                raise ValueError(f"CSV line {line}: {error}") from error
    frames = sorted({frame for frame, _ in rows})
    if len(frames) < 2 or frames != list(range(frames[0], frames[-1] + 1)):
        raise ValueError("CSV needs at least two consecutive source frames")
    if len(rows) != len(frames) * body_count:
        raise ValueError("missing body in a source frame")
    data = np.array([[rows[frame, body] for body in range(body_count)] for frame in frames])
    if not np.allclose(data[..., 7:], data[:, :1, 7:], rtol=0, atol=1e-8):
        raise ValueError("movement request differs between bodies in one frame")
    return {"source_frames": np.array(frames), "positions": data[..., :3],
            "rotations": continuous_quaternions(data[..., 3:7]), "requests": data[:, 0, 7:]}


def resample(data: dict[str, np.ndarray], source_hz: float) -> dict[str, np.ndarray]:
    if isinstance(source_hz, bool) or not np.isfinite(source_hz) or source_hz <= 0:
        raise ValueError("source_hz must be positive and finite")
    duration = (len(data["source_frames"]) - 1) / source_hz
    count = int(np.floor(duration * OUTPUT_HZ + 1e-8)) + 1
    if count < 2:
        raise ValueError("source window is too short for two 60 Hz samples")
    times = np.arange(count) / OUTPUT_HZ
    coordinate = np.minimum(times * source_hz, len(data["source_frames"]) - 1)
    lo = np.floor(coordinate).astype(int)
    hi = np.minimum(lo + 1, len(data["source_frames"]) - 1)
    weight = (coordinate - lo)[:, None, None]
    q0, q1 = data["rotations"][lo], data["rotations"][hi]
    dot = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)
    theta = np.arccos(np.clip(np.abs(dot), 0, 1))
    denominator = np.sin(theta)
    a = np.divide(np.sin((1 - weight) * theta), denominator,
                  out=np.broadcast_to(1 - weight, denominator.shape).copy(), where=denominator > 1e-8)
    b = np.divide(np.sin(weight * theta), denominator,
                  out=np.broadcast_to(weight, denominator.shape).copy(), where=denominator > 1e-8)
    return {"times": times, "source_frames": coordinate + data["source_frames"][0],
            "positions": (1 - weight) * data["positions"][lo] + weight * data["positions"][hi],
            "rotations": continuous_quaternions(a * q0 + b * q1),
            "requests": data["requests"][lo]}


def build_reference(data: dict[str, np.ndarray], skeleton: dict[str, Any], source_hz: float) -> dict[str, Any]:
    samples = resample(data, source_hz)
    p, q = samples["positions"], samples["rotations"]
    local_p, local_q = p.copy(), q.copy()
    for child, parent in enumerate(skeleton["parent_indices"]):
        if parent >= 0:
            inverse = conjugate(q[:, parent])
            local_p[:, child] = rotate(inverse, p[:, child] - p[:, parent])
            local_q[:, child] = multiply(inverse, q[:, child])
    local_q = continuous_quaternions(local_q)
    dt = 1 / OUTPUT_HZ
    velocity = np.gradient(p, dt, axis=0, edge_order=1)
    joint_velocity = angular_velocity(local_q, dt)
    root_relative = p - p[:, skeleton["root_index"]:skeleton["root_index"] + 1]
    seam_position = p[0] - p[-1]
    seam_joint_rotation = rotation_vector(multiply(local_q[0], conjugate(local_q[-1])))
    interior_rotation = rotation_vector(multiply(local_q[1:], conjugate(local_q[:-1])))
    diagnostics = {
        "interpretation": "unclosed last-to-first interval of 1/60 s; measurements, not loop acceptance",
        "seam_model_position_step_max": float(np.linalg.norm(seam_position, axis=-1).max()),
        "interior_model_position_step_max": float(np.linalg.norm(np.diff(p, axis=0), axis=-1).max()),
        "seam_root_relative_position_step_max": float(np.linalg.norm(root_relative[0] - root_relative[-1], axis=-1).max()),
        "seam_joint_rotation_step_max_radians": float(np.linalg.norm(seam_joint_rotation, axis=-1).max()),
        "interior_joint_rotation_step_max_radians": float(np.linalg.norm(interior_rotation, axis=-1).max()),
        "seam_joint_angular_speed_max": float(np.linalg.norm(seam_joint_rotation, axis=-1).max() / dt),
        "boundary_joint_angular_velocity_difference_max": float(np.linalg.norm(joint_velocity[0] - joint_velocity[-1], axis=-1).max()),
        "boundary_model_linear_velocity_difference_max": float(np.linalg.norm(velocity[0] - velocity[-1], axis=-1).max()),
    }
    result = {
        "schema": "reference_motion_v1", "tracker_task": "PLAY-01", "scope": "reference_preparation_diagnostic",
        "sample_hz": OUTPUT_HZ, "frame_count": len(p), "duration_seconds": float(samples["times"][-1]),
        "skeleton": skeleton,
        "conventions": {
            "quaternion": "xyzw, active child-to-parent rotation",
            "positions": "model-space bone origins in OpenMW model units, not world COM positions",
            "parent_local_angular_velocity": "radians/second, derivative of relative joint rotation in parent axes; root in model axes",
            "finite_difference": "centered interior, one-sided endpoints, shortest quaternion logarithm; no loop wrap",
            "resampling": "60 Hz position linear/quaternion slerp; request zero-order hold; tail beyond last grid sample omitted",
            "phase": "sample index divided by frame_count; no anatomical stance phase inferred",
        },
        "physical_contacts": None, "contact_status": "not_provided; animation foot height is not physical contact evidence",
        "dynamic_feasibility": "unassessed; no support, torque, balance or tracking claim",
        "root_motion_status": "world trajectory unavailable in dump-animation; requested velocity is intent, not achieved speed",
        "loop_seam": diagnostics,
    }
    arrays = {
        "times_seconds": samples["times"], "source_frame_coordinates": samples["source_frames"],
        "phase_fraction": np.arange(len(p)) / len(p), "requested_horizontal_velocities": samples["requests"],
        "model_positions": p, "model_rotations_xyzw": q, "model_linear_velocities": velocity,
        "model_angular_velocities": angular_velocity(q, dt), "parent_local_positions": local_p,
        "parent_local_rotations_xyzw": local_q, "parent_local_angular_velocities": joint_velocity,
        "root_relative_positions": root_relative,
    }
    result.update({name: values.tolist() for name, values in arrays.items()})
    return result


def measure_reference_errors(reference: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    """Compare synchronized reference-format motion; never label physics success.

    Joint pose/velocity excludes the free root. End-effector errors subtract each
    motion's root translation but retain model axes, so root drift is separate.
    Candidate physical recordings must first be converted to these bone frames.
    """
    if (reference["skeleton"] != candidate["skeleton"] or reference["sample_hz"] != candidate["sample_hz"]
            or reference["times_seconds"] != candidate["times_seconds"]):
        raise ValueError("reference comparison requires identical skeleton, rate and synchronized times")
    count, bodies = reference["frame_count"], len(reference["skeleton"]["body_names"])
    arrays = []
    for motion in (reference, candidate):
        fields = []
        for name, width in (("parent_local_rotations_xyzw", 4), ("parent_local_angular_velocities", 3),
                            ("model_positions", 3), ("model_linear_velocities", 3)):
            values = np.asarray(motion[name], dtype=float)
            if values.shape != (count, bodies, width) or not np.isfinite(values).all():
                raise ValueError(f"invalid comparison array {name}")
            fields.append(normalize_quaternions(values) if width == 4 else values)
        arrays.append(fields)
    (rq, rw, rp, rv), (cq, cw, cp, cv) = arrays
    root = reference["skeleton"]["root_index"]
    joints = [i for i in range(bodies) if i != root]
    ends = list(reference["skeleton"]["end_effector_indices"].values())
    pose_error = rotation_vector(multiply(cq, conjugate(rq)))[:, joints]
    position_error = cp - rp
    end_error = (position_error - position_error[:, root:root + 1])[:, ends]
    def stats(value: np.ndarray) -> dict[str, float]:
        squared = np.sum(value * value, axis=-1)
        return {"rmse": float(np.sqrt(np.mean(squared))), "maximum": float(np.sqrt(np.max(squared)))}
    return {
        "scope": "synchronized_reference_errors_not_physical_acceptance",
        "joint_pose_radians": stats(pose_error), "joint_angular_velocity_radians_per_second": stats((cw - rw)[:, joints]),
        "root_relative_end_effector_model_units": stats(end_error),
        "model_linear_velocity_units_per_second": stats(cv - rv), "root_position_model_units": stats(position_error[:, root]),
    }


def prepare(csv_path: Path, profile_path: Path, metadata_path: Path,
            start_frame: int | None = None, frame_count: int | None = None) -> dict[str, Any]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
    required = {"schema_version", "clip_name", "source_hz", "source_commit", "source_replay",
                "source_replay_sha256", "source_csv_sha256", "profile_sha256", "body_names"}
    if not isinstance(metadata, dict) or not required.issubset(metadata) or metadata["schema_version"] != 1:
        raise ValueError("incomplete or unsupported source metadata")
    revision = metadata["source_commit"]
    if revision is not None and (not isinstance(revision, str) or not re.fullmatch(r"[0-9a-fA-F]{40}", revision)):
        raise ValueError("source_commit must be a full git hash or null for an unknown historical recording")
    if not isinstance(metadata["clip_name"], str) or not metadata["clip_name"].strip():
        raise ValueError("clip_name must identify the source motion")
    replay = Path(metadata["source_replay"])
    if not replay.is_absolute():
        replay = metadata_path.parent / replay
    paths = {"source_replay_sha256": replay, "source_csv_sha256": csv_path, "profile_sha256": profile_path}
    for field, path in paths.items():
        if sha256(path).lower() != str(metadata[field]).lower():
            raise ValueError(f"hash mismatch: {field}")
    skeleton = load_body_map(profile_path, metadata["body_names"])
    data = read_animation_csv(csv_path, len(skeleton["body_names"]))
    start = int(data["source_frames"][0]) if start_frame is None else start_frame
    length = int(data["source_frames"][-1]) - start + 1 if frame_count is None else frame_count
    if length < 2 or start not in data["source_frames"] or start + length - 1 > data["source_frames"][-1]:
        raise ValueError("requested frame window is outside the complete source CSV or has fewer than two frames")
    first = start - int(data["source_frames"][0])
    result = build_reference({name: values[first:first + length] for name, values in data.items()},
                             skeleton, metadata["source_hz"])
    result["provenance"] = {**metadata, "source_replay": str(replay.resolve()),
                            "metadata_sha256": sha256(metadata_path), "preparer_sha256": sha256(Path(__file__)),
                            "selected_source_start_frame": start, "selected_source_frame_count": length,
                            "hash_validation": "replay, CSV and profile bytes matched supplied hashes",
                            "source_commit_status": "unknown" if revision is None else "caller_declared",
                            "csv_replay_relationship": "caller_declared; this preparer does not decode replay bytes"}
    coverage = metadata.get("coverage_tags", [])
    if not isinstance(coverage, list) or not all(isinstance(tag, str) for tag in coverage):
        raise ValueError("coverage_tags must be a list of caller-declared motion classes")
    required_coverage = {"forward", "backward", "left_strafe", "right_strafe", "diagonal", "turns", "starts", "stops"}
    result["coverage_audit"] = {"caller_declared_tags": coverage,
                                "classes_without_declared_coverage": sorted(required_coverage - set(coverage)),
                                "interpretation": "labels are not quality acceptance or inferred behavior"}
    result["reference_error_self_check"] = measure_reference_errors(result, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--start-frame", type=int)
    parser.add_argument("--frame-count", type=int)
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    try:
        if args.output.resolve() in {args.csv.resolve(), args.profile.resolve(), args.metadata.resolve()}:
            raise ValueError("output must not overwrite an input")
        reference = prepare(args.csv, args.profile, args.metadata, args.start_frame, args.frame_count)
        if args.output.resolve() == Path(reference["provenance"]["source_replay"]).resolve():
            raise ValueError("output must not overwrite the source replay")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(reference, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError, TypeError, KeyError) as error:
        parser.error(str(error))
    print(json.dumps({"task": args.tracker_task, "scope": reference["scope"], "frames": reference["frame_count"],
                      "sample_hz": reference["sample_hz"], "output": str(args.output.resolve()),
                      "loop_seam": reference["loop_seam"]}, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
