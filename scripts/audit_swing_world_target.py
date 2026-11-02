"""Reconstruct command-frame swing targets from an exact replay directory.

Requires numpy/scipy. This is a kinematic diagnostic, not a physics replay.
The hip-goal endpoint holds the measured knee angle and leg offsets fixed;
it is explicitly not a full forward-kinematics solution of all motor targets.
"""
import argparse
import csv
import hashlib
import json
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def audit(directory, steps):
    verification = json.loads((directory / "witness-verification.json").read_text())
    if not verification["exact_recorded_physical_match"]:
        raise ValueError("Requires an exactly reproduced witness")
    warmup = verification["driver_result"]["warmup_frames"]
    data = (directory / "offline-physical.omwrdpl").read_bytes()
    version, _, _, dt, _, _, inputs, states, bodies, count = struct.unpack_from("<IQQfIIIIII", data, 8)
    if data[:8] != b"OMWRDPL\0" or version != 10 or inputs or states:
        raise ValueError("Expected version10 physical-only replay")
    stride = 8 + bodies * 52
    if len(data) != 56 + count * stride:
        raise ValueError("Invalid physical replay size")
    worlds = {(int(r["frame"]), int(r["body"])): r
              for r in csv.DictReader((directory / "world-bodies.csv").open())}
    commands = {(int(r["step"]), int(r["body"])): r
                for r in csv.DictReader((directory / "startup-commands.csv").open())}
    rows = {int(r["step"]): r for r in csv.DictReader((directory / "offline.csv").open())}

    def pose(frame, body):
        row = worlds[frame, body]
        return (np.array([float(row[a]) for a in "xyz"]),
                Rotation.from_quat([float(row["q" + a]) for a in "xyzw"]))

    def model(frame, body):
        offset = 56 + frame * stride
        if struct.unpack_from("<Q", data, offset)[0] != frame:
            raise ValueError("Noncontiguous replay")
        return np.array(struct.unpack_from("<13f", data, offset + 8 + body * 52))

    output = []
    for step in steps:
        row = rows[step]
        if row["reference_stage"] != "swing" or row["reference_stance_hip_body"] != "17":
            raise ValueError("This audit selects airborne left-leg commands only")
        previous = step - warmup - 1
        # scenario.cpp updates the reference transform after the solve. The
        # preceding body's model/world pair therefore gives this command's
        # root frame. Reject a nonrigid/incorrect reconstruction across all bodies.
        m = model(previous, 0)
        world, rotation = pose(previous, 0)
        root_rotation = rotation * Rotation.from_quat(m[3:7]).inv()
        root_position = world - root_rotation.apply(m[:3])
        residual = max(np.linalg.norm(root_position + root_rotation.apply(model(previous, b)[:3])
                                      - pose(previous, b)[0]) for b in range(bodies))
        if residual > 1e-3:
            raise ValueError(f"Root reconstruction failed: {residual}")
        trajectory = root_position + root_rotation.apply(
            [float(row["swing_motor_target_" + a]) for a in "xyz"])
        actual = pose(step - warmup, 16)[0]
        hip, hip_rotation = pose(previous, 14)
        knee, _ = pose(previous, 15)
        foot, _ = pose(previous, 16)
        hip_command = commands[step, 14]
        if hip_command["world_orientation"] != "1":
            raise ValueError("Expected world hip target")
        target = Rotation.from_quat([float(hip_command["child_world_q" + a]) for a in "xyzw"])
        hip_goal_endpoint = hip + (target * hip_rotation.inv()).apply(foot - hip)
        # Maximum endpoint displacement from correcting only the measured
        # knee angle. Does not cover anchor error or other joint constraints.
        knee_delta = abs(float(commands[step, 15]["angular_error"]))
        knee_bound = 2 * np.linalg.norm(foot - knee) * np.sin(knee_delta / 2)
        output.append({"scenario_step": step, "previous_capture_frame": previous,
                       "root_reconstruction_max_error": float(residual),
                       "trajectory_world": trajectory.tolist(), "actual_world": actual.tolist(),
                       "trajectory_minus_actual": (trajectory - actual).tolist(),
                       "hip_goal_endpoint_measured_knee": hip_goal_endpoint.tolist(),
                       "trajectory_to_hip_goal_distance": float(np.linalg.norm(trajectory - hip_goal_endpoint)),
                       "knee_angle_only_endpoint_bound": float(knee_bound)})
    names = ("offline-physical.omwrdpl", "world-bodies.csv", "startup-commands.csv", "offline.csv")
    return {"directory": str(directory.resolve()), "warmup_frames": warmup,
            "sha256": {n: hashlib.sha256((directory / n).read_bytes()).hexdigest() for n in names},
            "samples": output}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("steps", nargs="+", type=int)
    args = parser.parse_args()
    print(json.dumps(audit(args.directory, args.steps), indent=2))
