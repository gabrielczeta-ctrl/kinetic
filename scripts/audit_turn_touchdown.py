"""Read-only, phase-labelled comparison of the A63 second restart swing.

The CSV mixes pre-solve command telemetry and post-solve body/contact data.
Schema2 separates raw return impulses from support-accepted contact onset.
Neither alone certifies a stable landing. Distances below compare that step's placement command with its resulting foot
position; they are not IK error or a same-state counterfactual comparison.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def classify_contact_events(swing, following):
    """Separate one-frame return impulses from controller-accepted support.

    The accepted onset is the final uninterrupted positive-impulse run before
    the next command switches to double support on this left leg. It is not
    a guarantee of stable landing, adequate force, or successful walking.
    """
    if not swing or any(int(b["step"]) != int(a["step"]) + 1 for a, b in zip(swing, swing[1:])):
        raise ValueError("Contact classification requires contiguous swing frames")
    airborne = False
    first_return = None
    for row in swing:
        if float(row["left_contact_normal_impulse"]) == 0:
            airborne = True
        elif airborne and first_return is None:
            first_return = row
    accepted = None
    if (following is not None and following["reference_stage"] == "double_support"
            and following["reference_stance_hip_body"] == "14"
            and int(following["step"]) == int(swing[-1]["step"]) + 1):
        for row in reversed(swing):
            if float(row["left_contact_normal_impulse"]) <= 0:
                break
            accepted = row
    return first_return, accepted


def inspect(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    # Select by command-side reference stance, not the post-solve gait_phase,
    # which can already have switched sides on the touchdown observation.
    swing = []
    following = None
    for row in rows:
        if int(row["step"]) < 570:
            continue
        selected = (row["reference_stage"] == "swing"
                    and row["reference_stance_hip_body"] == "17")
        if selected:
            swing.append(row)
        elif swing:
            following = row
            break
    if not swing:
        raise ValueError(f"Missing second restart swing in {path}")
    touchdown, accepted = classify_contact_events(swing, following)
    commands_path = path.parent / "startup-commands.csv"
    commands = None
    if commands_path.exists():
        with commands_path.open(newline="") as stream:
            commands = {(int(r["step"]), int(r["body"])): r for r in csv.DictReader(stream)}

    def measure(row):
        if row is None:
            return None
        motor = commands[(int(row["step"]), 14)] if commands is not None else None
        error = [float(row[f"reference_placement_{a}"])
                 - float(row[f"left_foot_{a}"]) for a in "xy"]
        return {
            "scenario_step": int(row["step"]),
            "command_stage_time_s": float(row["reference_time_in_stage"]),
            "post_solve_placement_error_xy": error,
            "post_solve_horizontal_error": math.hypot(*error),
            "post_solve_left_impulse": float(row["left_contact_normal_impulse"]),
            "post_solve_pelvis_up": float(row["pelvis_up"]),
            "pre_solve_hip_angular_error": float(motor["angular_error"] if motor else row["swing_hip_target_error"]),
            "pre_solve_hip_target_clamp": float(motor["target_clamp_delta"] if motor else row["swing_hip_target_clamp"]),
            "post_solve_hip_utilization": float(row["swing_hip_motor_utilization"]),
            "motor_source": "final startup-commands.csv" if motor else "frame summary",
            "command_reach_limited": int(row["reference_swing_reach_limited"]),
        }
    return {"schema": 2,
            "first_contact_semantics": "legacy: first positive impulse after any one zero-impulse frame; not accepted touchdown",
            "accepted_contact_onset_semantics": "final continuous positive-impulse run leading to next-command double support on the left leg; not stability acceptance",
            "accepted_contact_onset": measure(accepted),
            "support_transition_step": int(following["step"]) if accepted is not None else None,
            "final_commands_sha256": hashlib.sha256(commands_path.read_bytes()).hexdigest() if commands is not None else None,
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "swing_start_step": int(swing[0]["step"]),
            "first_contact": measure(touchdown),
            "last_swing_command": measure(swing[-1]),
            "reach_limited_steps": sum(int(r["reference_swing_reach_limited"]) for r in swing)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([inspect(path) for path in args.csv], indent=2))
