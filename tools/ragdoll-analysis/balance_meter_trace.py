#!/usr/bin/env python3
"""Audit schema-2 fixed-step BAL measurements; never infer unavailable values.

This is a measurement-integrity diagnostic, not a BAL-01 authority/quality gate.
It reads raw native trace rows, recomputes mass, load and relative tangent slip,
and verifies sample uniqueness and applicability. Source/binary IDs are supplied
by the coordinated launcher; the raw log SHA-256 is computed here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shlex


def number(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"nonfinite {name}")
    return value


def flag(row, name):
    if row[name] not in ("0", "1"):
        raise ValueError(f"invalid boolean {name}")
    return row[name] == "1"


def optional(row, name):
    known = flag(row, name + "_known")
    if not known:
        if name in row:
            raise ValueError(f"unknown {name} has a fabricated numeric value")
        return None
    return number(row, name)


def vector(row, name):
    values = tuple(float(value) for value in row[name].split(","))
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"invalid vector {name}")
    return values


def near(actual, expected, name):
    if not math.isclose(actual, expected, rel_tol=2e-6, abs_tol=1e-4):
        raise ValueError(f"{name}: measured {actual}, reconstructed {expected}")


def tangent_speed(contact):
    if not flag(contact, "foot_velocity_known") or not flag(contact, "surface_velocity_known"):
        return None
    normal = vector(contact, "normal")
    length = math.sqrt(sum(value * value for value in normal))
    if length == 0:
        raise ValueError("zero contact normal")
    normal = tuple(value / length for value in normal)
    relative = tuple(a - b for a, b in zip(vector(contact, "foot_velocity"), vector(contact, "surface_velocity")))
    projected = sum(a * b for a, b in zip(relative, normal))
    return math.sqrt(sum((value - projected * axis) ** 2 for value, axis in zip(relative, normal)))


def validate_sample(row, masses, contacts):
    if number(row, "dt") <= 0:
        raise ValueError("nonpositive fixed duration")
    if flag(row, "authority_enabled"):
        raise ValueError("measurement-only run enabled balance authority")
    if len(masses) != int(row["mass_body_count"]):
        raise ValueError("incomplete mass rows")
    if len(contacts) != int(row["contact_count"]):
        raise ValueError("incomplete contact rows")
    if flag(row, "mass_known"):
        if not masses:
            raise ValueError("known mass without assembly")
        total = sum(number(body, "mass") for body in masses)
        if any(number(body, "mass") <= 0 for body in masses):
            raise ValueError("nonpositive declared mass")
        near(number(row, "total_mass"), total, "assembly mass")
        for output, field in (("com", "com"), ("com_velocity", "velocity")):
            expected = [sum(number(body, "mass") * vector(body, field)[axis] for body in masses) / total for axis in range(3)]
            for actual, target in zip(vector(row, output), expected):
                near(actual, target, output)
        roots = sum(flag(body, "declared_kinematic_root") for body in masses)
        if roots != int(flag(row, "root_kinematic")):
            raise ValueError("declared root mass ownership mismatch")
        excluded = optional(row, "excluded_detached_weapon")
        if excluded is not None and any(int(body["body"]) == excluded for body in masses):
            raise ValueError("detached weapon retained in actor mass assembly")
    identities = set()
    for point in contacts:
        identity = tuple(point[key] for key in (
            "self_body_id", "surface_body_id", "subshape1", "subshape2", "manifold_point", "first_body"))
        if identity in identities:
            raise ValueError("duplicate contact manifold point")
        identities.add(identity)
    measured_loads = []
    supported = []
    loaded_points = []
    minimum_dot = number(row, "support_normal_minimum_dot")
    if not 0 <= minimum_dot <= 1:
        raise ValueError("invalid support normal threshold")
    for side in ("left", "right"):
        complete = flag(row, side + "_contacts_complete")
        if complete and (not flag(row, side + "_active_before")
                         or not flag(row, side + "_geometry_complete") or int(row["update_error"]) != 0):
            raise ValueError(f"{side}: falsely complete contact set")
        body = optional(row, side + "_foot_body")
        qualified = []
        available = complete and body is not None
        for contact in contacts:
            if body is None or int(contact["body"]) != body:
                continue
            normal = vector(contact, "normal")
            length = math.sqrt(sum(value * value for value in normal))
            if length == 0:
                available = False
                continue
            if normal[2] / length < minimum_dot:
                continue
            impulse = optional(contact, "normal_impulse")
            if impulse is None or impulse < 0:
                available = False
            elif impulse > 0:
                qualified.append(contact)
        expected = "unknown" if not available else "supported" if qualified else "unsupported"
        if row[side + "_support"] != expected:
            raise ValueError(f"{side}: support should be {expected}")
        supported.append(expected)
        load = optional(row, side + "_normal_impulse")
        speed = optional(row, side + "_maximum_tangent_speed")
        distance = optional(row, side + "_maximum_tangent_distance_step")
        fraction = optional(row, side + "_load_fraction")
        if not available:
            if any(value is not None for value in (load, speed, distance, fraction)):
                raise ValueError(f"{side}: unavailable contacts produced measurements")
        else:
            expected_load = sum(number(point, "normal_impulse") for point in qualified)
            if load is None:
                raise ValueError(f"{side}: complete load missing")
            near(load, expected_load, side + " normal impulse")
            speeds = [tangent_speed(point) for point in qualified]
            if not speeds or any(value is None for value in speeds):
                if speed is not None or distance is not None:
                    raise ValueError(f"{side}: unsupported/unknown velocity reported slip")
            else:
                if speed is None or distance is None:
                    raise ValueError(f"{side}: known relative slip missing")
                near(speed, max(speeds), side + " maximum tangent speed")
                near(distance, max(speeds) * number(row, "dt"), side + " per-step slip")
            loaded_points.extend(qualified)
        measured_loads.append(load)
    if all(value is not None for value in measured_loads) and sum(measured_loads) > 0:
        for side, load in zip(("left", "right"), measured_loads):
            fraction = optional(row, side + "_load_fraction")
            if fraction is None:
                raise ValueError("known loaded feet have no load fraction")
            near(fraction, load / sum(measured_loads), side + " load fraction")
    elif any(optional(row, side + "_load_fraction") is not None for side in ("left", "right")):
        raise ValueError("load fraction has unknown or zero denominator")
    expected_support = "unknown" if "unknown" in supported or not flag(row, "mass_known") else (
        "supported" if "supported" in supported else "unsupported")
    if row["support"] != expected_support:
        raise ValueError("whole-actor support availability inconsistent")
    support_distance = optional(row, "com_support_distance")
    if expected_support == "supported":
        if support_distance is None or support_distance < 0:
            raise ValueError("supported hull has no nonnegative distance")
    elif support_distance is not None:
        raise ValueError("unsupported/unknown hull has a numeric distance")
    if flag(row, "capture_point_known"):
        if row["capture_applicability"] != "valid-stationary-horizontal-plane" or expected_support != "supported":
            raise ValueError("capture point reported outside its applicability")
        plane = optional(row, "support_plane_height")
        if plane is None or not loaded_points:
            raise ValueError("capture point has no measured support plane")
        for point in loaded_points:
            if optional(point, "surface_static") != 1 or not flag(point, "surface_point_known"):
                raise ValueError("capture point uses an unknown/moving surface")
            normal = vector(point, "normal")
            length = math.sqrt(sum(value * value for value in normal))
            deviation = math.sqrt(sum((value / length - axis) ** 2 for value, axis in zip(normal, (0, 0, 1))))
            if deviation > number(row, "plane_normal_tolerance") + 1e-8:
                raise ValueError("capture point uses sloped support")
            if abs(vector(point, "surface_point")[2] - plane) > number(row, "plane_height_tolerance") + 1e-8:
                raise ValueError("capture point uses noncoplanar support")
        near(vector(row, "capture_point")[2], plane, "capture point plane height")
        gravity = vector(row, "gravity")
        com, velocity = vector(row, "com"), vector(row, "com_velocity")
        if gravity[0] != 0 or gravity[1] != 0 or gravity[2] >= 0 or com[2] <= plane:
            raise ValueError("capture point has invalid height or gravity")
        time = math.sqrt((com[2] - plane) / -gravity[2])
        for axis in range(2):
            near(vector(row, "capture_point")[axis], com[axis] + velocity[axis] * time, "capture point")



def unsigned(row, name):
    value = row[name]
    if not re.fullmatch(r"0|[1-9][0-9]*", value) or int(value) > 2**64 - 1:
        raise ValueError(f"invalid unsigned integer {name}")
    return int(value)


def optional_unsigned(row, name):
    if not flag(row, name + "_known"):
        if name in row:
            raise ValueError(f"unknown {name} has a fabricated integer value")
        return None
    return unsigned(row, name)


def validate_knee(row, main):
    """Validate a joined observation, not whether the physical pose is good."""
    side = row["foot"]
    if side not in ("left", "right"):
        raise ValueError("invalid knee foot")
    prefix = side + "_"
    other = "right_" if side == "left" else "left_"
    if any(key.startswith(other) for key in row):
        raise ValueError("knee row contains the opposite foot's fields")
    for name in ("knee_motor", "knee_clip", "knee_composed", "composed_foot_availability",
                 "animation_availability", "composed_source"):
        if not row[prefix + name]:
            raise ValueError(f"empty knee availability {name}")
    floats = {name: optional(row, prefix + name) for name in (
        "composed_foot_error", "knee_animation", "knee_composed_angle", "knee_final_to_actual",
        "knee_actual", "knee_requested", "knee_final", "knee_minimum", "knee_maximum", "knee_adjustment")}
    integers = {name: optional_unsigned(row, prefix + name) for name in (
        "animation_sample_serial", "animation_sample_next_step", "composed_serial", "composed_step",
        "motor_observation_step", "motor_state")}
    step = unsigned(main, "step")
    for first, second in (("animation_sample_serial", "animation_sample_next_step"),
                          ("composed_serial", "composed_step")):
        if (integers[first] is None) != (integers[second] is None):
            raise ValueError(f"incomplete knee identity {first}/{second}")
    animation_step = integers["animation_sample_next_step"]
    if animation_step is not None and animation_step > step:
        raise ValueError("knee animation sample belongs to a future physics step")
    for name in ("composed_step", "motor_observation_step"):
        if integers[name] is not None and integers[name] != step:
            raise ValueError(f"knee {name} differs from completed physics step")
    if floats["knee_animation"] is not None and animation_step is None:
        raise ValueError("knee animation angle has no sample identity")
    if floats["knee_composed_angle"] is not None and integers["composed_step"] is None:
        raise ValueError("knee composed angle has no pose identity")
    if integers["motor_state"] is not None and integers["motor_state"] not in (0, 1, 2):
        raise ValueError("invalid knee motor state")
    for name in ("knee_requested", "knee_final"):
        if floats[name] is not None and integers["motor_observation_step"] is None:
            raise ValueError("knee motor angle has no observation step")
    for delta, first, second in (("knee_adjustment", "knee_requested", "knee_final"),
                                  ("knee_final_to_actual", "knee_final", "knee_actual")):
        if floats[delta] is not None:
            if floats[first] is None or floats[second] is None:
                raise ValueError(f"knee {delta} has an unavailable operand")
            near(floats[delta], abs(floats[first] - floats[second]), side + " " + delta)
    if floats["knee_minimum"] is not None and floats["knee_maximum"] is not None:
        if floats["knee_minimum"] > floats["knee_maximum"]:
            raise ValueError("reversed knee angle limits")
    vectors = {name: vector(row, prefix + name) if prefix + name in row else None for name in (
        "composed_foot_reference", "actual_foot_origin", "composed_foot_error_vector")}
    error = floats["composed_foot_error"]
    if error is not None:
        if error < 0 or any(value is None for value in vectors.values()):
            raise ValueError("composed foot error lacks current/reference/vector observations")
        actual, reference = vectors["actual_foot_origin"], vectors["composed_foot_reference"]
        expected = tuple(a - b for a, b in zip(actual, reference))
        for observed, target in zip(vectors["composed_foot_error_vector"], expected):
            near(observed, target, side + " signed foot error")
        near(error, math.sqrt(sum(value * value for value in expected)), side + " foot error magnitude")
    elif vectors["composed_foot_error_vector"] is not None:
        raise ValueError("unknown composed foot error has a fabricated vector")


def audit(text):
    samples = {}
    masses = {}
    contacts = {}
    knees = {}
    violations = []
    previous_step = {}
    for line_number, line in enumerate(text.splitlines(), 1):
        match = re.search(r"\b(RAGDOLL_BALANCE(?:_MASS|_CONTACT|_KNEE)?)(?=\s|$)", line)
        if not match:
            continue
        try:
            parts = shlex.split(line[match.start():])
            row = {}
            for token in parts[1:]:
                key, value = token.split("=", 1)
                if key in row:
                    raise ValueError(f"duplicate field {key}")
                row[key] = value
            expected_schema = "1" if parts[0] == "RAGDOLL_BALANCE_KNEE" else "2"
            if row["schema"] != expected_schema:
                raise ValueError("unsupported " + parts[0] + " schema")
            key = (unsigned(row, "actor_serial"), unsigned(row, "step"))
            unsigned(row, "epoch")
            if parts[0] == "RAGDOLL_BALANCE":
                if key in samples:
                    raise ValueError("duplicate completed-step sample")
                if key[0] in previous_step and key[1] <= previous_step[key[0]]:
                    raise ValueError("nonmonotonic actor step")
                previous_step[key[0]] = key[1]
                samples[key] = row
            elif parts[0] == "RAGDOLL_BALANCE_KNEE":
                side = row["foot"]
                if side not in ("left", "right"):
                    raise ValueError("invalid knee foot")
                if side in knees.setdefault(key, {}):
                    raise ValueError("duplicate knee detail row")
                knees[key][side] = row
            else:
                group = masses if parts[0].endswith("_MASS") else contacts
                index = int(row["body"] if group is masses else row["point"])
                if index in group.setdefault(key, {}):
                    raise ValueError("duplicate detail row")
                group[key][index] = row
        except (ValueError, KeyError) as error:
            violations.append(f"line {line_number}: {error}")
    for key, row in samples.items():
        try:
            detail = (list(masses.get(key, {}).values()) + list(contacts.get(key, {}).values())
                      + list(knees.get(key, {}).values()))
            if any(part["epoch"] != row["epoch"] for part in detail):
                raise ValueError("detail rows disagree on lifecycle epoch")
            validate_sample(row, list(masses.get(key, {}).values()), list(contacts.get(key, {}).values()))
            if "knee_schema" in row:
                if row["knee_schema"] != "1":
                    raise ValueError("unsupported knee_schema marker")
                if set(knees.get(key, {})) != {"left", "right"}:
                    raise ValueError("incomplete bilateral knee rows")
                for knee in knees[key].values():
                    validate_knee(knee, row)
            elif key in knees:
                raise ValueError("knee rows have no main knee_schema marker")
        except (ValueError, KeyError, TypeError, ZeroDivisionError) as error:
            violations.append(f"actor {key[0]} step {key[1]}: {error}")
    for key in (masses.keys() | contacts.keys() | knees.keys()) - samples.keys():
        violations.append(f"actor {key[0]} step {key[1]}: orphan detail rows")
    if not samples:
        violations.append("no schema-2 balance samples")
    episode_count = 0
    if any(row.get("episode_schema") == "1" for row in samples.values()):
        from balance_episode_trace import audit_episodes
        episodes = audit_episodes(text)
        episode_count = episodes["sample_count"]
        violations.extend(episodes["violations"])
    from balance_intent_trace import audit_intents
    intents = audit_intents(text)
    violations.extend(intents["violations"])
    return {"sample_count": len(samples), "episode_sample_count": episode_count,
            "intent": {key: value for key, value in intents.items() if key != "violations"},
            "knee_sample_count": sum(len(value) for value in knees.values()), "actor_count": len(previous_step),
            "violation_count": len(violations), "violations": violations,
            "scope": "cached-meter-integrity-only", "success": not violations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--binary-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9a-fA-F]{40}", args.source_commit):
        parser.error("--source-commit must be a full Git commit hash")
    if not re.fullmatch(r"[0-9a-fA-F]{64}", args.binary_sha256):
        parser.error("--binary-sha256 must be a SHA-256 hash")
    raw = args.log.read_bytes()
    result = audit(raw.decode("utf-8", errors="replace"))
    result.update(schema_version=1, source_commit=args.source_commit.lower(),
        binary_sha256=args.binary_sha256.lower(), raw_log=str(args.log.resolve()),
        raw_sha256=hashlib.sha256(raw).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
