#!/usr/bin/env python3
"""Export and render a standalone PLAY-01 physical-motion comparison viewer.

The exporter runs only the unperturbed development control. It does not open
qualification holdouts, optimize a policy, or change a checkpoint.
"""

from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import gzip
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from types import SimpleNamespace
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from play01_runner import (  # noqa: E402
    Fixture, Runner, TORSO_BODY_NAME, build_policy, resolved_case_documents,
    sha256, sha256_json, training_run_contract, utc_now, validate_checkpoint_state,
    verify_forward_review,
)


SCHEMA = "play01_motion_viewer_v1"
REFERENCE_KIND = "kinematic_reference"
ANIMATION_ONLY_KINDS = {REFERENCE_KIND, "animation_target"}
NATIVE_MANIFEST_SCHEMA = "play01_native_viewer_manifest_v1"
IDENTITY_FIELDS = (
    "executable_sha256", "context_sha256", "profile_sha256", "library_sha256",
    "generator_source_sha256", "spec_sha256", "generator_version",
)


def validate_checkpoint_source(checkpoint_source: str, fixture: Fixture,
                               allow_ancestor_source: bool) -> dict[str, Any]:
    """Allow a declared ancestor only across documentation/viewer-only changes."""
    current = fixture.identity["git"]["commit"]
    if not re.fullmatch(r"[0-9a-f]{40}", checkpoint_source or ""):
        raise ValueError("checkpoint source commit is missing or invalid")
    if checkpoint_source == current:
        return {"mode": "exact", "checkpoint_source": checkpoint_source, "viewer_source": current}
    if not allow_ancestor_source:
        raise ValueError("checkpoint source differs; --allow-ancestor-source is required for verified portable inference")
    lineage = subprocess.run(
        ["git", "merge-base", "--is-ancestor", checkpoint_source, current],
        cwd=fixture.source_root, capture_output=True, check=False)
    if lineage.returncode != 0:
        raise ValueError("checkpoint source is not an ancestor of viewer source")
    difference = subprocess.run(
        ["git", "diff", "--name-only", checkpoint_source, current],
        cwd=fixture.source_root, capture_output=True, text=True, check=True)
    changed = difference.stdout.splitlines()
    viewer_only = {"scripts/ragdoll-learning/play01_viewer.py", "scripts/ragdoll-learning/test_play01_viewer.py"}
    unsafe = [name for name in changed if not name.startswith("docs/") and name not in viewer_only]
    if unsafe:
        raise ValueError("checkpoint source changes inference/physics inputs: " + ", ".join(unsafe))
    return {"mode": "verified_ancestor", "checkpoint_source": checkpoint_source,
            "viewer_source": current, "changed_paths": changed}


def load_viewer_checkpoint(args: argparse.Namespace, fixture: Fixture) -> tuple[dict[str, Any], dict[str, Any]]:
    """Verify checkpoint, source, manifest and cases before any worker starts."""
    import torch
    checkpoint_path = Path(args.checkpoint).resolve()
    declared_hash = args.checkpoint_sha256
    if not re.fullmatch(r"[0-9a-f]{64}", declared_hash or ""):
        raise ValueError("--checkpoint-sha256 must declare the exact checkpoint SHA-256")
    if not checkpoint_path.is_file() or sha256(checkpoint_path) != declared_hash:
        raise ValueError("checkpoint path/hash mismatch")
    if not args.training_manifest:
        raise ValueError("checkpoint export requires --training-manifest to verify its run/case contract")
    manifest_path = Path(args.training_manifest).resolve()
    run_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    state = torch.load(checkpoint_path, weights_only=False, map_location="cpu")
    validate_checkpoint_state(state, fixture, "transfer")
    if state["normalization"] != {"running_normalizer": None, "fixed_scales": True}:
        raise ValueError("checkpoint normalization contract mismatch")
    checkpoint_identity = state["identity"]
    for key in ("runner_version", "floor", "bound_speed", "command_speed", "control_mode", "control_contract"):
        if checkpoint_identity.get(key) != fixture.identity.get(key):
            raise ValueError("checkpoint inference identity mismatch: " + key)
    source = checkpoint_identity.get("git", {}).get("commit")
    source_verification = validate_checkpoint_source(source, fixture, args.allow_ancestor_source)
    bindings = run_manifest["specification_and_source_hashes"]
    expected = {"source_commit": source, "runner_version": checkpoint_identity["runner_version"],
                "specification_id": fixture.identity["specification_id"],
                "spec_sha256": fixture.identity["spec_sha256"],
                "engine_binary_sha256": fixture.identity["executable_sha256"]}
    mismatched = [key for key, value in expected.items() if bindings.get(key) != value]
    body = run_manifest["body_profile_and_motion_generator_hashes"]
    mismatched += [key for key in ("context_sha256", "profile_sha256", "library_sha256",
                                   "generator_version", "generator_source_sha256")
                   if body.get(key) != fixture.identity[key]]
    if mismatched:
        raise ValueError("checkpoint manifest identity mismatch: " + ", ".join(mismatched))
    if run_manifest["stage_and_command_scope"].get("control_contract") != fixture.identity.get("control_contract"):
        raise ValueError("checkpoint manifest control contract mismatch")
    coverage_path = fixture.library_path.parent / "content-coverage.json"
    if not coverage_path.is_file() or sha256(coverage_path) != run_manifest["content_coverage_manifest_sha256"]:
        raise ValueError("checkpoint manifest content coverage identity mismatch")
    torso = fixture.library["body_names"].index(TORSO_BODY_NAME)
    selection, cases_doc = resolved_case_documents(fixture.spec["case_generation"], torso)
    if (run_manifest["fresh_holdout_manifest_sha256"] != cases_doc["holdout_manifest_sha256"]
            or run_manifest["curriculum_and_case_manifest_hashes"]["selection_manifest_sha256"] != sha256_json(selection)):
        raise ValueError("checkpoint manifest case identity mismatch")
    validate_checkpoint_state(state, fixture, "transfer", training_run_contract(run_manifest, cases_doc))
    review = verify_forward_review(run_manifest["forward_reference_review"], manifest_path.parent, fixture)
    if not state["policy"] or any(not torch.isfinite(value).all() for value in state["policy"].values()):
        raise ValueError("checkpoint policy contains empty or non-finite tensors")
    return state, {
        "path": str(checkpoint_path), "sha256": declared_hash, "source_commit": source,
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "source_verification": source_verification, "forward_reference_review": review,
        "counters": state["counters"], "inference": "deterministic_actor_mean",
    }


def projection_bounds(data: dict[str, Any], view: str, camera: str) -> list[float]:
    """One fixed scale across variants and time; following preserves world height."""
    axes = {"side": (1, 2), "front": (0, 2), "top": (0, 1)}[view]
    low = [math.inf, math.inf]
    high = [-math.inf, -math.inf]
    for variant in data["variants"]:
        for frame in variant["frames"]:
            root = frame["positions"][data["pelvis"]]
            for point in frame["positions"]:
                for dimension, axis in enumerate(axes):
                    value = float(point[axis])
                    if camera == "follow" and axis != 2:
                        value -= float(root[axis])
                    low[dimension] = min(low[dimension], value)
                    high[dimension] = max(high[dimension], value)
    if view != "top":
        low[1] = min(0.0, low[1])
    pad = max(5.0, *(0.08 * (high[d] - low[d]) for d in range(2)))
    return [low[0] - pad, high[0] + pad, low[1] - pad, high[1] + pad]


def validated_viewer_data(data: dict[str, Any]) -> dict[str, Any]:
    """Reject broken trajectories and normalize serialized numeric support flags."""
    if data.get("schema") != SCHEMA or not data.get("variants"):
        raise ValueError("viewer data has the wrong schema or no variants")
    data = copy.deepcopy(data)
    body_count = len(data.get("body_names", []))
    parents = data.get("parents", [])
    if not body_count or len(parents) != body_count or any(
            not isinstance(parent, int) or parent < -1 or parent >= body_count for parent in parents):
        raise ValueError("viewer body/parent layout is invalid")
    for key in ("pelvis", "left_foot", "right_foot"):
        if not isinstance(data.get(key), int) or not 0 <= data[key] < body_count:
            raise ValueError("viewer landmark is invalid: " + key)
    if not math.isfinite(float(data.get("dt", 0))) or float(data["dt"]) <= 0:
        raise ValueError("viewer timestep must be positive and finite")
    seen = set()
    for variant in data["variants"]:
        if not variant.get("frames") or variant.get("id") in seen:
            raise ValueError("viewer variants require frames and unique IDs")
        seen.add(variant.get("id"))
        if not re.fullmatch(r"#[0-9a-fA-F]{6}", variant.get("color", "")):
            raise ValueError("viewer variant color is invalid")
        reference = variant.get("kind") in ANIMATION_ONLY_KINDS
        if reference and variant.get("metrics"):
            raise ValueError("kinematic reference cannot have physical outcome metrics")
        for frame in variant["frames"]:
            shapes = [("positions", (body_count, 3))]
            if not reference:
                shapes.append(("com_velocity", (3,)))
            elif any(frame.get(key) is not None for key in (
                    "com_velocity", "left_supported", "right_supported", "nonfoot_support",
                    "requested_speed", "achieved_speed", "pelvis_height", "left_load",
                    "right_load", "slip_speed")):
                raise ValueError("animation-only variant cannot claim physical telemetry")
            for key, shape in shapes:
                array = np.asarray(frame.get(key), dtype=float)
                if array.shape != shape or not np.isfinite(array).all():
                    raise ValueError("viewer frame has invalid/non-finite " + key)
                frame[key] = array.tolist()
            for key in ("left_supported", "right_supported", "nonfoot_support"):
                if reference:
                    frame[key] = None
                    continue
                value = float(frame.get(key, 0))
                if value not in (0.0, 1.0):
                    raise ValueError("viewer support flag must be numeric 0 or 1")
                frame[key] = bool(value)
            for key in ("requested_speed", "achieved_speed", "pelvis_height", "left_load",
                        "right_load", "slip_speed", "reference_phase", "phase_progress"):
                value = frame.get(key)
                if value is not None and not math.isfinite(float(value)):
                    raise ValueError("viewer frame has invalid/non-finite " + key)
                frame[key] = None if value is None else float(value)
    data["projection_bounds"] = {
        camera: {view: projection_bounds(data, view, camera) for view in ("side", "front", "top")}
        for camera in ("follow", "world")}
    return data


def _native_rows(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    if not rows or rows[0].get("tracker_task") != "PLAY-01":
        raise ValueError("native trace is missing its PLAY-01 header")
    frames = [row for row in rows if row.get("kind") == "frame" and row.get("state")]
    if not frames:
        raise ValueError("native trace contains no recorded state frames")
    stop = next((row for row in reversed(rows) if row.get("kind") == "capture_stop"), {})
    return rows[0], frames, stop


def _ordered_body_names(state: dict[str, Any]) -> list[str]:
    names = state.get("body_names")
    if isinstance(names, list):
        return names
    if isinstance(names, dict):
        return [names[key] for key in sorted(names, key=lambda value: int(value))]
    raise ValueError("native trace body_names is invalid")


def _contact_telemetry(state: dict[str, Any], body_names: list[str]) -> tuple[float | None, float, float, bool]:
    """Return max contact-point tangential slip, left/right impulse loads and non-foot contact."""
    contacts = state.get("contacts") or {}
    if isinstance(contacts, list):
        contacts = {str(index): value for index, value in enumerate(contacts)}
    bodies = state.get("bodies") or {}
    by_name = bodies if isinstance(bodies, dict) else {body["name"]: body for body in bodies}
    foot_names = {"left_foot", "right_foot"}
    impulses = {name: 0.0 for name in foot_names}
    slips: list[float] = []
    nonfoot = False
    for contact in contacts.values():
        index = int(contact.get("bodyIndex", 0)) - 1
        if not 0 <= index < len(body_names):
            continue
        name = body_names[index]
        impulse = max(0.0, float(contact.get("normalImpulse", 0.0)))
        if name not in foot_names:
            nonfoot = nonfoot or impulse > 0
            continue
        impulses[name] += impulse
        if impulse <= 0:
            continue
        body = by_name.get(name)
        if body is None:
            continue
        linear = np.asarray(body.get("linearVelocity"), dtype=float)
        angular = np.asarray(body.get("angularVelocity"), dtype=float)
        center = np.asarray(body.get("centerOfMass"), dtype=float)
        point = np.asarray(contact.get("position"), dtype=float)
        normal = np.asarray(contact.get("normal"), dtype=float)
        if any(value.shape != (3,) or not np.isfinite(value).all()
               for value in (linear, angular, center, point, normal)):
            continue
        length = np.linalg.norm(normal)
        if length <= 1e-9:
            continue
        normal /= length
        velocity = linear + np.cross(angular, point - center)
        tangential = velocity - normal * np.dot(velocity, normal)
        slips.append(float(np.linalg.norm(tangential)))
    total = impulses["left_foot"] + impulses["right_foot"]
    left = impulses["left_foot"] / total if total > 0 else 0.0
    right = impulses["right_foot"] / total if total > 0 else 0.0
    return (max(slips) if slips else None), left, right, nonfoot


def _native_variant(spec: dict[str, Any], library: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    trace_path = Path(spec["trace"]["path"]).resolve()
    batch_path = Path(spec["batch"]["path"]).resolve()
    for label, path, binding in (("trace", trace_path, spec["trace"]),
                                 ("batch", batch_path, spec["batch"])):
        if not path.is_file() or sha256(path).lower() != str(binding.get("sha256", "")).lower():
            raise ValueError(f"native {label} path/hash mismatch for {spec.get('id')}")
    batch = json.loads(batch_path.read_text(encoding="utf-8"))
    if batch.get("tracker_task") != "PLAY-01":
        raise ValueError("native batch is not bound to PLAY-01")
    record = next((item for item in batch.get("preflight_records", [])
                   if item.get("id") == spec.get("record_id")), None)
    if record is None:
        raise ValueError("native batch lacks requested record " + str(spec.get("record_id")))
    roots = [Path(value).resolve() for value in record.get("artifact_directories", [])]
    if not any(trace_path.is_relative_to(root) for root in roots):
        raise ValueError("native trace is outside the selected batch record")
    header, rows, stop = _native_rows(trace_path)
    first_state = rows[0]["state"]
    full_body_names = _ordered_body_names(first_state)
    if full_body_names != library.get("body_names"):
        raise ValueError("native trace body layout does not match viewer library")
    first_bodies = first_state.get("bodies") or {}
    present_names = set(first_bodies if isinstance(first_bodies, dict)
                        else (body["name"] for body in first_bodies))
    body_names = [name for name in full_body_names if name in present_names]
    if not {"pelvis", "left_foot", "right_foot"}.issubset(body_names):
        raise ValueError("native trace omits required viewer landmarks")
    position_field = spec.get("position_field", "position")
    kind = spec.get("kind", "simulated_body")
    if kind not in {"simulated_body", "animation_target"}:
        raise ValueError("native viewer variant kind is invalid")
    if (kind == "simulated_body" and position_field != "position") or (
            kind == "animation_target" and position_field != "targetPosition"):
        raise ValueError("native viewer position field does not match variant authority")
    output_frames = []
    for row in rows:
        state, controller = row["state"], row["state"].get("controller") or {}
        bodies = state.get("bodies") or {}
        by_name = bodies if isinstance(bodies, dict) else {body["name"]: body for body in bodies}
        try:
            positions = [by_name[name][position_field] for name in body_names]
        except KeyError as error:
            raise ValueError(f"native trace lacks {position_field} for {error.args[0]}") from error
        segment = str(row.get("segment", "unknown"))
        elapsed = float(row.get("elapsed", 0.0))
        segments = header.get("protocol", {}).get("segments", {})
        ordered_segments = [segments[key] for key in sorted(segments, key=lambda value: int(value))]
        segment_index = next((index for index, candidate in enumerate(ordered_segments)
                              if candidate.get("segment") == segment), len(ordered_segments))
        start = sum(float(item["duration"]) for item in ordered_segments[:segment_index])
        duration = (float(ordered_segments[segment_index]["duration"])
                    if segment_index < len(ordered_segments) else 1.0)
        progress = min(1.0, max(0.0, (elapsed - start) / duration))
        base = {"positions": positions, "protocol_phase": segment, "phase_progress": progress,
                "gait_phase": controller.get("gaitPhase"),
                "reference_phase": controller.get("coherentReferencePhase")}
        if kind == "animation_target":
            base.update({key: None for key in (
                "com_velocity", "left_supported", "right_supported", "nonfoot_support",
                "requested_speed", "achieved_speed", "pelvis_height", "left_load",
                "right_load", "slip_speed")})
        else:
            support = state.get("support") or {}
            slip, left_load, right_load, nonfoot = _contact_telemetry(state, full_body_names)
            requested = controller.get("targetHorizontalVelocity") or controller.get("rawTargetHorizontalVelocity")
            achieved = controller.get("centerOfMassVelocity")
            base.update({
                "com_velocity": achieved,
                "left_supported": bool(support.get("leftFoot")),
                "right_supported": bool(support.get("rightFoot")),
                "nonfoot_support": nonfoot,
                "requested_speed": float(requested[1]) if requested else None,
                "achieved_speed": float(achieved[1]) if achieved else None,
                "pelvis_height": controller.get("pelvisHeight"),
                "left_load": left_load, "right_load": right_load, "slip_speed": slip,
            })
        output_frames.append(base)
    physical = kind == "simulated_body"
    steady = [frame for frame in output_frames if frame["protocol_phase"] == "steady"]
    achieved = [frame["achieved_speed"] for frame in steady if frame.get("achieved_speed") is not None]
    requested = [frame["requested_speed"] for frame in steady if frame.get("requested_speed") is not None]
    total_duration = sum(float(item["duration"]) for item in header["protocol"]["segments"].values())
    complete = float(rows[-1].get("elapsed", 0.0)) >= total_duration - 1 / 120
    metrics = {} if not physical else {
        "mean_achieved_along_command_speed_walk": float(np.mean(achieved)) if achieved else None,
        "commanded_velocity_rmse_walk": float(np.sqrt(np.mean(np.square(
            np.subtract(achieved, requested))))) if achieved and len(achieved) == len(requested) else None,
        "maximum_contact_point_slip": max((frame["slip_speed"] for frame in output_frames
                                            if frame.get("slip_speed") is not None), default=None),
        "trace_complete": complete,
    }
    variant = {"id": spec["id"], "label": spec["label"], "color": spec["color"],
               "kind": kind, "metrics": metrics, "frames": output_frames,
               "provenance": {"trace_path": str(trace_path), "trace_sha256": sha256(trace_path),
                              "batch_path": str(batch_path), "batch_sha256": sha256(batch_path),
                              "record_id": record["id"], "record_passed": record.get("passed"),
                              "source_commit": batch.get("source_commit"),
                              "executable_sha256": batch.get("engine_sha256"),
                              "controller_mode": batch.get("build_identity", {}).get("controller_mode"),
                              "profile_sha256": batch.get("build_identity", {}).get("profile_sha256"),
                              "fixture": batch.get("build_identity", {}).get("fixture"),
                              "trace_termination": stop.get("termination"),
                              "first_failure": stop.get("first_failure"),
                              "position_field": position_field}}
    if kind == "animation_target":
        variant["reference"] = {"source": "recorded targetPosition",
                                "physical_contacts": None,
                                "timing": "Same native trace frames and protocol phase as its simulated body."}
    return variant, {"header": header, "batch": batch, "rows": rows,
                     "body_names": body_names}


def load_native_comparison(manifest_path: Path) -> dict[str, Any]:
    """Load hash-bound native diagnostics without running physics or reconstructing poses."""
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != NATIVE_MANIFEST_SCHEMA or manifest.get("tracker_task") != "PLAY-01":
        raise ValueError("native viewer manifest schema/task mismatch")
    library_binding = manifest.get("library") or {}
    library_path = Path(library_binding.get("path", "")).resolve()
    if not library_path.is_file() or sha256(library_path).lower() != str(library_binding.get("sha256", "")).lower():
        raise ValueError("native viewer library path/hash mismatch")
    library = json.loads(library_path.read_text(encoding="utf-8"))
    if library.get("schema") != "play01_motion_library_v1":
        raise ValueError("native viewer library schema mismatch")
    variants, bindings = [], []
    for spec in manifest.get("variants", []):
        variant, binding = _native_variant(spec, library)
        variants.append(variant)
        bindings.append(binding)
    if not variants or len({variant["id"] for variant in variants}) != len(variants):
        raise ValueError("native viewer manifest requires unique variants")
    tests = {(binding["header"].get("test"), binding["header"].get("common_seed")) for binding in bindings}
    if len(tests) != 1:
        raise ValueError("native viewer traces are not the same test and common seed")
    layouts = {tuple(binding["body_names"]) for binding in bindings}
    if len(layouts) != 1:
        raise ValueError("native viewer traces do not expose the same diagnostic bodies")
    body_names = list(next(iter(layouts)))
    library_names = library["body_names"]
    parents = []
    for name in body_names:
        parent = library["parent_indices"][library_names.index(name)]
        while parent >= 0 and library_names[parent] not in body_names:
            parent = library["parent_indices"][parent]
        parents.append(-1 if parent < 0 else body_names.index(library_names[parent]))
    primary = next((binding for variant, binding in zip(variants, bindings)
                    if variant["kind"] == "simulated_body" and variant["id"].startswith("v22")), bindings[-1])
    header, batch = primary["header"], primary["batch"]
    elapsed = [float(row["elapsed"]) for binding in bindings for row in binding["rows"]]
    diffs = [b - a for a, b in zip(elapsed, elapsed[1:]) if b > a and b - a < 0.1]
    dt = float(np.median(diffs)) if diffs else 1 / 60
    segment_counts = {name: max(sum(row.get("segment") == name for row in binding["rows"])
                                    for binding in bindings)
                      for name in ("idle", "start", "steady", "stop")}
    captured = datetime.fromtimestamp(max(Path(spec["trace"]["path"]).stat().st_mtime
                                          for spec in manifest["variants"]), timezone.utc).isoformat()
    data = {
        "schema": SCHEMA, "tracker_task": "PLAY-01", "captured_utc": captured,
        "identity": {"git": {"commit": batch["source_commit"]},
                     "executable_sha256": batch["engine_sha256"]},
        "native_manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "qualification_holdout_opened": False, "optimization_performed": False,
        "command_speed": float(header["protocol"]["targetSpeed"]), "dt": dt,
        "episode_steps": max(len(variant["frames"]) for variant in variants),
        "protocol": {"warmup_steps": segment_counts["idle"],
                     "start_blend_steps": segment_counts["start"],
                     "walk_steps": segment_counts["steady"],
                     "stop_blend_steps": segment_counts["stop"]},
        "sync": {"mode": "native_protocol_phase", "test": header["test"],
                 "common_seed": header["common_seed"],
                 "note": "Same 60 Hz native protocol clock; candidates end at their recorded failure."},
        "body_names": body_names, "parents": parents,
        "pelvis": body_names.index("pelvis"),
        "left_foot": body_names.index("left_foot"),
        "right_foot": body_names.index("right_foot"),
        "variants": variants,
    }
    return validated_viewer_data(data)


def add_morrowind_reference(data: dict[str, Any], library_path: Path) -> dict[str, Any]:
    """Append the captured forward loop at native cadence, with no physics.

    The library preserves native animation rotations on fixed captured bone
    offsets. This is not the speed-adapted idle/start/walk/stop generator and
    has neither physical contacts nor world travel. Never fabricate either.
    """
    from motion_library import forward_kinematics
    from reference_generator import sample_loop

    library_path = library_path.resolve()
    library = json.loads(library_path.read_text(encoding="utf-8"))
    if (library.get("schema") != "play01_motion_library_v1"
            or library.get("sha256") != sha256_json({k: v for k, v in library.items() if k != "sha256"})):
        raise ValueError("Morrowind library schema/content hash mismatch")
    expected = {"library_sha256": library["sha256"],
                "context_sha256": library["creation_context"]["sha256"],
                "profile_sha256": library["profile_sha256"]}
    if any(data.get("identity", {}).get(k) != v for k, v in expected.items()):
        raise ValueError("Morrowind reference does not match recorded library/context/profile")
    if (data["body_names"] != library["body_names"]
            or data["parents"] != library["parent_indices"]):
        raise ValueError("Morrowind reference body layout mismatch")
    if any(v["id"] == "morrowind-reference" for v in data["variants"]):
        raise ValueError("Morrowind reference is already present")
    clip = library["clips"]["forward"]
    if clip.get("sha256") != sha256_json({k: v for k, v in clip.items() if k != "sha256"}):
        raise ValueError("Morrowind forward clip hash mismatch")
    case = library["cases"][clip["case"]]
    if (case["family"] != "forward" or not library.get("usage_rights")
            or any(not re.fullmatch(r"[0-9a-f]{64}", clip.get(k, ""))
                   or clip[k] != case["decode"].get(k) for k in ("csv_sha256", "replay_sha256"))):
        raise ValueError("Morrowind forward clip capture provenance mismatch")
    hz, duration = float(library["sample_hz"]), float(clip["duration_seconds"])
    count, bodies = int(clip["frame_count"]), len(data["body_names"])
    rotations = np.asarray(clip["parent_local_rotations_xyzw"], dtype=float)
    roots = np.asarray(clip["root_positions"], dtype=float)
    if (not math.isfinite(hz) or hz <= 0 or count <= 0
            or not math.isclose(duration, count / hz, rel_tol=1e-9)
            or rotations.shape != (count, bodies, 4) or roots.shape != (count, 3)
            or not np.isfinite(rotations).all() or not np.isfinite(roots).all()
            or not np.allclose(np.linalg.norm(rotations, axis=-1), 1.0, atol=1e-6)):
        raise ValueError("Morrowind forward clip geometry/timing is invalid")
    skeleton = {"parent_indices": data["parents"], "local_offsets": library["creation_local_offsets"]}
    sampler = {"_rotations": rotations, "_roots": roots}
    frames = []
    for index in range(max(len(v["frames"]) for v in data["variants"])):
        phase = (index * data["dt"] / duration) % 1.0
        local, root = sample_loop(sampler, phase)
        positions, _ = forward_kinematics(local[None], root[None], skeleton)
        frames.append({"positions": positions[0].tolist(), "phase": phase,
                       "com_velocity": None, "left_supported": None,
                       "right_supported": None, "nonfoot_support": None})
    result = copy.deepcopy(data)
    result["variants"].append({
        "id": "morrowind-reference", "kind": REFERENCE_KIND,
        "label": "Morrowind walk", "color": "#70cfff", "metrics": {}, "frames": frames,
        "reference": {**expected, "library_file_sha256": sha256(library_path),
                      "clip_sha256": clip["sha256"], "clip": "forward", "case": clip["case"],
                      "source_frames": clip["source_frames"], "sample_hz": hz,
                      "duration_seconds": duration, "playback_rate": 1.0,
                      "replay_sha256": clip["replay_sha256"], "csv_sha256": clip["csv_sha256"],
                      "usage_rights": library["usage_rights"],
                      "geometry": "Native animation rotations retargeted onto fixed captured bone offsets.",
                      "timing": "Continuous forward loop at native cadence; not synchronized to physics gait or start/stop.",
                      "root_motion": "Model-space only; no added world travel.",
                      "physical_contacts": None}})
    validated_viewer_data(result)
    return result


def validate_comparison_report(report: dict[str, Any], fixture: Fixture) -> None:
    """Require the viewer baseline to be the exact validated physical setup."""
    if report.get("schema") != "play01_teacher_ablation_result_v1":
        raise ValueError("comparison report is not a PLAY-01 teacher ablation result")
    if report.get("qualification_holdout_opened") is not False:
        raise ValueError("comparison report does not prove holdouts stayed closed")
    missing = [field for field in IDENTITY_FIELDS
               if report.get("identity", {}).get(field) != fixture.identity.get(field)]
    if missing:
        raise ValueError("comparison report identity mismatch: " + ", ".join(missing))
    for key in ("production_full", "production_pose_only_oracle", "direct_zero_residual"):
        if key not in report:
            raise ValueError(f"comparison report lacks {key}")


def _variant(identifier: str, label: str, color: str, result: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": identifier,
        "label": label,
        "color": color,
        "metrics": result["metrics"],
        "frames": result["visual_frames"],
    }


def viewer_document(data: dict[str, Any]) -> str:
    """Build one offline HTML document with its trajectory data embedded."""
    data = validated_viewer_data(data)
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    template = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PLAY-01 Motion Viewer</title>
<style>
:root{color-scheme:dark;--bg:#080b11;--panel:#111722;--line:#263142;--text:#f4f7fb;--muted:#9ba9bc;--accent:#63e6be}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 15% -10%,#1b2740 0,transparent 35%),var(--bg);color:var(--text);font:14px/1.4 Inter,Segoe UI,sans-serif}
main{max-width:1500px;margin:auto;padding:24px}.eyebrow{letter-spacing:.14em;text-transform:uppercase;color:var(--accent);font-size:11px;font-weight:700}h1{font-size:clamp(26px,4vw,48px);line-height:1;margin:7px 0 9px}p{color:var(--muted);margin:0}.toolbar,.card,.stage{background:rgba(17,23,34,.92);border:1px solid var(--line);border-radius:14px}.toolbar{margin:18px 0 12px;padding:12px;display:flex;gap:12px;align-items:center;flex-wrap:wrap}.toolbar button,.toolbar select{border:1px solid #35445b;background:#172131;color:var(--text);border-radius:8px;padding:8px 12px}.toolbar button{cursor:pointer;font-weight:700}.toolbar input[type=range]{flex:1;min-width:240px;accent-color:var(--accent)}.stage{overflow:hidden;position:relative}.stage canvas{width:100%;height:480px;display:block}.legend{display:flex;gap:14px;flex-wrap:wrap;margin-left:auto}.legend label{display:flex;gap:6px;align-items:center;color:var(--muted)}.dot{width:10px;height:10px;border-radius:50%;display:inline-block}.cards{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin:12px 0}.card{padding:14px 16px}.card .name{font-weight:800;font-size:15px}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}.stat strong{font-size:18px;display:block}.stat span,.foot{color:var(--muted);font-size:11px}.chart{height:210px}.chart canvas{height:210px}.phasebar{height:7px;display:flex;margin-top:10px;border-radius:20px;overflow:hidden}.phasebar i:nth-child(1){background:#435066}.phasebar i:nth-child(2){background:#7c5cff}.phasebar i:nth-child(3){background:#26a269}.phasebar i:nth-child(4){background:#d97828}.foot{margin-top:12px;display:flex;justify-content:space-between;gap:20px;flex-wrap:wrap}.pill{border-radius:99px;background:#1a2535;padding:4px 8px;color:#cbd5e1}.bad{color:#ff7b72}.good{color:#63e6be}@media(max-width:850px){.cards{grid-template-columns:1fr}.stage canvas{height:620px}.stats{grid-template-columns:repeat(3,1fr)}}
</style>
<style>
.cards{grid-template-columns:repeat(auto-fit,minmax(250px,1fr))}.reference-note{margin-top:8px;max-width:1100px}.reference-note:empty{display:none}.live{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:12px;padding-top:10px;border-top:1px solid var(--line)}.live b{display:block;font-size:14px}.live small,.badge{color:var(--muted);font-size:10px}.badge{display:inline-block;border:1px solid #35445b;border-radius:99px;padding:2px 7px;margin-top:6px}.animation-badge{color:#70cfff;border-color:#315c73}@media(max-width:450px){main{padding:12px}.toolbar{gap:8px}.toolbar select{max-width:100%}.legend{margin-left:0}.live{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body><main>
<div class="eyebrow">Physical trajectory diagnostic · development control only</div>
<h1>PLAY-01 Motion Viewer</h1>
<p id="subtitle"></p>
<p id="checkpoint"></p>
<p id="reference-note" class="reference-note"></p>
<div class="toolbar">
  <button id="play">▶ Play</button>
  <select id="view"><option value="side">Side · forward / height</option><option value="front">Front · lateral / height</option><option value="top">Top · lateral / forward</option></select>
  <select id="camera"><option value="follow">Follow character</option><option value="world">World / displacement</option></select>
  <select id="rate"><option value="0.25">¼×</option><option value="0.5">½×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
  <span id="clock" class="pill"></span>
  <input id="timeline" type="range" min="0" value="0" step="1">
  <div class="legend" id="legend"></div>
</div>
<div class="stage"><canvas id="motion"></canvas></div>
<div class="phasebar" id="phasebar"></div>
<div class="cards" id="cards"></div>
<div class="stage chart"><canvas id="chart"></canvas></div>
<div class="foot"><span id="identity"></span><span>Physics only: green feet = support, red pelvis = non-foot support. Animation has no contact data. Scrub or use ←/→.</span></div>
</main>
<script>
const DATA=__DATA__;
const $=id=>document.getElementById(id), motion=$('motion'), chart=$('chart'), ctx=motion.getContext('2d'), cctx=chart.getContext('2d');
const maxFrame=Math.max(...DATA.variants.map(v=>v.frames.length))-1; $('timeline').max=maxFrame;
let frame=0,playing=false,last=0,acc=0;
const isReference=v=>v.kind==='kinematic_reference'||v.kind==='animation_target';
const hasReference=DATA.variants.some(isReference);
const selected=new Set(DATA.variants.filter(v=>!hasReference||v.id!=='zero-correction').map(v=>v.id));
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function metric(v,k,d=1){const x=v.metrics[k];return x==null?'—':Number(x).toFixed(d)}
$('subtitle').textContent=`Captured ${DATA.captured_utc} · ${DATA.command_speed.toFixed(1)} units/s · ${DATA.episode_steps} frames at ${Math.round(1/DATA.dt)} Hz`;
$('identity').textContent=`source ${DATA.identity.git.commit.slice(0,10)} · exe ${DATA.identity.executable_sha256.slice(0,12)} · data ${DATA.data_sha256||'embedded'}`;
$('checkpoint').textContent=DATA.checkpoint?`Learned checkpoint ${DATA.checkpoint.sha256.slice(0,16)} · source ${DATA.checkpoint.source_commit.slice(0,10)} · ${Number(DATA.checkpoint.counters.training_usable_optimizer_transitions||0).toLocaleString()} usable training samples`:'Recorded controller diagnostics; no learned checkpoint in this recording.';
$('reference-note').textContent=DATA.sync?`${DATA.sync.test} · seed ${DATA.sync.common_seed}. ${DATA.sync.note} Target lanes are the exact recorded motor targets, not simulated bodies.`:(hasReference?'Morrowind walk: captured vanilla animation rotations on this skeleton, looping at native cadence (1×). Animation only: no physics, contacts or world travel. It is not synchronized to the simulated start/stop or adapted training-reference clock.':'');
$('legend').innerHTML=DATA.variants.map(v=>`<label><input type="checkbox" data-id="${esc(v.id)}" ${selected.has(v.id)?'checked':''}><i class="dot" style="background:${v.color}"></i>${esc(v.label)}</label>`).join('');
$('cards').innerHTML=DATA.variants.map(v=>{const native=!!v.provenance;const complete=v.metrics.trace_complete;const referenceDetail=v.kind==='kinematic_reference'&&v.reference.duration_seconds?`${v.reference.duration_seconds.toFixed(2)} s native loop`:'recorded targetPosition';return `<div class="card" data-card="${esc(v.id)}"><div class="name" style="color:${v.color}">${esc(v.label)}</div><span class="badge ${isReference(v)?'animation-badge':''}">${isReference(v)?'ANIMATION-ONLY TARGET':'SIMULATED BODY'}</span>${isReference(v)?`<div class="stats"><div class="stat"><strong>Animation</strong><span>not simulated</span></div><div class="stat"><strong>${referenceDetail}</strong><span>pose source</span></div><div class="stat"><strong>N/A</strong><span>physical outcomes</span></div></div>`:`<div class="stats"><div class="stat"><strong class="${native&&!complete?'bad':'good'}">${native?(complete?'Complete trace':'Ended early'):(v.metrics.fall_step==null?'No fall':'Frame '+v.metrics.fall_step)}</strong><span>${native?'recording status':'first fall'}</span></div><div class="stat"><strong>${metric(v,'mean_achieved_along_command_speed_walk')}</strong><span>steady signed speed</span></div><div class="stat"><strong>${metric(v,'commanded_velocity_rmse_walk')}</strong><span>velocity RMSE</span></div></div>`}<div class="live" data-live="${esc(v.id)}"></div>${native?`<div class="foot">${esc(v.provenance.record_id)} · source ${esc(v.provenance.source_commit.slice(0,10))} · ${esc(v.provenance.position_field)}</div>`:''}</div>`}).join('');
document.querySelectorAll('#legend input').forEach(el=>el.onchange=()=>{el.checked?selected.add(el.dataset.id):selected.delete(el.dataset.id);draw()});
const protocol=DATA.protocol; [protocol.warmup_steps,protocol.start_blend_steps,protocol.walk_steps,protocol.stop_blend_steps].forEach((n,i)=>{const el=document.createElement('i');el.style.flex=n;$('phasebar').appendChild(el)});
function fit(canvas,context){const dpr=Math.min(devicePixelRatio||1,2),r=canvas.getBoundingClientRect();if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)){canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);context.setTransform(dpr,0,0,dpr,0,0)}return r}
function axes(p,view){return view==='side'?[p[1],p[2]]:view==='front'?[p[0],p[2]]:[p[0],p[1]]}
function bounds(view,camera){return DATA.projection_bounds[camera][view]}
function phaseText(f){const protocol=f.protocol_phase?`${f.protocol_phase} ${((f.phase_progress||0)*100).toFixed(0)}%`:'';const gait=f.gait_phase?` · ${f.gait_phase}`:'';const ref=f.reference_phase==null?'':` · ref ${(f.reference_phase*100).toFixed(1)}%`;return protocol+gait+ref}
function drawSkeleton(v,x,y,w,h,view,camera){const index=Math.min(frame,v.frames.length-1),f=v.frames[index];if(!f)return;const b=bounds(view,camera),root=f.positions[DATA.pelvis],map=p=>{let q=axes(p,view),r=axes(root,view);if(camera==='follow')q=[q[0]-r[0],view==='top'?q[1]-r[1]:q[1]];const scale=Math.min((w-35)/(b[1]-b[0]),(h-60)/(b[3]-b[2]));return[x+w/2+(q[0]-(b[0]+b[1])/2)*scale,y+h/2-(q[1]-(b[2]+b[3])/2)*scale]};ctx.save();ctx.beginPath();ctx.rect(x,y,w,h);ctx.clip();if(view!=='top'){const floor=map([0,0,0])[1];ctx.strokeStyle='#435066';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x+12,floor);ctx.lineTo(x+w-12,floor);ctx.stroke()}if(!isReference(v)&&view==='side'&&camera==='world'){ctx.strokeStyle=v.color+'55';ctx.beginPath();for(let i=0;i<=index;i+=4){const p=v.frames[i].positions[DATA.pelvis],q=map(p);i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1])}ctx.stroke()}ctx.strokeStyle=v.color;ctx.lineWidth=3;ctx.lineCap='round';for(let i=0;i<DATA.parents.length;i++){const p=DATA.parents[i];if(p<0)continue;const a=map(f.positions[p]),b2=map(f.positions[i]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b2);ctx.stroke()}for(let i=0;i<f.positions.length;i++){const q=map(f.positions[i]);ctx.fillStyle=i===DATA.left_foot&&Number(f.left_supported)>0||i===DATA.right_foot&&Number(f.right_supported)>0?'#63e6be':(Number(f.nonfoot_support)>0&&i===DATA.pelvis?'#ff5d68':v.color);ctx.beginPath();ctx.arc(q[0],q[1],i===DATA.pelvis?5:3,0,Math.PI*2);ctx.fill()}ctx.fillStyle='#f4f7fb';ctx.font='700 13px Segoe UI';ctx.fillText(v.label,x+14,y+22);ctx.fillStyle=Number(f.nonfoot_support)>0?'#ff7b72':'#9ba9bc';ctx.font='12px Segoe UI';const status=frame>=v.frames.length?`trace ended · ${phaseText(f)}`:(isReference(v)?`Animation only · ${phaseText(f)}`:`${phaseText(f)} · speed ${Number(f.achieved_speed??f.com_velocity[1]).toFixed(1)}`);ctx.fillText(status,x+14,y+40);ctx.restore()}
function drawChart(){const r=fit(chart,cctx),W=r.width,H=r.height;cctx.clearRect(0,0,W,H);cctx.fillStyle='#9ba9bc';cctx.font='12px Segoe UI';cctx.fillText('Physics pelvis displacement (world units) — animation excluded',14,20);const vals=[];for(const v of DATA.variants)if(selected.has(v.id)&&!isReference(v)){const y0=v.frames[0].positions[DATA.pelvis][1];vals.push(...v.frames.map(f=>f.positions[DATA.pelvis][1]-y0))}const mn=Math.min(0,...vals),mx=Math.max(1,...vals),mapX=i=>45+i/(maxFrame||1)*(W-60),mapY=y=>H-24-(y-mn)/(mx-mn)*(H-52);cctx.strokeStyle='#263142';cctx.beginPath();cctx.moveTo(45,mapY(0));cctx.lineTo(W-15,mapY(0));cctx.stroke();for(const v of DATA.variants)if(selected.has(v.id)&&!isReference(v)){const y0=v.frames[0].positions[DATA.pelvis][1];cctx.strokeStyle=v.color;cctx.lineWidth=2;cctx.beginPath();v.frames.forEach((f,i)=>{const q=[mapX(i),mapY(f.positions[DATA.pelvis][1]-y0)];i?cctx.lineTo(...q):cctx.moveTo(...q)});cctx.stroke()}cctx.strokeStyle='#f4f7fb66';cctx.beginPath();cctx.moveTo(mapX(frame),24);cctx.lineTo(mapX(frame),H-20);cctx.stroke()}
function draw(){
  const visible=DATA.variants.filter(v=>selected.has(v.id)), width=motion.getBoundingClientRect().width;
  const cols=Math.max(1,Math.min(visible.length,Math.floor(width/300))),rows=Math.max(1,Math.ceil(visible.length/cols));
  motion.style.height=(rows>1?rows*310:480)+'px';
  const r=fit(motion,ctx),W=r.width,H=r.height,pw=W/cols,ph=H/rows;
  ctx.clearRect(0,0,W,H);
  visible.forEach((v,i)=>drawSkeleton(v,(i%cols)*pw,Math.floor(i/cols)*ph,pw,ph,$('view').value,$('camera').value));
  document.querySelectorAll('[data-card]').forEach(el=>el.style.display=selected.has(el.dataset.card)?'':'none');
  for(const v of DATA.variants){const el=document.querySelector(`[data-live="${CSS.escape(v.id)}"]`);if(!el)continue;const f=v.frames[Math.min(frame,v.frames.length-1)],fmt=(x,d=1)=>x==null?'—':Number(x).toFixed(d),support=f.left_supported==null?'N/A':`${f.left_supported?'L✓':'L–'} ${f.right_supported?'R✓':'R–'}`;el.innerHTML=isReference(v)?`<div><b>${esc(phaseText(f)||'—')}</b><small>synchronized phase</small></div><div><b>N/A</b><small>support</small></div><div><b>N/A</b><small>slip</small></div>`:`<div><b>${fmt(f.requested_speed)}</b><small>requested speed</small></div><div><b>${fmt(f.achieved_speed)}</b><small>achieved speed</small></div><div><b>${fmt(f.pelvis_height)}</b><small>pelvis height</small></div><div><b>${support}</b><small>foot support</small></div><div><b>${fmt(f.left_load,2)} / ${fmt(f.right_load,2)}</b><small>L / R impulse load</small></div><div><b>${fmt(f.slip_speed)}</b><small>max contact slip</small></div>`}
  $('clock').textContent=`${(frame*DATA.dt).toFixed(2)} s · frame ${frame}/${maxFrame}`;$('timeline').value=frame;drawChart()
}
function tick(ts){if(playing){acc+=ts-last;const per=1000*DATA.dt/Number($('rate').value);while(acc>=per){frame=(frame+1)%(maxFrame+1);acc-=per}draw()}last=ts;requestAnimationFrame(tick)}
$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'❚❚ Pause':'▶ Play'};$('timeline').oninput=e=>{frame=Number(e.target.value);draw()};$('view').onchange=draw;$('camera').onchange=draw;window.onresize=draw;window.onkeydown=e=>{if(e.key==='ArrowRight'||e.key==='ArrowLeft'){frame=Math.max(0,Math.min(maxFrame,frame+(e.key==='ArrowRight'?1:-1)));draw()}};draw();requestAnimationFrame(tick);
</script></body></html>'''
    return template.replace("__DATA__", encoded)


def export(args: argparse.Namespace) -> dict[str, Path]:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise SystemExit("viewer output directory must be empty")
    fixture = Fixture(SimpleNamespace(
        source_root=args.source_root, exe=args.exe, library=args.library,
        floor=args.floor, workers=1, bound_speed=args.bound_speed,
        command_speed=args.command_speed, tracker_task="PLAY-01", control_mode=args.control_mode))
    if fixture.identity["git"]["dirty_files"]:
        raise SystemExit("viewer export requires a clean source worktree")
    checkpoint_state = checkpoint_identity = report_binding = None
    try:
        if args.checkpoint:
            checkpoint_state, checkpoint_identity = load_viewer_checkpoint(args, fixture)
        else:
            if not args.comparison_report:
                raise ValueError("diagnostic export requires --comparison-report")
            if args.control_mode != "direct":
                raise ValueError("three-control diagnostic export uses --control-mode direct")
            report_path = Path(args.comparison_report).resolve()
            report = json.loads(report_path.read_text(encoding="utf-8"))
            validate_comparison_report(report, fixture)
            report_binding = {"path": str(report_path), "sha256": sha256(report_path),
                              "source_commit": report["identity"]["git"]["commit"]}
    except (ValueError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
    runner = Runner(fixture, out)
    try:
        runner.ensure_workers(1)
        policy = None
        if checkpoint_state is not None:
            policy = build_policy(runner.builder.dimension, runner.builder.action_count, fixture.spec)
            policy.load_state_dict(checkpoint_state["policy"], strict=True)
            policy.eval()
        case = {"id": "unperturbed-control", "perturbed": False, "impulse": None,
                "command_speed": float(args.command_speed)}
        production = runner.run_episode(
            runner.workers[0], "reference", case, label="viewer-production",
            purpose="diagnostic", collect_teacher=policy is None, capture_visual=True)
        if policy is None:
            samples = production["teacher_samples"]
            if len(samples["oracle_actions"]) != fixture.episode_steps:
                raise RuntimeError("production oracle trajectory is incomplete")
            replay = {
                "actions": np.asarray(samples["oracle_actions"], dtype=np.float32),
                "poses": np.asarray(samples["oracle_poses"], dtype=np.float32),
            }
            pose_only = runner.run_episode(
                runner.workers[0], "direct", case, replay=replay, label="viewer-pose-only",
                purpose="diagnostic", capture_visual=True)
            zero = runner.run_episode(
                runner.workers[0], "direct", case, label="viewer-zero",
                purpose="diagnostic", capture_visual=True)
            variants = [
                _variant("production", "Production walker", "#63e6be", production),
                _variant("pose-only", "Pose-only oracle", "#b59cff", pose_only),
                _variant("zero", "Zero residual", "#ff9f68", zero),
            ]
        else:
            controller = fixture.policy_controller
            zero = runner.run_episode(
                runner.workers[0], controller, case, label="viewer-zero-correction",
                purpose="diagnostic", capture_visual=True)
            learned = runner.run_episode(
                runner.workers[0], controller, case, policy=policy, stochastic=False,
                label="viewer-learned-checkpoint", purpose="diagnostic", capture_visual=True)
            zero_label = "Production + zero correction" if args.control_mode == "production-residual" else "Direct zero residual"
            variants = [
                _variant("production", "Production walker", "#63e6be", production),
                _variant("zero-correction", zero_label, "#ff9f68", zero),
                _variant("learned", "Learned checkpoint", "#b59cff", learned),
            ]
        if any(len(variant["frames"]) != fixture.episode_steps for variant in variants):
            raise RuntimeError("viewer requires full-length diagnostic trajectories")
        bodies = runner.manifest["bodies"]
        data = {
            "schema": SCHEMA,
            "tracker_task": "PLAY-01",
            "captured_utc": utc_now(),
            "identity": fixture.identity,
            "comparison_report": report_binding,
            "checkpoint": checkpoint_identity,
            "qualification_holdout_opened": False,
            "optimization_performed": False,
            "command_speed": float(args.command_speed),
            "dt": fixture.dt,
            "episode_steps": fixture.episode_steps,
            "protocol": {key: getattr(fixture, key) for key in (
                "warmup_steps", "start_blend_steps", "walk_steps", "stop_blend_steps")},
            "body_names": [body["name"] for body in bodies],
            "parents": [int(body["parent"]) for body in bodies],
            "pelvis": int(runner.manifest["landmarks"]["pelvis"]),
            "left_foot": int(runner.manifest["landmarks"]["left_foot"]),
            "right_foot": int(runner.manifest["landmarks"]["right_foot"]),
            "variants": variants,
            "counters": runner.counter_snapshot(),
        }
        if args.include_morrowind_reference:
            data = add_morrowind_reference(data, Path(args.library))
        data_path = out / "viewer-data.json"
        data_path.write_text(json.dumps(data, indent=1, allow_nan=False) + "\n", encoding="utf-8")
        data["data_sha256"] = sha256(data_path)
        html_path = out / "PLAY-01-motion-viewer.html"
        html_path.write_text(viewer_document(data), encoding="utf-8")
        summary_path = out / "SUMMARY.txt"
        summary_path.write_text(
            "PLAY-01 motion viewer\n"
            f"source: {fixture.identity['git']['commit']}\n"
            f"executable sha256: {fixture.identity['executable_sha256']}\n"
            f"comparison report sha256: {report_binding['sha256'] if report_binding else 'not applicable'}\n"
            f"checkpoint sha256: {checkpoint_identity['sha256'] if checkpoint_identity else 'none'}\n"
            f"checkpoint source: {checkpoint_identity['source_commit'] if checkpoint_identity else 'none'}\n"
            f"viewer data sha256: {sha256(data_path)}\n"
            f"html sha256: {sha256(html_path)}\n"
            f"physics steps: {runner.physics_steps}\n"
            "qualification holdouts opened: false\n"
            "optimization performed: false\n",
            encoding="utf-8")
        return {"html": html_path, "data": data_path, "summary": summary_path}
    finally:
        runner.close()


def render_existing(data_path: Path, out: Path, reference_library: Path | None = None) -> dict[str, Path]:
    """Render recorded physical data without constructing a fixture or worker."""
    data_path, out = data_path.resolve(), out.resolve()
    data = json.loads(data_path.read_text(encoding="utf-8"))
    original_path, original_hash = data_path, sha256(data_path)
    if reference_library is not None:
        data = add_morrowind_reference(data, reference_library)
    data["presentation"] = {"original_data_path": str(original_path), "original_data_sha256": original_hash,
                            "renderer_source_sha256": sha256(Path(__file__)),
                            "new_physics_steps": 0, "optimization_performed": False}
    html_path = out / "PLAY-01-motion-viewer.html"
    if out.exists() and any(out.iterdir()):
        raise ValueError("render destination already exists; choose a new --out directory")
    # Validate before creating any output. Existing recordings remain untouched.
    viewer_document(data)
    out.mkdir(parents=True, exist_ok=True)
    data_path = out / "viewer-data.json"
    data_path.write_text(json.dumps(data, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    data["data_sha256"] = sha256(data_path)
    document = viewer_document(data)
    html_path.write_text(document, encoding="utf-8")
    summary_path = out / "SUMMARY.txt"
    reference = next((v["reference"] for v in data["variants"] if v.get("kind") == REFERENCE_KIND), None)
    summary_path.write_text(
        "PLAY-01 motion viewer: render saved trajectories\n"
        f"original data sha256: {original_hash}\n"
        f"renderer sha256: {data['presentation']['renderer_source_sha256']}\n"
        f"checkpoint sha256: {(data.get('checkpoint') or {}).get('sha256', 'none')}\n"
        f"reference clip sha256: {reference['clip_sha256'] if reference else 'none'}\n"
        f"viewer data sha256: {sha256(data_path)}\n"
        f"html sha256: {sha256(html_path)}\n"
        "new physics steps: 0\noptimization performed: false\n"
        "qualification holdouts opened by renderer: false\n"
        "Morrowind reference: native-rate continuous kinematic loop; no physical outcome or world travel claims.\n",
        encoding="utf-8")
    return {"html": html_path, "data": data_path, "summary": summary_path}


def render_native(manifest_path: Path, out: Path) -> dict[str, Path]:
    """Render hash-bound native production/candidate traces without running the game."""
    out = out.resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError("render destination already exists; choose a new --out directory")
    data = load_native_comparison(manifest_path)
    data["presentation"] = {"renderer_source_sha256": sha256(Path(__file__)),
                            "new_physics_steps": 0, "optimization_performed": False}
    viewer_document(data)
    out.mkdir(parents=True, exist_ok=True)
    data_path = out / "viewer-data.json"
    data_path.write_text(json.dumps(data, indent=1, allow_nan=False) + "\n", encoding="utf-8")
    data["data_sha256"] = sha256(data_path)
    html_path = out / "PLAY-01-motion-viewer.html"
    html_path.write_text(viewer_document(data), encoding="utf-8")
    summary_path = out / "SUMMARY.txt"
    simulated = [variant for variant in data["variants"] if variant.get("kind") == "simulated_body"]
    summary_path.write_text(
        "PLAY-01 native production/candidate motion viewer\n"
        f"manifest sha256: {data['native_manifest']['sha256']}\n"
        f"renderer sha256: {data['presentation']['renderer_source_sha256']}\n"
        f"viewer data sha256: {sha256(data_path)}\n"
        f"html sha256: {sha256(html_path)}\n"
        f"variants: {', '.join(variant['id'] for variant in data['variants'])}\n"
        f"incomplete physical traces: {', '.join(variant['id'] for variant in simulated if not variant['metrics']['trace_complete']) or 'none'}\n"
        "synchronization: native 60 Hz protocol phase; failed candidates remain ended\n"
        "animation target: exact recorded targetPosition; no physical outcome claims\n"
        "slip: maximum reported foot contact-point tangential speed per frame\n"
        "new physics steps: 0\noptimization performed: false\n"
        "qualification holdouts opened by renderer: false\n",
        encoding="utf-8")
    return {"html": html_path, "data": data_path, "summary": summary_path}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root")
    parser.add_argument("--exe")
    parser.add_argument("--library")
    parser.add_argument("--comparison-report")
    parser.add_argument("--checkpoint", help="Export this learned checkpoint with production/zero-correction baselines")
    parser.add_argument("--checkpoint-sha256", help="Exact SHA-256 of the checkpoint to deserialize")
    parser.add_argument("--training-manifest", help="Manifest whose run/case contract the checkpoint must match")
    parser.add_argument("--allow-ancestor-source", action="store_true", help="Permit verified documentation/viewer-only source ancestry")
    parser.add_argument("--control-mode", choices=["direct", "production-residual"], default="direct")
    parser.add_argument("--render-data", help="Render existing viewer-data.json without running any physics")
    parser.add_argument("--native-comparison-manifest",
                        help="Render hash-bound native production/candidate traces without running physics")
    parser.add_argument("--include-morrowind-reference", action="store_true",
                        help="Add the captured native-rate forward animation loop from the matching --library (no physics)")
    parser.add_argument("--out", required=True)
    parser.add_argument("--floor", default="tiled", choices=["tiled", "mesh", "box"])
    parser.add_argument("--bound-speed", type=float, default=None)
    parser.add_argument("--command-speed", type=float, default=45.0)
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    if args.include_morrowind_reference and not args.library:
        parser.error("--include-morrowind-reference requires --library")
    if args.render_data or args.native_comparison_manifest:
        if args.checkpoint or args.comparison_report:
            parser.error("recorded-data rendering cannot be combined with checkpoint or diagnostic export")
        if args.render_data and args.native_comparison_manifest:
            parser.error("choose --render-data or --native-comparison-manifest")
        if args.native_comparison_manifest:
            if args.include_morrowind_reference:
                parser.error("native comparison already carries its exact adapted target")
            paths = render_native(Path(args.native_comparison_manifest), Path(args.out))
        else:
            paths = render_existing(Path(args.render_data), Path(args.out),
                                    Path(args.library) if args.include_morrowind_reference else None)
    else:
        if not all((args.source_root, args.exe, args.library)):
            parser.error("physical export requires --source-root, --exe and --library")
        if args.checkpoint and not args.checkpoint_sha256:
            parser.error("--checkpoint requires --checkpoint-sha256")
        if not args.checkpoint and (args.checkpoint_sha256 or args.training_manifest or args.allow_ancestor_source):
            parser.error("checkpoint verification options require --checkpoint")
        if args.checkpoint and args.comparison_report:
            parser.error("choose either checkpoint export or comparison-report diagnostics")
        paths = export(args)
    print(json.dumps({key: str(value) for key, value in paths.items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
