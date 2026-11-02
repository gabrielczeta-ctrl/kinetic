"""Check the recorded-input fall witness, NOT walking acceptance or full parity.

Runs the replay driver and requires the native fall frame to be reproduced
before physical divergence. Native modes are read from decimal observed logs.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import struct
from pathlib import Path


def replay_environment(environ):
    # C++ diagnostics use getenv presence, including a value of "0".
    names = (
        "OPENMW_RAGDOLL_CONTINUOUS_SWING",
        "OPENMW_RAGDOLL_DIRECTED_HIP_YAW",
        "OPENMW_RAGDOLL_KEEP_EMERGENCY_STANCE",
        "OPENMW_RAGDOLL_BALANCED_COORDINATED_TURN",
        "OPENMW_RAGDOLL_HOLD_TRANSFER_COM",
        "OPENMW_RAGDOLL_MEASURED_FOOT_HEADING",
        "OPENMW_RAGDOLL_BODY_ALIGNED_TRAVEL",
        "OPENMW_RAGDOLL_HEADING_ALIGNED_SPEED",
        "OPENMW_RAGDOLL_FEASIBLE_PARENT_TARGET",
        "OPENMW_RAGDOLL_TURN_BEND_FROM_STEP",
        "OPENMW_RAGDOLL_TURN_BEND_TO_FACING",
        "OPENMW_JOLT_GEOMETRIC_SECOND_CONTACT",
        "OPENMW_JOLT_CONTACT_SELECTION",
        "OPENMW_RAGDOLL_REPLAY_COMPUTE_HORIZONTAL_ROOT",
        "OPENMW_RAGDOLL_REPLAY_PLAYER_RAW_TRAVEL",
        "OPENMW_RAGDOLL_REPLAY_NPC_RAW_TRAVEL",
        "OPENMW_RAGDOLL_REPLAY_NATIVE_SUPPORT_RESET",
        "OPENMW_RAGDOLL_REPLAY_RESTART_OBSERVATION",
        "OPENMW_RAGDOLL_REPLAY_TRANSFER_BRAKING",
        "OPENMW_RAGDOLL_REPLAY_LEGACY_TRANSFER",
        "OPENMW_RAGDOLL_REPLAY_SKIP_BRACE_LIMITS",
        "OPENMW_RAGDOLL_REPLAY_SKIP_RECOVERY_REQUEST",
        "OPENMW_RAGDOLL_REPLAY_SKIP_BRACE_TARGETS",
        "OPENMW_RAGDOLL_REPLAY_FIXED_ROOT",
    )
    return {name: {"present": name in environ, "value": environ.get(name)} for name in names}


def native_fall(log, allow_no_fall=False, actor_kind="player"):
    if actor_kind not in ("player", "npc"):
        raise ValueError("unknown actor kind")
    player_flag = "1" if actor_kind == "player" else "0"
    selected_actors = set()
    frames, modes = {}, {}
    for line in log.read_text(errors="replace").splitlines():
        fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
        if "RAGDOLL_REPLAY_FRAME " in line:
            step, frame = int(fields["step"]), int(fields["frame"])
            if step in frames:
                raise ValueError("duplicate replay step; select one capture")
            frames[step] = frame
        if "RAGDOLL_STEP " in line and fields.get("phase") == "observed" and fields.get("player") == player_flag:
            actor = re.search(r'actor=("(?:\\.|[^"\\])*"|[^\s]+)', line)
            selected_actors.add(actor.group(1) if actor else "<unnamed>")
            if len(selected_actors) > 1:
                raise ValueError("multiple selected actors; use an isolated capture")
            if fields.get("integer_radix") != "10":
                raise ValueError("native step radix not explicit")
            step = int(fields["step"])
            if step in modes:
                raise ValueError("duplicate observed player step")
            modes[step] = int(fields["mode"])
    if not frames or sorted(frames.values()) != list(range(len(frames))):
        raise ValueError("missing or noncontiguous replay frame mapping")
    if any(step not in modes for step in frames):
        raise ValueError("missing native observations")
    falls = [frame for step, frame in frames.items() if modes[step] in (3, 4)]
    if not falls and not allow_no_fall:
        raise ValueError("capture is not a native Falling/Fallen witness")
    return min(falls) if falls else None, len(frames)


def verify(result, native_frame, frame_count):
    if result.get("comparison_scope") != "physical_state":
        raise ValueError("unexpected comparison scope")
    if result.get("frames") != frame_count:
        raise ValueError("native/offline frame counts differ")
    if result.get("offline_first_fall_frame") != native_frame:
        raise ValueError("fall onset does not match native")
    divergent = result.get("first_divergent_frame")
    if divergent is not None and (native_frame is None or divergent <= native_frame):
        raise ValueError("physical divergence precedes or coincides with fall onset")
    if result.get("swing_history_compared"):
        if "first_swing_history_difference_frame" not in result:
            raise ValueError("missing swing history comparison result")
        if result["first_swing_history_difference_frame"] is not None:
            raise ValueError("swing history does not match native")
    if result.get("equivalence_proven") is not False:
        raise ValueError("recorded-input witness must not assert full equivalence")


def verify_exact_physical_curve(path, frame_count):
    """Require complete decoded physical equality, not autonomous equivalence."""
    fields = ("max_translation_error", "max_linear_velocity_error",
        "max_angular_velocity_error", "max_quaternion_component_error")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != frame_count or frame_count <= 0:
        raise ValueError("physical error curve has incomplete frame coverage")
    for index, row in enumerate(rows):
        if row.get("frame") != str(index):
            raise ValueError("physical error curve frames are missing, duplicated or reordered")
        for field in fields:
            try:
                value = float(row[field])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("missing or invalid physical error channel: " + field) from error
            if not math.isfinite(value) or value != 0:
                raise ValueError(f"physical mismatch at frame {index}: {field}={value}")
    return {"frames": frame_count, "maximum_errors": dict.fromkeys(fields, 0.0),
        "scope": "decoded physical state; quaternion sign equivalence; recorded adapter data supplied"}


def context_dependencies(context):
    """Hash external shape bytes, not merely their path in the context file."""
    lines = context.read_text(encoding="utf-8").splitlines()
    version = int(lines[0].split()[0])
    if version not in (4, 5):
        return {}
    bone_count = int(lines[6])
    ground = lines[8 + bone_count]
    quote = ground.find('"')
    if quote < 0:
        raise ValueError("captured ground path missing from context")
    path = Path(json.loads(ground[quote:]))
    if not path.is_absolute():
        raise ValueError("captured ground path must be absolute")
    data = path.read_bytes()
    if not data or len(data) > 16 * 1024 * 1024:
        raise ValueError("captured ground shape missing or oversized")
    return {"ground_shape": {"path": str(path.resolve()), "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest()}}


def verify_world_positions(path, frame_count, tolerance=0.0001):
    # Native recordings store float32 model poses. Reconstructing world points
    # introduces a small coordinate round-trip error, unlike exact model checks.
    with path.open(newline='', encoding='utf-8') as stream:
        rows=list(csv.DictReader(stream))
    if len(rows)!=frame_count or frame_count<=0:
        raise ValueError('Incomplete world position coverage')
    maximum=0.0
    for index,row in enumerate(rows):
        value=float(row['max_world_position_error'])
        if row['frame']!=str(index) or not math.isfinite(value) or value<0 or value>tolerance:
            raise ValueError('World position mismatch at frame '+str(index))
        maximum=max(maximum,value)
    return {'frames':frame_count, 'max_world_position_error':maximum, 'tolerance_game_units':tolerance,
        'scope':'all bodies; native world positions reconstructed from recorded model pose and root'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("driver", "capture", "context", "profile", "native-log", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--require-exact-physical", action="store_true",
        help="Also require zero decoded physical error throughout the complete recording")
    parser.add_argument("--expect-no-fall", action="store_true",
        help="Verify a surviving native recording; requires --require-exact-physical")
    parser.add_argument("--profile-override", action="append", default=[],
        help="Recorded controller parameter override, e.g. velocity_lead_fraction=1; repeat as needed")
    parser.add_argument("--actor-kind", choices=("player", "npc"), default="player")
    args = parser.parse_args()
    if args.expect_no_fall and not args.require_exact_physical:
        parser.error("--expect-no-fall requires --require-exact-physical")
    fall, count = native_fall(args.native_log, allow_no_fall=args.expect_no_fall, actor_kind=args.actor_kind)
    if args.expect_no_fall and fall is not None:
        raise ValueError("native recording contains a fall despite --expect-no-fall")
    dependencies = context_dependencies(args.context)
    driver_environment = os.environ.copy()
    run = subprocess.run([str(args.driver.resolve()), "--replay-capture", str(args.capture.resolve()),
        str(args.context.resolve()), str(args.output.resolve()), str(args.profile.resolve()), *args.profile_override],
        capture_output=True, text=True, check=True, env=driver_environment)
    reports = [json.loads(line) for line in run.stdout.splitlines() if line.startswith('{"frames":')]
    if len(reports) != 1:
        raise ValueError("expected one driver report")
    result = reports[0]
    with args.capture.open("rb") as capture_stream:
        header = capture_stream.read(48)
    if len(header) >= 48 and struct.unpack_from("<I", header, 44)[0] == 32:
        if result.get("swing_history_compared") is not True:
            raise ValueError("extended recording requires swing history comparison")
    verify(result, fall, count)
    if context_dependencies(args.context) != dependencies:
        raise ValueError("replay context dependency changed during verification")
    exact = None
    world = None
    if args.require_exact_physical:
        if result.get("first_divergent_frame") is not None:
            raise ValueError("driver reports physical divergence")
        exact = verify_exact_physical_curve(args.output / "physical-error-curve.csv", count)
        if result.get('moving_reference_root'):
            world = verify_world_positions(args.output / 'world-position-error-curve.csv', count)
    report = {"actor_kind": args.actor_kind, "replay_environment": replay_environment(driver_environment),
        "context_version": int(args.context.read_text(encoding="utf-8").split()[0]),
        "online_pose_selection": args.context.read_text(encoding="utf-8").split()[0] == "5",
        "witness_reproduced": fall is not None, "walking_accepted": False,
        "verification_kind": "surviving_recording" if args.expect_no_fall else "fall_witness",
        "context_dependencies": dependencies,
        "exact_recorded_physical_match": exact,
        "exact_recorded_swing_history_match": True if result.get("swing_history_compared") else None,
        "world_position_match": world,
        "native_first_fall_frame": fall, "driver_result": result,
        "sha256": {name: hashlib.sha256(getattr(args, name).read_bytes()).hexdigest()
            for name in ("driver", "capture", "context", "profile", "native_log")}}
    (args.output / "witness-verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
