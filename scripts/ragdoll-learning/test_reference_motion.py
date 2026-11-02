"""Analytic frame/velocity checks and malformed-source controls for PLAY-01."""

from __future__ import annotations

import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

import reference_motion as motion


NAMES = ["pelvis", "left_hand", "right_hand", "left_foot", "right_foot"]


def profile() -> dict:
    return {"id": "test_articulated", "bodies": [{"name": "pelvis"}]
            + [{"name": name, "parent": "pelvis"} for name in NAMES[1:]],
            "landmarks": {name: name for name in NAMES}}


def static_rows(frames: int = 4) -> list[list]:
    return [[frame, body, body, 0, 0, 0, 0, 0, 1, 0, 30]
            for frame in range(frames) for body in range(len(NAMES))]


class ReferenceMotionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.csv_path = self.directory / "animation.csv"
        self.profile_path = self.directory / "humanoid.yaml"
        self.profile_path.write_text(yaml.safe_dump(profile()), encoding="utf-8")
        self.skeleton = motion.load_body_map(self.profile_path, NAMES)

    def write_rows(self, rows):
        with self.csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["frame", "body", *motion.TRANSFORM_FIELDS, "req_x", "req_y"])
            writer.writerows(rows)

    def reference(self, rows, hz=60):
        self.write_rows(rows)
        data = motion.read_animation_csv(self.csv_path, len(NAMES))
        return motion.build_reference(data, self.skeleton, hz)

    def test_parent_rotation_and_translation_are_removed_from_child_joint_motion(self):
        # Parent rotates around Z at pi rad/s and translates at 3 units/s.
        # Each child keeps a constant 90-degree X rotation in its parent frame.
        rows = []
        for frame in range(5):
            t, angle = frame / 30, np.pi * frame / 30
            c, s = np.cos(angle / 2), np.sin(angle / 2)
            rows.append([frame, 0, 3*t, 0, 0, 0, 0, s, c, 0, 0])
            for body in range(1, len(NAMES)):
                rows.append([frame, body, 3*t + body*np.cos(angle), body*np.sin(angle), 0,
                             c/np.sqrt(2), s/np.sqrt(2), s/np.sqrt(2), c/np.sqrt(2), 0, 0])
        result = self.reference(rows, hz=30)
        self.assertEqual(result["frame_count"], 9)
        self.assertEqual(result["sample_hz"], 60)
        q = np.asarray(result["parent_local_rotations_xyzw"])
        np.testing.assert_allclose(q[:, 1:], np.broadcast_to([1/np.sqrt(2), 0, 0, 1/np.sqrt(2)], q[:, 1:].shape), atol=1e-12)
        omega = np.asarray(result["parent_local_angular_velocities"])
        np.testing.assert_allclose(omega[:, 1:], 0, atol=1e-12)
        np.testing.assert_allclose(omega[:, 0], np.broadcast_to([0, 0, np.pi], (9, 3)), atol=1e-12)
        np.testing.assert_allclose(np.asarray(result["model_linear_velocities"])[:, 0],
                                   np.broadcast_to([3, 0, 0], (9, 3)), atol=1e-12)
        # Original frames recover exact local offsets. Between source frames,
        # model-position linear interpolation is explicitly an approximation.
        np.testing.assert_allclose(np.asarray(result["parent_local_positions"])[::2, 1:, 0],
                                   np.broadcast_to([1, 2, 3, 4], (5, 4)), atol=1e-12)

    def test_sign_equivalent_quaternions_do_not_create_motion_or_seam(self):
        rows = static_rows()
        for row in rows:
            if (row[0] + row[1]) % 2:
                row[5:9] = [-value for value in row[5:9]]
        result = self.reference(rows)
        np.testing.assert_allclose(result["parent_local_angular_velocities"], 0, atol=1e-12)
        self.assertEqual(result["loop_seam"]["seam_joint_rotation_step_max_radians"], 0)
        self.assertIsNone(result["physical_contacts"])

    def test_known_joint_speed_survives_sign_flip(self):
        rows = static_rows(6)
        for row in rows:
            if row[1] == 1:
                angle = 2.4 * row[0] / 60
                sign = -1 if row[0] % 2 else 1
                row[5:9] = [0, 0, sign*np.sin(angle/2), sign*np.cos(angle/2)]
        result = self.reference(rows)
        np.testing.assert_allclose(np.asarray(result["parent_local_angular_velocities"])[:, 1],
                                   np.broadcast_to([0, 0, 2.4], (6, 3)), atol=1e-12)

    def test_malformed_csv_is_rejected_before_resampling(self):
        cases = {}
        cases["missing_body"] = static_rows()[:-1]
        cases["duplicate"] = static_rows() + [static_rows()[0]]
        cases["missing_frame"] = [row for row in static_rows() if row[0] != 1]
        for label, column, value in [("nonfinite", 2, float("nan")), ("zero_quaternion", 8, 0),
                                      ("distorted_quaternion", 8, 2), ("bad_request", 9, 7), ("bad_body", 1, 5)]:
            cases[label] = static_rows()
            cases[label][0][column] = value
        for label, rows in cases.items():
            with self.subTest(label=label), self.assertRaises(ValueError):
                self.reference(rows)

    def test_seam_discontinuity_is_measured_without_smoothing_or_acceptance(self):
        rows = static_rows(5)
        for row in rows:
            row[2] += row[0] * 5
        result = self.reference(rows)
        seam = result["loop_seam"]
        self.assertEqual(seam["seam_model_position_step_max"], 20)
        self.assertEqual(seam["interior_model_position_step_max"], 5)
        self.assertEqual(seam["seam_root_relative_position_step_max"], 0)
        self.assertEqual(np.asarray(result["model_positions"])[-1, 0, 0], 20)
        self.assertNotIn("pass", result)

    def test_reference_errors_separate_root_drift_pose_and_end_effectors(self):
        reference = self.reference(static_rows())
        candidate = copy.deepcopy(reference)
        positions = np.asarray(candidate["model_positions"])
        positions[:, :, 0] += 5
        candidate["model_positions"] = positions.tolist()
        errors = motion.measure_reference_errors(reference, candidate)
        self.assertEqual(errors["root_position_model_units"]["rmse"], 5)
        self.assertEqual(errors["root_relative_end_effector_model_units"]["rmse"], 0)
        positions[:, 1, 1] += 2
        candidate["model_positions"] = positions.tolist()
        candidate["parent_local_rotations_xyzw"] = np.asarray(candidate["parent_local_rotations_xyzw"])
        candidate["parent_local_rotations_xyzw"][:, 1] = [0, 0, np.sin(.2), np.cos(.2)]
        errors = motion.measure_reference_errors(reference, candidate)
        self.assertAlmostEqual(errors["joint_pose_radians"]["rmse"], .2)
        self.assertEqual(errors["root_relative_end_effector_model_units"]["rmse"], 1)
        candidate["times_seconds"] = [0]
        with self.assertRaises(ValueError):
            motion.measure_reference_errors(reference, candidate)

    def test_profile_order_mapping_is_explicit_and_cycles_reject(self):
        reversed_names = NAMES[::-1]
        mapping = motion.load_body_map(self.profile_path, reversed_names)
        self.assertEqual(mapping["root_index"], 4)
        self.assertEqual(mapping["parent_indices"], [4, 4, 4, 4, -1])
        broken = profile()
        broken["bodies"][1]["parent"] = "right_hand"
        broken["bodies"][2]["parent"] = "left_hand"
        self.profile_path.write_text(yaml.safe_dump(broken), encoding="utf-8")
        with self.assertRaises(ValueError):
            motion.load_body_map(self.profile_path, NAMES)

    def test_hash_provenance_and_unknown_revision_are_explicit(self):
        self.write_rows(static_rows())
        replay = self.directory / "source.omwrdpl"
        replay.write_bytes(b"test bytes; the importer does not decode replay")
        metadata = {"schema_version": 1, "clip_name": "test-static", "source_hz": 60, "source_commit": None,
                    "source_replay": replay.name, "source_replay_sha256": motion.sha256(replay),
                    "source_csv_sha256": motion.sha256(self.csv_path), "profile_sha256": motion.sha256(self.profile_path),
                    "body_names": NAMES, "coverage_tags": ["forward"]}
        path = self.directory / "source.json"
        path.write_text(json.dumps(metadata), encoding="utf-8")
        result = motion.prepare(self.csv_path, self.profile_path, path, start_frame=1, frame_count=2)
        self.assertEqual(result["provenance"]["source_commit_status"], "unknown")
        self.assertEqual(result["source_frame_coordinates"], [1, 2])
        self.assertIn("diagonal", result["coverage_audit"]["classes_without_declared_coverage"])
        self.assertEqual(result["reference_error_self_check"]["joint_pose_radians"]["maximum"], 0)
        replay.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            motion.prepare(self.csv_path, self.profile_path, path)


if __name__ == "__main__":
    unittest.main()
