#!/usr/bin/env python3
"""Focused tests for the offline PLAY-01 physical-motion viewer."""

from __future__ import annotations

import unittest
import copy
import gzip
import json
from pathlib import Path
import tempfile
import numpy as np
from types import SimpleNamespace
from unittest.mock import patch

from play01_viewer import (IDENTITY_FIELDS, SCHEMA, add_morrowind_reference, load_native_comparison,
                           load_viewer_checkpoint, projection_bounds, render_existing, render_native,
                           validate_checkpoint_source, validate_comparison_report,
                           validated_viewer_data, viewer_document)
from play01_runner import sha256, sha256_json, training_run_contract


def viewer_data():
    frame = {"positions": [[0, 0, 10], [-1, 0, 0], [1, 0, 0]], "com_velocity": [0, 0, 0],
             "left_supported": "0", "right_supported": "1", "nonfoot_support": "0"}
    return {"schema": SCHEMA, "body_names": ["pelvis", "left_foot", "right_foot"],
            "parents": [-1, 0, 0], "pelvis": 0, "left_foot": 1, "right_foot": 2, "dt": 1 / 60,
            "variants": [{"id": "production", "color": "#63e6be", "frames": [frame]}]}


def reference_fixture():
    data = viewer_data()
    data["dt"] = 0.25
    data["variants"][0]["frames"] *= 5
    rotations = np.tile([0., 0., 0., 1.], (2, 3, 1))
    rotations[1, 0] = [0., 0., np.sqrt(0.5), np.sqrt(0.5)]
    clip = {"case": "native-forward", "frame_count": 2, "duration_seconds": 1.,
            "source_frames": [10, 12], "parent_local_rotations_xyzw": rotations.tolist(),
            "root_positions": [[0, 0, 10], [0, 1, 11]], "csv_sha256": "a" * 64,
            "replay_sha256": "b" * 64}
    library = {"schema": "play01_motion_library_v1", "body_names": data["body_names"],
               "parent_indices": data["parents"], "sample_hz": 2.,
               "creation_context": {"sha256": "c" * 64}, "profile_sha256": "d" * 64,
               "creation_local_offsets": [[0, 0, 10], [-1, 0, -10], [1, 0, -10]],
               "clips": {"forward": clip}, "usage_rights": "Private captured native animation",
               "cases": {"native-forward": {"family": "forward", "decode": {
                   "csv_sha256": clip["csv_sha256"], "replay_sha256": clip["replay_sha256"]}}}}
    rehash_reference(data, library)
    return data, library


def rehash_reference(data, library):
    clip = library["clips"]["forward"]
    clip["sha256"] = sha256_json({k: v for k, v in clip.items() if k != "sha256"})
    library["sha256"] = sha256_json({k: v for k, v in library.items() if k != "sha256"})
    data["identity"] = {"library_sha256": library["sha256"], "context_sha256": "c" * 64,
                        "profile_sha256": "d" * 64}


class ViewerContractTests(unittest.TestCase):
    def test_native_comparison_uses_recorded_body_and_target_without_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            run.mkdir()
            library = {"schema": "play01_motion_library_v1",
                       "body_names": ["pelvis", "left_foot", "right_foot"],
                       "parent_indices": [-1, 0, 0]}
            library_path = root / "library.json"
            library_path.write_text(json.dumps(library), encoding="utf-8")
            batch = {"tracker_task": "PLAY-01", "source_commit": "a" * 40,
                     "engine_sha256": "b" * 64, "build_identity": {
                         "controller_mode": "animation_contact_root", "profile_sha256": "c" * 64,
                         "fixture": "test floor; 60 Hz"}, "preflight_records": [{
                             "id": "candidate", "passed": False,
                             "artifact_directories": [str(run)]}]}
            batch_path = root / "batch.json"
            batch_path.write_text(json.dumps(batch), encoding="utf-8")
            names = {"1": "pelvis", "2": "left_foot", "3": "right_foot"}
            def body(name, x):
                return {"name": name, "position": [x, 0, 10], "targetPosition": [x, 2, 11],
                        "linearVelocity": [2, 0, 0], "angularVelocity": [0, 0, 0],
                        "centerOfMass": [x, 0, 10]}
            rows = [{"kind": "capture_start", "tracker_task": "PLAY-01", "test": "same native test",
                     "common_seed": 1, "protocol": {"targetSpeed": 45, "segments": {
                         "1": {"segment": "steady", "duration": 2 / 60}}}}]
            for index in range(2):
                rows.append({"kind": "frame", "tracker_task": "PLAY-01", "test": "same native test",
                             "elapsed": (index + 1) / 60, "segment": "steady", "state": {
                                 "body_names": names,
                                 "bodies": {"pelvis": body("pelvis", 0),
                                            "left_foot": body("left_foot", -1),
                                            "right_foot": body("right_foot", 1)},
                                 "support": {"leftFoot": True, "rightFoot": False},
                                 "contacts": {"1": {"bodyIndex": 2, "normalImpulse": 4,
                                                      "position": [-1, 0, 10], "normal": [0, 0, 1]}},
                                 "controller": {"targetHorizontalVelocity": [0, 45, 0],
                                                "centerOfMassVelocity": [0, 40, 0],
                                                "pelvisHeight": 72, "gaitPhase": "left_stance",
                                                "coherentReferencePhase": .25}}})
            rows.append({"kind": "capture_stop", "termination": "completed"})
            trace_path = run / "trace.jsonl.gz"
            with gzip.open(trace_path, "wt", encoding="utf-8") as stream:
                for row in rows:
                    stream.write(json.dumps(row) + "\n")
            common = {"record_id": "candidate",
                      "trace": {"path": str(trace_path), "sha256": sha256(trace_path)},
                      "batch": {"path": str(batch_path), "sha256": sha256(batch_path)}}
            manifest = {"schema": "play01_native_viewer_manifest_v1", "tracker_task": "PLAY-01",
                        "library": {"path": str(library_path), "sha256": sha256(library_path)},
                        "variants": [
                            {**common, "id": "v22-body", "label": "body", "color": "#112233",
                             "kind": "simulated_body", "position_field": "position"},
                            {**common, "id": "v22-target", "label": "target", "color": "#445566",
                             "kind": "animation_target", "position_field": "targetPosition"}]}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            data = load_native_comparison(manifest_path)
            self.assertEqual(data["variants"][0]["frames"][0]["positions"][0], [0, 0, 10])
            self.assertEqual(data["variants"][1]["frames"][0]["positions"][0], [0, 2, 11])
            self.assertEqual(data["variants"][0]["frames"][0]["slip_speed"], 2)
            self.assertIsNone(data["variants"][1]["frames"][0]["slip_speed"])
            self.assertTrue(data["variants"][0]["metrics"]["trace_complete"])
            with patch("play01_viewer.Fixture", side_effect=AssertionError("fixture forbidden")), \
                    patch("play01_viewer.Runner", side_effect=AssertionError("worker forbidden")):
                paths = render_native(manifest_path, root / "viewer")
            html = paths["html"].read_text(encoding="utf-8")
            self.assertIn("SIMULATED BODY", html)
            self.assertIn("ANIMATION-ONLY TARGET", html)
            self.assertIn("max contact slip", html)

    def test_reference_preserves_recording_and_native_loop_clock(self):
        data, library = reference_fixture()
        before = copy.deepcopy(data)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "library.json"
            path.write_text(json.dumps(library), encoding="utf-8")
            with patch("play01_viewer.Fixture", side_effect=AssertionError("fixture forbidden")), \
                    patch("play01_viewer.Runner", side_effect=AssertionError("worker forbidden")):
                result = add_morrowind_reference(data, path)
            self.assertEqual(data, before)
            self.assertEqual(result["variants"][:-1], before["variants"])
            reference = result["variants"][-1]
            self.assertEqual(reference["reference"]["library_file_sha256"], sha256(path))
            self.assertEqual(reference["reference"]["source_frames"], [10, 12])
            frames = reference["frames"]
            self.assertEqual([f["phase"] for f in frames], [0, .25, .5, .75, 0])
            np.testing.assert_allclose(frames[0]["positions"], frames[4]["positions"])
            np.testing.assert_allclose(frames[2]["positions"][0], [0, 1, 11])
            for frame in frames:
                for child in (1, 2):
                    self.assertAlmostEqual(np.linalg.norm(np.subtract(
                        frame["positions"][child], frame["positions"][0])), np.sqrt(101))
                for field in ("com_velocity", "left_supported", "right_supported", "nonfoot_support"):
                    self.assertIsNone(frame[field])
            self.assertEqual(reference["metrics"], {})
            html = viewer_document(result)
            self.assertIn("Physics pelvis displacement", html)
            self.assertIn("selected.has(v.id)&&!isReference(v)", html)
            self.assertIn("physical outcomes", html)

    def test_reference_rejects_wrong_identity_layout_content_or_capture(self):
        for defect in ("identity", "layout", "content", "capture", "nonfinite", "timing"):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as directory:
                data, library = reference_fixture()
                if defect == "identity":
                    data["identity"]["context_sha256"] = "wrong"
                elif defect == "layout":
                    data["body_names"] = list(reversed(data["body_names"]))
                elif defect == "content":
                    library["clips"]["forward"]["root_positions"][0][2] += 1
                elif defect == "capture":
                    library["cases"]["native-forward"]["decode"]["csv_sha256"] = "wrong"
                    rehash_reference(data, library)
                elif defect == "nonfinite":
                    library["clips"]["forward"]["parent_local_rotations_xyzw"][0][0] = [0, 0, 0, 0]
                    rehash_reference(data, library)
                else:
                    library["clips"]["forward"]["duration_seconds"] = 2
                    rehash_reference(data, library)
                path = Path(directory) / "library.json"
                path.write_text(json.dumps(library), encoding="utf-8")
                with self.assertRaises(ValueError):
                    add_morrowind_reference(data, path)

    def test_kinematic_support_and_success_claims_rejected(self):
        data = viewer_data()
        variant = data["variants"][0]
        variant["kind"] = "kinematic_reference"
        variant["metrics"] = {"fall_step": None}
        with self.assertRaisesRegex(ValueError, "physical outcome"):
            validated_viewer_data(data)
        variant["metrics"] = {}
        with self.assertRaisesRegex(ValueError, "physical telemetry"):
            validated_viewer_data(data)

    def test_render_reference_creates_new_bound_artifacts_without_workers(self):
        data, library = reference_fixture()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original, libpath = root / "original.json", root / "library.json"
            original.write_text(json.dumps(data), encoding="utf-8")
            libpath.write_text(json.dumps(library), encoding="utf-8")
            original_hash = sha256(original)
            with patch("play01_viewer.Fixture", side_effect=AssertionError("fixture forbidden")), \
                    patch("play01_viewer.Runner", side_effect=AssertionError("worker forbidden")):
                paths = render_existing(original, root / "render", libpath)
            self.assertEqual(sha256(original), original_hash)
            result = json.loads(paths["data"].read_text(encoding="utf-8"))
            self.assertEqual(result["presentation"]["original_data_sha256"], original_hash)
            self.assertEqual(result["presentation"]["new_physics_steps"], 0)
            self.assertTrue(paths["summary"].is_file())
            self.assertEqual(result["variants"][:-1], data["variants"])
            with self.assertRaisesRegex(ValueError, "already exists"):
                render_existing(original, root / "render", libpath)

    def test_embedded_data_cannot_close_the_script_element(self):
        data = viewer_data()
        data["variants"][0]["id"] = "unsafe</script>"
        html = viewer_document(data)
        self.assertNotIn("unsafe</script>", html)
        self.assertIn("unsafe\\u003c/script>", html)

    def test_comparison_report_must_match_physical_inputs_and_keep_holdouts_closed(self):
        identity = {field: field for field in IDENTITY_FIELDS}
        fixture = SimpleNamespace(identity=identity)
        report = {
            "schema": "play01_teacher_ablation_result_v1",
            "identity": identity.copy(),
            "qualification_holdout_opened": False,
            "production_full": {},
            "production_pose_only_oracle": {},
            "direct_zero_residual": {},
        }
        validate_comparison_report(report, fixture)
        report["identity"] = {**identity, "executable_sha256": "wrong"}
        with self.assertRaisesRegex(ValueError, "executable_sha256"):
            validate_comparison_report(report, fixture)
        report["identity"] = identity.copy()
        report["qualification_holdout_opened"] = True
        with self.assertRaisesRegex(ValueError, "holdouts"):
            validate_comparison_report(report, fixture)

    def test_viewer_rejects_empty_or_unknown_data(self):
        with self.assertRaises(ValueError):
            viewer_document({"schema": "wrong", "variants": [{}]})
        with self.assertRaises(ValueError):
            viewer_document({"schema": SCHEMA, "variants": []})

    def test_serialized_zero_support_is_false(self):
        frame = validated_viewer_data(viewer_data())["variants"][0]["frames"][0]
        self.assertIs(frame["left_supported"], False)
        self.assertIs(frame["right_supported"], True)
        self.assertIs(frame["nonfoot_support"], False)

    def test_nonfinite_geometry_rejected_before_render(self):
        data = viewer_data()
        data["variants"][0]["frames"][0]["positions"][0][2] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite positions"):
            viewer_document(data)

    def test_shared_bounds_preserve_fall_height_and_world_displacement(self):
        data = viewer_data()
        fallen = copy.deepcopy(data["variants"][0])
        fallen["id"] = "fallen"
        fallen["frames"][0]["positions"] = [[0, 100, 2], [-1, 99, 0], [1, 101, 0]]
        data["variants"].append(fallen)
        follow = projection_bounds(data, "side", "follow")
        world = projection_bounds(data, "side", "world")
        self.assertLess(follow[0], -1)
        self.assertGreater(follow[3], 10)
        self.assertLess(follow[2], 0)
        self.assertGreater(world[1], 101)
        fixed = validated_viewer_data(data)["projection_bounds"]
        self.assertEqual(fixed["follow"]["side"], follow)
        self.assertEqual(fixed["world"]["side"], world)

    def test_render_existing_never_constructs_fixture_or_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "data.json"
            source.write_text(json.dumps(viewer_data()), encoding="utf-8")
            with patch("play01_viewer.Fixture", side_effect=AssertionError("fixture forbidden")), \
                    patch("play01_viewer.Runner", side_effect=AssertionError("worker forbidden")):
                result = render_existing(source, Path(directory) / "render")
            self.assertTrue(result["html"].is_file())

    def test_checkpoint_source_requires_explicit_verified_ancestry(self):
        old, current = "a" * 40, "b" * 40
        fixture = SimpleNamespace(identity={"git": {"commit": current}}, source_root=Path.cwd())
        self.assertEqual(validate_checkpoint_source(current, fixture, False)["mode"], "exact")
        with self.assertRaisesRegex(ValueError, "allow-ancestor-source"):
            validate_checkpoint_source(old, fixture, False)
        with patch("play01_viewer.subprocess.run", side_effect=[
                SimpleNamespace(returncode=0), SimpleNamespace(stdout="docs/ragdoll/evidence/result.md\n")]):
            self.assertEqual(validate_checkpoint_source(old, fixture, True)["mode"], "verified_ancestor")
        with patch("play01_viewer.subprocess.run", side_effect=[
                SimpleNamespace(returncode=0), SimpleNamespace(stdout="scripts/ragdoll-learning/play01_runner.py\n")]):
            with self.assertRaisesRegex(ValueError, "inference/physics"):
                validate_checkpoint_source(old, fixture, True)

    def test_bad_checkpoint_hash_is_rejected_before_deserializing(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "checkpoint.pt"
            checkpoint.write_bytes(b"not a checkpoint")
            args = SimpleNamespace(checkpoint=checkpoint, checkpoint_sha256="a" * 64)
            with patch("torch.load", side_effect=AssertionError("must not deserialize")):
                with self.assertRaisesRegex(ValueError, "path/hash mismatch"):
                    load_viewer_checkpoint(args, SimpleNamespace())

    def test_checkpoint_export_checks_control_and_run_case_identity(self):
        import torch
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            coverage = root / "content-coverage.json"
            coverage.write_text("{}", encoding="utf-8")
            identity = {field: field for field in IDENTITY_FIELDS}
            identity.update({"git": {"commit": "a" * 40}, "runner_version": "test-runner",
                             "specification_id": "test-spec", "floor": "tiled", "bound_speed": 60.0,
                             "command_speed": 45.0, "control_mode": "production-residual",
                             "control_contract": {"controller": "production-residual"}})
            fixture = SimpleNamespace(identity=identity, source_root=root, library_path=root / "library.json",
                                      library={"body_names": ["spine2"]}, spec={"case_generation": {}},
                                      make_generator=lambda: SimpleNamespace(identity=lambda: {"generator": "exact"}))
            selection = [{"id": "development-case"}]
            cases = {"selection": selection, "holdout_manifest_sha256": "closed-holdout-hash"}
            manifest = {
                "run_id": "viewer-test", "content_coverage_manifest_sha256": sha256(coverage),
                "target_usable_optimizer_transitions": 100, "maximum_total_physics_steps": 1000,
                "maximum_cumulative_wall_seconds": 60, "maximum_storage_bytes": 1000000,
                "stop_and_continuation_rules": {},
                "fresh_holdout_manifest_sha256": cases["holdout_manifest_sha256"],
                "curriculum_and_case_manifest_hashes": {"selection_manifest_sha256": sha256_json(selection)},
                "forward_reference_review": {}, "stage_and_command_scope": {"control_contract": identity["control_contract"]},
                "resolved_rollout_batch_and_optimizer_settings": {}, "selection_evaluation_schedule": {},
                "checkpoint_interval_and_retention": {},
                "specification_and_source_hashes": {
                    "source_commit": identity["git"]["commit"], "runner_version": identity["runner_version"],
                    "specification_id": identity["specification_id"], "spec_sha256": identity["spec_sha256"],
                    "engine_binary_sha256": identity["executable_sha256"]},
                "body_profile_and_motion_generator_hashes": {
                    key: identity[key] for key in ("context_sha256", "profile_sha256", "library_sha256",
                                                  "generator_version", "generator_source_sha256")}}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            state = {"identity": identity, "policy": {"actor.weight": torch.zeros(1)}, "optimizer": {},
                     "torch_generator": torch.zeros(1), "torch_rng": torch.zeros(1), "numpy_rngs": {},
                     "counters": {}, "normalization": {"running_normalizer": None, "fixed_scales": True},
                     "generator_identity": {"generator": "exact"},
                     "extra": {"run_contract": training_run_contract(manifest, cases)}}
            checkpoint = root / "checkpoint.pt"
            torch.save(state, checkpoint)
            args = SimpleNamespace(checkpoint=checkpoint, checkpoint_sha256=sha256(checkpoint),
                                   training_manifest=manifest_path, allow_ancestor_source=False)
            with patch("play01_viewer.resolved_case_documents", return_value=(selection, cases)), \
                    patch("play01_viewer.verify_forward_review", return_value={"reviewed": True}):
                _, binding = load_viewer_checkpoint(args, fixture)
                self.assertEqual(binding["sha256"], args.checkpoint_sha256)
                manifest["fresh_holdout_manifest_sha256"] = "wrong"
                manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "case identity"):
                    load_viewer_checkpoint(args, fixture)
                manifest["fresh_holdout_manifest_sha256"] = cases["holdout_manifest_sha256"]
                manifest["stage_and_command_scope"]["control_contract"] = {"controller": "direct"}
                manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "control contract"):
                    load_viewer_checkpoint(args, fixture)


if __name__ == "__main__":
    unittest.main()
