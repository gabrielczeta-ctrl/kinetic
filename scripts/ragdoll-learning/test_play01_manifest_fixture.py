"""Reject changed simulation fixtures when loading an allocated manifest."""

import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import play01_runner as runner


class ManifestFixtureTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="play01-manifest-fixture-")
        self.addCleanup(temporary.cleanup)
        self.out = Path(temporary.name)
        spec_path = self.out / "scripts/ragdoll-learning/pilot_spec.json"
        spec_path.parent.mkdir(parents=True)
        spec_path.write_text(json.dumps({"training_run_manifest": {"required_fields": []}}), encoding="utf-8")
        coverage = self.out / "content-coverage.json"
        coverage.write_text("{}", encoding="utf-8")
        identity = {key: key for key in ("implementation_sha256", "executable_sha256", "context_sha256",
                    "profile_sha256", "library_sha256", "generator_source_sha256", "spec_sha256", "specification_id")}
        identity.update({"git": {"commit": "a" * 40, "dirty_files": []},
                         "control_contract": {"controller": "production-residual"},
                         "floor": "tiled", "command_speed": 45.0, "bound_speed": 60.0})
        self.fixture = SimpleNamespace(identity=identity, control_mode="production-residual", episode_steps=600,
                                       library_path=self.out / "library.json", spec={"optimizer": {}})
        evidence_path = self.out / "integration.json"
        evidence_path.write_text(json.dumps({"identity": copy.deepcopy(identity), "ready_for_allocation": True,
                                             "qualification_holdout_opened": False}), encoding="utf-8")
        self.manifest = {
            "compute_allocation_and_authorization_reference": "bounded unit-test fixture only",
            "worker_count": 3,
            "stage_and_command_scope": {"control_contract": identity["control_contract"],
                "command_speed_schedule_units_per_second": [{"through_usable_optimizer_transitions": 100, "speed": 45.0}]},
            "target_usable_optimizer_transitions": 100, "maximum_total_physics_steps": 20000,
            "maximum_cumulative_wall_seconds": 300, "maximum_storage_bytes": 1000000,
            "checkpoint_interval_and_retention": {"interval_updates": 1, "retain_milestone_every_updates": 1,
                                                   "evaluation_milestones_usable_transitions": [100]},
            "selection_evaluation_schedule": {"interval_updates": 1},
            "specification_and_source_hashes": {
                "specification_id": identity["specification_id"], "spec_sha256": identity["spec_sha256"],
                "source_commit": identity["git"]["commit"], "engine_binary_sha256": identity["executable_sha256"],
                "runner_version": runner.RUNNER_VERSION},
            "body_profile_and_motion_generator_hashes": {
                **{key: identity[key] for key in ("context_sha256", "profile_sha256", "library_sha256", "generator_source_sha256")},
                "generator_version": runner.GENERATOR_VERSION},
            "content_coverage_manifest_sha256": runner.sha256(coverage),
            "resolved_rollout_batch_and_optimizer_settings": {"optimizer": {}, "episode_physics_steps": 600,
                                                               "training_perturbations_enabled": False, "episodes_per_batch": 6},
            "forward_reference_review": {},
            "integration_evidence": {"path": str(evidence_path), "sha256": runner.sha256(evidence_path)},
        }
        manifest_path = self.out / "manifest.json"
        manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")
        self.args = SimpleNamespace(manifest=str(manifest_path), source_root=str(self.out))

    def load(self):
        with patch.object(runner, "Fixture", return_value=self.fixture):
            with patch.object(runner, "verify_forward_review", return_value={"reviewed": True}):
                with patch.object(runner, "Worker", side_effect=AssertionError("no worker may start while checking identities")) as worker:
                    result = runner.load_training_manifest(self.args, require_authorization=True)
                    worker.assert_not_called()
                    return result

    def test_matching_fixture_loads_without_starting_physics(self):
        loaded = self.load()
        self.assertEqual(loaded[1], self.manifest)

    def test_floor_command_and_bound_mismatches_are_rejected_before_physics(self):
        for field, changed in (("floor", "box"), ("command_speed", 60.0), ("bound_speed", 61.0)):
            original = self.fixture.identity[field]
            with self.subTest(field=field):
                self.fixture.identity[field] = changed
                with self.assertRaisesRegex(ValueError, f"integration identity mismatch: {field}"):
                    self.load()
            self.fixture.identity[field] = original


if __name__ == "__main__":
    unittest.main()
