#!/usr/bin/env python3
"""Allocation arithmetic and provenance regressions; no physics or training."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from play01_overnight import (EVIDENCE_SCHEMA, INTEGRATION_IDENTITIES,
                              allocation_budget, build_proposal, evaluation_milestones,
                              prepare, validate_integration)


def measured_throughput():
    return {"complete_update_usable_transitions_per_second": 1000.0,
            "rollout_physics_steps_per_second": 1500.0,
            "workers": 3, "episodes_per_batch": 6, "batches": 2,
            "measured_counters": {"training_simulated_physics_steps": 6000,
                                  "training_usable_optimizer_transitions": 5400,
                                  "training_warmup_physics_steps": 600,
                                  "total_simulated_physics_steps": 10000}}


def fixture_and_evidence(directory: Path):
    spec = json.loads((Path(__file__).parent / "pilot_spec.json").read_text(encoding="utf-8"))
    identity = {key: key for key in INTEGRATION_IDENTITIES}
    identity.update({"git": {"commit": "a" * 40, "dirty_files": []}, "runner_version": "test",
                     "specification_id": "test-spec", "control_mode": "production-residual",
                     "control_contract": {"controller": "production-residual"},
                     "command_speed": 45.0, "bound_speed": 60.0, "floor": "tiled"})
    fixture = SimpleNamespace(identity=identity, spec=spec, workers=3, control_mode="production-residual",
                              command_speed=45.0, episode_steps=600, exe=directory / "worker.exe",
                              source_root=directory, library_path=directory / "library.json",
                              library={"body_names": ["spine2"]})
    evidence = {"schema": EVIDENCE_SCHEMA, "identity": copy.deepcopy(identity),
                "ready_for_allocation": True, "qualification_holdout_opened": False,
                "throughput": measured_throughput()}
    return fixture, evidence


class OvernightAllocationTests(unittest.TestCase):
    def test_budget_uses_new_usable_rate_and_separate_physics_cap(self):
        budget = allocation_budget(measured_throughput(), 8, 600, 16)
        self.assertEqual(budget["target_usable_optimizer_transitions"], 21_500_000)
        self.assertEqual(budget["maximum_cumulative_wall_seconds"], 28_800)
        self.assertEqual(budget["maximum_storage_bytes"], 2_000_000_000)
        self.assertEqual(budget["budget_training_physics_steps_per_usable_transition"], 1.112)
        self.assertEqual(budget["maximum_total_physics_steps"],
                         budget["training_physics_cap_with_25_percent_margin"] + budget["reserved_evaluation_physics_steps"])
        self.assertGreater(budget["maximum_total_physics_steps"], budget["target_usable_optimizer_transitions"])

    def test_terminal_overhead_changes_physics_cap_not_sample_target(self):
        fast = measured_throughput()
        overhead = copy.deepcopy(fast)
        overhead["measured_counters"]["training_simulated_physics_steps"] *= 3
        first, second = allocation_budget(fast, 8, 600, 16), allocation_budget(overhead, 8, 600, 16)
        self.assertEqual(first["target_usable_optimizer_transitions"], second["target_usable_optimizer_transitions"])
        self.assertGreater(second["maximum_total_physics_steps"], first["maximum_total_physics_steps"])

    def test_milestones_are_frequent_initially_and_always_include_final(self):
        self.assertEqual(evaluation_milestones(1_750_000),
                         [250_000, 500_000, 750_000, 1_000_000, 1_500_000, 1_750_000])
        self.assertEqual(evaluation_milestones(250_000), [250_000])

    def test_bad_or_inadequate_measurement_rejected(self):
        for rate in (0, -1, float("nan"), float("inf"), 1):
            throughput = measured_throughput()
            throughput["complete_update_usable_transitions_per_second"] = rate
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                allocation_budget(throughput, 8, 600, 16)
        with self.assertRaises(ValueError):
            allocation_budget(measured_throughput(), 9, 600, 16)

    def test_evidence_rejects_changed_controller_or_opened_holdouts(self):
        fixture, evidence = fixture_and_evidence(Path.cwd())
        validate_integration(evidence, fixture)
        evidence["identity"]["control_contract"] = {"controller": "direct"}
        with self.assertRaisesRegex(ValueError, "control_contract"):
            validate_integration(evidence, fixture)
        evidence["identity"] = copy.deepcopy(fixture.identity)
        evidence["qualification_holdout_opened"] = True
        with self.assertRaisesRegex(ValueError, "holdouts"):
            validate_integration(evidence, fixture)

    def test_proposal_removes_old_experiment_and_authorization(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, evidence = fixture_and_evidence(Path(directory))
            proposal = build_proposal(fixture, evidence, {"path": "integration", "sha256": "e" * 64},
                                      {"path": "review", "sha256": "r" * 64}, 8, Path(directory))
        manifest = proposal["proposed_training_run_manifest"]
        self.assertIsNone(manifest["compute_allocation_and_authorization_reference"])
        self.assertEqual(proposal["launch_status"], "pending_explicit_compute_authorization")
        self.assertEqual(manifest["parent_checkpoint_and_transfer_policy"]["initial_mode"], "fresh")
        self.assertIsNone(manifest["parent_checkpoint_and_transfer_policy"]["checkpoint"])
        self.assertNotIn("startup_horizon_curriculum", manifest["stage_and_command_scope"])
        self.assertEqual(manifest["stage_and_command_scope"]["control_contract"], fixture.identity["control_contract"])
        self.assertEqual(manifest["stage_and_command_scope"]["command_speed_schedule_units_per_second"],
                         [{"through_usable_optimizer_transitions": 21_500_000, "speed": 45.0}])
        self.assertEqual(manifest["stop_and_continuation_rules"]["evaluation_guard"]["plateau_minimum_usable_transitions"], 4_000_000)
        self.assertNotIn("candidates", proposal)
        self.assertFalse(set(fixture.spec["training_run_manifest"]["required_fields"]) - manifest.keys())

    def test_dirty_tree_rejects_preparation_before_reading_evidence(self):
        fixture, _ = fixture_and_evidence(Path.cwd())
        fixture.identity["git"]["dirty_files"] = [" M play01_runner.py"]
        with patch("play01_overnight.Fixture", return_value=fixture):
            with self.assertRaisesRegex(ValueError, "clean source"):
                prepare(SimpleNamespace())

    def test_completed_integration_totals_include_final_controls(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, evidence = fixture_and_evidence(Path(directory))
            evidence["total_physics_steps"] = 30_000
            evidence["wall_seconds"] = 900.0
            evidence["throughput"]["overall_steps_per_second_including_controls"] = 100.0
            proposal = build_proposal(fixture, evidence, {"path": "integration", "sha256": "e" * 64},
                                      {"path": "review", "sha256": "r" * 64}, 8, Path(directory))
        overhead = proposal["proposed_training_run_manifest"]["measurement_protocol_and_overhead"]
        self.assertEqual(overhead["integration_physics_steps"], 30_000)
        self.assertEqual(overhead["integration_elapsed_seconds"], 900.0)
        self.assertEqual(overhead["measured_counters"]["total_simulated_physics_steps"], 10_000)
        self.assertEqual(proposal["recommended"]["measured_training_physics_steps_per_usable_transition"], 6000 / 5400)


if __name__ == "__main__":
    unittest.main()
