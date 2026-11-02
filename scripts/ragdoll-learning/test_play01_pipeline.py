#!/usr/bin/env python3
"""Focused regression tests for PLAY-01 rollout, GAE, resume and confidence contracts."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from play01_runner import (Runner, gae_returns, is_physical_early_end, policy_controls_step,
                           project_residual_action, should_end_rollout,
                           resolve_episode_steps,
                           teacher_action_from_local_targets, teacher_filter_proposal,
                           terminal_reward_with_lost_command_opportunity,
                           training_episode_is_perturbed,
                           validate_checkpoint_mode, validate_checkpoint_state, validate_repair_migration,
                           validate_startup_horizon_curriculum, validate_teacher_ablation_config,
                           validate_teacher_bootstrap_config)
from reference_generator import ReferenceGenerator
from play01_worker_client import worker_creation_flags


def toy_library() -> dict:
    names = ["pelvis", "left_hand", "right_hand", "left_foot", "right_foot"]
    identity = np.zeros((2, len(names), 4), dtype=float)
    identity[..., 3] = 1.0
    roots = np.zeros((2, 3), dtype=float)

    def clip(confidence):
        return {
            "parent_local_rotations_xyzw": identity.tolist(), "root_positions": roots.tolist(),
            "duration_seconds": 1.0, "style_confidence": confidence,
        }

    return {
        "sha256": "toy-library", "body_names": names, "parent_indices": [-1, 0, 0, 0, 0],
        "creation_local_offsets": [[0, 0, 0], [-1, 0, 1], [1, 0, 1], [-1, 0, -1], [1, 0, -1]],
        "forward_axis_mapping": {"model_forward": [0, 1, 0]},
        "clips": {
            "idle": clip({name: 0.0 for name in names}),
            "forward": clip({name: 0.0 for name in names}),
            "backward": clip({name: 1.0 for name in names}),
        },
    }


class TerminalAndGaeTests(unittest.TestCase):
    def test_native_worker_isolated_from_windows_console_interrupt_group(self):
        import os
        import subprocess
        if os.name == "nt":
            flags = worker_creation_flags()
            self.assertTrue(flags & subprocess.CREATE_NEW_PROCESS_GROUP)
            self.assertTrue(flags & subprocess.CREATE_NO_WINDOW)
        else:
            self.assertEqual(worker_creation_flags(), 0)

    def test_production_pose_inverts_into_existing_residual_and_filter_basis(self):
        reference = np.zeros((3, 4), dtype=float)
        reference[:, 3] = 1.0
        target = reference.copy()
        target[1] = [0.0, 0.0, np.sin(0.25 / 2), np.cos(0.25 / 2)]
        target[2] = [np.sin(0.10 / 2), 0.0, 0.0, np.cos(0.10 / 2)]
        layout = [
            {"body": 1, "first_scalar": 0, "dimensions": 3},
            {"body": 2, "first_scalar": 3, "dimensions": 1,
             "hinge_axis_parent_local": [1.0, 0.0, 0.0]},
        ]
        desired, diagnostics = teacher_action_from_local_targets(target, reference, layout)
        np.testing.assert_allclose(desired, [0.0, 0.0, 0.5, 0.2], atol=1e-6)
        self.assertEqual(diagnostics["clipped_joint_count"], 0)
        proposal, delivered, filter_diagnostics = teacher_filter_proposal(
            desired * 0.2, np.zeros(4), layout)
        np.testing.assert_allclose(proposal, desired, atol=1e-6)
        np.testing.assert_allclose(delivered, desired * 0.2, atol=1e-6)
        self.assertLess(filter_diagnostics["maximum_delivery_error"], 1e-6)

    def test_residual_projection_keeps_every_joint_inside_float_boundary(self):
        layout = [{"first_scalar": 0, "dimensions": 3}, {"first_scalar": 3, "dimensions": 1}]
        projected = project_residual_action(np.array([1.0, 1.0, 1.0, -1.0], dtype=np.float32), layout)
        self.assertLess(float(np.linalg.norm(projected[:3])), 1.0)
        self.assertLess(abs(float(projected[3])), 1.0)
        self.assertTrue(np.all(np.isfinite(projected)))

    def test_only_training_ends_at_first_terminal(self):
        self.assertTrue(should_end_rollout("training", True, True, False))
        self.assertFalse(should_end_rollout("evaluation", True, True, False))
        self.assertFalse(should_end_rollout("diagnostic", True, True, False))
        self.assertFalse(should_end_rollout("training", True, True, True))

    def test_observationless_end_requires_physical_terminal_evidence(self):
        self.assertTrue(is_physical_early_end({"kind": "end", "fall_step": 12, "nonfoot_support_frames": 0}))
        self.assertTrue(is_physical_early_end({"kind": "end", "fall_step": None, "nonfoot_support_frames": 1}))
        self.assertFalse(is_physical_early_end({"kind": "end", "fall_step": None, "nonfoot_support_frames": 0}))
        self.assertFalse(is_physical_early_end({"kind": "error", "fall_step": 12}))

    def test_terminal_transition_is_kept_and_does_not_bootstrap(self):
        rewards = np.array([1.0, 2.0, 100.0], dtype=np.float32)
        values = np.array([0.5, 0.25, 90.0], dtype=np.float32)
        continuation = np.array([1.0, 0.0, 1.0], dtype=np.float32)
        advantages, targets = gae_returns(rewards, values, continuation, gamma=0.9, lam=1.0)
        # Transition one is terminal: its target is its reward, and later data
        # cannot leak backward across the boundary.
        self.assertAlmostEqual(float(targets[1]), 2.0, places=6)
        self.assertAlmostEqual(float(targets[0]), 1.0 + 0.9 * 2.0, places=6)
        self.assertEqual(len(advantages), 3)

    def test_terminal_opportunity_is_bounded_to_future_command_reward(self):
        # Two future commanded frames at gamma=0.5 lose 0.55 * (1 + 0.5),
        # rather than synthesizing repeated terminal penalties.
        self.assertAlmostEqual(terminal_reward_with_lost_command_opportunity(1, 4, 0.5), -3.825)
        self.assertEqual(terminal_reward_with_lost_command_opportunity(3, 4, 0.5), -3.0)
        with self.assertRaises(ValueError):
            terminal_reward_with_lost_command_opportunity(1, 4, 0.0)

    def test_policy_cannot_act_during_excluded_warmup(self):
        self.assertFalse(policy_controls_step(0, 60))
        self.assertFalse(policy_controls_step(59, 60))
        self.assertTrue(policy_controls_step(60, 60))

    def test_startup_training_disables_perturbations(self):
        self.assertFalse(training_episode_is_perturbed(0, False))
        self.assertFalse(training_episode_is_perturbed(1, False))
        self.assertFalse(training_episode_is_perturbed(2, False))
        self.assertFalse(training_episode_is_perturbed(0, True))
        self.assertTrue(training_episode_is_perturbed(1, True))

    def test_startup_horizons_are_monotone_bounded_and_finish_full_length(self):
        document = {"probe_episodes": 3, "probe_interval_usable_transitions": 1000, "stages": [
            {"episode_steps": 120, "minimum_stage_usable_transitions": 1000,
             "maximum_stage_usable_transitions": 2000},
            {"episode_steps": 600, "minimum_stage_usable_transitions": 1000,
             "maximum_stage_usable_transitions": 2000, "require_positive_signed_speed": True},
        ]}
        self.assertEqual(len(validate_startup_horizon_curriculum(document, 600)), 2)
        document["stages"][1]["episode_steps"] = 120
        with self.assertRaisesRegex(ValueError, "increase uniquely"):
            validate_startup_horizon_curriculum(document, 600)

    def test_startup_horizon_curriculum_is_optional_but_full_gate_requires_forward_motion(self):
        self.assertEqual(validate_startup_horizon_curriculum(None, 600), [])
        document = {"probe_episodes": 1, "probe_interval_usable_transitions": 1000, "stages": [
            {"episode_steps": 600, "minimum_stage_usable_transitions": 1000,
             "maximum_stage_usable_transitions": 2000, "require_positive_signed_speed": False},
        ]}
        with self.assertRaisesRegex(ValueError, "full-horizon.*positive signed speed"):
            validate_startup_horizon_curriculum(document, 600)
        document["stages"][0]["require_positive_signed_speed"] = "yes"
        with self.assertRaisesRegex(ValueError, "must be boolean"):
            validate_startup_horizon_curriculum(document, 600)

    def test_teacher_bootstrap_contract_is_frozen_and_rejects_duplicate_cases(self):
        document = {
            "source_controller": "production_walker",
            "teacher_case_ids": ["selection-0", "unperturbed-control"],
            "teacher_case_manifest_sha256": "bound-by-manifest",
            "epochs": 40, "minibatch_size": 512, "learning_rate": 0.0003,
            "minimum_teacher_survival_fraction": 0.8,
            "minimum_teacher_pose_coverage_fraction": 0.8,
            "require_positive_teacher_signed_speed": True,
            "student_success_gate": "positive signed speed and fewer full-length falls than fresh zero residual",
        }
        self.assertIs(validate_teacher_bootstrap_config(document), document)
        document["teacher_case_ids"] = ["selection-0", "selection-0"]
        with self.assertRaisesRegex(ValueError, "non-empty and unique"):
            validate_teacher_bootstrap_config(document)
        document["teacher_case_ids"] = ["selection-0"]
        document["minimum_teacher_pose_coverage_fraction"] = 1.1
        with self.assertRaisesRegex(ValueError, "bounded pose coverage"):
            validate_teacher_bootstrap_config(document)

    def test_teacher_pose_ablation_variants_and_gate_are_frozen(self):
        document = {
            "source_controller": "production_walker",
            "case_ids": ["selection-0", "unperturbed-control"],
            "variants": ["production_full", "production_pose_only_oracle", "direct_zero_residual"],
            "pose_only_success_gate": "positive signed speed and fewer full-length falls than direct zero residual",
        }
        self.assertIs(validate_teacher_ablation_config(document), document)
        document["variants"] = ["production_full", "direct_zero_residual"]
        with self.assertRaisesRegex(ValueError, "variants are not frozen"):
            validate_teacher_ablation_config(document)

    def test_case_bound_horizon_controls_the_episode_loop_and_accounting_contract(self):
        self.assertEqual(resolve_episode_steps({"episode_steps": 120}, None, 600), 120)
        self.assertEqual(resolve_episode_steps({"episode_steps": 120}, 144, 600), 144)
        self.assertEqual(resolve_episode_steps({}, None, 600), 600)
        with self.assertRaisesRegex(ValueError, "outside"):
            resolve_episode_steps({"episode_steps": 601}, None, 600)


class CheckpointModeTests(unittest.TestCase):
    fresh_treatment = {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"}
    resume_treatment = {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"}

    def test_modes_are_explicit_and_cannot_fall_back(self):
        fresh = {"initial_mode": "fresh", "checkpoint": None, "treatment": self.fresh_treatment,
                 "exact_resume_treatment": self.resume_treatment}
        validate_checkpoint_mode("fresh", fresh, False)
        validate_checkpoint_mode("exact-resume", fresh, True)
        with self.assertRaises(ValueError):
            validate_checkpoint_mode("transfer", fresh, False)
        with self.assertRaises(ValueError):
            validate_checkpoint_mode("exact-resume", fresh, False)
        with self.assertRaises(ValueError):
            validate_checkpoint_mode("fresh", fresh, True)

    def test_transfer_requires_declared_hashed_parent(self):
        transfer = {"initial_mode": "transfer", "checkpoint": {"path": "parent.pt", "sha256": "abc"},
                    "treatment": self.fresh_treatment, "exact_resume_treatment": self.resume_treatment}
        validate_checkpoint_mode("transfer", transfer, False)
        transfer["checkpoint"] = None
        with self.assertRaises(ValueError):
            validate_checkpoint_mode("transfer", transfer, False)

    def test_checkpoint_identity_is_checked_before_state_is_applied(self):
        identity = {key: key for key in ("executable_sha256", "context_sha256", "profile_sha256", "library_sha256",
                                         "generator_source_sha256", "spec_sha256", "generator_version", "runner_version")}
        identity["git"] = {"commit": "current"}
        generator_identity = {"generator": "exact"}
        fixture = SimpleNamespace(identity=identity, make_generator=lambda: SimpleNamespace(identity=lambda: generator_identity))
        counters = {key: 0 for key in Runner.counter_snapshot_fields()}
        counters["cumulative_wall_seconds"] = 0.0
        state = {"identity": dict(identity), "policy": {}, "optimizer": {}, "torch_generator": [], "torch_rng": [],
                 "numpy_rngs": {}, "counters": counters, "normalization": {"running_normalizer": None, "fixed_scales": True},
                 "generator_identity": generator_identity, "extra": {"run_contract": {"case": "same"}}}
        validate_checkpoint_state(state, fixture, "exact-resume", {"case": "same"})
        state["identity"] = {**identity, "library_sha256": "wrong"}
        with self.assertRaisesRegex(RuntimeError, "library_sha256"):
            validate_checkpoint_state(state, fixture, "transfer")
        state["identity"] = identity
        with self.assertRaisesRegex(RuntimeError, "run/case"):
            validate_checkpoint_state(state, fixture, "exact-resume", {"case": "different"})

    def test_authorized_repair_migration_preserves_exact_state_contract(self):
        current = {key: key for key in ("executable_sha256", "context_sha256", "profile_sha256", "library_sha256",
                                        "generator_source_sha256", "spec_sha256", "generator_version")}
        current.update({"runner_version": "v5", "git": {"commit": "new"}})
        old = {**current, "runner_version": "v4", "git": {"commit": "old"}}
        generator_identity = {"generator": "exact"}
        fixture = SimpleNamespace(identity=current, make_generator=lambda: SimpleNamespace(identity=lambda: generator_identity))
        counters = {key: 0 for key in Runner.counter_snapshot_fields()}
        counters["cumulative_wall_seconds"] = 5.0
        state = {"identity": old, "policy": {}, "optimizer": {}, "torch_generator": [], "torch_rng": [],
                 "numpy_rngs": {}, "counters": counters, "normalization": {"running_normalizer": None, "fixed_scales": True},
                 "generator_identity": generator_identity, "extra": {"run_contract": {"same": True}}}
        migration = {"from_source_commit": "old", "from_runner_version": "v4", "from_checkpoint_sha256": "checkpoint",
                     "from_manifest_sha256": "manifest", "to_source_commit": "new", "to_runner_version": "v5",
                     "treatment": "restore_policy_optimizer_rng_normalization_counters_and_schedule",
                     "reason": "early-END repair", "authorization_reference": "user authorized continuation"}
        validate_repair_migration(migration, state, fixture, "checkpoint", "manifest")
        validate_checkpoint_state(state, fixture, "exact-resume", {"same": True}, allow_repair_transition=True)
        migration["from_checkpoint_sha256"] = "wrong"
        with self.assertRaisesRegex(RuntimeError, "from_checkpoint_sha256"):
            validate_repair_migration(migration, state, fixture, "checkpoint", "manifest")


class StyleConfidenceTests(unittest.TestCase):
    def test_inactive_clip_cannot_restore_excluded_channel(self):
        generator = ReferenceGenerator(toy_library(), bound_speed=60.0, start_blend_steps=1)
        result = generator.step(np.array([0.0, 60.0]), np.array([0.0, 1.0]), 60.0)
        self.assertEqual(result["family_weights"]["forward"], 1.0)
        self.assertEqual(result["style_confidence"]["left_foot"], 0.0)

    def test_active_family_blend_propagates_confidence(self):
        generator = ReferenceGenerator(toy_library(), bound_speed=60.0)
        generator.family_weights = {"forward": 0.25, "backward": 0.75, "strafe_left": 0.0, "strafe_right": 0.0}
        confidence = generator._active_confidence(1.0)
        self.assertAlmostEqual(confidence["left_foot"], 0.75)


if __name__ == "__main__":
    unittest.main()
