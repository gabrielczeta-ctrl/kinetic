#!/usr/bin/env python3
"""Prepare an explicitly pending PLAY-01 production-residual overnight allocation.

Preparation verifies existing integration evidence and writes a proposal only.
It never starts workers, imports a parent policy, or authorizes training.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

from play01_runner import (Fixture, TORSO_BODY_NAME, propose_allocation,
                           resolved_case_documents, sha256, sha256_json, utc_now,
                           verify_forward_review)


EVIDENCE_SCHEMA = "play01_production_residual_integration_v1"
PROPOSAL_SCHEMA = "play01_production_residual_overnight_proposal_v1"
INTEGRATION_IDENTITIES = (
    "implementation_sha256", "executable_sha256", "context_sha256", "profile_sha256",
    "library_sha256", "generator_source_sha256", "spec_sha256", "generator_version",
    "control_mode", "control_contract", "floor", "bound_speed", "command_speed",
)
SAMPLE_QUANTUM = 250_000


def validate_integration(evidence: dict[str, Any], fixture: Fixture) -> None:
    if evidence.get("schema") != EVIDENCE_SCHEMA:
        raise ValueError("unknown integration evidence schema")
    if evidence.get("ready_for_allocation") is not True or evidence.get("qualification_holdout_opened") is not False:
        raise ValueError("integration evidence must be ready with qualification holdouts unopened")
    identity = evidence.get("identity", {})
    mismatches = [key for key in INTEGRATION_IDENTITIES if identity.get(key) != fixture.identity.get(key)]
    if mismatches:
        raise ValueError("integration identity mismatch: " + ", ".join(mismatches))
    if fixture.control_mode != "production-residual" or fixture.command_speed != 45.0 or fixture.workers != 3:
        raise ValueError("overnight preparation requires production-residual, 45 units/s and three workers")
    measured = evidence.get("throughput", {})
    rate = measured.get("complete_update_usable_transitions_per_second")
    if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
        raise ValueError("integration requires positive finite measured complete-update usable throughput")
    if measured.get("workers") != 3 or measured.get("episodes_per_batch") != 6 or measured.get("batches", 0) < 1:
        raise ValueError("integration throughput must measure three workers and six episodes per completed batch")
    counters = measured.get("measured_counters", {})
    usable = counters.get("training_usable_optimizer_transitions", 0)
    physics = counters.get("training_simulated_physics_steps", 0)
    if not isinstance(usable, int) or not isinstance(physics, int) or usable <= 0 or physics < usable:
        raise ValueError("integration training physics/usable counters are invalid")


def evaluation_milestones(target: int) -> list[int]:
    milestones = list(range(SAMPLE_QUANTUM, min(target, 1_000_000) + 1, SAMPLE_QUANTUM))
    milestones.extend(range(1_500_000, target + 1, 500_000))
    return sorted(set(milestones + [target]))


def allocation_budget(throughput: dict[str, Any], hours: float, episode_steps: int,
                      selection_cases: int) -> dict[str, Any]:
    """Size samples from useful work, with a separate conservative physics cap."""
    if not math.isfinite(hours) or not 0 < hours <= 8:
        raise ValueError("overnight wall budget must be positive and at most eight hours")
    measured_rate = float(throughput["complete_update_usable_transitions_per_second"])
    if not math.isfinite(measured_rate) or measured_rate <= 0:
        raise ValueError("measured usable throughput must be positive and finite")
    wall = int(hours * 3600)
    projected_rate = 0.75 * measured_rate
    target = math.floor(projected_rate * wall / SAMPLE_QUANTUM) * SAMPLE_QUANTUM
    if target < SAMPLE_QUANTUM:
        raise ValueError("measured throughput cannot support one 250k-sample allocation milestone")
    counters = throughput["measured_counters"]
    usable, physics = counters["training_usable_optimizer_transitions"], counters["training_simulated_physics_steps"]
    if usable <= 0 or physics < usable:
        raise ValueError("invalid measured training physics/usable counters")
    measured_ratio = physics / usable
    budget_ratio = max(measured_ratio, 1.112)
    milestones = evaluation_milestones(target)
    # Reserve one initial baseline, each sample milestone, and a final evaluation.
    evaluation_batches = len(milestones) + 2
    evaluation_steps = evaluation_batches * (selection_cases + 1) * episode_steps
    training_cap = math.ceil(target * budget_ratio * 1.25)
    physics_rate = float(throughput.get("rollout_physics_steps_per_second") or measured_rate * measured_ratio)
    if not math.isfinite(physics_rate) or physics_rate <= 0:
        raise ValueError("measured physics throughput must be positive and finite")
    return {
        "target_usable_optimizer_transitions": target,
        "maximum_total_physics_steps": training_cap + evaluation_steps,
        "maximum_cumulative_wall_seconds": wall,
        "maximum_storage_bytes": 2_000_000_000,
        "projected_usable_transitions_per_second_after_overhead": projected_rate,
        "projected_training_seconds": target / measured_rate,
        "projected_training_seconds_after_overhead": target / projected_rate,
        "measured_training_physics_steps_per_usable_transition": measured_ratio,
        "budget_training_physics_steps_per_usable_transition": budget_ratio,
        "training_physics_cap_with_25_percent_margin": training_cap,
        "evaluation_milestones_usable_transitions": milestones,
        "reserved_evaluation_physics_steps": evaluation_steps,
        "reserved_evaluation_batches": evaluation_batches,
        "reserved_evaluation_wall_seconds": math.ceil(evaluation_steps / physics_rate),
    }


def build_proposal(fixture: Fixture, evidence: dict[str, Any], binding: dict[str, str],
                   review: dict[str, str], hours: float, evidence_dir: Path) -> dict[str, Any]:
    validate_integration(evidence, fixture)
    throughput = evidence["throughput"]
    selection, cases_doc = resolved_case_documents(
        fixture.spec["case_generation"], fixture.library["body_names"].index(TORSO_BODY_NAME))
    budget = allocation_budget(throughput, hours, fixture.episode_steps, len(selection))
    measured_counters = throughput["measured_counters"]
    # The throughput snapshot precedes final evaluation and resume controls.
    # Use the evidence's completed integration totals for whole-run overhead.
    used_steps = int(evidence.get("total_physics_steps", measured_counters.get("total_simulated_physics_steps", 0)))
    elapsed = evidence.get("wall_seconds", evidence.get("elapsed_seconds"))
    if elapsed is None:
        overall_rate = throughput.get("overall_steps_per_second_including_controls")
        elapsed = used_steps / float(overall_rate) if overall_rate else None
    # Populate the required schema using the existing producer, then replace
    # every experiment-specific or authorization-bearing field below.
    template = propose_allocation(fixture, throughput, used_steps, elapsed or 0.0, evidence_dir)
    manifest = copy.deepcopy(template["proposed_training_run_manifest"])
    target = budget["target_usable_optimizer_transitions"]
    manifest.update({
        "run_id": f"PLAY-01-production-residual-{hours:g}h-{target}-proposed",
        "stage_and_command_scope": {
            "stage": "production_residual_forward_learning", "commands": ["idle", "start", "forward", "stop"],
            "control_contract": fixture.identity["control_contract"],
            "command_speed_schedule_units_per_second": [{"through_usable_optimizer_transitions": target, "speed": 45.0}],
            "qualification_claim": "none; development selection evaluations only",
        },
        "integration_evidence": binding,
        "forward_reference_review": review,
        "compute_allocation_and_authorization_reference": None,
        "parent_checkpoint_and_transfer_policy": {
            "initial_mode": "fresh", "checkpoint": None,
            "treatment": {"optimizer": "initialize", "rng": "initialize", "normalization": "fixed_identity", "counters": "zero"},
            "exact_resume_treatment": {"optimizer": "restore", "rng": "restore", "normalization": "restore", "counters": "restore"},
            "contract": "fresh zero-initialized residual around production; exact-resume restores the same run's policy, optimizer, RNG, normalization, cumulative limits and evaluation guard",
        },
        "worker_count": 3,
        "resolved_rollout_batch_and_optimizer_settings": {
            "episodes_per_batch": 6, "episode_physics_steps": fixture.episode_steps,
            "training_perturbations_enabled": False, "optimizer": fixture.spec["optimizer"]},
        "checkpoint_interval_and_retention": {
            "interval_updates": 25, "durable_latest_every_completed_update": True,
            "retain_latest": True, "retain_best": True, "retain_initial": True,
            "retain_milestone_every_updates": 1000,
            "evaluation_milestones_usable_transitions": budget["evaluation_milestones_usable_transitions"]},
        "selection_evaluation_schedule": {
            "interval_updates": 1_000_000, "cases": len(selection), "repeated_unperturbed_control": True,
            "command_speed_units_per_second": 45.0,
            "baseline": "own initial production-plus-zero-correction policy on the same selection/control cases"},
        "reserved_evaluation_steps_wall_and_storage": {
            "physics_steps": budget["reserved_evaluation_physics_steps"],
            "wall_seconds": budget["reserved_evaluation_wall_seconds"], "storage_bytes": 200_000_000,
            "work_unit_wall_reserve_seconds": 120,
            "scope": "one initial baseline, all sample milestones, and one final full-length selection/control evaluation"},
        "curriculum_and_case_manifest_hashes": {
            "selection_manifest_sha256": sha256_json(selection),
            "training_case_rng": "PCG64 state in checkpoint; per-episode policy seeds; unperturbed forward training",
            "command_speed_schedule": f"{target} usable optimizer transitions at fixed 45 units/s; full-length 600-step ceiling; first terminal ends training only"},
        "fresh_holdout_manifest_sha256": cases_doc["holdout_manifest_sha256"],
        "stop_and_continuation_rules": {
            "stop": ["integrity/control failure", "non-finite state", "worker failure or timeout",
                     "sample target reached", "physics/wall/storage cap", "evaluation reserve cannot fit",
                     "frozen regression or plateau guard"],
            "evaluation_guard": {
                "minimum_usable_transitions": 500_000, "consecutive_evaluations": 3,
                "additional_falls_limit": 3, "plateau_minimum_usable_transitions": 4_000_000,
                "minimum_relative_rmse_improvement": 0.02},
            "continuation": "checkpoint and report at the first stop condition; no automatic budget extension",
            "holdout": "remain unopened; broader turns/directions and native gameplay qualification are separate",
        },
        "measurement_protocol_and_overhead": {
            "integration_evidence": binding, "integration_physics_steps": used_steps,
            "integration_elapsed_seconds": elapsed, "episodes_per_batch": 6,
            "evaluation_and_checkpoint_reserve_fraction": 0.25,
            "counter_definitions": throughput.get("counter_definitions", {}),
            "measured_counters": measured_counters,
            "first_terminal_timing": throughput.get("first_terminal_timing"),
            "training_warmup_and_reset": "included in measured complete-update wall time, excluded from usable samples; counters remain separate",
            "measurement_source_commit": evidence["identity"]["git"]["commit"],
        },
        "measured_steps_and_updates_per_second": {
            "measurement_source_commit": evidence["identity"]["git"]["commit"],
            "measurement_runner_version": evidence["identity"]["runner_version"], "workers": 3,
            "complete_update_usable_transitions_per_second": throughput["complete_update_usable_transitions_per_second"],
            "training_physics_steps_per_usable_transition": budget["measured_training_physics_steps_per_usable_transition"],
            "projected_allocated_usable_transitions_per_second_after_overhead": budget["projected_usable_transitions_per_second_after_overhead"]},
    })
    for key in ("target_usable_optimizer_transitions", "maximum_total_physics_steps", "maximum_cumulative_wall_seconds", "maximum_storage_bytes"):
        manifest[key] = budget[key]
    manifest["specification_and_source_hashes"]["implementation_sha256"] = fixture.identity["implementation_sha256"]
    manifest["budget_rationale"] = (
        f"New production-residual allocation: {target:,} usable optimizer transitions, calculated from 75% of the newly measured complete-update rate over {hours:g} hours and rounded down to 250k samples. "
        f"The independent physics cap is {budget['maximum_total_physics_steps']:,}: {budget['training_physics_cap_with_25_percent_margin']:,} training physics steps including a 25% ratio margin plus {budget['reserved_evaluation_physics_steps']:,} reserved evaluation steps. "
        "The 8-hour-or-less wall cap can stop the run before the sample target; there is no convergence promise. Previous 60M/direct-controller sample budgets and runtime estimates do not apply.")
    missing = set(fixture.spec["training_run_manifest"]["required_fields"]) - manifest.keys()
    if missing:
        raise ValueError("prepared manifest lacks required fields: " + ", ".join(sorted(missing)))
    return {
        "schema": PROPOSAL_SCHEMA, "created_utc": utc_now(), "tracker_task": "PLAY-01",
        "launch_status": "pending_explicit_compute_authorization", "integration_evidence": binding,
        "identity": fixture.identity, "measured": throughput, "recommended": budget,
        "qualification_holdout_opened": False, "optimization_performed": False,
        "required_before_launch": ["explicit approval of this exact allocation", "authorized manifest copy", "fresh preflight"],
        "broader_coverage_gaps": ["turning and other directions", "long-horizon native gameplay qualification"],
        "sample_budget_rationale": manifest["budget_rationale"],
        "proposed_training_run_manifest": manifest,
    }


def powershell_quote(value: str | Path) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def prepare(args: argparse.Namespace) -> dict[str, str]:
    args.control_mode, args.command_speed, args.workers = "production-residual", 45.0, 3
    fixture = Fixture(args)
    if fixture.identity["git"]["dirty_files"]:
        raise ValueError("overnight preparation requires a clean source worktree")
    smoke = Path(args.smoke).resolve()
    evidence_path = smoke / "integration-evidence.json"
    if not re.fullmatch(r"[0-9a-f]{64}", args.integration_sha256 or ""):
        raise ValueError("--integration-sha256 requires the exact evidence SHA-256")
    if not evidence_path.is_file() or sha256(evidence_path) != args.integration_sha256:
        raise ValueError("integration evidence path/hash mismatch")
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    validate_integration(evidence, fixture)
    # Content hashes allow a later documentation commit, while a real source
    # lineage check records exactly which measured implementation is reused.
    measured_source = evidence["identity"].get("git", {}).get("commit", "")
    if not re.fullmatch(r"[0-9a-f]{40}", measured_source):
        raise ValueError("integration source commit is invalid")
    ancestry = subprocess.run(["git", "merge-base", "--is-ancestor", measured_source,
                              fixture.identity["git"]["commit"]], cwd=fixture.source_root, capture_output=True)
    if ancestry.returncode != 0:
        raise ValueError("integration source is not in the current source lineage")
    review_path = Path(args.reference_review).resolve()
    review = verify_forward_review({"path": str(review_path), "sha256": sha256(review_path)}, review_path.parent, fixture)
    measured_review = evidence.get("forward_reference_review", evidence.get("reference_review", {}))
    if measured_review.get("sha256") != review["sha256"]:
        raise ValueError("integration and allocation forward-reference review differ")
    binding = {"path": str(evidence_path), "sha256": sha256(evidence_path)}
    proposal = build_proposal(fixture, evidence, binding, review, args.hours, smoke)
    out = Path(args.out).resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError("overnight proposal output directory must be empty")
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / "training-run-manifest.pending.json"
    proposal_path = out / "allocation-proposal.json"
    summary_path = out / "SUMMARY.txt"
    commands_path = out / "REPRODUCTION.ps1"
    manifest = proposal["proposed_training_run_manifest"]
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    proposal_path.write_text(json.dumps(proposal, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    common = " ".join(f"--{key} {powershell_quote(value)}" for key, value in (
        ("source-root", fixture.source_root), ("exe", fixture.exe), ("library", fixture.library_path),
        ("floor", fixture.floor), ("bound-speed", fixture.bound_speed), ("command-speed", 45),
        ("control-mode", "production-residual"), ("workers", 3), ("tracker-task", "PLAY-01")))
    runner_path = fixture.source_root / "scripts/ragdoll-learning/play01_runner.py"
    authorized_path = out / "training-run-manifest.authorized.json"
    run_out = out / "training"
    python = powershell_quote(sys.executable)
    preflight = f"& {python} {powershell_quote(runner_path)} preflight {common} --checkpoint-mode fresh --manifest {powershell_quote(manifest_path)} --training-out {powershell_quote(run_out)} --out {powershell_quote(out / 'preflight')}"
    watchdog_path = fixture.source_root / "scripts/ragdoll-learning/play01_watchdog.py"
    worker_launch = f"{python} {powershell_quote(runner_path)} train {common} --checkpoint-mode fresh --manifest {powershell_quote(authorized_path)} --out {powershell_quote(run_out)}"
    launch = f"& {python} {powershell_quote(watchdog_path)} --manifest {powershell_quote(authorized_path)} --out {powershell_quote(run_out)} -- {worker_launch}"
    resume = launch.replace("--checkpoint-mode fresh", "--checkpoint-mode exact-resume")
    # This is a command reference, deliberately inert if opened/executed.
    commands_path.write_text(
        "# PLAY-01 overnight reproduction commands (all commented; preparation does not grant compute).\n"
        f"# Pending manifest SHA-256: {sha256(manifest_path)}\n"
        "# First approve the exact budget and create an authorized manifest copy with that approval reference.\n"
        f"# Set-Location -LiteralPath {powershell_quote(fixture.source_root)}\n"
        "# Preflight performs no physics:\n# " + preflight + "\n"
        "# Fresh launch after authorization:\n# " + launch + "\n"
        "# Resume the same authorized manifest and run directory:\n# " + resume + "\n",
        encoding="utf-8")
    budget = proposal["recommended"]
    summary_path.write_text(
        "PLAY-01 production-residual overnight allocation — PREPARED, NOT AUTHORIZED\n"
        f"source: {fixture.identity['git']['commit']}\n"
        f"executable SHA-256: {fixture.identity['executable_sha256']}\n"
        f"integration evidence SHA-256: {binding['sha256']}\n"
        f"pending manifest SHA-256: {sha256(manifest_path)}\n"
        f"measured complete-update usable transitions/s: {evidence['throughput']['complete_update_usable_transitions_per_second']}\n"
        f"sample target: {budget['target_usable_optimizer_transitions']:,} usable optimizer transitions\n"
        f"independent total physics cap: {budget['maximum_total_physics_steps']:,}\n"
        f"wall cap: {budget['maximum_cumulative_wall_seconds']} seconds; storage cap: 2,000,000,000 bytes\n"
        f"evaluation sample milestones: {budget['evaluation_milestones_usable_transitions']}\n"
        "workers: 3; episodes/batch: 6; command: fixed 45 units/s; start: fresh production residual\n"
        "checkpoint: durable latest every completed update; initial and best retained; milestone every 1000 updates\n"
        "qualification holdouts opened: false; allocated training launched: false\n"
        "Broader gaps: turns/other directions; native gameplay qualification.\n"
        "Previous 60M allocation and its runtime estimate are not reused.\n",
        encoding="utf-8")
    return {"manifest": str(manifest_path), "proposal": str(proposal_path),
            "summary": str(summary_path), "commands": str(commands_path)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=["prepare"])
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--exe", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--smoke", required=True, help="Directory containing integration-evidence.json")
    parser.add_argument("--integration-sha256", required=True)
    parser.add_argument("--reference-review", required=True)
    parser.add_argument("--hours", type=float, default=8.0)
    parser.add_argument("--out", required=True)
    parser.add_argument("--floor", choices=["tiled", "mesh", "box"], default="tiled")
    parser.add_argument("--bound-speed", type=float, default=None)
    parser.add_argument("--tracker-task", choices=["PLAY-01"], required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(prepare(args)))
    except (ValueError, RuntimeError) as error:
        parser.exit(1, str(error) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
