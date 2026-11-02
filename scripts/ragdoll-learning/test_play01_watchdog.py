"""Pure budget/command and fake-process watchdog tests; no physics or training."""

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import play01_watchdog as watchdog


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="play01-watchdog-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.out = self.root / "training"
        self.manifest = self.root / "manifest.json"
        self.document = {"run_id": "PLAY-01-watchdog-test", "maximum_cumulative_wall_seconds": 100,
                         "maximum_total_physics_steps": 10000, "target_usable_optimizer_transitions": 5000}
        self.write(self.manifest, self.document)

    @staticmethod
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def command(self, mode="fresh"):
        return [sys.executable, "-u", str(watchdog.RUNNER), "train", "--manifest", str(self.manifest),
                "--out", str(self.out), "--checkpoint-mode", mode]

    def state(self):
        state = {"run_id": self.document["run_id"],
                 "manifest_sha256": hashlib.sha256(self.manifest.read_bytes()).hexdigest(),
                 "checkpoint_generation": 3, "cumulative_wall_seconds": 30,
                 "counters": {"total_simulated_physics_steps": 1000,
                              "training_usable_optimizer_transitions": 500},
                 "recovery_reservations": {"unknown_physics_steps_reserved": 100, "wall_seconds_reserved": 10}}
        self.write(self.out / "train-state.json", state)
        return state

    @staticmethod
    def pending(pid=4321, start=100.0, bound=5.0, base=3, reservation="new-unit"):
        return {"schema": "play01_pending_work_v1", "reservation_id": reservation, "owner_pid": pid,
                "started_monotonic_seconds": start, "base_checkpoint_generation": base,
                "physics_upper_bound": 200, "wall_upper_bound": bound}

    def test_command_requires_exact_runner_stage_and_matching_unique_paths(self):
        valid, mode = watchdog.validate_command(["--"] + self.command(), self.manifest, self.out)
        self.assertEqual(mode, "fresh")
        self.assertEqual(Path(valid[2]), watchdog.RUNNER)
        for command in (self.command() + ["--out", str(self.out)],
                        self.command() + ["--manifest=" + str(self.manifest)],
                        self.command()[:3] + ["teacher-bootstrap"] + self.command()[4:],
                        [sys.executable, "-c", "pass"] + self.command()[4:],
                        [sys.executable, str(self.root / "play01_runner.py"), "train"] + self.command()[4:],
                        self.command()[:-4] + ["--out", str(self.root / "wrong"), "--checkpoint-mode", "fresh"]):
            with self.subTest(command=command), self.assertRaises(ValueError):
                watchdog.validate_command(command, self.manifest, self.out)

    def test_fresh_does_not_create_output_and_resume_preserves_cumulative_charges(self):
        plan = watchdog.prepare_launch(self.manifest, self.out, self.command())
        self.assertEqual(plan.remaining_wall_seconds, 100)
        self.assertFalse(self.out.exists())
        self.state()
        self.write(self.out / "pending-work.json", self.pending())
        plan = watchdog.prepare_launch(self.manifest, self.out, self.command("exact-resume"))
        self.assertEqual(plan.remaining_wall_seconds, 65)  # prior recovery already included in 30
        self.assertEqual(plan.reserved_unknown_physics_steps, 300)
        self.assertEqual(plan.previous_pending_id, "new-unit")
        state = self.state()
        state["checkpoint_generation"] = 4
        self.write(self.out / "train-state.json", state)
        plan = watchdog.prepare_launch(self.manifest, self.out, self.command("exact-resume"))
        self.assertEqual(plan.remaining_wall_seconds, 70)  # committed state already covers stale ledger
        self.assertTrue((self.out / "pending-work.json").exists())

    def test_resume_rejects_wrong_identity_modes_invalid_and_exhausted_budgets(self):
        with self.assertRaises(ValueError):
            watchdog.prepare_launch(self.manifest, self.out, self.command("exact-resume"))
        original = self.state()
        for field, value in (("manifest_sha256", "wrong"), ("run_id", "wrong"),
                             ("cumulative_wall_seconds", 100), ("cumulative_wall_seconds", float("nan"))):
            changed = dict(original, **{field: value})
            self.write(self.out / "train-state.json", changed)
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                watchdog.prepare_launch(self.manifest, self.out, self.command("exact-resume"))
        for counter, value in (("total_simulated_physics_steps", 9900),
                               ("training_usable_optimizer_transitions", 5000)):
            changed = dict(original, counters={**original["counters"], counter: value})
            self.write(self.out / "train-state.json", changed)
            with self.assertRaises(ValueError):
                watchdog.prepare_launch(self.manifest, self.out, self.command("exact-resume"))
        self.write(self.out / "train-state.json", original)
        with self.assertRaises(ValueError):
            watchdog.prepare_launch(self.manifest, self.out, self.command("fresh"))

    def simulate(self, *, wall_cap=100, pending=None, child_exit_at=None, child_status=0):
        self.document["maximum_cumulative_wall_seconds"] = wall_cap
        self.write(self.manifest, self.document)
        plan = watchdog.prepare_launch(self.manifest, self.out, self.command())
        now = [100.0]
        killed = []

        class Child:
            pid = 4321
            status = None

            def poll(child):
                if child_exit_at is not None and now[0] >= 100 + child_exit_at:
                    child.status = child_status
                return child.status

        child = Child()

        def spawn(command, **options):
            self.assertEqual(command, plan.command)
            self.assertFalse(self.out.exists(), "supervisor contaminated fresh output")
            self.state()
            if pending is not None:
                self.write(self.out / "pending-work.json", pending)
            self.assertTrue("creationflags" in options or options.get("start_new_session"))
            return child

        def advance(seconds):
            self.assertLessEqual(seconds, 1)
            now[0] += seconds

        def terminate(process):
            self.assertIs(process, child)
            killed.append(process.pid)
            child.status = 1

        code = watchdog.supervise(plan, popen=spawn, clock=lambda: now[0], sleep=advance, terminate=terminate)
        result = json.loads((self.out / "watchdog-result.json").read_text(encoding="utf-8"))
        return code, result, killed

    def test_owned_pending_deadline_kills_only_child_and_preserves_ledger(self):
        pending = self.pending(bound=3.5)
        code, result, killed = self.simulate(pending=pending)
        self.assertEqual(code, 124)
        self.assertEqual(result["reason"], "pending_work_deadline")
        self.assertEqual(result["execution_elapsed_seconds"], 3.5)
        self.assertEqual(killed, [4321])
        self.assertEqual(json.loads((self.out / "pending-work.json").read_text()), pending)

    def test_overall_deadline_bounds_setup_even_before_any_pending_unit(self):
        code, result, killed = self.simulate(wall_cap=2.5)
        self.assertEqual(code, 124)
        self.assertEqual(result["reason"], "cumulative_wall_deadline")
        self.assertEqual(result["execution_elapsed_seconds"], 2.5)
        self.assertEqual(killed, [4321])

    def test_foreign_pending_owner_never_triggers_tree_kill(self):
        pending = self.pending(pid=9999, start=1, bound=1)
        code, result, killed = self.simulate(pending=pending, child_exit_at=2, child_status=7)
        self.assertEqual(code, 7)
        self.assertEqual(result["child_exit_code"], 7)
        self.assertEqual(result["reason"], "child_exited")
        self.assertEqual(killed, [])
        self.assertEqual(json.loads((self.out / "pending-work.json").read_text()), pending)

    def test_invalid_owned_ledger_stops_child_and_records_error(self):
        pending = self.pending(bound=float("nan"))
        code, result, killed = self.simulate(pending=pending)
        self.assertEqual(code, 125)
        self.assertEqual(result["reason"], "watchdog_error")
        self.assertIn("pending wall bound", result["error"])
        self.assertEqual(killed, [4321])
        self.assertTrue(result["pending_ledger_preserved"])


if __name__ == "__main__":
    unittest.main()
