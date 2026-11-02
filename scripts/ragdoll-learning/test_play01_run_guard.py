"""Interrupted-write and budget-ledger controls using temporary directories."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import play01_run_guard as guard


class RunGuardTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="play01-run-guard-")
        self.addCleanup(self.temporary.cleanup)
        self.out = Path(self.temporary.name)

    @staticmethod
    def save(path):
        content = b"checkpoint state and optimizer/RNG snapshot"
        path.write_bytes(content)
        return {"path": str(path), "sha256": hashlib.sha256(content).hexdigest(),
                "counters": {"cumulative_wall_seconds": 5.0}}

    def commit(self, generation, **state):
        return guard.commit_generation(self.out, generation, self.save, {"counters": {"steps": generation}, **state})

    def state(self):
        return json.loads((self.out / guard.STATE_FILE).read_text(encoding="utf-8"))

    def test_commit_selects_hashed_immutable_checkpoint(self):
        initial = {"run_id": "PLAY-01-test"}
        state = guard.commit_generation(self.out, 1, self.save, initial)
        self.assertEqual(initial, {"run_id": "PLAY-01-test"})
        self.assertEqual(self.state(), state)
        self.assertEqual(state["checkpoint_generation"], 1)
        self.assertEqual(Path(state["latest_checkpoint"]["path"]), self.out / "checkpoint-state-1.pt")
        self.assertEqual(state["latest_checkpoint"]["sha256"], hashlib.sha256(Path(state["latest_checkpoint"]["path"]).read_bytes()).hexdigest())
        self.assertEqual(state["latest_checkpoint"]["counters"]["cumulative_wall_seconds"], 5.0)

    def test_checkpoint_written_but_pointer_interrupted_preserves_previous(self):
        first = self.commit(1)
        reservation = guard.begin_work(self.out, 600, 30.0)
        real_atomic = guard._atomic_json

        def interrupt_pointer(path, value):
            if path.name == guard.STATE_FILE:
                raise OSError("simulated interruption before state pointer")
            real_atomic(path, value)

        with patch.object(guard, "_atomic_json", side_effect=interrupt_pointer):
            with self.assertRaisesRegex(OSError, "simulated interruption"):
                self.commit(2)
        self.assertEqual(self.state(), first)
        self.assertTrue(Path(first["latest_checkpoint"]["path"]).is_file())
        self.assertTrue((self.out / "checkpoint-state-2.pt").is_file())
        self.assertEqual(guard.recover_pending(self.out), reservation)
        self.assertEqual(guard.next_generation(self.out), 3)
        recovered = self.commit(3, unknown_physics_steps_reserved=600)
        self.assertEqual(recovered["unknown_physics_steps_reserved"], 600)
        self.assertIsNone(guard.recover_pending(self.out))

    def test_interrupted_callback_keeps_old_checkpoint_and_ledger(self):
        first = self.commit(1)
        reservation = guard.begin_work(self.out, 1200, 50.0)

        def partial_save(path):
            path.write_bytes(b"partial checkpoint")
            raise OSError("interrupted checkpoint write")

        with self.assertRaisesRegex(OSError, "interrupted checkpoint write"):
            guard.commit_generation(self.out, 2, partial_save, {})
        self.assertEqual(self.state(), first)
        self.assertEqual(guard.recover_pending(self.out), reservation)
        self.assertEqual(guard.next_generation(self.out), 3)

    def test_recovery_does_not_delete_or_double_count_pending(self):
        self.commit(1)
        pending = guard.begin_work(self.out, 1800, 60.0)
        for _ in range(2):
            self.assertEqual(guard.recover_pending(self.out), pending)
            self.assertTrue((self.out / guard.PENDING_FILE).is_file())
        with self.assertRaisesRegex(RuntimeError, "pending work"):
            guard.begin_work(self.out, 1800, 60.0)
        real_unlink = Path.unlink

        def interrupt_cleanup(path, *args, **kwargs):
            if path.name == guard.PENDING_FILE:
                raise OSError("interrupted after pointer commit")
            return real_unlink(path, *args, **kwargs)

        with patch.object(Path, "unlink", interrupt_cleanup):
            with self.assertRaisesRegex(OSError, "after pointer commit"):
                self.commit(2, unknown_physics_steps_reserved=1800)
        self.assertEqual(self.state()["checkpoint_generation"], 2)
        self.assertTrue((self.out / guard.PENDING_FILE).is_file())
        self.assertIsNone(guard.recover_pending(self.out))
        next_pending = guard.begin_work(self.out, 600, 20.0)
        self.assertEqual(next_pending["base_checkpoint_generation"], 2)
        self.assertEqual(guard.recover_pending(self.out), next_pending)

    def test_cleanup_retains_two_committed_files_and_unrelated_content(self):
        untouched = self.out / "checkpoint-state-not-a-generation.pt"
        untouched.write_bytes(b"keep")
        nested = self.out / "nested"
        nested.mkdir()
        (nested / "checkpoint-state-1.pt").write_bytes(b"keep nested")
        self.commit(1)
        self.commit(2)
        state = self.commit(3)
        self.assertFalse((self.out / "checkpoint-state-1.pt").exists())
        self.assertTrue((self.out / "checkpoint-state-2.pt").is_file())
        self.assertTrue((self.out / "checkpoint-state-3.pt").is_file())
        self.assertEqual(state["committed_checkpoint_generations"], [2, 3])
        self.assertEqual(untouched.read_bytes(), b"keep")
        self.assertEqual((nested / "checkpoint-state-1.pt").read_bytes(), b"keep nested")

    def test_hash_mismatch_does_not_replace_committed_pointer(self):
        first = self.commit(1)

        def bad_hash(path):
            metadata = self.save(path)
            metadata["sha256"] = "0" * 64
            return metadata

        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            guard.commit_generation(self.out, 2, bad_hash, {})
        self.assertEqual(self.state(), first)

    def test_existing_generation_never_overwritten(self):
        self.commit(1)
        with self.assertRaisesRegex(ValueError, "must increase"):
            self.commit(1)
        (self.out / "checkpoint-state-2.pt").write_bytes(b"orphan")
        with self.assertRaises(FileExistsError):
            self.commit(2)
        self.assertEqual((self.out / "checkpoint-state-2.pt").read_bytes(), b"orphan")

    def test_reservation_bounds_are_validated(self):
        for physics, wall in ((-1, 1.0), (1.2, 1.0), (True, 1.0), (1, 0.0), (1, float("inf")), (1, float("nan"))):
            with self.subTest(physics=physics, wall=wall), self.assertRaises(ValueError):
                guard.begin_work(self.out, physics, wall)
        self.assertFalse((self.out / guard.PENDING_FILE).exists())

    def test_atomic_pointer_serialization_failure_keeps_old_state(self):
        first = self.commit(1)
        with self.assertRaises(ValueError):
            self.commit(2, invalid_metric=float("nan"))
        self.assertEqual(self.state(), first)

    def child_lock(self, body):
        script = ("import sys, time\n"
                  "from pathlib import Path\n"
                  "from play01_run_guard import exclusive_run\n"
                  "with exclusive_run(Path(sys.argv[1])):\n"
                  + "\n".join("    " + line for line in body.splitlines()) + "\n")
        return subprocess.Popen([sys.executable, "-u", "-c", script, str(self.out)],
                                cwd=Path(guard.__file__).parent, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True)

    def test_run_mutex_rejects_second_owner_and_releases_normally(self):
        with guard.exclusive_run(self.out):
            child = self.child_lock('print("acquired", flush=True)')
            stdout, stderr = child.communicate(timeout=5)
            self.assertNotEqual(child.returncode, 0)
            self.assertEqual(stdout, "")
            self.assertIn("active owner", stderr)
        with guard.exclusive_run(self.out):
            self.assertTrue((self.out / guard.LOCK_FILE).is_file())

    def test_run_mutex_is_released_after_owner_process_crashes(self):
        child = self.child_lock('print("acquired", flush=True)\ntime.sleep(30)')
        try:
            self.assertEqual(child.stdout.readline().strip(), "acquired")
            with self.assertRaisesRegex(RuntimeError, "active owner"):
                with guard.exclusive_run(self.out):
                    pass
            child.kill()
            child.communicate(timeout=5)
            with guard.exclusive_run(self.out):
                self.assertTrue((self.out / guard.LOCK_FILE).is_file())
        finally:
            if child.poll() is None:
                child.kill()
                child.communicate(timeout=5)


if __name__ == "__main__":
    unittest.main()
