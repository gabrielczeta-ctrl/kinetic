"""Real child-process controls for bounded worker protocol failures (no physics)."""

from pathlib import Path
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

import numpy as np

from play01_worker_client import MAX_FRAME_BYTES, Worker, WorkerError


CHILD_HELPERS = r'''
import json, struct, sys, time
def frame(tag, payload):
    sys.stdout.buffer.write(struct.pack("<II", tag, len(payload)) + payload)
    sys.stdout.buffer.flush()
def manifest():
    frame(1, json.dumps({"kind": "manifest", "observation_size": 1,
        "action_count": 1, "bodies": [], "observation_layout": [["test", 1]]}).encode())
def read_frame():
    header = sys.stdin.buffer.read(8)
    tag, length = struct.unpack("<II", header)
    payload = sys.stdin.buffer.read(length)
    return tag, payload
'''


class WorkerDeadlineTests(unittest.TestCase):
    def setUp(self):
        self.children = []
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.close()
        for process in self.children:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)

    def worker(self, child, **timeouts):
        real_popen = subprocess.Popen

        def spawn(_command, **kwargs):
            process = real_popen([sys.executable, "-u", "-c", CHILD_HELPERS + child], **kwargs)
            self.children.append(process)
            return process

        with patch("play01_worker_client.subprocess.Popen", side_effect=spawn):
            worker = Worker(Path(sys.executable), Path("unused-context"), Path("unused-profile"),
                            **{"handshake_timeout_seconds": 5.0, "frame_timeout_seconds": 0.2, **timeouts})
        self.workers.append(worker)
        return worker

    def test_silent_handshake_times_out_and_reaps_child(self):
        started = time.monotonic()
        with self.assertRaisesRegex(WorkerError, "handshake timed out"):
            self.worker("time.sleep(30)\n", handshake_timeout_seconds=0.2)
        self.assertLess(time.monotonic() - started, 4.0)
        self.assertIsNotNone(self.children[0].poll())

    def test_partial_frame_times_out_and_reaps_child(self):
        worker = self.worker('manifest()\nread_frame()\nsys.stdout.buffer.write(b"\\x02")\nsys.stdout.buffer.flush()\ntime.sleep(30)\n')
        with self.assertRaisesRegex(WorkerError, "frame timed out"):
            worker.start_episode(np.asarray([1.0, 1.0], dtype=np.float32))
        self.assertIsNotNone(worker.process.poll())
        self.assertFalse(worker._reader.is_alive())

    def test_partial_payload_has_one_deadline_for_whole_frame(self):
        worker = self.worker('manifest()\nread_frame()\nsys.stdout.buffer.write(struct.pack("<II", 2, 100))\nsys.stdout.buffer.flush()\n'
                             'for _ in range(100):\n    sys.stdout.buffer.write(b"x")\n    sys.stdout.buffer.flush()\n    time.sleep(0.05)\n')
        started = time.monotonic()
        with self.assertRaisesRegex(WorkerError, "frame timed out"):
            worker.start_episode(np.asarray([1.0, 1.0], dtype=np.float32))
        self.assertLess(time.monotonic() - started, 4.0)
        self.assertIsNotNone(worker.process.poll())

    def test_stalled_physics_action_times_out_without_counting_a_solve(self):
        worker = self.worker('manifest()\nread_frame()\nframe(2, struct.pack("<f", 0.0))\nread_frame()\ntime.sleep(30)\n')
        worker.start_episode(np.asarray([1.0, 1.0], dtype=np.float32))
        with self.assertRaisesRegex(WorkerError, "frame timed out"):
            worker.send_action(np.zeros(1, dtype=np.float32))
        self.assertEqual(worker.physics_steps, 0)
        self.assertIsNotNone(worker.process.poll())

    def test_completed_frames_and_final_end_preserve_protocol(self):
        worker = self.worker('manifest()\nread_frame()\nframe(2, struct.pack("<f", 0.0))\n'
                             'for index in range(3):\n    read_frame()\n    frame(2, struct.pack("<f", index + 1.0))\n'
                             'frame(1, json.dumps({"kind": "end", "frames": 3}).encode())\n', frame_timeout_seconds=5.0)
        reader = worker._reader
        np.testing.assert_array_equal(worker.start_episode(np.asarray([1.0, 3.0], dtype=np.float32)), [0.0])
        for index in range(3):
            observation, end = worker.send_action(np.zeros(1, dtype=np.float32))
            np.testing.assert_array_equal(observation, [float(index + 1)])
            self.assertEqual(end, {"kind": "end", "frames": 3} if index == 2 else None)
            self.assertIs(worker._reader, reader)
        self.assertEqual(worker.physics_steps, 3)

    def test_eof_is_reported_promptly(self):
        worker = self.worker('manifest()\nread_frame()\n')
        with self.assertRaisesRegex(WorkerError, "closed its output"):
            worker.start_episode(np.asarray([1.0, 1.0], dtype=np.float32))
        self.assertIsNotNone(worker.process.poll())

    def test_oversized_frame_rejected_before_payload_allocation(self):
        worker = self.worker(f'manifest()\nread_frame()\nsys.stdout.buffer.write(struct.pack("<II", 2, {MAX_FRAME_BYTES + 1}))\nsys.stdout.buffer.flush()\ntime.sleep(30)\n')
        with self.assertRaisesRegex(WorkerError, "exceeds limit"):
            worker.start_episode(np.asarray([1.0, 1.0], dtype=np.float32))
        self.assertIsNotNone(worker.process.poll())

    def test_timeout_must_be_finite_positive(self):
        for value in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Worker(Path(sys.executable), Path("unused"), Path("unused"), frame_timeout_seconds=value)


if __name__ == "__main__":
    unittest.main()
