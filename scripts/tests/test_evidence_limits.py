import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_recording import TestRecording
from diagnostic_report import summarize


class EvidenceLimits(unittest.TestCase):
    def test_full_recording_budget_preserves_existing_files_and_skips_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / 'existing.mp4'
            existing.write_bytes(b'123456')
            recorder = TestRecording(directory)
            recorder.TOTAL_BYTES, recorder.MAX_BYTES = 10, 6
            with patch('test_recording.subprocess.Popen') as start:
                recorder.start('budget check')
                start.assert_not_called()
            self.assertEqual(existing.read_bytes(), b'123456')

    def test_report_separates_sampling_loss_from_retention_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.jsonl.gz'
            rows = [
                {'kind': 'frame', 'test': 'turn', 'physics_step': 10, 'simulation_time': 1,
                 'state': {'controller': {'mode': 'bracing'}}},
                {'kind': 'frame', 'test': 'turn', 'physics_step': 15, 'simulation_time': 2,
                 'skipped_steps': 1, 'state': {'controller': {'mode': 'falling'}}, 'input': {'movement': 1}},
            ]
            with gzip.open(path, 'wt') as stream:
                for row in rows:
                    stream.write(json.dumps(row) + '\n')
            report = json.loads(summarize(path).read_text())['tests']['turn']
            self.assertEqual(report['skipped_steps'], 1)
            self.assertEqual(report['omitted_between_windows'], 3)
            self.assertEqual(report['first_failure']['input']['movement'], 1)
            self.assertIsNone(report['replay'])


if __name__ == '__main__':
    unittest.main()
