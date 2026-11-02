from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from fps_sampling_audit import audit


class SamplingAudit(unittest.TestCase):
    def report(self, fps, steps, extra=''):
        dt = struct.unpack('f', struct.pack('f', 1 / fps))[0]
        text = (f'RAGDOLL_FPS_CLOCK requested_fps={fps}\n'
                f'RAGDOLL_FPS_CLOCK_ARMED fps={fps}\n')
        text += ''.join(f'RAGDOLL_ANIMATION_RAW render={i} next_step={step} render_dt={dt}\n'
                        for i, step in enumerate(steps))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'log'
            path.write_text(text + extra)
            return audit(path)

    def test_three_valid_cadences_are_not_behavior_qualification(self):
        for fps, steps in ((30, [0, 2, 4, 6]), (60, [0, 1, 2, 3]), (120, [0, 0, 1, 1])):
            result = self.report(fps, steps)
            self.assertTrue(result['cadence_valid'], result)
            self.assertFalse(result['qualification_ready'])

    def test_lost_step_or_startup_error_rejects_cadence(self):
        self.assertFalse(self.report(60, [0, 1, 3, 4])['cadence_valid'])
        self.assertFalse(self.report(120, [0, 0, 0, 1])['cadence_valid'])
        self.assertFalse(self.report(60, [0, 1, 2, 3], '[00:00:00 E] Lua error: failed')['cadence_valid'])
