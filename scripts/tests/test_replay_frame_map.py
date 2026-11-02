import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_frame_map import frame_map


class FrameMapTest(unittest.TestCase):
    def test_warmup_and_next_command_are_distinct(self):
        log = 'RAGDOLL_REPLAY_FRAME step=19 frame=0\nRAGDOLL_REPLAY_FRAME step=20 frame=1\n'
        result = frame_map(log, {'driver_result': {'frames': 2, 'warmup_frames': 3}})
        self.assertEqual(result[19]['physical_scenario_step'], 3)
        self.assertEqual(result[19]['following_command_scenario_step'], 4)
        self.assertEqual(result[20]['physical_scenario_step'], 4)
        self.assertIsNone(result[20]['following_command_scenario_step'])

    def test_rejects_missing_duplicate_and_wrong_capture_coverage(self):
        report = {'driver_result': {'frames': 2, 'warmup_frames': 1}}
        for log in ('', 'RAGDOLL_REPLAY_FRAME step=8 frame=0\n',
                    'RAGDOLL_REPLAY_FRAME step=8 frame=0\nRAGDOLL_REPLAY_FRAME step=8 frame=1\n',
                    'RAGDOLL_REPLAY_FRAME step=8 frame=0\nRAGDOLL_REPLAY_FRAME step=9 frame=0\n'):
            with self.subTest(log=log), self.assertRaises(ValueError):
                frame_map(log, report)


if __name__ == '__main__':
    unittest.main()
