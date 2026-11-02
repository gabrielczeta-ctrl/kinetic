import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "replay_clock_audit", Path(__file__).parents[1] / "replay_clock_audit.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def trace(step, substep=1, backlog=0, render=1):
    return (f'RAGDOLL_STEP phase=observed actor="npc" player=0 step={step} '
        f'substep={substep} render={render} physics_dt=0.125 '
        f'render_dt=1.5 backlog={backlog}')


class ClockAuditTests(unittest.TestCase):
    def test_discard_is_distinct_from_missing_recording(self):
        lines = [trace(20, 10, 0.25), trace(21, 1, 0, 2),
            'RAGDOLL_REPLAY_FRAME frame=0 step=20',
            'RAGDOLL_REPLAY_FRAME frame=1 step=21']
        result = module.audit(lines, 'npc')
        self.assertTrue(result['executed_step_coverage_complete'])
        self.assertEqual(result['recorded_physics_seconds'], 0.25)
        self.assertEqual(result['discarded_seconds'], 0.25)
        self.assertEqual(result['catchup_cap_events'][0]['discarded_whole_steps'], 2)

    def test_backlog_below_cap_is_not_discarded(self):
        result = module.audit([trace(20, 9, 0.25),
            'RAGDOLL_REPLAY_FRAME frame=0 step=20'], 'npc')
        self.assertEqual(result['catchup_cap_events'], [])

    def test_missing_executed_step_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'coverage incomplete'):
            module.audit([trace(20), trace(22),
                'RAGDOLL_REPLAY_FRAME frame=0 step=20',
                'RAGDOLL_REPLAY_FRAME frame=1 step=22'], 'npc')

    def test_multiple_recordings_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'recording indices'):
            module.audit([trace(20), trace(21),
                'RAGDOLL_REPLAY_FRAME frame=0 step=20',
                'RAGDOLL_REPLAY_FRAME frame=0 step=21'], 'npc')


if __name__ == '__main__':
    unittest.main()
