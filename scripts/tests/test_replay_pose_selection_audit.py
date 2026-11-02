import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('pose_audit', Path(__file__).parents[1] / 'replay_pose_selection_audit.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PoseSelectionTests(unittest.TestCase):
    def render_fixture(self):
        return [
            f'RAGDOLL_ANIMATION_INPUT schema=1 player=1 actor="player" serial={i} next_step=10 body=0 bodies=1 position={i},0,0 rotation=0,0,0,1'
            for i in range(3)] + [
            'RAGDOLL_ANIMATION_RAW schema=1 player=1 actor="player" serial=2 next_step=10 previous_known=1 render=20 raw=0,60,0',
            'RAGDOLL_STEP phase=input player=1 step=10 mode=0',
            'RAGDOLL_REPLAY_BASE player=1 step=10 body=0 position=1,0,0 rotation=0,0,0,1',
            'RAGDOLL_REPLAY_FRAME frame=0 step=10']

    def test_render_history_includes_animation_without_physics(self):
        result = module.audit_render_history(self.render_fixture(), 'player')
        self.assertTrue(result['selected_poses_exact'])
        self.assertEqual(result['animation_captures'], 3)
        self.assertEqual(result['frames'], 1)

    def test_missing_render_history_is_rejected(self):
        lines = [l for l in self.render_fixture() if 'serial=1 ' not in l]
        with self.assertRaisesRegex(ValueError, 'Missing animation capture history'):
            module.audit_render_history(lines, 'player')

    def test_reconstruction_does_not_consume_base_observations(self):
        lines = [line for line in self.render_fixture() if 'RAGDOLL_REPLAY_BASE ' not in line]
        result = module.audit_render_history(lines, 'player', reconstruct_only=True)
        self.assertEqual(result[10], (1., 0., 0., 0., 0., 0., 1.))

    def test_explicit_mode_history_controls_recovery_selection(self):
        lines = [line.replace('mode=0', 'mode=invalid') for line in self.render_fixture()]
        result = module.audit_render_history(lines, 'player', reconstruct_only=True, input_modes={10: 5})
        self.assertEqual(result[10], (2., 0., 0., 0., 0., 0., 1.))
        with self.assertRaises(KeyError):
            module.audit_render_history(lines, 'player', reconstruct_only=True, input_modes={})

    def test_paired_candidates_do_not_read_native_mode_or_base(self):
        lines = [line.replace('mode=0', 'mode=invalid') for line in self.render_fixture()
                 if 'RAGDOLL_REPLAY_BASE ' not in line]
        pairs = module.audit_render_history(lines, 'player', reconstruct_only=True, paired_candidates=True)
        self.assertEqual(pairs[10], ((1., 0., 0., 0., 0., 0., 1.), (2., 0., 0., 0., 0., 0., 1.)))

    def test_trace_truncation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'truncated'):
            module.audit_render_history(self.render_fixture() + ['RAGDOLL_ANIMATION_INPUT_TRUNCATED player=1'], 'player')

    def test_initial_pose_backfill_does_not_change_event_order(self):
        lines = self.render_fixture()
        lines.insert(0, 'RAGDOLL_ANIMATION_RAW schema=1 player=1 actor="player" serial=0 next_step=9 previous_known=0 render=19 raw=0,0,0')
        self.assertTrue(module.audit_render_history(lines, 'player')['selected_poses_exact'])

    def run_case(self, render_ids, moving, actual_values):
        # Deliberately distinct animation values make an incorrect frozen
        # sample detectable, rather than testing only a constant idle pose.
        data = b'OMWRDPL\0' + struct.pack('<IQQfIIIIII', 10, 1, 2, 1/60, 1, 1, 17, 10, 1, len(render_ids))
        lines = []
        for i, render in enumerate(render_ids):
            pose = (float(i), 0, 0, 0, 0, 0, 1)
            data += struct.pack('<Q', i) + struct.pack('<40f', *([0.] * 10), *pose, *([0.] * 23))
            lines.extend([
                f'RAGDOLL_REPLAY_FRAME frame={i} step={i}',
                f'RAGDOLL_STEP phase=input actor="player" player=1 step={i} render={render} raw=0,{60 if moving[i] else 0},0 mode=0',
                f'RAGDOLL_REPLAY_BASE player=1 step={i} body=0 position={actual_values[i]},0,0 rotation=0,0,0,1'])
        with tempfile.TemporaryDirectory() as folder:
            capture = Path(folder) / 'capture'; log = Path(folder) / 'log'
            capture.write_bytes(data); log.write_text('\n'.join(lines))
            return module.audit(capture, log, 'player')

    def test_freezes_previous_render_animation_until_raw_stop(self):
        result = self.run_case([10, 11, 12, 13], [False, True, True, False], [0, 0, 0, 3])
        self.assertTrue(result['all_pose_components_exact'])
        self.assertEqual(result['counts']['previous_render_frozen_exact'], 2)

    def test_unrecorded_intermediate_render_is_not_invented(self):
        result = self.run_case([10, 12, 14], [False, True, True], [0, 0.5, 0.5])
        self.assertFalse(result['all_pose_components_exact'])
        self.assertEqual(result['first_mismatches'][0]['frame'], 1)

    def test_initial_movement_without_history_is_unavailable(self):
        result = self.run_case([10], [True], [0])
        self.assertFalse(result['all_pose_components_exact'])
        self.assertEqual(result['counts']['unavailable_history'], 1)


if __name__ == '__main__':
    unittest.main()
