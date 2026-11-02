import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('replay_context', Path(__file__).parents[1] / 'replay_context.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayContextTests(unittest.TestCase):
    def contact_input(self, step, **changes):
        values = dict(duration='0.0166666675', locomotion_requested='1', interrupted='0',
            left_supported='1', right_supported='1', production_ready='1',
            measured_velocity='1,2,0', left_slip='0.25', right_slip='0.5',
            left_load='0.6', right_load='0.4', support_error='0.1',
            incoming_available='0', incoming_error='0', requested_direction='0,1,0')
        values.update(changes)
        return ('RAGDOLL_CONTACT_ROOT_INPUT schema=1 integer_radix=10 present=1 player=1 '
            f'step={step} ' + ' '.join(f'{name}={value}' for name,value in values.items()))

    def fixture(self):
        return '\n'.join([
            'RAGDOLL_REPLAY_RIG profile_id=test profile_hash=1 controller_steps=1 mass=75 scale=1 root=0,0,1 rotation=0,0,0,1 velocity=0,0,0 angular_velocity=0,0,0',
            'RAGDOLL_REPLAY_BONE index=0 name="pelvis" parent=-1 position=0,0,75 rotation=0,0,0,1',
            'RAGDOLL_REPLAY_FRAME frame=0 step=11 root=0,2,1 rotation=0,0,0,1',
            *[f'RAGDOLL_STEP phase=applied player=1 integer_radix=10 step={step} root=0,1,1 filtered=0,0,0 facing=0,1,0' for step in (10,11)],
            *[f'RAGDOLL_REPLAY_BASE step={step} player=1 body=0 position=0,0,{height} rotation=0,0,0,1' for step,height in ((10,74),(11,73))],
        ])

    def extract(self, text, actor_kind="player"):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'log'; output = Path(folder) / 'context'
            log.write_text(text, encoding='utf-8')
            result = module.extract(log, output, actor_kind=actor_kind)
            return result, output.read_text()

    def test_callback_boundary_is_preserved_not_inferred(self):
        text = self.fixture().replace('profile_id=test', 'recovery_event_schema=2 profile_id=test')
        event = '\nRAGDOLL_REPLAY_RECOVERY_FINISH integer_radix=10 player=1 next_step=11'
        result, context = self.extract(text + event)
        self.assertEqual(result['recovery_finish_steps'], [1])
        self.assertTrue(context.endswith('recovery_finish_events_v1 1\n1\n'))
        legacy = text.replace('recovery_event_schema=2', 'recovery_event_schema=1')
        legacy_result, legacy_context = self.extract(legacy + event.replace('next_step=11', 'next_step=12'))
        self.assertEqual(legacy_context, context)
        self.assertEqual(legacy_result['recovery_event_step_adjustment'], -1)
        result, context = self.extract(text)
        self.assertTrue(result['recovery_finish_events_captured'])
        self.assertTrue(context.endswith('recovery_finish_events_v1 0\n'))
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.extract(text + event + event)
        with self.assertRaisesRegex(ValueError, 'lack schema'):
            self.extract(self.fixture() + event)

    def test_npc_selection_ignores_player_records(self):
        npc = self.fixture().replace('player=1', 'player=0 actor="npc_a"')
        player_records = '\n'.join(l for l in self.fixture().splitlines()
            if 'RAGDOLL_STEP ' in l or 'RAGDOLL_REPLAY_BASE ' in l)
        result, selected = self.extract(npc + '\n' + player_records, 'npc')
        _, expected = self.extract(self.fixture())
        self.assertEqual(selected, expected)
        self.assertEqual(result['actor_kind'], 'npc')

    def test_multiple_npcs_are_rejected_even_at_different_steps(self):
        npc = self.fixture().replace('player=1', 'player=0 actor="npc_a"')
        extra = 'RAGDOLL_STEP phase=applied player=0 actor="npc_b" integer_radix=10 step=99'
        with self.assertRaisesRegex(ValueError, 'Multiple selected actors'):
            self.extract(npc + '\n' + extra, 'npc')

    def test_consumed_pose_includes_warmup_and_pre_step_root(self):
        result, text = self.extract(self.fixture())
        self.assertTrue(text.startswith('2 '))
        self.assertIn('0 1 1 0 0 0 1\n0 0 74 0 0 0 1', text)
        self.assertIn('0 1 1 0 0 0 1\n0 0 73 0 0 0 1', text)
        self.assertEqual(result['warmup_frames'], 1)

    def test_missing_consumed_pose_is_rejected(self):
        with self.assertRaises(KeyError):
            self.extract(self.fixture().replace('RAGDOLL_REPLAY_BASE step=10', 'IGNORED step=10'))

    def test_body_id_and_sequence_are_preserved(self):
        record = '\nRAGDOLL_REPLAY_INITIAL player=1 body=0 body_id=16777367'
        _, text = self.extract(self.fixture() + record)
        self.assertTrue(text.startswith('3 '))
        self.assertIn('\n16777367\n1\n', text)
        with self.assertRaises(ValueError):
            self.extract(self.fixture() + record + record)

    def test_duplicate_or_wrong_radix_is_rejected(self):
        line = next(line for line in self.fixture().splitlines() if 'phase=applied' in line)
        for text in (self.fixture() + '\n' + line, self.fixture().replace('integer_radix=10', 'integer_radix=16')):
            with self.assertRaises(ValueError):
                self.extract(text)

    def test_ground_capture_preserves_material_and_id(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); log=root/'log'; output=root/'context'
            (root/'ground-25166269.jolt').write_bytes(b'fixture')
            record='\nRAGDOLL_REPLAY_GROUND body_id=25166269 saved=1 position=0,0,0 rotation=0,0,0,1 friction=0.2 restitution=0'
            text=self.fixture()+'\nRAGDOLL_REPLAY_INITIAL player=1 body=0 body_id=16777367'+record
            log.write_text(text)
            module.extract(log,output,root)
            self.assertTrue(output.read_text().startswith('4 '))
            self.assertIn('25166269 0 0 0 0 0 0 1 0.2 0 ',output.read_text())
            log.write_text(text+record)
            with self.assertRaises(ValueError):module.extract(log,output,root)


    def test_reconstructed_poses_replace_observations_and_require_warmup(self):
        text = '\n'.join(line for line in self.fixture().splitlines()
                         if 'RAGDOLL_REPLAY_BASE ' not in line)
        poses = {step: (0., 0., 72., 0., 0., 0., 1.) for step in (10, 11)}
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'log'; output = Path(folder) / 'context'
            log.write_text(text)
            result = module.extract(log, output, reconstructed_poses=poses)
            self.assertEqual(result['warmup_pose_assumption'], 'reconstructed from render history')
            self.assertEqual(output.read_text().count('0.0 0.0 72.0 0.0 0.0 0.0 1.0'), 2)
            del poses[10]
            with self.assertRaisesRegex(ValueError, 'Missing or incomplete reconstructed pose'):
                module.extract(log, output, reconstructed_poses=poses)

    def test_unsupported_direct_events_are_not_silently_discarded(self):
        for event in ('RAGDOLL_REPLAY_RECOVERY_REQUEST',):
            with self.assertRaisesRegex(ValueError, 'Undeclared recovery request'):
                self.extract(self.fixture() + '\n' + event + ' player=1 schema=1 next_step=11')

    def test_direct_impulse_preserves_pre_solve_step_and_world_point(self):
        text=self.fixture().replace('profile_id=test',
            'direct_event_schema=1 recovery_event_schema=2 profile_id=test')
        event='\nRAGDOLL_REPLAY_EXTERNAL_IMPULSE player=1 schema=1 integer_radix=10 next_step=11 body=0 impulse=1,2,3 point_present=1 point=4,5,6'
        _, context=self.extract(text+event)
        self.assertTrue(context.endswith('direct_impulses_v1 1\n1 0 1 2 3 1 4 5 6\nrecovery_requests_v1 0\n'))
        with self.assertRaisesRegex(ValueError, 'outside replay coverage'):
            self.extract(text+event.replace('next_step=11','next_step=12'))
        with self.assertRaisesRegex(ValueError, 'lacks rig declaration'):
            self.extract(self.fixture()+event)

    def test_raw_facing_uses_request_and_rejects_missing_history(self):
        with tempfile.TemporaryDirectory() as directory:
            log=Path(directory)/'log'; output=Path(directory)/'context'
            log.write_text(self.fixture().replace('facing=0,1,0','facing=0,1,0 facing_request=1,0,0'))
            report=module.extract(log,output,raw_facing=True)
            self.assertEqual(report['facing_source'],'native consumed raw request')
            self.assertIn('0 0 1 0 0 0 1 0 0',output.read_text())
            log.write_text(self.fixture())
            with self.assertRaises(KeyError): module.extract(log,output,raw_facing=True)

    def test_manual_recovery_request_timeline(self):
        text=self.fixture().replace('profile_id=test',
            'direct_event_schema=1 recovery_event_schema=2 profile_id=test')
        event='\nRAGDOLL_REPLAY_RECOVERY_REQUEST player=1 schema=1 integer_radix=10 next_step=11'
        _,context=self.extract(text+event)
        self.assertTrue(context.endswith('recovery_requests_v1 1\n1\n'))
        with self.assertRaisesRegex(ValueError,'Invalid recovery request timeline'):
            self.extract(text+event+event)

    def test_combat_event_is_rejected_until_interruption_is_replayed(self):
        with self.assertRaisesRegex(ValueError,'refusing to discard hit-stun'):
            self.extract(self.fixture()+'\nRAGDOLL_REPLAY_COMBAT_IMPULSE player=1 schema=1')

    def test_exact_contact_root_cycle_is_appended_and_validated(self):
        cycle = '\n'.join([
            'RAGDOLL_CONTACT_ROOT_CYCLE schema=1 frames=2 bodies=1 left_touchdown=0.1 right_touchdown=0.6 stride=0,2,0 stride_length=2 phase_rate=0.5',
            'RAGDOLL_CONTACT_ROOT_FRAME frame=0 stance=1 root_delta=0,0,0 vertex=0,0,0',
            'RAGDOLL_CONTACT_ROOT_POSE frame=0 body=0 position=0,0,75 rotation=0,0,0,1',
            'RAGDOLL_CONTACT_ROOT_FRAME frame=1 stance=2 root_delta=0,2,0 vertex=0,1,0',
            'RAGDOLL_CONTACT_ROOT_POSE frame=1 body=0 position=0,2,75 rotation=0,0,0,1',
        ])
        result, context = self.extract(self.fixture() + '\n' + cycle)
        self.assertEqual(result['contact_root_cycle'], 'captured adapted native cycle')
        self.assertTrue(context.endswith(
            'contact_root_cycle_v1 2 1 0.1 0.6 0 2 0 2 0.5\n'
            '1 0 0 0 0 0 0\n0 0 75 0 0 0 1\n'
            '2 0 2 0 0 1 0\n0 2 75 0 0 0 1\n'))
        with self.assertRaisesRegex(ValueError, 'Incomplete contact-root cycle'):
            self.extract(self.fixture() + '\n' + '\n'.join(cycle.splitlines()[:-1]))

    def test_complete_contact_root_input_timeline_is_appended_and_validated(self):
        inputs = '\n'.join(self.contact_input(step) for step in (10, 11))
        result, context = self.extract(self.fixture() + '\n' + inputs)
        self.assertEqual(result['contact_root_inputs'], 'captured complete native timeline')
        self.assertTrue(context.endswith(
            'contact_root_inputs_v1 2\n'
            '0 1 0.0166666675 1 0 1 1 1 0 1 2 0 0.25 0.5 0.6 0.4 0.1 0 0 1 0\n'
            '1 1 0.0166666675 1 0 1 1 1 0 1 2 0 0.25 0.5 0.6 0.4 0.1 0 0 1 0\n'))
        absent = 'RAGDOLL_CONTACT_ROOT_INPUT schema=1 integer_radix=10 step=10 present=0 player=1'
        _, absent_context = self.extract(self.fixture() + '\n' + absent + '\n' + self.contact_input(11))
        self.assertIn('contact_root_inputs_v1 2\n0 0\n1 1 ', absent_context)
        with self.assertRaisesRegex(ValueError, 'Incomplete contact-root input timeline'):
            self.extract(self.fixture() + '\n' + self.contact_input(10))
        with self.assertRaisesRegex(ValueError, 'Duplicate contact-root input step'):
            self.extract(self.fixture() + '\n' + inputs + '\n' + self.contact_input(10))
        with self.assertRaisesRegex(ValueError, 'Invalid contact-root input boolean'):
            self.extract(self.fixture() + '\n' + inputs.replace('interrupted=0', 'interrupted=2'))
        with self.assertRaisesRegex(ValueError, 'Non-finite'):
            self.extract(self.fixture() + '\n' + inputs.replace('left_slip=0.25', 'left_slip=nan'))


if __name__ == '__main__':
    unittest.main()
