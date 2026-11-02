import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('step_trace_report', Path(__file__).parents[1] / 'step_trace_report.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def row(step, phase, filtered, raw='0,0,0'):
    return (f'RAGDOLL_STEP integer_radix=10 actor=player player=1 step={step} phase={phase} '
            f'filtered={filtered} raw={raw} mode=0 substep=1')


class StepTraceTests(unittest.TestCase):
    def test_rejects_undeclared_or_hexadecimal_integer_fields(self):
        valid = row(85, 'input', '0,0,0')
        with self.assertRaisesRegex(ValueError, 'radix'):
            report.summarize([valid.replace('integer_radix=10 ', '')])
        with self.assertRaisesRegex(ValueError, 'Non-decimal'):
            report.summarize([valid.replace('step=85', 'step=8d')])
        with self.assertRaisesRegex(ValueError, 'radix'):
            report.summarize(['RAGDOLL_LEG_COMMAND actor=player body=14 step=85'])

    def test_travel_runs_do_not_bridge_missing_or_incomplete_physics_steps(self):
        lines = []
        for step in (0, 2):
            for phase in ('input', 'applied', 'observed'):
                lines.append(row(step, phase, '0,4,0', '0,4,0')
                             + ' measured=1 velocity=0,4,0 physics_dt=0.5')
        for middle in ([], [row(1, 'input', '0,0,0')]):
            r = report.summarize(lines + middle)['actors']['player:player=1']
            self.assertEqual(len(r['requested_travel_runs']), 2)
            self.assertEqual([v['velocity_integral_progress'] for v in r['requested_travel_runs']], [2, 2])
            self.assertEqual(r['gaps_between_complete_steps'], 1)

    def test_final_leg_report_rejects_cross_space_and_missing_step_comparisons(self):
        base = dict(actor='player', body='14', step='1', valid='1', motors='1',
                    stage='1', world='1', orientation_known='1', target='0,0,0,1')
        rows = [base, dict(base, step='2', stage='2', world='0', target='0,0,1,0'),
                dict(base, step='3', stage='2', world='0', target='0,0,-1,0'),
                dict(base, step='5', world='0')]
        r = report.leg_command_summary(rows)['joints']['player:body=14']
        self.assertEqual(r['incompatible_target_pairs'], 1)
        self.assertEqual(r['gaps_between_valid_rows'], 1)
        self.assertEqual(r['comparable_target_pairs'], 1)
        self.assertAlmostEqual(r['maximum_target_change']['radians'], 0)
        self.assertIsNone(r['maximum_stage_transition_target_change'])
        self.assertIsNone(r['maximum_native_limit_rejection'])

    def test_final_leg_report_keeps_hinge_jumps_and_limit_provenance(self):
        base = dict(actor='player', body='15', step='1', valid='1', motors='1',
                    stage='1', world='0', hinge_known='1', hinge_target='0.9', clamp='2.0')
        rows = [base, dict(base, step='2', stage='2', hinge_target='0.4'),
                dict(base, step='3', valid='0')]
        r = report.leg_command_summary(rows)['joints']['player:body=15']
        self.assertAlmostEqual(r['maximum_stage_transition_target_change']['radians'], 0.5)
        self.assertEqual(r['invalid_or_undriven_rows'], 1)
        self.assertEqual(r['native_limit_samples'], 0)
        rows[1].update(native_limit_known='1', native_limit_clamp='0.2')
        r = report.leg_command_summary(rows)['joints']['player:body=15']
        self.assertEqual(r['maximum_native_limit_rejection']['radians'], 0.2)
        with self.assertRaises(ValueError):
            report.leg_command_summary([base, base])
        with self.assertRaises(ValueError):
            report.leg_command_summary([dict(base, hinge_target='nan')])
        with self.assertRaises(ValueError):
            report.leg_command_summary([dict(base, hinge_known='0', orientation_known='1', target='0,0,0,0')])

    def test_contact_loads_preserve_time_boundary_units_and_unknown(self):
        applied = dict(measured='1', request_valid='1', support_valid='1',
                       physics_dt='0.5', filtered='0,4,0', velocity='0,2,0',
                       requested_accel='0,8,0', budgeted_accel='0,6,0',
                       left_load='0.1', right_load='1.2')
        after = dict(measured='1', velocity='0,3,0', left_load='0.8', right_load='0')
        result = report.propulsion_sample(7, applied, after)
        self.assertEqual(result['contact_load_body_weights_before'], {'left': 0.1, 'right': 1.2})
        self.assertEqual(result['contact_load_body_weights_after'], {'left': 0.8, 'right': 0})
        result = report.propulsion_sample(7, applied, dict(measured='1', velocity='0,3,0', left_load='-1'))
        self.assertEqual(result['contact_load_body_weights_after'], {'left': None, 'right': None})
        with self.assertRaises(ValueError):
            report.propulsion_sample(7, applied, dict(after, left_load='nan'))

    def test_travel_integral_distinguishes_backtracking_from_distance(self):
        lines = [row(0, phase, '0,4,0', '0,4,0') for phase in ('input', 'applied', 'observed')]
        lines[1] += ' measured=1 velocity=20,-2,0 physics_dt=0.5'
        lines[2] += ' measured=1 velocity=20,-4,0'
        run = report.summarize(lines)['actors']['player:player=1']['requested_travel_runs'][0]
        self.assertEqual(run['velocity_integral_progress'], -1.5)
        self.assertEqual(run['measured_steps'], 1)
        self.assertEqual(run['missing_steps'], 0)
        lines[2] = lines[2].replace('measured=1', 'measured=0')
        missing = report.summarize(lines)['actors']['player:player=1']['requested_travel_runs'][0]
        self.assertEqual(missing['measured_steps'], 0)
        self.assertEqual(missing['missing_steps'], 1)

    def test_stationary_braking_uses_recorded_facing_and_keeps_world_vectors(self):
        applied = dict(measured='1', request_valid='1', support_valid='1',
                       physics_dt='0.5', filtered='0,0,0', facing='1,0,0',
                       velocity='10,2,0', requested_accel='-8,3,0', budgeted_accel='-6,1,0')
        after = dict(measured='1', velocity='9,3,0')
        result = report.propulsion_sample(7, applied, after)
        self.assertEqual(result['axis_source'], 'facing')
        self.assertEqual(result['requested_forward_accel'], -8)
        self.assertEqual(result['budgeted_forward_accel'], -6)
        self.assertEqual(result['net_acceleration'], [-2, 2, 0])
        self.assertIsNone(report.propulsion_sample(7, dict(applied, facing='0,0,0'), after))
        with self.assertRaises(ValueError):
            report.propulsion_sample(7, dict(applied, facing='nan,0,0'), after)

    def test_propulsion_uses_pre_update_request_and_same_step_response(self):
        applied = dict(measured='1', request_valid='1', support_valid='1', support_applied='1',
                       physics_dt='0.5', filtered='0,4,0', velocity='10,2,0',
                       requested_accel='0,8,0', budgeted_accel='0,6,0')
        after = dict(measured='1', velocity='9,3,0', requested_accel='0,999,0')
        result = report.propulsion_sample(7, applied, after)
        self.assertEqual(result['requested_forward_accel'], 8)
        self.assertEqual(result['budgeted_forward_accel'], 6)
        self.assertEqual(result['net_forward_accel'], 2)
        self.assertEqual(result['forward_speed_after'], 3)
        self.assertIsNone(result['hull_projection_distance'])
        self.assertIsNone(result['friction_limited'])
        geometry = dict(applied, hull_known='1', hull_delta='3,4', friction_limited='0')
        projected = report.propulsion_sample(7, geometry, after)
        self.assertEqual(projected['hull_projection_distance'], 5)
        self.assertFalse(projected['friction_limited'])
        with self.assertRaises(ValueError):
            report.propulsion_sample(7, dict(geometry, hull_delta='nan,0'), after)
        self.assertIsNone(report.propulsion_sample(7, {}, after))
        for invalid in ('nan', '0', '-1'):
            with self.assertRaises(ValueError):
                report.propulsion_sample(7, dict(applied, physics_dt=invalid), after)
        with self.assertRaises(ValueError):
            report.propulsion_sample(7, applied, dict(after, velocity='0,nan,0'))

    def test_expected_post_step_update_is_not_a_discontinuity(self):
        lines = [row(0, 'input', '0,0,0'), row(0, 'applied', '0,0,0'), row(0, 'observed', '0,1,0'),
                 row(1, 'input', '0,1,0', '0,2,0'), row(1, 'applied', '0,1,0'), row(1, 'observed', '0,2,0')]
        result = report.summarize(lines)['actors']['player:player=1']
        self.assertEqual(result['complete_steps'], 2)
        self.assertEqual(result['first_filter_update_after_physics'], 0)
        self.assertEqual(result['first_nonzero_raw_step'], 1)
        self.assertEqual(result['first_nonzero_applied_filtered_step'], 1)
        self.assertIsNone(result['first_filter_discontinuity_between_steps'])
        self.assertIsNone(result['first_filter_change_during_command_application'])
        lines[3] = row(1, 'input', '0,9,0')
        result = report.summarize(lines)['actors']['player:player=1']
        self.assertEqual(result['first_filter_discontinuity_between_steps'], 1)
        self.assertEqual(result['first_filter_change_during_command_application'], 1)

    def test_missing_and_duplicate_phases_are_visible(self):
        line = row(0, 'input', '0,0,0')
        self.assertEqual(report.summarize([line])['actors']['player:player=1']['incomplete_steps'], [0])
        with self.assertRaises(ValueError):
            report.summarize([line, line])

    def test_coordinated_pose_counts_use_applied_boundary_and_preserve_unknown(self):
        lines = [row(0, phase, '0,1,0') for phase in ('input', 'applied', 'observed')]
        result = report.summarize(lines)['actors']['player:player=1']
        self.assertIsNone(result['coordinated_pose_applied_steps'])
        lines[0] += ' coordinated_pose=1'
        lines[1] += ' coordinated_pose=0'
        result = report.summarize(lines)['actors']['player:player=1']
        self.assertEqual(result['coordinated_pose_applied_steps'], 0)
        lines[1] = lines[1].replace('coordinated_pose=0', 'coordinated_pose=1')
        result = report.summarize(lines)['actors']['player:player=1']
        self.assertEqual(result['coordinated_pose_applied_steps'], 1)
        self.assertEqual(result['first_coordinated_pose_step'], 0)

    def test_landing_rebase_measurements_are_not_assumed_present(self):
        self.assertIsNone(report.summarize([])['maximum_landing_rebase_world_error'])
        result = report.summarize(['RAGDOLL_LANDING_REBASE world_error=0.000003',
                                   'RAGDOLL_LANDING_REBASE world_error=0.000002'])
        self.assertEqual(result['landing_rebase_samples'], 2)
        self.assertEqual(result['maximum_landing_rebase_world_error'], 0.000003)
        with self.assertRaises(ValueError):
            report.summarize(['RAGDOLL_LANDING_REBASE world_error=nan'])


if __name__ == '__main__':
    unittest.main()
