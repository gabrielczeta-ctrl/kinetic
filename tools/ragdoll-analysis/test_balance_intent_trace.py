import unittest

from balance_intent_trace import IDS, VECS, audit_intents


def line(kind, row):
    return kind + ' ' + ' '.join(f'{key}={value}' for key, value in row.items())


def fixture(step=10, serial=1, actor=7, epoch=0, known=True, powered=True, **changes):
    main = dict(schema='2', actor_serial=str(actor), step=str(step), epoch=str(epoch),
                powered=str(int(powered)), intent_schema='1')
    row = dict(schema='1', actor_serial=str(actor), step=str(step), epoch=str(epoch),
               availability='actor-queue-before-processing' if known else
               'uncaptured' if powered else 'passive-no-locomotion-capture')
    values = dict(capture_serial=str(serial), capture_actor_serial=str(actor), capture_epoch=str(epoch),
        capture_render_frame='2', first_possible_step='10', consumed_step=str(step),
        queued_world_velocity='3,4,5', processed_raw_horizontal_velocity='3,4,0',
        filtered_command_velocity='2,2,0', locomotion_interrupted='0')
    for field in IDS + VECS + ('locomotion_interrupted',):
        row[field + '_known'] = str(int(known))
        if known:
            row[field] = values[field]
    row.update(changes)
    return main, row


def render(pair):
    return line('RAGDOLL_BALANCE', pair[0]) + '\n' + line('RAGDOLL_BALANCE_INTENT', pair[1])


class BalanceIntentTraceTest(unittest.TestCase):
    def check(self, text, success=True):
        result = audit_intents(text)
        self.assertEqual(result['success'], success, result)
        return result

    def test_reuse_preserves_queue_allows_changed_filtered_command(self):
        result = self.check(render(fixture()) + '\n' + render(fixture(11, filtered_command_velocity='3,4,0')))
        self.assertEqual(result['reused_capture_step_count'], 1)

    def test_latest_capture_can_skip_render_samples_at_same_first_step(self):
        self.check(render(fixture()) + '\n' + render(fixture(11, serial=4, queued_world_velocity='-8,0,0')))

    def test_known_zero_distinct_from_uncaptured_and_passive(self):
        zero = self.check(render(fixture(queued_world_velocity='0,0,0')))
        missing = self.check(render(fixture(known=False)))
        passive = self.check(render(fixture(known=False, powered=False)))
        self.assertEqual((zero['captured_sample_count'], missing['unavailable_sample_count'],
                          passive['unavailable_sample_count']), (1, 1, 1))

    def test_pre_mask_queue_survives_zero_interrupted_command(self):
        self.check(render(fixture(locomotion_interrupted='1', processed_raw_horizontal_velocity='0,0,0')))

    def test_reused_capture_cannot_change_vector_frame_epoch_or_first_step(self):
        for field, value in [('queued_world_velocity', '0,0,0'), ('capture_render_frame', '3'),
                             ('first_possible_step', '11')]:
            with self.subTest(field=field):
                self.check(render(fixture()) + '\n' + render(fixture(11, **{field: value})), False)
        self.check(render(fixture()) + '\n' + render(fixture(11, epoch=1)), False)

    def test_lifecycle_invalidates_and_requires_fresh_serial(self):
        gap = fixture(11, epoch=1, known=False, availability='lifecycle-invalidated')
        self.check(render(fixture()) + '\n' + render(gap) + '\n' + render(fixture(12, serial=2, epoch=1)))
        self.check(render(fixture()) + '\n' + render(gap) + '\n' + render(fixture(12, epoch=1)), False)

    def test_unknown_cannot_retain_stale_values(self):
        self.check(render(fixture(known=False, queued_world_velocity='0,0,0')), False)
        pair = fixture(known=False)
        pair[1]['capture_serial_known'] = '1'
        self.check(render(pair), False)

    def test_invalid_capture_reasons_are_truthful_absent_observations(self):
        for reason in ('invalid-queued-velocity', 'actor-queue-unavailable', 'capture-identity-mismatch'):
            self.check(render(fixture(known=False, availability=reason)))

    def test_invalid_capture_gap_cannot_resume_old_serial(self):
        gap = fixture(11, known=False, availability='invalid-queued-velocity')
        self.check(render(fixture()) + '\n' + render(gap) + '\n' + render(fixture(12)), False)
        self.check(render(fixture()) + '\n' + render(gap) + '\n' + render(fixture(12, serial=2)))

    def test_independent_actors_can_have_same_entry_local_capture_serial(self):
        result = self.check(render(fixture()) + '\n' + render(fixture(actor=8, queued_world_velocity='-1,0,0')))
        self.assertEqual(result['captured_sample_count'], 2)
        self.check(render(fixture(capture_actor_serial='8')), False)

    def test_step_identity_and_capture_epoch_reject_mismatch(self):
        for fields in ({'consumed_step': '9'}, {'first_possible_step': '11'}, {'capture_epoch': '1'}):
            self.check(render(fixture(**fields)), False)

    def test_duplicate_missing_orphan_and_unmarked_rows_fail(self):
        pair = fixture()
        self.check(render(pair) + '\n' + line('RAGDOLL_BALANCE_INTENT', pair[1]), False)
        self.check(line('RAGDOLL_BALANCE', pair[0]), False)
        self.check(line('RAGDOLL_BALANCE_INTENT', pair[1]), False)
        del pair[0]['intent_schema']
        self.check(render(pair), False)

    def test_published_after_main_and_before_applied_victim_strike(self):
        pair = fixture()
        strike = 'RAGDOLL_PHYSICAL_STRIKE applied=1 victim_serial=7 step=10'
        result = self.check(render(pair) + '\n' + strike)
        self.assertEqual(result['matched_applied_victim_strike_count'], 1)
        self.check(line('RAGDOLL_BALANCE', pair[0]) + '\n' + strike + '\n'
                   + line('RAGDOLL_BALANCE_INTENT', pair[1]), False)
        self.check(line('RAGDOLL_BALANCE_INTENT', pair[1]) + '\n' + line('RAGDOLL_BALANCE', pair[0]), False)

    def test_absent_strikes_and_intent_coverage_are_not_claimed(self):
        result = self.check(render(fixture()))
        self.assertEqual(result['matched_applied_victim_strike_count'], 0)
        pair = fixture()
        del pair[0]['intent_schema']
        result = self.check(line('RAGDOLL_BALANCE', pair[0]))
        self.assertEqual((result['sample_count'], result['coverage']), (0, 'not-present'))
        self.assertEqual(result['protocol_axis_acknowledgment'], 'not-observed')

    def test_uint64_exactness_and_invalid_identifiers(self):
        self.check(render(fixture(actor=2**63 + 1, serial=2**63 + 2)))
        for value in ('-1', '1.5', '01', str(2**64)):
            self.check(render(fixture(capture_serial=value)), False)

    def test_invalid_vectors_flags_and_schema(self):
        for fields in ({'queued_world_velocity': 'nan,0,0'}, {'queued_world_velocity': '1,2'},
                       {'schema': '2'}, {'locomotion_interrupted': '2'},
                       {'queued_world_velocity_known': '2'}):
            self.check(render(fixture(**fields)), False)

    def test_unknown_processed_value_preserves_known_queue(self):
        pair = fixture()
        pair[1]['filtered_command_velocity_known'] = '0'
        del pair[1]['filtered_command_velocity']
        self.check(render(pair))

    def test_capture_serial_and_first_possible_step_never_regress(self):
        self.check(render(fixture(serial=2)) + '\n' + render(fixture(11, serial=1)), False)
        self.check(render(fixture()) + '\n' + render(fixture(11, serial=2, first_possible_step='9')), False)


if __name__ == '__main__':
    unittest.main()
