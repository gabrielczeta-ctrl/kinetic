import unittest

from balance_episode_trace import audit_episodes


def line(kind, fields):
    return kind + ' ' + ' '.join(f'{key}={value}' for key, value in fields.items())


def reset(epoch=0, previous=None, last=None):
    values = {'schema': 1, 'actor_serial': 7, 'epoch': epoch, 'previous_epoch_known': int(previous is not None),
        'last_step_known': int(last is not None), 'reason': 'fixture'}
    if previous is not None:
        values['previous_epoch'] = previous
    if last is not None:
        values['last_step'] = last
    return line('RAGDOLL_BALANCE_EPISODE_RESET', values)


def sample(step, support='supported', epoch=0, distance=.25):
    values = {'schema': 2, 'episode_schema': 1, 'actor_serial': 7, 'step': step, 'epoch': epoch, 'dt': .125}
    for side, availability in (('left', support), ('right', 'unsupported')):
        known = availability == 'supported' and distance is not None
        values.update({side + '_support': availability, side + '_maximum_tangent_speed_known': int(known),
            side + '_maximum_tangent_distance_step_known': int(known)})
        if known:
            values[side + '_maximum_tangent_speed'] = 2
            values[side + '_maximum_tangent_distance_step'] = distance
    return line('RAGDOLL_BALANCE', values)


def segment(first, last, onset, end='open', epoch=0, measured=None):
    count = last - first + 1
    if measured is None:
        measured = count
    return {'actor': 7, 'epoch': epoch, 'first': first, 'last': last, 'onset': int(onset), 'end': end,
        'support_steps': count, 'support_duration': .125 * count, 'slip_steps': measured,
        'slip_duration': .125 * measured, 'travel_known': int(measured > 0),
        **({'travel': .25 * measured} if measured else {}), 'slip_complete': int(count == measured),
        'episode_complete': int(onset and end == 'unsupported'),
        'episode_travel_complete': int(onset and end == 'unsupported' and count == measured)}


def row(step, foot='left', support='supported', epoch=0, current=None, closed=None, invalidated=None,
        unknown=0, missing=0, total_missing=0):
    values = {'schema': 1, 'actor_serial': 7, 'step': step, 'epoch': epoch, 'foot': foot, 'status': 'consumed',
        'missing_steps': missing, 'total_missing_steps': total_missing, 'support': support,
        'unknown_steps': unknown, 'unknown_duration': unknown * .125}
    for name, value in (('current', current), ('closed', closed), ('invalidated', invalidated)):
        values[name + '_known'] = int(value is not None)
        if value is not None:
            values.update({name + '_' + key: item for key, item in value.items()})
    return line('RAGDOLL_BALANCE_EPISODE', values)


def rows(step, support='supported', epoch=0, current=None, closed=None, invalidated=None,
        unknown=0, missing=0, total_missing=0, distance=.25):
    return [sample(step, support, epoch, distance), row(step, support=support, epoch=epoch, current=current,
        closed=closed, invalidated=invalidated, unknown=unknown, missing=missing, total_missing=total_missing),
        row(step, foot='right', support='unsupported', epoch=epoch, missing=missing, total_missing=total_missing)]


class BalanceEpisodeTraceTest(unittest.TestCase):
    def assertValid(self, lines):
        result = audit_episodes('\n'.join(lines))
        self.assertTrue(result['success'], result)

    def assertInvalid(self, lines, reason):
        result = audit_episodes('\n'.join(lines))
        self.assertFalse(result['success'])
        self.assertTrue(any(reason in value for value in result['violations']), result)

    def test_complete_episode_reconstructs_duration_and_travel(self):
        lines = [reset()] + rows(1, 'unsupported') + rows(2, current=segment(2, 2, True))
        lines += rows(3, current=segment(2, 3, True))
        lines += rows(4, 'unsupported', closed=segment(2, 3, True, 'unsupported'))
        self.assertValid(lines)
        self.assertInvalid([value.replace('closed_travel=0.5', 'closed_travel=1.0') for value in lines], 'closed_travel')

    def test_unknown_closes_partial_without_new_onset(self):
        lines = [reset()] + rows(1, 'unsupported') + rows(2, current=segment(2, 2, True))
        lines += rows(3, 'unknown', closed=segment(2, 2, True, 'unknown-support'), unknown=1)
        lines += rows(4, current=segment(4, 4, False), unknown=1)
        self.assertValid(lines)
        self.assertInvalid([value.replace('current_first=4 current_last=4 current_onset=0',
            'current_first=4 current_last=4 current_onset=1') for value in lines], 'current_onset')

    def test_missing_ids_do_not_extrapolate_duration(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False))
        lines += rows(4, current=segment(4, 4, False), closed=segment(1, 1, False, 'missing-steps'),
            missing=2, total_missing=2)
        self.assertValid(lines)
        self.assertInvalid([value.replace('current_support_duration=0.125', 'current_support_duration=0.375')
            for value in lines], 'current_support_duration')

    def test_generation_reset_retains_prior_partial_only_as_invalidated(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False)) + [reset(1, 0, 1)]
        lines += rows(2, epoch=1, current=segment(2, 2, False, epoch=1),
            invalidated=segment(1, 1, False, 'lifecycle-invalidated'))
        self.assertValid(lines)
        self.assertInvalid([value.replace('invalidated_epoch=0', 'invalidated_epoch=1') for value in lines],
            'invalidated_epoch')

    def test_repeated_resets_preserve_first_pending_closure_and_erase_needs_no_new_sample(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False)) + [reset(1, 0, 1), reset(2, 1)]
        self.assertValid(lines)
        lines += rows(3, 'unknown', epoch=2, unknown=1,
            invalidated=segment(1, 1, False, 'lifecycle-invalidated'))
        self.assertValid(lines)
        self.assertInvalid(lines + [reset(2, 2, 3)], 'same or older')

    def test_missing_slip_is_incomplete_not_zero(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False, measured=0), distance=None)
        self.assertValid(lines)
        self.assertInvalid([value.replace('current_travel_known=0', 'current_travel_known=1 current_travel=0')
            for value in lines], 'field presence')

    def test_duplicate_or_missing_rows_and_stale_steps_fail(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False))
        self.assertInvalid(lines + [lines[2]], 'duplicate episode detail')
        self.assertInvalid(lines[:2] + lines[3:], 'missing valid episode row')
        self.assertInvalid(lines + [lines[1]], 'duplicate or older')
        self.assertInvalid(lines[1:], 'explicit generation binding')

    def test_near_zero_false_travel_is_not_complete(self):
        lines = [reset()] + rows(1, current=segment(1, 1, False, measured=0), distance=9e-7)
        lines[1] = lines[1].replace('left_maximum_tangent_speed=2', 'left_maximum_tangent_speed=0')
        self.assertValid(lines)
        forged = segment(1, 1, False)
        forged['travel'] = 9e-7
        lines[2] = row(1, current=forged)
        self.assertInvalid(lines, 'field presence')


if __name__ == '__main__':
    unittest.main()
