"""Replay episode schema 1 against raw per-step meter samples; no stride-axis claim."""
import copy
import math
import re
import shlex
import struct


def _float32(value):
    try:
        return struct.unpack('f', struct.pack('f', value))[0]
    except OverflowError:
        return math.inf


def _valid_slip(row, side, duration):
    speed_key = side + '_maximum_tangent_speed'
    distance_key = side + '_maximum_tangent_distance_step'
    if row[speed_key + '_known'] != '1' or row[distance_key + '_known'] != '1':
        return None
    speed, distance = float(row[speed_key]), float(row[distance_key])
    if not all(math.isfinite(value) and value >= 0 for value in (speed, distance)):
        return None
    expected = _float32(_float32(speed) * _float32(duration))
    if not math.isfinite(expected):
        return None
    # An exact zero product cannot acquire a nonzero travel allowance.
    if expected == 0:
        return distance if distance == 0 else None
    bits = struct.unpack('I', struct.pack('f', expected))[0]
    next_value = struct.unpack('f', struct.pack('I', bits + 1))[0]
    return distance if abs(distance - expected) <= 2 * (next_value - expected) else None


def _foot():
    return {'availability': 'unknown', 'current': None, 'unknown_steps': 0, 'unknown_duration': 0.0}


def _close(segment, ending):
    if segment is None:
        return None
    closed = copy.copy(segment)
    closed['end'] = ending
    return closed


def _segment_fields(prefix, segment):
    result = {prefix + '_known': int(segment is not None)}
    if segment is None:
        return result
    complete = segment['onset'] and segment['end'] == 'unsupported'
    result.update({prefix + '_' + key: value for key, value in segment.items() if key != 'travel'})
    result[prefix + '_travel_known'] = int(segment['travel'] is not None)
    if segment['travel'] is not None:
        result[prefix + '_travel'] = segment['travel']
    result[prefix + '_episode_complete'] = int(complete)
    result[prefix + '_episode_travel_complete'] = int(complete and segment['slip_complete'])
    return result


def _advance(actor, row, context):
    step, epoch = int(row['step']), int(row['epoch'])
    if epoch != context['epoch']:
        raise ValueError('sample epoch has no matching lifecycle reset')
    last = context['last']
    if last is not None and step <= last:
        raise ValueError('duplicate or older episode physics step')
    duration = float(row['dt'])
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('invalid episode duration')
    missing = step - last - 1 if last is not None else 0
    context['missing'] += missing
    expected = {}
    for side in ('left', 'right'):
        foot = context[side]
        closed = None
        if missing:
            closed = _close(foot['current'], 'missing-steps')
            foot['current'] = None
            foot['availability'] = 'unknown'
        availability = row[side + '_support']
        if availability not in ('supported', 'unsupported', 'unknown'):
            raise ValueError('invalid raw support availability')
        if availability == 'supported':
            if foot['current'] is None:
                foot['current'] = {'actor': actor, 'epoch': epoch, 'first': step, 'last': step,
                    'onset': int(last is not None and foot['availability'] == 'unsupported'), 'end': 'open',
                    'support_steps': 0, 'support_duration': 0.0, 'slip_steps': 0, 'slip_duration': 0.0,
                    'travel': None, 'slip_complete': 1}
            current = foot['current']
            current['last'] = step
            current['support_steps'] += 1
            current['support_duration'] += duration
            distance = _valid_slip(row, side, duration)
            if distance is None:
                current['slip_complete'] = 0
            else:
                current['slip_steps'] += 1
                current['slip_duration'] += duration
                current['travel'] = (current['travel'] or 0.0) + distance
        else:
            if foot['current'] is not None:
                closed = _close(foot['current'], 'unsupported' if availability == 'unsupported' else 'unknown-support')
            foot['current'] = None
            if availability == 'unknown':
                foot['unknown_steps'] += 1
                foot['unknown_duration'] += duration
        foot['availability'] = availability
        output = {'schema': 1, 'actor_serial': actor, 'step': step, 'epoch': epoch, 'foot': side,
            'status': 'consumed', 'missing_steps': missing, 'total_missing_steps': context['missing'],
            'support': availability, 'unknown_steps': foot['unknown_steps'],
            'unknown_duration': foot['unknown_duration']}
        output.update(_segment_fields('current', foot['current']))
        output.update(_segment_fields('closed', closed))
        output.update(_segment_fields('invalidated', context['pending'][side]))
        expected[(actor, step, side)] = output
        context['pending'][side] = None
    context['last'] = step
    return expected


def _compare(actual, expected):
    if actual.keys() != expected.keys():
        raise ValueError('episode field presence mismatch: '
            + repr(sorted(actual.keys() ^ expected.keys())))
    for key, value in expected.items():
        if isinstance(value, float):
            observed = float(actual[key])
            if not math.isfinite(observed) or not math.isclose(observed, value, rel_tol=1e-10, abs_tol=1e-12):
                raise ValueError(f'episode {key} differs from observed raw samples')
        elif str(value) != actual[key]:
            raise ValueError(f'episode {key} differs from observed raw samples')


def audit_episodes(text):
    contexts, expected, observed, violations = {}, {}, set(), []
    sample_count, reset_count = 0, 0
    pattern = re.compile(r'\b(RAGDOLL_BALANCE(?:_EPISODE(?:_RESET)?)?) schema=([12])\b')
    for number, line in enumerate(text.splitlines(), 1):
        match = pattern.search(line)
        if not match:
            continue
        try:
            tokens = shlex.split(line[match.start():])
            pairs = [token.split('=', 1) for token in tokens[1:]]
            row = dict(pairs)
            if len(row) != len(pairs):
                raise ValueError('duplicate episode field')
            kind = tokens[0]
            actor = int(row['actor_serial'])
            if kind == 'RAGDOLL_BALANCE_EPISODE_RESET':
                if row['schema'] != '1':
                    raise ValueError('unsupported reset schema')
                old = contexts.get(actor)
                previous = {'previous_epoch_known': int(old is not None),
                    'last_step_known': int(old is not None and old['last'] is not None)}
                if old is not None:
                    previous['previous_epoch'] = old['epoch']
                if old is not None and old['last'] is not None:
                    previous['last_step'] = old['last']
                actual_previous = {key: value for key, value in row.items()
                    if key.startswith('previous_epoch') or key.startswith('last_step')}
                _compare(actual_previous, previous)
                epoch = int(row['epoch'])
                if old is not None and epoch <= old['epoch']:
                    raise ValueError('same or older lifecycle reset')
                pending = copy.deepcopy(old['pending']) if old else {'left': None, 'right': None}
                if old:
                    for side in ('left', 'right'):
                        if old[side]['current']:
                            pending[side] = _close(old[side]['current'], 'lifecycle-invalidated')
                contexts[actor] = {'epoch': epoch, 'last': None, 'missing': 0,
                    'left': _foot(), 'right': _foot(), 'pending': pending}
                reset_count += 1
            elif kind == 'RAGDOLL_BALANCE':
                if row.get('episode_schema') != '1':
                    continue
                if actor not in contexts:
                    raise ValueError('episode sample lacks explicit generation binding')
                generated = _advance(actor, row, contexts[actor])
                if expected.keys() & generated.keys():
                    raise ValueError('duplicate episode sample')
                expected.update(generated)
                sample_count += 1
            else:
                key = (actor, int(row['step']), row['foot'])
                if key in observed:
                    raise ValueError('duplicate episode detail')
                if key not in expected:
                    raise ValueError('orphan or premature episode detail')
                _compare(row, expected[key])
                observed.add(key)
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            violations.append(f'episode line {number}: {error}')
    for key in expected.keys() - observed:
        violations.append(f'episode actor {key[0]} step {key[1]} foot {key[2]}: missing valid episode row')
    if not sample_count:
        violations.append('no episode-enabled balance samples')
    return {'sample_count': sample_count, 'reset_count': reset_count,
        'violation_count': len(violations), 'violations': violations, 'success': not violations,
        'scope': 'observed-support-segments-and-conservative-tangent-travel-only'}
