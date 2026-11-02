"""Audit immutable actor-queue observations; no protocol-axis or quality verdict.

The existing BAL meter auditor calls this module. Historical logs without an
intent schema marker remain historical: zero intent coverage is explicit.
"""
from __future__ import annotations

import math
import re
import shlex

IDS = ('capture_serial', 'capture_actor_serial', 'capture_epoch',
       'capture_render_frame', 'first_possible_step', 'consumed_step')
VECS = ('queued_world_velocity', 'processed_raw_horizontal_velocity', 'filtered_command_velocity')
KNOWN = 'actor-queue-before-processing'
UNKNOWN = {'uncaptured', 'lifecycle-invalidated', 'passive-no-locomotion-capture',
           'invalid-queued-velocity', 'actor-queue-unavailable', 'capture-serial-exhausted',
           'capture-identity-mismatch', 'snapshot-identity-mismatch'}


def uint(value):
    if not re.fullmatch(r'0|[1-9][0-9]*', value) or int(value) > 2**64 - 1:
        raise ValueError('invalid unsigned identifier')
    return int(value)


def boolean(value):
    if value not in ('0', '1'):
        raise ValueError('invalid boolean')
    return value == '1'


def vec(value):
    result = tuple(map(float, value.split(',')))
    if len(result) != 3 or not all(math.isfinite(x) for x in result):
        raise ValueError('invalid intent vector')
    return result


def opt(row, name, parse):
    known = boolean(row[name + '_known'])
    if known != (name in row):
        raise ValueError('intent Known/value mismatch: ' + name)
    return parse(row[name]) if known else None


def validate_intent(row, main):
    parsed = {name: opt(row, name, uint) for name in IDS}
    parsed.update({name: opt(row, name, vec) for name in VECS})
    parsed['locomotion_interrupted'] = opt(row, 'locomotion_interrupted', boolean)
    if row['availability'] == KNOWN:
        if any(parsed[name] is None for name in IDS + ('queued_world_velocity', 'locomotion_interrupted')):
            raise ValueError('known queue lacks capture identity/value')
        if not boolean(main['powered']):
            raise ValueError('passive actor fabricated queued intent')
        if parsed['capture_serial'] == 0:
            raise ValueError('capture serial must be positive')
        if parsed['capture_actor_serial'] != uint(main['actor_serial']) or parsed['capture_epoch'] != uint(main['epoch']):
            raise ValueError('queue capture belongs to another actor/lifecycle')
        if parsed['consumed_step'] != uint(main['step']):
            raise ValueError('consumed intent step differs from completed step')
        if parsed['first_possible_step'] > parsed['consumed_step']:
            raise ValueError('queue capture belongs to a future step')
        if parsed['capture_render_frame'] > 2**32 - 1:
            raise ValueError('capture render frame exceeds native unsigned')
    else:
        if row['availability'] not in UNKNOWN:
            raise ValueError('unknown intent availability')
        if any(value is not None for value in parsed.values()):
            raise ValueError('unavailable intent retained fabricated/stale fields')
    if not boolean(main['powered']) and row['availability'] != 'passive-no-locomotion-capture':
        raise ValueError('passive intent reason is not explicit')
    return parsed


def audit_intents(text):
    mains, rows, violations, strikes = {}, {}, [], []
    for line_number, line in enumerate(text.splitlines(), 1):
        match = re.search(r'\b(RAGDOLL_BALANCE(?:_INTENT)?|RAGDOLL_PHYSICAL_STRIKE)(?=\s|$)', line)
        if not match:
            continue
        try:
            parts = shlex.split(line[match.start():])
            row = {}
            for token in parts[1:]:
                name, value = token.split('=', 1)
                if name in row:
                    raise ValueError('duplicate trace field ' + name)
                row[name] = value
            if parts[0] == 'RAGDOLL_PHYSICAL_STRIKE':
                if row.get('applied') == '1' and 'victim_serial' in row:
                    strikes.append(((uint(row['victim_serial']), uint(row['step'])), line_number))
                continue
            key = (uint(row['actor_serial']), uint(row['step']))
            uint(row['epoch'])
            if parts[0] == 'RAGDOLL_BALANCE':
                if row['schema'] != '2':
                    raise ValueError('unsupported main schema')
                target = mains
            else:
                if row['schema'] != '1':
                    raise ValueError('unsupported intent schema')
                target = rows
            if key in target:
                raise ValueError('duplicate main/intent row')
            target[key] = (row, line_number)
        except (ValueError, KeyError) as error:
            violations.append(f'line {line_number}: {error}')
    previous = {}
    invalidated_capture = set()
    captured = reused = unavailable = 0
    # Insertion order follows publication order, not sorted identifiers.
    for key, (main, main_line) in mains.items():
        try:
            marker = main.get('intent_schema')
            if marker is None:
                if key in rows:
                    raise ValueError('intent row has no main schema marker')
                continue
            if marker != '1':
                raise ValueError('unsupported intent_schema marker')
            if key not in rows:
                raise ValueError('missing intent detail row')
            row, line = rows[key]
            if line <= main_line:
                raise ValueError('intent published before completed main snapshot')
            if row['epoch'] != main['epoch']:
                raise ValueError('intent detail lifecycle mismatch')
            value = validate_intent(row, main)
            if value['capture_serial'] is None:
                unavailable += 1
                invalidated_capture.add(key[0])
                continue
            captured += 1
            immutable = tuple(value[name] for name in ('capture_actor_serial', 'capture_epoch',
                'capture_render_frame', 'first_possible_step', 'queued_world_velocity'))
            last = previous.get(key[0])
            if last:
                serial, payload, last_step = last
                if key[1] <= last_step:
                    raise ValueError('nonmonotonic observed intent step')
                if value['capture_serial'] < serial:
                    raise ValueError('capture serial moved backwards')
                if value['capture_serial'] == serial:
                    if key[0] in invalidated_capture:
                        raise ValueError('unavailable gap reused retired capture')
                    if immutable != payload:
                        raise ValueError('reused capture changed immutable payload/lifecycle')
                    reused += 1
                elif value['first_possible_step'] < payload[3]:
                    raise ValueError('new capture first-possible step moved backwards')
            previous[key[0]] = (value['capture_serial'], immutable, key[1])
            invalidated_capture.discard(key[0])
        except (ValueError, KeyError, TypeError) as error:
            violations.append(f'actor {key[0]} step {key[1]}: {error}')
    for key in rows.keys() - mains.keys():
        violations.append(f'actor {key[0]} step {key[1]}: orphan intent row')
    matched_strikes = 0
    for key, line in strikes:
        if key in mains and mains[key][0].get('intent_schema') == '1':
            if key not in rows or rows[key][1] >= line:
                violations.append(f'actor {key[0]} step {key[1]}: victim intent missing/after applied strike')
            else:
                matched_strikes += 1
    return {'sample_count': len(rows), 'captured_sample_count': captured,
            'unavailable_sample_count': unavailable, 'reused_capture_step_count': reused,
            'matched_applied_victim_strike_count': matched_strikes,
            'coverage': 'present' if rows else 'not-present',
            'protocol_axis_acknowledgment': 'not-observed',
            'violations': violations, 'success': not violations}
