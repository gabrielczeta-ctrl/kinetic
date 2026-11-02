"""Audit bounded RAGDOLL_STEP logs without replaying or changing physics."""
import argparse
import json
import math
import re
from pathlib import Path


def leg_command_summary(rows):
    """Compare final commands only across contiguous, compatible observations."""
    joints = {}
    def number(fields, key):
        value = float(fields[key])
        if not math.isfinite(value):
            raise ValueError('Nonfinite leg command: ' + key)
        return value

    def rotation(fields):
        q = [float(v) for v in fields['target'].split(',')]
        if len(q) != 4 or not all(math.isfinite(v) for v in q):
            raise ValueError('Invalid leg target quaternion')
        norm = math.sqrt(sum(v * v for v in q))
        if abs(norm - 1) > 0.001:
            raise ValueError('Leg target quaternion is not normalized')
        return [v / norm for v in q]

    for fields in rows:
        key = fields['actor'] + ':body=' + fields['body']
        frames = joints.setdefault(key, {})
        step = int(fields['step'])
        if step in frames:
            raise ValueError('Duplicate leg actor/body/step; identity is ambiguous')
        frames[step] = fields
    result = {}
    for key, frames in joints.items():
        previous = None
        changes = []
        incompatible = gaps = invalid = 0
        limits = []
        for step, fields in sorted(frames.items()):
            if fields.get('valid') != '1' or fields.get('motors') != '1':
                invalid += 1
                previous = None
                continue
            if fields.get('native_limit_known') == '1':
                value = number(fields, 'native_limit_clamp')
                if value < 0:
                    raise ValueError('Negative native limit rejection')
                limits.append({'step': step, 'stage': fields.get('stage'), 'radians': value})
            kind = 'hinge' if fields.get('hinge_known') == '1' else (
                'orientation' if fields.get('orientation_known') == '1' else None)
            if kind == 'orientation' and fields.get('world') not in ('0', '1'):
                kind = None
            value = number(fields, 'hinge_target') if kind == 'hinge' else rotation(fields) if kind else None
            if previous:
                old_step, old, old_kind, old_value = previous
                if step != old_step + 1:
                    gaps += 1
                elif kind is None or old_kind != kind or old.get('world') != fields.get('world'):
                    incompatible += 1
                else:
                    # q and -q are the same orientation. Never compare a world
                    # target directly with a constraint-space target.
                    delta = abs(value - old_value) if kind == 'hinge' else 2 * math.acos(
                        min(1.0, abs(sum(a * b for a, b in zip(value, old_value)))))
                    changes.append({'step': step, 'from_stage': old.get('stage'),
                                    'to_stage': fields.get('stage'), 'radians': delta})
            previous = step, fields, kind, value
        result[key] = {
            'recorded_rows': len(frames), 'invalid_or_undriven_rows': invalid,
            'gaps_between_valid_rows': gaps, 'incompatible_target_pairs': incompatible,
            'comparable_target_pairs': len(changes),
            'maximum_target_change': max(changes, key=lambda r: r['radians'], default=None),
            'maximum_stage_transition_target_change': max(
                (r for r in changes if r['from_stage'] != r['to_stage']),
                key=lambda r: r['radians'], default=None),
            'native_limit_samples': len(limits),
            'maximum_native_limit_rejection': max(limits, key=lambda r: r['radians'], default=None),
        }
    return {'scope': 'All recorded driven stages, not a stable-walking subset or an acceptance verdict. '
            'Legacy clamp/target_adjustment is not interpreted as native-limit rejection. '
            'Target changes are radians between contiguous commands in the same coordinate space.',
            'joints': result}


def propulsion_sample(step, applied, after):
    """Pair pre-Update requests with this Update's net COM response.

    Net acceleration includes contacts, motors and every other physical force;
    it is not the acceleration delivered by an individual actuator.
    """
    if applied.get('measured') != '1' or after.get('measured') != '1':
        return None
    if applied.get('request_valid') != '1' or applied.get('support_valid') != '1':
        return None
    def vector(key, fields):
        values = [float(v) for v in fields[key].split(',')]
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid propulsion vector: ' + key)
        return values
    dt = float(applied['physics_dt'])
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Invalid physics timestep')
    heading = vector('filtered', applied)
    length = math.hypot(*heading[:2])
    axis_source = 'filtered_travel'
    if length == 0:
        # Zero travel is the command under investigation during stationary
        # braking. Preserve those samples using explicitly recorded facing.
        if 'facing' not in applied:
            return None
        heading = vector('facing', applied)
        length = math.hypot(*heading[:2])
        axis_source = 'facing'
        if length == 0:
            return None
    heading = [v / length for v in heading[:2]]
    before_velocity, after_velocity = vector('velocity', applied), vector('velocity', after)
    measured = [(b - a) / dt for a, b in zip(before_velocity, after_velocity)]
    project = lambda v: sum(a * b for a, b in zip(v, heading))
    hull_distance = None
    friction_limited = None
    if applied.get('hull_known') == '1':
        delta = [float(v) for v in applied['hull_delta'].split(',')]
        if len(delta) != 2 or not all(math.isfinite(v) for v in delta):
            raise ValueError('Invalid hull projection displacement')
        hull_distance = math.hypot(*delta)
        friction_limited = applied['friction_limited'] == '1'
    def loads(fields):
        result = {}
        for foot in ('left', 'right'):
            raw = fields.get(foot + '_load')
            value = float(raw) if raw is not None else None
            if value is not None and not math.isfinite(value):
                raise ValueError('Invalid contact load')
            # Native -1 denotes unavailable impulse data, not negative load.
            result[foot] = value if value is not None and value >= 0 else None
        return result
    return {'step': step, 'stage': applied.get('reference_stage'),
            'contact_load_body_weights_before': loads(applied),
            'contact_load_body_weights_after': loads(after),
            'axis_source': axis_source,
            'requested_acceleration': vector('requested_accel', applied),
            'budgeted_acceleration': vector('budgeted_accel', applied),
            'net_acceleration': measured,
            'support_applied': applied.get('support_applied') == '1',
            'requested_forward_accel': project(vector('requested_accel', applied)),
            'budgeted_forward_accel': project(vector('budgeted_accel', applied)),
            'net_forward_accel': project(measured),
            'hull_projection_distance': hull_distance,
            'friction_limited': friction_limited,
            'forward_speed_before': project(before_velocity),
            'forward_speed_after': project(after_velocity)}


def declared_decimal_fields(text):
    fields = dict(re.findall(r'(\w+)=([^\s]+)', text))
    if fields.get('integer_radix') != '10':
        raise ValueError('Trace integer radix is undeclared or non-decimal; audit legacy step/body IDs before comparison')
    for key in ('step', 'body', 'render', 'substep'):
        if key in fields and not re.fullmatch(r'[0-9]+', fields[key]):
            raise ValueError('Non-decimal trace integer: ' + key)
    return fields


def summarize(lines):
    actors = {}
    landing_errors = []
    leg_commands = []
    for line in lines:
        if 'RAGDOLL_LEG_COMMAND ' in line:
            leg_commands.append(declared_decimal_fields(line.split('RAGDOLL_LEG_COMMAND ', 1)[1]))
        if 'RAGDOLL_LANDING_REBASE ' in line:
            match = re.search(r'world_error=([^\s]+)', line)
            if not match:
                raise ValueError('Missing landing rebase error')
            error = float(match[1])
            if not math.isfinite(error) or error < 0:
                raise ValueError('Invalid landing rebase error')
            landing_errors.append(error)
        if 'RAGDOLL_STEP ' not in line:
            continue
        fields = declared_decimal_fields(line.split('RAGDOLL_STEP ', 1)[1])
        key = fields['actor'] + ':player=' + fields['player']
        actor = actors.setdefault(key, {})
        step = int(fields['step'])
        phase = fields['phase']
        if phase not in ('input', 'applied', 'observed'):
            raise ValueError('Unknown trace phase')
        frame = actor.setdefault(step, {})
        if phase in frame:
            raise ValueError('Duplicate actor/step/phase; actor identity is ambiguous')
        frame[phase] = fields
    result = {}
    for actor, frames in actors.items():
        complete = [step for step, phases in frames.items() if len(phases) == 3]
        missing = [step for step, phases in frames.items() if len(phases) != 3]
        previous = None
        discontinuity = None
        changed_during_application = None
        first_raw_change = None
        first_filter_change = None
        first_mode_change = None
        first_nonzero_raw = None
        first_nonzero_applied = None
        gaps = 0
        max_substep = 0
        propulsion = []
        coordinated_known = 0
        coordinated_steps = []
        travel_runs = []
        travel_run = None
        for step in sorted(complete):
            phases = frames[step]
            before, applied, after = (phases[k] for k in ('input', 'applied', 'observed'))
            # A missing/incomplete step may contain a release or direction
            # change. Do not merge progress across that unobserved interval.
            if previous and step != previous[0] + 1:
                travel_run = None
            raw = [float(v) for v in applied['raw'].split(',')]
            raw_length = math.hypot(*raw[:2])
            if raw_length:
                if travel_run is None:
                    travel_run = {'first_step': step, 'last_step': step,
                                  'measured_steps': 0, 'missing_steps': 0,
                                  'velocity_integral_progress': 0.0}
                    travel_runs.append(travel_run)
                travel_run['last_step'] = step
                if applied.get('measured') == '1' and after.get('measured') == '1':
                    v0 = [float(v) for v in applied['velocity'].split(',')]
                    v1 = [float(v) for v in after['velocity'].split(',')]
                    dt = float(applied['physics_dt'])
                    if len(raw) != 3 or len(v0) != 3 or len(v1) != 3 or dt <= 0 or not all(
                            math.isfinite(v) for v in [*raw, *v0, *v1, dt]):
                        raise ValueError('Invalid travel integration sample')
                    travel_run['velocity_integral_progress'] += sum(
                        (a + b) * 0.5 * r / raw_length for a, b, r in zip(v0[:2], v1[:2], raw[:2])) * dt
                    travel_run['measured_steps'] += 1
                else:
                    travel_run['missing_steps'] += 1
            else:
                travel_run = None
            if 'coordinated_pose' in applied:
                if applied['coordinated_pose'] not in ('0', '1'):
                    raise ValueError('Invalid coordinated pose flag')
                coordinated_known += 1
                if applied['coordinated_pose'] == '1':
                    coordinated_steps.append(step)
            sample = propulsion_sample(step, applied, after)
            if sample is not None:
                propulsion.append(sample)
            if first_nonzero_raw is None and any(float(v) != 0 for v in before['raw'].split(',')):
                first_nonzero_raw = step
            if first_nonzero_applied is None and any(float(v) != 0 for v in applied['filtered'].split(',')):
                first_nonzero_applied = step
            max_substep = max(max_substep, int(before['substep']))
            # Strings are round-tripped log values, not bitwise float evidence.
            if before['filtered'] != applied['filtered'] and changed_during_application is None:
                changed_during_application = step
            if before['filtered'] != after['filtered'] and first_filter_change is None:
                first_filter_change = step
            if before['mode'] != after['mode'] and first_mode_change is None:
                first_mode_change = step
            if previous:
                old_step, old = previous
                if step != old_step + 1:
                    gaps += 1
                else:
                    if before['filtered'] != old['filtered'] and discontinuity is None:
                        discontinuity = step
                    if before['raw'] != old['raw'] and first_raw_change is None:
                        first_raw_change = step
            previous = step, after
        stage_summary = {}
        for sample in propulsion:
            stage_summary.setdefault(sample['stage'], []).append(sample)
        for stage, samples in stage_summary.items():
            known = [s for s in samples if s['hull_projection_distance'] is not None]
            stage_summary[stage] = {
                'samples': len(samples),
                'facing_axis_samples': sum(s['axis_source'] == 'facing' for s in samples),
                **{key: sum(s[key] for s in samples) / len(samples) for key in
                   ('requested_forward_accel', 'budgeted_forward_accel', 'net_forward_accel')},
                'hull_samples': len(known),
                'maximum_hull_projection_distance': max((s['hull_projection_distance'] for s in known), default=None),
                'friction_limited_samples': sum(s['friction_limited'] for s in known) if known else None,
            }
        result[actor] = {
            'complete_steps': len(complete), 'incomplete_steps': missing,
            'gaps_between_complete_steps': gaps, 'maximum_substep_in_render_frame': max_substep,
            'first_raw_change_between_steps': first_raw_change,
            'first_filter_update_after_physics': first_filter_change,
            'first_mode_update_after_physics': first_mode_change,
            'first_nonzero_raw_step': first_nonzero_raw,
            'first_nonzero_applied_filtered_step': first_nonzero_applied,
            'first_filter_change_during_command_application': changed_during_application,
            'first_filter_discontinuity_between_steps': discontinuity,
            'propulsion_samples': len(propulsion),
            'requested_travel_runs': travel_runs,
            'travel_note': 'Trapezoidal COM velocity integral along pre-step raw request; not actor displacement.',
            'coordinated_pose_observed_steps': coordinated_known,
            'coordinated_pose_applied_steps': len(coordinated_steps) if coordinated_known else None,
            'first_coordinated_pose_step': next(iter(coordinated_steps), None),
            'propulsion_by_reference_stage': stage_summary,
            'first_propulsion_sample': next(iter(propulsion), None),
            'largest_forward_acceleration_shortfall': max(propulsion,
                key=lambda s: s['requested_forward_accel'] - s['net_forward_accel'], default=None),
            'propulsion_note': 'Budgeted acceleration precedes motor limits; net COM response includes all forces.',
            'contact_load_note': 'Before uses the preceding solved contact impulses available to this command; '
                'after uses this Update\'s solved impulses. Each foot is normalized by total body weight times dt, '
                'not by summed foot load; values may exceed one. Missing or negative sentinels become null. '
                'Normal impulse is not world-vertical impulse on a sloped contact.',
        }
    return {'scope': 'Logged scalar continuity only; not full command or physics equivalence.', 'actors': result,
            'leg_commands': leg_command_summary(leg_commands),
            'landing_rebase_samples': len(landing_errors),
            'maximum_landing_rebase_world_error': max(landing_errors, default=None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    with args.log.open(encoding='utf-8-sig', errors='replace') as stream:
        result = summarize(stream)
    print(json.dumps(result, indent=2))
    if not result['actors']:
        raise SystemExit('No RAGDOLL_STEP samples found')


if __name__ == '__main__':
    main()
