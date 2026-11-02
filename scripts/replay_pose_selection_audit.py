"""Check native motor-base selection against recorded animation poses.

The previous-render sample is a hypothesis: render frames with no physics
solve may update animation without leaving a binary replay frame. Do not
claim missing history has been reconstructed when this audit disagrees.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import struct


def animation_frames(path):
    data = path.read_bytes()
    if len(data) < 56 or data[:8] != b'OMWRDPL\0':
        raise ValueError('Invalid replay header')
    version, seed, profile, dt, workers, collision, inputs, states, bodies, count = struct.unpack_from('<IQQfIIIIII', data, 8)
    if version != 10 or inputs != 10 + 7 * bodies or not (0 < bodies <= 1024):
        raise ValueError('Expected version10 powered pose input layout')
    stride = 8 + 4 * (inputs + states + 13 * bodies)
    if len(data) != 56 + count * stride:
        raise ValueError('Replay length does not match header')
    result = []
    for index in range(count):
        offset = 56 + index * stride
        if struct.unpack_from('<Q', data, offset)[0] != index:
            raise ValueError('Noncontiguous replay indices')
        result.append(struct.unpack_from('<' + 'f' * (7 * bodies), data, offset + 8 + 40))
    return result, bodies


def audit_render_history(lines, actor_kind, *, reconstruct_only=False, input_modes=None, paired_candidates=False):
    """Reconstruct selection from animation/RAW events, never BASE outputs."""
    flag = '1' if actor_kind == 'player' else '0'
    samples = {}
    actors = set()
    current = None
    frozen = None
    moving = False
    predictions = {}
    actual = {}
    recorded = []
    # The first animation capture can backfill serial0 (the initial pose).
    # Resolve declared samples first, then replay raw events in log order.
    lines = list(lines)
    for line in lines:
        fields = dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)', line))
        if fields.get('player') != flag:
            continue
        if 'RAGDOLL_ANIMATION_INPUT ' in line:
            serial, body, count = (int(fields[k]) for k in ('serial', 'body', 'bodies'))
            expected_count, values = samples.setdefault(serial, (count, {}))
            if count != expected_count or body in values:
                raise ValueError('Duplicate or inconsistent animation sample')
            values[body] = tuple(float(v) for k in ('position', 'rotation') for v in fields[k].split(','))
    def pose(serial):
        if serial not in samples:
            raise ValueError('Missing animation capture history')
        count, values = samples[serial]
        if sorted(values) != list(range(count)):
            raise ValueError('Incomplete animation capture')
        return tuple(v for i in range(count) for v in values[i])
    for line in lines:
        fields = dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)', line))
        if 'RAGDOLL_REPLAY_FRAME ' in line:
            recorded.append(int(fields['step']))
        if fields.get('player') != flag:
            continue
        if 'RAGDOLL_ANIMATION_INPUT_TRUNCATED ' in line:
            raise ValueError('Animation capture truncated')
        if 'RAGDOLL_ANIMATION_INPUT ' in line or 'RAGDOLL_ANIMATION_RAW ' in line:
            if fields.get('schema') != '1': raise ValueError('Unknown animation input schema')
            actors.add(fields['actor'])
            if len(actors) > 1: raise ValueError('Isolated actor required')
        if 'RAGDOLL_ANIMATION_RAW ' in line:
            serial = int(fields['serial'])
            current = pose(serial)
            next_moving = sum(float(v)**2 for v in fields['raw'].split(',')) > 1
            if next_moving and not moving:
                frozen = pose(serial - 1) if fields['previous_known'] == '1' else current
            elif moving and not next_moving:
                frozen = None
            moving = next_moving
        elif 'RAGDOLL_STEP phase=input ' in line:
            step = int(fields['step'])
            if step in predictions: raise ValueError('Duplicate physics input step')
            if paired_candidates:
                predictions[step] = (current if frozen is None else frozen, current)
            else:
                mode = int(fields['mode']) if input_modes is None else input_modes[step]
                predictions[step] = current if mode in (5, 6) or frozen is None else frozen
        elif 'RAGDOLL_REPLAY_BASE ' in line and not reconstruct_only:
            step, body = int(fields['step']), int(fields['body'])
            values = actual.setdefault(step, {})
            if body in values: raise ValueError('Duplicate motor base body')
            values[body] = tuple(float(v) for k in ('position', 'rotation') for v in fields[k].split(','))
    if not recorded or len(set(recorded)) != len(recorded):
        raise ValueError('Need one nonempty recording')
    if reconstruct_only:
        if any(predictions.get(step) is None for step in recorded):
            raise ValueError('Missing initial animation state')
        return predictions
    mismatches = []
    for step in recorded:
        predicted = predictions.get(step)
        if predicted is None: raise ValueError('Missing initial animation state')
        values = actual.get(step, {})
        if sorted(values) != list(range(len(predicted) // 7)):
            raise ValueError('Missing motor base observations')
        target = tuple(v for i in range(len(values)) for v in values[i])
        # Native model transforms and binary inputs are float32.
        pack = lambda values: struct.pack('<' + 'f' * len(values), *values)
        if pack(predicted) != pack(target): mismatches.append(step)
    return {'frames': len(recorded), 'animation_captures': len(samples),
        'mismatching_steps': mismatches, 'selected_poses_exact': not mismatches}


def audit(capture, log, actor_kind):
    poses, bodies = animation_frames(capture)
    flags = '1' if actor_kind == 'player' else '0'
    frames = {}
    commands = {}
    bases = {}
    actors = set()
    for line in log.read_text(encoding='utf-8').splitlines():
        fields = dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)', line))
        if 'RAGDOLL_REPLAY_FRAME ' in line:
            index = int(fields['frame'])
            if index in frames: raise ValueError('Multiple recordings')
            frames[index] = int(fields['step'])
        elif 'RAGDOLL_STEP phase=input ' in line and fields.get('player') == flags:
            actors.add(fields['actor'])
            step = int(fields['step'])
            if step in commands: raise ValueError('Duplicate command step')
            commands[step] = fields
        elif 'RAGDOLL_REPLAY_BASE ' in line and fields.get('player') == flags:
            key = (int(fields['step']), int(fields['body']))
            if key in bases: raise ValueError('Duplicate base pose')
            values = [float(v) for field in ('position', 'rotation') for v in fields[field].split(',')]
            bases[key] = struct.unpack('<7f', struct.pack('<7f', *values))
    if len(actors) != 1 or sorted(frames) != list(range(len(poses))):
        raise ValueError('Need one isolated actor and complete recording')
    counts = Counter()
    mismatches = []
    render = None
    current_animation = None
    previous_animation = None
    was_moving = False
    frozen = None
    for index, animation in enumerate(poses):
        step = frames[index]
        command = commands[step]
        new_render = int(command['render'])
        if new_render != render:
            previous_animation = current_animation
            current_animation = animation
            render = new_render
        moving = sum(float(v) ** 2 for v in command['raw'].split(',')) > 1
        if moving and not was_moving:
            frozen = previous_animation
        elif was_moving and not moving:
            frozen = None
        was_moving = moving
        recovering = int(command['mode']) in (5, 6)
        predicted = animation if recovering or not moving else frozen
        source = 'animation' if recovering or not moving else 'previous_render_frozen'
        if predicted is None:
            counts['unavailable_history'] += 1
            continue
        actual = tuple(v for body in range(bodies) for v in bases[(step, body)])
        if predicted == actual:
            counts[source + '_exact'] += 1
        else:
            counts[source + '_mismatch'] += 1
            mismatches.append({'frame': index, 'step': step, 'render': render, 'source': source,
                'mode': int(command['mode']), 'max_component_difference': max(abs(a-b) for a,b in zip(predicted, actual))})
    return {'counts': dict(counts), 'frames': len(poses), 'first_mismatches': mismatches[:12],
        'all_pose_components_exact': not mismatches and not counts['unavailable_history'],
        'comparison': 'float32 model pose components; not physical replay or quaternion sign equivalence',
        'hypothesis': 'native frozen locomotion base equals previous recorded render animation pose',
        'sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in [('capture', capture), ('log', log)]}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--actor-kind', choices=('player', 'npc'), default='player')
    parser.add_argument('--render-history', action='store_true',
        help='Audit new render-level events; capture argument is hashed for provenance')
    args = parser.parse_args()
    if args.render_history:
        with args.log.open(encoding='utf-8') as stream:
            report = audit_render_history(stream, args.actor_kind)
        report['sha256'] = {name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in [('capture', args.capture), ('log', args.log)]}
        report['scope'] = 'motor-base selection from animation capture events; not physical replay'
    else:
        report = audit(args.capture, args.log, args.actor_kind)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
