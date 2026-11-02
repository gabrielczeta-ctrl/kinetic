"""Audit the opt-in render clock's observed native sampling cadence.

This does not compare controller behavior or certify identical authored inputs.
"""
import argparse
import json
from pathlib import Path
import re
import struct

from native_setup_errors import setup_error


def audit(path):
    requested = None
    armed = False
    samples = []
    errors = []
    for line in path.read_text(encoding='utf-8').splitlines():
        fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
        failure = setup_error(line)
        if failure and len(errors) < 20:
            errors.append(failure)
        if 'RAGDOLL_FPS_CLOCK ' in line:
            if requested is not None:
                errors.append('multiple clock declarations')
            requested = int(fields['requested_fps'])
        elif 'RAGDOLL_FPS_CLOCK_ARMED ' in line:
            if armed or int(fields['fps']) != requested:
                errors.append('invalid clock arm event')
            armed = True
        elif armed and 'RAGDOLL_ANIMATION_RAW ' in line:
            samples.append(fields)
        elif 'TRUNCATED' in line or 'dropped' in line.lower() and 'physics' in line.lower():
            errors.append('trace truncation or dropped physics time')
    if requested not in (30, 60, 120) or not armed or len(samples) < 4:
        errors.append('missing clock or insufficient samples')
    else:
        expected_dt = struct.unpack('f', struct.pack('f', 1 / requested))[0]
        for index, sample in enumerate(samples):
            if float(sample.get('render_dt', 'nan')) != expected_dt:
                errors.append(f'render_dt mismatch at sample {index}')
        deltas = [int(b['next_step']) - int(a['next_step']) for a, b in zip(samples, samples[1:])]
        expected = {30: {2}, 60: {1}, 120: {0, 1}}[requested]
        if set(deltas) != expected:
            errors.append(f'unexpected fixed-step cadence: {sorted(set(deltas))}')
        if requested == 120 and any(a == b for a, b in zip(deltas, deltas[1:])):
            errors.append('120 FPS cadence does not alternate zero/one solve')
        if any(int(b['render']) <= int(a['render']) for a, b in zip(samples, samples[1:])):
            errors.append('multiple actors or unordered render samples')
    return {'requested_fps': requested, 'sample_count': len(samples),
            'cadence_valid': not errors, 'errors': errors,
            'qualification_ready': False, 'fps_behavior_verdict': 'not evaluated'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = audit(args.log)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
