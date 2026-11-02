"""First logged channel differences at equal native physics step, without alignment."""
import argparse
import json
from pathlib import Path
import re

CHANNELS = ('raw', 'filtered', 'facing_request', 'facing', 'base_pose_hash',
            'mode', 'reference_stage', 'reference_phase', 'placement',
            'pelvis_height', 'root', 'velocity', 'left', 'right',
            'requested_accel', 'budgeted_accel')


def read(path):
    result = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if 'RAGDOLL_STEP ' not in line:
            continue
        fields = dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)', line))
        key = (int(fields['step']), fields['phase'])
        if key in result:
            raise ValueError('multiple actors or duplicate step')
        result[key] = fields
    if not result:
        raise ValueError('missing step trace')
    return result


def compare(reference, candidate):
    left, right = read(reference), read(candidate)
    differences = {}
    for phase in ('input', 'applied', 'observed'):
        differences[phase] = {}
        for channel in CHANNELS:
            found = None
            for step, selected in sorted(left.keys() & right.keys()):
                if selected != phase:
                    continue
                a, b = left[step, phase][channel], right[step, phase][channel]
                # Preserve integer hash precision; numeric formatting differences
                # are reported as logged differences, never as bitwise proof.
                if a != b:
                    found = {'native_step': step, 'reference': a, 'candidate': b}
                    break
            differences[phase][channel] = found
    return {'first_logged_differences': differences,
            'unpaired_steps': {'reference': sorted(set(s for s, _ in left) - set(s for s, _ in right)),
                               'candidate': sorted(set(s for s, _ in right) - set(s for s, _ in left))},
            'scope': 'selected serialized scalar channels, no phase realignment; not a full physical/tolerance verdict'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = compare(args.reference, args.candidate)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
