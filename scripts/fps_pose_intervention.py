"""Create a labeled counterfactual pose history; never modify a native recording."""
import argparse
import hashlib
import json
from pathlib import Path


def layout(path):
    lines = path.read_text(encoding='utf-8').splitlines()
    if lines[0].split()[0] != '5':
        raise ValueError('requires paired-pose context v5')
    warmup = int(lines[0].split()[-1])
    bodies = int(lines[6])
    count_index = 7 + bodies + 2
    frames = int(lines[count_index]) + warmup
    start, stride = count_index + 1, 2 + 2 * bodies
    for index in range(frames):
        if int(lines[start + index * stride].split()[0]) != index:
            raise ValueError('noncontiguous input history')
    return lines, start, stride, frames


def replace(recipient, donor, output, group='both', begin=0, end=None):
    if group not in ('both', 'locomotion', 'animation'):
        raise ValueError('unknown pose group')
    if output.resolve() in (recipient.resolve(), donor.resolve()):
        raise ValueError('cannot overwrite source evidence')
    a, start, stride, frames = layout(recipient)
    b, other_start, other_stride, other_frames = layout(donor)
    if start != other_start or stride != other_stride or a[:start-2] != b[:start-2]:
        raise ValueError('mismatched rig, IDs, initialization or context schema')
    shared = min(frames, other_frames)
    stop = shared if end is None else end
    if not 0 <= begin <= stop <= shared:
        raise ValueError('invalid shared-prefix intervention window')
    changed = []
    for step in range(begin, stop):
        first = start + step * stride + 2
        last = start + (step + 1) * stride
        body_count = (stride - 2) // 2
        if group == 'locomotion': last = first + body_count
        if group == 'animation': first += body_count
        if a[first:last] != b[first:last]:
            changed.append(step)
        a[first:last] = b[first:last]
    output.write_text('\n'.join(a) + '\n', encoding='utf-8')
    report = {'counterfactual_only': True, 'native_equivalence_claim': False,
              'intervention': group + ' pose candidate(s) only; raw travel and outer frames retained',
              'shared_scenario_steps': shared, 'unchanged_recipient_tail_steps': frames-shared,
              'window_begin_inclusive': begin, 'window_end_exclusive': stop,
              'changed_steps': len(changed), 'first_changed_step': changed[0] if changed else None,
              'sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest()
                         for name, path in (('recipient',recipient),('donor',donor),('output',output))}}
    output.with_suffix('.intervention.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recipient', type=Path)
    parser.add_argument('donor', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--group', choices=('both','locomotion','animation'), default='both')
    parser.add_argument('--begin', type=int, default=0)
    parser.add_argument('--end', type=int)
    args = parser.parse_args()
    print(json.dumps(replace(args.recipient,args.donor,args.output,args.group,args.begin,args.end), indent=2))
