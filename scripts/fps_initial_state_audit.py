"""Reject unmatched FPS starting states; this is not an FPS behavior verdict."""
import argparse
import hashlib
import json
import re
from pathlib import Path


def creation(path):
    rig = None
    bones = []
    initial = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        f = dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)', line))
        if 'RAGDOLL_REPLAY_RIG ' in line:
            if rig is not None: raise ValueError('Multiple rig captures')
            rig = {k: f[k] for k in ('profile_id', 'profile_hash', 'mass', 'scale',
                                     'root', 'rotation', 'velocity', 'angular_velocity')}
        elif 'RAGDOLL_REPLAY_BONE ' in line:
            bones.append({k: f[k] for k in ('index', 'name', 'parent', 'position', 'rotation')})
        elif 'RAGDOLL_REPLAY_INITIAL ' in line:
            body = f['body']
            if body in initial: raise ValueError('Multiple initial states; isolate one actor')
            initial[body] = {k: f.get(k) for k in ('body_id', 'position', 'rotation', 'velocity', 'angular_velocity')}
    if rig is None or not bones or len(initial) != len(bones):
        raise ValueError('Incomplete creation capture')
    return {'rig': rig, 'bones': bones, 'initial': initial}


def compare(reference, candidate):
    left, right = creation(reference), creation(candidate)
    differences = []
    missing = []
    def visit(a, b, path):
        if a is None or b is None:
            missing.append(path)
        elif isinstance(a, dict) and isinstance(b, dict):
            for key in sorted(a.keys() | b.keys()): visit(a.get(key), b.get(key), path + '.' + key)
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b): differences.append(path + '.length')
            for i, (x,y) in enumerate(zip(a,b)): visit(x,y,path + '.' + str(i))
        elif a != b:
            # Normalize numeric formatting without changing numerical tolerance.
            try: same = tuple(float(v) for v in a.split(',')) == tuple(float(v) for v in b.split(','))
            except (ValueError, AttributeError): same = False
            if not same: differences.append(path)
    visit(left,right,'initial_state')
    return {'initial_state_matches': not differences and not missing,
        'first_difference': differences[0] if differences else None,
        'differences': differences, 'missing_fields': missing,
        'fps_verdict': 'not evaluated',
        'qualification_ready': False,
        'remaining_preflight': 'common authored source and native sampling schedule must also be verified',
        'sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest()
                   for name,path in [('reference',reference),('candidate',candidate)]}}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference',type=Path); parser.add_argument('candidate',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    report=compare(args.reference,args.candidate)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
