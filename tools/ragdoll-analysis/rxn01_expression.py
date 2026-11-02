#!/usr/bin/env python3
"""RXN-01 fixed expression screen, measured from physical body recordings."""
import argparse
import concurrent.futures
import csv
import hashlib
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np

SCENES = ('face_guard', 'arm_cradle', 'doubled_over', 'stumble_flail')
THRESHOLDS = dict(zip(SCENES, (10.0, 10.0, 8.0, 4.0)))


def read_bodies(path):
    data = np.genfromtxt(path, delimiter=',', names=True)
    count = int(data['body'].max()) + 1
    return {key: data[key].reshape(-1, count) for key in data.dtype.names}


def rotate(q, v):
    t = 2 * np.cross(q[..., :3], v)
    return v + q[..., 3:] * t + np.cross(q[..., :3], t)


def landmarks(path):
    data = read_bodies(path)
    xyz = np.stack([data[c] for c in ('x', 'y', 'z')], axis=-1)
    q = np.stack([data[c][:, 0] for c in ('qx', 'qy', 'qz', 'qw')], axis=-1)
    inverse = q.copy()
    inverse[:, :3] *= -1
    # Remove translation/rotation of the dynamic pelvis, then express in the
    # original upright frame. A fall cannot masquerade as raised hands.
    local = rotate(inverse[:, None, :], xyz - xyz[:, :1, :])
    points = rotate(q[0][None, None, :], local)
    left = points[:, 9] - points[:, 7]
    right = points[:, 13] - points[:, 11]
    trunk = points[:, 4] - points[:, 1]
    return {
        'left_lift': left[:, 2], 'right_lift': right[:, 2],
        'left_spread': -left[:, 0], 'right_spread': right[:, 0],
        'trunk_degrees': np.degrees(np.arctan2(trunk[:, 1], trunk[:, 2])),
        'left_head_gap': points[:, 5, 2] - points[:, 9, 2],
        'right_head_gap': points[:, 5, 2] - points[:, 13, 2],
    }


def measure(directory, scene, controls):
    body = directory / f'RXN-01-{scene}-bodies.csv'
    metrics = directory / f'RXN-01-{scene}-reactions.csv'
    values, control = landmarks(body), landmarks(controls / f'RXN-01-{scene}-control-bodies.csv')
    rows = list(csv.DictReader(metrics.open(newline='')))
    active = np.array([r['behavior'] == scene for r in rows])
    delta = {k: v - control[k][:len(v)] for k, v in values.items()}
    if scene == 'face_guard':
        score = np.minimum(delta['left_lift'], delta['right_lift'])
    elif scene == 'arm_cradle':
        score = delta['left_lift']
    elif scene == 'doubled_over':
        score = delta['trunk_degrees']
    else:
        score = np.minimum(delta['left_spread'], delta['right_spread'])
    eligible = active & (np.arange(len(rows)) >= 121)
    visible = eligible & (score >= THRESHOLDS[scene])
    longest = current = 0
    for value in visible:
        current = current + 1 if value else 0
        longest = max(longest, current)
    peak = float(score[eligible].max()) if eligible.any() else 0.0
    head_gap = float(np.maximum(values['left_head_gap'], values['right_head_gap'])[eligible].min()) if eligible.any() else None
    recognizable = peak >= THRESHOLDS[scene] and longest >= 12
    if scene == 'face_guard':
        recognizable = recognizable and head_gap is not None and head_gap <= 15 and longest >= 30
    return {'peak_delta': round(peak, 3), 'threshold': THRESHOLDS[scene],
            'visible_seconds': round(float(visible.sum()) / 60, 3),
            'longest_visible_seconds': round(longest / 60, 3),
            'peak_frame': int(np.argmax(np.where(eligible, score, -1e9))),
            'minimum_head_gap': round(head_gap, 3) if head_gap is not None else None,
            'passed': recognizable,
            'body_sha256': hashlib.sha256(body.read_bytes()).hexdigest()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--walksearch', type=Path, required=True)
    p.add_argument('--controls', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--baseline-only', action='store_true')
    p.add_argument('--rescore', action='store_true', help='Re-measure existing physics recordings without rerunning the batch')
    p.add_argument('--snapshot', type=Path, help='Optional committed location for the generated screen summary')
    args = p.parse_args()
    root, out = args.root.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    controls = args.controls.resolve()
    baseline = {s: measure(controls, s, controls) for s in SCENES}
    (out / 'baseline.json').write_text(json.dumps(baseline, indent=2) + '\n')
    if args.baseline_only:
        print(json.dumps(baseline, indent=2))
        return 0
    exe = args.walksearch.resolve()

    def run(item):
        scale, duration, scene = item
        directory = out / f'expression-{scale}-duration-{duration}'
        directory.mkdir(exist_ok=True)
        command = [str(exe), '--rxn01-scene', scene, '--native-body',
                   str(root / 'files/data/ragdoll/calibration/context-symP.creation.txt'),
                   str(root / 'files/data/ragdoll/humanoid-v1.yaml'), '--seconds', '20',
                   '--speed', '30', '--rate', '60', '--trace', str(directory),
                   '--name', f'RXN-01-{scene}', '--rxn01-expression', str(scale),
                   '--rxn01-duration', str(duration), 'mesh_ground=true']
        result = subprocess.run(command, capture_output=True, text=True)
        (directory / f'{scene}.log').write_text(result.stdout + result.stderr)
        summary = next((json.loads(l) for l in reversed(result.stdout.splitlines()) if l.startswith('{')), {})
        measured = measure(directory, scene, controls)
        return {'scale': scale, 'duration': duration, 'scene': scene, 'measurements': measured,
                'runner_ok': result.returncode == 0 and summary.get('checks_passed') is True,
                'summary': summary, 'command': command}

    cells = list(itertools.product((0.8, 1.0, 1.2), (1.0, 1.5, 2.0)))
    original = None
    if args.rescore:
        original = json.loads((out / 'screen.json').read_text())
        results = original['results']
        for result in results:
            directory = out / f"expression-{result['scale']}-duration-{result['duration']}"
            measurements = measure(directory, result['scene'], controls)
            if measurements['body_sha256'] != result['measurements']['body_sha256']:
                raise ValueError('Cannot rescore modified physics recordings')
            result['measurements'] = measurements
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(run, [(s, d, scene) for s, d in cells for scene in SCENES]))
    scores = []
    for scale, duration in cells:
        trials = [r for r in results if r['scale'] == scale and r['duration'] == duration]
        passed = all(r['runner_ok'] and r['measurements']['passed'] for r in trials)
        # The three non-falling original scenes must remain standing.
        no_new_falls = all(not r['summary'].get('fallen') for r in trials if r['scene'] != 'stumble_flail')
        scores.append({'scale': scale, 'duration': duration, 'passed': passed and no_new_falls,
                       'no_new_falls': no_new_falls,
                       'measurements': {r['scene']: r['measurements'] for r in trials}})
    qualified = sorted((s for s in scores if s['passed']), key=lambda s: (s['duration'], s['scale']))
    report = {'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
              'working_tree_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True).strip()),
              'binary_sha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
              'baseline': baseline, 'cells': scores, 'results': results,
              'selected': qualified[0] if qualified else None, 'visual_acceptance': 'pending'}
    if original:
        for key in ('source_commit', 'working_tree_dirty', 'binary_sha256'):
            report[key] = original[key]
    report['guard_review_requirement'] = 'Both wrists within 15 units below the head; raised at least 0.5 seconds'
    (out / 'screen.json').write_text(json.dumps(report, indent=2) + '\n')
    if args.snapshot:
        args.snapshot.write_text(json.dumps(report, indent=2) + '\n')
    lines = ['RXN-01 physical expression screen; visual acceptance pending']
    for cell in scores:
        lines.append(f"scale={cell['scale']} duration={cell['duration']} passed={cell['passed']} "
                     + ' '.join(f"{s}={m['peak_delta']}/{m['longest_visible_seconds']}s" for s, m in cell['measurements'].items()))
    lines.append('Selected: ' + json.dumps(report['selected']))
    (out / 'SUMMARY.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0 if qualified else 1


if __name__ == '__main__':
    raise SystemExit(main())
