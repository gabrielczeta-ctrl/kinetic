#!/usr/bin/env python3
"""Record the six fixed RXN-01 physical review scenes for tools/offline-viewer.

This diagnostic pack does not award tracker or visual acceptance. Use
rxn01_qualify.py --tracker-task RXN-01 for the registered offline gate.
"""
import argparse
import concurrent.futures
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import yaml
from rxn01_expression import SCENES as EXPRESSION_SCENES, measure

SCENES = {
    'face_guard': ('Face guard', 'head', 1400, None,
                   'Head impact; the elbows fold and both arms rise into a guard, held for 1.35 seconds.'),
    'arm_cradle': ('Arm cradle', 'left_forearm', 300, 0.55,
                   'Left forearm impact with explicit fixture injury; the injured arm folds toward the chest and stays cradled.'),
    'limp': ('Limp', 'left_calf', 300, 0.60,
             'Left calf injury persists. The current adapter consumes speed scale; asymmetric stance bias is not yet connected.'),
    'doubled_over': ('Doubled over', 'spine', 2200, None,
                     'Abdomen impact; the spine folds forward and the arms draw inward, with reduced stabilization.'),
    'stumble_flail': ('Stumble / flail', 'right_upper_arm', 2500, None,
                       'Moving upper-arm impact; open arms reach outward to catch balance before the fall.'),
    'knockdown': ('Knockdown and recovery', 'spine', 5200, None,
                  'Heavy impact; inspect the fall, powered rise, blend-back and resumed walking.'),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--walksearch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--recovery-archive', type=Path, help='Installed Morrowind.bsa; actual kinematic-root recovery clips')
    parser.add_argument('--snapshot', type=Path, help='Optional committed evidence location for the generated summary')
    args = parser.parse_args()
    root, exe, out = args.root.resolve(), args.walksearch.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    context = root / 'files/data/ragdoll/calibration/context-symP.creation.txt'
    profile = root / 'files/data/ragdoll/humanoid-v1.yaml'
    bodies = yaml.safe_load(profile.read_text())['bodies']
    names = [b['name'] for b in bodies]
    parents = [names.index(b['parent']) if b.get('parent') else -1 for b in bodies]
    commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True).strip())
    provenance = {'source_commit': commit, 'working_tree_dirty': dirty,
                  'binary_sha256': digest(exe), 'context_sha256': digest(context),
                  'profile_sha256': digest(profile)}
    if args.recovery_archive:
        provenance['recovery_archive_sha256'] = digest(args.recovery_archive)
        provenance['recovery_clips'] = ['meshes/base_anim.nif', 'meshes/xbase_anim.kf',
                                        'knockdown: loop stop -> stop', 'knockout: loop stop -> stop']

    def run(item):
        scene, control = item
        title, body, impulse, wound, description = SCENES[scene]
        name = 'RXN-01-' + scene + ('-control' if control else '')
        command = [str(exe), '--rxn01-scene', scene, '--native-body', str(context), str(profile),
                   '--seconds', '20', '--speed', '30', '--rate', '60', '--trace', str(out),
                   '--name', name, 'mesh_ground=true']
        if control:
            command.append('--rxn01-reactions-off')
        if args.recovery_archive:
            command += ['--rxn01-recovery-archive', str(args.recovery_archive.resolve())]
        proc = subprocess.run(command, capture_output=True, text=True)
        (out / (name + '.log')).write_text(proc.stdout + proc.stderr, encoding='utf-8')
        summary = {}
        for line in reversed(proc.stdout.splitlines()):
            if line.strip().startswith('{'):
                summary = json.loads(line)
                break
        metrics = list(csv.DictReader((out / (name + '-reactions.csv')).open(newline='')))
        main_rows = list(csv.DictReader((out / (name + '.csv')).open(newline='')))
        body_rows = list(csv.DictReader((out / (name + '-bodies.csv')).open(newline='')))
        steps = [int(r['step']) for r in metrics]
        checks = {
            'runner': proc.returncode == 0 and summary.get('checks_passed') is True,
            'complete_60hz': len(steps) == 1200 and steps == list(range(1200)),
            'aligned_metrics': steps == [int(r['step']) for r in main_rows],
            'complete_bodies': len(body_rows) == len(steps) * len(names)
                and {(int(r['step']), int(r['body'])) for r in body_rows}
                == {(s, b) for s in steps for b in range(len(names))},
            'one_impulse': sum(int(r['impact']) for r in metrics) == 1,
            'single_receipt': sum(int(r['served_receipt']) != 0 for r in metrics)
                == (1 if wound is not None and not control else 0),
            'bounded_strength': all(0 <= float(r['stabilization_strength']) <= 1 for r in metrics),
            'bounded_speed': all(0 <= float(r['speed_scale']) <= 1 for r in metrics),
            'authored_motor_budget': all(0 <= float(r['motor_budget_ratio']) <= 1.00001 for r in metrics),
        }
        if scene == 'knockdown' and not control:
            checks['fall_recover_resume'] = summary.get('fallen') and summary.get('resumed_walking')
            checks['motors_off_control'] = summary.get('passive_fell') and not summary.get('passive_recovered')
        if args.recovery_archive:
            checks['root_ownership_boundary'] = all(
                r['root_kinematic'] != '1' or (i > 0 and main_rows[i-1]['mode'] in ('5', '6'))
                for i, r in enumerate(main_rows))
            checks['single_root_owner'] = all(r['root_kinematic'] != '1' or r['upright_applied'] == '0'
                                              for r in main_rows)
            if scene == 'knockdown' and not control:
                checks['kinematic_recovery_exercised'] = any(r['root_kinematic'] == '1' for r in main_rows)
        file_names = [name + suffix for suffix in ('.csv', '-bodies.csv', '-reactions.csv')]
        metadata = {
            'schema_version': 1, 'task': 'RXN-01', 'scene': scene,
            'title': title + (' · reactions off' if control else ''),
            'description': description,
            'scope': ('Offline hybrid review; visual acceptance pending. Physical Gait walking, kinematic pelvis '
                      'during native-clip recovery; dynamic limbs; 60 Hz. Not native gameplay parity.'
                      if args.recovery_archive else
                      'Physical review fixture; visual acceptance pending. Dynamic captured body; 60 Hz; triangle mesh.'),
            'recovery_controller': 'KinematicRoot / game clips' if args.recovery_archive else 'Dynamic legacy recovery',
            'injury_note': f'Explicit fixture severity {wound}; not computed RPG damage.' if wound is not None else 'No injury receipt.',
            'impact_step': 120, 'impact_body': body, 'impact_body_index': names.index(body),
            'impact_magnitude': impulse, 'reactions_enabled': not control,
            'compare_trace': (out / ('RXN-01-' + scene + ('' if control else '-control') + '-bodies.csv')).relative_to(root.parent).as_posix(),
            'body_names': names, 'parents': parents, 'provenance': provenance,
            'files': {f: digest(out / f) for f in file_names},
            'checks': checks, 'summary': summary, 'command': command,
        }
        write_json(out / (name + '-scene.json'), metadata)
        return {'scene': scene, 'control': control, 'checks': checks, 'summary': summary,
                'metadata_sha256': digest(out / (name + '-scene.json')), 'artifact_hashes': metadata['files'],
                'trace': (out / (name + '-bodies.csv')).relative_to(root.parent).as_posix()}

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, [(scene, control) for scene in SCENES for control in (False, True)]))
    # Inspect solved body motion against the identical-impulse control. These
    # are readability diagnostics; the user still decides visual acceptance.
    for scene in EXPRESSION_SCENES:
        measurements = measure(out, scene, out)
        for result in (r for r in results if r['scene'] == scene):
            name = 'RXN-01-' + scene + ('-control' if result['control'] else '')
            metadata_path = out / (name + '-scene.json')
            metadata = json.loads(metadata_path.read_text())
            metadata['expression_measurements'] = measurements
            metadata['review_peak_step'] = measurements['peak_frame']
            write_json(metadata_path, metadata)
            result['metadata_sha256'] = digest(metadata_path)
            result['expression_measurements'] = measurements
    failed = [r['trace'] for r in results if not all(r['checks'].values())]
    report = {'provenance': provenance, 'results': results, 'failures': failed}
    write_json(out / 'RXN-01-scenes.json', report)
    if args.snapshot:
        write_json(args.snapshot.resolve(), report)
    lines = ['RXN-01 physical review pack (visual acceptance pending)', f'Source: {commit}',
             f'Dirty source: {dirty}', f'Traces: {len(results)}; failed diagnostic traces: {len(failed)}']
    for r in results:
        lines.append(f"{r['scene']}{' / reactions off' if r['control'] else ''}: "
                     f"checks={all(r['checks'].values())}, behavior_frames={r['summary'].get('behavior_frames')}, "
                     f"fallen={r['summary'].get('fallen')}, resumed={r['summary'].get('resumed_walking')}")
    (out / 'SUMMARY.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))
    return int(bool(failed))


if __name__ == '__main__':
    raise SystemExit(main())
