#!/usr/bin/env python3
"""Fixed RXN-01 animation/PGC handoff diagnostic; does not award tracker acceptance."""
import argparse
import concurrent.futures
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--walksearch', required=True, type=Path)
    parser.add_argument('--recovery-archive', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--viewer-out', type=Path)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--preview-case', help='Explicit impulse:trial showcase; never changes the four-direction verdict')
    args = parser.parse_args()
    root, exe, out = args.root.resolve(), args.walksearch.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    context = root / 'files/data/ragdoll/calibration/context-symP.creation.txt'
    profile = root / 'files/data/ragdoll/humanoid-v1.yaml'
    names = [b['name'] for b in yaml.safe_load(profile.read_text())['bodies']]
    parents = [names.index(b['parent']) if b.get('parent') else -1
               for b in yaml.safe_load(profile.read_text())['bodies']]
    provenance = {
        'source_commit': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
        'working_tree_dirty': bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True).strip()),
        'source_diff_sha256': hashlib.sha256(subprocess.check_output(['git', '-C', str(root), 'diff', 'HEAD'])).hexdigest(),
        'binary_sha256': digest(exe), 'profile_sha256': digest(profile), 'context_sha256': digest(context),
        'archive_sha256': digest(args.recovery_archive), 'walking_clip': 'meshes/xbase_anim.kf: walkforwardhh',
    }

    def run(item):
        impulse, trial, control = item
        name = f'RXN-01-hybrid-{impulse}-{trial}' + (f'-{control}' if control else '')
        command = [str(exe), '--rxn01-scene', 'hybrid_catch', '--native-body', str(context), str(profile),
                   '--rxn01-recovery-archive', str(args.recovery_archive.resolve()),
                   '--rxn01-hybrid-impulse', str(impulse), '--rxn01-trial', str(trial),
                   '--seconds', '20', '--speed', '30', '--rate', '60', '--trace', str(out),
                   '--name', name, 'mesh_ground=true']
        if control:
            command += ['--rxn01-hybrid-control', control]
        proc = subprocess.run(command, capture_output=True, text=True)
        (out / (name + '.log')).write_text(proc.stdout + proc.stderr, encoding='utf-8')
        summary = next((json.loads(line) for line in reversed(proc.stdout.splitlines())
                        if line.strip().startswith('{')), {})
        result = {'name': name, 'impulse': impulse, 'trial': trial, 'control': control,
                  'command': command, 'runner_summary': summary, 'returncode': proc.returncode}
        paths = [out / (name + suffix) for suffix in ('.csv', '-bodies.csv', '-reactions.csv')]
        if not all(p.exists() for p in paths):
            return result | {'checks': {'artifacts': False}, 'passed': False}
        with paths[0].open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        with paths[1].open(newline='') as stream:
            bodies = list(csv.DictReader(stream))
        with paths[2].open(newline='') as stream:
            reactions = list(csv.DictReader(stream))
        hit = 270 + trial
        returned = next((i for i, r in enumerate(rows) if r['hybrid_stage'] == '3'), None)
        physical = [i for i, r in enumerate(rows) if i >= hit and r['root_kinematic'] == '0'
                    and r['reference_stage'] != '']
        checks = {
            'runner': proc.returncode == 0 and summary.get('checks_passed') is True,
            'complete_60hz': len(rows) == 1200 and [int(r['step']) for r in rows] == list(range(1200)),
            'complete_bodies': len(bodies) == len(rows) * len(names),
            'aligned_reactions': [r['step'] for r in reactions] == [r['step'] for r in rows],
            'animated_before_hit': all(r['root_kinematic'] == '1' for r in rows[:hit]),
            'single_root_owner': all(r['root_kinematic'] != '1' or
                (r['upright_applied'] == '0' and r['reference_stage'] == '') for r in rows),
            'authored_motor_budget': all(0 <= float(r['motor_budget_ratio']) <= 1.00001 for r in reactions),
        }
        if control == 'no-hit':
            checks.update(no_hit=sum(int(r['impact']) for r in reactions) == 0,
                          stays_kinematic=all(r['root_kinematic'] == '1' for r in rows),
                          no_physical_catch=not physical and returned is None)
        else:
            checks.update(one_hit=sum(int(r['impact']) for r in reactions) == 1,
                          released_on_hit=rows[hit]['root_kinematic'] == '0',
                          velocity_preserved=rows[hit]['root_release_velocity_error'] != '' and
                              float(rows[hit]['root_release_velocity_error']) <= 1e-6)
            if control == 'motors-off':
                checks.update(no_reacquisition=all(r['root_kinematic'] == '0' for r in rows[hit:]),
                              passive_falls=any(r['mode'] in ('3', '4') for r in rows[hit:]))
            else:
                checks.update(physical_gait_observed=bool(physical),
                              supported_return=returned is not None and returned >= hit + 60 and
                                  rows[returned-1]['left_supported'] == '1' and
                                  rows[returned-1]['right_supported'] == '1',
                              no_fall=all(r['mode'] not in ('3', '4', '5', '6') for r in rows),
                              finishes_kinematic_walk=rows[-1]['root_kinematic'] == '1' and
                                  rows[-1]['hybrid_stage'] == '1')
        root_bodies = [b for b in bodies if b['body'] == '0']
        distance = lambda a, b: sum((float(a[k])-float(b[k]))**2 for k in ('x', 'y'))**0.5
        result.update(checks=checks, passed=all(checks.values()), first_physical_step=physical[0] if physical else None,
                      reacquire_step=returned, physical_plan_frames=len(physical),
                      physical_displacement=distance(root_bodies[hit], root_bodies[returned-1]) if returned else None,
                      resumed_displacement=distance(root_bodies[returned], root_bodies[-1]) if returned else None,
                      files={p.name: digest(p) for p in paths})
        if not control:
            checks['physical_displacement'] = returned is not None and result['physical_displacement'] > 1
            checks['resumed_displacement'] = returned is not None and result['resumed_displacement'] > 30
            result['passed'] = all(checks.values())
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, [(i, t, '') for i in (600, 1000, 1400) for t in (0, 4, 8, 12)]))
    selected = max((i for i in (600, 1000, 1400) if all(r['passed'] for r in results if r['impulse'] == i)), default=None)
    preview = tuple(map(int, args.preview_case.split(':'))) if args.preview_case else None
    representative = next((r for r in results if r['impulse'] == selected and r['trial'] == 0), None)
    if representative is None and preview:
        representative = next((r for r in results if (r['impulse'], r['trial']) == preview and r['passed']), None)
    control_case = (representative['impulse'], representative['trial']) if representative else (1000, 0)
    controls = [run((*control_case, c)) for c in ('no-hit', 'motors-off')]
    passed = selected is not None and all(c['passed'] for c in controls)
    report = {'schema_version': 1, 'task': 'RXN-01', 'scope': 'Offline hybrid catch diagnostic; not native acceptance',
              'provenance': provenance, 'selected_impulse': selected, 'passed': passed, 'runs': results, 'controls': controls,
              'preview_case': representative['name'] if representative else None,
              'preview_only': representative is not None and selected is None}
    if args.viewer_out and representative and all(c['passed'] for c in controls):
        viewer = args.viewer_out.resolve()
        viewer.mkdir(parents=True, exist_ok=True)
        for r in [representative] + controls:
            for filename in r['files']:
                shutil.copy2(out / filename, viewer / filename)
            control = r['control']
            compare = representative if control else controls[0]
            metadata = {
                'schema_version': 1, 'task': 'RXN-01', 'scene': 'hybrid_catch',
                'title': 'KinematicRoot → PGC → KinematicRoot' + (f' · {control}' if control else ''),
                'description': 'Combat walk animation releases its pelvis on impact. Physical Gait catches the hit; supported, stable motion allows a matched blend back into walking.',
                'scope': ('Offline hybrid preview; actual game walk clip and bounded dynamic limbs. '
                          + ('Four-direction catch screen FAILED; this is a successful direction only. ' if selected is None else '')
                          + 'Native integration and visual acceptance pending.'),
                'injury_note': 'No injected injury. Motor-off and no-hit controls are available in Browse all recordings.',
                'reactions_enabled': not bool(control), 'impact_step': None if control == 'no-hit' else 270 + r['trial'],
                'impact_body': 'spine', 'impact_body_index': names.index('spine'), 'impact_magnitude': 0 if control == 'no-hit' else r['impulse'],
                'review_peak_step': representative['first_physical_step'], 'body_names': names, 'parents': parents,
                'compare_trace': viewer.name + '/' + compare['name'] + '-bodies.csv',
                'compare_label': 'Return: hit and catch' if control else 'Compare: no hit',
                'provenance': provenance, 'files': r['files'],
                'directional_screen_passed': passed,
            }
            write(viewer / (r['name'] + '-scene.json'), metadata)
    write(out / 'RXN-01-hybrid-screen.json', report)
    if args.snapshot:
        write(args.snapshot, report)
    print(json.dumps({'passed': passed, 'selected_impulse': selected,
                      'runs': [{k: r[k] for k in ('name', 'passed', 'checks', 'reacquire_step') if k in r} for r in results],
                      'controls': [{k: r[k] for k in ('name', 'passed', 'checks')} for r in controls]}))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
