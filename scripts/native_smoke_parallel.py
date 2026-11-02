"""NATIVE-04: isolated integration-test shards under the caller's live lease."""
from concurrent.futures import ThreadPoolExecutor
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): digest.update(block)
    return digest.hexdigest()


def balanced_shards(selected, count, timings=None):
    """Stable longest-processing-time assignment, based only on prior serial data."""
    timings = timings or {}
    if not selected or len(set(selected)) != len(selected):
        raise ValueError('selection must be nonempty and unique')
    count = min(count, len(selected))
    shards, loads = [[] for _ in range(count)], [0.0] * count
    for key in sorted(selected, key=lambda k: (-timings.get(k, 1.0), k)):
        target = min(range(count), key=lambda i: (loads[i], i))
        shards[target].append(key)
        loads[target] += timings.get(key, 1.0)
    return [sorted(shard) for shard in shards]


def merge_results(selected, reports):
    expected = set(selected)
    observed, errors, suites = {}, [], {}
    for shard, report in reports:
        if report is None:
            errors.append(f'{shard}: missing result summary')
            continue
        for suite in report['suites']:
            target = suites.setdefault(suite['name'], {'name': suite['name'], 'passed': True,
                'matched_tests': 0, 'cases': [], 'found_tests': [], 'fatal_errors': []})
            target['passed'] = target['passed'] and suite['passed']
            target['fatal_errors'].extend(suite.get('fatal_errors', []))
            for case in suite.get('cases', []):
                key = (suite['name'], case['name'])
                if key in observed: errors.append(f'duplicate case: {key}')
                if key not in expected: errors.append(f'unexpected case: {key}')
                observed[key] = case['status']
                target['cases'].append(dict(case, shard=shard))
            if suite['matched_tests'] != len(suite.get('cases', [])):
                errors.append(f'{shard}: case count disagrees with matched_tests')
    for key in sorted(expected - set(observed)): errors.append(f'missing case: {key}')
    for key, status in observed.items():
        if status not in ('passed', 'failed'): errors.append(f'incomplete case: {key}')
    for suite in suites.values():
        suite['cases'].sort(key=lambda c: c['name'])
        suite['matched_tests'] = len(suite['cases'])
        suite['passed'] = suite['passed'] and all(c['status'] == 'passed' for c in suite['cases'])
    return sorted(suites.values(), key=lambda s: s['name']), errors


def run_parallel(args):
    from integration_tests import discover_suites
    if args.video_directory:
        raise ValueError('parallel smoke does not share the bounded video recorder')
    if args.list_tests or args.case_names:
        raise ValueError('parallel coordination requires ordinary test selection')
    started = time.perf_counter()
    root = Path(args.workdir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    suites = discover_suites([Path(p).resolve() for p in args.suites])
    source = Path(__file__).resolve().parents[1]
    source_commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    engine_hash = file_hash(args.omw)

    def launch(directory, selected=None):
        directory.mkdir(parents=True, exist_ok=False)
        command = [sys.executable, str(Path(__file__).with_name('integration_tests.py'))]
        command += [str(suites[name]) for name in sorted({s for s, _ in selected} if selected else suites)]
        command += ['--omw', str(Path(args.omw).resolve()), '--workdir', str(directory),
                    '--render_fps', str(args.render_fps)]
        for option in ('config', 'ini'):
            if getattr(args, option): command += ['--' + option, str(Path(getattr(args, option)).resolve())]
        for path in args.data: command += ['--data', str(Path(path).resolve())]
        for option in ('suite_timeout', 'physics_threads', 'test_filter', 'diagnostic_budget_mib'):
            value = getattr(args, option)
            if value is not None: command += ['--' + option, str(value)]
        if args.native_mouse: command.append('--native_mouse')
        if args.renderer_stats: command.append('--renderer_stats')
        if selected is None:
            command.append('--list_tests')
        else:
            names = directory / 'selected-tests.json'
            names.write_text(json.dumps([name for _, name in selected]), encoding='utf-8')
            command += ['--case_names', str(names)]
        environment = dict(os.environ)
        environment.update(SDL_WINDOW_NO_ACTIVATION_WHEN_SHOWN='1', SDL_HINT_FORCE_RAISEWINDOW='0',
                           OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR=str(directory / 'test-results/replays'))
        Path(environment['OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR']).mkdir(parents=True)
        # Per-tick manual recordings are per process; shards must not share one.
        if environment.get('OPENMW_MANUAL_DIAGNOSTICS'):
            environment['OPENMW_MANUAL_DIAGNOSTICS'] = str(directory / 'manual-recording')
        if environment.get('OPENMW_RAGDOLL_REPLAY_SHAPE_DIR'):
            environment['OPENMW_RAGDOLL_REPLAY_SHAPE_DIR'] = str(directory / 'test-results/shapes')
            Path(environment['OPENMW_RAGDOLL_REPLAY_SHAPE_DIR']).mkdir()
        with (directory / 'launcher.log').open('w', encoding='utf-8') as log:
            process = subprocess.run(command, cwd=directory, env=environment, stdout=log, stderr=subprocess.STDOUT)
        summary = directory / 'result-summary.json'
        return process.returncode, json.loads(summary.read_text(encoding='utf-8')) if summary.is_file() else None

    exit_code, discovery = launch(root / 'discovery')
    if exit_code or discovery is None:
        raise ValueError('test discovery failed; see discovery/launcher.log')
    selected = [(s['name'], name) for s in discovery['suites'] for name in s['found_tests']]
    if len({name for _, name in selected}) != len(selected):
        raise ValueError('parallel selection requires unique test names across suites')
    timings = {}
    if args.timing_report:
        prior = json.loads(Path(args.timing_report).read_text(encoding='utf-8'))
        for suite in prior['suites']:
            for case in suite['cases']:
                duration = case.get('duration_ms')
                if isinstance(duration, (float, int)) and duration > 0:
                    timings[(suite['name'], case['name'])] = duration
    shards = balanced_shards(selected, args.parallel, timings)
    plan = [{'index': i, 'tests': [list(key) for key in shard]} for i, shard in enumerate(shards)]
    (root / 'shard-plan.json').write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')
    with ThreadPoolExecutor(max_workers=len(shards)) as pool:
        futures = [pool.submit(launch, root / f'shard-{i}', shard) for i, shard in enumerate(shards)]
        completed = []
        for i, future in enumerate(futures):
            try:
                code, report = future.result()
            except Exception as error:
                code, report = -1, None
                (root / f'shard-{i}-error.txt').write_text(str(error), encoding='utf-8')
            completed.append((code, report))
    merged, errors = merge_results(selected, [(f'shard-{i}', report) for i, (_, report) in enumerate(completed)])
    identities = []
    for i, (code, report) in enumerate(completed):
        directory = root / f'shard-{i}'
        if code: errors.append(f'shard-{i}: process exit {code}')
        if report is None: continue
        manifest = json.loads((directory / 'run-manifest.json').read_text(encoding='utf-8'))
        if report['source_commit'] != source_commit or manifest['engine_sha256'] != engine_hash:
            errors.append(f'shard-{i}: source or engine identity mismatch')
        identities.append({'index': i, 'directory': str(directory), 'exit_code': code,
                           'summary_sha256': file_hash(directory / 'result-summary.json'),
                           'manifest_sha256': file_hash(directory / 'run-manifest.json')})
    failed = sum(not s['passed'] for s in merged) + len(errors)
    summary = {'schema_version': 1, 'generated': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
               'source_commit': source_commit, 'manifest': 'run-manifest.json', 'suites': merged,
               'parallel': {'requested': args.parallel, 'actual': len(shards), 'selected_tests': [list(k) for k in selected],
                            'shards': identities, 'errors': errors, 'wall_seconds': time.perf_counter() - started},
               'tracker': {'producer': 'integration_tests.py', 'total_checks': len(selected), 'failed_checks': failed}}
    manifest = {'engine': str(Path(args.omw).resolve()), 'engine_sha256': engine_hash, 'arguments': vars(args),
                'physics_fps_environment': os.environ.get('OPENMW_PHYSICS_FPS'),
                'source_sha256': {str(Path(__file__)): file_hash(__file__)}, 'shard_plan_sha256': file_hash(root / 'shard-plan.json')}
    (root / 'run-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    (root / 'result-summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(f'Parallel smoke: {len(selected)} tests, {len(shards)} shards, {failed} failures; {summary["parallel"]["wall_seconds"]:.3f}s')
    for error in errors: print(error)
    return int(failed != 0)
