"""Run the frozen PLAY-04 four-cell drive diagnostic; never promote a result."""
import argparse
from datetime import datetime, timezone, timedelta
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--engine-source', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deadline', required=True, help='UTC ISO-8601 stop time')
    args = parser.parse_args()
    deadline = datetime.fromisoformat(args.deadline.replace('Z', '+00:00'))
    if deadline.tzinfo is None:
        parser.error('deadline must include a time zone')
    engine = args.engine.resolve()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    cells = [
        {'name': 'f60-d1-default', 'drive': '4,60,1,1,1', 'default': True},
        {'name': 'f35-d1', 'drive': '4,35,1,1,1', 'default': False},
        {'name': 'f35-d2p4', 'drive': '4,35,2.4,1,1', 'default': False},
        {'name': 'f60-d2p4', 'drive': '4,60,2.4,1,1', 'default': False},
    ]
    frozen = {
        'task': 'PLAY-04', 'scope': 'Diagnostic screening only; no promotion or tracker acceptance',
        'engine_source': args.engine_source, 'engine_sha256': digest(engine),
        'recipe_sha256': digest(Path(__file__)), 'fps': 60,
        'filter': '^PLAY04 quick and held iron spear input contract$',
        'deadline': deadline.isoformat(), 'cells': cells,
    }
    (root / 'frozen-batch.json').write_text(json.dumps(frozen, indent=2) + '\n')
    results = []
    runner = Path(__file__).with_name('run_native.py')
    for cell in cells:
        # Keep time for the private-desktop runner to finish and clean up.
        if datetime.now(timezone.utc) + timedelta(seconds=120) >= deadline:
            break
        print('SCREEN ' + cell['name'], flush=True)
        output = root / cell['name']
        command = [sys.executable, str(runner), '--engine', str(engine),
                   '--engine-source', args.engine_source, '--output', str(output),
                   '--suite', 'test_combat_fixes', '--test-filter', frozen['filter'],
                   '--render-fps', '60', '--diagnostic-budget-mib', '128',
                   '--env', 'OPENMW_RAGDOLL_ATTACK_DRIVE=' + cell['drive']]
        run = subprocess.run(command, capture_output=True, text=True, errors='replace')
        (root / (cell['name'] + '.runner.log')).write_text(run.stdout + run.stderr)
        result = dict(cell, exit_code=run.returncode, eligible=False, artifacts=[])
        summary_path = output / 'result-summary.json'
        cases = []
        if summary_path.exists():
            summary = json.loads(summary_path.read_text())
            cases = [case for suite in summary['suites'] for case in suite['cases']]
            result['cases'] = cases
        measurements = []
        for path in output.rglob('*.log.gz'):
            with gzip.open(path, 'rt', errors='replace') as stream:
                for line in stream:
                    match = re.search(r'PLAY04_INPUT_SUMMARY weapon=iron spear failures=(\d+) '
                                      r'max_grip=([\d.]+) max_support=([\d.]+)', line)
                    if match:
                        measurements.append(tuple(map(float, match.groups())))
        if len(measurements) == 1:
            failures, grip, support = measurements[0]
            result.update(failures=int(failures), max_grip=grip, max_support=support,
                          score=max(grip, support))
            result['eligible'] = (run.returncode == 0 and len(cases) == 1
                                  and cases[0]['status'] == 'passed' and failures == 0
                                  and grip <= 1 and support <= 1)
        for path in sorted(output.rglob('*')):
            if path.is_file() and (path.name.endswith('.log.gz')
                                  or path.name in ('result-summary.json', 'diagnostic-manifest.json')):
                result['artifacts'].append({'path': str(path.relative_to(root)), 'sha256': digest(path)})
        results.append(result)
        print(json.dumps({key: result.get(key) for key in
                          ('name', 'eligible', 'max_grip', 'max_support', 'exit_code')}), flush=True)
    survivors = sorted((r for r in results if r['eligible']),
                       key=lambda r: (r['score'], not r['default']))
    selected = survivors[0]['drive'] if len(results) == len(cells) and survivors else None
    report = dict(frozen, completed_cells=len(results), results=results,
                  selected_for_holdout=selected, promotion_allowed=False)
    (root / 'screening-summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Selected for hold-out: ' + str(selected), flush=True)
    return 0 if selected else 1


if __name__ == '__main__':
    sys.exit(main())
