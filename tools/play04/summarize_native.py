"""Index preserved PLAY-04 native diagnostics without qualifying a release."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = []
    for result in sorted(args.workspace.glob('runtime-PLAY-04-guided-*/result-summary.json')):
        manifest_path = result.parent / 'diagnostic-manifest.json'
        if not manifest_path.exists():
            continue
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        data = json.loads(result.read_text(encoding='utf-8'))
        cases = [case for suite in data['suites'] for case in suite.get('cases', [])]
        runs.append({
            'path': result.parent.name,
            'engine_source': manifest['engine_source'],
            'engine_sha256': manifest['engine_sha256'],
            'script_source': manifest['script_source_commit'],
            'fps': manifest['render_fps'],
            'environment': manifest.get('environment', {}),
            'passed': sum(case['status'] == 'passed' for case in cases),
            'total': len(cases),
            'failures': [{'name': case['name'], 'error': case.get('error')}
                         for case in cases if case['status'] != 'passed'],
            'fatal_errors': [error for suite in data['suites'] for error in suite.get('fatal_errors', [])],
            'result_sha256': digest(result),
            'manifest_sha256': digest(manifest_path),
        })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({
        'task': 'PLAY-04',
        'scope': 'Scoped native diagnostics only; no release qualification or tracker acceptance',
        'runs': runs,
    }, indent=2) + '\n', encoding='utf-8')
    print(f'Indexed {len(runs)} preserved native runs in {args.output}')


if __name__ == '__main__':
    main()
