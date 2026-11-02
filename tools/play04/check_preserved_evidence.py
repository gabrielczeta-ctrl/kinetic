"""Verify PLAY-04 raw inventories against both working files and Git blobs."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', default='HEAD', help='Git revision, or INDEX for staged blobs')
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[2]
    problems, count = [], 0
    for inventory in sorted((source / 'docs/ragdoll/evidence/raw').glob('PLAY-04-guided-20260922-*/artifacts.json')):
        for name, wanted in json.loads(inventory.read_text())['sha256'].items():
            path = inventory.parent / name
            count += 1
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != wanted:
                problems.append(f'Working file differs: {path}')
            relative = path.relative_to(source).as_posix()
            spec = ':' + relative if args.revision == 'INDEX' else args.revision + ':' + relative
            blob = subprocess.run(['git', '-c', 'safe.directory=' + source.as_posix(),
                                   '-C', str(source), 'show', spec], capture_output=True)
            if blob.returncode or hashlib.sha256(blob.stdout).hexdigest() != wanted:
                problems.append(f'Git {args.revision} blob differs: {relative}')
    if problems:
        raise SystemExit('\n'.join(problems))
    print(f'All {count} preserved artifacts match working files and Git {args.revision} bytes.')


if __name__ == '__main__':
    main()
