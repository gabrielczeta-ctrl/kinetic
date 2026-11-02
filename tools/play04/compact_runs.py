"""Compress the tick recordings of automated PLAY-04 gate runs.

    python compact_runs.py [--keep N] [--dry-run] <dev root>

Every `runtime-PLAY-04-gates-*` run directory except the newest N (default 4)
has each `manual-recording/ticks-*.jsonl` replaced by `ticks-*.jsonl.gz`. The
compressed copy is read back and compared line by line before the original is
removed, so no data is lost; gates.py and make_replay.py read either form.
Manual play sessions (`runtime-PLAY-04-guided-manual-*`) are never touched.
A run of about 250 MB drops to about 25 MB.
"""
import argparse
import glob
import gzip
import os
import shutil


def compress(path):
    target = path + '.gz'
    with open(path, 'rb') as source, gzip.open(target, 'wb', compresslevel=6) as sink:
        shutil.copyfileobj(source, sink, 1 << 20)
    with open(path, 'rb') as a, gzip.open(target, 'rb') as b:
        while True:
            x, y = a.read(1 << 20), b.read(1 << 20)
            if x != y:
                os.remove(target)
                raise RuntimeError('verification failed: ' + path)
            if not x:
                break
    saved = os.path.getsize(path) - os.path.getsize(target)
    os.remove(path)
    return saved


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root')
    parser.add_argument('--keep', type=int, default=4)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    runs = [d for d in glob.glob(os.path.join(args.root, 'runtime-PLAY-04-gates-*')) if os.path.isdir(d)]
    runs.sort(key=os.path.getmtime, reverse=True)
    total = 0
    for run in runs[args.keep:]:
        for chunk in glob.glob(os.path.join(run, '**', 'manual-recording', 'ticks-*.jsonl'), recursive=True):
            if args.dry_run:
                total += os.path.getsize(chunk)
                continue
            total += compress(chunk)
    print('%s %.2f GB in %d runs' % ('would compress' if args.dry_run else 'saved', total / 1e9, max(0, len(runs) - args.keep)))


if __name__ == '__main__':
    main()
