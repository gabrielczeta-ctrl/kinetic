"""Pelvis shake check: angular speed of the player's (and bump opponent's)
pelvis per segment. A healthy standing body is near 0-20 deg/s; a value that
flips sign every physics step at hundreds of deg/s is numerical ringing.

usage: shake_report.py RUN_DIR [RUN_DIR ...]
"""
import gzip
import json
import math
import sys
from collections import defaultdict
from pathlib import Path


def speed(v):
    return math.degrees(math.sqrt(sum(x * x for x in v)))


def main(runs):
    for run in runs:
        seg = defaultdict(list)
        flips = defaultdict(lambda: [0, 0])
        previous = {}
        for path in sorted(Path(run).glob('shard-*/*.player-diagnostics.jsonl.gz')):
            with gzip.open(path, 'rt', encoding='utf-8') as stream:
                for line in stream:
                    r = json.loads(line)
                    if r.get('kind') != 'frame':
                        continue
                    pelvis = ((r.get('state') or {}).get('bodies') or {}).get('pelvis')
                    for who, body in (('player', pelvis), ('opponent', r.get('opponent'))):
                        if not body or not isinstance(body.get('angularVelocity'), list):
                            continue
                        w = body['angularVelocity']
                        key = (who, r.get('segment'))
                        seg[key].append(speed(w))
                        prev = previous.get((who, r.get('test')))
                        if prev is not None and speed(w) > 30:
                            flips[key][1] += 1
                            if sum(a * b for a, b in zip(prev, w)) < 0:
                                flips[key][0] += 1
                        previous[(who, r.get('test'))] = w
        print(Path(run).name)
        for key in sorted(seg, key=str):
            a = sorted(seg[key])
            f = flips[key]
            print(f"  {key[0]:>8} {str(key[1]):<14} p50 {a[len(a) // 2]:5.0f} p90 {a[int(0.9 * (len(a) - 1))]:5.0f} deg/s"
                  f"  sign flips {f[0]}/{f[1]}")


if __name__ == '__main__':
    main(sys.argv[1:])
