#!/usr/bin/env python3
"""Bounded diagnostic screen for RXN-01 get-up; does not award PLAY-02."""
import argparse
import concurrent.futures
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess


def measure(path):
    rows = list(csv.DictReader(path.open(newline='')))
    starts = [i for i, r in enumerate(rows) if r['mode'] == '5' and rows[i-1]['mode'] != '5']
    start = starts[0] if starts else len(rows)
    end = next((i for i in range(start, len(rows)) if rows[i]['mode'] == '0'), len(rows)-1)
    interval = rows[start:end+1]
    bodies = list(csv.DictReader(path.with_name(path.stem + '-bodies.csv').open(newline='')))
    angles = {}
    for body in (0, 3):
        qs = [[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in bodies
              if int(r['body']) == body and start <= int(r['step']) <= end]
        qs = [[x/math.sqrt(sum(y*y for y in q)) for x in q] for q in qs]
        angles[str(body)] = math.degrees(sum(2*math.acos(min(1, abs(sum(x*y for x,y in zip(a,b)))))
                                           for a,b in zip(qs, qs[1:])))
    return {'attempts': len(starts), 'first_rise_step': start, 'first_standing_step': end,
            'completed': bool(starts) and end < len(rows)-1,
            'rise_seconds': (end-start)/60,
            'upward_com_speed': max((float(r['com_velocity_z']) for r in interval), default=0),
            'rotation_degrees': angles,
            'later_falls': sum(r['mode'] == '3' and rows[i-1]['mode'] != '3'
                               for i,r in enumerate(rows) if i > end),
            'final_mode': rows[-1]['mode'],
            'trace_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--exe', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--recovery-archive', type=Path)
    args = p.parse_args()
    root, exe, out = args.root.resolve(), args.exe.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    def run(cell):
        duration, trial = cell
        name = f'RXN-01-getup-{duration:g}-trial-{trial}' if not args.recovery_archive else f'RXN-01-kinematic-trial-{trial}'
        cmd = [str(exe), '--rxn01-episode', '--native-body',
               str(root/'files/data/ragdoll/calibration/context-symP.creation.txt'),
               str(root/'files/data/ragdoll/humanoid-v1.yaml'), '--seconds', '20', '--speed', '30',
               '--rate', '60', '--rxn01-trial', str(trial), '--rxn01-getup', str(duration),
               '--trace', str(out), '--name', name, 'mesh_ground=true']
        if args.recovery_archive:
            cmd += ['--rxn01-recovery-archive', str(args.recovery_archive.resolve())]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        (out/(name+'.log')).write_text(proc.stdout+proc.stderr)
        result = {'duration': duration, 'trial': trial, 'exit_code': proc.returncode, 'command': cmd}
        if (out/(name+'.csv')).exists(): result.update(measure(out/(name+'.csv')))
        return result
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(run, [(d,t) for d in ((0,) if args.recovery_archive else (0,2.4,3.2,4.0))
                                     for t in (0,4,8,12)]))
    report = {'scope': 'Offline diagnostic; not PLAY-02 acceptance',
              'provenance': {
                  'source_commit': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
                  'working_tree_dirty': bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True).strip()),
                  'source_diff_sha256': hashlib.sha256(subprocess.check_output(['git', '-C', str(root), 'diff', 'HEAD'])).hexdigest(),
                  'binary_sha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
                  'producer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
              'results': results}
    if args.recovery_archive:
        report['provenance']['recovery_archive_sha256'] = hashlib.sha256(args.recovery_archive.read_bytes()).hexdigest()
        report['scope'] = 'Offline kinematic-root diagnostic; external root ownership; not PLAY-02 or physical-gait acceptance'
    (out/'screen.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = []
    for r in results:
        lines.append(f"duration={r['duration']} trial={r['trial']} completed={r.get('completed')} "
                     f"attempts={r.get('attempts')} later_falls={r.get('later_falls')} "
                     f"rise={r.get('rise_seconds',0):.2f}s upward={r.get('upward_com_speed',0):.1f} "
                     f"rotation={r.get('rotation_degrees')} exit={r['exit_code']}")
    (out/'SUMMARY.txt').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    return int(any(r['exit_code'] != 0 for r in results))


if __name__ == '__main__': raise SystemExit(main())
