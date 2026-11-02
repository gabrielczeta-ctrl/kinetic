"""Locate discrete gait divergence using existing native traces, without tuning.

Usage: python fps_support_boundary_audit.py MANIFEST OUTPUT
The manifest contains isolated player runs at 60, 30 and 120 FPS.
Native step and capture frame are explicitly paired by the recorder. These
are not scenario CSV indices (which can include warm-up frames).
"""
import argparse
import json
import re
from pathlib import Path


def read_run(run):
    log = Path(run['directories'][0]) / 'test_ragdoll/openmw.log'
    frames, observations = {}, {}
    for line in log.read_text(errors='replace').splitlines():
        fields = dict(re.findall(r'(\w+)=([^ ]+)', line))
        if 'RAGDOLL_REPLAY_FRAME ' in line:
            step = int(fields['step'])
            assert step not in frames, 'multiple captures in log'
            frames[step] = int(fields['frame'])
        if 'RAGDOLL_STEP phase=observed ' in line and fields.get('player') == '1':
            assert fields['integer_radix'] == '10'
            step = int(fields['step'])
            assert step not in observations, 'multiple player observations'
            observations[step] = fields
    assert frames, 'no replay capture frames'
    assert sorted(frames.values()) == list(range(len(frames)))
    result = []
    for step, frame in sorted(frames.items(), key=lambda item: item[1]):
        fields = observations[step]
        result.append(dict(capture_frame=frame, native_step=step,
            stage=int(fields['reference_stage']), phase=int(fields['reference_phase']),
            stage_time=float(fields['reference_time']),
            left_load=float(fields['left_load']), right_load=float(fields['right_load']),
            left=int(fields['left']), right=int(fields['right'])))
    return result


def audit(manifest, start_frame):
    runs = json.loads(manifest.read_text())['runs']
    assert len(runs) == 3 and {r['fps'] for r in runs} == {30, 60, 120}
    traces = {run['fps']: read_run(run) for run in runs}
    result = dict(scope='diagnostic boundary, not acceptance or proof of fall causality',
        start_capture_frame=start_frame, comparisons=[])
    for fps in (30, 120):
        pair = next(((a, b) for a, b in zip(traces[60], traces[fps])
            if a['capture_frame'] >= start_frame
            and (a['stage'], a['phase']) != (b['stage'], b['phase'])), None)
        if pair is None:
            result['comparisons'].append(dict(fps=fps, first_boundary=None))
            continue
        a, b = pair
        assert a['capture_frame'] == b['capture_frame']
        frame = a['capture_frame']
        result['comparisons'].append(dict(fps=fps, first_boundary=frame,
            reference=a, candidate=b,
            reference_window=traces[60][max(0, frame-3):frame+4],
            candidate_window=traces[fps][max(0, frame-3):frame+4]))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start-frame', type=int, required=True)
    args = parser.parse_args()
    report = audit(args.manifest, args.start_frame)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    for comparison in report['comparisons']:
        print(comparison['fps'], comparison['first_boundary'],
              comparison.get('reference'), comparison.get('candidate'))
