"""Map native observations to replay physics and following commands.

offline.csv's step indexes include driver warm-up. Body state at that step is
post-solve, while reference_stage and startup-commands.csv describe actuation
for that solve. A state transition observed after solving affects the NEXT
command. Use recorder frame pairs and driver warmup_frames, never guess offsets.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path


def frame_map(log_text, verification):
    driver = verification['driver_result']
    count, warmup = driver['frames'], driver['warmup_frames']
    if not isinstance(count, int) or count <= 0 or not isinstance(warmup, int) or warmup < 0:
        raise ValueError('Invalid driver frame count or warm-up')
    pairs = {}
    for line in log_text.splitlines():
        if 'RAGDOLL_REPLAY_FRAME ' not in line:
            continue
        fields = dict(re.findall(r'(\w+)=([^ ]+)', line))
        step, frame = int(fields['step']), int(fields['frame'])
        if step in pairs:
            raise ValueError('Multiple captures or duplicate native step')
        pairs[step] = frame
    if sorted(pairs.values()) != list(range(count)):
        raise ValueError('Recorder coverage differs from verified capture')
    return {step: {
        'native_observation_step': step,
        'capture_frame': frame,
        'physical_scenario_step': frame + warmup,
        'following_command_scenario_step': frame + warmup + 1 if frame + 1 < count else None,
    } for step, frame in pairs.items()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('native_log', type=Path)
    parser.add_argument('verification_json', type=Path)
    parser.add_argument('--native-step', type=int, action='append', required=True)
    args = parser.parse_args()
    verification = json.loads(args.verification_json.read_text())
    raw_log = args.native_log.read_bytes()
    if hashlib.sha256(raw_log).hexdigest() != verification['sha256']['native_log']:
        raise ValueError('Native log is not the one identified by this verification')
    mapping = frame_map(raw_log.decode(errors='replace'), verification)
    print(json.dumps([mapping[step] for step in args.native_step], indent=2))
