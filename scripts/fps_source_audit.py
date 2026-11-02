"""Validate a registered FPS authored timeline independently of physical outcomes."""
import argparse
import json
import math
from pathlib import Path
import re

from native_setup_errors import setup_error


def audit(path):
    plans, samples, errors = [], [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        if setup_error(line):
            errors.append('native script failure')
        fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
        if 'RAGDOLL_FPS_SOURCE_PLAN ' in line:
            plans.append(fields)
        elif 'RAGDOLL_FPS_SOURCE ' in line:
            samples.append(fields)
    if len(plans) != 1 or not samples:
        return {'source_valid': False, 'errors': ['missing or multiple source plans/samples'],
                'qualification_ready': False}
    plan = plans[0]
    scenario = plan.get('scenario')
    if scenario in ('S2','S2Straight'):
        expected = {'schema':'2','scenario':scenario,'start_step':'60','stop_step':'300','restart_step':'540','final_stop_step':'792','end_step':'1092'}
    else:
        expected = {'schema':'1','scenario':'S1','start_step':'60','stop_step':'660','end_step':'960'}
    if any(plan.get(key) != value for key, value in expected.items()):
        errors.append('source definition differs from registered scenario')
    ticks = []
    previous_tick = 0
    for index, sample in enumerate(samples):
        tick = int(sample['half_tick'])
        ticks.append(tick)
        movement = int(120 <= tick < (600 if scenario in ('S2','S2Straight') else 1320))
        yaw_change = 0.0
        if scenario in ('S2','S2Straight'):
            movement = int(bool(movement) or 1080 <= tick < 1584)
            def yaw(t): return (2 if scenario == 'S2' else 0) * min(0.84, max(0, (t - 1080) / 120))
            yaw_change = yaw(tick) - yaw(previous_tick)
            if not math.isclose(float(sample.get('authored_yaw','nan')), yaw(tick), rel_tol=0, abs_tol=1e-12):
                errors.append(f'authored yaw mismatch at sample {index}')
        previous_tick = tick
        if sample.get('epoch_half_tick') != plan.get('epoch_half_tick'):
            errors.append(f'epoch mismatch at sample {index}')
        if float(sample['movement']) != movement or not math.isclose(float(sample['yaw_change']), yaw_change, rel_tol=0, abs_tol=1e-12):
            errors.append(f'command mismatch at sample {index}')
    if ticks[0] > 120 or ticks[-1] < int(expected['end_step']) * 2:
        errors.append('source timeline incomplete')
    if any(b <= a for a, b in zip(ticks, ticks[1:])):
        errors.append('unordered or multiple-actor source samples')
    return {'source_valid': not errors, 'plan': plan, 'sample_count': len(samples),
            'first_half_tick': ticks[0], 'last_half_tick': ticks[-1],
            'errors': errors, 'qualification_ready': False,
            'scope': 'authored source only; native consumption and behavior require separate comparisons'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.log)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
