"""Summarize observed player failures without inferring missing physics frames."""
import gzip
import json
from pathlib import Path


def summarize(path):
    tests = {}
    previous = {}
    previous_step = {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            name = row.get('test', 'unknown')
            result = tests.setdefault(name, {'frames': 0, 'skipped_steps': 0, 'omitted_between_windows': 0, 'transitions': [],
                                             'first_failure': None, 'windows': [], 'replay': None})
            if row['kind'] == 'window':
                result['windows'].append({k: row.get(k) for k in ('reason', 'buffered_samples', 'buffered_bytes')})
            elif row['kind'] == 'fixed_step_replay':
                result['replay'] = row
            elif row['kind'] == 'frame':
                result['frames'] += 1
                result['skipped_steps'] += row.get('skipped_steps', 0)
                step = row.get('physics_step')
                if step is not None and name in previous_step:
                    result['omitted_between_windows'] += max(0, step - previous_step[name] - 1 - row.get('skipped_steps', 0))
                if step is not None:
                    previous_step[name] = step
                state = row.get('state', {})
                controller = state.get('controller', {})
                mode = controller.get('mode', 'missing')
                transition = {
                    'physics_step': row.get('physics_step'), 'simulation_time': row['simulation_time'],
                    'mode': mode, 'input': row.get('input'), 'support': state.get('support'),
                    'pelvis_up': controller.get('pelvisUpDot'), 'heading_error': controller.get('headingError'),
                    'raw_velocity': controller.get('rawTargetHorizontalVelocity'),
                    'planned_velocity': controller.get('plannedHorizontalVelocity'),
                    'measured_velocity': controller.get('centerOfMassVelocity'),
                    'motor_utilization': controller.get('jointSupportMotorUtilization'),
                    'gait': state.get('experimentalWalking'),
                }
                if previous.get(name) != mode:
                    result['transitions'].append(transition)
                previous[name] = mode
                if result['first_failure'] is None and mode in ('missing', 'falling', 'fallen', 'getting_up', 'blend_back'):
                    result['first_failure'] = transition
    report = {'sampling': 'render-observed snapshots; ring windows may omit earlier history',
              'source': str(path), 'tests': tests}
    output = Path(str(path).replace('.jsonl.gz', '.summary.json'))
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    return output


if __name__ == '__main__':
    import sys
    print(summarize(Path(sys.argv[1])))
