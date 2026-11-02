"""Summarize captured weapon motion without turning diagnostic coverage into a gate."""
import argparse
import json
import math
from pathlib import Path
import statistics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path)
args = parser.parse_args()
rows = [json.loads(line) for path in sorted((args.run / 'test-results').glob('*.telemetry.jsonl'))
        for line in path.read_text().splitlines() if line.strip()]
rows = [r for r in rows if r.get('scenario') == 'play04_handling']
groups = {}
for row in rows:
    groups.setdefault((row['weapon'], row['phase']), []).append(row)
summary = []
for (weapon, phase), group in groups.items():
    result = {'weapon': weapon, 'phase': phase, 'frames': len(group),
              'modes': sorted(set(r['mode'] for r in group))}
    for prefix in ('hand', 'request', 'drive', 'endpoint'):
        present = [r for r in group if prefix + '_x' in r]
        if present:
            result[prefix + '_start'] = [round(present[0][prefix + '_' + a], 2) for a in 'xyz']
            result[prefix + '_end'] = [round(present[-1][prefix + '_' + a], 2) for a in 'xyz']
            result[prefix + '_range'] = [round(max(r[prefix + '_' + a] for r in present)
                                              - min(r[prefix + '_' + a] for r in present), 2) for a in 'xyz']
    for name in ('ik_error', 'tracking_error', 'grip_error', 'grip_angle', 'contacts'):
        values = [r[name] for r in group if name in r]
        if values:
            result[name + '_max'] = round(max(values), 3)
            result[name + '_median'] = round(statistics.median(values), 3)
    tracking = [math.sqrt(sum((r['drive_' + a] - r['hand_' + a]) ** 2 for a in 'xyz')) for r in group]
    result['drive_hand_error_median'] = round(statistics.median(tracking), 3)
    summary.append(result)
output = {'scope': 'diagnostic motion measurements; not full PLAY-04 acceptance',
          'frame_count': len(rows), 'phases': summary}
(args.run / 'handling-summary.json').write_text(json.dumps(output, indent=2))
for r in summary:
    print(r['weapon'], r['phase'], 'hand', r['hand_end'], 'request', r.get('request_end'),
          'hand_range', r['hand_range'], 'drive_error', r['drive_hand_error_median'],
          'ik', r.get('ik_error_median'), 'grip', r.get('grip_error_max'))
