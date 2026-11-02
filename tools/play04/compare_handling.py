"""Evaluate the preregistered loaded-arm mechanism screen (not PLAY-04 acceptance)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

PHASES = ('rest', 'held', 'right', 'left', 'up', 'down', 'release')
WEAPONS = ('fists', 'iron dagger', 'iron longsword', 'iron warhammer')


def capture(directory):
    manifest = json.loads((directory / 'diagnostic-manifest.json').read_text())
    result = json.loads((directory / 'result-summary.json').read_text())
    if len(result['suites']) != 1 or not result['suites'][0]['passed']:
        raise ValueError(f'{directory}: capture tests did not pass')
    groups, hashes = {}, {}
    for path in sorted((directory / 'test-results').glob('*.telemetry.jsonl')):
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        for line in path.read_text().splitlines():
            row = json.loads(line)
            if row.get('scenario') != 'play04_handling':
                continue
            if any(isinstance(v, (float, int)) and not math.isfinite(v) for v in row.values()):
                raise ValueError(f'{directory}: non-finite telemetry')
            groups.setdefault((row['weapon'], row['phase']), []).append(row)
    for weapon in WEAPONS:
        for phase in PHASES:
            if len(groups.get((weapon, phase), [])) < 5:
                raise ValueError(f'{directory}: incomplete {weapon}/{phase} capture')
    return manifest, hashes, groups


def compare(baseline, candidate):
    bm, bh, bg = capture(baseline)
    cm, ch, cg = capture(candidate)
    for field in ('render_fps', 'physics_fps', 'script_sha256', 'environment'):
        if bm[field] != cm[field]:
            raise ValueError(f'Mismatched {field}: rerun the same fixture for both binaries')
    measurements, failures = [], []
    for weapon in WEAPONS:
        for phase in PHASES:
            old, new = bg[weapon, phase], cg[weapon, phase]
            metric = {'weapon': weapon, 'phase': phase}
            if any(r['mode'] != 'standing' for r in new):
                failures.append(f'{weapon}/{phase}: left standing')
            if weapon != 'fists':
                if any(not r['attached'] for r in new):
                    failures.append(f'{weapon}/{phase}: weapon detached')
                metric['grip_max'] = max(r['grip_error'] for r in new)
                # The known hammer grip offset is visible but outside this
                # response mechanism. Never silently turn it into a pass.
                if weapon != 'iron warhammer' and metric['grip_max'] >= 1:
                    failures.append(f'{weapon}/{phase}: grip separation >= 1')
            a = [r['tracking_error'] for r in old if 'tracking_error' in r]
            b = [r['tracking_error'] for r in new if 'tracking_error' in r]
            if a and b:
                before, after = statistics.median(a), statistics.median(b)
                metric.update(tracking_before=before, tracking_after=after)
                if weapon == 'iron longsword' and phase in ('left', 'up'):
                    if after > 0.75 * before:
                        failures.append(f'{weapon}/{phase}: less than 25% tracking improvement')
                elif weapon in ('fists', 'iron longsword') and phase != 'rest':
                    if after > 1.25 * before + 1:
                        failures.append(f'{weapon}/{phase}: tracking regression')
            elif phase not in ('rest',):
                failures.append(f'{weapon}/{phase}: missing tracking measurement')
            measurements.append(metric)
    return {
        'scope': 'PLAY-04 loaded-arm mechanism screen only; not task or manual acceptance',
        'packet': 'docs/ragdoll/designs/PLAY-04-loaded-arm-response-v1.md',
        'baseline_manifest': bm, 'candidate_manifest': cm,
        'baseline_raw_sha256': bh, 'candidate_raw_sha256': ch,
        'screen_passed': not failures, 'failures': failures, 'measurements': measurements,
        'remaining': ['Hammer grip offset', 'Full native input/movement/contact matrix',
                      'Agent visual review of rendered captures on the same candidate'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('candidate', type=Path)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate)
    (args.candidate / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    summary = ['Loaded-arm mechanism screen: ' + ('PASS' if result['screen_passed'] else 'FAIL'),
               result['scope']]
    for row in result['measurements']:
        if row['weapon'] == 'iron longsword' and 'tracking_before' in row:
            summary.append(f"{row['phase']}: median measured tracking error "
                           f"{row['tracking_before']:.3f} -> {row['tracking_after']:.3f}")
    summary.extend('FAIL: ' + failure for failure in result['failures'])
    summary.extend('Remaining: ' + item for item in result['remaining'])
    text = '\n'.join(summary) + '\n'
    (args.candidate / 'SUMMARY.txt').write_text(text)
    print(text, end='')
    return 0 if result['screen_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
