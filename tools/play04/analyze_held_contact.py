"""Describe held-sweep contact selection; this is not an acceptance producer."""
import argparse
import gzip
import hashlib
import json
import re
from pathlib import Path


def fields(line):
    return dict(re.findall(r'(\w+)=(.*?)(?= \w+=|$)', line.strip()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime', type=Path)
    args = parser.parse_args()
    contacts, windows, raw = [], {}, []
    for path in sorted(args.runtime.glob('*.log.gz')):
        raw.append({'path': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        for line in gzip.open(path, 'rt', encoding='utf-8', errors='replace'):
            if 'RAGDOLL_CONTACT_WORK ' in line:
                item = fields(line.split('RAGDOLL_CONTACT_WORK ', 1)[1])
                if 'selected' in item:
                    contacts.append(item)
            elif 'PLAY04_MEASURED_PHASE ' in line:
                item = fields(line.split('PLAY04_MEASURED_PHASE ', 1)[1])
                key = (item['weapon'], int(item['attempt']))
                window = windows.setdefault(key, {})
                window[item['phase']] = int(item['step'])
                if 'prior_injury' in item:
                    window['prior_injury'] = float(item['prior_injury'])
    attempts = []
    for (weapon, attempt), window in sorted(windows.items()):
        if not all(key in window for key in ('sweep_start', 'sweep_end')):
            raise ValueError(f'Incomplete sweep markers: {weapon} {attempt}')
        events = [item for item in contacts if item['weapon'] == weapon
                  and window['sweep_start'] < int(item['step']) <= window['sweep_end']]
        identities = {(item['attacker'], item['identity']) for item in events}
        earlier = [item for item in contacts if item['weapon'] == weapon
                   and int(item['step']) <= window['sweep_start']
                   and (item['attacker'], item['identity']) in identities
                   and int(item['selected']) > 0]
        attempts.append({
            'weapon': weapon, 'attempt': attempt, **window,
            'raw_contacts': len(events),
            'eligible_contacts': sum(int(item['eligible']) for item in events),
            'active_eligible_approach_at_least_one': sum(
                int(item['eligible']) and int(item['swing']) and float(item['approach']) >= 1
                for item in events),
            'selected_contacts': sum(int(item['selected']) for item in events),
            'earlier_selected_in_same_identity': len(earlier),
            'maximum_approach': max((float(item['approach']) for item in events), default=None),
            # The native wound snapshot currently records velocity only for
            # profiled blades/fists. Its zero on other records is unavailable.
            'maximum_tangent': max((float(item['tangent']) for item in events), default=None)
                if weapon in ('iron dagger', 'iron longsword') else None,
            'maximum_incident_energy': max((float(item['energy']) for item in events), default=None),
            'identities': sorted(identities),
        })
    result = {
        'task': 'PLAY-04', 'scope': 'Read-only contact-selection diagnostic, not acceptance',
        'trace_rows': len(contacts), 'trace_at_capacity': len(contacts) >= 6000,
        'raw_logs': raw, 'attempts': attempts,
        'limitations': 'Incident energy precedes solver-impulse reduction. A one-unit approach is only the legacy aggregator threshold. Tangent is unavailable for weapons without a wound profile. Prior selection may belong to excluded target placement. No selection does not by itself identify the rejection cause.',
    }
    output = args.runtime / 'held-contact-selection.json'
    output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(f'{len(attempts)} sweeps, {len(contacts)} contact rows: {output}')
    for item in attempts:
        print(item)


if __name__ == '__main__':
    main()
