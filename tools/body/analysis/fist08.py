"""FIST-08 report (K2_PALM_PARRY): per phase of the case (none, guard, palm)
the blows on the bare-fisted player - the defence outcomes (RAGDOLL_DEFENSE
rows whose victim is the player), the damage that got through
(RAGDOLL_DAMAGE), the palm parries (RAGDOLL_PALM_PARRY) and the case's own
health and fatigue lines.

usage: python tools/body/analysis/fist08.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

for root in sys.argv[1:]:
    print(root)
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        lines = open(log, encoding='utf-8', errors='replace').read().split('\n')
        phase = None
        rows = collections.OrderedDict()
        for i, line in enumerate(lines):
            m = re.search(r'FIST08_BEGIN phase=(\w+)', line)
            if m:
                phase = m.group(1)
                rows[phase] = {'defense': collections.Counter(), 'blows': 0, 'health': 0.0, 'fatigue': 0.0,
                               'regions': collections.Counter(), 'palm': collections.Counter(), 'cover': [],
                               'end': ''}
                continue
            m = re.search(r'FIST08_END phase=\w+ (.*)', line)
            if m and phase:
                rows[phase]['end'] = m.group(1).strip()
                phase = None
                continue
            if not phase:
                continue
            r = rows[phase]
            m = re.search(r'RAGDOLL_PALM_PARRY .*outcome=(\w+)', line)
            if m:
                r['palm'][m.group(1)] += 1
                continue
            m = re.search(r'RAGDOLL_DEFENSE outcome=(\w+) guard=(\d) cover=(\S+)', line)
            if m:
                victim = None
                for k in lines[i + 1:i + 10]:
                    if 'RAGDOLL_DEFENSE' in k:
                        break
                    mm = re.search(r'RAGDOLL_(?:HIT_ROLL|DAMAGE|IMPACT|CONTACT_OUTCOME) attacker=\S+ victim=(\S+)', k)
                    if mm:
                        victim = mm.group(1)
                        break
                if victim == '"player"':
                    r['defense'][m.group(1)] += 1
                    if m.group(2) == '1':
                        r['cover'].append(float(m.group(3)))
                continue
            m = re.search(r'RAGDOLL_DAMAGE attacker=\S+ victim="player" weapon=(.*?) (?:type|band)=.* final=([\d.]+)', line)
            if m:
                r['blows'] += 1
                r['fatigue' if m.group(1) == 'fists' else 'health'] += float(m.group(2))
            m = re.search(r'RAGDOLL_CONTACT_OUTCOME attacker=\S+ victim="player" region=(\w+)', line)
            if m:
                r['regions'][m.group(1)] += 1
        for phase, r in rows.items():
            cover = sorted(r['cover'])
            print('  %-5s defense %-34s paid blows %2d (health %.1f fatigue %.1f) regions %s palm %s cover %s | %s' % (
                phase, dict(r['defense']), r['blows'], r['health'], r['fatigue'], dict(r['regions']), dict(r['palm']),
                ('%.0f-%.0f' % (cover[0], cover[-1])) if cover else '-', r['end']))
