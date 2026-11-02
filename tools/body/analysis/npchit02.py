"""NPCHIT-02 report (AI-02 items 4 and 6, 10-08): per guard zone the player
holds, the NPC's line choices (RAGDOLL_AI2 what=line: chosen, open), its cuts
and what they met (RAGDOLL_AI_CUT_BEGIN/END with the contacts between them),
the player's defence outcomes (RAGDOLL_DEFENSE) and the damage taken.

usage: python tools/body/analysis/npchit02.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

for root in sys.argv[1:]:
    print(root)
    rows = collections.OrderedDict()
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        zone = None
        cutting = False
        cut = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'NPCHIT02_BEGIN zone=(\w+) read=(\S+)', line)
            if m:
                zone = m.group(1)
                rows.setdefault(zone, {'read': m.group(2), 'lines': collections.Counter(), 'open': [0, 0],
                                       'cuts': 0, 'body': 0, 'guard': 0, 'damage': 0.0, 'blows': 0,
                                       'defense': collections.Counter()})
                continue
            if 'NPCHIT02_END' in line:
                zone = None
                continue
            if not zone:
                continue
            r = rows[zone]
            m = re.search(r'what=line chosen=(\S+) .* open=(\S+)', line)
            if m:
                r['lines'][round(float(m.group(1)))] += 1
                r['open'][0] += float(m.group(2)) >= 0.99
                r['open'][1] += 1
                continue
            if 'RAGDOLL_AI_CUT_BEGIN' in line:
                r['cuts'] += 1
                cutting = True
                cut = [False, False]
                continue
            if 'RAGDOLL_AI_CUT_END' in line and cutting:
                cutting = False
                r['body'] += cut[0]
                r['guard'] += cut[1] and not cut[0]
                continue
            m = re.search(r'RAGDOLL_DAMAGE attacker=\S+ victim="player".* final=([\d.]+)', line)
            if m:
                r['damage'] += float(m.group(1))
                r['blows'] += 1
            m = re.search(r'RAGDOLL_DEFENSE outcome=(\w+)', line)
            if m:
                r['defense'][m.group(1)] += 1
            if cutting and 'RAGDOLL_IMPACT' in line and 'victim="player"' in line:
                if 'blocked=1' in line:
                    cut[1] = True
                else:
                    cut[0] = True
            if cutting and ('RAGDOLL_PARRY' in line or 'RAGDOLL_GUARD_BLOW actor="player"' in line):
                cut[1] = True
    for zone, r in rows.items():
        print('  guard %-5s (read %-6s) lines %-28s open %d/%d  cuts %d: body %d guard %d  blows %d damage %.1f  defense %s' % (
            zone, r['read'], dict(r['lines']), r['open'][0], r['open'][1], r['cuts'], r['body'], r['guard'],
            r['blows'], r['damage'], dict(r['defense'])))
