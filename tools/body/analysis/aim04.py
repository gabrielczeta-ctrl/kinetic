"""AIM-04 report (look at head/chest/legs, 2026-10-06): for each looked-at part
and mode, the regions the player's blows struck (RAGDOLL_IMPACT between the
AIM04_BEGIN/END lines).

usage: python tools/body/analysis/aim04.py RUNTIME_DIR ...
"""
import glob, re, sys, collections
BEGIN = re.compile(r'AIM04_BEGIN mode=(\w+) part=(\w+)')
HIT = re.compile(r'RAGDOLL_IMPACT attacker="player" victim=\S+ region=(\w+)')
GROUP = {'head': 'head', 'neck': 'head', 'chest': 'torso', 'abdomen': 'torso', 'pelvis': 'torso'}
def group(region):
    if region in GROUP: return GROUP[region]
    if 'thigh' in region or 'calf' in region or 'foot' in region: return 'leg'
    if 'arm' in region or 'hand' in region or 'shoulder' in region: return 'arm'
    return region
for root in sys.argv[1:]:
    table = collections.defaultdict(collections.Counter)
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = BEGIN.search(line)
            if m:
                current = m.groups(); table[current]['attacks'] += 1; first = True; continue
            if 'AIM04_END' in line:
                current = None; continue
            h = HIT.search(line)
            if h and current and first:
                table[current][group(h.group(1))] += 1; first = False
    print(root)
    for key in sorted(table):
        c = table[key]; n = c.pop('attacks')
        print('  %-6s look=%-10s %d attacks: %s' % (key[0], key[1], n, dict(c)))
