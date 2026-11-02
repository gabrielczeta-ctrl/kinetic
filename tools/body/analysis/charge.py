"""AI02-CHARGE report (AI-02 item 2, 10-08): per charge at an NPC, its read
(RAGDOLL_AI2 what=threat: T, response), its duel phases (RAGDOLL_DUEL), the
player's blows that landed (RAGDOLL_DAMAGE attacker="player") and the NPC's
blows on the player, between AI02_CHARGE_BEGIN/END.

usage: python tools/body/analysis/charge.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys

for root in sys.argv[1:]:
    print(root)
    totals = collections.defaultdict(lambda: [0, 0, 0, 0.0, collections.Counter()])
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        current = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'AI02_CHARGE_BEGIN npc=(\w+) n=(\d+)', line)
            if m:
                current = {'npc': m.group(1), 'n': m.group(2), 'reads': [], 'phases': [], 'landed': 0, 'taken': 0}
                continue
            if current is None:
                continue
            if 'AI02_CHARGE_END' in line:
                t = totals[current['npc']]
                t[0] += 1
                t[1] += current['landed']
                t[2] += current['taken']
                for read in current['reads']:
                    t[4][read] += 1
                print('  %-8s %s  read %-22s landed %d taken %d  phases %s' % (
                    current['npc'], current['n'], ','.join(current['reads']) or '-', current['landed'],
                    current['taken'], ' '.join(current['phases'][:6])))
                current = None
                continue
            m = re.search(r'what=threat T=(\S+) .*response=(\S+)', line)
            if m:
                current['reads'].append('%s(%.2f)' % (m.group(2), float(m.group(1))))
                continue
            m = re.search(r'RAGDOLL_DUEL actor=\S+ phase=(\S+) reason=(.+?) distance', line)
            if m:
                current['phases'].append('%s/%s' % (m.group(1), m.group(2).strip()))
                continue
            if re.search(r'RAGDOLL_DAMAGE attacker="player"', line):
                current['landed'] += 1
            elif re.search(r'RAGDOLL_DAMAGE attacker=\S+ victim="player"', line):
                current['taken'] += 1
    for npc, t in totals.items():
        print('  %-8s charges %d  player blows landed %d (%.2f per charge)  taken %d  reads %s' % (
            npc, t[0], t[1], t[1] / max(1, t[0]), t[2], dict(t[4])))
