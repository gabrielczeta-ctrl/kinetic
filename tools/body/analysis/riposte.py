"""RIPOSTE-01 / duel report (AI-02 item 5, 10-08): knockaways each fighter won
(RAGDOLL_CLASH outcome=knockaway, the `parry=` side), its riposte decisions
(RAGDOLL_AI2 what=riposte) and its blows (RAGDOLL_DAMAGE attacker=it) within
0.8 s of the knockaway, from the log's timestamps.

usage: python tools/body/analysis/riposte.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import sys


def seconds(line):
    m = re.match(r'\[(\d+):(\d+):([\d.]+)', line)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


for root in sys.argv[1:]:
    print(root)
    won = collections.Counter()
    answered = collections.Counter()
    decisions = collections.Counter()
    clashes = collections.Counter()
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        pending = []  # (time, parry side)
        for line in open(log, encoding='utf-8', errors='replace'):
            t = seconds(line)
            m = re.search(r'RAGDOLL_CLASH attacker=\S+ parry=(\S+) outcome=(\w+)', line)
            if m:
                clashes[m.group(2)] += 1
                if m.group(2) in ('knockaway', 'bounce'):
                    won[m.group(1)] += 1
                    pending.append((t, m.group(1)))
                continue
            m = re.search(r'RAGDOLL_AI2 actor=(\S+) what=riposte chosen=(\S+)', line)
            if m:
                decisions[(m.group(1), m.group(2))] += 1
                continue
            m = re.search(r'RAGDOLL_DAMAGE attacker=(\S+) ', line)
            if m and t is not None:
                for i, (when, side) in enumerate(pending):
                    if side == m.group(1) and when is not None and 0 <= t - when <= 0.8:
                        answered[side] += 1
                        pending.pop(i)
                        break
            pending = [(w, s) for (w, s) in pending if w is None or t is None or t - w <= 0.8]
    print('  clash outcomes', dict(clashes))
    for side, n in won.items():
        print('  %-20s parries won (knockaway, bounce) %d  answered with a blow within 0.8 s %d' % (side, n, answered[side]))
    print('  riposte decisions', dict(decisions))
