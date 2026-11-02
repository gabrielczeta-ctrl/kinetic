"""NPC accuracy (AI-02 item 1, 10-08): per NPC cut on the wheel
(RAGDOLL_AI_CUT_BEGIN/END, OPENMW_RAGDOLL_AI_AIM_TRACE=1) what it met between
start and recovery - the body (RAGDOLL_IMPACT by it, not blocked), a guard
(a blocked impact, RAGDOLL_PARRY or RAGDOLL_GUARD_BLOW from it) or nothing -
and the closest the blade came to the foe's chest. Per actor, per line and per
gap band at the cut's start.

usage: python tools/body/analysis/npcaim.py RUNTIME_DIR ...
"""
import collections
import glob
import re
import statistics
import sys


def pct(values, q):
    if not values:
        return float('nan')
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))]


def band(gap):
    if gap < 0:
        return 'nofoe'
    for edge in (60, 80, 100, 120, 150):
        if gap < edge:
            return '<%d' % edge
    return '150+'


for root in sys.argv[1:]:
    print(root)
    cuts = []
    for log in glob.glob(root + '/shard-*/*/openmw.log') + glob.glob(root + '/*/openmw.log'):
        open_cuts = {}
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'RAGDOLL_AI_CUT_BEGIN actor=(\S+) n=(\d+) line=(\S+) foe=(\d) gap=(\S+) az=(\S+) dz=(\S+)', line)
            if m:
                actor = m.group(1)
                open_cuts[actor] = {'actor': actor, 'line': round(float(m.group(3))), 'gap': float(m.group(5)),
                                    'az': float(m.group(6)), 'body': [], 'guard': 0, 'closest': None}
                continue
            m = re.search(r'RAGDOLL_AI_CUT_END actor=(\S+) n=\d+ closest=(\S+)', line)
            if m:
                cut = open_cuts.pop(m.group(1), None)
                if cut:
                    cut['closest'] = float(m.group(2))
                    cuts.append(cut)
                continue
            m = re.search(r'RAGDOLL_IMPACT attacker=(\S+) victim=\S+ region=(\S+).* blocked=(\d)', line)
            if m and m.group(1) in open_cuts:
                if m.group(3) == '1':
                    open_cuts[m.group(1)]['guard'] += 1
                else:
                    open_cuts[m.group(1)]['body'].append(m.group(2))
                continue
            m = re.search(r'RAGDOLL_PARRY kind=\S+ attacker=(\S+)', line) or re.search(r'RAGDOLL_GUARD_BLOW actor=\S+ from=(\S+)', line)
            if m and m.group(1) in open_cuts:
                open_cuts[m.group(1)]['guard'] += 1
    by_actor = collections.defaultdict(list)
    for cut in cuts:
        by_actor[cut['actor']].append(cut)

    def summary(label, group):
        bit = sum(1 for c in group if c['body'])
        guard = sum(1 for c in group if not c['body'] and c['guard'])
        close = [c['closest'] for c in group if c['closest'] is not None and c['closest'] >= 0]
        print('  %-24s cuts %3d  body %3d (%3.0f%%)  guard %3d  miss %3d  closest p50 %5.1f p90 %5.1f' % (
            label, len(group), bit, 100.0 * bit / max(1, len(group)), guard, len(group) - bit - guard,
            pct(close, 0.5), pct(close, 0.9)))

    for actor, group in sorted(by_actor.items()):
        summary(actor, group)
    if cuts:
        summary('ALL', cuts)
        for line in sorted({c['line'] for c in cuts}):
            summary('  line %d' % line, [c for c in cuts if c['line'] == line])
        for b in ('<60', '<80', '<100', '<120', '<150', '150+', 'nofoe'):
            group = [c for c in cuts if band(c['gap']) == b]
            if group:
                summary('  gap %s' % b, group)
        regions = collections.Counter(r for c in cuts for r in c['body'][:1])
        print('  first region bitten:', dict(regions.most_common()))
        gaps = [c['gap'] for c in cuts if c['gap'] >= 0]
        if gaps:
            print('  gap at cut start p10/p50/p90: %.0f / %.0f / %.0f' % (pct(gaps, 0.1), statistics.median(gaps), pct(gaps, 0.9)))
