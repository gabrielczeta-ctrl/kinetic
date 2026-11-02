"""Tip error (physical vs commanded) and blade/wrist numbers by phase group.

    phase_errors.py <run dir> [case substring]

Groups: probe, range, flick, circle, rhythm, click, guard/rest, release.
Held ticks only for the error columns, as in G2.
"""
import collections
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates as G


def group(phase):
    for key in ('probe', 'range', 'flick', 'circle', 'rhythm', 'click', 'guard', 'rest', 'release', 'draw', 'sheathe'):
        if phase.startswith(key) or key in phase:
            return key
    return phase


def main():
    run = sys.argv[1]
    only = sys.argv[2] if len(sys.argv) > 2 else ''
    for name, marks, ticks, contacts in G.load_run(run):
        if only not in name:
            continue
        c = G.Case(name, marks, ticks, contacts)
        rows = collections.defaultdict(lambda: dict(err=[], all=[], wrist=[]))
        for k in sorted(ticks):
            phase = None
            for step, p in marks:
                if step <= k:
                    phase = p
            if phase is None:
                continue
            t = ticks[k]
            p, q = c.tips(t), c.tips(t, 'command')
            g = group(phase)
            if p and q and len(p) == len(q):
                e = max(G.norm(G.sub(x, y)) for x, y in zip(p, q))
                rows[g]['all'].append(e)
                if t['held']:
                    rows[g]['err'].append(e)
            rows[g]['wrist'] += [x for x in t.get('wrist_excess', []) if x >= 0]
        print(name)
        for g, r in rows.items():
            print('   %-8s held n=%4d p50 %6.1f p90 %6.1f | all p90 %6.1f | wrist p99 %5.1f' % (
                g, len(r['err']), G.pct(r['err'], .5) if r['err'] else -1, G.pct(r['err'], .9) if r['err'] else -1,
                G.pct(r['all'], .9) if r['all'] else -1, G.pct(r['wrist'], .99) if r['wrist'] else -1))


if __name__ == '__main__':
    main()
