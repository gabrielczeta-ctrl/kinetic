"""Per-drag peak tip speed in replay runs (spec 8.2: can a committed drag build speed).

    replay_speed.py <label>=<run dir> [...]

For every replay case, the peak physical tip speed of each held drag (more
than 10 ticks); prints the median and the best drag per case and label.
"""
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates as G


def drags(case):
    ks = sorted(case.ticks)
    run, out = [], []
    for a, b in zip(ks, ks[1:]):
        ta, tb = case.ticks[a], case.ticks[b]
        if b == a + 1 and ta['held'] and tb['held']:
            pa, pb = case.tips(ta), case.tips(tb)
            if pa and pb:
                run.append(max(G.norm(G.sub(x, y)) for x, y in zip(pa, pb)) * 60)
        elif run:
            if len(run) > 10:
                out.append(max(run))
            run = []
    return out


rows = {}
for arg in sys.argv[1:]:
    label, path = arg.split('=', 1)
    for name, marks, ticks, contacts in G.load_run(path):
        if not ticks:
            continue
        peaks = drags(G.Case(name, marks, ticks, contacts))
        if peaks:
            rows.setdefault(name, {})[label] = (statistics.median(peaks), max(peaks), len(peaks))
labels = [a.split('=', 1)[0] for a in sys.argv[1:]]
print('%-26s ' % 'case' + ' '.join('%-22s' % l for l in labels))
for name in sorted(rows):
    print('%-26s ' % name[:26] + ' '.join('%-22s' % ('med %4d max %4d n%-3d' % rows[name][l] if l in rows[name] else '-')
                                          for l in labels))
