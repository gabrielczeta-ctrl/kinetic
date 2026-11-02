"""The goal's arm bars (2026-10-05) from test runs: sums the ARMQ lines that
REPLAY-* and MOVE-01 print (test_combat_fixes armTally) per group of tests.

  guard: steps with the hand turned > 45 deg from its command   <= 2 %
  rest:  the hand's distance from its grip goal in the guard, p90 <= 8 units:
         at most 10% of the group's guard steps over 8 (pooled; the worst
         single test's p90 is printed too)
  attack (wind-up and cut): steps turned > 90 deg for over 0.25 s <= 1 %
         (a flip or stuck wrist; every step > 90 is printed too: a heavy
         blade trailing a fast command catches up within 3-6 steps)
  re-seats per minute of hand control                            <= 1

usage: python tools/body/arm_quality.py RUN [RUN ...]   (runtime-ONE-RUN in the dev root)
"""
import gzip
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LINE = re.compile(r'ARMQ (.+?) guard=(\d+) g45=(\d+) restP90=([-\d.]+) attack=(\d+) a90=(\d+) reseats=(\d+) steps=(\d+)'
                  r'(?:.*held90=(\d+))?(?:.*rest8=(\d+))?')


def rows(run):
    for path in (ROOT / ('runtime-ONE-' + run)).glob('shard-*/*.log.gz'):
        for line in gzip.open(path, 'rt', errors='replace'):
            m = LINE.search(line)
            if m:
                yield m.group(1), [float(x) if x is not None else -1.0 for x in m.groups()[1:]]


def main():
    groups = defaultdict(lambda: [0.0] * 9)
    worst_rest = defaultdict(float)
    for run in sys.argv[1:]:
        for label, v in rows(run):
            group = ' '.join(label.split()[:3]) if label.startswith('MOVE') else label.split()[0]
            g = groups[group]
            for i in (0, 1, 3, 4, 5, 6, 7, 8):
                g[i] += v[i]
            worst_rest[group] = max(worst_rest[group], v[2])
            print('  %-60s guard>45 %5.1f%%  rest p90 %5.1f  attack>90 %5.1f%%  re-seats %d' % (
                label[:60], 100 * v[1] / max(1, v[0]), v[2], 100 * v[4] / max(1, v[3]), v[5]))
    ok = True
    for group, g in sorted(groups.items()):
        guard, g45, attack, a90, reseats, steps, held, rest8 = g[0], g[1], g[3], g[4], g[5], g[6], g[7], g[8]
        minutes = steps / 3600.0
        bars = [('guard>45 %.1f%% <= 2' % (100 * g45 / max(1, guard)), g45 <= 0.02 * guard),
                ('rest >8 in %.1f%% of guard steps <= 10 (worst test p90 %.1f)' % (100 * rest8 / max(1, guard), worst_rest[group]),
                 rest8 >= 0 and rest8 <= 0.10 * guard),
                ('attack>90 held %.2f%% <= 1 (all >90: %.2f%%)' % (100 * held / max(1, attack), 100 * a90 / max(1, attack)),
                 held <= 0.01 * attack),
                ('re-seats %.1f/min <= 1' % (reseats / max(minutes, 1e-6)), reseats <= max(minutes, 1e-6))]
        print('%-22s %s' % (group, '  '.join(('ok ' if b else 'FAIL ') + t for t, b in bars)))
        ok &= all(b for _, b in bars)
    print('ARM BARS ' + ('PASS' if ok else 'FAIL'))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
