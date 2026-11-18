"""Weapon variants against their kind (VARIANT-01/02 in test_combat_fixes,
user 11-07: "make sure variations of weapons do not change the behavior").

usage: python tools/body/variants.py RUN_DIR [RUN_DIR ...]

Reads every openmw.log under the runs (the runs pool: means per variant). Per
variant: the cut strokes' (chop, forehand, backhand) top blade turn (deg/step)
and tip speed (that x the weapon's length: a longer haft turns fewer degrees
at the same tip speed - the steel spear, 138 long against the iron's 106),
the whole run's hand error p50/p90, wrist past its limit p90, jolts and
support, and the weapon proxy (tip side, length, grip share, edge axis).

A variant's model alone (VARIANT-01: every variant at its kind's iron weight)
is flagged when, against the median of its kind's variants (the iron one is
one of them, not the measure: the wooden staff's backhand sat above all six
other staffs):
  - a cut's turn and its tip speed both below 80% of the median's (one of
    them alone is the length: the shortest battle axe's tip, the longest
    spear's turn);
  - hand p90 over 1.5x the median's + 3 units;
  - wrist p90 over the median's + 10 degrees;
  - support below 0.95 for a two-handed weapon, or jolts over 2x the median's + 5.
"""
import glob, os, re, sys
from collections import defaultdict

CUTS = ('chop-cut', 'forehand-cut', 'backhand-cut')


def kinds():
    here = os.path.dirname(os.path.abspath(__file__))
    lua = os.path.join(here, '..', '..', 'scripts', 'data', 'integration_tests', 'test_combat_fixes', 'global.lua')
    text = open(lua, encoding='utf-8').read()
    block = text[text.index('local variantKinds = {'):]
    block = block[:block.index('\n}\n')]
    result = []
    for m in re.finditer(r"\{ '([^']+)', [\d.]+, \{([^}]*)\} \}", block, re.S):
        result.append([m.group(1)] + re.findall(r"'([^']+)'", m.group(2)))
    return result


LINE = re.compile(r"Global\[global\.lua\]:\t(VARIANT-0[12]) (.+?) (chop-cut|forehand-cut|backhand-cut|total) +peaks.*?"
                  r"jolts=(\d+).*?speed_max=([\d.]+).*?hand_p50=([-\d.]+) hand_p90=([-\d.]+).*?wrist_out_p90=(\d+) "
                  r"support=([-\d.]+)")
START = re.compile(r"TEST_START\t\d+\t(VARIANT-0[12]) (.+)$")
PROXY = re.compile(r"RAGDOLL_WEAPON_PROXY record=(.+?) source=\S+ length=([\d.]+) long_axis=(\d) tip_sign=(-?\d+) "
                   r"center=([-\d.e]+),([-\d.e]+),([-\d.e]+) half_extents=([-\d.e]+),([-\d.e]+),([-\d.e]+)")


def read(dirs):
    data = defaultdict(lambda: defaultdict(list))  # (family, weapon) -> phase -> [tuple]
    proxy = {}
    for d in dirs:
        for log in glob.glob(os.path.join(d, '**', 'openmw.log'), recursive=True):
            current = None
            for line in open(log, encoding='utf-8', errors='replace'):
                st = START.search(line)
                if st:
                    current = st.group(2).strip()
                    continue
                m = LINE.search(line)
                if m:
                    fam, weapon, phase = m.group(1), m.group(2), m.group(3)
                    data[(fam, weapon)][phase].append(tuple(float(x) for x in m.groups()[3:]))
                    continue
                p = PROXY.search(line)
                if p and current and current not in proxy:
                    axis = int(p.group(3))
                    c = [float(p.group(i)) for i in (5, 6, 7)]
                    h = [float(p.group(i)) for i in (8, 9, 10)]
                    others = sorted((a for a in range(3) if a != axis), key=lambda a: -h[a])
                    proxy[current] = dict(length=float(p.group(2)), axis=axis, tip=int(p.group(4)),
                                          share=c[axis] / h[axis] if h[axis] else 0, edge='xyz'[others[0]],
                                          width=h[others[0]] * 2, thick=h[others[1]] * 2)
    return data, proxy


def mean(v):
    return sum(v) / len(v) if v else float('nan')


def median(v):
    v = sorted(x for x in v if x == x)
    return v[len(v) // 2] if v else float('nan')


def summary(phases, length):
    out = {}
    for ph in CUTS:
        out[ph] = mean([r[1] for r in phases.get(ph, [])])
        out['tip ' + ph] = out[ph] * length if length else float('nan')
    tot = phases.get('total', [])
    out['jolts'] = mean([r[0] for r in tot])
    out['v'] = mean([r[1] for r in tot])
    out['h50'] = mean([r[2] for r in tot])
    out['h90'] = mean([r[3] for r in tot])
    out['wrist'] = mean([r[4] for r in tot])
    out['support'] = mean([r[5] for r in tot])
    out['n'] = len(tot)
    return out


def main():
    dirs = sys.argv[1:] or ['.']
    data, proxy = read(dirs)
    flagged = []
    for kind in kinds():
        print('\n== %s' % kind[0])
        print('   %-22s %-3s %5s %5s %5s | tip%% vs median %4s %4s %4s | %4s %4s %3s %3s %4s | proxy' % (
            'weapon', 'fam', 'chop', 'fore', 'back', 'chop', 'fore', 'back', 'h50', 'h90', 'wr', 'jl', 'sup'))
        for fam in ('VARIANT-01', 'VARIANT-02'):
            rows = {}
            for w in kind:
                px = proxy.get('%s' % w) if fam == 'VARIANT-02' else None
                length = (proxy.get(w) or {}).get('length', 0.0)
                s = summary(data.get((fam, w), {}), length)
                if s['n']:
                    rows[w] = s
            if not rows:
                continue
            med = {k: median([r[k] for r in rows.values()]) for k in next(iter(rows.values()))}
            for w, s in rows.items():
                px = proxy.get(w, {})
                pxt = ('tip%+d len %.0f grip %.2f edge %s %.1fx%.1f' % (px['tip'], px['length'], px['share'],
                       px['edge'], px['width'], px['thick'])) if px else ''
                pct = {ph: 100.0 * s['tip ' + ph] / med['tip ' + ph] if med['tip ' + ph] else float('nan') for ph in CUTS}
                bad = []
                if fam == 'VARIANT-01':
                    for ph in CUTS:
                        turn = 100.0 * s[ph] / med[ph] if med[ph] else float('nan')
                        # Slow in both the turn and the tip: not the length.
                        if pct[ph] < 80.0 and turn < 80.0:
                            bad.append('%s turn %.0f%% tip %.0f%%' % (ph.split('-')[0], turn, pct[ph]))
                    if s['h90'] > 1.5 * med['h90'] + 3:
                        bad.append('hand p90 %.1f' % s['h90'])
                    if s['wrist'] > med['wrist'] + 10:
                        bad.append('wrist %.0f' % s['wrist'])
                    if 0 <= s['support'] < 0.95 and med['support'] >= 0.95:
                        bad.append('support %.2f' % s['support'])
                    if s['jolts'] > 2 * med['jolts'] + 5:
                        bad.append('jolts %.0f' % s['jolts'])
                if px and px['tip'] < 0:
                    bad.append('tip reversed')
                if bad:
                    flagged.append((kind[0], w, bad))
                print('   %-22s %-3s %5.1f %5.1f %5.1f | %16s %4.0f %4.0f %4.0f | %4.1f %4.1f %3.0f %3.0f %4.2f | %s%s' % (
                    w[:22], fam[-2:], s['chop-cut'], s['forehand-cut'], s['backhand-cut'], '',
                    pct['chop-cut'], pct['forehand-cut'], pct['backhand-cut'], s['h50'], s['h90'], s['wrist'],
                    s['jolts'], s['support'], pxt, ('  <-- ' + ', '.join(bad)) if bad else ''))
    print('\nFLAGGED %d' % len(flagged))
    for ref, w, bad in flagged:
        print('  %s (%s): %s' % (w, ref, ', '.join(bad)))


if __name__ == '__main__':
    main()
