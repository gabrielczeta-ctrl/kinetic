"""Parry roles report (alpha 0.4 item 5): every RAGDOLL_PARRY_ROLES clash in
the logs. Per run: clashes (and clashes handled twice in one step), how often
the slower blade got the smaller kick, mean kick of the faster and slower
blade, and the slower blade's kick while its owner guarded.

usage: python tools/body/analysis/parry.py RUNTIME_DIR ...
"""
import glob, re, sys
ROW = re.compile(r'\[(\S+) I\] RAGDOLL_PARRY_ROLES reporter=\S+ speed=(\S+) other_speed=(\S+) reporter_attacks=\S+ '
                 r'kick=(\S+) other_kick=(\S+) reporter_guard=(\S+) other_guard=(\S+) blade=1')
for root in sys.argv[1:]:
    rows = []
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        for line in open(log, encoding='utf-8', errors='replace'):
            m = ROW.search(line)
            if m:
                rows.append((m.group(1), *[float(x) for x in m.groups()[1:5]], m.group(6) == '1', m.group(7) == '1'))
    if not rows:
        print(root, 'no clashes'); continue
    twice = sum(1 for a, b in zip(rows, rows[1:]) if a[0][:8] == b[0][:8] and abs(a[1] - b[2]) < 1e-3 and abs(a[2] - b[1]) < 1e-3)
    fastK, slowK, slowGuard, right = [], [], [], 0
    for t, s1, s2, k1, k2, g1, g2 in rows:
        if s1 >= s2:
            fk, sk, sg = k1, k2, g2
        else:
            fk, sk, sg = k2, k1, g1
        fastK.append(fk); slowK.append(sk)
        if sg: slowGuard.append(sk)
        right += sk < fk
    mean = lambda v: sum(v) / len(v) if v else float('nan')
    print('%s clashes=%d doubled=%d slower_kicked_less=%.2f fast_kick=%.0f slow_kick=%.0f slow_kick_guarding=%.0f (n=%d)' % (
        root, len(rows), twice, right / len(rows), mean(fastK), mean(slowK), mean(slowGuard), len(slowGuard)))
