"""Per-change check: run a few cases in play settings and print their pass/fail
and the sentinel's bars for their suite (walking and movement, combat, duel).

usage: python tools/body/cases.py NAME SUITE 'lua pattern' ['lua pattern' ...] [K=V ...] [--play] [--report]
e.g.   python tools/body/cases.py stag test_play01_walking '^PLAY%-01 unarmed vanilla [kf]'

Output: runtime-ONE-NAME in the dev root; every case also goes to the results
ledger (ledger.py summary/history show its earlier runs). Suites are for play
builds and releases (sentinel.py, see PLAN.md).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ledger  # noqa: E402
import sentinel  # noqa: E402

args = sys.argv[1:]
if len(args) < 3:
    sys.exit(__doc__)
name, suite = args[0], args[1]
patterns = [a for a in args[2:] if not a.startswith('--') and '=' not in a]
envs = [a for a in args[2:] if '=' in a and not a.startswith('--')]
if '--play' in args:
    # The play launcher's switches first; K=V arguments still win.
    envs = sentinel.play_envs() + envs
out = sentinel.ROOT / ('runtime-ONE-%s' % name)
if '--report' not in args:
    sentinel.run_stage(out, suite, patterns, envs)
for case, status in sorted(sentinel.statuses(out).items()):
    print('%-60s %s' % (case, status))
checks = ledger.default_checks(out, suite, envs)
for case, items in sorted(checks.items()):
    print(case)
    for check in items:
        print('   %-28s %-12s %s' % (check[0], check[1], 'ok' if check[2] else 'FAIL'))
ledger.record(out, checks=checks)
