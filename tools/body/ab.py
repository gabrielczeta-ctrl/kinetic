"""A/B a few cases on two engines, several runs each, every bar side by side.

usage: python tools/body/ab.py NAME SUITE 'lua pattern' [...] [K=V ...]
                               [--runs N] [--a SIDE] [--b SIDE] [--a-env K=V,K=V] [--b-env K=V,...] [--keep]
e.g.   python tools/body/ab.py swing test_combat_fixes '^GUARD%-03' --runs 3
       python tools/body/ab.py k2 test_combat_fixes '^PACE' --a this --b this --b-env OPENMW_RAGDOLL_K2_LOCKOUT=1

A side is one of:
  this       build-footfall's openmw.exe with the game's values (cases.py --play)
  kinetic-1  openmw-kinetic-1.exe with KINETIC-1's own settings, as played
             (dist/KINETIC-1/ragdoll: ragdoll.cfg, active-body.cfg and the
             gait table beside them; the engine predates the play table)
  play       the playthrough's engine (dist/KINETIC-playthrough/openmw.exe,
             copied beside the test exe for the run) with the game's values
  PATH       any other exe, with the game's values
Defaults: --a kinetic-1 --b this --runs 3. K=V arguments go to both sides
(after the side's own values, so they win); --a-env / --b-env to one side
only (after those). With the same side twice the columns are named A and B. Physics is not repeatable: one
run is never a verdict (PLAN.md "Testing").

Every run goes to the results ledger as usual (runtime-ONE-NAME-<side>-<n>);
the run directories are deleted afterwards unless --keep (each is
250 MB - 1.5 GB). Runs while a game window is open are refused by the
runner; queue with test_queue.py.
"""
import re
import os
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ledger  # noqa: E402
import sentinel  # noqa: E402

DIST = sentinel.ROOT / 'dist'
TEST_EXE = sentinel.BUILD / 'RelWithDebInfo/openmw.exe'


def side_setup(side):
    """(engine path, envs, cleanup) for a side."""
    if side == 'this':
        return str(TEST_EXE), sentinel.play_envs(), None
    if side == 'kinetic-1':
        cfg = DIST / 'KINETIC-1/ragdoll'
        envs = ['OPENMW_RAGDOLL_SETTINGS=%s' % (cfg / 'ragdoll.cfg'),
                'OPENMW_RAGDOLL_ACTIVE_BODY=%s' % (cfg / 'active-body.cfg'),
                # The scripts power the player, as in every scripted case.
                'OPENMW_RAGDOLL_POWER_PLAYER=0']
        return str(sentinel.BUILD / 'RelWithDebInfo/openmw-kinetic-1.exe'), envs, None
    if side == 'play':
        copy = sentinel.BUILD / 'RelWithDebInfo/openmw-ab-play-tmp.exe'
        shutil.copy2(DIST / 'KINETIC-playthrough/openmw.exe', copy)
        return str(copy), sentinel.play_envs(), lambda: copy.unlink(missing_ok=True)
    path = Path(side)
    if not path.is_file():
        sys.exit('no such engine: %s' % side)
    return str(path), sentinel.play_envs(), None


def run_once(out, suite, patterns, engine, envs):
    """One runner pass, retried while the last run's engine lease lingers."""
    os.environ['RAGDOLL_TEST_ENGINE'] = engine
    for attempt in range(6):
        sentinel.run_stage(out, suite, patterns, envs)
        log = Path(str(out) + '.runner.log')
        text = log.read_text(errors='replace') if log.exists() else ''
        if 'engine lease' not in text:
            break
        print('  engine lease held, retrying in 15 s')
        time.sleep(15)
    checks = ledger.default_checks(out, suite, envs)
    ledger.record(out, checks=checks)
    statuses = sentinel.statuses(out)
    return statuses, checks


def main(argv):
    if len(argv) < 3:
        sys.exit(__doc__)
    name, suite = argv[0], argv[1]
    opts = {'--runs': '3', '--a': 'kinetic-1', '--b': 'this', '--a-env': '', '--b-env': ''}
    rest = []
    i = 2
    while i < len(argv):
        if argv[i] in opts and i + 1 < len(argv):
            opts[argv[i]] = argv[i + 1]
            i += 2
            continue
        rest.append(argv[i])
        i += 1
    keep = '--keep' in rest
    patterns = [a for a in rest if not a.startswith('--') and '=' not in a]
    extra = [a for a in rest if '=' in a and not a.startswith('--')]
    runs = int(opts['--runs'])
    sides = [opts['--a'], opts['--b']]
    # Split before each OPENMW_ key only, so a value may hold commas
    # (--b-env OPENMW_RAGDOLL_K2_GAIT_CAP=350,1).
    side_envs = [[e for e in re.split(r',(?=OPENMW_)', opts[k]) if e] for k in ('--a-env', '--b-env')]
    labels = sides if sides[0] != sides[1] else ['A ' + sides[0], 'B ' + sides[1]]

    # results[case][check][side] = [value per run]; statuses likewise.
    results = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    outs = []
    saved_engine = os.environ.get('RAGDOLL_TEST_ENGINE')
    try:
        for index, side in enumerate(sides):
            engine, envs, cleanup = side_setup(side)
            envs = envs + extra + side_envs[index]
            column = labels[index]
            label = ('k1' if side == 'kinetic-1' else ('path' if os.sep in side or '/' in side else side)) + 'ab'[index]
            try:
                for n in range(1, runs + 1):
                    out = sentinel.ROOT / ('runtime-ONE-%s-%s-%d' % (name, label, n))
                    outs.append(out)
                    print('%s run %d/%d: %s' % (side, n, runs, Path(engine).name))
                    statuses, checks = run_once(out, suite, patterns, engine, envs)
                    for case, status in statuses.items():
                        results[case]['asserts'][column].append(status)
                    for case, items in checks.items():
                        for check in items:
                            results[case][check[0]][column].append('%s%s' % (check[1], '' if check[2] else '!'))
            finally:
                if cleanup:
                    cleanup()
    finally:
        if saved_engine is None:
            os.environ.pop('RAGDOLL_TEST_ENGINE', None)
        else:
            os.environ['RAGDOLL_TEST_ENGINE'] = saved_engine
        if not keep:
            for out in outs:
                shutil.rmtree(out, ignore_errors=True)
                Path(str(out) + '.runner.log').unlink(missing_ok=True)

    print()
    print('%-34s %-30s %-30s' % ('check ("!" = fails its bar)', labels[0], labels[1]))
    for case in sorted(results):
        print(case)
        for check, by_side in results[case].items():
            cells = [' '.join(str(v) for v in by_side.get(column, [])) or '-' for column in labels]
            print('   %-31s %-30s %-30s' % (check, cells[0], cells[1]))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
