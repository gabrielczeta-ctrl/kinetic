"""Sentinel: the focused test suites and the quick keep/revert check.

usage: sentinel.py NAME [--play] [--env K=V ...] [--suite SUITE ...] [--stage STAGE ...] [--report-only]

Focused suites (10-12 test review; a suite is one or more stages):
  walking   walk     gait course, starts/stops, runs, pushes, knockdowns, get-up
  balance   balance  PGC-03 behaviours: STAGGER-01, LEDGE-01, STAIRJUMP-01,
                     TEETER-01, WALL-01, SLOPE-01, MANTLE-01, CRAWL-01 and the stairs set
                     (behaviour_report.py reads the same output for steps,
                     skid, slide, steady time)
  (stage)   drop     DROP-01 drop roll cases on the fixture towers (--stage drop;
                     behaviour_report.py's inv/rolled/headv/jolt/travel/roll)
  combat    combat   FIX-08 player blows (wheel, fists, drags), FIST-01
  duel      duel     NPC duels: DMG-04 damage band, DUEL-A02, SCALE-01/03
  movement  move     the user's 10-30 movement notes as bars (move_report.py): run
                     float, long flights, landing reaches on flat ground,
                     reversal time, step timing - course runs of every loadout
                     and the reversals
  release   all of the above plus life (test_ragdoll lifecycle, death,
            save/load, scale, rigs), visible (weapon attach and visibility)
            and release (WPN-02: a dropped or broken weapon is released
            exactly once)
Without --suite or --stage: walk, combat and duel (the sentinel, ~5 min).
--play: the play launcher's switches (Run-Game-MotionBricks.cmd) first.

Three launches in play settings with Footfall (the checks.sh setup): a
walking sample, a FIX-08 sample (cases that pass every run) and the duels.
Each case is checked against quality bars, not just its own asserts: course
scores, falls, pushes caught, get-up slide, idle shake, blows landed, damage
in the Morrowind band. Physics is not repeatable run to run (not even with
one physics thread), so a case that fails a bar is run once more: failing
twice is FAIL, failing once is FLAKY. Exit code 1 on any FAIL.

Per change: cases.py; per play build and release: these suites (PLAN.md,
Testing). Output: runtime-SENT-NAME-<stage>[-again], report in
runtime-SENT-NAME-sentinel.txt; every case's checks also go to the results
ledger (ledger.py), which outlives the output.
"""
import argparse
import gzip
import shutil
import json
import math
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import course_report  # noqa: E402
import ledger  # noqa: E402
import move_report  # noqa: E402
import push_report  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]  # the workspace (source-footfall/tools/body/..)
TREE = HERE.parent.parent
BUILD = ROOT / 'build-footfall'
FALL_MODES = push_report.FALL_MODES

STAGES = {
    'walk': ('test_play01_walking', [
        '^PLAY%-01 fists course$', '^PLAY%-01 claymore course$', '^PLAY%-01 fists course run$',
        '^PLAY%-01 unarmed push [sw][ta][al][nk][^ ]* [a-z]+ 2000$', '^PLAY%-01 unarmed push walk front 3000$',
        '^PLAY%-01 [uw][na][ar][^ ]* knockdown 6000$', '^PLAY%-01 unarmed no input$', '^PLAY%-01 unarmed start stop$',
        '^PLAY%-01 unarmed diagonal left$', '^PLAY%-01 warhammer run fast turn 3$', '^PLAY%-01 claymore run diagonal$', '^PLAY%-01 claymore run zigzag$',
        '^PLAY%-01 [uw][na][ar][^ ]* stand displaced side$', '^PLAY%-01 unarmed knockdown balmora hill$',
        '^PLAY%-01 [uw][na][ar][^ ]* powered lying$', '^PLAY%-01 unarmed vanilla [kf]',
    ]),
    'combat': ('test_combat_fixes', [
        '^FIX%-08 .* guided fists$', '^FIX%-08 .* quick fists$', '^FIX%-08 .* guided iron claymore first person$',
        '^FIX%-08 .* guided iron warhammer first person$', '^FIX%-08 .* held drag iron claymore .*',
        '^FIX%-08 .* held drag iron longsword .*', '^FIX%-08 .* fast drag iron longsword .*', '^FIST%-01',
    ]),
    'duel': ('test_duel', ['^DMG%-04', '^DUEL%-A02', '^SCALE%-01', '^SCALE%-03']),
    'move': ('test_play01_walking', ['^PLAY%-01 [fdclwu][a-z]+ course run$', '^PLAY%-01 [usw][a-z]+ reversal$']),
    'balance': ('test_play01_walking', [
        'STAGGER%-01', 'LEDGE%-01', 'STAIRJUMP%-01', 'TEETER%-01', 'WALL%-01', 'SLOPE%-01', 'MANTLE%-01', 'CRAWL%-01',
        '^PLAY%-01 unarmed stairs up and back$', '^PLAY%-01 unarmed STAIRS%-01 run up$',
        '^PLAY%-01 unarmed stairs drop off$', '^PLAY%-01 unarmed JUMP%-01 ledge jump',
    ]),
    # The drop roll (PGC-03 C8b): run, walk and strafe off the fixture
    # towers (not the Balmora bank case, also DROP-01). Asserts only: the
    # 500 run falls without the roll and the 800 run falls with it, by
    # design; behaviour_report.py judges them (rolled, inv, headv, roll).
    'drop': ('test_play01_walking', ['DROP%-01 run off %d', 'DROP%-01 walk off %d', 'DROP%-01 strafe off %d']),
    # The lifecycle cases (passive death, resurrection, cleanup, save/load,
    # rigs, scale). They expect living NPCs without a ragdoll: the stage runs
    # with OPENMW_RAGDOLL_POWER_NPCS=0 (STAGE_ENV; the shipped ragdoll.cfg
    # powers every NPC near the player).
    'life': ('test_ragdoll', [
        '^passive ragdoll lifecycle and impulse$', '^powered living ragdoll follows animation',
        '^powered living ragdoll transitions to passive death$', '^powered living ragdoll disables motors after a fall$',
        '^passive ragdoll remains bounded and settles naturally$', '^repeated death resurrection cycles',
        '^ragdoll cap evicts the oldest sleeping corpse$', '^dead actor cell transfer', '^unsupported creature falls back',
        '^player death and resurrection', '^uniform actor scale', '^female humanoid assembles', '^beast humanoid assembles',
        '^dead ragdoll actor remains activatable$', 'save load',
    ]),
    # Not 'hybrid native charged attack': the wheel takes the click, the
    # native charged attack it waits for no longer starts in play settings.
    'visible': ('test_weapon_visibility', ['^WPN%-05 visibility equipped', '^WPN%-05 visibility ordinary',
                                           '^WPN%-05 visibility replacement', '^WPN%-05 visibility same%-frame',
                                           '^WPN%-05 visibility sheath']),
    'release': ('test_physical_weapon', ['^WPN%-02']),
}
# balance: its cases ask whether steps catch a blow; the vanilla knockdown
# roll (KNOCKDOWN, 10-31) fells a stagger by dice (heavy test blows on the
# Agility-30 test player: 35%), which is the duels' business, not these bars.
STAGE_ENV = {'life': ['OPENMW_RAGDOLL_POWER_NPCS=0'], 'balance': ['OPENMW_RAGDOLL_KNOCKDOWN=0']}
SUITES = {
    'walking': ['walk'], 'balance': ['balance'], 'combat': ['combat'], 'duel': ['duel'], 'movement': ['move'],
    'release': ['walk', 'balance', 'move', 'combat', 'duel', 'life', 'visible', 'release'],
}

# Bars. Course scores are the per-segment medians of course_report (walk and
# run segments); the definition-of-done bars (5 %) are the target, these are
# "no worse than the 2026-09-29 build, with the run-to-run spread".
BARS = {
    'course': {  # (low, high) for the median. Calibrated on 5 runs of one build (cal1-5, 2026-09-29):
        # walk stepT 5-23, stepL 13-27, side 0.2-1.3, sink -1.1..-1.35, scuff 0.3-1.0;
        # run stepT 39-49, stepL 20-42, side 1.3-3.2, sink -2.3..-2.4, scuff 0.7-1.3.
        # Step timing/length are noise-dominated (5 segments): the bars only catch gross breaks.
        # walk stepT 28 -> 50 (10-30 re-baseline: build 88 42.7, build 89 25-46 run to run).
        'walk': {'stepT': (0, 50), 'stepL': (0, 32), 'side': (0, 2.5), 'sink': (-2.5, 9), 'scuff': (0, 1.5)},
        'run': {'stepT': (0, 60), 'stepL': (0, 50), 'side': (0, 4.5), 'sink': (-3.5, 9), 'scuff': (0, 2.0)},
    },
    # Near-deterministic over 5 runs, so tight: push recovery 0.45-0.68 s, get-up 3.5 s, slide 0-1.
    # 0.9 -> 1.6 (10-30 re-baseline): since STEP-01 a push is caught with
    # steps, not a slide; walking front pushes settle in 1.0-1.4 s (10-29/30).
    'push_recover': 2.2,     # s (build 88 and 89 alike: 0.7-2.0 on walk front 3000), pushes of 2000 and 3000 must not fall and settle in this
    'getup_slide': 10.0,     # units the pelvis moves from getting up to standing
    'getup_time': 4.5,       # s
    'end_lean': 12.0,        # deg torso tilt over the last second (stood still or stopping); seen 0-6
    'idle_p90': 10.0,        # deg/s pelvis angular speed, standing still; seen 0
    'blows_min': 2,          # physical blows per FIX-08 case
    'duel_damage': (4.0, 45.0),  # median final damage of a DMG-04 blow
    # Movement (10-30, build 89 with RUN_BOB 0.3,5: run float 3-19% by loadout,
    # the sheathed/unarmed run highest; build 88 22-35%). One flight of 0.25 s
    # in a 14 s run is 4.3/min. Reversals 0.08-0.13 s.
    'move': {'air': 0.24, 'long/min': 9.0, 'reaches': 0, 'turn_s': 0.5, 'step_cv': 0.6},
}


# The play switches change what a bar can expect (10-06, user: "change
# that, we should get a decent crouch animation"). main() reads them from
# --env: with an armed stance knee the course's armed walk sits lower than the
# upright armed clip by design (sink median at 0.8: -4.6..-4.8, at 1.0 -6.9),
# so the sink bars move down 4 units per unit of it. Knockdowns are physical
# (the knock-down rule left the code with build 72): a knockdown push may be
# caught by the legs, so a case with no get-up passes if it ends upright, and
# SCALE-03 (it asserts a knockdown) is left out.
PLAY = {'armed_knee': 0.0, 'physical_knockdown': True}
BASE_SINK = {kind: BARS['course'][kind]['sink'] for kind in ('walk', 'run')}


def apply_play_switches(envs):
    """Idempotent: the ledger applies each recorded run's own switches."""
    PLAY['armed_knee'] = 0.0
    for e in envs:
        key, _, value = e.partition('=')
        if key == 'OPENMW_RAGDOLL_FOOTFALL_ARMED_STANCE_KNEE':
            PLAY['armed_knee'] = max(0.0, float(value or 0))
    for kind in ('walk', 'run'):
        lo, hi = BASE_SINK[kind]
        BARS['course'][kind]['sink'] = (lo - 4.0 * PLAY['armed_knee'], hi)
    if PLAY['physical_knockdown']:
        suite, patterns = STAGES['duel']
        STAGES['duel'] = (suite, [p for p in patterns if 'SCALE%-03' not in p])


COURSE = 'course medians'
COURSE_CASES = ['PLAY-01 fists course', 'PLAY-01 claymore course', 'PLAY-01 fists course run']


def lua_escape(name):
    return '^' + re.sub(r'([\^$()%.\[\]*+\-?])', r'%\1', name) + '$'


def run_stage(out, suite, patterns, envs):
    subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                    str(ROOT / 'tools-workspace/Assert-DiskSpace.ps1')], check=True, stdout=subprocess.DEVNULL)
    # The runner refuses an existing output directory (checks.sh does the same rm).
    shutil.rmtree(out, ignore_errors=True)
    Path(str(out) + '.runner.log').unlink(missing_ok=True)
    cfg = TREE / 'files/data/ragdoll'
    env = ['OPENMW_RAGDOLL_SETTINGS=%s' % (cfg / 'ragdoll.cfg'), 'OPENMW_RAGDOLL_ACTIVE_BODY=%s' % (cfg / 'active-body.cfg'),
           'OPENMW_RAGDOLL_FOOTFALL=1', 'OPENMW_RAGDOLL_DAMAGE_TRACE=1',
           # The user plays with PGC on since 2026-09-30 ("PGC integrated is the
           # superior controller"); tests run what is played (later --env wins).
           'OPENMW_RAGDOLL_PGC=1,1,1,1', 'OPENMW_RAGDOLL_PGC_BRACE=1'] + list(envs)
    head = subprocess.run(['git', '-C', str(TREE), 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
    # RAGDOLL_TEST_ENGINE: another exe (an A/B against a kept build, e.g.
    # openmw-play.exe); default the one test exe.
    engine = os.environ.get('RAGDOLL_TEST_ENGINE') or str(BUILD / 'RelWithDebInfo/openmw.exe')
    cmd = [sys.executable, str(TREE / 'tools/play04/run_watched.py'), '--engine', engine,
           '--engine-source', head + '-dirty', '--output', str(out), '--suite', suite,
           '--test-filter', ' || '.join(patterns), '--parallel', '4', '--suite-timeout', '900',
           '--diagnostic-budget-mib', '1200']
    # RAGDOLL_TEST_RENDER_FPS: 30, 60 (default) or 120 rendered frames a second.
    if os.environ.get('RAGDOLL_TEST_RENDER_FPS'):
        cmd += ['--render-fps', os.environ['RAGDOLL_TEST_RENDER_FPS']]
    for e in env:
        cmd += ['--env', e]
    prov = ledger.provenance(env, engine)  # before the run can see later edits
    with open(str(out) + '.runner.log', 'w') as log:
        code = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
    ledger.write_provenance(out, prov)
    if not (out / 'result-summary.json').exists():
        print('runner failed (exit %d), see %s.runner.log' % (code, out))


def statuses(out):
    try:
        s = json.load(open(out / 'result-summary.json'))
    except (OSError, ValueError):
        return {}
    return {c['name']: c['status'] for su in s['suites'] for c in su['cases']}


def frames(out):
    by = defaultdict(list)
    for path in sorted(out.glob('shard-*/*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get('kind') == 'frame' and r.get('test'):
                    by[r['test']].append(r)
    return by


def mode(r):
    return str((((r.get('state') or {}).get('controller')) or {}).get('mode', '')).lower()


def end_lean(checks, rows):
    """A stuck lean (play test 2026-09-28, "smooth criminal lean"): the torso
    still tilted over the case's last second, standing or stopped."""
    end = rows[-1].get('elapsed', 0)
    ups = [((r.get('state') or {}).get('controller') or {}).get('torsoUpDot') for r in rows
           if r.get('elapsed', 0) >= end - 1.0]
    ups = [u for u in ups if isinstance(u, (int, float))]
    if ups:
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, min(ups)))))
        checks.append(('end lean', '%.0f deg' % tilt, tilt < BARS['end_lean']))


def walk_checks(out):
    """{case: [(check, value, ok)]}"""
    res = defaultdict(list)
    by = frames(out)
    for test, rows in by.items():
        modes = [mode(r) for r in rows]
        if test.split(' ', 2)[-1].startswith('vanilla '):
            # Morrowind's knockdown/flinch clip set on a powered body (user
            # 2026-09-29: no vanilla reaction clips): the animation's pelvis
            # stays up and its chest does not jerk.
            def target(r, body):
                return (((r.get('state') or {}).get('bodies') or {}).get(body) or {}).get('targetPosition')
            rows_ = [r for r in rows if target(r, 'pelvis') and target(r, 'spine2')]
            if rows_:
                z0 = target(rows_[0], 'pelvis')[2]
                c0 = target(rows_[0], 'spine2')
                drop = max(z0 - target(r, 'pelvis')[2] for r in rows_)
                jerk = max(math.dist(target(r, 'spine2'), c0) for r in rows_)
                res[test].append(('clip pelvis drop', '%.0f' % drop, drop < 15))
                res[test].append(('clip chest moves', '%.0f' % jerk, jerk < 15))
            continue
        if 'powered lying' in test:
            # Powered during a knockout clip ("I cant get up", 2026-09-29):
            # it may fall once, but must end up standing and stay up.
            first = next((i for i, m in enumerate(modes) if m == 'standing'), None)
            res[test].append(('stands up', 'never' if first is None else '%.1f s' % rows[first]['elapsed'],
                              first is not None))
            if first is not None:
                falls = sum(1 for a, b in zip(modes[first:], modes[first + 1:]) if b == 'falling' and a != 'falling')
                res[test].append(('falls after standing', falls, falls == 0))
                end_lean(res[test], rows)
            continue
        if 'knockdown' in test:
            if 'getting_up' not in modes:
                if PLAY['physical_knockdown'] and 'fallen' not in modes:
                    res[test].append(('knocked down', 'no (physical knockdowns: the legs caught it)', True))
                    end_lean(res[test], rows)
                else:
                    res[test].append(('get-up', 'none', False))
                continue
            i0 = modes.index('getting_up')
            i1 = next((i for i in range(i0, len(modes)) if modes[i] == 'standing'), None)
            if i1 is None:
                res[test].append(('stands up', 'never', False))
                continue
            P = lambda i: rows[i]['state']['bodies']['pelvis']['position']  # noqa: E731
            slide = math.hypot(P(i1)[0] - P(i0)[0], P(i1)[1] - P(i0)[1])
            took = rows[i1]['elapsed'] - rows[i0]['elapsed']
            res[test].append(('get-up slide', '%.0f' % slide, slide < BARS['getup_slide']))
            res[test].append(('get-up time', '%.1f s' % took, took < BARS['getup_time']))
            res[test].append(('falls after get-up', modes[i1:].count('falling'), 'falling' not in modes[i1:]))
            end_lean(res[test], rows)
            continue
        fell = next((r.get('segment') for r, m in zip(rows, modes) if m in FALL_MODES), None)
        res[test].append(('no fall', fell or '-', fell is None))
        if fell is None:
            end_lean(res[test], rows)
        if test.endswith('no input'):
            w = sorted(math.degrees(math.sqrt(sum(x * x for x in r['state']['bodies']['pelvis']['angularVelocity'])))
                       for r in rows if 'bodies' in r.get('state', {}))
            p90 = w[int(0.9 * (len(w) - 1))] if w else float('nan')
            res[test].append(('idle shake p90', '%.1f' % p90, p90 < BARS['idle_p90']))
    # Course scores: one case's step timing is +-10 noisy, so the bars are on
    # the median over the course cases' walk (and run) segments.
    scores = defaultdict(lambda: defaultdict(list))
    for test, rows in by.items():
        segs = defaultdict(list)
        for r in rows:
            seg = r.get('segment') or ''
            if seg.startswith('c ') and 'bodies' in r.get('state', {}):
                segs[seg[2:]].append(r)
        for seg, part in segs.items():
            kind = {'walk': 'walk', 'walk again': 'walk', 'run': 'run', 'run again': 'run'}.get(seg)
            sc = course_report.score(part, seg) if kind else None
            for key in ('stepT', 'stepL', 'side', 'sink', 'scuff') if sc else ():
                if sc[key] == sc[key]:
                    scores[kind][key].append(float(sc[key]))
    for kind, keys in scores.items():
        for key, values in keys.items():
            values.sort()
            med = values[len(values) // 2]
            lo, hi = BARS['course'][kind][key]
            res[COURSE].append(('%s %s median of %d' % (kind, key, len(values)), '%.2f' % med, lo <= med <= hi))
    # Pushes.
    for test, row in push_report.summarise(str(out)).items():
        if 'knockdown' in test:
            continue
        rec = row['recover']
        res[test].append(('push recover', '-' if rec is None else '%.2f s' % rec,
                          not row['fell'] and rec is not None and rec < BARS['push_recover']))
    return res


def damage_lines(out):
    by = defaultdict(list)
    for log in sorted(out.glob('shard-*/*/openmw.log')) + sorted(out.glob('*/openmw.log')):
        case = None
        for line in open(log, encoding='utf-8', errors='replace'):
            m = re.search(r'TEST_START\t\d+\t(.*)', line)
            if m:
                case = m.group(1).strip()
            elif 'RAGDOLL_DAMAGE ' in line and case:
                by[case].append(dict(re.findall(r'(\w+)=(\S+)', line)))
    return by


def combat_checks(out):
    res = defaultdict(list)
    for case, lines in damage_lines(out).items():
        if case.startswith('FIX-08'):
            res[case].append(('physical blows', len(lines), len(lines) >= BARS['blows_min']))
        if case.startswith('DMG-04'):
            final = sorted(float(d['final']) for d in lines if float(d.get('final', 0)) > 0)
            med = final[len(final) // 2] if final else float('nan')
            lo, hi = BARS['duel_damage']
            res[case].append(('median blow damage', '%.1f (%d blows)' % (med, len(final)), lo <= med <= hi))
    return res


def balance_checks(out):
    """Falls only: the walking bars (push recovery 0.9 s, end lean 12 deg)
    do not fit slopes and edges - the slope stance leans into a 40 deg slope
    by design. Steps, skid, slide and steady time: behaviour_report.py."""
    res = defaultdict(list)
    for case, checks in walk_checks(out).items():
        res[case] = [c for c in checks if c[0] == 'no fall']
    return res


def move_checks(out):
    """The movement bars (BARS['move']) per case; a measure a case does not
    produce (no run segment, no reversal) is not checked."""
    res = defaultdict(list)
    for case, m in move_report.per_case(out).items():
        for key, bar in BARS['move'].items():
            if key in m:
                res[case].append((key, '%.2f' % m[key], m[key] <= bar))
    return res


CHECKS = {'walk': walk_checks, 'balance': balance_checks, 'combat': combat_checks, 'duel': combat_checks,
          'move': move_checks,
          'drop': lambda out: {}, 'life': lambda out: {}, 'visible': lambda out: {}, 'release': lambda out: {}}


def play_envs():
    """The play launcher's switches (its "set OPENMW_..." lines, minus file paths)."""
    play = []
    for line in (ROOT / 'Run-Game-MotionBricks.cmd').read_text(errors='replace').splitlines():
        m = re.match(r'^set "?(OPENMW_[A-Z0-9_]+)=([^"]*)"?\s*$', line.strip())
        if m and '%~dp0' not in m.group(2):
            play.append('%s=%s' % (m.group(1), m.group(2)))
    return play


def evaluate(out):
    status = statuses(out)
    res = defaultdict(list)
    for name, st in status.items():
        res[name].append(('asserts', st, st == 'passed'))
    return status, res


def check_stage(stage, out):
    status, res = evaluate(out)
    for case, items in CHECKS[stage](out).items():
        res[case].extend(items)
    if not status:
        res['(runner)'].append(('result summary', 'missing', False))
    if PLAY['physical_knockdown']:
        for case in [c for c in res if c.startswith('SCALE-03')]:
            del res[case]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('--env', action='append', default=[])
    ap.add_argument('--stage', action='append', choices=list(STAGES))
    ap.add_argument('--suite', action='append', choices=list(SUITES))
    ap.add_argument('--play', action='store_true', help="the play launcher's switches first (--env still wins)")
    ap.add_argument('--report-only', action='store_true', help='re-check existing output, run nothing')
    ap.add_argument('--verbose', action='store_true', help='list every check, not only failures')
    args = ap.parse_args()
    if args.play:
        args.env = play_envs() + args.env
    apply_play_switches(args.env)
    lines, failed_any = [], False
    if PLAY['armed_knee'] or PLAY['physical_knockdown']:
        lines.append('(play switches: armed knee %.2f -> sink bars walk %.1f run %.1f; physical knockdowns %s%s)' % (
            PLAY['armed_knee'], BARS['course']['walk']['sink'][0], BARS['course']['run']['sink'][0],
            PLAY['physical_knockdown'], ', SCALE-03 skipped' if PLAY['physical_knockdown'] else ''))
    stages = list(args.stage or [])
    for suite in args.suite or []:
        stages += [st for st in SUITES[suite] if st not in stages]
    for stage in stages or ['walk', 'combat', 'duel']:
        suite, patterns = STAGES[stage]
        out = ROOT / ('runtime-SENT-%s-%s' % (args.name, stage))
        envs = args.env + STAGE_ENV.get(stage, [])
        if not args.report_only:
            run_stage(out, suite, patterns, envs)
        first = check_stage(stage, out)
        ledger.record(out, checks=first, stage=stage)
        bad = sorted(c for c, items in first.items() if not all(ok for _, _, ok in items))
        again = {}
        if bad and '(runner)' not in bad:
            out2 = ROOT / ('runtime-SENT-%s-%s-again' % (args.name, stage))
            if not args.report_only:
                again_cases = sorted({x for c in bad for x in (COURSE_CASES if c == COURSE else [c])})
                run_stage(out2, suite, [lua_escape(c) for c in again_cases], envs)
            if out2.exists():
                again = check_stage(stage, out2)
                ledger.record(out2, checks=again, stage=stage, rerun=True)
        lines.append('== %s: %d cases' % (stage, len(first)))
        for case in sorted(first):
            items = first[case]
            if case not in bad:
                if args.verbose:
                    lines.append('  ok    %s: %s' % (case, '; '.join('%s %s' % (k, v) for k, v, _ in items)))
                continue
            second = again.get(case)
            verdict = 'FLAKY' if second and all(ok for _, _, ok in second) else 'FAIL'
            failed_any |= verdict == 'FAIL'
            why = '; '.join('%s %s' % (k, v) for k, v, ok in items if not ok)
            why2 = '; '.join('%s %s' % (k, v) for k, v, ok in (second or []) if not ok) if second else 'not rerun'
            lines.append('  %-5s %s\n        first: %s\n        again: %s' % (verdict, case, why, why2 or 'ok'))
        lines.append('  ok    %d of %d' % (len(first) - len(bad), len(first)))
    lines.append('SENTINEL %s' % ('FAIL' if failed_any else 'PASS'))
    text = '\n'.join(lines)
    print(text)
    (ROOT / ('runtime-SENT-%s-sentinel.txt' % args.name)).write_text(text + '\n')
    sys.exit(1 if failed_any else 0)


if __name__ == '__main__':
    main()
