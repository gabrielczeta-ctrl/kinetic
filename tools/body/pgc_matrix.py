"""PGC robustness matrix (2026-10-09): how the balance controller catches,
stumbles and falls across pushes, locomotion states and landings, run in the
real game (no offline simulator: these are the engine's own physics and
controllers, so a result here is the game's result).

usage:
  pgc_matrix.py run NAME [--reps N] [--set SET] [--no-play] [K=V ...]
  pgc_matrix.py report NAME [NAME_B]       (B = the A/B comparison)
  pgc_matrix.py cases NAME                 (per-case table, all repeats)

run: the play launcher's switches (Run-Game-MotionBricks.cmd) then K=V, the
engine from RAGDOLL_TEST_ENGINE (default build-pgc's openmw.exe), output
runtime-ONE-pgc-NAME-rK in the dev root. Physics is not deterministic, so
every comparison reads medians and fall rates over the repeats.
"""
import gzip
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import push_report  # noqa: E402
import sentinel  # noqa: E402

ROOT = sentinel.ROOT
ENGINE = ROOT / 'build-pgc/RelWithDebInfo/openmw.exe'
SUITE = 'test_play01_walking'

# Case sets (Lua patterns). 'core' is the matrix; 'saves' the movements with
# no dedicated save yet; 'quick' a 3-minute smoke of the riskiest cells.
SETS = {
    'core': [
        '^PLAY%-01 unarmed push stand [a-z]+ [2345]000$',
        '^PLAY%-01 unarmed push walk [a-z]+ [2345]000$',
        '^PLAY%-01 unarmed push run [a-z]+ [23]000$',
        '^PLAY%-01 warhammer push stand [a-z]+ 3000$',
        '^PLAY%-01 warhammer push walk [a-z]+ 3000$',
        '^PLAY%-01 unarmed STOPPUSH%-01 [a-z]+ 3000 at',
        '^PLAY%-01 unarmed near push [a-z]+ [0-9]+ 8000$',
        '^PLAY%-01 unarmed RUNBACK%-01',
        '^PLAY%-01 unarmed JUMP%-01',
        '^PLAY%-01 unarmed DROP%-01 balmora',
        '^PLAY%-01 unarmed stairs drop off$',
        '^PLAY%-01 unarmed knockdown 6000$',
        '^PLAY%-01 unarmed KNOCKRUN%-01 get up and run$',
    ],
    'quick': [
        '^PLAY%-01 unarmed push walk [a-z]+ 3000$',
        '^PLAY%-01 unarmed push run front 3000$',
        '^PLAY%-01 unarmed near push stand [0-9]+ 8000$',
        '^PLAY%-01 unarmed RUNBACK%-01 run backward$',
        '^PLAY%-01 unarmed DROP%-01 balmora',
    ],
    'saves': [
        '^PLAY%-01 unarmed SAVE%-01',
    ],
    'trace': [
        '^PLAY%-01 unarmed stairs drop off$',
        '^PLAY%-01 unarmed JUMP%-01 ledge jump high running$',
        '^PLAY%-01 unarmed SAVE%-01 push landing front 2500$',
        '^PLAY%-01 unarmed SAVE%-01 push walk downhill 3000$',
        '^PLAY%-01 unarmed push run side 3000$',
        '^PLAY%-01 unarmed push walk front 3000$',
        '^PLAY%-01 unarmed near push stand 0 8000$',
        '^PLAY%-01 unarmed stairs up and back$',
        '^PLAY%-01 claymore course balmora down$',
    ],
    'landing': [
        '^PLAY%-01 unarmed SAVE%-01 push landing front 2500$',
        '^PLAY%-01 unarmed JUMP%-01',
        '^PLAY%-01 unarmed jump forward$',
        '^PLAY%-01 unarmed DROP%-01 balmora',
        '^PLAY%-01 unarmed SAVE%-01 push walk downhill 3000$',
        '^PLAY%-01 claymore course balmora down$',
    ],
    'stand': [
        '^PLAY%-01 unarmed push stand [a-z]+ [2345]000$',
        '^PLAY%-01 unarmed STOPPUSH%-01 [a-z]+ 3000 at',
        '^PLAY%-01 warhammer push stand [a-z]+ 3000$',
    ],
    'near': [
        '^PLAY%-01 unarmed near push stand [0-9]+ 8000$',
        '^PLAY%-01 unarmed near push stand [0-9]+ 12000$',
    ],
    'stairs': [
        '^PLAY%-01 unarmed stairs up and back$',
        '^PLAY%-01 unarmed STAIRS%-01 run up$',
        '^PLAY%-01 claymore course balmora [ud][po]w?n?$',
        '^PLAY%-01 unarmed strafe left$',
        '^PLAY%-01 unarmed strafe right$',
    ],
}

# Row groups: (label, regex on the case name, may it fall?)
GROUPS = [
    ('stand push 2000-3000', r'unarmed push stand \w+ [23]000$', False),
    ('walk push 2000-3000', r'unarmed push walk \w+ [23]000$', False),
    ('stand push 4000-5000', r'unarmed push stand \w+ [45]000$', True),
    ('walk push 4000-5000', r'unarmed push walk \w+ [45]000$', True),
    ('run push 2000-3000', r'unarmed push run ', True),
    ('armed push 3000', r'warhammer push ', False),
    ('stop push 3000', r'STOPPUSH', True),
    ('near push stand 8000', r'near push stand \d+ 8000', True),
    ('near push stand 12000', r'near push stand \d+ 12000', True),
    ('near push walk 8000', r'near push walk ', True),
    ('run backward', r'RUNBACK', False),
    ('ledge jumps', r'JUMP-01', False),
    ('jump forward', r'unarmed jump forward$', False),
    ('bank run off', r'DROP-01 balmora', False),
    ('stairs drop off', r'stairs drop off', False),
    ('stairs walk/run', r'stairs up and back|STAIRS-01', False),
    ('hill course armed', r'course balmora', True),
    ('flat strafe', r'unarmed strafe (left|right)$', False),
    ('knockdown + get-up', r'knockdown 6000|KNOCKRUN', True),
    ('saves', r'SAVE-01', True),
]


def launcher_envs():
    play = []
    for line in (ROOT / 'Run-Game-MotionBricks.cmd').read_text(errors='replace').splitlines():
        m = re.match(r'^set "?(OPENMW_[A-Z0-9_]+)=([^"]*)"?\s*$', line.strip())
        if m and '%~dp0' not in m.group(2):
            play.append('%s=%s' % (m.group(1), m.group(2)))
    return play


def engines_running():
    try:
        text = subprocess.run(['tasklist'], capture_output=True, text=True).stdout.lower()
    except OSError:
        return False
    # Any OpenMW exe: openmw.exe, openmw-play.exe, named test copies
    # (openmw-settle.exe ...) and the frozen alphas.
    return re.search(r'^openmw[\w.-]*\.exe', text, re.M) is not None


WANT_OTHER = ROOT / 'runtime-LEASE-WANT-combat'
WANT_MINE = ROOT / 'runtime-LEASE-WANT-pgc'


def wait_for_engine():
    """Engine sharing with the combat session (10-09): its want-file first
    (it holds or wants the engine), then mine (it waits for me), then no
    engine running. release_engine() after the rep deletes mine."""
    waited = 0
    while WANT_OTHER.exists():
        if waited % 300 == 0:
            print('  combat session wants the engine, waiting', flush=True)
        time.sleep(20)
        waited += 20
    WANT_MINE.write_text('pgc matrix %s\n' % time.ctime())
    while engines_running():
        if waited % 300 == 0:
            print('  engine busy (another session or a game), waiting', flush=True)
        time.sleep(15)
        waited += 15


def out_dirs(name):
    return sorted(p for p in ROOT.glob('runtime-ONE-pgc-%s-r*' % name) if p.is_dir())


def run(args):
    name = args[0]
    reps, case_set, play = 3, 'core', True
    envs = []
    i = 1
    while i < len(args):
        a = args[i]
        if a == '--reps':
            reps, i = int(args[i + 1]), i + 1
        elif a == '--set':
            case_set, i = args[i + 1], i + 1
        elif a == '--no-play':
            play = False
        elif '=' in a:
            envs.append(a)
        i += 1
    envs = (launcher_envs() if play else []) + envs
    sentinel.apply_play_switches(envs)
    os.environ.setdefault('RAGDOLL_TEST_ENGINE', str(ENGINE))
    patterns = []
    for s in case_set.split('+'):
        patterns += SETS[s]
    try:
        run_reps(name, reps, patterns, envs)
    finally:
        WANT_MINE.unlink(missing_ok=True)


def run_reps(name, reps, patterns, envs):
    for rep in range(1, reps + 1):
        out = ROOT / ('runtime-ONE-pgc-%s-r%d' % (name, rep))
        print('run %s rep %d -> %s' % (name, rep, out.name), flush=True)
        # One engine at a time (another session's tests, or the user's game):
        # wait for a free engine, retry when the lease was taken first.
        for attempt in range(240):
            wait_for_engine()
            sentinel.run_stage(out, SUITE, patterns, envs)
            log = Path(str(out) + '.runner.log')
            if not (log.exists() and 'owns the engine lease' in log.read_text(errors='replace')):
                break
            time.sleep(15)
        WANT_MINE.unlink(missing_ok=True)
        time.sleep(30)  # a gap for the other session's waiter
        st = sentinel.statuses(out)
        bad = [c for c, s in st.items() if s != 'passed']
        print('  %d cases, %d not passed %s' % (len(st), len(bad), bad[:6]), flush=True)


def records(out):
    for path in sorted(out.rglob('*.player-diagnostics.jsonl.gz')):
        if 'discovery' in path.parts:
            continue
        try:
            with gzip.open(path, 'rt', encoding='utf-8') as stream:
                for line in stream:
                    try:
                        yield json.loads(line)
                    except ValueError:
                        continue
        except (EOFError, OSError):
            continue


def general(out):
    """Every case: fell (and when), seconds bracing/stumbling, worst torso
    tilt while not fallen, final tilt, pelvis lowest height while upright."""
    rows = defaultdict(list)
    impacts = {}
    for r in records(out):
        if r.get('kind') == 'frame' and r.get('test'):
            rows[r['test']].append(r)
        elif r.get('kind') == 'impact' and r.get('test'):
            impacts[r['test']] = r.get('simulation_time', 0)
    res = {}
    for test, frames in rows.items():
        frames.sort(key=lambda r: r.get('simulation_time', 0))
        fell_t, hover, tilt, prev = None, 0.0, 0.0, None
        t_start = frames[0].get('simulation_time', 0)
        for r in frames:
            t = r.get('simulation_time', 0)
            c = ((r.get('state') or {}).get('controller')) or {}
            m = str(c.get('mode', '')).lower()
            if m in push_report.FALL_MODES and fell_t is None:
                fell_t = t - t_start
            if prev is not None and m in ('bracing', 'stumbling'):
                hover += t - prev
            up = c.get('torsoUpDot')
            if fell_t is None and isinstance(up, (int, float)):
                tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, up)))))
            prev = t
        last = ((frames[-1].get('state') or {}).get('controller')) or {}
        up = last.get('torsoUpDot')
        end_tilt = math.degrees(math.acos(max(-1.0, min(1.0, up)))) if isinstance(up, (int, float)) else float('nan')
        # Pulled back (build 55's stagger drag-back): how far the center of
        # mass was carried from where the push found it (horizontal, 3 s,
        # standing pushes only) against where it ends up.
        pulled = None
        t0 = impacts.get(test)
        if t0 is not None and ' stand ' in test:
            start, far, final = None, 0.0, None
            for r in frames:
                t = r.get('simulation_time', 0)
                com = (r.get('state') or {}).get('centerOfMass')
                if t < t0 or not isinstance(com, list):
                    continue
                if t - t0 > 3.0:
                    break
                start = start or com
                d = math.hypot(com[0] - start[0], com[1] - start[1])
                far, final = max(far, d), d
            if final is not None:
                pulled = far - final
        res[test] = {'fell': fell_t is not None, 'fell_t': fell_t, 'hover_total': hover, 'tilt': tilt,
                     'end_tilt': end_tilt, 'end_mode': str(last.get('mode', '')), 'pulled': pulled}
    return res


def collect(name):
    """{case: [per-repeat dict]} merging the push table and the general one."""
    per = defaultdict(list)
    # Summaries of runs whose raw output was deleted (disk) live in the archive.
    archive = ROOT / 'analysis-artifacts/pgc-matrix-2026-10-10'
    present = {out.name for out in out_dirs(name)}
    for kept in sorted(archive.glob('%s-r[0-9]*.json' % name)):
        if 'runtime-ONE-pgc-' + kept.stem not in present:
            for test, row in json.loads(kept.read_text()).items():
                per[test.replace('PLAY-01 ', '')].append(row)
    for out in out_dirs(name):
        if not (out / 'result-summary.json').exists():
            continue  # still running or failed
        cache = out / 'pgc-summary.json'
        if cache.exists():
            rows = json.loads(cache.read_text())
        else:
            st = sentinel.statuses(out)
            g = general(out)
            try:
                p = push_report.summarise(out)
            except Exception:  # noqa: BLE001 - layouts differ between runner versions
                p = {}
            rows = {}
            for test, row in g.items():
                row = dict(row)
                row.update(p.get(test, {}))
                row['status'] = st.get(test, '?')
                rows[test] = row
            cache.write_text(json.dumps(rows))
        for test, row in rows.items():
            per[test.replace('PLAY-01 ', '')].append(row)
    return per


def med(values):
    values = [v for v in values if isinstance(v, (int, float)) and not math.isnan(v)]
    return statistics.median(values) if values else float('nan')


def group_rows(per):
    out = []
    for label, rx, may_fall in GROUPS:
        cases = {c: rows for c, rows in per.items() if re.search(rx, c)}
        if not cases:
            continue
        rows = [r for rs in cases.values() for r in rs]
        falls = sum(1 for r in rows if r['fell'])
        out.append({
            'group': label, 'cases': len(cases), 'n': len(rows), 'falls': falls, 'may_fall': may_fall,
            'steps': med([r.get('steps') for r in rows]),
            'slide': med([r.get('slide') for r in rows]),
            'recover': med([r.get('recover') for r in rows if r.get('recover') is not None]),
            'never': sum(1 for r in rows if 'recover' in r and r.get('recover') is None and not r['fell']),
            'drift': med([r.get('drift') for r in rows]),
            'pulled': med([r.get('pulled') for r in rows]),
            'hover': med([r.get('hover_total') for r in rows]),
            'tilt': med([r.get('tilt') for r in rows]),
            'end_tilt': max([r.get('end_tilt', 0) for r in rows if not r['fell']] or [float('nan')]),
            'failed': sum(1 for r in rows if r['status'] not in ('passed', '?')),
        })
    return out


def fmt(v, f='%.1f'):
    return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else f % v


def report(args):
    names = args[:2]
    tables = [group_rows(collect(n)) for n in names]
    print('PGC matrix: %s  (falls/n, medians over cases x repeats; steps/slide/drift: 2 s after the push)' % ' vs '.join(names))
    head = '%-22s %-3s %9s %6s %6s %7s %5s %6s %6s %6s %6s %7s %5s'
    print(head % ('group', 'run', 'falls', 'steps', 'slide', 'recover', 'never', 'drift', 'pulled', 'hover', 'tilt', 'endtilt', 'fail'))
    labels = [g[0] for g in GROUPS]
    for label in labels:
        for i, t in enumerate(tables):
            row = next((r for r in t if r['group'] == label), None)
            if not row:
                continue
            flag = '' if row['may_fall'] or row['falls'] == 0 else ' !'
            print(head % (label[:22], 'AB'[i], '%d/%d%s' % (row['falls'], row['n'], flag), fmt(row['steps']),
                          fmt(row['slide']), fmt(row['recover'], '%.2f'), row['never'], fmt(row['drift']),
                          fmt(row['pulled']), fmt(row['hover'], '%.2f'), fmt(row['tilt'], '%.0f'), fmt(row['end_tilt'], '%.0f'),
                          row['failed']))


def cases(args):
    per = collect(args[0])
    print('%-46s %-6s %-12s %6s %6s %7s %6s %6s %s' % ('case', 'falls', 'fell_t', 'steps', 'slide', 'recover', 'hover', 'tilt', 'status'))
    for case in sorted(per):
        rows = per[case]
        falls = sum(r['fell'] for r in rows)
        fell_t = ','.join('%.1f' % r['fell_t'] for r in rows if r['fell'])
        print('%-46s %-6s %-12s %6s %6s %7s %6s %6s %s' % (
            case[:46], '%d/%d' % (falls, len(rows)), fell_t[:12], fmt(med([r.get('steps') for r in rows])),
            fmt(med([r.get('slide') for r in rows])), fmt(med([r.get('recover') for r in rows if r.get('recover') is not None]), '%.2f'),
            fmt(med([r.get('hover_total') for r in rows]), '%.2f'), fmt(med([r.get('tilt') for r in rows]), '%.0f'),
            ','.join(sorted({r['status'] for r in rows}))))


if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in ('run', 'report', 'cases'):
        sys.exit(__doc__)
    {'run': run, 'report': report, 'cases': cases}[sys.argv[1]](sys.argv[2:])
