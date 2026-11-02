"""Results ledger: one JSON line per test case run, kept after the run's output
is cleaned up (tools-workspace is outside the cleanup's reach).

usage: ledger.py summary [CASE_PATTERN] [--since YYYY-MM-DD] [--by commit|exe|sw]
       ledger.py history CASE_PATTERN [--metric NAME] [--last N]
       ledger.py switches HASH            the switch set behind a hash
       ledger.py add RUN_DIR [...]        record a run directory now (metrics recomputed)
       ledger.py backfill [--no-metrics]  record every runtime-SENT-*/runtime-ONE-* run not yet recorded

Why (2026-10-31 review): physics is not repeatable, so a case's verdict means
something only next to its own history - how often it passes, and its spread,
per build. Run directories are deleted after two days; this file is not.

A row: t (the run's time, UTC), run (directory), stage, suite, case, status
(the asserts), err, commit (HEAD at launch), dirty (hash of `git diff HEAD`
at launch: '' clean, null unknown), exe and exe_sha, sw (hash of the switch
set, see results-switches.jsonl; null unknown), cfg (hash of ragdoll.cfg +
active-body.cfg), fps (rendered), rerun (a sentinel "again" run), m (the
checks' measured values: numbers where the value starts with one), ok (each
check against its bar at the time), pass (asserts and every bar), res (the
whole run's machine load from RUN_DIR/resource-usage.json, the same on each
of its rows: wall_s, cpu_avg/peak %, engine_cores_avg/peak, engine_gb_peak,
free_gb_min, cores, mem_gb; null before 2026-11-01).

sentinel.run_stage writes RUN_DIR/ledger-provenance.json at launch;
sentinel.py and cases.py record after their checks. Other run_stage callers:
`ledger.py add RUN_DIR`. A run is recorded once (RUN_DIR/ledger-recorded).
RAGDOLL_LEDGER moves the file (tests of this tool).
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TREE = HERE.parent.parent
LEDGER = Path(os.environ.get('RAGDOLL_LEDGER') or ROOT / 'tools-workspace/results-ledger.jsonl')
SWITCHES = LEDGER.with_name('results-switches.jsonl')
PROVENANCE = 'ledger-provenance.json'
RECORDED = 'ledger-recorded'
# File paths differ per checkout; their content goes into 'cfg' instead.
PATH_KEYS = ('OPENMW_RAGDOLL_SETTINGS', 'OPENMW_RAGDOLL_ACTIVE_BODY')


def _sha(data, n=10):
    return hashlib.sha256(data).hexdigest()[:n]


def switch_set(envs):
    """The effective K=V set (later wins, as in the runner), sorted, paths left out."""
    last = {}
    for e in envs:
        key, _, value = e.partition('=')
        if key not in PATH_KEYS:
            last[key] = value
    return ['%s=%s' % kv for kv in sorted(last.items())]


def provenance(envs, engine):
    """Taken at launch, before the run can see later edits."""
    def git(*args):
        return subprocess.run(['git', '-C', str(TREE)] + list(args), capture_output=True).stdout
    commit = git('rev-parse', '--short=10', 'HEAD').decode().strip()
    diff = git('diff', 'HEAD', '--binary')
    cfg = TREE / 'files/data/ragdoll'
    for e in envs:
        key, _, value = e.partition('=')
        if key == 'OPENMW_RAGDOLL_SETTINGS':
            cfg = Path(value).parent
    cfg_bytes = b''.join((cfg / f).read_bytes() for f in ('ragdoll.cfg', 'active-body.cfg') if (cfg / f).exists())
    sw = switch_set(envs)
    return {'commit': commit, 'dirty': _sha(diff) if diff else '', 'exe': str(engine),
            'sw': _sha('\n'.join(sw).encode(), 8), 'env': sw, 'cfg': _sha(cfg_bytes, 8),
            'fps': int(os.environ.get('RAGDOLL_TEST_RENDER_FPS') or 60),
            'started': datetime.now(timezone.utc).isoformat(timespec='seconds')}


def write_provenance(out, prov):
    """After the run: the runner refuses an output directory that exists."""
    if Path(out).is_dir():
        (Path(out) / PROVENANCE).write_text(json.dumps(prov, indent=1))


def _number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return round(float(value), 4)
    if value in ('-', None):  # 'no fall': nothing to report
        return None
    m = re.match(r'\s*(-?\d+(?:\.\d+)?)', str(value))
    return float(m.group(1)) if m else str(value)


def default_checks(out, suite, envs=None):
    """The checks the sentinel would apply to a suite (cases.py, and runs recorded
    after the fact); envs: the run's switches, which move some bars."""
    sys.path.insert(0, str(HERE))
    import sentinel
    if envs is not None:
        sentinel.apply_play_switches(envs)
    res = defaultdict(list)
    if suite == 'test_play01_walking':
        for case, items in sentinel.walk_checks(out).items():
            res[case].extend(items)
        for case, items in sentinel.move_checks(out).items():
            res[case].extend(items)
    elif suite in ('test_combat_fixes', 'test_duel'):
        for case, items in sentinel.combat_checks(out).items():
            res[case].extend(items)
    return res


def _switch_known(sw, env):
    if not sw:
        return
    if SWITCHES.exists():
        for line in SWITCHES.read_text(encoding='utf-8').splitlines():
            if line.startswith('{"sw": "%s"' % sw):
                return
    with open(SWITCHES, 'a', encoding='utf-8') as f:
        f.write(json.dumps({'sw': sw, 'env': env}) + '\n')


def record(out, checks=None, stage=None, rerun=False, metrics=True):
    """Append the run's cases to the ledger. checks: {case: [(name, value, ok)]}
    as the sentinel computes them (None: recompute; metrics=False: asserts only)."""
    out = Path(out)
    try:
        summary = json.loads((out / 'result-summary.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return 0
    stamp = summary.get('generated', '')
    marker = out / RECORDED
    if marker.exists() and marker.read_text().strip() == stamp:
        return 0
    try:
        prov = json.loads((out / PROVENANCE).read_text())
    except (OSError, ValueError):
        prov = {}
    try:
        manifest = json.loads((out / 'run-manifest.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        manifest = {}
    try:
        res = json.loads((out / 'resource-usage.json').read_text())['summary']
    except (OSError, ValueError, KeyError):
        res = None
    suite = summary['suites'][0]['name'] if summary.get('suites') else None
    if checks is None and metrics and suite:
        checks = default_checks(out, suite, prov.get('env'))
    checks = checks or {}
    exe = prov.get('exe') or manifest.get('engine') or ''
    base = {
        't': stamp, 'run': out.name, 'stage': stage, 'suite': suite,
        'commit': prov.get('commit') or (summary.get('source_commit') or '')[:10] or None,
        'dirty': prov.get('dirty'), 'exe': Path(exe).name or None,
        'exe_sha': (manifest.get('engine_sha256') or '')[:12] or None,
        'sw': prov.get('sw'), 'cfg': prov.get('cfg'),
        'fps': prov.get('fps') or (manifest.get('arguments') or {}).get('render_fps'),
        'rerun': rerun or out.name.endswith('-again'), 'res': res,
    }
    rows = []
    for su in summary.get('suites', []):
        for c in su.get('cases', []):
            items = [i for i in checks.get(c['name'], []) if i[0] != 'asserts']
            row = dict(base, suite=su['name'], case=c['name'], status=c.get('status'),
                       ms=round(c.get('duration_ms') or 0), err=(c.get('error') or '')[:200] or None,
                       m={k: _number(v) for k, v, _ in items}, ok={k: bool(ok) for k, _, ok in items})
            row['pass'] = c.get('status') == 'passed' and all(row['ok'].values())
            rows.append(row)
    # Aggregates that are not a case (the course medians) get their own row.
    for name, items in checks.items():
        if name.startswith('(') or any(r['case'] == name for r in rows):
            continue
        row = dict(base, case=name, status=None, ms=None, err=None,
                   m={k: _number(v) for k, v, _ in items}, ok={k: bool(ok) for k, _, ok in items})
        row['pass'] = all(row['ok'].values())
        rows.append(row)
    if not rows:
        return 0
    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    _switch_known(prov.get('sw'), prov.get('env'))
    # One write per run: peer sessions append to the same file.
    with open(LEDGER, 'a', encoding='utf-8') as f:
        f.write(''.join(json.dumps(r, separators=(',', ':')) + '\n' for r in rows))
    marker.write_text(stamp)
    return len(rows)


def rows(pattern=None, since=None):
    if not LEDGER.exists():
        return []
    rx = re.compile(pattern) if pattern else None
    found = []
    for line in LEDGER.read_text(encoding='utf-8').splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if rx and not rx.search(r.get('case') or ''):
            continue
        if since and (r.get('t') or '') < since:
            continue
        found.append(r)
    return found


def _key(r, by):
    if by == 'exe':
        return r.get('exe_sha') or '?'
    if by == 'sw':
        return r.get('sw') or '?'
    return '%s%s' % (r.get('commit') or '?', '+' + r['dirty'] if r.get('dirty') else '')


def summary(pattern, since, by):
    groups = defaultdict(list)
    for r in rows(pattern, since):
        groups[(r['case'], _key(r, by) if by else '')].append(r)
    print('%-62s %-22s %5s %6s  %s' % ('case', by or '', 'runs', 'pass', 'failing checks (count)'))
    for (case, key), rs in sorted(groups.items()):
        bad = defaultdict(int)
        for r in rs:
            if r.get('status') not in (None, 'passed'):
                bad['asserts'] += 1
            for k, ok in (r.get('ok') or {}).items():
                if not ok:
                    bad[k] += 1
        npass = sum(1 for r in rs if r['pass'])
        print('%-62s %-22s %5d %5.0f%%  %s' % (case[:62], key[:22], len(rs), 100.0 * npass / len(rs),
                                              ', '.join('%s %d' % kv for kv in sorted(bad.items()))))


def history(pattern, metric, last):
    rs = rows(pattern)[-last:]
    for r in rs:
        m = r.get('m') or {}
        shown = {metric: m.get(metric)} if metric else m
        print('%s %-10s %-11s %-4s %-44s %s %s' % (
            (r.get('t') or '')[:16], r.get('commit') or '?', ('+' + r['dirty']) if r.get('dirty') else '',
            'PASS' if r['pass'] else 'fail', r['case'][:44], r.get('status') or '-',
            ' '.join('%s=%s' % kv for kv in shown.items())))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('summary')
    s.add_argument('pattern', nargs='?')
    s.add_argument('--since')
    s.add_argument('--by', choices=['commit', 'exe', 'sw'])
    h = sub.add_parser('history')
    h.add_argument('pattern')
    h.add_argument('--metric')
    h.add_argument('--last', type=int, default=40)
    w = sub.add_parser('switches')
    w.add_argument('hash')
    a = sub.add_parser('add')
    a.add_argument('dirs', nargs='+')
    b = sub.add_parser('backfill')
    b.add_argument('--no-metrics', action='store_true')
    args = ap.parse_args()
    if args.cmd == 'summary':
        summary(args.pattern, args.since, args.by)
    elif args.cmd == 'history':
        history(args.pattern, args.metric, args.last)
    elif args.cmd == 'switches':
        for line in SWITCHES.read_text(encoding='utf-8').splitlines() if SWITCHES.exists() else []:
            r = json.loads(line)
            if r['sw'].startswith(args.hash):
                print('\n'.join(r['env']))
    elif args.cmd == 'add':
        for d in args.dirs:
            print('%s: %d rows' % (d, record(Path(d))))
    elif args.cmd == 'backfill':
        dirs = sorted(list(ROOT.glob('runtime-SENT-*')) + list(ROOT.glob('runtime-ONE-*')),
                      key=lambda p: p.stat().st_mtime)
        for d in dirs:
            if d.is_dir() and (d / 'result-summary.json').exists():
                stage = d.name.rsplit('-', 2)[-2] if d.name.endswith('-again') else d.name.rsplit('-', 1)[-1]
                n = record(d, stage=stage if d.name.startswith('runtime-SENT-') else None,
                           metrics=not args.no_metrics)
                print('%s: %d rows' % (d.name, n), flush=True)


if __name__ == '__main__':
    main()
