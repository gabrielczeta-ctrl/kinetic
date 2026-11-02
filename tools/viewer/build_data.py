"""Behaviour viewer data: the game's own recordings, case by case.

usage: build_data.py RUN_DIR [RUN_DIR ...] [--out DIR] [--label RUN_DIR=TEXT]
       build_data.py --latest N        (the N newest runtime-* test runs)

Reads what a test run already wrote - the per-frame player diagnostics
(shard-*/*.player-diagnostics.jsonl.gz: every body's position, the centre of
mass, foot contacts, the controller), the shard's openmw.log (RAGDOLL_*
events, placed on the timeline through the diagnostic chunk records, which
match the jsonl lines one to one) - and the verdicts the tools judge by
(sentinel.py statuses and walk checks, behaviour_report.py metrics, the
stairs and drop tables). Writes DIR/catalog.json, DIR/cases/*.json and
DIR/floor.json (the test floor's fixtures). Nothing is simulated: the viewer
shows the recorded run.

Film clips (film.py, OPENMW_RAGDOLL_TEST_FILM runs) are linked when a run
has films/<case>.mp4, or a showcase-*/ folder in the dev root has one.
"""
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
TREE = HERE.parents[1]
ROOT = TREE.parent
sys.path.insert(0, str(TREE / 'tools/body'))
import behaviour_report  # noqa: E402
import sentinel  # noqa: E402

BODIES = ['pelvis', 'spine', 'spine1', 'spine2', 'neck', 'head',
          'left_clavicle', 'left_upper_arm', 'left_forearm', 'left_hand',
          'right_clavicle', 'right_upper_arm', 'right_forearm', 'right_hand',
          'left_thigh', 'left_calf', 'left_foot', 'right_thigh', 'right_calf', 'right_foot']
# Log lines too frequent or too internal for the timeline.
QUIET = {'RAGDOLL_DIAGNOSTIC_CHUNK', 'RAGDOLL_REPLAY_FRAME', 'RAGDOLL_REPLAY_BONE', 'RAGDOLL_REPLAY_RIG',
         'RAGDOLL_FOOTFALL', 'RAGDOLL_FOOTFALL_CONTACT', 'RAGDOLL_ARM', 'RAGDOLL_FOOTFALL_PATH',
         'RAGDOLL_ROLL_STEP', 'RAGDOLL_TILT_HOLD', 'RAGDOLL_TUCK', 'RAGDOLL_TUCK_ENTRY', 'RAGDOLL_LUNGE',
         'RAGDOLL_KINEMATIC', 'RAGDOLL_SITUATION', 'RAGDOLL_FOOTFALL_INPUT', 'RAGDOLL_DAMAGE_TRACE',
         'RAGDOLL_FOOTFALL_KNEE_AIR', 'RAGDOLL_FOOTFALL_LIFT'}
EVENT_CAP = 40  # per tag per case
TIME = re.compile(r'^\[(\d+):(\d+):(\d+)\.(\d+)')
FAMILIES = [
    ('Getting hit', r'STAGGER-01|push |knockdown|hit '),
    ('Landing', r'LEDGE-01|JUMP-01|jump|STAIRJUMP-01 jump'),
    ('Drop roll', r'DROP-01'),
    ('Stairs', r'stairs|STAIRS-01|STAIRJUMP-01'),
    ('Slopes', r'SLOPE-01|downhill|balmora'),
    ('Edges', r'TEETER-01'),
    ('Catches', r'WALL-01'),
    ('Mantle and crawl', r'MANTLE-01|CRAWL-01'),
    ('Combat', r'FIX-08|FIST-01|DUEL|duel|DMG-04|SCALE-0'),
    ('Walking', r'course|walk|run|strafe|turn|idle|stand'),
]


def family(name):
    for label, pattern in FAMILIES:
        if re.search(pattern, name):
            return label
    return 'Other'


def slug(name):
    return re.sub(r'[^A-Za-z0-9._-]+', '-', name).strip('-')


def log_seconds(line):
    m = TIME.match(line)
    if not m:
        return None
    h, mi, s, ms = m.groups()
    return int(h) * 3600 + int(mi) * 60 + int(s) + int(ms) / 1000.0


def shard_log(shard):
    for path in sorted(shard.glob('*/openmw.log')):
        return open(path, encoding='utf-8', errors='replace')
    for path in sorted(shard.glob('*.log.gz')):
        return gzip.open(path, 'rt', encoding='utf-8', errors='replace')
    return None


def read_shard(shard):
    """(records, record log times, events by case): the jsonl records in
    order, the log time of each, and the RAGDOLL_* lines per TEST_START case."""
    records = []
    for path in sorted(shard.glob('*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                try:
                    records.append(json.loads(line))
                except ValueError:
                    records.append(None)
    times = [None] * len(records)
    events = defaultdict(list)
    stream = shard_log(shard)
    if stream is None:
        return records, times, events
    case, chunk = None, 0
    with stream:
        for line in stream:
            if '\tTEST_START\t' in line:
                case = line.rstrip('\r\n').split('\t')[-1].strip()
                continue
            if 'RAGDOLL_DIAGNOSTIC_CHUNK\t' in line:
                parts = line.split('RAGDOLL_DIAGNOSTIC_CHUNK\t', 1)[1].split('\t')
                if len(parts) >= 3 and parts[1] == '1':
                    if chunk < len(times):
                        times[chunk] = log_seconds(line)
                    chunk += 1
                continue
            i = line.find('RAGDOLL_')
            if i < 0 or case is None:
                continue
            body = line[i:].rstrip('\r\n')
            tag = body.split(' ', 1)[0].split('\t', 1)[0]
            if tag in QUIET or tag.startswith('RAGDOLL_REPLAY'):
                continue
            actor = re.search(r'actor="?([^"\s]+)"?', body)
            if actor and actor.group(1) != 'player':
                continue
            t = log_seconds(line)
            if t is not None:
                events[case].append((t, tag, body[:240]))
    return records, times, events


def r1(v):
    return round(float(v), 1)


def case_data(name, frames, events):
    """frames: [(record, log time)] of one case, in order."""
    t0 = frames[0][0].get('simulation_time') or 0.0
    out = {'name': name, 'family': family(name), 't': [], 'com': [], 'bodies': {b: [] for b in BODIES},
           'mode': [], 'up': [], 'speed': [], 'feet': [], 'contacts': [], 'segments': [], 'yaw': []}
    present = set()
    log_at = []
    last_segment = None
    for record, logt in frames:
        s = record['state']
        t = round((record.get('simulation_time') or 0.0) - t0, 4)
        out['t'].append(t)
        log_at.append((logt, t))
        com = s.get('centerOfMass') or [0, 0, 0]
        out['com'].append([r1(v) for v in com[:3]])
        bodies = s.get('bodies') or {}
        for b in BODIES:
            p = (bodies.get(b) or {}).get('position')
            if isinstance(p, list) and len(p) >= 3:
                present.add(b)
                out['bodies'][b].append([r1(v) for v in p[:3]])
            else:
                out['bodies'][b].append(None)
        c = s.get('controller') or {}
        out['mode'].append(str(c.get('mode', '')))
        out['up'].append(round(float(c.get('torsoUpDot', 1.0) or 0.0), 3))
        v = c.get('centerOfMassVelocity') or [0, 0, 0]
        out['speed'].append([r1(x) for x in v[:3]])
        sup = s.get('support') or {}
        out['feet'].append([1 if sup.get('leftFoot') else 0, 1 if sup.get('rightFoot') else 0])
        cts = []
        for ct in (s.get('contacts') or {}).values():
            p = ct.get('position')
            if isinstance(p, list) and len(p) >= 3:
                cts.append([r1(p[0]), r1(p[1]), r1(p[2])])
        out['contacts'].append(cts[:12])
        out['yaw'].append(round(float(record.get('gameplay_yaw') or 0.0), 3))
        seg = record.get('segment')
        if seg != last_segment:
            out['segments'].append([t, seg])
            last_segment = seg
    out['bodies'] = {b: v for b, v in out['bodies'].items() if b in present}
    # Events: the log time onto the nearest earlier frame.
    timeline = [(lt, t) for lt, t in log_at if lt is not None]
    counts = defaultdict(int)
    out['events'] = []
    for lt, tag, text in events:
        counts[tag] += 1
        if counts[tag] > EVENT_CAP or not timeline:
            continue
        if lt < timeline[0][0] - 1.0 or lt > timeline[-1][0] + 1.0:
            continue
        t = timeline[0][1]
        for flt, ft in timeline:
            if flt <= lt:
                t = ft
            else:
                break
        out['events'].append([t, tag.replace('RAGDOLL_', ''), text])
    out['duration'] = out['t'][-1] if out['t'] else 0.0
    return out


def verdicts(run):
    out = defaultdict(dict)
    try:
        for name, status in sentinel.statuses(run).items():
            out[name]['status'] = status
    except Exception as e:  # noqa: BLE001 - a run without a summary still shows
        print('  statuses:', e)
    try:
        for name, checks in sentinel.walk_checks(run).items():
            out[name]['checks'] = [[c[0], str(c[1]), bool(c[2])] for c in checks]
    except Exception as e:  # noqa: BLE001
        print('  walk checks:', e)
    try:
        for name, row in behaviour_report.summarise(run, ('PLAY-01',)).items():
            out[name]['behaviour'] = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}
    except Exception as e:  # noqa: BLE001
        print('  behaviour:', e)
    try:
        frames = behaviour_report.stair_frames(run)
        logs = behaviour_report.stair_log(run)
        for name in set(frames) | set(logs):
            row = dict(frames.get(name, {}))
            row.update({k: v for k, v in dict(logs.get(name, {})).items()})
            out[name]['stairs'] = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}
    except Exception as e:  # noqa: BLE001
        print('  stairs:', e)
    return out


def find_film(run, name):
    """(path under the dev root, note): this run's film.py clip, else a
    showcase-*/ clip of the same case (another run - the note says so)."""
    clip = re.sub(r'[^A-Za-z0-9]+', '-', name).strip('-') + '.mp4'  # film.py's names
    own = run / 'films' / clip
    if own.exists():
        return own.relative_to(ROOT).as_posix(), 'this run'
    for c in sorted(ROOT.glob('showcase-*/' + clip), reverse=True):
        return c.relative_to(ROOT).as_posix(), 'another run (%s)' % c.parent.name
    return None, None


def floor():
    """The test floor's fixture triangles (the second drawable of the mesh)."""
    mesh = TREE / 'scripts/data/ragdoll_manual/meshes/ragdoll_test_floor.osgt'
    text = mesh.read_text(errors='replace').replace('\r\n', '\n')
    m = re.search(r'UniqueID 22\s*\n\s*Binding BIND_PER_VERTEX\s*\n\s*vector \d+ \{\n(.*?)\n\s*\}\n', text, re.S)
    if not m:
        return []
    verts = [[float(v) for v in l.split()] for l in m.group(1).split('\n') if l.strip()]
    return [verts[i] + verts[i + 1] + verts[i + 2] for i in range(0, len(verts) - 2, 3)]


def build(runs, out, labels):
    (out / 'cases').mkdir(parents=True, exist_ok=True)
    catalog_path = out / 'catalog.json'
    catalog = json.loads(catalog_path.read_text()) if catalog_path.exists() else {'runs': []}
    for run in runs:
        run = Path(run).resolve()
        if not list(run.glob('shard-*/*.player-diagnostics.jsonl.gz')):
            print('skip (no diagnostics):', run)
            continue
        print('run', run.name)
        judged = verdicts(run)
        by_case = defaultdict(list)
        events = defaultdict(list)
        fixture = {}
        for shard in sorted(run.glob('shard-*')):
            records, times, shard_events = read_shard(shard)
            for name, ev in shard_events.items():
                events[name] += ev
            for record, lt in zip(records, times):
                if not record:
                    continue
                if record.get('kind') == 'anchor' and record.get('test'):
                    fixture[record['test']] = record.get('fixture')
                if record.get('kind') == 'frame' and isinstance(record.get('state'), dict) and record.get('test'):
                    by_case[record['test']].append((record, lt))
        cases = []
        for name, frames in sorted(by_case.items()):
            if len(frames) < 5:
                continue
            frames.sort(key=lambda f: f[0].get('simulation_time') or 0.0)
            data = case_data(name, frames, sorted(events.get(name, [])))
            data['run'] = run.name
            data['fixture'] = fixture.get(name)
            data['verdict'] = judged.get(name, {})
            data['film'], data['film_note'] = find_film(run, name)
            file = slug(run.name) + '__' + slug(name) + '.json'
            (out / 'cases' / file).write_text(json.dumps(data, separators=(',', ':')))
            status = data['verdict'].get('status')
            fell = (data['verdict'].get('behaviour') or {}).get('fell')
            cases.append({'name': name, 'file': file, 'family': data['family'], 'duration': data['duration'],
                          'status': status, 'fell': fell, 'film': data['film'] is not None,
                          'bodies': len(data['bodies'])})
        entry = {'id': run.name, 'label': labels.get(str(run), run.name), 'cases': cases,
                 'when': run.stat().st_mtime, 'order': len(catalog['runs'])}
        catalog['runs'] = [r for r in catalog['runs'] if r['id'] != run.name]
        if cases:
            catalog['runs'].insert(0, entry)  # the newest build is the default
        print('  %d cases' % len(cases))
    catalog_path.write_text(json.dumps(catalog, indent=1))
    (out / 'floor.json').write_text(json.dumps(floor(), separators=(',', ':')))
    print('wrote', out)


def main():
    args = sys.argv[1:]
    out = ROOT / 'viewer-data'
    labels = {}
    if '--out' in args:
        i = args.index('--out')
        out = Path(args[i + 1])
        del args[i:i + 2]
    while '--label' in args:
        i = args.index('--label')
        run, text = args[i + 1].split('=', 1)
        labels[str(Path(run).resolve())] = text
        del args[i:i + 2]
    runs = list(args)
    if '--latest' in runs:
        i = runs.index('--latest')
        n = int(runs[i + 1])
        del runs[i:i + 2]
        found = [p for p in ROOT.glob('runtime-*') if p.is_dir() and list(p.glob('shard-*/*.player-diagnostics.jsonl.gz'))]
        runs += [str(p) for p in sorted(found, key=lambda p: -p.stat().st_mtime)[:n]]
    if not runs:
        sys.exit(__doc__)
    build(runs, out, labels)


if __name__ == '__main__':
    main()
