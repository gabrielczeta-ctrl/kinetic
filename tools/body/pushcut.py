"""Quick push A/B on the standing rows 2000-4000 only (a few minutes):
python tools/body/pushcut.py REPS NAME=K=V[;K=V] NAME2=- ...
Reports falls, steps, slide single/double, hover (s, max stretch between
standing and falling), recovery."""
import statistics, sys
from collections import defaultdict
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import sentinel, push_report
kind = 'walk' if '--walk' in sys.argv else 'stand'
pats = ['^PLAY%-01 [uw][na][ar][^ ]* push ' + kind + '[^ ]* [a-z]+ [234]000$']
reps = int(sys.argv[1])
variants = [(a.partition('=')[0], [] if a.partition('=')[2] in ('', '-') else a.partition('=')[2].split(';'))
            for a in sys.argv[2:] if not a.startswith('--')]
for tag, envs in variants:
    rows = defaultdict(list)
    for rep in range(1, reps + 1):
        out = sentinel.ROOT / ('runtime-PUSHCUT-%s%s%d' % (kind, tag, rep))
        if '--report' not in sys.argv:
            sentinel.run_stage(out, 'test_play01_walking', pats, envs)
        for test, row in push_report.summarise(str(out)).items():
            rows[int(test.split()[-1])].append(row)
    for strength in sorted(rows):
        r = rows[strength]
        rec = [x['recover'] for x in r if x['recover'] is not None and not x['fell']]
        print('%-8s %4d fell %d/%d steps %.1f single %3.0f double %3.0f hover max %.2f mean %.2f recov %s' % (
            tag, strength, sum(x['fell'] for x in r), len(r), statistics.mean(x['steps'] for x in r),
            statistics.mean(x['slide_single'] for x in r), statistics.mean(x['slide_double'] for x in r),
            max(x['hover'] for x in r), statistics.mean(x['hover'] for x in r),
            ('%.2f' % statistics.mean(rec)) if rec else '-'), flush=True)
