# Get-up: peak hand height above spine2 (actual vs target), peak spine2 angular speed, time from getting_up to standing, fell again?
import gzip, json, glob, sys, numpy as np
P = lambda b, n, k='position': np.array(b[n][k])
for run in sys.argv[1:]:
    for test in ('unarmed look knockdown', 'warhammer look knockdown'):
        rows = []
        for p in sorted(glob.glob(run + '/shard-*/*.player-diagnostics.jsonl.gz')):
            for l in gzip.open(p, 'rt'):
                r = json.loads(l)
                if r.get('kind') == 'frame' and r.get('test', '').endswith(test) and 'bodies' in r.get('state', {}):
                    rows.append(r)
        if not rows: continue
        modes = [r['state']['controller']['mode'] for r in rows]
        if 'getting_up' not in modes: print(run, test, 'no get-up'); continue
        i0 = modes.index('getting_up'); i1 = next((i for i in range(i0, len(modes)) if modes[i] == 'standing'), len(modes) - 1)
        seg = rows[i0:i1 + 30]
        hz = [max(P(r['state']['bodies'], h)[2] - P(r['state']['bodies'], 'spine2')[2] for h in ('left_hand', 'right_hand')) for r in seg]
        av = [np.linalg.norm(r['state']['bodies']['spine2']['angularVelocity']) for r in seg]
        print('%-18s %-24s get-up %.2f s | hand above chest peak %5.1f | spine2 angvel peak %5.1f | falls after %d' % (
            run[-18:], test, rows[i1]['elapsed'] - rows[i0]['elapsed'], max(hz), max(av), modes[i1:].count('falling')))
