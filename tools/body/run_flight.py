"""Running flights in test runs (play 10-30: no foot pinned 33% of running,
53 flights of 0.25 s+ in 10 minutes): from a test output's player
diagnostics, frames of the given segments.

usage: python tools/body/run_flight.py RUN_DIR [RUN_DIR ...] [--segment 'c run']
Prints per run: air share (no foot supported), flights >= 0.25 s per minute,
median flight frames, step time CV (touchdown to touchdown, per foot), and
the pelvis height above the actor (median, p90).
"""
import glob
import gzip
import json
import statistics
import sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
segments = {'c run', 'c run again'}
if '--segment' in sys.argv:
    segments = {sys.argv[sys.argv.index('--segment') + 1]}
    args = [a for a in args if a not in segments]
for run in args:
    air = frames = longs = 0
    flight_lengths, gaps, heights = [], [], []
    for path in sorted(glob.glob(run + '/shard-*/*.player-diagnostics.jsonl.gz')):
        by = {}
        for line in gzip.open(path, 'rt', errors='replace'):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get('kind') == 'frame' and r.get('segment') in segments and 'support' in r.get('state', {}):
                by.setdefault((r['test'], r['segment']), []).append(r)
        for rows in by.values():
            run_len, last, touch = 0, None, {0: [], 1: []}
            for i, r in enumerate(rows):
                s = r['state']['support']
                feet = (bool(s.get('leftFoot')), bool(s.get('rightFoot')))
                frames += 1
                if not any(feet):
                    air += 1
                    run_len += 1
                else:
                    if run_len:
                        flight_lengths.append(run_len)
                        if run_len >= 15:
                            longs += 1
                    run_len = 0
                if last:
                    for side in (0, 1):
                        if feet[side] and not last[side]:
                            touch[side].append(i)
                last = feet
                pel = r['state'].get('bodies', {}).get('pelvis')
                if pel:
                    heights.append(pel['position'][2] - r['player_position'][2])
            for side in (0, 1):
                gaps += [(b - a) / 60.0 for a, b in zip(touch[side], touch[side][1:]) if 9 < b - a < 90]
    minutes = frames / 3600.0
    cv = statistics.pstdev(gaps) / statistics.mean(gaps) if len(gaps) > 4 else float('nan')
    heights.sort()
    print('%s frames=%d air=%.0f%% long_flights/min=%.1f median_flight=%s step_cv=%.2f pelvis_h=%.1f/%.1f' % (
        run, frames, 100.0 * air / max(frames, 1), longs / minutes if minutes else 0,
        statistics.median(flight_lengths) if flight_lengths else '-', cv,
        heights[len(heights) // 2] if heights else 0, heights[int(len(heights) * 0.9)] if heights else 0))
