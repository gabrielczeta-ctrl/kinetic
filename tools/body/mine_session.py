"""Find the worst stretches of a play recording for the weapon arm, to replay
as tests (2026-10-05: the loose cut end, re-seats and stuck wind-ups showed in
the user's play and not in scripted tests; bookmarks catch only some).

Scores each 25 s window ending at a step (the length replay_cases.py replays)
by the arm's bad steps under hand control:
- held: steps more than 90 deg off for longer than 0.25 s (wind-up and cut);
- end: steps more than 90 deg off while held at a cut's end;
- reseats: safety-net re-seats started;
and prints the worst windows that do not overlap, per weapon, as arguments
for replay_cases.py.

usage: python tools/body/mine_session.py RECORDING_DIR [--top N]
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

WINDOW = 1500  # steps (25 s at 60 Hz), as replay_cases.py replays


def rows(directory):
    directory = Path(directory)
    if not (directory / 'ticks-0.jsonl').exists():
        directory = directory / 'manual-recording'
    for chunk in sorted(directory.glob('ticks-*.jsonl'), key=lambda p: int(p.stem.split('-')[1])):
        with open(chunk, encoding='utf-8') as f:
            for line in f:
                if '"hsc"' not in line:
                    continue
                row = json.loads(line)
                if row.get('hsc'):
                    yield row


def bad_steps(directory):
    """(tick, weapon, kind) for every bad step."""
    run, last_reseats = 0, None
    for row in rows(directory):
        h, tick, weapon = row['hsc'], row['tick'], row.get('weapon', '')
        off = h.get('wrist_err', 0) > 90
        if h.get('stage') in (1, 2) and off:
            run += 1
            if run > 15:
                yield tick, weapon, 'held'
        else:
            run = 0
        if h.get('stage') == 2 and h.get('u', 0) >= 1.99 and off:
            yield tick, weapon, 'end'
        reseats = h.get('reseats', 0)
        if last_reseats is not None and reseats > last_reseats:
            yield tick, weapon, 'reseats'
        last_reseats = reseats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording')
    parser.add_argument('--top', type=int, default=2, help='windows per weapon')
    args = parser.parse_args()
    by_weapon = defaultdict(list)
    for tick, weapon, kind in bad_steps(args.recording):
        by_weapon[weapon].append((tick, kind))
    picked = []
    for weapon, steps in sorted(by_weapon.items()):
        # score = held + end + 30 per re-seat, over the window ending at each bad step
        ticks = [t for t, _ in steps]
        weight = [30 if k == 'reseats' else 1 for _, k in steps]
        windows, lo, total = [], 0, 0
        for hi, end in enumerate(ticks):
            total += weight[hi]
            while ticks[lo] <= end - WINDOW:
                total -= weight[lo]
                lo += 1
            windows.append((total, end))
        chosen = []
        for score, end in sorted(windows, reverse=True):
            if len(chosen) >= args.top:
                break
            if all(abs(end - other) >= WINDOW for _, other in chosen):
                chosen.append((score, end))
        for score, end in chosen:
            counts = defaultdict(int)
            for t, k in steps:
                if end - WINDOW < t <= end:
                    counts[k] += 1
            print('%-18s window ending t%d: score %d (held %d, end %d, re-seats %d)'
                  % (weapon, end, score, counts['held'], counts['end'], counts['reseats']))
            picked.append(end)
    print('replay_cases.py argument: %s:%s' % (args.recording, ','.join('t%d' % t for t in sorted(picked))))


if __name__ == '__main__':
    main()
