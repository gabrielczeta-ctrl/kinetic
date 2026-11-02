"""Join film-mode stills into one clip per case (showcase, 10-11).

usage: film.py RUN_DIR [--fps 15] [--label TEXT]

Run the cases with OPENMW_RAGDOLL_TEST_FILM=90 (side camera yaw offset,
degrees): every segment is filmed at 15 stills per simulated second. Pairs
the screenshot requests in the shard diagnostics (kind=screenshot, in order)
with the shard's screenshots/*.png, like look_sheet.py, stamps the case name
and time on each frame and encodes RUN_DIR/films/<case>.mp4 with ffmpeg.
"""
import gzip
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

args = sys.argv[1:]
fps, label = 15, ''
if '--fps' in args:
    i = args.index('--fps')
    fps = int(args[i + 1])
    del args[i:i + 2]
if '--label' in args:
    i = args.index('--label')
    label = args[i + 1]
    del args[i:i + 2]
run = Path(args[0])
out = run / 'films'
out.mkdir(exist_ok=True)
ffmpeg = shutil.which('ffmpeg')
if not ffmpeg:
    sys.exit('ffmpeg not found')
BAR_BITS, BAR_PITCH, BAR_SIZE, BAR_MARGIN = 24, 10, 8, 6


def barcode_time(path):
    """The simulation time a still shows, from the film barcode (player.lua
    in test_play01_walking: top right, a white and a black reference square,
    then 24 bits of centiseconds), or None."""
    im = Image.open(path).convert('L')
    x0 = im.width - BAR_MARGIN - (BAR_BITS + 2) * BAR_PITCH
    y = BAR_MARGIN + BAR_SIZE // 2

    def level(k):
        x = x0 + k * BAR_PITCH + BAR_SIZE // 2
        return sum(im.getpixel((x + dx, y + dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1)) / 9.0

    white, black = level(0), level(1)
    if white - black < 120:
        return None
    mid = (white + black) / 2
    value = 0
    for k in range(BAR_BITS):
        value = value * 2 + (1 if level(k + 2) > mid else 0)
    return value / 100.0


def pair_by_barcode(images, requests):
    """{case: [(request, image)]} from the barcodes: each still is placed at
    the simulation time it shows, in the case whose film requests span it."""
    ranges = {}
    for r in requests:
        if r.get('view') != 'film' or 'simulation_time' not in r:
            continue
        lo, hi, start, seg = ranges.get(r['test'], (r['simulation_time'], r['simulation_time'],
                                                   r['simulation_time'] - r['at'], []))
        ranges[r['test']] = (min(lo, r['simulation_time']), max(hi, r['simulation_time']), start, seg)
        seg.append((r['simulation_time'], r.get('segment')))
    cases, decoded = {}, 0
    for image in images:
        t = barcode_time(image)
        if t is None:
            continue
        decoded += 1
        for name, (lo, hi, start, seg) in ranges.items():
            if lo - 0.05 <= t <= hi + 0.1:
                segment = min(seg, key=lambda s: abs(s[0] - t))[1]
                cases.setdefault(name, []).append(({'at': t - start, 'segment': segment, 'test': name}, image))
                break
    for shots in cases.values():
        shots.sort(key=lambda s: s[0]['at'])
    return cases, decoded


def pair_by_case(images, timed):
    """{case: [(request, image)]}: each still with the film request it shows.
    The requests' real clock loses time on every case start (the teleport
    and cell load stall the frame), so one offset from the first pair
    drifted ~0.2 s a case - 2.4 s by the twelfth (DROP-01 walk off 500's
    clip showed the roll 1-2 s before the recording had it). Each case is
    aligned on its own teleport instead: the biggest jump between
    consecutive stills near the case's expected start is its first request;
    within a case the clocks run together."""
    mtimes = [p.stat().st_mtime for p in images]
    small = [list(Image.open(p).convert('L').resize((40, 30)).getdata()) for p in images]
    jump = [0.0] + [sum(abs(a - b) for a, b in zip(small[k - 1], small[k])) / 1200.0 for k in range(1, len(images))]
    order, by_case = [], {}
    for r in timed:
        if r['test'] not in by_case:
            order.append(r['test'])
            by_case[r['test']] = []
        by_case[r['test']].append(r)
    offset = mtimes[0] - timed[0]['real']
    cases = {}
    for n, name in enumerate(order):
        rs = by_case[name]
        expected = rs[0]['real'] + offset
        near = [k for k in range(1, len(images)) if abs(mtimes[k] - expected) <= 1.5]
        if n and near:
            offset = mtimes[max(near, key=lambda k: jump[k])] - rs[0]['real']
        start, end = rs[0]['real'] + offset - 0.02, rs[-1]['real'] + offset + 0.2
        for k, m in enumerate(mtimes):
            if start <= m <= end:
                t = m - offset
                r = min(rs, key=lambda q: abs(q['real'] - t))
                if r.get('view') == 'film':
                    cases.setdefault(name, []).append((r, images[k]))
    return cases


for shard in sorted({p.parent for p in run.rglob('*.player-diagnostics.jsonl.gz')}):
    requests = []
    for path in sorted(shard.glob('*.player-diagnostics.jsonl.gz')):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                if '"screenshot"' not in line:
                    continue
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get('kind') == 'screenshot':
                    requests.append(r)
    if not requests:
        continue
    # By number: screenshot1002.png sorted as text came before
    # screenshot119.png, and past the 1000th still every clip got frames
    # of other moments (the viewer's films ran ~1 s off, 10-13).
    images = sorted(shard.rglob('screenshots/*.png'),
                    key=lambda p: (int(re.sub(r'\D', '', p.stem) or 0), p.name))
    if len(images) != len(requests):
        print(f'{shard.name}: {len(requests)} requests, {len(images)} images; pairing in order')
    # Screenshots can be dropped (a request while the last one is still
    # being written): each image goes to the request it shows (pair_by_case).
    cases = {}
    timed = [r for r in requests if 'real' in r]
    by_barcode, decoded = pair_by_barcode(images, requests) if images else ({}, 0)
    if decoded >= 0.8 * len(images) and by_barcode:
        print(f'{shard.name}: {decoded}/{len(images)} stills timed by their barcode')
        cases = by_barcode
    elif timed and len(timed) == len(requests) and images:
        cases = pair_by_case(images, timed)
    else:
        for request, image in zip(requests, images):
            if request.get('view') == 'film':
                cases.setdefault(request['test'], []).append((request, image))
    for test, shots in cases.items():
        name = re.sub(r'[^A-Za-z0-9]+', '-', test).strip('-')
        with tempfile.TemporaryDirectory() as tmp:
            # Hold each frame until the next one's time (dropped stills
            # would otherwise speed the clip up).
            frames = []
            for k, (request, image) in enumerate(shots):
                nxt = shots[k + 1][0]['at'] if k + 1 < len(shots) else request['at'] + 1.0 / fps
                frames += [(request, image)] * max(1, round((nxt - request['at']) * fps))
            for n, (request, image) in enumerate(frames):
                im = Image.open(image).convert('RGB')
                draw = ImageDraw.Draw(im)
                text = f"{label + '  ' if label else ''}{test}   t={request['at']:.2f}s   {request.get('segment') or ''}"
                draw.rectangle((0, 0, im.width, 18), fill=(0, 0, 0))
                draw.text((6, 3), text, fill=(255, 230, 120))
                if im.width % 2 or im.height % 2:
                    im = im.crop((0, 0, im.width - im.width % 2, im.height - im.height % 2))
                im.save(Path(tmp) / f'{n:05d}.png')
            target = out / f'{name}.mp4'
            subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(fps),
                            '-i', str(Path(tmp) / '%05d.png'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '23', str(target)], check=True)
        print(target, len(shots), 'frames')
