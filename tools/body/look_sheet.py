"""Tile the LOOK case stills into one contact sheet per case.

usage: look_sheet.py RUN_DIR [--crop x0,y0,x1,y1] [--cols N]

Pairs the screenshot requests in the shard diagnostics (kind=screenshot, in
order) with the shard's screenshots/*.png (sequential names). A run with
parallel 1 has one shard. Writes RUN_DIR/sheets/<case>.png.
"""
import gzip
import json
import re
import sys
from pathlib import Path

from PIL import Image, ImageDraw

args = sys.argv[1:]
crop, cols = None, 8
if '--crop' in args:
    i = args.index('--crop')
    crop = tuple(int(v) for v in args[i + 1].split(','))
    del args[i:i + 2]
if '--cols' in args:
    i = args.index('--cols')
    cols = int(args[i + 1])
    del args[i:i + 2]
run = Path(args[0])
out = run / 'sheets'
out.mkdir(exist_ok=True)
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
    cases = {}
    for request, image in zip(requests, images):
        cases.setdefault(request['test'], []).append((request, image))
    for test, shots in cases.items():
        tiles = []
        for request, image in shots:
            im = Image.open(image).convert('RGB')
            w, h = im.size
            box = crop or (int(w * 0.30), int(h * 0.12), int(w * 0.70), int(h * 0.95))
            im = im.crop(box)
            im.thumbnail((260, 400))
            ImageDraw.Draw(im).text((4, 4), f"{request['segment']} {request.get('view') or ''} {request['at']:.2f}", fill=(255, 255, 0))
            tiles.append(im)
        tw, th = tiles[0].size
        rows = (len(tiles) + cols - 1) // cols
        sheet = Image.new('RGB', (tw * cols, th * rows))
        for i, tile in enumerate(tiles):
            sheet.paste(tile, ((i % cols) * tw, (i // cols) * th))
        name = re.sub(r'[^A-Za-z0-9]+', '-', test).strip('-')
        sheet.save(out / f'{name}.png')
        print(out / f'{name}.png', len(tiles))
