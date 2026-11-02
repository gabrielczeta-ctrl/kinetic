"""Head height from the LOOK profile stills, against the vanilla animation.

usage: look_height.py RUN_DIR

The profile camera sits at a fixed distance and height from the player, so
the top of the character's silhouette (the first image row, from the top,
whose pixels differ from the plain sky) measures how tall the character
stands. Per case: the median over the steady profile stills, as a share of
the image height, and the difference from the vanilla loadout's same case
converted to world units (230-unit camera distance, ~65 deg vertical field).
"""
import gzip
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def top_row(path):
    im = np.asarray(Image.open(path).convert('L'), dtype=np.int16)
    h, w = im.shape
    band = im[: int(h * 0.55), int(w * 0.25): int(w * 0.75)]
    sky = np.median(band[:5])
    rows = np.flatnonzero((np.abs(band - sky) > 25).sum(axis=1) > 3)
    return (rows[0] / h if len(rows) else float('nan')), h


def main(run):
    run = Path(run)
    tops = {}
    for shard in sorted({p.parent for p in run.rglob('*.player-diagnostics.jsonl.gz')}):
        requests = []
        for path in sorted(shard.glob('*.player-diagnostics.jsonl.gz')):
            with gzip.open(path, 'rt', encoding='utf-8') as stream:
                for line in stream:
                    if '"screenshot"' in line:
                        r = json.loads(line)
                        if r.get('kind') == 'screenshot':
                            requests.append(r)
        images = sorted(shard.rglob('screenshots/*.png'))
        for request, image in zip(requests, images):
            if request['segment'] == 'steady' and not request.get('view'):
                value, height = top_row(image)
                tops.setdefault(request['test'], []).append(value)
    med = {k: float(np.nanmedian(v)) for k, v in tops.items()}
    # image share -> world units at the camera distance
    units_per_share = 2 * 230 * math.tan(math.radians(65 / 2))
    print('%-32s %8s %10s' % ('case', 'top', 'vs vanilla'))
    for name in sorted(med):
        if ' vanilla ' in name:
            continue
        key = re.sub(r'^PLAY-01 \w+ ', '', name)
        van = med.get('PLAY-01 vanilla ' + key)
        diff = (van - med[name]) * units_per_share if van is not None else float('nan')
        print('%-32s %8.3f %+9.1f u' % (name[8:], med[name], diff))


if __name__ == '__main__':
    main(sys.argv[1])
