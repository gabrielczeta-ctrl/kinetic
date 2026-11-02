"""Check diagnostic line colors in a 640x480 native side capture, outside the HUD.

This checks visible instrumentation only, never the quality of the movement.
"""
import argparse
import json
from pathlib import Path
from PIL import Image

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('capture', type=Path)
args = parser.parse_args()
im = Image.open(args.capture).convert('RGB')
if im.size != (640, 480):
    raise SystemExit('This fixture requires the native 640x480 diagnostic capture')
counts = dict(cyan=0, purple=0, physical_green=0, target_yellow=0)
for r, g, b in im.crop((0, 70, 375, 395)).getdata():
    counts['cyan'] += r < 65 and g > 150 and b > 190
    counts['purple'] += r > 140 and 35 < g < 140 and b > 190
    counts['physical_green'] += r < 100 and g > 140 and b < 100
    counts['target_yellow'] += r > 190 and g > 190 and b < 80
result = dict(purpose='ghost color visibility only', capture=str(args.capture), pixels=counts,
              all_colors_visible=all(n >= 8 for n in counts.values()))
print(json.dumps(result, indent=2))
raise SystemExit(not result['all_colors_visible'])
