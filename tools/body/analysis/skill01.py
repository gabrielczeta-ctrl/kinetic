"""SKILL-01: per attacker skill, the share of rolled blows that bit against the mean logged chance.

usage: python tools/body/analysis/skill01.py RUN_NAME [RUN_NAME ...]   (runtime-ONE-RUN_NAME)
Vanilla parity (PLAN.md Alpha 1.0): bite rate within 5 points of the chance.
"""
import glob
import gzip
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = str(Path(__file__).resolve().parents[4])  # the workspace that holds the checkout
rolls = defaultdict(list)  # skill -> [(chance, bites)]
for name in sys.argv[1:]:
    for path in glob.glob(str(Path(ROOT, 'runtime-ONE-%s' % name, 'shard-*', '*.log.gz'))):
        attacker_skill = {}
        for line in gzip.open(path, 'rt', encoding='utf-8', errors='replace'):
            m = re.search(r'SKILL-01 attacker=(\S+) skill=(\d+)', line)
            if m:
                attacker_skill[m.group(1).strip('"')] = int(m.group(2))
                continue
            m = re.search(r'RAGDOLL_HIT_ROLL attacker=(\S+) victim=\S+ chance=([-0-9.]+) bites=(\d)', line)
            if m and m.group(1).strip('"') in attacker_skill:
                rolls[attacker_skill[m.group(1).strip('"')]].append((float(m.group(2)), int(m.group(3))))
for skill in sorted(rolls):
    r = rolls[skill]
    chance = sum(c for c, _ in r) / len(r)
    bit = 100.0 * sum(b for _, b in r) / len(r)
    print('skill %2d  blows %3d  mean chance %5.1f  bit %5.1f  diff %+5.1f  %s' % (
        skill, len(r), chance, bit, bit - chance, 'ok' if abs(bit - chance) <= 5 else 'OFF'))
