"""REACT-04 report (RXN-02 M6, 2026-10-07): blows on an attacking dummy.
Per part x phase (struck in the wind-up or in the cut) x strength: the
stage when struck, the controller's tier and interrupt (the RAGDOLL_RXN2_BLOW
line after REACT04_BEGIN), and whether the attack still cut afterwards.
Design: a wind-up breaks from a flinch up; a committed cut only from a
stagger.

usage: python tools/body/analysis/react04.py RUNTIME_DIR ...
"""
import glob
import re
import sys

BEGIN = re.compile(r'REACT04_BEGIN part=(\S+) phase=(\S+) strength=(\S+)')
ROW = re.compile(r'REACT04 part=(\S+) phase=(\S+) strength=(\S+) reached=(\S+) stage_at_blow=(-?\d+) '
                 r'cut_after=(\S+) max_after=(-?\d+)')
BLOW = re.compile(r'RAGDOLL_RXN2_BLOW actor=\S+ blow=\d+ region=(\S+) tier=(\S+) scale=\S+ reflex=\S+ interrupt=(\d)')
for root in sys.argv[1:]:
    print(root)
    print('  %-7s %-5s %-6s %-7s %-5s %-8s %-6s %-9s' % ('part', 'phase', 'str', 'reached', 'stage', 'tier', 'break', 'cut after'))
    for log in glob.glob(root + '/shard-*/*/openmw.log'):
        blow = None
        for line in open(log, encoding='utf-8', errors='replace'):
            if BEGIN.search(line):
                blow = None
                continue
            m = BLOW.search(line)
            if m and blow is None:
                blow = m
                continue
            r = ROW.search(line)
            if r:
                tier = blow.group(2) if blow else '-'
                interrupt = blow.group(3) if blow else '-'
                print('  %-7s %-5s %-6s %-7s %-5s %-8s %-6s %-9s' % (r.group(1), r.group(2), r.group(3), r.group(4),
                      r.group(5), tier, interrupt, r.group(6)))
