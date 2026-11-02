"""Write an active-body settings variant: base file plus key=value overrides.

usage: make_cfg.py OUT.cfg key=value [key=value ...]
The base is files/data/ragdoll/active-body.cfg; the printed path is ready for
OPENMW_RAGDOLL_ACTIVE_BODY.
"""
import sys
from pathlib import Path

base = Path(__file__).resolve().parents[2] / 'files/data/ragdoll/active-body.cfg'
out = Path(sys.argv[1])
overrides = dict(arg.split('=', 1) for arg in sys.argv[2:])
lines = []
for line in base.read_text(encoding='utf-8').splitlines():
    key = line.split('=', 1)[0].strip()
    if '=' in line and not line.lstrip().startswith('#') and key in overrides:
        line = f'{key} = {overrides.pop(key)}'
    lines.append(line)
if overrides:
    sys.exit(f'unknown keys: {", ".join(overrides)}')
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(out.resolve())
