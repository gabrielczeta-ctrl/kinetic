"""Count the ragdoll switches the code reads: the KINETIC-2 migration's gauge.

Usage: python tools/body/switch_census.py [--files] [--names]

Every distinct OPENMW_RAGDOLL_* / OPENMW_KINETIC_* name quoted in the engine
and ragdollcore sources is one switch read by the code. Each is classed:
  registered - in the KINETIC-2 registry (components/ragdollcore/src/switchregistry.cpp)
  play       - in the play table (playdefaults.cpp), still the old way
  old        - read with only a code fallback
The old integration's switches go with their area (PLAN.md "KINETIC-2");
the old + play count falls to the registry's ~50 as the phases land.
--files adds the count per file, --names lists the names by class.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
NAME = re.compile(r'"((?:OPENMW_RAGDOLL|OPENMW_KINETIC)_[A-Z0-9_]+)"')
SOURCES = [ROOT / 'apps/openmw', ROOT / 'components/ragdollcore/src', ROOT / 'components/ragdollcore/include']
TABLES = {'switchregistry.cpp', 'playdefaults.cpp'}


def names_in(path):
    return set(NAME.findall(path.read_text(encoding='utf-8', errors='replace')))


def main(argv):
    registered = names_in(ROOT / 'components/ragdollcore/src/switchregistry.cpp')
    play = names_in(ROOT / 'components/ragdollcore/src/playdefaults.cpp')
    read = {}
    for base in SOURCES:
        for path in sorted(base.rglob('*')):
            if path.suffix not in ('.cpp', '.hpp', '.inl', '.h') or path.name in TABLES:
                continue
            found = names_in(path)
            if found:
                read[path.relative_to(ROOT).as_posix()] = found
    every = set().union(*read.values()) if read else set()
    classes = {
        'registered': sorted(every & registered),
        'play': sorted((every & play) - registered),
        'old': sorted(every - registered - play),
    }
    print(f'switches read: {len(every)}  registered {len(classes["registered"])}  '
          f'play {len(classes["play"])}  old {len(classes["old"])}')
    unread = sorted(registered - every)
    if unread:
        print(f'registered but not read yet: {len(unread)} ({", ".join(unread)})')
    if '--files' in argv:
        for name, found in sorted(read.items(), key=lambda item: -len(item[1])):
            print(f'  {len(found):5d}  {name}')
    if '--names' in argv:
        for label, names in classes.items():
            print(f'{label}:')
            for name in names:
                print(f'  {name}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
