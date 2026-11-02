"""Switch parsing check (STEPPING audit 2026-11-01, item 4).

Most OPENMW_RAGDOLL_* list switches are read with a hand-written
`std::sscanf(text, "%f,%f,...", &v[0], ...)` into a fixed array, and a short
value silently keeps the defaults for the missing fields. On 11-01 an
unclaimed find-and-replace cut eight 4-field reads to 3 and nothing noticed.
This finds, per read:
  format   the format's field count differs from the arguments it fills
  array    the format reads fewer fields than its std::array holds
  value    the play launcher or a settings file sets more or fewer fields
           than the read takes
New code reads lists with RagdollCore::tuningList(), which also warns at
run time (RAGDOLL_TUNING_WARNING in the log).

usage: python tools/body/switch_check.py [--values-only|--code-only]
Exit status 1 when a format or array mismatch is found.
"""
import pathlib
import re
import sys

SOURCE = pathlib.Path(__file__).resolve().parents[2]
ROOT = SOURCE.parent
FILES = [p for d in ('apps/openmw', 'components/ragdollcore') for ext in ('*.cpp', '*.inl', '*.hpp')
         for p in (SOURCE / d).rglob(ext)]
READ = re.compile(r'tuningText\("(OPENMW_[A-Z0-9_]+)"\)')
SCANF = re.compile(r'sscanf\(\s*\w+\s*,\s*"([^"]*)"\s*,')


def call_arguments(text, start):
    """The rest of the sscanf call from `start` up to its closing parenthesis."""
    depth = 1
    for i in range(start, len(text)):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[start:i]
    return text[start:]
ARRAY = re.compile(r'std::array<\s*(?:float|int|double)\s*,\s*(\d+)\s*>\s*(\w+)')


def reads():
    """(file, line, name, format fields, argument count, array size or None)."""
    out = []
    for path in FILES:
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        for i, line in enumerate(lines):
            m = READ.search(line)
            if not m:
                continue
            if not SCANF.search('\n'.join(lines[i:i + 4])):
                continue
            window = '\n'.join(lines[i:i + 16])
            s = SCANF.search(window)
            # A long format is split over adjacent string literals.
            literals = re.match(r'(\s*"[^"]*")*', window[s.start(1) - 1:])
            fmt = ''.join(re.findall(r'"([^"]*)"', literals.group(0)))
            fields = len(re.findall(r'%l?[fdiu]', fmt))
            rest = call_arguments(window, s.start(1) - 1 + literals.end())
            args = len(re.findall(r'(?<!&)&(?!&)', rest))
            target = re.search(r'&\s*(?:\w+\.)?(\w+)\[0\]', rest)
            size = None
            if target:
                for back in lines[max(0, i - 8):i + 1][::-1]:
                    a = ARRAY.search(back)
                    if a and a.group(2) == target.group(1):
                        size = int(a.group(1))
                        break
            out.append((path.relative_to(SOURCE).as_posix(), i + 1, m.group(1), fields, args, size))
    return out


def values():
    """name -> [(where, field count)] from the play launcher and settings files."""
    found = {}
    sources = [ROOT / 'Run-Game-MotionBricks.cmd', SOURCE / 'files/data/ragdoll/ragdoll.cfg']
    for src in sources:
        if not src.exists():
            continue
        for line in src.read_text(encoding='utf-8', errors='replace').splitlines():
            m = re.match(r'\s*(?:set\s+"?)?(OPENMW_[A-Z0-9_]+)\s*=\s*([^"\r\n]*)', line)
            if m and m.group(2).strip():
                found.setdefault(m.group(1), []).append((src.name, len(m.group(2).strip().split(','))))
    return found


def main():
    problems = 0
    table = reads()
    if '--values-only' not in sys.argv:
        for path, line, name, fields, args, size in table:
            if fields != args:
                print('format  %s:%d %s reads %d fields into %d arguments' % (path, line, name, fields, args))
                problems += 1
            elif size is not None and fields < size:
                print('array   %s:%d %s reads %d fields into a %d-field array' % (path, line, name, fields, size))
                problems += 1
    if '--code-only' not in sys.argv:
        taken = {}
        for path, line, name, fields, args, size in table:
            taken.setdefault(name, set()).add(fields)
        for name, where in sorted(values().items()):
            for src, count in where:
                if name in taken and count not in taken[name] and count > max(taken[name]):
                    print('value   %s sets %s with %d fields; the code reads %s' %
                          (src, name, count, '/'.join(str(x) for x in sorted(taken[name]))))
    print('%d sscanf reads checked, %d format/array problems' % (len(table), problems))
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
