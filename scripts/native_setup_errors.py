"""Recognize engine script failures that invalidate a native test environment."""
import re


def setup_error(line):
    # Require an engine error record: quoted diagnostic text in a test result
    # must not accidentally be classified as an engine failure.
    if not re.search(r'\[[^\]\r\n]* E\]', line):
        return None
    if 'Lua error:' in line or "Can't start " in line:
        return line.strip()
    return None
