"""run_native.py with the desktop window watcher around it.

    run_watched.py <run_native.py arguments>

Starts watch_windows.py, runs the hidden-desktop suite, stops the watcher and
fails if the watcher saw (and killed) a visible game window. Measures the
machine's load meanwhile (resource_sampler.py: a "resources:" line, and
OUTPUT/resource-usage.json).
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from resource_sampler import ResourceSampler, line

HERE = Path(__file__).resolve().parent
stop = Path(tempfile.gettempdir()) / f'play04-watch-stop-{os.getpid()}'
# The user is playing (a visible game window): do not start at all.
if subprocess.call([sys.executable, str(HERE / 'watch_windows.py'), '--check']) == 4:
    print('REFUSED: the user has a game open; not starting a test run')
    sys.exit(4)
stop.unlink(missing_ok=True)
watcher = subprocess.Popen([sys.executable, str(HERE / 'watch_windows.py'), '7200', '--stop-file', str(stop)],
                           stdout=subprocess.PIPE, text=True)
output = Path(sys.argv[sys.argv.index('--output') + 1]) if '--output' in sys.argv else None
sampler = ResourceSampler().start()
try:
    code = subprocess.call([sys.executable, str(HERE / 'run_native.py')] + sys.argv[1:])
finally:
    stop.write_text('stop')
    out, _ = watcher.communicate(timeout=30)
    stop.unlink(missing_ok=True)
    usage = sampler.stop(output)
print(out.strip())
print(line(usage), flush=True)
if watcher.returncode == 3:
    print('WATCHER: a game window appeared on the user desktop; results invalid')
    sys.exit(3)
sys.exit(code)
