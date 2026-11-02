"""A test queue that waits for the user's game (test review 10-30: "tests
wait for your game to close; a queue starts them automatically when you
quit, so the gaps get used").

usage:
  python tools/body/test_queue.py add 'COMMAND' ['COMMAND' ...]   queue commands (run from source-footfall)
  python tools/body/test_queue.py run [--wait-minutes N]           run them, one at a time
  python tools/body/test_queue.py list                             show what is queued and done

`run` takes the oldest command, waits while a game window is open on the
user's desktop (tools/play04/watch_windows.py --check, the runner's own
rule: a game started from any folder counts, the playthrough included),
then runs it and logs its exit code and last lines; a game opened while a
command runs makes the runner refuse or stop that command, which is then
put back at the front. It ends when the queue is empty (or after
--wait-minutes of waiting with nothing run, default 600). Queue and log:
runtime-test-queue.txt and runtime-test-queue.log in the dev root.
"""
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
TREE = HERE.parent.parent
ROOT = TREE.parent
QUEUE = ROOT / 'runtime-test-queue.txt'
LOG = ROOT / 'runtime-test-queue.log'


def game_open():
    check = subprocess.run([sys.executable, str(TREE / 'tools/play04/watch_windows.py'), '--check'],
                           capture_output=True, text=True)
    return check.returncode == 4


def read():
    return [l for l in QUEUE.read_text().splitlines() if l.strip()] if QUEUE.exists() else []


def write(items):
    QUEUE.write_text(''.join(i + '\n' for i in items))


def log(text):
    with open(LOG, 'a', encoding='utf-8') as f:
        f.write('[%s] %s\n' % (datetime.now().strftime('%m-%d %H:%M:%S'), text))
    print(text, flush=True)


def run(wait_minutes):
    waited_since = time.time()
    while True:
        items = read()
        if not items:
            log('queue empty')
            return
        if game_open():
            if time.time() - waited_since > wait_minutes * 60:
                log('gave up waiting for the game to close (%d queued)' % len(items))
                return
            time.sleep(20)
            continue
        command = items[0]
        write(items[1:])
        log('START ' + command)
        # Split like bash, run without a shell: through cmd (shell=True on
        # Windows) the usage's single-quoted Lua patterns kept their quotes,
        # lost their ^ and split at spaces, and every filter matched the
        # wrong cases or none (10-31).
        done = subprocess.run(shlex.split(command), cwd=str(TREE), capture_output=True, text=True)
        tail = (done.stdout + done.stderr).strip().splitlines()[-6:]
        # cases.py and sentinel.py report the runner's refusal (exit 4, a
        # game window open) only as "runner failed (exit 4)": a run refused
        # that way was logged as done and dropped (10-31).
        refused = any('REFUSED' in l or 'game window is open' in l or 'runner failed (exit 4)' in l for l in tail)
        if refused:
            write([command] + read())
            log('REFUSED (the game opened) - put back: ' + command)
            continue
        log('END exit=%d %s\n    %s' % (done.returncode, command, '\n    '.join(tail)))
        waited_since = time.time()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] == 'add':
        write(read() + sys.argv[2:])
        print('%d queued' % len(read()))
    elif sys.argv[1] == 'list':
        for item in read():
            print('queued:', item)
        if LOG.exists():
            print(''.join(LOG.read_text().splitlines(True)[-20:]))
    elif sys.argv[1] == 'run':
        wait = float(sys.argv[sys.argv.index('--wait-minutes') + 1]) if '--wait-minutes' in sys.argv else 600.0
        run(wait)
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
