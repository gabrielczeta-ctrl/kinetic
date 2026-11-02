"""Guard the user's desktop during hidden native runs.

    watch_windows.py <seconds> [--stop-file PATH]

Polls every 0.5 s for a visible top-level window owned by any openmw*.exe
process on the user's (input) desktop. If one appears, the process is killed
and the watcher exits with code 3. A run on the private desktop never has a
window here. Prints one summary line at exit.

Never touches a game that was already running when the watcher started (the
user's own session: 10-08 it killed the user's game when a test was started
while they played). `--check` only reports whether a visible game window is
on the desktop now (exit 4 if so, else 0).
"""
import ctypes as C
import ctypes.wintypes as W
import os
import subprocess
import sys
import time

user = C.windll.user32
EnumProc = C.WINFUNCTYPE(W.BOOL, W.HWND, W.LPARAM)


def openmw_pids():
    out = subprocess.run(['tasklist', '/FO', 'CSV', '/NH'], capture_output=True, text=True).stdout
    pids = {}
    for line in out.splitlines():
        cells = line.strip('"').split('","')
        if len(cells) > 1 and cells[0].lower().startswith('openmw'):
            pids[int(cells[1])] = cells[0]
    return pids


def visible_pids(running):
    hits = []

    def visit(hwnd, _):
        pid = W.DWORD()
        user.GetWindowThreadProcessId(hwnd, C.byref(pid))
        if pid.value in running and user.IsWindowVisible(hwnd):
            hits.append(pid.value)
        return True
    user.EnumWindows(EnumProc(visit), 0)
    return sorted(set(hits))


def main():
    if sys.argv[1] == '--check':
        hits = visible_pids(openmw_pids())
        if hits:
            print('a game window is open on the user desktop', hits, flush=True)
            sys.exit(4)
        sys.exit(0)
    seconds = float(sys.argv[1])
    # The user's own sessions, running before the test: never killed.
    protected = set(openmw_pids())
    stop = sys.argv[sys.argv.index('--stop-file') + 1] if '--stop-file' in sys.argv else None
    deadline, most = time.time() + seconds, 0
    while time.time() < deadline and not (stop and os.path.exists(stop)):
        running = {pid: name for pid, name in openmw_pids().items() if pid not in protected}
        most = max(most, len(running))
        hits = visible_pids(running)
        if hits:
            for pid in set(hits):
                subprocess.run(['taskkill', '/F', '/PID', str(pid)], capture_output=True)
            print('VISIBLE openmw window on the user desktop; killed', sorted(set(hits)), flush=True)
            sys.exit(3)
        time.sleep(0.5)
    print('watch done: max openmw processes', most, 'visible hits 0', flush=True)


if __name__ == '__main__':
    main()
