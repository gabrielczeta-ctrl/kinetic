"""Run the PLAY-05 circuit wiring check on a private Windows desktop."""

from __future__ import annotations

import argparse
import ctypes as C
from ctypes import wintypes as W
import os
from pathlib import Path
import re
import subprocess
import sys


SOURCE = Path(__file__).resolve().parents[2]
WORKSPACE = SOURCE.parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--circuits", type=int, choices=range(1, 5), default=2)
    parser.add_argument("--seconds", type=int, choices=range(1, 11), default=3)
    parser.add_argument("--render-fps", type=int, choices=(30, 60, 120), default=60)
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args()

    user = C.WinDLL("user32", use_last_error=True)
    user.CreateDesktopW.argtypes = [W.LPCWSTR, W.LPCWSTR, C.c_void_p, W.DWORD, W.DWORD, C.c_void_p]
    user.CreateDesktopW.restype = W.HANDLE
    user.CloseDesktop.argtypes = [W.HANDLE]

    # ragdoll-dev.ps1 owns the shared live-game mutex; this wrapper only keeps
    # its entire process tree on a private desktop.
    desktop_name = "PLAY05Diagnostic" + str(os.getpid())
    desktop = user.CreateDesktopW(desktop_name, None, None, 0, 0x01FF, None)
    if not desktop:
        raise C.WinError(C.get_last_error())

    command = [
        "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
        str(SOURCE / "tools/launch-manual-hub.ps1"),
        "-Seconds", str(args.seconds), "-CheckCircuits", str(args.circuits),
        "-RenderFps", str(args.render_fps),
    ]
    startup = subprocess.STARTUPINFO()
    startup.lpDesktop = desktop_name
    startup.dwFlags = subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    try:
        result = subprocess.run(command, cwd=SOURCE, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=args.timeout,
                                startupinfo=startup)
    finally:
        user.CloseDesktop(desktop)
    sys.stdout.write(result.stdout)
    match = re.search(r"Manual hub prepared: (.+)", result.stdout)
    if result.returncode != 0 or not match:
        return result.returncode or 2
    runtime = Path(match.group(1).strip())
    log = runtime / "config/openmw.log"
    if not log.is_file():
        print(f"missing native log: {log}")
        return 2
    text = log.read_text(encoding="utf-8", errors="replace")
    completed = text.count("PLAY05_HUB circuit complete")
    print(f"PLAY-05 wiring diagnostic: {completed}/{args.circuits} circuits completed")
    return 0 if completed == args.circuits else 1


if __name__ == "__main__":
    raise SystemExit(main())
