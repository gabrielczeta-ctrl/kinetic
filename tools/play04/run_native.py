"""Run a focused native suite on a private desktop, respecting the live lease.

Uses the existing integration runner and native input path. Results are scoped
suite diagnostics, not a replacement for PLAY-04's full qualification producer.
"""
import argparse
import ctypes as C
from ctypes import wintypes as W
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

SOURCE = Path(__file__).resolve().parents[2]
WORKSPACE = SOURCE.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--engine', type=Path, required=True)
parser.add_argument('--engine-source', required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--render-fps', type=int, choices=[30, 60, 120], default=60)
parser.add_argument('--test-filter', default='^PLAY04')
parser.add_argument('--suite', default='test_play01_walking')
parser.add_argument('--task-id', default='PLAY-04')
parser.add_argument('--hub-player', action='store_true',
                    help='use the real manual-hub player HUD without starting its global circuit')
parser.add_argument('--manual-recording', action='store_true', help='record every player physics tick and raw mouse event')
parser.add_argument('--env', action='append', default=[])
parser.add_argument('--suite-timeout', type=int, default=240, help='seconds before the suite is abandoned')
parser.add_argument('--parallel', type=int, default=1, help='isolated shard processes (integration_tests --parallel)')
parser.add_argument('--physics-threads', type=int, choices=range(65), default=None,
                    help='async physics threads; 0 = single-threaded Jolt, for run-to-run identical A/B')
parser.add_argument('--diagnostic-budget-mib', type=int, choices=range(1, 2049), default=32,
                    help='uncompressed diagnostic budget for this focused shard (default: 32 MiB)')
parser.add_argument('--keep-raw-openmw-log', action='store_true',
                    help='retain the uncompressed log in addition to the compressed archive')
parser.add_argument('--watch', action='store_true',
                    help='run on the visible desktop so the user can watch; one shard, real time')
args = parser.parse_args()
if args.watch:
    # Only ever started by the user (Watch-Tests.cmd). Never used by agents.
    args.parallel = 1
    args.env = [e for e in args.env if not e.startswith('OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER=')]
    os.environ['PLAY04_PRIVATE_DESKTOP'] = 'visible'
    os.environ['PLAY04_RUNNER_LOG'] = str(args.output.resolve().with_name(args.output.name + '.runner.log'))


def run_on_private_desktop():
    """Re-run this script on a fresh desktop and return its exit code.

    subprocess.STARTUPINFO has no lpDesktop field: the attribute set in
    quiet_popen below was silently ignored and every "private" run opened the
    game window on the user's desktop. CreateProcessW takes the desktop, and
    every process started from there (shards, the engine) inherits it.
    """
    class STARTUPINFOW(C.Structure):
        _fields_ = [('cb', W.DWORD), ('lpReserved', W.LPWSTR), ('lpDesktop', W.LPWSTR), ('lpTitle', W.LPWSTR),
                    ('dwX', W.DWORD), ('dwY', W.DWORD), ('dwXSize', W.DWORD), ('dwYSize', W.DWORD),
                    ('dwXCountChars', W.DWORD), ('dwYCountChars', W.DWORD), ('dwFillAttribute', W.DWORD),
                    ('dwFlags', W.DWORD), ('wShowWindow', W.WORD), ('cbReserved2', W.WORD),
                    ('lpReserved2', C.c_void_p), ('hStdInput', W.HANDLE), ('hStdOutput', W.HANDLE),
                    ('hStdError', W.HANDLE)]

    class PROCESS_INFORMATION(C.Structure):
        _fields_ = [('hProcess', W.HANDLE), ('hThread', W.HANDLE), ('dwProcessId', W.DWORD), ('dwThreadId', W.DWORD)]

    kernel32, user32 = C.WinDLL('kernel32', use_last_error=True), C.WinDLL('user32', use_last_error=True)
    user32.CreateDesktopW.restype = W.HANDLE
    user32.CreateDesktopW.argtypes = [W.LPCWSTR, W.LPCWSTR, C.c_void_p, W.DWORD, W.DWORD, C.c_void_p]
    kernel32.CreateProcessW.argtypes = [W.LPCWSTR, W.LPWSTR, C.c_void_p, C.c_void_p, W.BOOL, W.DWORD,
                                        C.c_void_p, W.LPCWSTR, C.POINTER(STARTUPINFOW),
                                        C.POINTER(PROCESS_INFORMATION)]
    kernel32.WaitForSingleObject.argtypes = [W.HANDLE, W.DWORD]
    kernel32.GetExitCodeProcess.argtypes = [W.HANDLE, C.POINTER(W.DWORD)]
    kernel32.CloseHandle.argtypes = [W.HANDLE]
    name = 'PLAY04Hidden' + str(os.getpid())
    desktop = user32.CreateDesktopW(name, None, None, 0, 0x01FF, None)
    if not desktop:
        raise C.WinError(C.get_last_error())
    log = args.output.resolve().with_name(args.output.name + '.runner.log')
    environment = dict(os.environ, PLAY04_PRIVATE_DESKTOP=name, PLAY04_RUNNER_LOG=str(log))
    block = ''.join(f'{k}={v}\0' for k, v in environment.items()) + '\0'
    startup = STARTUPINFOW()
    startup.cb = C.sizeof(STARTUPINFOW)
    startup.lpDesktop = 'WinSta0\\' + name
    info = PROCESS_INFORMATION()
    command = subprocess.list2cmdline([sys.executable, os.path.abspath(__file__)] + sys.argv[1:])
    if not kernel32.CreateProcessW(None, command, None, None, False, 0x08000400,  # no window, unicode env
                                   C.create_unicode_buffer(block), os.getcwd(), C.byref(startup), C.byref(info)):
        user32.CloseDesktop(desktop)
        raise C.WinError(C.get_last_error())
    kernel32.WaitForSingleObject(info.hProcess, 0xFFFFFFFF)
    code = W.DWORD()
    kernel32.GetExitCodeProcess(info.hProcess, C.byref(code))
    kernel32.CloseHandle(info.hThread)
    kernel32.CloseHandle(info.hProcess)
    user32.CloseDesktop(desktop)
    if log.exists():
        sys.stdout.write(log.read_text(encoding='utf-8', errors='replace'))
    return code.value


if not os.environ.get('PLAY04_PRIVATE_DESKTOP'):
    sys.exit(run_on_private_desktop())
# On the private desktop: all output goes to the log the parent prints.
if not args.watch:
    sys.stdout = sys.stderr = open(os.environ['PLAY04_RUNNER_LOG'], 'w', encoding='utf-8', buffering=1)
engine, output = args.engine.resolve(), args.output.resolve()
if output.exists():
    raise RuntimeError('Use a fresh output directory to preserve earlier evidence')
output.mkdir(parents=True)
if args.manual_recording:
    os.environ['OPENMW_MANUAL_DIAGNOSTICS'] = str(output / 'manual-recording')
else:
    os.environ.pop('OPENMW_MANUAL_DIAGNOSTICS', None)
dependencies = WORKSPACE / 'upstream-clean/deps/vcpkg-x64-windows-2022-2026-02-24/installed/x64-windows/bin'
os.environ['PATH'] = os.pathsep.join([str(engine.parent),
    str(engine.parent.parent / '_deps/yaml-cpp-build/RelWithDebInfo'),
    str(dependencies / 'Release'), str(dependencies), os.environ['PATH']])
os.environ['OPENMW_PHYSICS_FPS'] = '60'
os.environ['OPENMW_DISABLE_ERROR_DIALOG'] = '1'
os.environ.pop('OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER', None)
os.environ['GIT_CONFIG_COUNT'] = '1'
os.environ['GIT_CONFIG_KEY_0'] = 'safe.directory'
os.environ['GIT_CONFIG_VALUE_0'] = SOURCE.as_posix()
for entry in args.env:
    key, value = entry.split('=', 1)
    if not key.startswith(('OPENMW_RAGDOLL_', 'OPENMW_JOLT_')):
        raise ValueError('Only explicit ragdoll diagnostic overrides are allowed')
    os.environ[key] = value

kernel, user = C.WinDLL('kernel32', use_last_error=True), C.WinDLL('user32', use_last_error=True)
kernel.CreateMutexW.argtypes = [C.c_void_p, W.BOOL, W.LPCWSTR]
kernel.CreateMutexW.restype = W.HANDLE
kernel.WaitForSingleObject.argtypes = [W.HANDLE, W.DWORD]
kernel.ReleaseMutex.argtypes = [W.HANDLE]
kernel.CloseHandle.argtypes = [W.HANDLE]
user.CreateDesktopW.argtypes = [W.LPCWSTR, W.LPCWSTR, C.c_void_p, W.DWORD, W.DWORD, C.c_void_p]
user.CreateDesktopW.restype = W.HANDLE
user.CloseDesktop.argtypes = [W.HANDLE]
mutex = kernel.CreateMutexW(None, False, 'Local\\OpenMW-ragdoll-live')
if kernel.WaitForSingleObject(mutex, 0) not in (0, 0x80):
    kernel.CloseHandle(mutex)
    raise RuntimeError('Another live session owns the engine lease')
original_popen = subprocess.Popen

def quiet_popen(*pargs, **kwargs):
    # The desktop itself comes from run_on_private_desktop; children inherit it.
    startup = subprocess.STARTUPINFO()
    startup.dwFlags = subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    kwargs['startupinfo'] = startup
    return original_popen(*pargs, **kwargs)

try:
    # Watch mode must show the window: SW_HIDE here kept it invisible.
    if not args.watch:
        subprocess.Popen = quiet_popen
    suite = SOURCE / 'scripts/data/integration_tests' / args.suite
    manual_data = SOURCE / 'scripts/data/ragdoll_manual'
    if args.hub_player:
        manual_data = output / 'hub-player-data'
        shutil.copytree(SOURCE / 'scripts/data/ragdoll_manual', manual_data)
        shutil.copyfile(manual_data / 'ragdoll_manual_player.lua', manual_data / 'manual_hub_player_base.lua')
        shutil.copytree(SOURCE / 'scripts/data/ragdoll_hub', manual_data, dirs_exist_ok=True)
        # Fixture-only event exercises the real H handler on the private
        # desktop. It is never copied by the user's manual launcher.
        player_script = manual_data / 'ragdoll_manual_player.lua'
        player_script.write_text('local result = (function()\n' + player_script.read_text()
            + '\nend)()\nresult.eventHandlers.rxnHudTestCycle = function()\n'
            + "    result.engineHandlers.onKeyPress({ code = require('openmw.input').KEY.H })\n"
            + 'end\nresult.eventHandlers.manualObservationTestKey = function(name)\n'
            + "    result.engineHandlers.onKeyPress({ code = require('openmw.input').KEY[name] })\n"
            + 'end\nreturn result\n', encoding='utf-8')
    (output / 'diagnostic-manifest.json').write_text(json.dumps({
        'task': args.task_id, 'scope': 'focused native mechanism diagnostic; no manual acceptance',
        'engine_source': args.engine_source, 'engine': str(engine),
        'engine_sha256': hashlib.sha256(engine.read_bytes()).hexdigest(),
        'engine_resources_sha256': {'shaders/compatibility/objects.frag': hashlib.sha256(
            (engine.parent / 'resources/shaders/compatibility/objects.frag').read_bytes()).hexdigest()},
        'script_source_commit': subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip(),
        'script_sha256': {str(p.relative_to(SOURCE)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in suite.iterdir() if p.is_file()},
        'render_fps': args.render_fps, 'physics_fps': 60,
        'diagnostic_budget_mib': args.diagnostic_budget_mib,
        'raw_openmw_log_retained': args.keep_raw_openmw_log,
        'suite': args.suite, 'test_filter': args.test_filter,
        'hub_player': args.hub_player,
        'manual_recording': args.manual_recording,
        'manual_data_sha256': {str(p.relative_to(manual_data)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in manual_data.rglob('*') if p.is_file()},
        'environment': {k: v for k, v in os.environ.items() if k.startswith('OPENMW_RAGDOLL_')},
    }, indent=2))
    data = Path('C:/Program Files (x86)/Steam/steamapps/common/Morrowind/Data Files')
    sys.path.insert(0, str(SOURCE / 'scripts'))
    sys.argv = [str(SOURCE / 'scripts/integration_tests.py'), str(suite), '--omw', str(engine),
        '--workdir', str(output), '--data', str(SOURCE / 'files/data'),
        '--data', str(manual_data), '--data', str(data),
        '--ini', str(data.parent / 'Morrowind.ini'), '--render_fps', str(args.render_fps),
        '--test_filter', args.test_filter, '--suite_timeout', str(args.suite_timeout),
        '--diagnostic_budget_mib', str(args.diagnostic_budget_mib)]
    if not args.keep_raw_openmw_log:
        sys.argv.append('--discard_raw_openmw_log')
    if args.parallel > 1:
        sys.argv += ['--parallel', str(args.parallel)]
    if args.physics_threads is not None:
        sys.argv += ['--physics_threads', str(args.physics_threads)]
    runpy.run_path(sys.argv[0], run_name='__main__')
finally:
    subprocess.Popen = original_popen
    kernel.ReleaseMutex(mutex)
    kernel.CloseHandle(mutex)
