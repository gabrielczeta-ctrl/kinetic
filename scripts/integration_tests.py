#!/usr/bin/env python3

import argparse
import datetime
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from collections import deque

from pathlib import Path
from test_recording import TestRecording
from diagnostic_report import summarize as summarize_player_diagnostics
from native_setup_errors import setup_error

def parse_args():
    parser = argparse.ArgumentParser(description="OpenMW integration tests.")
    parser.add_argument(
        "suites",
        nargs='*',
        type=str,
        default=None,
        help="suites to run: a directory with suites or subdirectories containing them",
    )
    parser.add_argument("--omw", type=str, default="openmw", help="path to openmw binary")
    parser.add_argument(
        "--workdir", type=str, default="integration_tests_output", help="directory for temporary files and logs"
    )
    parser.add_argument("--parallel", type=int, choices=range(1, 17), default=1)
    parser.add_argument("--timing_report", help="prior result-summary.json for deterministic cost-balanced shards")
    parser.add_argument("--case_names", help="JSON array of exact test names for one isolated shard")
    parser.add_argument("--verbose", action='store_true', help="print all openmw output")
    parser.add_argument("--video_directory", help="opt-in bounded OpenMW-window recordings; shared 512 MiB budget")
    parser.add_argument("--renderer_stats", action='store_true', help="opt-in verbose renderer timing log")
    parser.add_argument("--diagnostic_budget_mib", type=int, choices=range(1, 2049), default=128,
                        help="uncompressed player diagnostic budget per suite/shard (default: 128 MiB)")
    parser.add_argument(
        "--discard_raw_openmw_log", action='store_true',
        help="delete config/openmw.log after its compressed archive is written successfully",
    )
    parser.add_argument("--native_mouse", action='store_true', help="allow native relative mouse mode instead of --no-grab fallback")
    parser.add_argument(
        "--render_fps", type=int, choices=(30, 60, 120), default=60,
        help="render rate cap; does not change the physics step rate",
    )
    parser.add_argument(
        "--physics_threads", type=int, choices=range(65), default=None,
        help="override async physics threads for a diagnostic run; zero selects single-threaded Jolt",
    )
    parser.add_argument(
        "--test_filter", type=str, default=None,
        help="test cases to run (e.g. 'player.*running')",
    )
    parser.add_argument(
        "--list_tests", action='store_true',
        help="print test names instead of running them",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="extra directory with custom settings.cfg",
    )
    parser.add_argument(
        "--data", action='append', default=[],
        help="extra data directories with content files",
    )
    parser.add_argument(
        "--ini", type=str, default=None,
        help="INI file to import fallback values from via openmw-iniimporter (found next to --omw)",
    )
    parser.add_argument(
        "--suite_timeout", type=float, default=None,
        help="maximum seconds allowed for each OpenMW suite before its exact process is killed",
    )
    return parser.parse_args()


def lua_string_literal(value):
    escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return f'"{escaped}"'


def discover_suites(suite_roots):
    all_suites = dict()
    for root in suite_roots:
        if not root.is_dir():
            sys.exit(f"{root} is not a directory")
        if (root / "openmw.cfg").is_file():
            all_suites[root.name] = root
        else:
            for entry in sorted(root.glob("test_*")):
                if entry.is_dir() and (entry / "openmw.cfg").is_file():
                    all_suites[entry.name] = entry
    if not all_suites:
        sys.exit(f"No suites found in: {', '.join(str(v) for v in suite_roots)}")
    return all_suites


def write_test_config_module(test_config_dir, test_filter, list_tests, player_diagnostics=False, native_mouse=False, case_names=None):
    shutil.rmtree(test_config_dir, ignore_errors=True)
    test_config_dir.mkdir(parents=True)
    fields = []
    if case_names is not None:
        fields.append("names = {" + ",".join("[" + lua_string_literal(name) + "]=true" for name in case_names) + "}")
    if test_filter:
        fields.append(f"filter = {lua_string_literal(test_filter)}")
    if list_tests:
        fields.append("list = true")
    if player_diagnostics:
        fields.append('playerDiagnostics = true')
    if native_mouse:
        fields.append('nativeMouse = true')
    with open(test_config_dir / "test_config.lua", "w", encoding="utf-8") as stream:
        stream.write(f"return {{ {', '.join(fields)} }}\n")


def parse_test_name(status, line):
    return line.split(status)[1].strip().split("\t", maxsplit=1)


def remove_intro_movie_fallbacks(config_path):
    # These values come from Morrowind.ini. Their presence makes OpenMW play
    # both logo videos before the menu test script receives its first frame.
    # Remove the keys entirely: an empty fallback value is invalid.
    intro_prefixes = (
        "fallback=movies_company_logo,",
        "fallback=movies_morrowind_logo,",
    )
    with open(config_path, "r", encoding="utf-8") as stream:
        lines = stream.read().splitlines()
    lines = [line for line in lines if not line.casefold().startswith(intro_prefixes)]
    with open(config_path, "w", encoding="utf-8", newline="\n") as stream:
        stream.write("\n".join(lines) + "\n")


def write_generated_config(
    config_dir, userdata_dir, data_local_dir, test_config_dir, config_source_dir, args, iniimporter_binary, ini_path
):
    shutil.rmtree(config_dir, ignore_errors=True)
    config_dir.mkdir(parents=True)
    base_cfg_path = config_dir / ("openmw.base.cfg" if args.ini else "openmw.cfg")
    with open(base_cfg_path, "w", encoding="utf-8") as omw_cfg:
        omw_cfg.write(f'user-data="{userdata_dir}"\n')
        omw_cfg.write(f'data-local="{data_local_dir}"\n')
        omw_cfg.write(f'data="{test_config_dir}"\n')
    if args.ini:
        subprocess.run(
            [
                iniimporter_binary,
                "--ini", ini_path,
                "--cfg", base_cfg_path,
                "--output", config_dir / "openmw.cfg",
            ],
            check=True,
        )
        remove_intro_movie_fallbacks(config_dir / "openmw.cfg")
    with open(config_dir / "settings.cfg", "w", encoding="utf-8") as settings_cfg:
        if config_source_dir is not None:
            source_settings = config_source_dir / "settings.cfg"
            if source_settings.is_file():
                settings_cfg.write(source_settings.read_text(encoding="utf-8"))
        settings_cfg.write(
            "[Video]\n"
            f"resolution x = {1280 if args.video_directory else 640}\n"
            f"resolution y = {720 if args.video_directory else 480}\n"
            "fullscreen = false\n"
            f"framerate limit = {args.render_fps}\n"
            "[Game]\n"
            "smooth animation transitions = true\n"
            "[Lua]\n"
            f"memory limit = {1024 * 1024 * 256}\n"
            "lua profiler = true\n"
        )
        if args.physics_threads is not None:
            settings_cfg.write(f"[Physics]\nasync num threads = {args.physics_threads}\n")


def run_test(suite_name, suite_dir, config_dir, log_dir, time_str, data_dirs, args, openmw_binary):
    start = time.time()
    if not args.list_tests:
        print(f'[----------] Running tests from {suite_name}')
    command = [openmw_binary, "--replace=config", "--config", suite_dir, "--config", config_dir]
    if not args.native_mouse:
        command.append("--no-grab")
    else:
        # Suites may set no-grab=1 in openmw.cfg; omission alone does not
        # override that configuration.
        command.append("--no-grab=0")
    for path in data_dirs:
        command += ["--data", path]
    stdout_lines = deque(maxlen=2000)
    fatal_errors = list()
    telemetry = list()
    cases, found_tests = [], []
    active_case = None
    recording = TestRecording(args.video_directory)
    environment = dict(os.environ)
    environment['OPENMW_DISABLE_ERROR_DIALOG'] = '1'
    environment.pop('OPENMW_OSG_STATS_FILE', None)
    environment.pop('OPENMW_OSG_STATS_LIST', None)
    if args.renderer_stats:
        environment['OPENMW_OSG_STATS_FILE'] = str(log_dir / f"{suite_name}.{time_str}.osg_stats.log")
        environment['OPENMW_OSG_STATS_LIST'] = 'times'
    diagnostic_path = log_dir / f'{suite_name}.{time_str}.player-diagnostics.jsonl.gz'
    diagnostic_bytes = 0
    diagnostic_capped = False
    diagnostic_parts = []
    diagnostic_record = None
    with recording, gzip.open(diagnostic_path, 'wt', encoding='utf-8') as diagnostic_stream, subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding="utf-8",
        env=environment,
    ) as process:
        timed_out = threading.Event()

        def terminate_on_timeout():
            if process.poll() is not None:
                return
            try:
                process.kill()
            except OSError:
                return
            timed_out.set()

        watchdog = None
        if args.suite_timeout is not None:
            watchdog = threading.Timer(args.suite_timeout, terminate_on_timeout)
            watchdog.daemon = True
            watchdog.start()

        quit_requested = False
        running_test_number = None
        running_test_name = None
        count = 0
        failed_tests = list()
        test_start = None
        for line in process.stdout:
            failure = setup_error(line)
            if failure and len(fatal_errors) < 20:
                fatal_errors.append(f'native script environment invalid: {failure}')
            if args.verbose:
                sys.stdout.write(line)
            elif 'RAGDOLL_DIAGNOSTIC_CHUNK\t' not in line:
                stdout_lines.append(line)
            if "Quit requested by a Lua script" in line:
                quit_requested = True
            if "RAGDOLL_DIAGNOSTIC_CHUNK\t" in line:
                try:
                    record, part, total, chunk = line.split('RAGDOLL_DIAGNOSTIC_CHUNK\t', 1)[1].rstrip('\r\n').split('\t', 3)
                    part, total = int(part), int(total)
                    if part == 1:
                        if diagnostic_parts:
                            raise ValueError('incomplete preceding diagnostic record')
                        diagnostic_record = record
                    if record != diagnostic_record or part != len(diagnostic_parts) + 1 or not 1 <= part <= total <= 64:
                        raise ValueError('out-of-order or oversized diagnostic record')
                    diagnostic_parts.append(chunk)
                    if part != total:
                        continue
                    payload = ''.join(diagnostic_parts)
                    diagnostic_parts = []
                    sample = json.loads(payload)
                    if sample.get('kind') == 'capture_stop':
                        recording.stop()
                    sample['received_monotonic'] = time.monotonic()
                    encoded = json.dumps(sample, separators=(',', ':')) + '\n'
                    if diagnostic_bytes + len(encoded) <= args.diagnostic_budget_mib * 1024 * 1024:
                        diagnostic_stream.write(encoded)
                        diagnostic_bytes += len(encoded)
                    elif not diagnostic_capped:
                        diagnostic_capped = True
                        fatal_errors.append('player diagnostic byte budget exhausted; trace is incomplete')
                        process.kill()
                except (ValueError, TypeError) as error:
                    diagnostic_parts = []
                    if len(fatal_errors) < 5:
                        fatal_errors.append(f'invalid player diagnostic JSON: {error}')
            elif "RAGDOLL_TELEMETRY_JSON\t" in line:
                payload = line.split("RAGDOLL_TELEMETRY_JSON\t", maxsplit=1)[1].strip()
                try:
                    sample = json.loads(payload)
                    if not isinstance(sample, dict):
                        raise ValueError("telemetry payload is not an object")
                    sample.setdefault("suite", suite_name)
                    sample['received_monotonic'] = time.monotonic()
                    if running_test_name is not None:
                        sample.setdefault("test", running_test_name)
                    telemetry.append(sample)
                except (json.JSONDecodeError, ValueError) as error:
                    fatal_errors.append(f"invalid ragdoll telemetry JSON: {error}")
            elif "TEST_FOUND" in line:
                _, name = parse_test_name("TEST_FOUND", line)
                count += 1
                found_tests.append(name)
                print(name)
            elif "TEST_START" in line:
                test_start = time.time()
                number, name = parse_test_name("TEST_START", line)
                running_test_number = int(number)
                running_test_name = name
                active_case = {'name': name, 'status': 'incomplete', 'duration_ms': None}
                cases.append(active_case)
                recording.start(name)
                count += 1
                print(f"[ RUN      ] {running_test_name}")
            elif "TEST_OK" in line:
                recording.stop()
                duration = (time.time() - test_start) * 1000
                number, name = parse_test_name("TEST_OK", line)
                assert running_test_number == int(number) and running_test_name == name
                active_case.update(status="passed", duration_ms=duration)
                print(f"[       OK ] {running_test_name} ({duration:.3f} ms)")
            elif "TEST_FAILED" in line:
                recording.stop()
                duration = (time.time() - test_start) * 1000
                number, name, error = line.split("TEST_FAILED")[1].strip().split("\t", maxsplit=2)
                assert running_test_number == int(number) and running_test_name == name
                active_case.update(status="failed", duration_ms=duration, error=error)
                print(error)
                print(f"[  FAILED  ] {running_test_name} ({duration:.3f} ms)")
                failed_tests.append(running_test_name)
            elif "TEST_ERROR" in line:
                message = line.split("TEST_ERROR")[1].strip()
                fatal_errors.append(f"test framework error: {message}")
        recording.stop()
        if diagnostic_parts:
            fatal_errors.append('incomplete final diagnostic record')
        process.wait(5)
        if watchdog is not None:
            watchdog.cancel()
        if timed_out.is_set():
            fatal_errors.append(f"timed out after {args.suite_timeout:g} seconds")
        elif not quit_requested:
            fatal_errors.append("unexpected termination")
        if process.returncode != 0:
            fatal_errors.append(f"openmw exited with code {process.returncode}")
    raw_openmw_log = config_dir / 'openmw.log'
    if os.path.exists(raw_openmw_log):
        with open(raw_openmw_log, 'rb') as source, gzip.open(log_dir / f'{suite_name}.{time_str}.log.gz', 'wb') as target:
            shutil.copyfileobj(source, target)
        if args.discard_raw_openmw_log:
            os.remove(raw_openmw_log)
    if diagnostic_bytes:
        print(f'[ DIAGNOSTIC] {summarize_player_diagnostics(diagnostic_path)}')
    if any(case['status'] == 'incomplete' for case in cases):
        fatal_errors.append('test terminated without a verdict')
    if len({case['name'] for case in cases}) != len(cases):
        fatal_errors.append('duplicate test case in suite')
    if telemetry:
        telemetry_dir = log_dir / "test-results"
        telemetry_dir.mkdir(parents=True, exist_ok=True)
        telemetry_path = telemetry_dir / f"{suite_name}.{time_str}.telemetry.jsonl"
        with open(telemetry_path, "w", encoding="utf-8", newline="\n") as stream:
            for sample in telemetry:
                stream.write(json.dumps(sample, sort_keys=True, separators=(",", ":")) + "\n")
        print(f"[ TELEMETRY] {telemetry_path}")
    if fatal_errors and not args.verbose:
        sys.stdout.writelines(stdout_lines)
    if not args.list_tests:
        total_duration = (time.time() - start) * 1000
        print(f'\n[----------] {count} tests from {suite_name} ({total_duration:.3f} ms total)')
        print(f"[  PASSED  ] {count - len(failed_tests)} tests.")
    if fatal_errors:
        print(f"[  FAILED  ] fatal error: {'; '.join(fatal_errors)}")
    if failed_tests:
        print(f"[  FAILED  ] {len(failed_tests)} tests, listed below:")
        for failed_test in failed_tests:
            print(f"[  FAILED  ] {failed_test}")
    return len(failed_tests) == 0 and not fatal_errors, count, cases, found_tests, fatal_errors


def main():
    args = parse_args()
    if args.parallel > 1:
        from native_smoke_parallel import run_parallel
        sys.exit(run_parallel(args))

    if args.suite_timeout is not None and args.suite_timeout <= 0:
        sys.exit("--suite_timeout must be greater than zero")

    openmw_binary = Path(args.omw).resolve()
    if not openmw_binary.is_file():
        sys.exit(f"{openmw_binary} not found")

    iniimporter_binary = None
    ini_path = None
    if args.ini:
        ini_path = Path(args.ini).resolve()
        if not ini_path.is_file():
            sys.exit(f"{ini_path} not found")
        iniimporter_binary = openmw_binary.parent / f"openmw-iniimporter{openmw_binary.suffix}"
        if not iniimporter_binary.is_file():
            sys.exit(f"{iniimporter_binary} not found")

    config_source_dir = Path(args.config).resolve() if args.config is not None else None
    data_dirs = [Path(v).resolve() for v in args.data]

    work_dir = Path(args.workdir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    case_names = None
    if args.case_names:
        case_names = json.loads(Path(args.case_names).read_text(encoding='utf-8'))
        if (not isinstance(case_names, list) or not case_names or any(not isinstance(n, str) or not n for n in case_names)
                or len(set(case_names)) != len(case_names)):
            sys.exit('case_names must be a nonempty array of unique test names')
    test_config_dir = work_dir / "test_config"
    time_str = datetime.datetime.now().strftime("%Y-%m-%d-%H.%M.%S")

    suite_roots = [Path(v).resolve() for v in args.suites]
    all_suites = discover_suites(suite_roots)

    if args.video_directory:
        # Bound the complete new evidence set, including the engine's raw
        # log, rather than only the much smaller compressed/video exports.
        evidence_bytes = 0
        for path in work_dir.parent.glob('*/run-manifest.json'):
            try:
                recorded = json.loads(path.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if recorded.get('arguments', {}).get('video_directory') == args.video_directory:
                evidence_bytes += sum(p.stat().st_size for p in path.parent.rglob('*') if p.is_file())
        if evidence_bytes + 1024**3 > 2 * 1024**3 or shutil.disk_usage(work_dir).free < 2 * 1024**3:
            args.recording_skip_reason = '2 GiB evidence budget or 1 GiB free-space reserve reached'
            print(f'[ RECORDING] disabled for this run: {args.recording_skip_reason}')
            args.video_directory = None

    write_test_config_module(test_config_dir=test_config_dir, test_filter=args.test_filter, list_tests=args.list_tests,
                             player_diagnostics=bool(args.video_directory), native_mouse=args.native_mouse, case_names=case_names)
    def file_hash(path):
        digest = hashlib.sha256()
        with open(path, 'rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        return digest.hexdigest()
    source_files = [Path(__file__), Path(__file__).with_name('test_recording.py'),
                    Path(__file__).with_name('diagnostic_report.py'), Path(__file__).with_name('native_smoke_parallel.py'),
                    Path(__file__).parent / 'data/integration_tests/testing_util/testing_util.lua']
    for suite in all_suites.values():
        source_files.extend(suite.glob('*.lua'))
    manifest = {'engine': str(openmw_binary), 'engine_sha256': file_hash(openmw_binary),
                'arguments': vars(args), 'physics_fps_environment': os.environ.get('OPENMW_PHYSICS_FPS'),
                'ragdoll_environment': {key: value for key, value in sorted(os.environ.items())
                                        if key.startswith('OPENMW_RAGDOLL_')},
                'recording_changes_render_load': bool(args.video_directory),
                'source_sha256': {str(path): file_hash(path) for path in source_files}}
    (work_dir / 'run-manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')

    status = 0
    total_count = 0
    suite_results = []
    for name in sorted(all_suites):
        suite_work_dir = work_dir / name
        write_generated_config(
            config_dir=suite_work_dir,
            userdata_dir=suite_work_dir / "userdata",
            data_local_dir=suite_work_dir / "data-local",
            test_config_dir=test_config_dir,
            config_source_dir=config_source_dir,
            args=args,
            iniimporter_binary=iniimporter_binary,
            ini_path=ini_path,
        )
        manifest.setdefault('generated_config_sha256', {})[name] = {
            filename: file_hash(suite_work_dir / filename) for filename in ('settings.cfg', 'openmw.cfg')}
        (work_dir / 'run-manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        result, count, cases, found_tests, fatal_errors = run_test(
            suite_name=name,
            suite_dir=all_suites[name],
            config_dir=suite_work_dir,
            log_dir=work_dir,
            time_str=time_str,
            data_dirs=data_dirs,
            args=args,
            openmw_binary=openmw_binary,
        )
        total_count += count
        suite_results.append({'name': name, 'passed': bool(result), 'matched_tests': count,
                              'cases': cases, 'found_tests': found_tests, 'fatal_errors': fatal_errors})
        if not result:
            status = -1
    if args.test_filter and total_count == 0:
        print(f"[  WARNING ] no tests matched --test_filter {args.test_filter!r}")
        status = -1
    result_summary = {
        'schema_version': 1,
        'generated': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'source_commit': subprocess.run(
            ['git', '-C', str(Path(__file__).resolve().parents[1]), 'rev-parse', 'HEAD'],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        'manifest': 'run-manifest.json',
        'suites': suite_results,
        'tracker': {
            'producer': 'integration_tests.py',
            'total_checks': total_count,
            'failed_checks': sum(not item['passed'] for item in suite_results) + int(bool(args.test_filter and total_count == 0)),
        },
    }
    (work_dir / 'result-summary.json').write_text(json.dumps(result_summary, indent=2), encoding='utf-8')
    exit(status)


if __name__ == "__main__":
    main()
