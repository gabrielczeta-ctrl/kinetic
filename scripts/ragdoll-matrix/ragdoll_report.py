#!/usr/bin/env python3
"""Run the reference-gait short-test battery and robustness matrix and
write one table.

Reuses the existing ragdollcore-tests executable and its assertions. Each
test runs as its own process with an exact gtest filter, a JSON result
file and a private trace directory, so the report keeps: test name, gtest
pass/fail, raw process exit status, exact command line, wall duration,
captured log and every "TAG key=value ..." metric line the test printed.

Statuses
  PASS   gtest reports the single matched test passed and the process exited 0
  FAIL   gtest reports the test failed (an expected red case stays FAIL)
  ERROR  tooling problem: missing executable, no test matched the filter,
         more than one test matched, missing/malformed result file, exit
         status inconsistent with the result, or timeout.

Exit code of this script
  0  every run PASS
  1  at least one FAIL and no ERROR (known red cases produce 1, on purpose)
  2  at least one ERROR (the table cannot be trusted as a physical result)

Metrics that a test did not print are reported as "unavailable", never 0.

Usage
  python ragdoll_report.py --exe <baseline ragdollcore-tests.exe> --out <dir>
      [--candidate-exe <exe>] [--label baseline] [--candidate-label candidate]
      [--no-battery] [--no-matrix] [--case <gtest full name>]... [--timeout 600]
      [--source-dir <git tree of the baseline>] [--candidate-source-dir <tree>]
      [--manifest <snapshot manifest json>]
"""

import argparse
import csv
import datetime as _dt
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
import time

BATTERY = [
    ("RagdollCoreReferenceGaitTest.firstStepTransfersStanceWithoutTimeout", "REFERENCE_FIRST_STEP"),
    ("RagdollCoreReferenceGaitTest.singleSupportLiftAndHoldCarriesTheLoadedStance", "REFERENCE_LIFT_HOLD"),
    ("RagdollCoreReferenceGaitTest.coordinatedTransferHandsOffToFirstStep", "POSTURE_STEP_HANDOFF"),
    ("RagdollCoreReferenceGaitTest.referencePushDuringWalkTriggersRecoveryStep", "REFERENCE_PUSH"),
    ("RagdollCoreReferenceGaitTest.coordinatedPostureTransfersWeightWithFeetPlanted", "POSTURE_TRANSFER"),
    ("RagdollCoreReferenceGaitTest.coordinatedMotionLiftsFootOverTheLoadedStance", "SHARED_MOTION_LIFT"),
]
MATRIX_PREFIX = "Cases/ReferenceMatrixTest."
MATRIX_TAG = "REFERENCE_MATRIX"

# Columns of the compact table, in order: (column, metric key candidates).
TABLE_COLUMNS = [
    ("fall_step", ["first_falling_step", "first_falling"]),
    ("survived_s", ["survived_seconds_after_request"]),
    ("transfers", ["transfers"]),
    ("first_transfer", ["first_transfer"]),
    ("mean_achieved", ["mean_achieved"]),
    ("final_half_s", ["final_half_second_achieved"]),
    ("slide_max", ["max_foot_slide"]),
    ("saturated", ["saturated"]),
    ("ref_frames", ["reference_frames"]),
    ("sat_bodies", ["saturated_bodies"]),
    ("timed_out", ["timed_out_phases", "timed_out"]),
    ("airborne", ["airborne"]),
    ("free_vcap", ["free_velocity_limited"]),
    ("solver_vcap", ["solver_velocity_limited"]),
    ("no_support", ["no_support_frames"]),
    ("steps_after_push", ["steps_after_push"]),
    ("fell_after_push", ["fell_after_push"]),
    ("repeat_identical", ["repeat_identical"]),
]
NUMERIC_DELTA_COLUMNS = ["fall_step", "survived_s", "transfers", "mean_achieved", "final_half_s", "slide_max",
                         "saturated", "timed_out", "airborne", "free_vcap", "solver_vcap", "no_support"]

TAG_LINE = re.compile(r"^([A-Z][A-Z0-9_]+) (.*)$")
KEY_VALUE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=("[^"]*"|\S+)')


def sha256_of(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_identity(tree):
    if not tree or not os.path.isdir(tree):
        return {"tree": tree, "available": False}
    try:
        head = subprocess.check_output(["git", "-C", tree, "rev-parse", "HEAD"], text=True,
                                       stderr=subprocess.DEVNULL).strip()
        branch = subprocess.check_output(["git", "-C", tree, "rev-parse", "--abbrev-ref", "HEAD"], text=True,
                                         stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.check_output(["git", "-C", tree, "status", "--porcelain"], text=True,
                                        stderr=subprocess.DEVNULL)
        changed = [line for line in dirty.splitlines() if line.strip()]
        return {"tree": tree, "available": True, "head": head, "branch": branch,
                "uncommitted_or_untracked_files": len(changed), "changed_paths": changed[:200]}
    except (subprocess.CalledProcessError, OSError) as error:
        return {"tree": tree, "available": False, "error": str(error)}


def executable_identity(path):
    if not path:
        return {"path": path, "exists": False}
    if not os.path.isfile(path):
        return {"path": os.path.abspath(path), "exists": False}
    stat = os.stat(path)
    return {"path": os.path.abspath(path), "exists": True, "sha256": sha256_of(path), "bytes": stat.st_size,
            "modified": _dt.datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds")}


def list_matrix_tests(exe, timeout):
    """Discover the matrix cases from the executable itself."""
    if not os.path.isfile(exe):
        return [], "missing_executable"
    try:
        completed = subprocess.run([exe, "--gtest_list_tests", "--gtest_filter=" + MATRIX_PREFIX + "*"],
                                   capture_output=True, text=True, timeout=timeout, errors="replace")
    except (subprocess.TimeoutExpired, OSError) as error:
        return [], "list_failed: " + str(error)
    if completed.returncode != 0:
        return [], "list_exit_%d" % completed.returncode
    names, suite = [], None
    for raw in completed.stdout.splitlines():
        line = raw.rstrip()
        if not line or line.startswith("Running main"):
            continue
        if not line.startswith(" "):
            suite = line.split("#")[0].strip()
            continue
        if suite and suite.startswith(MATRIX_PREFIX.rstrip(".")):
            names.append(suite + line.strip().split("#")[0].strip())
    return names, None


def parse_metric_lines(text):
    tags = {}
    for raw in text.splitlines():
        line = raw.strip()
        match = TAG_LINE.match(line)
        if not match:
            continue
        tag, rest = match.group(1), match.group(2)
        values = {}
        for key, value in KEY_VALUE.findall(rest):
            values[key] = value.strip('"')
        if not values:
            continue
        tags.setdefault(tag, []).append(values)
    return tags


def sanitize(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)


def run_one(exe, name, out_dir, timeout):
    """Run one gtest case in its own process. Returns a result dict."""
    safe = sanitize(name)
    log_path = os.path.join(out_dir, safe + ".log")
    json_path = os.path.join(out_dir, safe + ".json")
    trace_dir = os.path.join(out_dir, "traces", safe)
    command = [exe, "--gtest_filter=" + name, "--gtest_output=json:" + json_path]
    record = {"name": name, "command": command, "log": log_path, "result_file": json_path, "trace_dir": trace_dir,
              "status": "ERROR", "reason": None, "exit_code": None, "duration_s": None, "gtest": None,
              "failures": [], "metrics": {}, "timed_out": False}
    if not exe or not os.path.isfile(exe):
        record["reason"] = "missing_executable"
        return record
    os.makedirs(trace_dir, exist_ok=True)
    if os.path.exists(json_path):
        os.remove(json_path)
    env = dict(os.environ)
    env["OPENMW_RAGDOLL_TRACE_DIR"] = trace_dir
    start = time.perf_counter()
    try:
        with open(log_path, "wb") as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=timeout)
        record["exit_code"] = completed.returncode
    except subprocess.TimeoutExpired:
        record["timed_out"] = True
        record["reason"] = "timeout_after_%ds" % timeout
        record["duration_s"] = round(time.perf_counter() - start, 3)
        return record
    except OSError as error:
        record["reason"] = "launch_failed: " + str(error)
        record["duration_s"] = round(time.perf_counter() - start, 3)
        return record
    record["duration_s"] = round(time.perf_counter() - start, 3)
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
            record["metrics"] = parse_metric_lines(handle.read())
    except OSError:
        record["metrics"] = {}

    if not os.path.isfile(json_path):
        record["reason"] = "missing_result_file (exit %s)" % record["exit_code"]
        return record
    try:
        with open(json_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError) as error:
        record["reason"] = "malformed_result_file: " + str(error)
        return record
    try:
        total = int(data["tests"])
        suites = data["testsuites"]
    except (KeyError, TypeError, ValueError):
        record["reason"] = "malformed_result_file: missing tests/testsuites"
        return record
    if total == 0:
        record["reason"] = "no_tests_matched"
        return record
    if total != 1:
        record["reason"] = "multiple_tests_matched (%d)" % total
        return record
    case = None
    for suite in suites:
        for test in suite.get("testsuite", []):
            case = test
    if case is None:
        record["reason"] = "malformed_result_file: no testcase entry"
        return record
    full = "%s.%s" % (case.get("classname", ""), case.get("name", ""))
    record["gtest"] = {"name": full, "status": case.get("status"), "result": case.get("result"),
                       "time": case.get("time")}
    if full != name:
        record["reason"] = "filter_mismatch (ran %s)" % full
        return record
    if case.get("status") != "RUN" or case.get("result") != "COMPLETED":
        record["reason"] = "test_not_run (%s/%s)" % (case.get("status"), case.get("result"))
        return record
    failures = case.get("failures", [])
    record["failures"] = [str(f.get("failure", ""))[:400] for f in failures]
    if failures:
        if record["exit_code"] == 0:
            record["reason"] = "inconsistent_exit (failures reported but exit 0)"
            return record
        record["status"] = "FAIL"
        record["reason"] = "%d assertion failure(s)" % len(failures)
        return record
    if record["exit_code"] != 0:
        record["reason"] = "inconsistent_exit (no failure reported but exit %d)" % record["exit_code"]
        return record
    record["status"] = "PASS"
    record["reason"] = "ok"
    return record


def primary_metrics(record, tag):
    """Metric dictionary from the test's own summary line, or None."""
    lines = record["metrics"].get(tag) if record["metrics"] else None
    if not lines:
        return None
    return lines[-1]


def table_row(record, tag):
    metrics = primary_metrics(record, tag)
    row = {}
    for column, keys in TABLE_COLUMNS:
        value = "unavailable"
        if metrics:
            for key in keys:
                if key in metrics:
                    value = metrics[key]
                    break
        if column == "transfers" and metrics and "transfers" in metrics and ";" in metrics["transfers"]:
            value = str(len([s for s in metrics["transfers"].split(";") if s]))
        row[column] = value
    row["metric_tag"] = tag if metrics else "unavailable"
    return row


def run_set(label, exe, cases, out_root, timeout):
    out_dir = os.path.join(out_root, label)
    os.makedirs(out_dir, exist_ok=True)
    results = []
    start = time.perf_counter()
    for kind, name, tag in cases:
        record = run_one(exe, name, out_dir, timeout)
        record["kind"] = kind
        record["label"] = label
        record["metric_tag"] = tag
        record["table"] = table_row(record, tag)
        results.append(record)
        print("  [%s] %-5s %-75s %6.2fs %s" % (label, record["status"], name, record["duration_s"] or 0.0,
                                               record["reason"]))
    return results, round(time.perf_counter() - start, 3)


def counts(results):
    summary = {"PASS": 0, "FAIL": 0, "ERROR": 0}
    for record in results:
        summary[record["status"]] += 1
    return summary


def verdict(results):
    c = counts(results)
    if not results:
        return "ERROR (no runs)"
    if c["ERROR"]:
        return "ERROR"
    if c["FAIL"]:
        return "RED"
    return "GREEN"


def short(value, width):
    value = str(value)
    return value if len(value) <= width else value[:width - 1] + "~"


def markdown_table(rows, columns):
    header = "| " + " | ".join(columns) + " |"
    sep = "|" + "|".join(["---"] * len(columns)) + "|"
    lines = [header, sep]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(c, "")) for c in columns) + " |")
    return "\n".join(lines)


def numeric(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def compare_fixtures(sets):
    """Compare reported settings AND scripts per case; not a full provenance proof."""
    labels = list(sets)
    if len(labels) != 2:
        return {}
    candidates = {r["name"]: r for r in sets[labels[1]]}
    comparisons = {}
    for baseline in sets[labels[0]]:
        candidate = candidates.get(baseline["name"])
        if not baseline["name"].startswith(MATRIX_PREFIX):
            continue
        pairs = [(primary_metrics(baseline, tag), primary_metrics(candidate, tag) if candidate else None)
                 for tag in ("REFERENCE_MATRIX_SETTINGS", "REFERENCE_MATRIX_SPEC")]
        comparisons[baseline["name"]] = (
            "unavailable" if any(a is None or b is None for a, b in pairs) else
            "matched_reported_fields" if all(a == b for a, b in pairs) else "different")
    return comparisons


def write_reports(out_root, meta, sets, timings, args):
    all_results = [r for label in sets for r in sets[label]]
    # JSON
    payload = {"meta": meta, "timings_s": timings, "verdicts": {label: verdict(sets[label]) for label in sets},
               "counts": {label: counts(sets[label]) for label in sets}, "results": all_results,
               "fixture_comparisons": compare_fixtures(sets)}
    with open(os.path.join(out_root, "results.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1)
    # CSV (flat)
    columns = ["label", "kind", "name", "status", "reason", "exit_code", "duration_s"] + [c for c, _ in TABLE_COLUMNS] + [
        "metric_tag", "log", "result_file", "trace_dir", "command"]
    with open(os.path.join(out_root, "results.csv"), "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for record in all_results:
            row = {"label": record["label"], "kind": record["kind"], "name": record["name"],
                   "status": record["status"], "reason": record["reason"], "exit_code": record["exit_code"],
                   "duration_s": record["duration_s"], "metric_tag": record["table"]["metric_tag"],
                   "log": record["log"], "result_file": record["result_file"], "trace_dir": record["trace_dir"],
                   "command": subprocess.list2cmdline(record["command"])}
            for c, _ in TABLE_COLUMNS:
                row[c] = record["table"][c]
            writer.writerow(row)
    # Markdown summary
    lines = ["# Reference gait short-test battery and robustness matrix", "",
             "Generated %s on %s (%s)." % (meta["generated"], meta["host"], meta["python"]), "",
             "Executables:", ""]
    for label, identity in meta["executables"].items():
        lines.append("- **%s**: `%s`" % (label, identity.get("path")))
        if identity.get("exists"):
            lines.append("  sha256 `%s`, %d bytes, modified %s" % (identity["sha256"], identity["bytes"],
                                                                  identity["modified"]))
        else:
            lines.append("  MISSING")
    lines.append("")
    for label, identity in meta["sources"].items():
        if identity.get("available"):
            lines.append("- source **%s**: `%s` @ `%s` (%s), %d uncommitted/untracked file(s)" % (
                label, identity["tree"], identity["head"], identity["branch"], identity["uncommitted_or_untracked_files"]))
        else:
            lines.append("- source **%s**: not a git tree (`%s`)" % (label, identity.get("tree")))
    if meta.get("manifest"):
        m = meta["manifest"]
        lines.append("- snapshot manifest: `%s` (snapshot commit `%s`, parent `%s`, %s copied files, byte-identical: %s)" % (
            meta["manifest_path"], m.get("snapshot_commit"), m.get("parent_head"), m.get("copied_files"),
            m.get("byte_identical_after_copy")))
    lines += ["", "## Verdict", ""]
    for label in sets:
        c = counts(sets[label])
        lines.append("- **%s**: %s (%d PASS, %d FAIL, %d ERROR of %d runs; battery %.1f s, matrix %.1f s)" % (
            label, verdict(sets[label]), c["PASS"], c["FAIL"], c["ERROR"], len(sets[label]),
            timings.get(label, {}).get("battery", 0.0), timings.get(label, {}).get("matrix", 0.0)))
    lines.append("")
    lines.append("FAIL is a real assertion failure of an existing test (known red cases stay FAIL). "
                 "ERROR means the tooling could not obtain a trustworthy result. Metrics a test did not print are "
                 "`unavailable`.")
    lines.append("Matrix PASS means the harness completed and the exact repeat matched. "
                 "It does not mean physical acceptance: inspect falls, slide, saturation and velocity caps. "
                 "The combined verdict describes only the selected assertions.")
    for kind, title in (("battery", "Six-test battery (mandatory for promotion)"),
                        ("matrix", "Robustness matrix (diagnostic; held-out cases chosen by the main developer)"),
                        ("explicit", "Explicitly requested cases (--case)")):
        for label in sets:
            rows = [r for r in sets[label] if r["kind"] == kind]
            if not rows:
                continue
            lines += ["", "## %s: %s" % (title, label), ""]
            table = []
            for r in rows:
                row = {"test": r["name"].replace(MATRIX_PREFIX, "").replace("RagdollCoreReferenceGaitTest.", ""),
                       "status": r["status"], "exit": r["exit_code"], "s": r["duration_s"], "reason": short(r["reason"], 40)}
                row.update({c: short(r["table"][c], 22) for c, _ in TABLE_COLUMNS})
                table.append(row)
            cols = ["test", "status", "exit", "s"] + [c for c, _ in TABLE_COLUMNS] + ["reason"]
            lines.append(markdown_table(table, cols))
            failures = [(r["name"], r["failures"]) for r in rows if r["failures"]]
            if failures:
                lines += ["", "Assertion failures:", ""]
                for name, texts in failures:
                    for text in texts[:3]:
                        first = text.strip().splitlines()[0] if text.strip() else ""
                        lines.append("- `%s`: %s" % (name.replace(MATRIX_PREFIX, "").replace(
                            "RagdollCoreReferenceGaitTest.", ""), short(first, 160)))
    labels = list(sets)
    if len(labels) == 2:
        base, cand = labels
        lines += ["", "## Comparison: %s vs %s (requested case names matched)" % (base, cand), ""]
        by_name = {r["name"]: r for r in sets[cand]}
        rows = []
        for r in sets[base]:
            c = by_name.get(r["name"])
            row = {"test": r["name"].replace(MATRIX_PREFIX, "").replace("RagdollCoreReferenceGaitTest.", ""),
                   base: r["status"], cand: c["status"] if c else "missing",
                   "fixture": payload["fixture_comparisons"].get(r["name"], "unavailable"),
                   "changed": "yes" if (c is None or c["status"] != r["status"]) else "no"}
            for col in NUMERIC_DELTA_COLUMNS:
                a, b = numeric(r["table"][col]), numeric(c["table"][col]) if c else None
                row["d_" + col] = "unavailable" if (a is None or b is None or
                    (r["name"].startswith(MATRIX_PREFIX) and row["fixture"] != "matched_reported_fields")) else ("%+g" % (b - a))
            rows.append(row)
        lines.append(markdown_table(rows, ["test", base, cand, "changed", "fixture"] + ["d_" + c for c in NUMERIC_DELTA_COLUMNS]))
        lines.append("")
        lines.append("Deltas are candidate minus baseline; `unavailable` when either side lacks the metric. "
                     "A status change on a battery test is a mandatory finding regardless of matrix deltas.")
        lines.append("Matrix fixture comparison checks reported settings and script fields per case; "
                     "unreported profile/capacity fields still require source review. Mismatched or missing "
                     "matrix metadata suppresses its deltas and makes the comparison exit 2. Battery fixtures "
                     "have no machine-readable specification and require source review.")
    lines += ["", "## Reproduction", "", "```", meta["command_line"], "```", "",
              "Per-run logs, gtest JSON results and traces are under `%s/<label>/`." % out_root]
    with open(os.path.join(out_root, "summary.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="baseline ragdollcore-tests executable")
    parser.add_argument("--candidate-exe", help="optional candidate executable run on identical cases")
    parser.add_argument("--out", required=True, help="output directory (created)")
    parser.add_argument("--label", default="baseline")
    parser.add_argument("--candidate-label", default="candidate")
    parser.add_argument("--source-dir", help="git tree of the baseline executable, for identity")
    parser.add_argument("--candidate-source-dir", help="git tree of the candidate executable")
    parser.add_argument("--manifest", help="snapshot manifest json to embed")
    parser.add_argument("--no-battery", action="store_true")
    parser.add_argument("--no-matrix", action="store_true")
    parser.add_argument("--case", action="append", default=[],
                        help="explicit gtest full name(s) instead of the built-in battery/matrix lists")
    parser.add_argument("--timeout", type=int, default=600, help="seconds per test process")
    parser.add_argument("--tracker-task", help="record results.json for a tracker gate")
    args = parser.parse_args()

    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.label) or not re.fullmatch(r"[A-Za-z0-9_-]+", args.candidate_label):
        parser.error("labels must use letters, digits, underscores or hyphens")
    if args.candidate_exe and args.label == args.candidate_label:
        parser.error("baseline and candidate labels must differ")
    if not args.case and args.no_battery and args.no_matrix:
        parser.error("no cases selected")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    out_root = os.path.abspath(args.out)
    if os.path.isdir(out_root) and os.listdir(out_root):
        parser.error("output directory must be empty; use a fresh path to preserve evidence")
    os.makedirs(out_root, exist_ok=True)
    started = time.perf_counter()

    # Absolute, OS-native paths so that identity, discovery and launch agree
    # (a relative forward-slash .cmd path can pass isfile() yet fail to launch).
    args.exe = os.path.normpath(os.path.abspath(args.exe))
    if args.candidate_exe:
        args.candidate_exe = os.path.normpath(os.path.abspath(args.candidate_exe))
    executables = {args.label: args.exe}
    if args.candidate_exe:
        executables[args.candidate_label] = args.candidate_exe
    meta = {"generated": _dt.datetime.now().isoformat(timespec="seconds"), "host": platform.node(),
            "platform": platform.platform(), "python": sys.version.split()[0],
            "command_line": subprocess.list2cmdline([sys.executable, os.path.abspath(__file__)] + sys.argv[1:]),
            "executables": {label: executable_identity(path) for label, path in executables.items()},
            "sources": {}, "manifest_path": args.manifest, "manifest": None, "cases": [], "matrix_discovery": {}}
    if args.source_dir:
        meta["sources"][args.label] = git_identity(args.source_dir)
    if args.candidate_source_dir:
        meta["sources"][args.candidate_label] = git_identity(args.candidate_source_dir)
    if args.manifest and os.path.isfile(args.manifest):
        try:
            with open(args.manifest, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            meta["manifest"] = {k: manifest.get(k) for k in ("taken", "parent_head", "snapshot_commit",
                                                              "snapshot_branch", "copied_files",
                                                              "byte_identical_after_copy")}
        except (OSError, ValueError) as error:
            meta["manifest"] = {"error": str(error)}

    cases = []
    if args.case:
        cases = [("explicit", name, MATRIX_TAG if name.startswith(MATRIX_PREFIX) else
                  dict(BATTERY).get(name, "unavailable")) for name in args.case]
    else:
        if not args.no_battery:
            cases += [("battery", name, tag) for name, tag in BATTERY]
        if not args.no_matrix:
            names, error = list_matrix_tests(args.exe, args.timeout)
            meta["matrix_discovery"] = {"from": args.exe, "count": len(names), "error": error}
            if error or not names:
                # A missing matrix is a tooling error, reported as one ERROR row.
                cases.append(("matrix", MATRIX_PREFIX + "runsCase/*", MATRIX_TAG))
            cases += [("matrix", name, MATRIX_TAG) for name in names]
    meta["cases"] = [{"kind": k, "name": n, "metric_tag": t} for k, n, t in cases]

    sets, timings = {}, {}
    for label, exe in executables.items():
        print("Running %d case(s) on %s: %s" % (len(cases), label, exe))
        results, seconds = run_set(label, exe, cases, out_root, args.timeout)
        sets[label] = results
        timings[label] = {"total": seconds,
                          "battery": round(sum(r["duration_s"] or 0 for r in results if r["kind"] == "battery"), 3),
                          "matrix": round(sum(r["duration_s"] or 0 for r in results if r["kind"] == "matrix"), 3)}
    report_start = time.perf_counter()
    payload = write_reports(out_root, meta, sets, timings, args)
    timings["report_generation"] = round(time.perf_counter() - report_start, 3)
    timings["wall_total"] = round(time.perf_counter() - started, 3)
    payload["timings_s"] = timings
    source_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    payload["source_commit"] = subprocess.run(
        ["git", "-C", source_root, "rev-parse", "HEAD"], check=True, capture_output=True, text=True
    ).stdout.strip()
    all_counts = [counts(sets[label]) for label in sets]
    payload["tracker"] = {
        "producer": "ragdoll_report.py",
        "total_checks": sum(sum(item.values()) for item in all_counts),
        "failed_checks": sum(item["FAIL"] + item["ERROR"] for item in all_counts),
    }
    with open(os.path.join(out_root, "results.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1)

    if args.tracker_task:
        subprocess.run([
            sys.executable, os.path.join(source_root, "tools", "tracker", "write_result.py"),
            "--root", source_root, "--task", args.tracker_task,
            "--producer", "matrix_v1", "--artifact", os.path.join(out_root, "results.json"),
        ], check=True)

    worst = 0
    if any(value != "matched_reported_fields" for value in payload["fixture_comparisons"].values()):
        worst = 2
        print("Comparison ERROR: matrix fixture metadata differs or is unavailable")
    for label in sets:
        c = counts(sets[label])
        print("%s: %s (%d PASS, %d FAIL, %d ERROR)" % (label, verdict(sets[label]), c["PASS"], c["FAIL"], c["ERROR"]))
        if c["ERROR"]:
            worst = max(worst, 2)
        elif c["FAIL"]:
            worst = max(worst, 1)
    print("Report: %s" % os.path.join(out_root, "summary.md"))
    print("Exit code %d (0 all pass, 1 failures present, 2 tooling errors)" % worst)
    return worst


if __name__ == "__main__":
    sys.exit(main())
