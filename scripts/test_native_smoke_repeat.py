#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path


SPEC = importlib.util.spec_from_file_location(
    "native_smoke_repeat", Path(__file__).with_name("native_smoke_repeat.py"),
)
repeat = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(repeat)


class NativeSmokeRepeatTest(unittest.TestCase):
    def test_adds_pinned_runtime_dll_directories(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            root = workspace / "source-master"
            (root / "docs/ragdoll").mkdir(parents=True)
            release = workspace / "deps/installed/x64-windows/bin/Release"
            binary = workspace / "deps/installed/x64-windows/bin"
            release.mkdir(parents=True)
            (binary / "marker").touch()
            manifest = {"dependencies": {"windows_bundle": {
                "relative_path": "deps", "triplet": "x64-windows",
            }}}
            (root / "docs/ragdoll/version-manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            engine = workspace / "build/RelWithDebInfo/openmw.exe"
            result = repeat.native_runtime_environment(root, engine, {"PATH": "original"})
            self.assertEqual(result["PATH"].split(os.pathsep), [str(release), str(binary), "original"])
            self.assertEqual(result["OSG_LIBRARY_PATH"], str(engine.parent.resolve()))

    def documents(self, status: str = "passed"):
        summary = {"source_commit": "abc", "suites": [{
            "name": "suite", "matched_tests": 1,
            "cases": [{"name": "case", "status": status, "duration_ms": 10.0}],
        }]}
        manifest = {"engine_sha256": "engine", "arguments": {
            "test_filter": "^case$", "list_tests": False,
        }, "source_sha256": {"source": "hash"}}
        return summary, manifest

    def test_accepts_one_complete_exact_case(self) -> None:
        summary, manifest = self.documents()
        checks, verdict, duration = repeat.validate_documents(
            summary, manifest, "abc", "engine", "case", "^case$",
        )
        self.assertTrue(all(checks.values()))
        self.assertEqual(verdict, "passed")
        self.assertEqual(duration, 10.0)

    def test_rejects_missing_or_wrong_case(self) -> None:
        summary, manifest = self.documents()
        summary["suites"][0]["cases"] = []
        checks, verdict, _ = repeat.validate_documents(
            summary, manifest, "abc", "engine", "case", "^case$",
        )
        self.assertFalse(checks["one_exact_case"])
        self.assertFalse(checks["complete_verdict"])
        self.assertIsNone(verdict)

    def test_preserves_a_complete_failed_verdict(self) -> None:
        summary, manifest = self.documents("failed")
        checks, verdict, _ = repeat.validate_documents(
            summary, manifest, "abc", "engine", "case", "^case$",
        )
        self.assertTrue(all(checks.values()))
        self.assertEqual(verdict, "failed")


if __name__ == "__main__":
    unittest.main()
