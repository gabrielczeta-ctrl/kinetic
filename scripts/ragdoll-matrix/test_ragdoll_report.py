"""Evidence integrity checks; no physical acceptance claims."""
import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import ragdoll_report as report


class ReportTests(unittest.TestCase):
    def record(self, name="nominal60", speed="60", gain="0.5"):
        return {"name": report.MATRIX_PREFIX + "runsCase/" + name, "metrics": {
            "REFERENCE_MATRIX_SETTINGS": [{"case": name, "velocity_gain": gain}],
            "REFERENCE_MATRIX_SPEC": [{"case": name, "speed": speed}]}}

    def test_fixture_comparison_checks_script_and_settings_per_case(self):
        baseline = [self.record(), self.record("unpushed30", "30", "1")]
        candidate = copy.deepcopy(baseline)
        self.assertEqual(set(report.compare_fixtures({"base": baseline, "candidate": candidate}).values()),
                         {"matched_reported_fields"})
        # A set comparison would miss gains swapped between cases.
        candidate[0]["metrics"]["REFERENCE_MATRIX_SETTINGS"][0]["velocity_gain"] = "1"
        candidate[1]["metrics"]["REFERENCE_MATRIX_SETTINGS"][0]["velocity_gain"] = "0.5"
        self.assertEqual(set(report.compare_fixtures({"base": baseline, "candidate": candidate}).values()),
                         {"different"})
        changed_script = self.record(speed="45")
        comparison = report.compare_fixtures({"base": [baseline[0]], "candidate": [changed_script]})
        self.assertEqual(list(comparison.values()), ["different"])

    def test_missing_fixture_metadata_is_not_equality(self):
        baseline = self.record()
        candidate = self.record()
        del candidate["metrics"]["REFERENCE_MATRIX_SPEC"]
        self.assertEqual(list(report.compare_fixtures({"base": [baseline], "candidate": [candidate]}).values()),
                         ["unavailable"])

    def test_missing_metrics_are_not_zero(self):
        row = report.table_row({"metrics": {}}, "REFERENCE_MATRIX")
        self.assertEqual(set(row.values()), {"unavailable"})

    def test_cli_rejects_empty_selection_duplicate_labels_and_reused_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory, "evidence.txt")
            for extra in (["--no-battery", "--no-matrix"],
                          ["--candidate-exe", sys.executable, "--candidate-label", "baseline"],
                          ["--label", "../escape"], ["--timeout", "0"]):
                result = subprocess.run([sys.executable, report.__file__, "--exe", sys.executable,
                                         "--out", directory] + extra, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(os.listdir(directory), [])
            marker.write_text("keep", encoding="utf-8")
            result = subprocess.run([sys.executable, report.__file__, "--exe", sys.executable,
                                     "--out", directory], capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(marker.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()
