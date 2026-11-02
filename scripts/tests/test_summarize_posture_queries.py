import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from summarize_posture_queries import summarize


class PostureQuerySummaryTest(unittest.TestCase):
    def run_summary(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "query.log"
            path.write_text(text)
            return summarize(path)

    def test_reports_geometry_success_and_rejection_separately(self):
        report = self.run_summary(
            "COORDINATED_QUERY schema=2 step=600 heading_fraction=1 heading_advance=1 valid=1 applied=0 shift=0,0,0\n"
            "COORDINATED_QUERY schema=2 step=600 heading_fraction=0.5 heading_advance=0.5 valid=1 applied=1 shift=0,0,2\n")
        rows = report["steps"][0]["queries"]
        self.assertFalse(rows[0]["pose_found"])
        self.assertTrue(rows[1]["pose_found"])
        self.assertAlmostEqual(rows[1]["advance_degrees"], 28.6478897565)

    def test_rejects_combined_runs(self):
        line = "COORDINATED_QUERY schema=2 step=600 heading_fraction=0 heading_advance=0 valid=1 applied=1 shift=0,0,0\n"
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.run_summary(line + line)

    def test_rejects_unlabeled_schema(self):
        with self.assertRaisesRegex(ValueError, "schema"):
            self.run_summary("COORDINATED_QUERY step=600 measured_heading=1 applied=1\n")


if __name__ == "__main__":
    unittest.main()
