import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_transfer_acceleration import analyze


class TransferAccelerationTest(unittest.TestCase):
    def sample(self, planned="1", gap=False):
        fields = ["step", "time", "reference_stage", "joint_support_planned"]
        fields += [name + axis for axis in ("x", "y") for name in
                   ("com_velocity_", "reference_requested_acceleration_", "joint_support_acceleration_")]
        rows = [[0, 0, "", 0, 10, 0, 0, 0, 0, 0],
                [2 if gap else 1, .1, "double_support", planned, 12, 30, 20, 0, 0, 0],
                [3 if gap else 2, .2, "swing", 1, 100, 0, 0, 0, 0, 0]]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(rows)
            return analyze(path, 1, 2)

    def test_uses_previous_observation_not_next_step(self):
        x = self.sample()["transfers"][0]["x"]
        self.assertAlmostEqual(x["requested_delta_v"], 3)
        self.assertAlmostEqual(x["projected_delta_v"], 2)
        self.assertAlmostEqual(x["observed_delta_v"], 2)

    def test_absent_support_plan_is_unknown(self):
        self.assertIsNone(self.sample("0")["transfers"][0]["x"]["projected_delta_v"])

    def test_rejects_missing_steps(self):
        with self.assertRaisesRegex(ValueError, "Noncontiguous"):
            self.sample(gap=True)


if __name__ == "__main__":
    unittest.main()
