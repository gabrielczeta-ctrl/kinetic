import sys
import unittest
import csv
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_turn_touchdown import classify_contact_events, inspect


def row(step, impulse, stage="swing", stance="17"):
    return {"step": str(step), "left_contact_normal_impulse": str(impulse),
            "reference_stage": stage, "reference_stance_hip_body": stance}


class ContactEventTests(unittest.TestCase):
    def test_one_missing_unloading_impulse_does_not_change_accepted_onset(self):
        control = [row(i, f) for i, f in enumerate([20, 4, 30, 0, 0, 60, 180], 582)]
        candidate = [dict(r) for r in control]
        candidate[1]["left_contact_normal_impulse"] = "0"
        following = row(589, 300, "double_support", "14")
        raw_a, accepted_a = classify_contact_events(control, following)
        raw_b, accepted_b = classify_contact_events(candidate, following)
        self.assertNotEqual(raw_a["step"], raw_b["step"])
        self.assertEqual(accepted_a["step"], "587")
        self.assertEqual(accepted_b["step"], "587")

    def test_return_contact_without_support_transition_is_not_accepted(self):
        rows = [row(582, 0), row(583, 20)]
        raw, accepted = classify_contact_events(rows, None)
        self.assertEqual(raw["step"], "583")
        self.assertIsNone(accepted)
        self.assertIsNone(classify_contact_events(rows, row(584, 10, "double_support", "17"))[1])

    def test_contact_loss_before_transition_has_no_continuous_onset(self):
        rows = [row(582, 0), row(583, 20), row(584, 0)]
        self.assertIsNone(classify_contact_events(rows, row(585, 20, "double_support", "14"))[1])

    def test_final_motor_command_wins_over_earlier_frame_summary(self):
        rows = [row(582, 0), row(583, 20), row(584, 50, "double_support", "14")]
        for r in rows:
            r.update(reference_placement_x="3", reference_placement_y="4", left_foot_x="0", left_foot_y="0",
                     reference_time_in_stage="0.2", pelvis_up="0.99", swing_hip_target_error="99",
                     swing_hip_target_clamp="99", swing_hip_motor_utilization="0.3", reference_swing_reach_limited="0")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offline.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            with (path.parent / "startup-commands.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["step", "body", "angular_error", "target_clamp_delta"])
                writer.writeheader(); writer.writerow(dict(step=583, body=14, angular_error=0.2, target_clamp_delta=0.01))
            report = inspect(path)
            self.assertEqual(report["accepted_contact_onset"]["pre_solve_hip_angular_error"], 0.2)
            self.assertEqual(report["accepted_contact_onset"]["motor_source"], "final startup-commands.csv")
            self.assertEqual(report["support_transition_step"], 584)

    def test_missing_transition_frame_does_not_certify_acceptance(self):
        rows = [row(582, 0), row(583, 20)]
        self.assertIsNone(classify_contact_events(rows, row(586, 20, "double_support", "14"))[1])


if __name__ == "__main__":
    unittest.main()
