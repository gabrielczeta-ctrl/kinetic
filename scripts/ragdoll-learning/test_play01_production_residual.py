"""Physical-selection and allocation safeguards for the production residual experiment."""
import copy
import unittest
from play01_runner import evaluation_guard, physical_selection_key


def metric(falls=1, rmse=22.0, speed=25.0):
    return {"falls": falls, "mean_commanded_velocity_rmse_walk": rmse,
            "mean_achieved_along_command_speed_walk": speed,
            "mean_joint_acceleration_rms": 80.0, "mean_support_slip_sum": 20.0,
            "violation_counts": {"nonfoot_support": falls, "slip": 10}}


class OvernightGuardTests(unittest.TestCase):
    rules = {"minimum_usable_transitions": 500000, "consecutive_evaluations": 3,
             "additional_falls_limit": 3, "plateau_minimum_usable_transitions": 4000000,
             "minimum_relative_rmse_improvement": .02}

    def test_a_fall_cannot_be_traded_for_minor_violation_reduction(self):
        fallen = metric(falls=2, rmse=1)
        fallen["violation_counts"]["slip"] = 0
        self.assertLess(physical_selection_key(metric()), physical_selection_key(fallen))

    def test_persistent_regression_requires_minimum_exposure_and_window(self):
        history = [{"summary": metric(falls=8)}] * 3
        self.assertIsNone(evaluation_guard(history, metric(), self.rules, 499999))
        self.assertIsNone(evaluation_guard(history[:2], metric(), self.rules, 1000000))
        self.assertIn("regression", evaluation_guard(history, metric(), self.rules, 1000000))

    def test_early_flat_learning_is_not_a_plateau_stop(self):
        history = [{"summary": metric()}] * 3
        self.assertIsNone(evaluation_guard(history, metric(), self.rules, 3999999))
        self.assertIn("minimum exposure", evaluation_guard(history, metric(), self.rules, 4000000))

    def test_speed_improvement_with_worse_physics_does_not_count(self):
        better = metric(rmse=20)
        history = [{"summary": better}] * 3
        self.assertIsNone(evaluation_guard(history, metric(), self.rules, 4000000))
        degraded = copy.deepcopy(better)
        degraded["violation_counts"]["slip"] += 1
        self.assertIsNotNone(evaluation_guard([{"summary": degraded}] * 3, metric(), self.rules, 4000000))


if __name__ == "__main__":
    unittest.main()
