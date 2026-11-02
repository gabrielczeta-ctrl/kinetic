"""Containment controls for the bounded process-restart comparison launcher."""

import subprocess
import unittest
from unittest.mock import Mock, patch

import play01_validate_resume as validation


class BoundedValidationLauncherTests(unittest.TestCase):
    def test_outer_timeout_terminates_supervisor_tree_before_propagating(self):
        process = Mock()
        timeout = subprocess.TimeoutExpired(["python", "watchdog.py"], 360)
        process.communicate.side_effect = timeout
        with patch.object(validation.subprocess, "Popen", return_value=process) as launch:
            with patch.object(validation, "kill_child_tree") as terminate:
                with self.assertRaises(subprocess.TimeoutExpired):
                    validation.run_bounded(["python", "watchdog.py"], cwd="owned-checkout")
                terminate.assert_called_once_with(process)
                process.communicate.assert_called_once_with(timeout=360)
                self.assertEqual(launch.call_args.kwargs["cwd"], "owned-checkout")

    def test_interruption_terminates_supervisor_tree(self):
        process = Mock()
        process.communicate.side_effect = KeyboardInterrupt
        with patch.object(validation.subprocess, "Popen", return_value=process):
            with patch.object(validation, "kill_child_tree") as terminate:
                with self.assertRaises(KeyboardInterrupt):
                    validation.run_bounded(["python", "watchdog.py"], cwd="owned-checkout")
                terminate.assert_called_once_with(process)

    def test_success_does_not_terminate_an_exited_child(self):
        process = Mock(returncode=0)
        with patch.object(validation.subprocess, "Popen", return_value=process):
            with patch.object(validation, "kill_child_tree") as terminate:
                validation.run_bounded(["python", "watchdog.py"], cwd="owned-checkout", timeout=10)
                terminate.assert_not_called()
                process.communicate.assert_called_once_with(timeout=10)

    def test_failure_exit_is_not_reported_as_success(self):
        process = Mock(returncode=7)
        with patch.object(validation.subprocess, "Popen", return_value=process):
            with self.assertRaises(subprocess.CalledProcessError) as error:
                validation.run_bounded(["python", "watchdog.py"], cwd="owned-checkout")
        self.assertEqual(error.exception.returncode, 7)


if __name__ == "__main__":
    unittest.main()
