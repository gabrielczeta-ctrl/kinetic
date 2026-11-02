#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


SPEC = importlib.util.spec_from_file_location(
    "duel_preflight", Path(__file__).with_name("duel_preflight.py"),
)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


class DuelPreflightTest(unittest.TestCase):
    def test_signatures_keep_outcomes_and_ignore_timestamps(self) -> None:
        run = {
            "cycles": [{
                "cycle": 1, "reactions": 1, "opponent_resumed": True,
                "saw_falling": True, "saw_fallen": True, "saw_getting_up": True,
                "saw_blend_back": True, "received_monotonic": 123,
                "recovery_seconds": 3.2,
            }],
            "duel_summary": {
                "cycles_requested": 3, "cycles_completed": 3, "reactions": 3,
                "opponent_strikes": 3, "soft_locks": 0, "final_mode": "standing",
                "maximum_recovery_seconds": 4.0, "test_step_multiplier": 4,
            },
        }
        self.assertEqual(preflight.cycle_signature(run), [{
            "cycle": 1, "reactions": 1, "opponent_resumed": True,
            "saw_falling": True, "saw_fallen": True, "saw_getting_up": True,
            "saw_blend_back": True,
        }])
        self.assertEqual(preflight.summary_signature(run), {
            "cycles_requested": 3, "cycles_completed": 3, "reactions": 3,
            "opponent_strikes": 3, "soft_locks": 0, "final_mode": "standing",
        })


if __name__ == "__main__":
    unittest.main()
