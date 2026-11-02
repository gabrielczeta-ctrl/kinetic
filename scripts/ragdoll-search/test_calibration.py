import copy
import importlib.util
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

CALIBRATION_SPEC = importlib.util.spec_from_file_location("calibration", HERE / "calibration.py")
calibration = importlib.util.module_from_spec(CALIBRATION_SPEC)
assert CALIBRATION_SPEC.loader is not None
CALIBRATION_SPEC.loader.exec_module(calibration)

QUALIFY_SPEC = importlib.util.spec_from_file_location("qualify", HERE / "qualify.py")
qualify = importlib.util.module_from_spec(QUALIFY_SPEC)
assert QUALIFY_SPEC.loader is not None
QUALIFY_SPEC.loader.exec_module(qualify)


class CalibrationContractTests(unittest.TestCase):
    def setUp(self):
        self.root = HERE.parents[1]
        context, profile = calibration.canonical_paths(self.root)
        self.valid = calibration.qualification_metadata(context, profile, 300, [778], 777, root=self.root)

    def test_portable_fixture_matches_profile_and_contract(self):
        checks = calibration.require_contract(self.valid)
        self.assertTrue(all(checks.values()))
        self.assertEqual(self.valid["fixture"]["bone_count"], 20)

    def test_each_calibration_dimension_is_enforced(self):
        mutations = {
            "body": ("fixture", "body_id", "context-symA"),
            "profile": ("fixture", "profile_id", "openmw_humanoid_v0"),
            "floor": (None, "primary_floor", "box"),
            "manifold": (None, "manifold_reduction", True),
            "rate": (None, "physics_hz", 120),
            "sample": (None, "sample_n_per_condition", 299),
            "holdout": (None, "holdout_disjoint", False),
            "matrix": (None, "promotion_conditions", ["box", "mesh", "tiled"]),
        }
        for name, (parent, field, value) in mutations.items():
            with self.subTest(name=name):
                changed = copy.deepcopy(self.valid)
                (changed[parent] if parent else changed)[field] = value
                with self.assertRaises(ValueError):
                    calibration.require_contract(changed)

    def test_screen_keeps_borderline_candidate_for_real_holdout(self):
        summary = lambda fall, crashes=0: {"fall_rate": fall, "crashes": crashes}
        rows = [
            {"label": "default", "summary": summary(0.05)},
            {"label": "close", "summary": summary(0.14)},
            {"label": "bad", "summary": summary(0.30)},
            {"label": "crash", "summary": summary(0.00, 1)},
        ]
        survivors, decisions = qualify.screen_survivors(rows)
        self.assertEqual(survivors, ["default", "close"])
        self.assertEqual([row["survived"] for row in decisions], [True, False, False])


if __name__ == "__main__":
    unittest.main()
