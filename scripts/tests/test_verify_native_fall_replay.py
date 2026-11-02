import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    "verify_native_fall_replay", Path(__file__).parents[1] / "verify_native_fall_replay.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WitnessVerificationTests(unittest.TestCase):
    def test_history_divergence_cannot_hide_behind_physical_match(self):
        result = dict(comparison_scope="physical_state", frames=2,
            offline_first_fall_frame=None, first_divergent_frame=None, equivalence_proven=False,
            swing_history_compared=True, first_swing_history_difference_frame=None)
        module.verify(result, None, 2)
        with self.assertRaises(ValueError):
            module.verify(result | dict(first_swing_history_difference_frame=1), None, 2)
        result.pop("first_swing_history_difference_frame")
        with self.assertRaises(ValueError):
            module.verify(result, None, 2)

    def test_npc_fall_does_not_use_player_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.log"
            path.write_text('RAGDOLL_REPLAY_FRAME frame=0 step=10\n'
                'RAGDOLL_STEP phase=observed player=1 actor="player" integer_radix=10 step=10 mode=0\n'
                'RAGDOLL_STEP phase=observed player=0 actor="npc a" integer_radix=10 step=10 mode=3\n')
            self.assertEqual(module.native_fall(path, actor_kind="npc"), (0, 1))
            self.assertEqual(module.native_fall(path, allow_no_fall=True), (None, 1))

    def test_replay_environment_preserves_presence_semantics(self):
        key = "OPENMW_RAGDOLL_REPLAY_RESTART_OBSERVATION"
        self.assertEqual(module.replay_environment({})[key], {"present": False, "value": None})
        for value in ("", "0", "1"):
            self.assertEqual(module.replay_environment({key: value})[key],
                {"present": True, "value": value})
        self.assertNotIn("UNRELATED_SECRET", module.replay_environment({"UNRELATED_SECRET": "hidden"}))

    def test_context_identity_includes_external_ground_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shape = root / "ground shape.jolt"
            shape.write_bytes(b"first shape")
            context = root / "context.txt"
            lines = ['4 "profile" 0 1', '75 1 0', '0 0 0', '0 0 0 1',
                '0 0 0', '0 0 0', '1', '"pelvis" -1 0 0 0 0 0 0 1', '1',
                '2 0 0 0 0 0 0 1 0.2 0 ' + json.dumps(str(shape.resolve()))]
            context.write_text('\n'.join(lines), encoding="utf-8")
            before = module.context_dependencies(context)
            shape.write_bytes(b"changed shape")
            self.assertNotEqual(before, module.context_dependencies(context))
            shape.write_bytes(b"")
            with self.assertRaises(ValueError):
                module.context_dependencies(context)
            shape.unlink()
            with self.assertRaises(FileNotFoundError):
                module.context_dependencies(context)

    def test_surviving_recording_requires_matching_no_fall_and_no_divergence(self):
        result = dict(comparison_scope="physical_state", frames=2,
            offline_first_fall_frame=None, first_divergent_frame=None, equivalence_proven=False)
        module.verify(result, None, 2)
        for change in (dict(offline_first_fall_frame=1), dict(first_divergent_frame=1)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                module.verify(result | change, None, 2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.log"
            path.write_text("RAGDOLL_REPLAY_FRAME frame=0 step=95\n"
                "RAGDOLL_STEP phase=observed player=1 integer_radix=10 step=95 mode=1\n")
            self.assertEqual(module.native_fall(path, allow_no_fall=True), (None, 1))
            with self.assertRaises(ValueError):
                module.native_fall(path)

    def test_exact_curve_requires_every_frame_and_every_channel(self):
        header = ("frame,max_translation_error,max_linear_velocity_error,"
            "max_angular_velocity_error,max_quaternion_component_error\n")
        valid = header + "0,0,0,0,0\n1,0,0,0,0\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "curve.csv"
            path.write_text(valid)
            self.assertEqual(module.verify_exact_physical_curve(path, 2)["frames"], 2)
            invalid = [header, valid.replace("1,0,0,0,0", "0,0,0,0,0"),
                valid.replace("1,0,0,0,0", "2,0,0,0,0"),
                header + "1,0,0,0,0\n0,0,0,0,0\n",
                valid.replace("max_quaternion_component_error", "unknown_channel")]
            for column in range(1, 5):
                for value in ("1e-30", "-1", "nan", "inf", "", "invalid"):
                    row = ["1", "0", "0", "0", "0"]
                    row[column] = value
                    invalid.append(header + "0,0,0,0,0\n" + ",".join(row) + "\n")
            for text in invalid:
                path.write_text(text)
                with self.subTest(curve=text), self.assertRaises(ValueError):
                    module.verify_exact_physical_curve(path, 2)

    def test_rejects_misleading_success_reports(self):
        baseline = dict(comparison_scope="physical_state", frames=600,
            offline_first_fall_frame=463, first_divergent_frame=489, equivalence_proven=False)
        module.verify(baseline, 463, 600)
        for change in [dict(offline_first_fall_frame=None), dict(offline_first_fall_frame=464),
                dict(first_divergent_frame=463), dict(first_divergent_frame=0),
                dict(frames=599), dict(equivalence_proven=True)]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                module.verify(baseline | change, 463, 600)

    def test_uses_observed_mode_and_capture_frame_not_absolute_step(self):
        text = ("RAGDOLL_REPLAY_FRAME frame=0 step=95\n"
            "RAGDOLL_STEP phase=input player=1 integer_radix=10 step=95 mode=3\n"
            "RAGDOLL_STEP phase=observed player=1 integer_radix=10 step=95 mode=1\n"
            "RAGDOLL_REPLAY_FRAME frame=1 step=96\n"
            "RAGDOLL_STEP phase=observed player=1 integer_radix=10 step=96 mode=3\n")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.log"
            path.write_text(text)
            self.assertEqual(module.native_fall(path), (1, 2))
            for invalid in [text.replace("integer_radix=10", "integer_radix=16"),
                    text.replace("frame=1", "frame=2"),
                    text + "RAGDOLL_REPLAY_FRAME frame=1 step=96\n",
                    text.replace("step=96 mode=3", "step=96 mode=1")]:
                path.write_text(invalid)
                with self.subTest(log=invalid), self.assertRaises(ValueError):
                    module.native_fall(path)


    def test_world_position_check_rejects_shift_and_missing_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'world.csv'
            path.write_text('frame,max_world_position_error\n0,0.00001\n')
            self.assertEqual(module.verify_world_positions(path,1)['frames'],1)
            for value in ('1', 'nan', '-1'):
                path.write_text('frame,max_world_position_error\n0,'+value+'\n')
                with self.assertRaises(ValueError): module.verify_world_positions(path,1)
            path.write_text('frame,max_world_position_error\n')
            with self.assertRaises(ValueError): module.verify_world_positions(path,1)


if __name__ == "__main__":
    unittest.main()
