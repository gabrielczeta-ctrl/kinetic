import copy
import math
import unittest

import balance_meter_trace as meter


def fixture(sleeping=False, stationary=False):
    speed = 5.0 if stationary else math.sqrt(20.0)
    row = {"schema": "2", "actor_serial": "1", "step": "1", "epoch": "0", "dt": ".02",
           "authority_enabled": "0", "update_error": "0", "mass_known": "1", "mass_body_count": "3",
           "total_mass": "4", "com": "1.5,0,10", "com_velocity": "3,0,0", "root_kinematic": "1",
           "excluded_detached_weapon_known": "0", "support_normal_minimum_dot": ".5",
           "support": "unknown" if sleeping else "supported", "contact_count": "0" if sleeping else "1",
           "com_support_distance_known": "0" if sleeping else "1",
           "capture_point_known": "1" if stationary and not sleeping else "0",
           "capture_applicability": "valid-stationary-horizontal-plane" if stationary else "support-not-known-static",
           "support_plane_height_known": "1" if stationary and not sleeping else "0",
           "plane_normal_tolerance": "1e-5", "plane_height_tolerance": "1e-4", "gravity": "0,0,-9.81"}
    if not sleeping:
        row["com_support_distance"] = "0"
    if stationary and not sleeping:
        row["support_plane_height"] = "0"
        row["capture_point"] = f"{1.5 + 3 * math.sqrt(10 / 9.81)},0,0"
    for side, index in (("left", 1), ("right", 2)):
        row.update({side + "_contacts_complete": "0" if sleeping else "1",
                    side + "_active_before": "0" if sleeping else "1", side + "_geometry_complete": "1",
                    side + "_foot_body_known": "1", side + "_foot_body": str(index),
                    side + "_support": "unknown" if sleeping else "supported" if side == "left" else "unsupported",
                    side + "_normal_impulse_known": "0" if sleeping else "1",
                    side + "_load_fraction_known": "0" if sleeping else "1",
                    side + "_maximum_tangent_speed_known": "1" if side == "left" and not sleeping else "0",
                    side + "_maximum_tangent_distance_step_known": "1" if side == "left" and not sleeping else "0"})
        if not sleeping:
            row[side + "_normal_impulse"] = "2" if side == "left" else "0"
            row[side + "_load_fraction"] = "1" if side == "left" else "0"
            if side == "left":
                row[side + "_maximum_tangent_speed"] = str(speed)
                row[side + "_maximum_tangent_distance_step"] = str(speed * .02)
    masses = []
    for index, mass in ((0, 1), (1, 2), (2, 1)):
        masses.append({"schema": "2", "actor_serial": "1", "step": "1", "epoch": "0", "body": str(index),
                       "body_id": str(index + 100), "mass": str(mass), "active": "0" if sleeping else "1",
                       "declared_kinematic_root": "1" if index == 0 else "0",
                       "com": "0,0,10" if index == 0 else "2,0,10", "velocity": "0,0,0" if index == 0 else "4,0,0"})
    contacts = [] if sleeping else [{"schema": "2", "actor_serial": "1", "step": "1", "epoch": "0",
        "point": "0", "body": "1", "self_body_id": "101", "surface_body_id": "200", "subshape1": "0",
        "subshape2": "0", "manifold_point": "0", "first_body": "1", "surface_static_known": "1",
        "surface_static": "1" if stationary else "0", "surface_point_known": "1", "surface_point": "1.5,0,0",
        "position": "1.5,0,0", "normal": "0,0,1", "normal_impulse_known": "1", "normal_impulse": "2",
        "foot_velocity_known": "1", "surface_velocity_known": "1", "foot_velocity": "3,4,2",
        "surface_velocity": "0,0,0" if stationary else "1,0,0"}]
    return row, masses, contacts


def line(prefix, row):
    return prefix + " " + " ".join(f"{key}={value}" for key, value in row.items())


def text(data):
    row, masses, contacts = data
    return "\n".join([line("RAGDOLL_BALANCE", row)]
        + [line("RAGDOLL_BALANCE_MASS", item) for item in masses]
        + [line("RAGDOLL_BALANCE_CONTACT", item) for item in contacts])


def knee_rows(data):
    main = data[0]
    main["knee_schema"] = "1"
    rows = []
    for side in ("left", "right"):
        row = {"schema": "1", "actor_serial": main["actor_serial"], "step": main["step"],
               "epoch": main["epoch"], "foot": side}
        for name, value in {
            "knee_motor": "recorded-request-and-target-before-update",
            "knee_clip": "raw-clip-unavailable-evaluated-animation-only",
            "knee_composed": "accepted-pose-submission",
            "composed_foot_availability": "frozen-reference-to-postsolve-origin",
            "animation_availability": "evaluated-animation-binding", "composed_source": "animation-drive",
            "composed_foot_reference": "0,0,10", "actual_foot_origin": ".3,0,10",
            "composed_foot_error_vector": ".3,0,0",
        }.items(): row[side + "_" + name] = value
        for name, value in {
            "composed_foot_error": ".3", "knee_animation": ".1", "knee_composed_angle": ".7",
            "knee_final_to_actual": ".1", "knee_actual": ".4", "knee_requested": ".7",
            "knee_final": ".5", "knee_minimum": "-1", "knee_maximum": "1", "knee_adjustment": ".2",
            "animation_sample_serial": "3", "animation_sample_next_step": main["step"],
            "composed_serial": "4", "composed_step": main["step"], "motor_observation_step": main["step"],
            "motor_state": "2",
        }.items():
            row[side + "_" + name + "_known"] = "1"
            row[side + "_" + name] = value
        rows.append(row)
    return rows


def with_knees(data, knees):
    return text(data) + "\n" + "\n".join(line("RAGDOLL_BALANCE_KNEE", row) for row in knees)


class BalanceMeterTraceTest(unittest.TestCase):
    def assert_rejected(self, data, reason):
        result = meter.audit(text(data))
        self.assertFalse(result["success"])
        self.assertTrue(any(reason in value for value in result["violations"]), result["violations"])

    def test_accepts_moving_surface_relative_slip_and_load(self):
        result = meter.audit(text(fixture()))
        self.assertTrue(result["success"], result)
        self.assertEqual(result["sample_count"], 1)

    def test_preserves_sleeping_mass_and_unknown_empty_support(self):
        data = fixture(sleeping=True)
        result = meter.audit(text(data))
        self.assertTrue(result["success"], result)
        data[0]["total_mass"] = "0"
        self.assert_rejected(data, "assembly mass")

    def test_rejects_duplicate_samples_and_detail_rows(self):
        data = fixture()
        for duplicate in (line("RAGDOLL_BALANCE", data[0]), line("RAGDOLL_BALANCE_CONTACT", data[2][0])):
            result = meter.audit(text(data) + "\n" + duplicate)
            self.assertFalse(result["success"])
            self.assertTrue(any("duplicate" in error for error in result["violations"]))

    def test_rejects_manifold_duplication_even_with_new_detail_index(self):
        data = fixture()
        duplicate = copy.deepcopy(data[2][0])
        duplicate["point"] = "1"
        data[2].append(duplicate)
        data[0]["contact_count"] = "2"
        self.assert_rejected(data, "duplicate contact manifold point")

    def test_rejects_fabricated_zero_when_unknown(self):
        data = fixture(sleeping=True)
        data[0]["left_maximum_tangent_speed"] = "0"
        self.assert_rejected(data, "fabricated")

    def test_rejects_world_speed_in_place_of_relative_tangent_speed(self):
        data = fixture()
        data[0]["left_maximum_tangent_speed"] = "5"
        self.assert_rejected(data, "maximum tangent speed")

    def test_rejects_accumulated_distance_disguised_as_one_step(self):
        data = fixture()
        data[0]["left_maximum_tangent_distance_step"] = "100"
        self.assert_rejected(data, "per-step slip")

    def test_rejects_incomplete_snapshot_and_lifecycle_mix(self):
        data = fixture()
        data[1].pop()
        self.assert_rejected(data, "incomplete mass rows")
        data = fixture()
        data[2][0]["epoch"] = "1"
        self.assert_rejected(data, "lifecycle epoch")

    def test_accepts_static_plane_capture_and_rejects_moving_substitution(self):
        data = fixture(stationary=True)
        result = meter.audit(text(data))
        self.assertTrue(result["success"], result)
        data[2][0]["surface_static"] = "0"
        self.assert_rejected(data, "unknown/moving")

    def test_rejects_wrong_capture_point_formula_and_fake_support_distance(self):
        data = fixture(stationary=True)
        data[0]["capture_point"] = "100,0,0"
        self.assert_rejected(data, "capture point")
        data = fixture(sleeping=True)
        data[0].update(com_support_distance_known="1", com_support_distance="0")
        self.assert_rejected(data, "numeric distance")

    def test_rejects_wrong_load_fractions_and_nonmonotonic_steps(self):
        data = fixture()
        data[0]["left_load_fraction"] = ".5"
        self.assert_rejected(data, "load fraction")
        first, second = fixture(), fixture()
        for row in [first[0], *first[1], *first[2]]:
            row["step"] = "5"
        result = meter.audit(text(first) + "\n" + text(second))
        self.assertFalse(result["success"])
        self.assertTrue(any("nonmonotonic" in item for item in result["violations"]))

    def test_empty_input_cannot_qualify(self):
        self.assertFalse(meter.audit("unrelated native output")["success"])


class BalanceKneeTraceTest(unittest.TestCase):
    def assert_rejected(self, raw, reason=None):
        result = meter.audit(raw)
        self.assertFalse(result["success"], result)
        if reason:
            self.assertTrue(any(reason in error for error in result["violations"]), result)

    def test_accepts_joined_knees_before_or_after_main(self):
        data = fixture()
        rows = knee_rows(data)
        for raw in (with_knees(data, rows), "\n".join(line("RAGDOLL_BALANCE_KNEE", r) for r in rows) + "\n" + text(data)):
            result = meter.audit(raw)
            self.assertTrue(result["success"], result)
            self.assertEqual(result["knee_sample_count"], 2)

    def test_missing_or_duplicate_bilateral_rows_fail(self):
        data = fixture()
        rows = knee_rows(data)
        self.assert_rejected(with_knees(data, rows[:1]), "incomplete bilateral")
        self.assert_rejected(with_knees(data, rows + rows[:1]), "duplicate knee")
        self.assert_rejected(text(data), "incomplete bilateral")

    def test_wrong_actor_step_epoch_cannot_join(self):
        for name in ("actor_serial", "step", "epoch"):
            data = fixture()
            rows = knee_rows(data)
            rows[1][name] = "99"
            self.assert_rejected(with_knees(data, rows), "epoch" if name == "epoch" else "orphan")

    def test_exact_integer_step_identity_above_lua_double_precision(self):
        data = fixture()
        for row in [data[0], *data[1], *data[2]]: row["step"] = str(2**53 + 32)
        rows = knee_rows(data)
        self.assertTrue(meter.audit(with_knees(data, rows))["success"])
        rows[1]["right_composed_step"] = str(2**53 + 33)
        self.assert_rejected(with_knees(data, rows), "differs from completed")

    def test_unknown_values_and_invalid_motor_state_fail(self):
        for field, value in (("left_knee_actual_known", "0"), ("left_knee_actual", "nan"),
                             ("left_motor_state", "3"), ("left_composed_step", "1.0")):
            data = fixture()
            rows = knee_rows(data)
            rows[0][field] = value
            self.assert_rejected(with_knees(data, rows))

    def test_future_animation_and_incomplete_pose_identity_fail(self):
        data = fixture()
        rows = knee_rows(data)
        rows[0]["left_animation_sample_next_step"] = "2"
        self.assert_rejected(with_knees(data, rows), "future")
        rows[0]["left_animation_sample_next_step"] = "1"
        rows[0]["left_composed_serial_known"] = "0"
        del rows[0]["left_composed_serial"]
        self.assert_rejected(with_knees(data, rows), "incomplete knee identity")

    def test_signed_foot_error_and_motor_deltas_reconstructed(self):
        for field, value in (("left_composed_foot_error_vector", "-.3,0,0"),
                             ("left_composed_foot_error", "4"), ("left_knee_adjustment", "9"),
                             ("left_knee_final_to_actual", "9")):
            data = fixture()
            rows = knee_rows(data)
            rows[0][field] = value
            self.assert_rejected(with_knees(data, rows))

    def test_off_motor_and_unavailable_stages_remain_explicit(self):
        data = fixture(sleeping=True)
        rows = knee_rows(data)
        for row in rows:
            side = row["foot"]
            for key in list(row):
                if key.endswith("_known") and not key.endswith(("motor_state_known", "motor_observation_step_known")):
                    row[key] = "0"
                    row.pop(key[:-6], None)
            for name in ("composed_foot_reference", "actual_foot_origin", "composed_foot_error_vector"):
                row.pop(side + "_" + name)
            row[side + "_motor_state"] = "0"
        result = meter.audit(with_knees(data, rows))
        self.assertTrue(result["success"], result)

    def test_truncated_and_malformed_rows_cannot_disappear(self):
        data = fixture()
        rows = knee_rows(data)
        raw = with_knees(data, rows)
        self.assert_rejected(raw.rsplit("right_motor_state=", 1)[0], "right_motor_state")
        self.assert_rejected(raw + "\nRAGDOLL_BALANCE_KNEE", "schema")
        self.assert_rejected(raw + "\nRAGDOLL_BALANCE_KNEE schema=1 actor_serial=1 step=2 epoch=0 foot=left broken")
        self.assert_rejected(raw + "\nRAGDOLL_BALANCE_KNEE schema=9 actor_serial=1 step=2 epoch=0 foot=left", "schema")

    def test_simulated_native_line_buffer_truncation_fails(self):
        data = fixture()
        rows = knee_rows(data)
        row = rows[1]
        state = row.pop("right_motor_state")
        row["padding"] = "x" * 4096
        row["right_motor_state"] = state
        truncated = line("RAGDOLL_BALANCE_KNEE", row)[:4095]
        raw = with_knees(data, rows[:1]) + "\n" + truncated
        self.assert_rejected(raw, "right_motor_state")

    def test_schema_marker_and_foot_ownership_cannot_be_forged(self):
        data = fixture()
        rows = knee_rows(data)
        data[0].pop("knee_schema")
        self.assert_rejected(with_knees(data, rows), "no main knee_schema")
        data[0]["knee_schema"] = "1"
        rows[0]["right_knee_actual_known"] = "0"
        self.assert_rejected(with_knees(data, rows), "opposite foot")


if __name__ == "__main__":
    unittest.main()
