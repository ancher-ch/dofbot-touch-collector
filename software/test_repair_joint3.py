import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from repair_joint3 import boundary_plan, repair


class Arm:
    def __init__(self):
        self.q = [86, 172, -4, 11, 89, 132]
        self.writes = []

    def Arm_serial_servo_read(self, joint):
        value = self.q[joint - 1]
        return value if value >= 0 else None

    def Arm_serial_servo_write(self, joint, target, duration):
        self.writes.append((joint, target, duration))
        self.q[joint - 1] = target


class RepairTests(unittest.TestCase):
    def test_raw_mapping_and_small_step_limit(self):
        plan = boundary_plan(3152)
        self.assertEqual(plan["sdk_mapped_angle"], -4)
        self.assertLessEqual(plan["maximum_change_degrees"], 5)
        for raw in (0, 900, 3100, 3162, -1, True):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                boundary_plan(raw)

    @patch("repair_joint3.read_raw_position", return_value=3152)
    def test_inspection_and_cancellation_never_move(self, _raw):
        arm = Arm()
        self.assertFalse(repair(arm))
        with patch("builtins.input", return_value="CANCEL"):
            self.assertFalse(repair(arm, True))
        self.assertEqual(arm.writes, [])

    @patch("repair_joint3.time.sleep")
    @patch("repair_joint3.read_raw_position", return_value=3152)
    def test_only_joint_three_moves_once_after_confirmation(self, _raw, _sleep):
        arm = Arm()
        with tempfile.TemporaryDirectory() as folder, patch("builtins.input", return_value="MOVE3"):
            self.assertTrue(repair(arm, True, Path(folder) / "state.json"))
        self.assertEqual(arm.writes, [(3, 0, 1500)])
        self.assertEqual(arm.q, [86, 172, 0, 11, 89, 132])

    @patch("repair_joint3.read_raw_position", side_effect=[3152, 3130])
    def test_drift_before_execution_prevents_write(self, _raw):
        arm = Arm()
        with patch("builtins.input", return_value="MOVE3"), self.assertRaises(RuntimeError):
            repair(arm, True)
        self.assertEqual(arm.writes, [])

    @patch("repair_joint3.time.sleep")
    @patch("repair_joint3.read_raw_position", return_value=3108)
    def test_already_valid_feedback_never_repeats_motion(self, _raw, _sleep):
        arm = Arm()
        arm.q[2] = 0
        self.assertTrue(repair(arm, True))
        self.assertEqual(arm.writes, [])

    @patch("repair_joint3.time.sleep")
    @patch("repair_joint3.read_raw_position", return_value=3152)
    def test_transient_feedback_failure_waits_without_resending_motion(self, _raw, _sleep):
        class SettlingArm(Arm):
            failures = 6

            def Arm_serial_servo_read(self, joint):
                if joint == 3 and self.writes and self.failures:
                    self.failures -= 1
                    return None
                return super().Arm_serial_servo_read(joint)

        arm = SettlingArm()
        with tempfile.TemporaryDirectory() as folder, patch("builtins.input", return_value="MOVE3"):
            self.assertTrue(repair(arm, True, Path(folder) / "state.json"))
        self.assertEqual(arm.writes, [(3, 0, 1500)])


if __name__ == "__main__":
    unittest.main()
