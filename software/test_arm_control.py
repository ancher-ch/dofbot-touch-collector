"""Software guards only; no Arm_Lib import and no hardware access."""
import math
import unittest
from unittest.mock import patch

from arm_control import read_angle, target_angle, write_angle, run_jog, run_demo, group_target, run_group


class SimulatedArm:
    def __init__(self):
        self.position = 53
        self.writes = []

    def Arm_serial_servo_read(self, joint):
        return self.position

    def Arm_serial_servo_write(self, joint, angle, duration):
        self.writes.append((joint, angle, duration))
        self.position = angle


class SimulatedGroupArm:
    def __init__(self):
        self.positions = [90, 135, 0, 45, 89, 52]
        self.writes = []

    def Arm_serial_servo_read(self, joint):
        return self.positions[joint - 1]

    def Arm_serial_servo_write6_array(self, pose, duration):
        self.writes.append((list(pose), duration))
        self.positions = list(pose)


class ArmControlGuardTests(unittest.TestCase):
    @patch("arm_control.time.sleep")
    def test_jog_reports_measured_progress(self, _sleep):
        arm = SimulatedArm()
        self.assertEqual(run_jog(arm, 4, 53, 58, 2000), 58)
        self.assertEqual(arm.writes, [(4, 58, 2000)])

    @patch("arm_control.time.sleep")
    def test_jog_stuck_feedback_fails_without_resend(self, _sleep):
        class StuckArm(SimulatedArm):
            def Arm_serial_servo_write(self, joint, angle, duration):
                self.writes.append((joint, angle, duration))
        arm = StuckArm()
        arm.position = 0
        with self.assertRaisesRegex(RuntimeError, "本次测试未通过"):
            run_jog(arm, 4, 0, 5, 2000)
        self.assertEqual(arm.writes, [(4, 5, 2000)])

    def test_jog_rejects_drift_without_write(self):
        arm = SimulatedArm()
        arm.position = 57
        with self.assertRaises(RuntimeError):
            run_jog(arm, 4, 53, 58, 2000)
        self.assertEqual(arm.writes, [])

    def test_rejects_group_id_and_nonfinite_or_large_steps(self):
        for joint, delta in [(0, 1), (7, 1), (1, 6), (1, -6), (1, 0), (1, math.nan), (1, math.inf)]:
            with self.subTest(joint=joint, delta=delta), self.assertRaises(ValueError):
                target_angle(joint, 90, delta)

    def test_keeps_sdk_ranges_and_supports_joint_five(self):
        for joint, current, delta in [(1, 179, 5), (6, 0, -1), (5, 269, 5), (1, -1, 1)]:
            with self.subTest(joint=joint, current=current), self.assertRaises(ValueError):
                target_angle(joint, current, delta)
        self.assertEqual(target_angle(5, 260, 2), 262)

    @patch("arm_control.time.sleep")
    def test_invalid_feedback_never_invokes_write(self, _sleep):
        class OfflineArm:
            def Arm_serial_servo_read(self, joint):
                return None

            def Arm_serial_servo_write(self, *args):
                raise AssertionError("No motion may be requested on a failed read")

        with self.assertRaises(RuntimeError):
            read_angle(OfflineArm(), 1)

    def test_vendor_swallowed_write_failure_is_reported(self):
        class FailedWriter:
            def Arm_serial_servo_write(self, *args):
                print("Arm_serial_servo_write I2C error")

        with self.assertRaises(RuntimeError):
            write_angle(FailedWriter(), 1, 91, 1500)

    @patch("arm_control.time.sleep")
    def test_demo_returns_to_start_and_stops_after_requested_cycles(self, _sleep):
        arm = SimulatedArm()
        run_demo(arm, 6, 53, 50, 1500, 5)
        self.assertEqual(arm.writes, [(6, angle, 1500) for angle in [50, 53] * 5])
        self.assertEqual(arm.position, 53)

    def test_demo_invalid_plans_never_write(self):
        for start, target, duration, cycles in [(53, 47, 1500, 5), (53, 50, 0, 5),
                                                (53, 50, 1500, 0), (53, 50, 1500, 11)]:
            arm = SimulatedArm()
            with self.subTest(cycles=cycles, target=target), self.assertRaises(ValueError):
                run_demo(arm, 6, start, target, duration, cycles)
            self.assertEqual(arm.writes, [])

    @patch("arm_control.time.sleep")
    def test_demo_stops_after_first_failed_motion_without_return_command(self, _sleep):
        class StuckArm(SimulatedArm):
            def Arm_serial_servo_write(self, joint, angle, duration):
                self.writes.append((joint, angle, duration))

        arm = StuckArm()
        with self.assertRaises(RuntimeError):
            run_demo(arm, 6, 53, 50, 1500, 5)
        self.assertEqual(arm.writes, [(6, 50, 1500)])

    def test_demo_refuses_stale_start_before_any_motion(self):
        arm = SimulatedArm()
        arm.position = 60
        with self.assertRaises(RuntimeError):
            run_demo(arm, 6, 53, 50, 1500, 5)
        self.assertEqual(arm.writes, [])

    @patch("arm_control.time.sleep")
    def test_demo_detects_no_progress_even_inside_position_tolerance(self, _sleep):
        class StuckArm(SimulatedArm):
            def Arm_serial_servo_write(self, joint, angle, duration):
                self.writes.append((joint, angle, duration))

        arm = StuckArm()
        with self.assertRaisesRegex(RuntimeError, "反馈没有显示朝目标移动"):
            run_demo(arm, 6, 53, 51, 1500, 5)
        self.assertEqual(len(arm.writes), 1)

    @patch("arm_control.time.sleep")
    def test_group_synchronizes_three_joints_and_returns_without_accumulating(self, _sleep):
        arm = SimulatedGroupArm()
        start = list(arm.positions)
        run_group(arm, start, 3)
        self.assertEqual(arm.writes, [(p, 2500) for p in
                         ([100, 135, 0, 45, 104, 67], start) * 3])
        self.assertEqual(arm.positions, start)

    def test_group_rejects_bad_pose_or_out_of_range_target(self):
        for pose in ([175, 135, 0, 45, 89, 52], [90, 135, 0, 45, 260, 52],
                     [90, 135, 0, 45, 89, 170], [math.nan] * 6, [90] * 5):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                group_target(pose)

    def test_group_rejects_drift_before_first_write(self):
        arm = SimulatedGroupArm()
        start = list(arm.positions)
        arm.positions[2] = 10
        with self.assertRaises(RuntimeError):
            run_group(arm, start, 3)
        self.assertEqual(arm.writes, [])

    @patch("arm_control.time.sleep")
    def test_group_failed_feedback_stops_without_return_motion(self, _sleep):
        class StuckGroup(SimulatedGroupArm):
            def Arm_serial_servo_write6_array(self, pose, duration):
                self.writes.append((list(pose), duration))

        arm = StuckGroup()
        with self.assertRaises(RuntimeError):
            run_group(arm, list(arm.positions), 3)
        self.assertEqual(len(arm.writes), 1)

    def test_group_sdk_error_stops_without_return_motion(self):
        class FailedGroup(SimulatedGroupArm):
            def Arm_serial_servo_write6_array(self, pose, duration):
                self.writes.append((list(pose), duration))
                print("Arm_serial_servo_write6 I2C error")

        arm = FailedGroup()
        with self.assertRaises(RuntimeError):
            run_group(arm, list(arm.positions), 3)
        self.assertEqual(len(arm.writes), 1)


if __name__ == "__main__":
    unittest.main()
