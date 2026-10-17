"""Simulated hardware only; no motor or camera access."""
import unittest
from unittest.mock import patch
from return_to_start import return_steps


class Arm:
    def __init__(self):
        self.pose = [70, 51, 26, 20, 91, 131]
        self.writes = []
    def Arm_serial_servo_read(self, joint):
        return self.pose[joint-1]
    def Arm_serial_servo_write(self, joint, target, duration):
        self.writes.append((joint, self.pose[joint-1], target, duration))
        self.pose[joint-1] = target


class ReturnTests(unittest.TestCase):
    @patch("return_to_start.time.sleep")
    def test_large_return_is_single_joint_small_steps(self, _sleep):
        arm = Arm()
        goal = [83, 122, 37, 17, 91, 131]
        final = return_steps(arm, goal, arm.pose[:], [])
        self.assertTrue(all(abs(a-b) <= 1 for a,b in zip(final, goal)))
        self.assertTrue(all(abs(target-before) <= 3 and duration == 1500 for _,before,target,duration in arm.writes))
        self.assertEqual(arm.writes[0][0], 2)
        self.assertTrue(all(j in range(1,7) for j,_,_,_ in arm.writes))

    @patch("return_to_start.time.sleep")
    def test_stuck_joint_stops_without_resend_or_return(self, _sleep):
        class Stuck(Arm):
            def Arm_serial_servo_write(self, joint, target, duration):
                self.writes.append((joint,target,duration))
        arm = Stuck()
        with self.assertRaisesRegex(RuntimeError, "不重发"):
            return_steps(arm, [83,122,37,17,91,131], arm.pose[:], [])
        self.assertEqual(len(arm.writes), 1)

    def test_invalid_target_and_drift_never_move(self):
        for goal, start in [([83,999,37,17,91,131],[70,51,26,20,91,131]),
                            ([83,122,37,17,91,131],[80,51,26,20,91,131])]:
            arm = Arm()
            with self.assertRaises((RuntimeError, ValueError)):
                return_steps(arm, goal, start, [])
            self.assertEqual(arm.writes, [])

    def test_stop_request_prevents_any_write(self):
        arm = Arm()
        with self.assertRaises(RuntimeError):
            return_steps(arm, [83,122,37,17,91,131], arm.pose[:], [], should_stop=lambda: True)
        self.assertEqual(arm.writes, [])

    @patch("return_to_start.time.sleep")
    def test_uncommanded_joint_motion_aborts(self, _sleep):
        class Loose(Arm):
            def Arm_serial_servo_write(self, joint, target, duration):
                super().Arm_serial_servo_write(joint,target,duration)
                self.pose[2] += 4
        arm = Loose()
        with self.assertRaisesRegex(RuntimeError, "其他关节"):
            return_steps(arm, [83,122,37,17,91,131], arm.pose[:], [])
        self.assertEqual(len(arm.writes), 1)


if __name__ == "__main__":
    unittest.main()
