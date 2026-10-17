"""No real robot or camera access: test teaching/replay guards with stubs."""
import copy
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import trajectory_teach as trajectory


class FakeArm:
    def __init__(self):
        self.pose = [90, 135, 0, 45, 89, 52]
        self.modes = []
        self.torques = []

    def Arm_serial_servo_read(self, joint):
        return self.pose[joint - 1]

    def Arm_Button_Mode(self, enable):
        self.modes.append(enable)

    def Arm_serial_set_torque(self, enable):
        self.torques.append(enable)

    def Arm_serial_servo_write6_array(self, *args):
        raise AssertionError("No position command is allowed during teaching or restoration")


def sample_document():
    return {"schema": "dofbot_joint_teaching_v1", "status": "recorded", "samples": [
        {"t": i / 10, "q": [90 + i, 135, 0, 45, 89, 52]} for i in range(11)
    ]}


class TrajectoryTests(unittest.TestCase):
    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_restore_can_hold_new_pose_then_require_human_confirmation(self, _select, _sleep):
        arm = FakeArm()
        arm.pose = [83, 123, 38, 17, 91, 130]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch", "reference_pose": [89, 166, 18, 0, 91, 29]})
            with patch("trajectory_teach.sys.stdin", io.StringIO("HOLDHERE\nHOLDING\n")):
                self.assertTrue(trajectory.restore_prompt(arm, path))
            self.assertFalse(trajectory.session_active(path))
        self.assertEqual(arm.torques, [1])
        self.assertEqual(arm.modes, [])

    def test_hold_confirmation_cannot_be_fabricated_or_accept_drift(self):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "hold_current_attempt": {"status": "feedback_verified", "after_pose": [80, 120, 30, 10, 90, 100]}})
            with self.assertRaisesRegex(RuntimeError, "真实的现场"):
                trajectory.confirm_hold_current(arm, path)
            with self.assertRaisesRegex(RuntimeError, "反馈已明显变化"):
                trajectory.confirm_hold_current(arm, path, operator_confirmed=True)
            self.assertTrue(trajectory.session_active(path))
        self.assertEqual(arm.torques + arm.modes, [])

    @patch("trajectory_teach.time.sleep")
    def test_hold_current_sends_only_torque_and_keeps_human_confirmation_pending(self, _sleep):
        arm = FakeArm()
        arm.pose = [83, 123, 38, 17, 91, 130]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch", "reference_pose": [89, 166, 18, 0, 91, 29]})
            self.assertEqual(trajectory.hold_current(arm, path), arm.pose)
            state = json.loads(path.read_text())
            self.assertTrue(state["learning_may_be_active"])
            self.assertTrue(state["hold_current_attempt"]["operator_confirmation_pending"])
            self.assertEqual(state["hold_current_attempt"]["status"], "feedback_verified")
            with self.assertRaisesRegex(RuntimeError, "禁止自动重发"):
                trajectory.hold_current(arm, path)
        self.assertEqual(arm.torques, [1])
        self.assertEqual(arm.modes, [])

    @patch("trajectory_teach.time.sleep")
    def test_hold_current_rejects_invalid_or_changing_feedback_before_power(self, _sleep):
        for poses in ([[90, 135, 10, 45, 89, 52], [95, 135, 10, 45, 89, 52]], [RuntimeError("invalid")]):
            arm = FakeArm()
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "state.json"
                trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch"})
                with patch("trajectory_teach.read_pose", side_effect=poses), self.assertRaises(RuntimeError):
                    trajectory.hold_current(arm, path)
            self.assertEqual(arm.torques + arm.modes, [])

    @patch("trajectory_teach.time.sleep")
    def test_hold_current_detects_post_enable_drift_without_more_commands(self, _sleep):
        arm = FakeArm()
        start = [83, 123, 38, 17, 91, 130]
        shifted = [83, 130, 38, 17, 91, 130]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch"})
            with patch("trajectory_teach.read_pose", side_effect=[start] * 3 + [shifted]), self.assertRaisesRegex(RuntimeError, "变化超过2"):
                trajectory.hold_current(arm, path)
            self.assertTrue(trajectory.session_active(path))
        self.assertEqual(arm.torques, [1])

    @patch("trajectory_teach.time.sleep")
    def test_raw_recovery_hint_preserves_negative_angle(self, _sleep):
        class Bus:
            def write_byte_data(self, address, register, value):
                self.joint = register - 0x30
            def read_word_data(self, address, register):
                raw = [1917, 1094, 3141, 2820, 1500, 2499][self.joint - 1]
                return ((raw >> 8) & 255) | ((raw << 8) & 65280)
        hint = trajectory.recovery_pose_hint(SimpleNamespace(bus=Bus()))
        self.assertEqual(hint, [83.2, 164.1, -3.4, 22.9, 91.1, 130.8])

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_raw_hint_never_authorizes_torque_when_sdk_feedback_invalid(self, _select, _sleep):
        class BoundaryArm(FakeArm):
            def Arm_serial_servo_read(self, joint):
                return None if joint == 3 else super().Arm_serial_servo_read(joint)
        arm = BoundaryArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch", "reference_pose": arm.pose[:]})
            with patch("trajectory_teach.recovery_pose_hint", return_value=[90, 135, -3, 45, 89, 52]), \
                    patch("trajectory_teach.sys.stdin", io.StringIO("HOLD\nLEAVE\n")):
                self.assertFalse(trajectory.restore_prompt(arm, path))
            self.assertEqual(arm.torques + arm.modes, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_unrestored_recording_does_not_recommend_replay(self, _select, _sleep):
        arm = FakeArm()
        arm.pose[2] = 10
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(trajectory, "TRAJECTORIES", root / "trajectories"), \
                    patch.object(trajectory, "SESSION", root / "session.json"), \
                    patch("builtins.input", return_value="RELEASE"), \
                    patch("trajectory_teach.sys.stdin", io.StringIO("START\n")), \
                    patch("trajectory_teach.restore_prompt", return_value=False), \
                    patch("sys.stdout", output):
                trajectory.teach(arm, SimpleNamespace(seconds=2, task="mock"))
        self.assertIn("恢复扭矩尚未完成，禁止回放", output.getvalue())
        self.assertNotIn("下一步运行：", output.getvalue())

    def test_retime_preserves_endpoints_and_limits_integer_steps(self):
        plan, scale = trajectory.prepare_plan(sample_document())
        self.assertGreaterEqual(scale, 2)
        self.assertEqual(plan[0], [90, 135, 0, 45, 89, 52])
        self.assertEqual(plan[-1], [100, 135, 0, 45, 89, 52])
        self.assertTrue(all(max(abs(x-y) for x, y in zip(a, b)) <= 2 for a, b in zip(plan, plan[1:])))

    def test_fast_recording_is_slowed_before_replay(self):
        doc = sample_document()
        for i, row in enumerate(doc["samples"]):
            row["q"][0] = 60 + i * 3
        plan, scale = trajectory.prepare_plan(doc)
        self.assertGreaterEqual(scale, 2.9)
        self.assertEqual(plan[-1][0], 90)

    def test_bad_timestamps_angles_and_incomplete_recordings_rejected(self):
        documents = []
        for value in (-1, float("nan"), float("inf"), True):
            doc = sample_document(); doc["samples"][2]["t"] = value; documents.append(doc)
        for value in (-1, 181, float("nan"), True):
            doc = sample_document(); doc["samples"][2]["q"][0] = value; documents.append(doc)
        doc = sample_document(); doc["samples"][2]["t"] = 0.1; documents.append(doc)
        doc = sample_document(); doc["samples"][2]["q"][0] += 20; documents.append(doc)
        doc = sample_document(); doc["status"] = "interrupted"; documents.append(doc)
        for doc in documents:
            with self.subTest(doc=doc), self.assertRaises(ValueError):
                trajectory.prepare_plan(doc)

    def test_swallowed_mode_errors_are_treated_as_unknown(self):
        class FailedArm:
            def Arm_Button_Mode(self, enable):
                print("Arm_Button_Mode I2C error")
        with self.assertRaises(RuntimeError):
            trajectory.learning_command(FailedArm(), 1)

    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_restore_does_not_enable_at_wrong_pose(self, _select):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            arm.pose[0] += 10
            with patch("trajectory_teach.sys.stdin", io.StringIO("HOLD\nLEAVE\n")):
                restored = trajectory.restore_prompt(arm, path)
            self.assertFalse(restored)
            self.assertEqual(arm.modes, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_restore_requires_explicit_hold_and_clears_persistent_marker(self, _select):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            with patch("trajectory_teach.sys.stdin", io.StringIO("HOLD\n")):
                self.assertTrue(trajectory.restore_prompt(arm, path))
            self.assertEqual(arm.modes, [0])
            self.assertFalse(trajectory.session_active(path))

    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_disconnect_never_automatically_enables_torque(self, _select):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            with patch("trajectory_teach.sys.stdin", io.StringIO("")):
                self.assertFalse(trajectory.restore_prompt(arm, path))
            self.assertEqual(arm.modes, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_teaching_reads_angles_without_opening_camera_or_sending_positions(self, _select, _sleep):
        arm = FakeArm()
        arm.pose[2] = 10
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(trajectory, "TRAJECTORIES", root / "trajectories"), \
                    patch.object(trajectory, "SESSION", root / "session.json"), \
                    patch("builtins.input", return_value="RELEASE"), \
                    patch("trajectory_teach.sys.stdin", io.StringIO("START\nHOLD\n")), \
                    patch("trajectory_teach.LatestCamera", side_effect=AssertionError("Camera must stay closed")):
                trajectory.teach(arm, SimpleNamespace(seconds=2, task="mock"))
            self.assertEqual(arm.modes, [])
            self.assertEqual(arm.torques, [0, 1])
            path = next((root / "trajectories").glob("*/trajectory.json"))
            doc = json.loads(path.read_text())
            self.assertEqual(len(doc["samples"]), 20)
            self.assertFalse(doc["camera_recorded"])
            self.assertFalse(trajectory.session_active(root / "session.json"))

    def test_recovery_cancellation_never_changes_modes_or_torque(self):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            with patch("builtins.input", return_value="CANCEL"):
                self.assertFalse(trajectory.recover(arm, path))
            self.assertEqual(arm.modes + arm.torques, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    def test_recovery_exits_legacy_learning_mode_before_retrying_feedback(self, _sleep):
        class LearningBlocksFeedback(FakeArm):
            def Arm_serial_servo_read(self, joint):
                return super().Arm_serial_servo_read(joint) if 0 in self.modes else None

        arm = LearningBlocksFeedback()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            with patch("builtins.input", side_effect=["RECOVER", "HOLDING"]):
                self.assertTrue(trajectory.recover(arm, path))
            self.assertEqual(arm.modes, [0])
            self.assertEqual(arm.torques, [])
            self.assertFalse(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    def test_recovery_failed_feedback_keeps_marker_active(self, _sleep):
        class Offline(FakeArm):
            def Arm_serial_servo_read(self, joint):
                return None

        arm = Offline()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            with patch("builtins.input", return_value="RECOVER"):
                self.assertFalse(trajectory.recover(arm, path))
            self.assertEqual(arm.modes, [0])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    def test_recovery_unconfirmed_hold_keeps_marker_active(self, _sleep):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "release_method": "torque_switch", "reference_pose": arm.pose[:]})
            with patch("builtins.input", side_effect=["RECOVER", "UNSURE"]):
                self.assertFalse(trajectory.recover(arm, path))
            self.assertEqual(arm.torques, [1])
            self.assertEqual(arm.modes, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([], [], []))
    def test_three_failed_restore_reads_stop_retry_loop_without_enabling(self, _select, _sleep):
        class Offline(FakeArm):
            def Arm_serial_servo_read(self, joint):
                return None

        arm = Offline()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            self.assertFalse(trajectory.restore_prompt(arm, path))
            self.assertEqual(arm.modes + arm.torques, [])
            self.assertTrue(trajectory.session_active(path))

    @patch("trajectory_teach.select.select", side_effect=KeyboardInterrupt)
    def test_ctrl_c_exits_restore_without_enabling_or_clearing_marker(self, _select):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            trajectory.save_json(path, {"learning_may_be_active": True, "reference_pose": arm.pose[:]})
            self.assertFalse(trajectory.restore_prompt(arm, path))
            self.assertEqual(arm.modes + arm.torques, [])
            self.assertTrue(trajectory.session_active(path))

    def test_ctrl_c_during_teaching_exits_without_entering_restore_loop(self):
        arm = FakeArm()
        arm.pose[2] = 10
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(trajectory, "TRAJECTORIES", root / "trajectories"), \
                    patch.object(trajectory, "SESSION", root / "session.json"), \
                    patch("builtins.input", return_value="RELEASE"), \
                    patch("trajectory_teach.time.sleep", side_effect=KeyboardInterrupt), \
                    patch("trajectory_teach.restore_prompt", side_effect=AssertionError("Must exit instead of prompting")):
                trajectory.teach(arm, SimpleNamespace(seconds=2, task="mock"))
            self.assertEqual(arm.torques, [0])
            self.assertTrue(trajectory.session_active(root / "session.json"))

    def test_boundary_start_still_requires_explicit_release(self):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(trajectory, "SESSION", root / "session.json"), \
                    patch.object(trajectory, "TRAJECTORIES", root / "trajectories"), \
                    patch("builtins.input", return_value="CANCEL"):
                trajectory.teach(arm, SimpleNamespace(seconds=2, task="mock"))
            doc = json.loads(next((root / "trajectories").glob("*/trajectory.json")).read_text())
            self.assertEqual(doc["near_sdk_boundary_joints"], [3])
            self.assertEqual(doc["status"], "cancelled")
        self.assertEqual(arm.torques + arm.modes, [])

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_preparation_does_not_start_away_from_reference(self, _select, _sleep):
        arm = FakeArm()
        reference = arm.pose[:]
        arm.pose[0] += 10
        with patch("trajectory_teach.sys.stdin", io.StringIO("START\nEND\n")):
            self.assertFalse(trajectory.wait_for_teaching_start(arm, reference))
        self.assertEqual(arm.modes + arm.torques, [])

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([], [], []))
    def test_preparation_stops_on_three_failed_reads(self, _select, _sleep):
        class Offline(FakeArm):
            def Arm_serial_servo_read(self, joint):
                return None
        arm = Offline()
        with self.assertRaisesRegex(RuntimeError, "连续三次读取失败"):
            trajectory.wait_for_teaching_start(arm, arm.pose)
        self.assertEqual(arm.modes + arm.torques, [])

    @patch("trajectory_teach.time.sleep")
    @patch("trajectory_teach.select.select", return_value=([True], [], []))
    def test_boundary_pose_can_be_recorded_with_direct_torque_after_start(self, _select, _sleep):
        arm = FakeArm()
        arm.pose = [90, 165, 18, 0, 89, 29]
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(trajectory, "SESSION", root / "session.json"), \
                    patch.object(trajectory, "TRAJECTORIES", root / "trajectories"), \
                    patch("builtins.input", return_value="RELEASE"), \
                    patch("trajectory_teach.sys.stdin", io.StringIO("START\nHOLD\n")), \
                    patch("trajectory_teach.LatestCamera", side_effect=AssertionError("Camera stays closed")):
                trajectory.teach(arm, SimpleNamespace(seconds=2, task="mock boundary"))
            doc = json.loads(next((root / "trajectories").glob("*/trajectory.json")).read_text())
            self.assertEqual(len(doc["samples"]), 20)
            self.assertEqual(doc["near_sdk_boundary_joints"], [4])
            self.assertFalse(trajectory.session_active(root / "session.json"))
        self.assertEqual(arm.torques, [0, 1])
        self.assertEqual(arm.modes, [])


if __name__ == "__main__":
    unittest.main()
