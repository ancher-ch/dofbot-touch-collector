"""No real hardware. Verify workflow, actuator gates, tracking and storage invariants."""
import copy
import io
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch, Mock

from touch_collector.core import Engine, home_plan, Stopped, teaching_plan, compensated_teaching
from touch_collector.hardware import DemoRobot, RealRobot, FeedbackError, PacedBus
from touch_collector.storage import Store, Capture, load
from trajectory_teach import save_json


def teaching():
    return {"schema":"dofbot_joint_teaching_v1", "id":"example", "task":"mock task", "status":"recorded",
            "reference_pose":[90,120,35,25,90,30],
            "joint_order":["joint_1","joint_2","joint_3","joint_4","joint_5","gripper"],
            "camera_recorded":False,"contains_motor_actions":False,
            "samples":[{"t":i/10,"q":[90,120,35,25,90+i,30]} for i in range(12)]}


class FakeCV:
    IMWRITE_JPEG_QUALITY = 1
    @staticmethod
    def imwrite(path, frame, options):
        Path(path).write_bytes(b"simulated-test-frame")
        return True


class FakeCamera:
    def __init__(self):
        self.sequence = 0
    def get(self, after=-1):
        self.sequence += 1
        return self.sequence,time.monotonic_ns(),object()
    def close(self):
        pass


class CollectorTests(unittest.TestCase):
    @staticmethod
    def gripping_teaching():
        d=teaching()
        d["samples"]=[{"t":i/10,"q":[90,120,35,25,90,g]} for i,g in
                      enumerate([100,100,110,125,134,134,134,125,110,100,100,100])]
        return d

    def test_gripper_extra_changes_only_replay_copy_and_preserves_widest_opening(self):
        source=self.gripping_teaching();before=copy.deepcopy(source)
        for extra in (1,2):
            adjusted,policy=compensated_teaching(source,extra)
            self.assertEqual(source,before)
            self.assertEqual(min(r["q"][5] for r in adjusted["samples"]),100)
            self.assertEqual(max(r["q"][5] for r in adjusted["samples"]),134+extra)
            self.assertEqual(policy["target_closed_angle"],134+extra)
            for old,new in zip(source["samples"],adjusted["samples"]):
                self.assertEqual(old["q"][:5],new["q"][:5])
                self.assertEqual(old["t"],new["t"])
                self.assertTrue(0<=new["q"][5]-old["q"][5]<=extra)
            self.assertEqual(adjusted["samples"][3]["q"],adjusted["samples"][7]["q"])

    def test_gripper_zero_is_identical_to_existing_plan(self):
        source=self.gripping_teaching()
        self.assertIs(compensated_teaching(source,0)[0],source)
        self.assertEqual(teaching_plan(source),teaching_plan(source,0))

    def test_compensated_gripper_plan_still_caps_steps_and_reaches_closed_target(self):
        for extra in (1,2):
            plan,_=teaching_plan(self.gripping_teaching(),extra)
            self.assertEqual(max(q[5] for q in plan),134+extra)
            self.assertEqual(plan[0][5],100)
            self.assertEqual(plan[-1][5],100)
            self.assertTrue(all(max(abs(a-b) for a,b in zip(x,y))<=2 for x,y in zip(plan,plan[1:])))

    def test_compensation_rejects_invalid_options_and_too_small_open_close_span(self):
        for extra in (-1,3,1.0,True,None,float('nan')):
            with self.subTest(extra=extra),self.assertRaises(ValueError):
                teaching_plan(self.gripping_teaching(),extra)
        source=self.gripping_teaching()
        for row in source["samples"]:row["q"][5]=100
        with self.assertRaisesRegex(ValueError,"不足5"):
            teaching_plan(source,1)
        teaching_plan(source,0)

    def test_compensation_cannot_mask_invalid_source_or_clip_above_sdk_limit(self):
        source=self.gripping_teaching()
        source["samples"][4]["q"][5]=179
        with self.assertRaisesRegex(ValueError,"180"):
            teaching_plan(source,2)
        source["samples"][4]["q"][5]=181
        with self.assertRaises(ValueError):teaching_plan(source,1)

    @patch('touch_collector.core.time.sleep')
    def test_compensated_capture_records_actual_sent_goals_and_unchanged_source(self,_sleep):
        source=self.gripping_teaching();before=copy.deepcopy(source)
        self.e.teaching=source;self.e.holding="confirmed"
        self.robot.pose=source["samples"][0]["q"][:]
        with patch.object(Capture,"encode"):
            self.e.replay_capture(True,gripper_extra_degrees=2)
        rows=[json.loads(x) for x in (self.e.capture.path/"samples.jsonl").read_text().splitlines()]
        self.assertEqual(max(row["action"][5] for row in rows),136)
        self.assertEqual([row["action"] for row in rows if row["command_sent"]],
                         [pose for pose,duration in self.robot.writes])
        self.assertEqual(load(self.e.capture.path/"teaching.json"),before)
        self.assertEqual(self.e.teaching,before)
        policy=load(self.e.capture.path/"metadata.json")["gripper_compensation"]
        self.assertEqual(policy["max_extra_degrees"],2)
        self.assertEqual(policy["target_closed_angle"],136)
        self.assertEqual(self.robot.torques,[])

    @patch('touch_collector.core.time.sleep')
    def test_bad_compensation_fails_before_return_or_capture_creation(self,_sleep):
        self.install_teaching()
        with patch.object(self.e,"return_home") as back:
            with self.assertRaises(ValueError):self.e.replay_capture(True,gripper_extra_degrees=2)
        back.assert_not_called()
        self.assertIsNone(self.e.capture)
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @unittest.skipUnless(os.name == "posix","Uses Linux flock")
    def test_gui_owner_blocks_old_cli_control(self):
        from arm_control import acquire_control_lock
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"lock"
            first = acquire_control_lock(path)
            try:
                with self.assertRaises(RuntimeError):
                    acquire_control_lock(path)
            finally:
                first.close()
            second = acquire_control_lock(path)
            second.close()

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        self.robot = DemoRobot()
        self.e = Engine(self.robot,self.root/"data",self.root/"session.json",FakeCamera(),FakeCV(),True)
    def tearDown(self):
        self.folder.cleanup()
    def install_teaching(self):
        d = teaching()
        p = self.e.store.item("teachings",d["id"])
        p.mkdir()
        self.e.store.write_teaching(d)
        self.e.teaching = d
        self.e.home = d["reference_pose"][:]
        self.e.stage = "saved_teach"
        self.e.holding = "confirmed"
        self.e.persist()

    def test_startup_and_set_home_are_read_only(self):
        self.e.refresh()
        with patch("touch_collector.core.time.sleep"):
            self.e.set_home(True)
        self.assertEqual(self.e.home,self.robot.pose)
        self.assertEqual(self.robot.writes+self.robot.torques,[])
        self.assertEqual(self.e.holding,"unknown")

    def test_every_power_or_motion_entry_requires_explicit_site_confirmation(self):
        for method in (self.e.set_home,self.e.begin_home_setup,self.e.finish_home_setup,self.e.release,self.e.hold_here,self.e.emergency_hold_here,self.e.confirm_holding,self.e.return_home,self.e.replay_capture):
            with self.subTest(method=method.__name__),self.assertRaises(RuntimeError):
                method()
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch("touch_collector.core.time.sleep")
    def test_brief_feedback_loss_keeps_teaching_with_real_timestamps_and_preview(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        self.e.begin_teaching()
        fault = FeedbackError([90,120,None,25,90,30],[None,None,-3,None,None,None])
        with patch.object(self.e,"preview_tick") as preview:
            with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_000_000_000,1_010_000_000]):
                self.e.poll()
            with patch.object(self.robot,"read",side_effect=fault):
                for t in (1_100_000_000,1_200_000_000):
                    with patch("touch_collector.core.time.monotonic_ns",return_value=t):
                        self.e.poll()
            self.assertEqual(self.e.stage,"teaching")
            self.assertEqual(len(self.e.teaching["samples"]),1)
            with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_300_000_000,1_310_000_000]):
                self.e.poll()
            self.assertEqual(preview.call_count,4)
        self.assertEqual([r["t"] for r in self.e.teaching["samples"]],[0,.3])
        self.assertEqual(self.e.feedback_failures,0)
        self.assertEqual(self.e.feedback_error,"")
        self.assertEqual(self.robot.writes,[])
        self.assertEqual(self.robot.torques,[0])

    @patch("touch_collector.core.time.sleep")
    def test_persistent_feedback_loss_preserves_error_draft_and_never_auto_holds(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        self.e.begin_teaching()
        with patch.object(self.robot,"read",side_effect=FeedbackError([90,120,None,25,90,30],[None]*6)):
            for _ in range(3):
                self.e.poll()
        self.assertEqual(self.e.stage,"teach_review")
        self.assertEqual(self.e.teaching["status"],"error")
        self.assertEqual(self.e.holding,"free")
        self.assertEqual(self.robot.torques,[0])
        with self.assertRaises(RuntimeError):
            self.e.save_teaching()
        events=[json.loads(line) for line in (self.e.store.root/"reports/events.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertIn("feedback_invalid",[e["event"] for e in events])
        self.assertIn("operation_failed",[e["event"] for e in events])
        self.assertEqual(load(self.e.store.item("teachings",self.e.teaching["id"])/"trajectory.json")["status"],"error")

    def test_recovery_clears_stale_feedback_command_error_without_auto_retry(self):
        self.e.stage="prepare"
        fault=FeedbackError([90,120,None,25,90,30],[None]*6)
        with patch.object(self.robot,"read",side_effect=fault):
            with self.assertRaises(FeedbackError): self.e.check_feedback()
        self.e.fail(fault)
        self.e.poll()
        self.assertEqual(self.e.error,"")
        self.assertEqual(self.e.feedback_error,"")
        self.assertIsNone(self.e.teaching)
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch("touch_collector.core.time.sleep")
    def test_recovered_feedback_cannot_hide_long_gap(self,_sleep):
        self.e.stage="prepare"
        self.e.holding="free"
        self.e.begin_teaching()
        with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_000_000_000,1_010_000_000]):
            self.e.poll()
        with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_800_000_000,1_810_000_000]):
            with self.assertRaises(RuntimeError): self.e.poll()
        self.assertEqual(len(self.e.teaching["samples"]),1)
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch("touch_collector.core.time.sleep")
    def test_return_then_release_then_teach_can_be_repeated_without_state_dead_end(self,_sleep):
        self.e.set_home(True)
        self.e.confirm_holding(True)
        for _ in range(3):
            self.e.release(True)
            self.e.hold_here(True)
            self.e.confirm_holding(True)
            self.assertEqual(self.e.stage,"ready")
            self.e.return_home(True,countdown=False)
            self.assertEqual(self.e.stage,"ready")
            self.e.release(True)
            self.e.begin_teaching()
            for i in range(11):
                with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_000_000_000+i*100_000_000,1_010_000_000+i*100_000_000]):
                    self.e.teaching_tick()
            self.e.end_teaching()
            self.e.hold_here(True)
            self.e.confirm_holding(True)
            self.e.save_teaching()
            self.e.next_teaching()
            self.assertEqual(self.e.stage,"ready")

    @patch("touch_collector.core.time.sleep")
    def test_hand_teaching_uses_actual_start_despite_four_degree_sag(self,_sleep):
        self.e.set_home(True)
        home=self.e.home[:]
        self.e.release(True)
        self.robot.pose[2]-=4
        self.e.begin_teaching()
        self.e.teaching_tick()
        self.assertEqual(self.e.teaching["reference_pose"],self.robot.pose)
        self.assertEqual(self.e.home,home)
        self.assertEqual(self.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_fast_gripper_is_preserved_reviewed_and_segment_retimed_for_replay(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        self.e.begin_teaching()
        for i in range(12):
            if i==6: self.robot.pose[5]+=30
            with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_000_000_000+i*100_000_000,1_010_000_000+i*100_000_000]):
                self.e.teaching_tick()
        self.assertEqual(len(self.e.teaching["samples"]),12)
        self.assertEqual(len(self.e.teaching["quality_warnings"]),1)
        self.e.end_teaching()
        with self.assertRaises(RuntimeError): self.e.save_teaching()
        self.e.save_teaching(reviewed=True)
        plan,scale=teaching_plan(self.e.teaching)
        self.assertTrue(all(max(abs(a-b) for a,b in zip(x,y))<=2 for x,y in zip(plan,plan[1:])))
        self.assertLess((len(plan)-1)/10,8)
        self.assertEqual(plan[0],self.e.teaching["samples"][0]["q"])
        self.assertEqual(plan[-1],self.e.teaching["samples"][-1]["q"])
        self.assertEqual(self.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_selecting_old_teaching_does_not_replace_global_home(self,_sleep):
        self.install_teaching()
        self.e.home=[80,110,40,20,90,35]
        self.e.select_teaching("example")
        self.assertEqual(self.e.home,[80,110,40,20,90,35])

    @patch("touch_collector.core.time.sleep")
    def test_explicit_emergency_hold_needs_no_feedback_and_never_sends_position(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        self.e.begin_teaching()
        with patch.object(self.robot,"read",side_effect=FeedbackError([90,120,None,25,90,30],[None]*6)):
            self.e.emergency_hold_here(True)
            self.assertEqual(self.robot.torques,[0,1])
            self.assertEqual(self.robot.writes,[])
            self.assertEqual(self.e.holding,"pending")
            self.assertEqual(self.e.teaching["status"],"interrupted")
            with self.assertRaises(FeedbackError):
                self.e.confirm_holding(True)
            with self.assertRaises(RuntimeError):
                self.e.return_home(True)
        session=load(self.e.session_path)
        self.assertTrue(session["learning_may_be_active"])
        self.assertFalse(session["hold_current_attempt"]["feedback_verified"])
        self.e.confirm_holding(True)
        self.assertFalse(load(self.e.session_path)["learning_may_be_active"])

    @patch("touch_collector.core.time.sleep")
    def test_emergency_hold_error_or_repeat_never_clears_uncertainty(self,_sleep):
        with patch.object(self.robot,"torque",side_effect=RuntimeError("I2C error")):
            with self.assertRaises(RuntimeError):
                self.e.emergency_hold_here(True)
        self.assertEqual(self.e.holding,"pending")
        self.assertEqual(load(self.e.session_path)["hold_current_attempt"]["status"],"command_result_unknown")
        with self.assertRaises(RuntimeError):
            self.e.emergency_hold_here(True)
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])

    @patch("touch_collector.core.time.sleep")
    def test_emergency_hold_cancels_home_setup_without_overwriting_home(self,_sleep):
        self.e.set_home(True)
        old=self.e.home[:]
        self.e.begin_home_setup(True)
        self.robot.pose[1]+=8
        self.e.emergency_hold_here(True)
        self.assertEqual(self.e.home,old)
        self.assertEqual(self.e.stage,"ready")
        self.assertEqual(self.e.holding,"pending")

    def test_emergency_hold_refuses_old_learning_mode(self):
        self.e._session(learning_may_be_active=True,release_method="learning_mode")
        with self.assertRaises(RuntimeError):
            self.e.emergency_hold_here(True)
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch("touch_collector.core.time.sleep")
    def test_pending_out_of_range_can_re_release_without_positions_or_fake_feedback(self,_sleep):
        self.e.set_home(True)
        home=self.e.home[:]
        self.e.release(True)
        fault=FeedbackError([89,107,None,28,87,97],[None,None,-4.4,None,None,None])
        with patch.object(self.robot,"read",side_effect=fault):
            self.e.emergency_hold_here(True)
            with self.assertRaises(FeedbackError): self.e.refresh()
            self.e.recover_manual(True)
            self.assertFalse(self.e.valid)
            self.assertIsNone(self.e.pose[2])
            with self.assertRaises(FeedbackError): self.e.begin_teaching()
            with self.assertRaises(RuntimeError): self.e.return_home(True)
        self.assertEqual(self.robot.torques,[0,1,0])
        self.assertEqual(self.robot.writes,[])
        self.assertEqual(self.e.home,home)
        self.assertEqual(self.e.stage,"prepare")
        self.assertEqual(self.e.holding,"free")
        self.assertIsNone(load(self.e.session_path)["hold_current_attempt"])
        restarted=Engine(self.robot,self.e.store.root,self.e.session_path)
        self.assertEqual(restarted.holding,"free")
        self.e.hold_here(True)
        self.e.confirm_holding(True)
        self.e.release(True)
        self.e.begin_teaching()
        self.assertEqual(self.e.stage,"teaching")

    def test_manual_recovery_requires_support_consent_and_known_torque_mode(self):
        self.e._session(release_method="torque_switch")
        for consent in (False,None,1):
            with self.assertRaises(RuntimeError): self.e.recover_manual(consent)
        self.e._session(release_method="learning_mode")
        with self.assertRaises(RuntimeError): self.e.recover_manual(True)
        self.assertEqual(self.robot.torques+self.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_manual_recovery_no_repeat_when_already_free(self,_sleep):
        self.e.set_home(True);self.e.release(True)
        with self.assertRaises(RuntimeError): self.e.recover_manual(True)
        self.assertEqual(self.robot.torques,[0])

    def test_manual_recovery_cannot_interrupt_motion_or_teaching(self):
        self.e._session(release_method="torque_switch")
        for activity,stage in (("returning","ready"),("replaying","ready"),("idle","teaching")):
            self.e.activity=activity;self.e.stage=stage
            with self.assertRaises(RuntimeError): self.e.recover_manual(True)
        self.assertEqual(self.robot.torques+self.robot.writes,[])

    def test_manual_recovery_failed_disk_never_releases(self):
        self.e._session(release_method="torque_switch")
        self.e.holding="pending"
        with patch("touch_collector.core.save_json",side_effect=OSError("disk")):
            with self.assertRaises(OSError): self.e.recover_manual(True)
        self.assertEqual(self.e.holding,"pending")
        self.assertEqual(self.robot.torques,[])

    def test_manual_recovery_failed_io_keeps_uncertain_free_marker_after_restart(self):
        self.e._session(release_method="torque_switch")
        self.e.holding="pending"
        with patch.object(self.robot,"torque",side_effect=OSError("bus")):
            with self.assertRaises(OSError): self.e.recover_manual(True)
        state=load(self.e.session_path)
        self.assertTrue(state["learning_may_be_active"])
        self.assertEqual(state["manual_recovery"]["status"],"command_result_unknown")
        self.assertEqual(Engine(self.robot,self.e.store.root,self.e.session_path).holding,"free")

    @patch("touch_collector.core.time.sleep")
    def test_manual_recovery_keeps_error_draft_and_old_home(self,_sleep):
        self.e.set_home(True);home=self.e.home[:]
        self.e.release(True);self.e.begin_teaching()
        self.e.fail(RuntimeError("invalid joint"))
        self.e.emergency_hold_here(True)
        teaching_id=self.e.teaching["id"]
        self.e.recover_manual(True)
        self.assertEqual(self.e.stage,"teach_review")
        self.assertEqual(self.e.teaching["status"],"error")
        self.assertEqual(self.e.teaching["id"],teaching_id)
        with self.assertRaises(RuntimeError): self.e.save_teaching()
        self.assertEqual(self.e.home,home)
        self.assertEqual(self.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_manual_recovery_cancels_pending_home_setup_without_saving(self,_sleep):
        self.e.set_home(True);home=self.e.home[:]
        self.e.begin_home_setup(True)
        self.robot.pose[2]+=10
        self.e.finish_home_setup(True)
        self.e.recover_manual(True)
        self.assertEqual(self.e.home,home)
        self.assertIsNone(self.e.home_candidate)
        self.assertEqual(self.e.stage,"prepare")
        self.assertEqual(self.robot.writes,[])

    @patch("touch_collector.hardware.time.sleep")
    def test_real_adapter_retries_single_bad_reply_without_raw_or_motor_commands(self,_sleep):
        robot=RealRobot.__new__(RealRobot)
        robot.arm=Mock()
        robot.arm.Arm_serial_servo_read.side_effect=[90,120,None,35,25,90,30]
        self.assertEqual(robot.read(),[90,120,35,25,90,30])
        self.assertEqual(robot.arm.Arm_serial_servo_read.call_count,7)
        self.assertEqual(robot.arm.bus.mock_calls,[])
        robot.arm.Arm_serial_set_torque.assert_not_called()

    @patch("touch_collector.hardware.time.sleep")
    def test_real_adapter_identifies_out_of_range_joint_without_clamping(self,_sleep):
        robot=RealRobot.__new__(RealRobot)
        robot.arm=Mock(addr=0x15)
        robot.arm.Arm_serial_servo_read.side_effect=lambda j: None if j==3 else [90,120,35,25,90,30][j-1]
        raw=3160
        robot.arm.bus.read_word_data.return_value=((raw&255)<<8)|(raw>>8)
        with self.assertRaises(FeedbackError) as caught:
            robot.read()
        self.assertIsNone(caught.exception.pose[2])
        self.assertLess(caught.exception.hints[2],0)
        self.assertIn("超出SDK范围",caught.exception.reasons[2])
        robot.arm.bus.write_byte_data.assert_called_once_with(0x15,0x33,0)
        robot.arm.Arm_serial_set_torque.assert_not_called()

    @patch("touch_collector.core.time.sleep")
    def test_storage_failure_during_abort_cannot_leave_recording_state_active(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        self.e.begin_teaching()
        with patch.object(self.e.store,"write_teaching",side_effect=OSError("disk full")):
            self.e.fail(RuntimeError("sampling error"))
        self.assertEqual(self.e.stage,"teach_review")
        self.assertEqual(self.e.teaching["status"],"error")
        self.assertIn("写入失败",self.e.message)
        self.assertEqual(self.robot.torques,[0])

    @patch("touch_collector.core.time.sleep")
    def test_first_home_can_be_hand_positioned_without_existing_home(self,_sleep):
        self.e.begin_home_setup(True)
        self.assertEqual(self.e.stage,"home_setup")
        self.assertIsNone(self.e.home)
        self.assertEqual(self.robot.torques,[0])
        self.robot.pose = [83,117,39,30,91,99]
        self.e.finish_home_setup(True)
        self.assertIsNone(self.e.home)
        self.assertFalse((self.e.store.root/"home.json").exists())
        self.assertEqual(self.e.holding,"pending")
        self.e.confirm_holding(True)
        self.assertEqual(self.e.home,self.robot.pose)
        self.assertEqual(load(self.e.store.root/"home.json")["pose"],self.robot.pose)
        self.assertEqual(self.e.stage,"ready")
        self.assertEqual(self.e.holding,"confirmed")
        self.assertEqual(self.robot.torques,[0,1])
        self.assertEqual(self.robot.writes,[])
        self.assertIsNone(self.e.teaching)
        self.assertIsNone(self.e.capture)
        self.assertFalse(list(self.e.store.root.rglob("*.jpg")))

    @patch("touch_collector.core.time.sleep")
    def test_cancel_home_keeps_old_home_and_does_not_change_torque(self,_sleep):
        self.e.set_home(True)
        original = load(self.e.store.root/"home.json")
        self.e.begin_home_setup(True)
        self.robot.pose[1] += 6
        self.e.cancel_home_setup()
        self.assertEqual(load(self.e.store.root/"home.json"),original)
        self.assertEqual(self.e.home,original["pose"])
        self.assertEqual(self.e.holding,"free")
        self.assertEqual(self.robot.torques,[0])
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])

    @patch("touch_collector.core.time.sleep")
    def test_invalid_home_feedback_never_overwrites_old_home_or_sends_hold(self,_sleep):
        self.e.set_home(True)
        original = load(self.e.store.root/"home.json")
        self.e.begin_home_setup(True)
        with patch.object(self.robot,"read",side_effect=FeedbackError([90,120,None,25,90,30],[None]*6)):
            with self.assertRaises(FeedbackError):
                self.e.finish_home_setup(True)
        self.assertEqual(load(self.e.store.root/"home.json"),original)
        self.assertEqual(self.robot.torques,[0])
        self.assertEqual(self.e.stage,"home_setup")
        self.assertIsNone(self.e.home_candidate)

    @patch("touch_collector.core.time.sleep")
    def test_home_hold_error_remains_pending_and_keeps_old_home(self,_sleep):
        self.e.set_home(True)
        old = self.e.home[:]
        self.e.begin_home_setup(True)
        with patch.object(self.robot,"torque",side_effect=RuntimeError("I2C error")):
            with self.assertRaises(RuntimeError):
                self.e.finish_home_setup(True)
        self.assertEqual(self.e.holding,"pending")
        self.assertEqual(self.e.home,old)
        self.assertIsNone(self.e.home_candidate)
        with self.assertRaises(RuntimeError):
            self.e.confirm_holding(True)
        self.e.cancel_home_setup()
        self.assertEqual(self.e.holding,"pending")
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])

    @patch("touch_collector.core.time.sleep")
    def test_home_drift_or_repeat_hold_cannot_commit_candidate(self,_sleep):
        self.e.begin_home_setup(True)
        self.e.finish_home_setup(True)
        with self.assertRaises(RuntimeError):
            self.e.finish_home_setup(True)
        self.robot.pose[1] += 4
        with self.assertRaises(RuntimeError):
            self.e.confirm_holding(True)
        self.assertIsNone(self.e.home)
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])
        self.assertEqual(self.robot.torques,[0,1])

    @patch("touch_collector.core.time.sleep")
    def test_home_setup_blocks_other_workflows_and_preserves_saved_teaching(self,_sleep):
        self.install_teaching()
        original = copy.deepcopy(self.e.teaching)
        self.e.begin_home_setup(True)
        for method in (self.e.set_home,self.e.begin_home_setup,self.e.release,self.e.return_home,self.e.replay_capture):
            with self.assertRaises(RuntimeError):
                method(True)
        for method in (self.e.begin_teaching,self.e.save_teaching,self.e.discard_teaching,self.e.next_teaching):
            with self.assertRaises(RuntimeError):
                method()
        self.e.finish_home_setup(True)
        self.e.confirm_holding(True)
        self.assertEqual(self.e.stage,"saved_teach")
        self.assertEqual(self.e.teaching,original)
        self.assertTrue(self.e.store.item("teachings",original["id"]).exists())

    @patch("touch_collector.core.time.sleep")
    def test_restart_during_home_setup_preserves_old_home_and_free_marker(self,_sleep):
        self.e.set_home(True)
        old = self.e.home[:]
        self.e.begin_home_setup(True)
        self.robot.pose[1] += 6
        self.e.finish_home_setup(True)
        other = Engine(DemoRobot(),self.e.store.root,self.e.session_path,demo=True)
        self.assertEqual(other.home,old)
        self.assertEqual(other.holding,"pending")
        self.assertEqual(other.robot.torques+other.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_home_setup_refuses_active_teaching_and_pending_holding(self,_sleep):
        self.e.stage = "teaching"
        with self.assertRaises(RuntimeError):
            self.e.begin_home_setup(True)
        self.e.stage = "ready"
        self.e.holding = "pending"
        with self.assertRaises(RuntimeError):
            self.e.begin_home_setup(True)
        self.assertEqual(self.robot.torques+ self.robot.writes,[])

    @patch("touch_collector.core.time.sleep")
    def test_hold_here_does_not_return_old_pose_or_clear_marker_without_human(self,_sleep):
        self.e.holding = "free"
        self.e.hold_here(True)
        self.assertEqual(self.robot.writes,[])
        self.assertEqual(self.robot.torques,[1])
        self.assertEqual(self.e.holding,"pending")
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])
        with self.assertRaises(RuntimeError):
            self.e.hold_here(True)
        self.e.confirm_holding(True)
        self.assertFalse(load(self.e.session_path)["learning_may_be_active"])
        self.assertFalse(load(self.e.session_path)["hold_current_attempt"]["operator_confirmation_pending"])

    @patch("touch_collector.core.time.sleep")
    def test_manual_teaching_records_no_camera_frames_or_position_writes(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        with patch.object(self.e.camera,"get",side_effect=AssertionError("No camera calls during teaching ticks")):
            self.e.begin_teaching()
            for i in range(12):
                with patch("touch_collector.core.time.monotonic_ns",side_effect=[1_000_000_000+i*100_000_000,1_010_000_000+i*100_000_000]):
                    self.e.teaching_tick()
            self.e.end_teaching()
        self.assertEqual(len(self.e.teaching["samples"]),12)
        self.assertFalse(self.e.teaching["camera_recorded"])
        self.assertEqual(self.robot.writes,[])
        self.assertEqual(self.robot.torques,[0])
        self.assertFalse(list(self.e.store.root.rglob("*.jpg")))
        self.e.save_teaching()
        self.assertEqual(self.e.teaching["status"],"recorded")

    @patch("touch_collector.core.time.sleep")
    def test_invalid_feedback_preserves_release_marker_and_cannot_move(self,_sleep):
        self.e.set_home(True)
        self.e.release(True)
        with patch.object(self.robot,"read",side_effect=FeedbackError([90,120,None,25,90,30],[None,None,-3,None,None,None])):
            with self.assertRaises(FeedbackError):
                self.e.hold_here(True)
        self.assertEqual(self.robot.torques,[0])
        self.assertTrue(load(self.e.session_path)["learning_may_be_active"])
        self.assertEqual(self.robot.writes,[])

    def test_five_second_plan_is_smooth_and_refuses_large_span(self):
        plan = home_plan([70,51,26,20,90,131],[83,122,37,17,91,131])
        self.assertEqual(plan[0],[70,51,26,20,90,131])
        self.assertEqual(plan[-1],[83,122,37,17,91,131])
        self.assertEqual((len(plan)-1)/10,4.5)
        self.assertTrue(all(max(abs(a-b) for a,b in zip(x,y))<=4 for x,y in zip(plan,plan[1:])))
        with self.assertRaises(ValueError):
            home_plan([0]*6,[180,180,180,180,270,180])

    @patch("touch_collector.core.time.sleep")
    def test_stop_event_prevents_first_motion_and_never_drops_torque(self,_sleep):
        self.e.stop.set()
        with self.assertRaises(Stopped):
            self.e._motion([self.robot.pose[:],[90,121,35,25,90,30]])
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch("touch_collector.core.time.sleep")
    def test_tracking_failure_stops_before_whole_trajectory(self,_sleep):
        with patch.object(self.robot,"write",side_effect=lambda p,d:self.robot.writes.append((p,d))):
            plan = [[90,120+i,35,25,90,30] for i in range(15)]
            with self.assertRaises(RuntimeError):
                self.e._motion(plan)
        self.assertLess(len(self.robot.writes),len(plan))
        self.assertEqual(self.robot.torques,[])

    def test_controller_lag_is_checked_against_recent_commands_not_instant_ideal(self):
        class Clock:
            t=0.
            def sleep(self,seconds): self.t+=max(0,seconds)
        clock=Clock()
        initial=self.robot.pose[:]
        commands=[(-1.,initial[:])]
        def read():
            return next(q[:] for stamp,q in reversed(commands) if stamp<=clock.t-.25)
        def write(q,ms): commands.append((clock.t,q[:]))
        target=[60,80,40,35,95,40]
        plan=home_plan(initial,target)
        with patch("touch_collector.core.time.monotonic",side_effect=lambda:clock.t),patch("touch_collector.core.time.monotonic_ns",side_effect=lambda:int(clock.t*1e9)),patch("touch_collector.core.time.sleep",side_effect=clock.sleep),patch.object(self.robot,"read",side_effect=read),patch.object(self.robot,"write",side_effect=write):
            self.e._motion(plan)
        self.assertEqual(commands[-1][1],target)
        self.assertEqual(self.e.pose,target)

    @patch("touch_collector.core.time.sleep")
    def test_repeated_targets_do_not_flood_bus_but_each_frame_is_recorded(self,_sleep):
        capture=Mock()
        self.e._motion([self.robot.pose[:]]*12,capture)
        self.assertEqual(len(self.robot.writes),1)
        self.assertEqual(capture.add.call_count,12)
        self.assertEqual(sum(c.kwargs["command_sent"] for c in capture.add.call_args_list),1)
        self.assertTrue(all(c.args[5:7]==(None,None) for c in capture.add.call_args_list[1:]))

    @patch('touch_collector.core.time.sleep')
    def test_unstable_camera_preflight_does_not_start_return_motion(self,_sleep):
        self.install_teaching()
        self.e.camera.prepare_recording=Mock(side_effect=RuntimeError('camera warming'))
        with self.assertRaisesRegex(RuntimeError,'camera warming'):
            self.e.replay_capture(True)
        self.assertEqual(self.robot.writes+self.robot.torques,[])
        self.assertIsNone(self.e.capture)

    @patch('touch_collector.core.time.sleep')
    def test_synchronized_camera_rejects_old_image_before_first_write(self,_sleep):
        camera=FakeCamera()
        camera.max_recording_age_ms=50
        camera.recording_anchor=lambda:time.monotonic()
        camera.get_for_tick=lambda after:(1,time.monotonic_ns()-70_000_000,object())
        self.e.camera=camera
        with self.assertRaisesRegex(RuntimeError,'相机帧过旧'):
            self.e._motion([self.robot.pose[:]]*2,Mock())
        self.assertEqual(self.robot.writes+self.robot.torques,[])

    @patch('touch_collector.core.time.sleep')
    def test_irregular_camera_interval_stops_before_the_next_goal(self,_sleep):
        camera=FakeCamera()
        camera.recording_anchor=lambda:time.monotonic()
        camera.max_recording_interval_error_ms=20
        origin=time.monotonic_ns()
        samples=iter([(1,origin,object()),(2,origin+133_000_000,object())])
        camera.get_for_tick=lambda after:next(samples)
        self.e.camera=camera
        capture=Mock()
        with self.assertRaisesRegex(RuntimeError,'相机采样间隔异常'):
            self.e._motion([self.robot.pose[:]]*2,capture)
        self.assertEqual(len(self.robot.writes),1)
        self.assertEqual(capture.add.call_count,1)

    def test_capture_held_target_has_no_fabricated_write_timestamps(self):
        self.install_teaching()
        c=Capture(self.e.store,self.e.teaching,FakeCV())
        now=time.monotonic_ns()
        p=self.robot.pose[:]
        c.add((1,now-10,object()),p,p,now,now+1,now+2,now+3,1)
        c.add((2,now+10,object()),p,p,now+20,now+21,None,None,1.1,command_sent=False)
        with self.assertRaises(RuntimeError):
            c.add((3,now+25,object()),p,[91,120,35,25,90,30],now+30,now+31,None,None,1.2,command_sent=False)
        c.finish("draft")
        rows=[json.loads(s) for s in (c.path/'samples.jsonl').read_text().splitlines()]
        self.assertFalse(rows[1]["command_sent"])
        self.assertIsNone(rows[1]["action_write_start_monotonic_ns"])
        self.assertEqual(rows[1]["active_command_write_start_monotonic_ns"],rows[0]["action_write_start_monotonic_ns"])
        self.assertEqual(c.meta["commands_sent"],1)

    @patch("touch_collector.core.time.sleep")
    def test_ui_wakeup_between_ticks_skips_near_duplicate_observation_without_abort(self,_sleep):
        self.e.set_home(True);self.e.release(True);self.e.begin_teaching()
        for t in (1_000_000_000,1_015_000_000,1_100_000_000):
            with patch("touch_collector.core.time.monotonic_ns",side_effect=[t,t+1_000_000]):
                self.e.teaching_tick()
        self.assertEqual(len(self.e.teaching["samples"]),2)
        self.assertEqual(self.e.stage,"teaching")

    def test_i2c_pacing_separates_transactions_and_preserves_raw_results(self):
        class Clock:
            t=1.
            def sleep(self,n): self.t+=n
        clock=Clock();times=[]
        raw=Mock()
        raw.write_byte_data.side_effect=lambda *a:times.append(clock.t)
        def read(*args): times.append(clock.t);return 1234
        raw.read_word_data.side_effect=read
        bus=PacedBus(raw)
        with patch("touch_collector.hardware.time.monotonic",side_effect=lambda:clock.t),patch("touch_collector.hardware.time.sleep",side_effect=clock.sleep):
            bus.write_byte_data(0x15,0x31,0)
            self.assertEqual(bus.read_word_data(0x15,0x31),1234)
            bus.write_byte_data(0x15,0x32,0)
        self.assertTrue(all(b-a>=.00299 for a,b in zip(times,times[1:])))
        self.assertEqual(bus.transactions,3)

    def test_delete_is_reversible_and_never_changes_hardware(self):
        self.install_teaching()
        p = self.e.store.item("teachings","example")
        self.e.discard_teaching()
        self.assertFalse(p.exists())
        self.assertEqual(len(list((self.e.store.root/"trash").iterdir())),1)
        self.assertEqual(self.robot.writes+self.robot.torques,[])
        for bad in ("../other","/etc","a/b"):
            with self.assertRaises(ValueError):
                self.e.store.trash("teachings",bad)

    def test_frames_and_commands_share_indices_and_stale_frame_is_rejected(self):
        self.install_teaching()
        c = Capture(self.e.store,self.e.teaching,FakeCV())
        now = time.monotonic_ns()
        c.add((1,now-10_000_000,object()),self.robot.pose,self.robot.pose,now,now+1,now+2,now+3,1)
        c.finish("draft")
        row = json.loads((c.path/"samples.jsonl").read_text())
        self.assertEqual(row["frame_index"],0)
        self.assertEqual(row["action"],self.robot.pose)
        self.assertTrue((c.path/row["image"]).exists())
        c2 = Capture(self.e.store,self.e.teaching,FakeCV())
        with self.assertRaises(RuntimeError):
            c2.add((2,now-300_000_000,object()),self.robot.pose,self.robot.pose,now,now+1,now+2,now+3,1)
        c2.finish("error")

    def test_only_complete_capture_can_be_saved_and_success_is_not_inferred(self):
        self.install_teaching()
        self.e.capture = Capture(self.e.store,self.e.teaching,FakeCV())
        self.e.capture.finish("error")
        with self.assertRaises(RuntimeError):
            self.e.save_capture(True)
        self.e.capture.meta.update(status="draft",capture_checks_passed=True)
        self.e.save_capture(None)
        d = load(self.e.capture.path/"metadata.json")
        self.assertEqual(d["status"],"saved")
        self.assertIsNone(d["operator_success"])
        self.assertFalse(d["train_ready"])

    def test_restart_restores_draft_and_never_enables_or_disables_torque(self):
        self.install_teaching()
        self.e.capture = Capture(self.e.store,self.e.teaching,FakeCV())
        self.e.capture.finish("draft")
        self.e.persist()
        robot = DemoRobot()
        other = Engine(robot,self.e.store.root,self.e.session_path,FakeCamera(),FakeCV(),True)
        self.assertEqual(other.stage,"capture_review")
        self.assertEqual(other.capture.id,self.e.capture.id)
        self.assertEqual(robot.writes+robot.torques,[])

    def test_saved_episodes_survive_teaching_deletion(self):
        self.install_teaching()
        c = Capture(self.e.store,self.e.teaching,FakeCV())
        c.finish("saved")
        self.e.capture = c
        self.e.discard_teaching()
        self.assertTrue(c.path.exists())
        self.assertTrue((c.path/"teaching.json").exists())

    def test_next_or_select_cannot_silently_discard_unsaved_capture(self):
        self.install_teaching()
        self.e.capture = Capture(self.e.store,self.e.teaching,FakeCV())
        self.e.capture.finish("draft")
        with self.assertRaises(RuntimeError):
            self.e.next_teaching()
        with self.assertRaises(RuntimeError):
            self.e.select_teaching("example")


if __name__ == "__main__":
    unittest.main()
