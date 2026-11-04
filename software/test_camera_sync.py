"""Camera timing/control regressions without opening any device."""
import subprocess
import time
import unittest
from unittest.mock import Mock,patch

from touch_collector.camera import SynchronizedCamera, fixed_rate_control, interval_summary,CameraReadinessError,CameraFrameError


class CameraSyncTests(unittest.TestCase):
    def test_reader_reselects_fresh_frame_after_preemption_at_snapshot_unlock(self):
        now=10_000_000_000;clock=[now];camera=SynchronizedCamera(Mock(),'/dev/not-opened')
        old=Mock();new=Mock();camera.latest=(1,now-10_000_000,old)
        original=camera.condition
        class PreemptOnUnlock:
            calls=0
            def __enter__(self):return original.__enter__()
            def __exit__(self,*args):
                original.__exit__(*args);self.calls+=1
                if self.calls==1:
                    clock[0]+=60_000_000
                    camera.latest=(3,clock[0]-10_000_000,new)
        camera.condition=PreemptOnUnlock()
        with patch('touch_collector.camera.time.monotonic_ns',side_effect=lambda:clock[0]):
            result=camera.read_latest(50)
        self.assertEqual(result,(3,clock[0]-10_000_000,new.copy.return_value))
        # The old reader tested the old 70ms snapshot after the unlock, despite
        # the newer 10ms frame already being available, and raised a false error.
        self.assertEqual((clock[0]-(now-10_000_000))/1e6,70)

    def test_copy_delay_cannot_deliver_stale_frame_and_reselection_is_bounded(self):
        now=10_000_000_000;clock=[now];camera=SynchronizedCamera(Mock(),'/dev/not-opened')
        first=Mock();second=Mock();third=Mock();camera.latest=(1,now-10_000_000,first)
        def delayed_copy(next_frame,seq):
            clock[0]+=60_000_000;camera.latest=(seq,clock[0]-10_000_000,next_frame)
            return object()
        first.copy.side_effect=lambda:delayed_copy(second,3)
        second.copy.side_effect=lambda:delayed_copy(third,5)
        with patch('touch_collector.camera.time.monotonic_ns',side_effect=lambda:clock[0]):
            with self.assertRaises(CameraFrameError) as caught:camera.read_latest(50)
        self.assertEqual(caught.exception.diagnostics['selection_attempt'],2)
        self.assertEqual(caught.exception.diagnostics['frame_age_ms'],70)
        first.copy.assert_called_once();second.copy.assert_called_once();third.copy.assert_not_called()

    def test_frame_rejection_keeps_exact_50ms_gate_and_selected_timestamp(self):
        now=10_000_000_000;camera=SynchronizedCamera(Mock(),'/dev/not-opened');frame=Mock()
        with patch('touch_collector.camera.time.monotonic_ns',return_value=now):
            camera.latest=(7,now-50_000_000,frame)
            self.assertEqual(camera.read_latest(50)[0],7)
            for stamp in [now-50_000_001,now+1]:
                camera.latest=(8,stamp,frame)
                with self.assertRaises(CameraFrameError) as caught:camera.read_latest(50)
                self.assertEqual(caught.exception.diagnostics['frame_receive_monotonic_ns'],stamp)
                self.assertEqual(caught.exception.diagnostics['max_age_ms'],50)
        self.assertEqual(frame.copy.call_count,1)

    def stable_camera(self,now):
        camera=SynchronizedCamera(Mock(),'/dev/not-opened')
        camera.stamps.extend([now-10_000_000-round((59-i)*1e9/30) for i in range(60)])
        return camera

    def test_status_snapshot_does_not_age_while_statistics_are_computed(self):
        now=10_000_000_000;clock=[now];camera=self.stable_camera(now)
        def delayed_summary(stamps):
            # Model preemption after the snapshot while the reader delivers new frames.
            clock[0]+=150_000_000
            camera.stamps.append(clock[0]-10_000_000)
            return interval_summary(stamps)
        with patch('touch_collector.camera.time.monotonic_ns',side_effect=lambda:clock[0]), \
             patch('touch_collector.camera.interval_summary',side_effect=delayed_summary):
            status=camera.status()
        self.assertTrue(status['ready'])
        self.assertEqual(status['latest_frame_age_ms'],10)
        self.assertEqual(status['observed_monotonic_ns'],now)
        # The previous implementation used clock[0] here, falsely producing age=160ms.
        self.assertGreater((clock[0]-(now-10_000_000))/1e6,100)

    def test_genuinely_stale_and_high_p95_frames_still_stop_with_exact_reason(self):
        now=10_000_000_000;camera=self.stable_camera(now)
        with patch('touch_collector.camera.time.monotonic_ns',return_value=now+150_000_000):
            with self.assertRaises(CameraReadinessError) as caught:camera.check_recording()
        self.assertIn('100ms',str(caught.exception))
        self.assertNotIn('未开始',str(caught.exception))
        self.assertEqual(caught.exception.diagnostics['status']['latest_frame_age_ms'],160)
        intervals=[24_000_000,42_900_000]*30
        stamps=[now-sum(intervals)]
        for delta in intervals:stamps.append(stamps[-1]+delta)
        camera.stamps.clear();camera.stamps.extend(stamps)
        with patch('touch_collector.camera.time.monotonic_ns',return_value=now):
            with self.assertRaises(CameraReadinessError) as caught:camera.check_recording()
        self.assertIn('P95',str(caught.exception))
        status=caught.exception.diagnostics['status']
        self.assertLess(status['max_interval_ms'],55)
        self.assertGreater(status['p95_interval_ms'],40)

    def test_runtime_check_preserves_low_rate_gap_and_disconnect_guards(self):
        now=10_000_000_000
        for kind in ('slow','gap','error'):
            camera=self.stable_camera(now)
            if kind=='slow':camera.stamps.clear();camera.stamps.extend([now-(59-i)*67_000_000 for i in range(60)])
            elif kind=='gap':camera.stamps[-2]-=30_000_000
            else:camera.error='USB disconnected'
            with self.subTest(kind=kind),patch('touch_collector.camera.time.monotonic_ns',return_value=now):
                with self.assertRaises(CameraReadinessError):camera.check_recording()

    def read_fixture(self,reads):
        class DecodeError(Exception):pass
        cv=Mock();cv.error=DecodeError
        cap=Mock();cv.VideoCapture.return_value=cap
        cap.isOpened.return_value=True
        camera=SynchronizedCamera(cv,'/dev/not-opened')
        def read():
            item=reads.pop(0)
            if isinstance(item,Exception):raise item
            if not reads:camera.stopping.set()
            return item
        cap.read.side_effect=read
        return camera,cap,DecodeError

    @patch('touch_collector.camera.fixed_rate_control',return_value={})
    def test_empty_startup_frame_retried_without_accepting_old_data(self,control):
        frame=Mock();frame.size=100
        camera,cap,error=self.read_fixture([(False,None),(True,frame)])
        camera._read()
        self.assertEqual(camera.latest[0],0)
        self.assertIs(camera.latest[2],frame)
        self.assertEqual(camera.configuration['startup_discarded_frames'],1)
        self.assertIsNone(camera.error)
        cap.release.assert_called_once()

    @patch('touch_collector.camera.fixed_rate_control',return_value={})
    def test_persistent_empty_startup_is_bounded_and_post_start_fault_is_not_retried(self,control):
        camera,cap,error=self.read_fixture([(False,None)]*8)
        camera._read()
        self.assertEqual(cap.read.call_count,6)
        self.assertIsNotNone(camera.error)
        self.assertIsNone(camera.latest)
        frame=Mock();frame.size=100
        camera,cap,error=self.read_fixture([(True,frame),(False,None)])
        camera._read()
        self.assertEqual(cap.read.call_count,2)
        self.assertIsNotNone(camera.error)

    def test_30hz_is_ready_but_15hz_or_a_stall_is_not(self):
        stamps=[round(i*1e9/30) for i in range(60)]
        self.assertTrue(interval_summary(stamps)['ready'])
        self.assertFalse(interval_summary([i*67_000_000 for i in range(60)])['ready'])
        stamps[-1]+=60_000_000
        self.assertFalse(interval_summary(stamps)['ready'])
        self.assertFalse(interval_summary([0])['ready'])

    @patch('touch_collector.camera.subprocess.run')
    def test_control_only_disables_dynamic_fps_and_verifies_readback(self,run):
        run.side_effect=[subprocess.CompletedProcess([],0,'exposure_dynamic_framerate: 1\n',''),
                         subprocess.CompletedProcess([],0,'',''),
                         subprocess.CompletedProcess([],0,'exposure_dynamic_framerate: 0\n','')]
        result=fixed_rate_control('/dev/video0')
        self.assertEqual((result['before'],result['after']),(1,0))
        writes=[c.args[0] for c in run.call_args_list if '--set-ctrl' in c.args[0]]
        self.assertEqual(writes,[['v4l2-ctl','-d','/dev/video0','--set-ctrl','exposure_dynamic_framerate=0']])

    @patch('touch_collector.camera.subprocess.run')
    def test_unsupported_control_is_reported_not_replaced_with_exposure_changes(self,run):
        run.return_value=subprocess.CompletedProcess([],1,'','unsupported')
        result=fixed_rate_control('/dev/video0')
        self.assertTrue(result['warning'])
        self.assertEqual(run.call_count,1)

    def test_preview_accepts_frame_that_recording_rejects_as_too_old(self):
        camera=SynchronizedCamera(Mock(),'/dev/not-opened')
        frame=Mock()
        camera.latest=(7,time.monotonic_ns()-70_000_000,frame)
        self.assertEqual(camera.read_latest()[0],7)
        with self.assertRaises(RuntimeError):camera.get_for_tick()
        camera.latest=(8,time.monotonic_ns(),frame)
        self.assertEqual(camera.get_for_tick(7)[0],8)
        with self.assertRaises(RuntimeError):camera.get_for_tick(8)

    def test_stale_or_error_status_cannot_pass_capture_preflight(self):
        camera=SynchronizedCamera(Mock(),'/dev/not-opened')
        now=time.monotonic_ns()
        camera.stamps.extend([now-round((59-i)*1e9/30) for i in range(60)])
        self.assertTrue(camera.prepare_recording()['ready'])
        camera.error='camera unplugged'
        with self.assertRaises(RuntimeError):camera.prepare_recording()


if __name__=='__main__':unittest.main()
