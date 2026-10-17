"""Camera timing/control regressions without opening any device."""
import subprocess
import time
import unittest
from unittest.mock import Mock,patch

from touch_collector.camera import SynchronizedCamera, fixed_rate_control, interval_summary


class CameraSyncTests(unittest.TestCase):
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
