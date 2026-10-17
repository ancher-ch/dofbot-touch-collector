"""Emergency latch, camera pairing and non-destructive dataset operations."""
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile
from touch_collector.safety import EmergencyLatch
from touch_collector.core import Engine
from touch_collector.hardware import DemoRobot
from touch_collector.multicamera import CameraRig,ViewSample,validate_config
from touch_collector.management import DatasetManager
from touch_collector.storage import Capture,Store,load
from trajectory_teach import save_json
from test_touch_collector import FakeCV,FakeCamera,teaching

class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.robot=DemoRobot();self.engine=Engine(self.robot,self.root,self.root/'session.json')
    def tearDown(self):self.tmp.cleanup()
    def test_trip_blocks_position_and_torque_at_adapter_and_survives_restart(self):
        self.engine.safety.trip()
        for call in (lambda:self.robot.write(self.robot.pose,100),lambda:self.robot.torque(1),lambda:self.robot.torque(0),lambda:self.engine.begin_teaching()):
            with self.assertRaises(RuntimeError):call()
        self.assertEqual(self.robot.writes+self.robot.torques,[])
        self.assertTrue(EmergencyLatch(self.root/'safety.json').tripped.is_set())
    def test_reset_needs_human_and_idle_and_never_actuates(self):
        self.engine.safety.trip()
        with self.assertRaises(RuntimeError):self.engine.reset_emergency()
        self.engine.activity='returning'
        with self.assertRaises(RuntimeError):self.engine.reset_emergency(True)
        self.engine.activity='idle';self.engine.holding='free'
        self.engine.reset_emergency(True)
        self.assertFalse(self.engine.safety.tripped.is_set());self.assertEqual(self.engine.holding,'free')
        self.assertEqual(self.robot.writes+self.robot.torques,[])
    def test_corrupt_state_fails_closed_and_storage_error_does_not_unlock(self):
        path=self.root/'broken.json';path.write_text('{')
        self.assertTrue(EmergencyLatch(path).tripped.is_set())
        with patch('touch_collector.safety.save_json',side_effect=OSError('disk')):
            self.engine.safety.trip();self.assertTrue(self.engine.safety.tripped.is_set())
            with self.assertRaises(OSError):self.engine.reset_emergency(True)
        self.assertTrue(self.engine.safety.tripped.is_set())
    def test_reset_during_supported_initial_setup_cannot_deadlock_or_save_home(self):
        self.engine.stage='home_setup';self.engine.holding='free';self.engine.home=[90,120,35,25,90,30]
        self.engine.safety.trip()
        self.engine.reset_emergency(True)
        self.assertFalse(self.engine.safety.tripped.is_set())
        self.assertEqual(self.engine.stage,'home_setup');self.assertEqual(self.engine.holding,'free')
        self.assertFalse((self.root/'home.json').exists())
        self.assertEqual(self.robot.writes+self.robot.torques,[])
    def test_trip_during_motion_prevents_later_targets_and_does_not_toggle_torque(self):
        normal=self.robot.write
        def write(q,ms):normal(q,ms);self.engine.safety.trip('injected stop')
        with patch.object(self.robot,'write',side_effect=write),patch('touch_collector.core.time.sleep'):
            with self.assertRaises(RuntimeError):self.engine._motion([self.robot.pose[:],[91,120,35,25,90,30]])
        self.assertEqual(len(self.robot.writes),1);self.assertEqual(self.robot.torques,[])
    def test_stop_while_free_preserves_session_and_no_automatic_holding(self):
        self.engine.holding='free';save_json(self.root/'session.json',{'learning_may_be_active':True,'release_method':'torque_switch'})
        self.engine.safety.trip();before=(self.root/'session.json').read_bytes()
        self.engine.poll()
        self.assertEqual((self.root/'session.json').read_bytes(),before)
        self.assertEqual(self.robot.torques,[])

class PairCamera:
    def __init__(self,seq,stamp):self.sample=(seq,stamp,object());self.error=None
    def read_latest(self,age):return self.sample
    def status(self):return {'ready':not self.error,'fps':30,'error':self.error}
    def prepare_recording(self):
        if self.error:raise RuntimeError(self.error)
    def close(self):pass

class CameraTests(unittest.TestCase):
    def make_rig(self):
        now=time.monotonic_ns();self.now=now
        self.a=PairCamera(10,now-10_000_000);self.b=PairCamera(20,now-20_000_000)
        return CameraRig(None,{'wrist':self.a,'overview':self.b},{'mode':'dual','wrist':'/dev/video0','overview':'/dev/video2'})
    def test_unique_devices_and_missing_second_rejected_before_open(self):
        devices=[{'path':'/dev/video0'}]
        with self.assertRaises(ValueError):validate_config({'mode':'dual','wrist':'/dev/video0','overview':'/dev/video0'},devices)
        with self.assertRaises(ValueError):validate_config({'mode':'dual','wrist':'/dev/video0','overview':'/dev/video2'},devices)
        self.assertEqual(validate_config({'mode':'single','wrist':'/dev/video0'},devices)['mode'],'single')
    def test_pair_preserves_each_sequence_timestamp_and_primary_compatibility(self):
        rig=self.make_rig();sample=rig.get_for_tick()
        self.assertEqual(tuple(sample),self.a.sample);self.assertEqual(sample.views['overview'],self.b.sample)
        with self.assertRaisesRegex(RuntimeError,'没有新帧'):rig.get_for_tick()
    def test_stale_or_skewed_or_disconnected_secondary_cannot_record(self):
        for delay in (48_000_000,100_000_000):
            rig=self.make_rig();self.b.sample=(20,self.now-delay,object())
            with self.assertRaises(RuntimeError):rig.get_for_tick()
        rig=self.make_rig();self.b.error='disconnected'
        with self.assertRaises(RuntimeError):rig.prepare_recording()
        self.assertFalse(rig.status()['ready'])
    def test_secondary_interval_gate_does_not_hide_missing_frames(self):
        rig=self.make_rig();rig.get_for_tick()
        self.a.sample=(11,self.now+190_000_000,object());self.b.sample=(21,self.now+190_000_000,object())
        with patch('touch_collector.multicamera.time.monotonic_ns',return_value=self.now+200_000_000):
            with self.assertRaisesRegex(RuntimeError,'采样间隔'):rig.get_for_tick(10)
    def test_two_views_share_row_and_have_independent_paths_and_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory);capture=Capture(store,teaching(),FakeCV)
            now=time.monotonic_ns();sample=ViewSample({'wrist':(1,now,object()),'overview':(5,now-10_000_000,object())})
            p=[90,120,35,25,90,30]
            capture.add(sample,p,p,now+1,now+2,now+3,now+4,1)
            capture.finish('recording')
            row=json.loads((capture.path/'samples.jsonl').read_text())
            self.assertEqual(row['cameras']['overview']['camera_sequence'],5)
            self.assertEqual(row['camera_pair_skew_ms'],10)
            self.assertTrue((capture.path/'images_overview/000000.jpg').is_file())
            self.assertEqual(capture.meta['camera_mode'],'dual')

class ManagementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.manager=DatasetManager(self.root)
        self.path=self.manager.store.item('episodes','episode1');self.path.mkdir()
        save_json(self.path/'metadata.json',{'id':'episode1','status':'saved','task':'old task','operator_success':None,'train_ready':True})
        (self.path/'samples.jsonl').write_text('raw actions');(self.path/'video.mp4').write_bytes(b'video')
    def tearDown(self):self.tmp.cleanup()
    def test_annotation_audited_and_raw_actions_unchanged(self):
        self.manager.annotate('episodes','episode1','put blue cube on red mark',True,'reviewed')
        meta=load(self.path/'metadata.json')
        self.assertEqual(meta['original_task'],'old task');self.assertFalse(meta['train_ready'])
        self.assertEqual(len(meta['annotation_history']),1);self.assertTrue(meta['operator_success'])
        self.assertEqual((self.path/'samples.jsonl').read_text(),'raw actions')
    def test_soft_delete_restore_and_collision_never_overwrite(self):
        deleted=self.manager.trash('episodes','episode1');self.assertFalse(self.path.exists())
        self.assertEqual(len(self.manager.trash_items()),1)
        self.path.mkdir()
        with self.assertRaises(ValueError):self.manager.restore(deleted.name)
        self.path.rmdir();self.manager.restore(deleted.name)
        self.assertEqual((self.path/'video.mp4').read_bytes(),b'video')
    def test_active_work_items_protected_against_edit_and_delete(self):
        save_json(self.root/'work.json',{'capture_id':'episode1'})
        with self.assertRaises(ValueError):self.manager.trash('episodes','episode1')
        with self.assertRaises(ValueError):self.manager.annotate('episodes','episode1','new',True,'')
    def test_export_manifest_validates_every_file_and_refuses_overwrite(self):
        destination=self.root/'exports/test.zip';self.manager.export('episodes','episode1',destination)
        with zipfile.ZipFile(destination) as z:
            manifest=json.loads(z.read('export_manifest.json'))
            for name,digest in manifest['files_sha256'].items():self.assertEqual(hashlib.sha256(z.read(name)).hexdigest(),digest)
        with self.assertRaises(ValueError):self.manager.export('episodes','episode1',destination)
    def test_cancel_export_cleans_partial_but_not_source(self):
        cancel=threading.Event();cancel.set()
        with self.assertRaises(RuntimeError):self.manager.export('episodes','episode1',self.root/'exports/cancel.zip',cancel)
        self.assertFalse(list((self.root/'exports').iterdir()));self.assertTrue(self.path.exists())
    def test_unsafe_ids_and_export_inside_episode_are_rejected(self):
        with self.assertRaises(ValueError):self.manager.trash('episodes','../bad')
        with self.assertRaises(ValueError):self.manager.restore('../bad')
        with self.assertRaises(ValueError):self.manager.export('episodes','episode1',self.path/'bad.zip')
    def test_legacy_trash_is_recoverable(self):
        legacy=self.root/'trash/episodes_episode1_abcdef';self.path.rename(legacy)
        self.assertEqual(self.manager.trash_items()[0]['original_id'],'episode1')
        self.manager.restore(legacy.name);self.assertTrue(self.path.exists())

if __name__=='__main__':unittest.main()
