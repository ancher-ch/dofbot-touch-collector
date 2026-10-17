"""Demo-only touchscreen checks: dual capture, management, F12 and latched stop."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch
import zipfile

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window
from .camera_ui import CameraDialog
from .manager_ui import ManagerDialog
from .playback import VideoDialog
from .storage import load
from .safety import EmergencyLatch
from test_touch_collector import teaching

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True);parser.add_argument('--shots',type=Path,required=True)
    args=parser.parse_args();args.shots.mkdir(parents=True,exist_ok=True)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([]);worker=Worker(args.root,True);window=Window(worker);window.show();worker.start()
    checks=[]
    def wait(predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():window.update_state();app.processEvents();return
            time.sleep(.01)
        raise AssertionError(worker.engine.message if worker.engine else worker.error)
    def shot(name,obj=window):
        app.processEvents();assert obj.width()<=1280 and obj.height()<=800,(name,obj.size())
        obj.grab().save(str(args.shots/(name+'.png')))
    try:
        wait(lambda:worker.engine and worker.engine.valid)
        e=worker.engine
        def configure():
            dialog=app.activeModalWidget();assert isinstance(dialog,CameraDialog)
            dialog.mode.setCurrentIndex(1);dialog.choices['wrist'].setCurrentIndex(0);dialog.choices['overview'].setCurrentIndex(1)
            assert not dialog.apply.isEnabled()
            dialog.support.setChecked(True);assert dialog.apply.isEnabled();shot('01_dual_config',dialog)
            QtTest.QTest.mouseClick(dialog.apply,QtCore.Qt.LeftButton)
        QtCore.QTimer.singleShot(80,configure);QtTest.QTest.mouseClick(window.camera_settings,QtCore.Qt.LeftButton)
        wait(lambda:not worker.pending and worker.camera_config['mode']=='dual')
        checks.append('dual_selection_requires_fixed_arm_confirmation')
        document=teaching();e.store.write_teaching(document);e.teaching=document;e.home=document['reference_pose'][:]
        e.stage='saved_teach';e.holding='confirmed';e.persist();window.update_state()
        assert worker.submit('replay_capture',confirmed=True)
        wait(lambda:not worker.pending and e.stage=='capture_review',25)
        assert e.capture.meta['status']=='draft',e.message
        episode=e.capture.path;rows=[json.loads(x) for x in (episode/'samples.jsonl').read_text().splitlines()]
        assert all(set(row['cameras'])=={'wrist','overview'} and row['camera_pair_skew_ms']<=35 for row in rows)
        for name in ('video.mp4','video_overview.mp4'):assert (episode/name).is_file()
        assert e.capture.meta['capture_checks_passed']
        checks.append('two_encoded_videos_share_frame_and_action_rows')
        assert worker.submit('save_capture',outcome=None);wait(lambda:not worker.pending)
        assert worker.submit('next_teaching');wait(lambda:not worker.pending)
        # Read-only two-view selection and MP4 fallback for the second camera.
        viewer=VideoDialog(args.root,e.cv2,current=episode,emergency=worker.emergency_stop)
        viewer.show();assert viewer.player.view_choice.count()==2
        viewer.player.view_choice.setCurrentIndex(1);app.processEvents();assert viewer.player.clip.role=='overview'
        shot('02_second_camera_viewer',viewer);viewer.accept()
        raw=episode/'images_overview';backup=episode/'images_overview_hold';raw.rename(backup)
        fallback=VideoDialog(args.root,e.cv2,current=episode)
        fallback.player.view_choice.setCurrentIndex(1);assert fallback.player.clip.decoder is not None
        fallback.accept();backup.rename(raw)
        checks.append('second_camera_viewing_and_video_fallback')
        original_rows=(episode/'samples.jsonl').read_bytes();errors=[]
        def manage():
            dialog=app.activeModalWidget()
            try:
                assert isinstance(dialog,ManagerDialog) and worker.viewing.is_set()
                dialog.task.setText('把蓝色方块放到红色标识内');dialog.success.setCurrentIndex(1);dialog.notes.setPlainText('软件验收模拟数据')
                QtTest.QTest.mouseClick(dialog.buttons['annotate'],QtCore.Qt.LeftButton)
                assert load(episode/'metadata.json')['operator_success'] is True
                exported=args.root/'exports'/'episode.zip'
                with patch.object(dialog,'choose_export_path',return_value=str(exported)):
                    QtTest.QTest.mouseClick(dialog.buttons['export'],QtCore.Qt.LeftButton)
                wait(lambda:dialog.job is None)
                assert exported.is_file(),dialog.message.text()
                with zipfile.ZipFile(exported) as z:assert 'export_manifest.json' in z.namelist() and 'video_overview.mp4' in z.namelist()
                with patch.object(W.QMessageBox,'question',return_value=W.QMessageBox.Yes):
                    QtTest.QTest.mouseClick(dialog.buttons['trash'],QtCore.Qt.LeftButton)
                assert not episode.exists()
                dialog.kind.setCurrentIndex(2);assert dialog.table.rowCount()==1
                QtTest.QTest.mouseClick(dialog.buttons['restore'],QtCore.Qt.LeftButton)
                assert episode.exists() and (episode/'samples.jsonl').read_bytes()==original_rows
                dialog.kind.setCurrentIndex(0);dialog.filter.setCurrentIndex(1);assert dialog.table.rowCount()==1
                dialog.search.setText('not-present');assert dialog.table.rowCount()==0
                dialog.search.clear();shot('03_dataset_manager',dialog)
                QtTest.QTest.keyClick(dialog,QtCore.Qt.Key_F12)
                assert worker.safety.tripped.is_set()
                checks.append('manager_edit_filter_export_trash_restore_and_modal_F12')
            except BaseException as error:errors.append(repr(error))
            finally:
                if isinstance(dialog,ManagerDialog):dialog.accept()
        old_confirm=window.confirm;window.confirm=lambda *a,**k:True
        QtCore.QTimer.singleShot(250,manage)
        QtTest.QTest.mouseClick(window.manager_btn,QtCore.Qt.LeftButton)
        wait(lambda:not worker.viewing.is_set() and not worker.pending,30)
        window.confirm=old_confirm
        assert not errors,errors
        before=(len(e.robot.writes),len(e.robot.torques))
        assert worker.submit('return_home',confirmed=True,countdown=False);wait(lambda:not worker.pending)
        assert before==(len(e.robot.writes),len(e.robot.torques))
        assert EmergencyLatch(args.root/'safety.json').tripped.is_set()
        shot('04_latched_stop')
        assert worker.submit('reset_emergency',confirmed=True);wait(lambda:not worker.pending)
        assert before==(len(e.robot.writes),len(e.robot.torques))
        assert not worker.safety.tripped.is_set()
        checks.append('queued_motion_blocked_restart_latched_reset_does_not_resume')
        single={'mode':'single','wrist':'/dev/video0','overview':None}
        assert worker.submit('configure_camera',config=single,confirmed=True);wait(lambda:not worker.pending)
        assert len(e.camera.cameras)==1 and load(args.root/'camera_config.json')['mode']=='single'
        with patch.object(worker,'camera_devices',return_value=[{'path':'/dev/video0'}]):
            assert worker.submit('configure_camera',config={'mode':'dual','wrist':'/dev/video0','overview':'/dev/video2'},confirmed=True)
            wait(lambda:not worker.pending)
        assert worker.camera_config==single and len(e.camera.cameras)==1
        checks.append('switch_back_single_and_missing_second_preserves_configuration')
        from . import app as module
        factory=module.make_rig
        class UnreleasedCamera:
            blocked=True
            def start(self):pass
            def status(self):return {'ready':False,'fps':0,'error':'injected startup failure'}
            def close(self):
                if self.blocked:raise RuntimeError('injected close failure')
        failed=UnreleasedCamera()
        with patch.object(module,'make_rig',return_value=failed):
            assert worker.submit('configure_camera',config=single,confirmed=True);wait(lambda:not worker.pending)
        assert e.camera is failed and worker.camera_config==single
        with patch.object(module,'make_rig',side_effect=AssertionError('must not open duplicate camera')) as opening:
            assert worker.submit('configure_camera',config=single,confirmed=True);wait(lambda:not worker.pending)
            opening.assert_not_called()
        failed.blocked=False
        assert worker.submit('configure_camera',config=single,confirmed=True);wait(lambda:not worker.pending)
        assert len(e.camera.cameras)==1
        checks.append('failed_camera_cleanup_retains_owner_and_prevents_duplicate_open')
        # Stop an actual simulated replay after commands begin. The real machine is never addressed.
        e.teaching=document;e.stage='saved_teach';e.holding='confirmed';e.robot.pose=document['reference_pose'][:];e.persist()
        writes=len(e.robot.writes)
        assert worker.submit('replay_capture',confirmed=True)
        wait(lambda:len(e.robot.writes)>=writes+3,15)
        worker.emergency_stop();stopped=len(e.robot.writes)
        wait(lambda:not worker.pending)
        assert len(e.robot.writes)==stopped and e.capture.meta['status']=='interrupted'
        assert e.robot.torques==[]
        assert worker.submit('reset_emergency',confirmed=True);wait(lambda:not worker.pending)
        assert len(e.robot.writes)==stopped
        checks.append('in_flight_replay_stop_preserves_interrupted_episode_without_torque_change')
        # Manual home setup must remain recoverable after a supported stop.
        e.stage='home_setup';e.holding='free'
        before=(len(e.robot.writes),len(e.robot.torques))
        worker.emergency_stop();window.update_state()
        assert window.reset_stop_btn.isEnabled()
        old_confirm=window.confirm;window.confirm=lambda *a,**k:True
        QtTest.QTest.mouseClick(window.reset_stop_btn,QtCore.Qt.LeftButton)
        wait(lambda:not worker.pending and not worker.safety.tripped.is_set())
        window.confirm=old_confirm
        assert e.stage=='home_setup' and e.holding=='free'
        assert before==(len(e.robot.writes),len(e.robot.torques))
        checks.append('touch_reset_in_initial_setup_preserves_free_state_without_deadlock')
        assert 'Arm_Lib' not in sys.modules
        report={'passed':True,'simulated':True,'real_motor_requests':0,'dual_frames':len(rows),'checks':checks}
        (args.shots/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    finally:
        window.timer.stop();app.removeEventFilter(window.emergency_key)
        worker.close()
        for widget in app.topLevelWidgets():widget.hide();widget.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
        app.quit()

if __name__=='__main__':main()
