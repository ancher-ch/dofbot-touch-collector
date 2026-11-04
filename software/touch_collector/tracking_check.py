"""Exercise persistent J6 failure in Qt, without importing real hardware."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtCore,QtWidgets as W
from .app import Worker,Window


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--shots',type=Path,required=True)
    args=parser.parse_args();args.shots.mkdir(parents=True,exist_ok=True)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([]);worker=Worker(args.root,True);window=Window(worker)
    window.show();worker.start()
    def wait(predicate,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            app.processEvents()
            if predicate():window.update_state();return
            time.sleep(.02)
        raise AssertionError(worker.engine.message if worker.engine else worker.error)
    try:
        wait(lambda:worker.engine is not None and worker.engine.valid)
        e=worker.engine;e.robot.pose[5]=96;e.holding='confirmed'
        original_write=e.robot.write
        def stalled_write(pose,duration):
            original_write(pose,duration)
            e.robot.pose[5]=96
        e.robot.write=stalled_write
        doc=e.store.new_teaching('模拟：J6不跟随',e.robot.pose)
        doc.update(status='recorded',simulated=True,quality_warnings=[])
        doc['samples']=[{'t':i/10,'q':[90,120,35,25,90,96+i]} for i in range(15)]
        e.store.write_teaching(doc);e.teaching=doc;e.stage='saved_teach';e.persist()
        assert worker.submit('replay_capture',confirmed=True,gripper_extra_degrees=3)
        wait(lambda:not worker.pending and e.stage=='capture_review',20)
        assert 'J6夹爪' in e.error and '不等于夹紧成功' in e.error
        assert e.capture.meta['status']=='error' and e.capture.meta['operator_success'] is None
        assert not e.capture.meta['capture_checks_passed']
        window.grab().save(str(args.shots/'replay_gripper_failed.png'))
        e.home=[90,120,35,25,90,106]
        assert worker.submit('return_home',confirmed=True,countdown=False)
        wait(lambda:not worker.pending)
        assert 'J6夹爪' in e.error
        assert e.robot.pose[5]==96
        window.grab().save(str(args.shots/'return_gripper_failed.png'))
        assert e.robot.torques==[] and 'Arm_Lib' not in sys.modules
        report={'passed':True,'simulated':True,'persistent_stall_still_stops_replay_and_return':True,
                'failed_capture_not_accepted':True,'real_motor_requests':0,'error':e.error}
        (args.shots/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False))
    finally:
        window.timer.stop();app.removeEventFilter(window.emergency_key);worker.close()
        for widget in app.topLevelWidgets():widget.hide();widget.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
        app.quit()


if __name__=='__main__':main()
