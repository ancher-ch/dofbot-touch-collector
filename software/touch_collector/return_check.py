"""Regression for adaptive home timing. Uses Worker(demo=True), no physical IO."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window
from .core import home_plan
from test_touch_collector import teaching


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--shots',type=Path,required=True)
    args=parser.parse_args()
    args.shots.mkdir(parents=True,exist_ok=True)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([])
    worker=Worker(args.root,True)
    window=Window(worker)
    window.show();worker.start()
    checks=[]

    def wait(predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents();window.update_state()
            if predicate():return
            time.sleep(.01)
        raise AssertionError(worker.engine.message if worker.engine else worker.error)

    try:
        wait(lambda:worker.engine is not None and worker.engine.valid)
        e=worker.engine
        assert e.robot.writes+e.robot.torques==[]
        assert window.home_btn.text()=='快速回初始'
        e.holding='confirmed'
        confirmations=[]
        original_confirm=window.confirm
        window.confirm=lambda title,text,boxes:confirmations.append((title,text,list(boxes))) or False
        start=[80,82,30,27,89,106];target=[89,66,30,16,89,30]
        e.robot.pose=start[:];e.home=target[:]
        wait(lambda:e.pose==start)
        assert window.motion_confirm(False) is False
        assert '4.8秒' in confirmations[-1][1] and len(confirmations[-1][2])==2
        assert '回位计划不能执行' not in confirmations[-1][0]
        assert e.robot.writes+e.robot.torques==[]
        checks.append('reported_76_degree_span_has_4_8_second_confirmation_instead_of_rejection')

        start=[90,120,35,25,90,30];target=[90,30,35,25,90,30]
        e.robot.pose=start[:];e.home=target[:]
        wait(lambda:e.pose==start)
        window.confirm=original_confirm
        accepted=[]
        def accept():
            dialog=app.activeModalWidget()
            assert dialog is not None
            text='\n'.join(label.text() for label in dialog.findChildren(W.QLabel))
            assert '5.7秒' in text and '超过5秒' in text
            for checkbox in dialog.findChildren(W.QCheckBox):checkbox.setChecked(True)
            dialog.grab().save(str(args.shots/'long_confirmation.png'))
            button=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=='确认执行')
            assert button.isEnabled()
            accepted.append(True)
            QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
        QtCore.QTimer.singleShot(100,accept)
        QtTest.QTest.mouseClick(window.home_btn,QtCore.Qt.LeftButton)
        assert accepted
        wait(lambda:e.activity=='returning')
        wait(lambda:e.activity=='returning' and e.elapsed>5.0)
        window.grab().save(str(args.shots/'still_returning_after_five_seconds.png'))
        wait(lambda:not worker.pending and e.activity=='idle')
        assert e.robot.pose==target and e.holding=='confirmed' and not e.error
        assert e.robot.torques==[]
        assert '5.7秒' in e.message
        checks.append('touch_confirmed_return_runs_beyond_five_seconds_and_finishes_without_torque_changes')

        document=teaching();e.teaching=document;e.stage='saved_teach'
        window.confirm=lambda title,text,boxes:confirmations.append((title,text,list(boxes))) or False
        assert window.motion_confirm(True) is False
        assert '本条示教起点' in confirmations[-1][1] and '5.7秒' in confirmations[-1][1]
        assert e.home==target
        checks.append('replay_confirmation_uses_adaptive_timing_for_its_own_first_pose_without_changing_home')
        assert (len(home_plan([0]*6,[180,180,180,180,270,180]))-1)/10==16.9
        assert 'Arm_Lib' not in sys.modules
        window.grab().save(str(args.shots/'ready_after_return.png'))
        report={'passed':True,'simulated':True,'real_motor_requests':0,'checks':checks,
                'long_return_planned_seconds':5.7,'largest_sdk_span_planned_seconds':16.9}
        (args.shots/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        window.timer.stop();app.removeEventFilter(window.emergency_key);worker.close()
        for widget in app.topLevelWidgets():widget.hide();widget.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
        app.quit()


if __name__=='__main__':main()
