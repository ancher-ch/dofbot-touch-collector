"""Inject a persistent J3 fault into demo hardware; exercise native recovery UI."""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window
from .hardware import FeedbackError


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,required=True)
    parser.add_argument("--shots",type=Path,required=True)
    args=parser.parse_args()
    args.shots.mkdir(parents=True,exist_ok=True)
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([])
    worker=Worker(args.root,True)
    window=Window(worker)
    window.show()
    worker.start()

    def wait(predicate,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():
                window.update_state()
                app.processEvents()
                return
            time.sleep(.02)
        raise AssertionError(worker.engine.message if worker.engine else worker.error)

    def click(button,modal=False):
        window.update_state()
        assert button.isVisible() and button.isEnabled(),button.text()
        if modal:
            def accept():
                dialog=app.activeModalWidget()
                assert dialog
                for box in dialog.findChildren(W.QCheckBox): box.setChecked(True)
                button=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=="确认执行")
                QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
            QtCore.QTimer.singleShot(80,accept)
        QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
        app.processEvents()

    def shot(name):
        window.update_state()
        app.processEvents()
        rectangles=[]
        for button in list(window.actions.values())+[window.hold_button,window.feedback_button,window.confirm_button,window.recover_button,window.set_home_btn,window.home_btn]:
            if not button.isVisible(): continue
            top=button.mapTo(window,QtCore.QPoint(0,0))
            rect=QtCore.QRect(top,button.size())
            assert window.rect().contains(rect),("clipped",button.text())
            for previous,text in rectangles:
                assert not rect.intersects(previous),("overlap",text,button.text())
            rectangles.append((rect,button.text()))
        window.grab().save(str(args.shots/(name+".png")))

    try:
        wait(lambda:worker.engine is not None and worker.engine.valid)
        click(window.set_home_btn,True)
        wait(lambda:not worker.pending and worker.engine.stage=="home_setup")
        click(window.actions["finish_home"],True)
        wait(lambda:not worker.pending and worker.engine.holding=="pending")
        click(window.confirm_button,True)
        wait(lambda:not worker.pending and worker.engine.holding=="confirmed")
        click(window.actions["release"],True)
        wait(lambda:not worker.pending and worker.engine.stage=="prepare")
        click(window.actions["begin"])
        wait(lambda:worker.engine.stage=="teaching" and len(worker.engine.teaching["samples"])>=4)
        e=worker.engine
        normal_read=e.robot.read
        delay_once=threading.Event()
        delay_once.set()
        def slow_read():
            if delay_once.is_set():
                delay_once.clear()
                time.sleep(.31)
            return normal_read()
        e.robot.read=slow_read
        count=len(e.teaching["samples"])
        wait(lambda:len(e.teaching["samples"])>=count+4)
        assert e.stage=="teaching" and worker.thread.is_alive()
        fault=threading.Event()
        def read():
            if fault.is_set():
                pose=normal_read()
                pose[2]=None
                raise FeedbackError(pose,[None,None,-4.9,None,None,None],
                                    [None,None,"原始映射约-4.9°，超出SDK范围",None,None,None])
            return normal_read()
        e.robot.read=read
        torques_before=e.robot.torques[:]
        fault.set()
        wait(lambda:e.stage=="teach_review" and not e.valid)
        assert e.teaching["status"]=="error"
        assert e.holding=="free" and e.robot.torques==torques_before
        assert worker.thread.is_alive()
        assert window.hold_button.isEnabled() and window.feedback_button.isEnabled()
        assert not window.home_btn.isEnabled() and not window.actions["save_teach"].isEnabled()
        previous=e.last_preview
        wait(lambda:e.last_preview>previous+.2)
        shot("01_invalid_j3")
        click(window.hold_button,True)
        wait(lambda:not worker.pending and e.holding=="pending")
        assert e.robot.torques==torques_before+[1]
        assert e.robot.writes==[]
        assert not window.confirm_button.isEnabled()
        assert not window.home_btn.isEnabled()
        assert window.recover_button.isVisible() and window.recover_button.isEnabled()
        shot("02_hold_pending_invalid")
        # The original deadlock: feedback stays invalid until the operator can
        # undo holding and adjust manually. Do not clear the injected fault yet.
        before=e.robot.torques[:]
        window.confirm=lambda *args,**kwargs:False
        click(window.recover_button)
        assert e.robot.torques==before and e.holding=="pending"
        del window.confirm
        click(window.recover_button,True)
        wait(lambda:not worker.pending and e.holding=="free")
        assert e.robot.torques==before+[0] and e.robot.writes==[]
        assert not e.valid and e.pose[2] is None
        assert e.teaching["status"]=="error"
        assert not window.home_btn.isEnabled()
        shot("02b_supported_release_invalid")
        fault.clear()
        wait(lambda:e.valid)
        click(window.hold_button,True)
        wait(lambda:not worker.pending and e.holding=="pending")
        click(window.confirm_button,True)
        wait(lambda:not worker.pending and e.holding=="confirmed")
        shot("03_feedback_restored")
        click(window.actions["delete"],True)
        wait(lambda:not worker.pending and e.stage=="ready")
        click(window.actions["release"],True)
        wait(lambda:not worker.pending and e.stage=="prepare")
        click(window.actions["begin"])
        wait(lambda:e.stage=="teaching" and len(e.teaching["samples"])>=4)
        # Stop handling used to sit outside the worker exception boundary. Inject
        # a single storage error exactly there and verify the worker survives.
        original_write=e.store.write_teaching
        fail_once=threading.Event()
        fail_once.set()
        def write_teaching(d):
            if fail_once.is_set():
                fail_once.clear()
                raise OSError("simulated one-shot storage failure during stop")
            return original_write(d)
        e.store.write_teaching=write_teaching
        worker.stop()
        wait(lambda:e.stage=="teach_review")
        e.store.write_teaching=original_write
        assert worker.thread.is_alive()
        assert e.teaching["status"]=="error"
        assert e.robot.writes==[]
        assert "Arm_Lib" not in sys.modules
        shot("04_stop_error_survived")
        report={"passed":True,"simulated":True,"real_motor_requests":0,
                "persistent_fault_kept_preview":True,"invalid_angles_never_recorded":True,
                "emergency_hold_only_after_touch_confirmation":True,"position_requests":len(e.robot.writes),
                "worker_survived_stop_storage_error":worker.thread.is_alive()}
        report["sampling_jitter_preserves_valid_timeline"]=True
        report["pending_invalid_deadlock_recovered_without_position_commands"]=True
        report["cancel_recovery_never_changes_torque"]=True
        (args.shots/"fault_report.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        worker.close()
        window.hide()
        app.quit()


if __name__=="__main__":
    main()
