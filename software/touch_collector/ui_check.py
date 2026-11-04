"""Touch-style Qt interaction smoke test. Demo devices only; never imports Arm_Lib."""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root",type=Path,required=True)
    p.add_argument("--shots",type=Path,required=True)
    args = p.parse_args()
    args.shots.mkdir(parents=True,exist_ok=True)
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app = W.QApplication([])
    worker = Worker(args.root,True)
    window = Window(worker)
    window.show()
    worker.start()

    def wait(predicate,timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():
                window.update_state()
                return
            time.sleep(.02)
        state=worker.engine.snapshot() if worker.engine else {}
        state.pop("preview",None)
        raise AssertionError("UI timeout: "+str(worker.error or state))

    def click(button,modal=False):
        assert button.isVisible() and button.isEnabled(),button.text()
        if modal:
            def accept():
                dialog = app.activeModalWidget()
                assert dialog is not None
                choice=dialog.findChild(W.QComboBox,"gripperCompensation")
                if choice is not None:
                    # This older full-workflow fixture has no gripper closure.
                    # Compensation-specific coverage lives in gripper_check.
                    choice.setCurrentIndex(0)
                    QtCore.QTimer.singleShot(80,accept)
                    button=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=="下一步：检查回放路径")
                    QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
                    return
                for box in dialog.findChildren(W.QCheckBox):
                    box.setChecked(True)
                candidates = [b for b in dialog.findChildren(W.QPushButton) if b.text() in ("确认执行","开始观看")]
                assert candidates and candidates[0].isEnabled()
                QtTest.QTest.mouseClick(candidates[0],QtCore.Qt.LeftButton)
            QtCore.QTimer.singleShot(80,accept)
        QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
        app.processEvents()

    def shot(name):
        window.update_state()
        app.processEvents()
        rects = []
        for button in list(window.actions.values())+[window.hold_button,window.confirm_button]:
            if button.isVisible():
                top = button.mapTo(window,QtCore.QPoint(0,0))
                assert top.y()+button.height() <= window.height(),("clipped",button.text())
                rect = QtCore.QRect(top,button.size())
                for previous, text in rects:
                    assert not rect.intersects(previous),("overlap",text,button.text())
                rects.append((rect,button.text()))
        window.grab().save(str(args.shots/(name+".png")))

    try:
        wait(lambda:worker.engine is not None and worker.engine.valid)
        assert worker.engine.robot.torques+worker.engine.robot.writes == []
        assert worker.engine.home is None
        custom_task = "从多个方块中抓起蓝色方块，放进固定盒子"
        def enter_task():
            dialog = app.activeModalWidget()
            assert dialog.objectName() == "taskEditor"
            dialog.edit.setText(custom_task)
            QtTest.QTest.mouseClick(dialog.save, QtCore.Qt.LeftButton)
        QtCore.QTimer.singleShot(100, enter_task)
        click(window.task_edit)
        wait(lambda:not worker.pending and worker.engine.task == custom_task)
        assert window.task.currentText() == custom_task
        click(window.set_home_btn,True)
        wait(lambda:not worker.pending and worker.engine.stage=="home_setup")
        shot("00_home_setup")
        worker.submit("demo_nudge",joint=1,delta=3)
        wait(lambda:not worker.pending and worker.engine.pose[1]==123)
        click(window.actions["finish_home"],True)
        wait(lambda:not worker.pending and worker.engine.holding=="pending")
        assert worker.engine.home is None
        assert worker.engine.robot.writes==[]
        shot("00_home_pending")
        click(window.confirm_button,True)
        wait(lambda:not worker.pending and worker.engine.holding=="confirmed")
        assert worker.engine.home==[90,123,35,25,90,30]
        assert worker.engine.robot.torques==[0,1]
        shot("01_ready")
        click(window.actions["release"],True)
        wait(lambda:not worker.pending and worker.engine.stage=="prepare")
        click(window.actions["begin"])
        wait(lambda:worker.engine.stage=="teaching")
        assert worker.engine.teaching["task"] == custom_task
        assert not window.task_edit.isEnabled()
        wait(lambda:len(worker.engine.teaching["samples"])>=4)
        worker.submit("demo_nudge",joint=4,delta=3)
        wait(lambda:not worker.pending)
        wait(lambda:len(worker.engine.teaching["samples"])>=13)
        click(window.actions["end"])
        wait(lambda:not worker.pending and worker.engine.stage=="teach_review")
        assert worker.engine.robot.writes==[]
        shot("02_teach_review")
        click(window.hold_button,True)
        wait(lambda:not worker.pending and worker.engine.holding=="pending")
        click(window.confirm_button,True)
        wait(lambda:not worker.pending and worker.engine.holding=="confirmed")
        click(window.actions["save_teach"])
        wait(lambda:not worker.pending and worker.engine.stage=="saved_teach")
        shot("03_saved_teaching")
        click(window.actions["replay"],True)
        wait(lambda:not worker.pending and worker.engine.stage=="capture_review",30)
        assert worker.engine.capture.meta["status"]=="draft",worker.engine.message
        assert worker.engine.capture.meta["task"] == custom_task
        assert window.task.currentText() == custom_task
        shot("04_capture_review")
        count = len(worker.engine.robot.writes)
        def close_preview():
            dialog = app.activeModalWidget()
            assert dialog
            dialog.grab().save(str(args.shots/"05_frame_preview.png"))
            dialog.accept()
        QtCore.QTimer.singleShot(600,close_preview)
        click(window.actions["preview_take"],True)
        wait(lambda:not worker.pending and not worker.viewing.is_set())
        assert len(worker.engine.robot.writes)==count
        def save_dialog():
            dialog = app.activeModalWidget()
            buttons = [b for b in dialog.findChildren(W.QPushButton) if b.text()=="保存到本机"]
            QtTest.QTest.mouseClick(buttons[0],QtCore.Qt.LeftButton)
        QtCore.QTimer.singleShot(100,save_dialog)
        click(window.actions["save_capture"])
        wait(lambda:not worker.pending and worker.engine.capture.meta["status"]=="saved")
        assert worker.engine.capture.meta["operator_success"] is None
        shot("06_saved_capture")
        saved_episode=str(worker.engine.capture.path)
        saved_frames=worker.engine.capture.meta["frames"]
        for cycle in range(3):
            click(window.actions["next"])
            wait(lambda:not worker.pending and worker.engine.stage=="ready")
            click(window.home_btn,True)
            wait(lambda:not worker.pending and worker.engine.activity=="idle",15)
            assert window.actions["release"].isVisible() and window.actions["release"].isEnabled()
            assert not window.actions["begin"].isVisible()
            shot(f"07_returned_cycle_{cycle}")
            click(window.actions["release"],True)
            wait(lambda:not worker.pending and worker.engine.holding=="free")
            worker.submit("demo_nudge",joint=2,delta=-4)
            wait(lambda:not worker.pending)
            click(window.actions["begin"])
            wait(lambda:worker.engine.stage=="teaching")
            wait(lambda:len(worker.engine.teaching["samples"])>=4)
            worker.submit("demo_nudge",joint=5,delta=15)
            wait(lambda:not worker.pending)
            wait(lambda:len(worker.engine.teaching["samples"])>=12)
            click(window.actions["end"])
            wait(lambda:not worker.pending and worker.engine.stage=="teach_review")
            click(window.hold_button,True)
            wait(lambda:not worker.pending and worker.engine.holding=="pending")
            click(window.confirm_button,True)
            wait(lambda:not worker.pending and worker.engine.holding=="confirmed")
            click(window.actions["save_teach"],True)
            wait(lambda:not worker.pending and worker.engine.stage=="saved_teach")
            assert worker.engine.teaching["warnings_reviewed"]
        report = {"passed":True,"simulated":True,"real_motor_requests":0,
                  "screenshots":str(args.shots),"episode":saved_episode,
                  "frames":saved_frames,"return_release_teach_cycles":3,
                  "four_degree_start_difference_allowed":True,"fast_gripper_reviewed_and_saved":True}
        report["custom_task_in_teaching_and_capture"] = custom_task
        (args.shots/"ui_report.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        worker.close()
        window.hide()
        app.quit()


if __name__=="__main__":
    main()
