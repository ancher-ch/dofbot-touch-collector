"""Qt task-name regression on demo devices only, including restart and old work."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtCore, QtWidgets as W, QtTest
from .app import Worker, Window
from .storage import load
from .tasks import SEQUENCE_TASK
from test_touch_collector import teaching


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--shots", type=Path, required=True)
    args = parser.parse_args()
    args.shots.mkdir(parents=True, exist_ok=True)
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app = W.QApplication([])
    workers = []
    windows = []
    checks = []

    def wait(predicate, timeout=10):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            app.processEvents()
            if predicate():
                return
            time.sleep(.02)
        raise AssertionError("UI wait timed out")

    def start():
        worker = Worker(args.root, True)
        window = Window(worker)
        workers.append(worker); windows.append(window)
        worker.start(); window.show()
        wait(lambda: worker.engine is not None and worker.engine.valid)
        window.update_state(); app.processEvents()
        return worker, window

    def edit(window, callback):
        failures = []
        def interact():
            dialog = app.activeModalWidget()
            try:
                assert dialog and dialog.objectName() == "taskEditor"
                callback(dialog)
            except BaseException as error:
                failures.append(error)
                if dialog:
                    dialog.reject()
        QtCore.QTimer.singleShot(100, interact)
        QtTest.QTest.mouseClick(window.task_edit, QtCore.Qt.LeftButton)
        if failures:
            raise failures[0]

    def click_text(dialog, text):
        button = next(b for b in dialog.findChildren(W.QPushButton) if b.text() == text)
        QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)

    try:
        worker, window = start()
        original = worker.engine.task
        def cancel(dialog):
            dialog.edit.setText("取消的名称")
            click_text(dialog, "取消")
        edit(window, cancel)
        assert worker.engine.task == original
        assert not (args.root/"task_config.json").exists()
        checks.append("cancel_changes_nothing")

        name = SEQUENCE_TASK
        def save(dialog):
            dialog.edit.clear()
            assert not dialog.save.isEnabled()
            dialog.edit.setText("   ")
            assert not dialog.save.isEnabled()
            click_text(dialog, "蓝色方块→盒子")
            assert dialog.edit.text() == "从多个方块中抓起蓝色方块，放进固定盒子"
            click_text(dialog, "黄方块→盒子，再红圆柱→盒子")
            assert dialog.edit.text() == name
            if dialog.keyboard_path:
                with patch.object(dialog.keyboard_controller, 'show', side_effect=lambda:dialog.keyboard_status(True,'')) as show_keyboard,patch.object(dialog.keyboard_controller,'hide') as hide_keyboard:
                    QtTest.QTest.mouseClick(dialog.keyboard, QtCore.Qt.LeftButton)
                    show_keyboard.assert_called_once_with()
                    assert dialog.keyboard.text() == '收起键盘'
                    QtTest.QTest.mouseClick(dialog.keyboard, QtCore.Qt.LeftButton)
                    hide_keyboard.assert_called_once_with()
            # The screenshot matches GBK decoded as Latin-1. Repair is previewed
            # and explicit; cancel must preserve the input byte-for-byte.
            for source_encoding in ('gb18030','utf-8'):
                broken = name.encode(source_encoding).decode('latin1')
                dialog.edit.setText(broken)
                for accept in (False,True):
                    preview_errors=[]
                    def preview():
                        modal=app.activeModalWidget()
                        try:
                            assert modal.objectName() == 'taskRepairPreview'
                            assert modal.findChild(W.QPlainTextEdit).toPlainText() == name
                            modal.accept() if accept else modal.reject()
                        except BaseException as error:
                            preview_errors.append(str(error));modal.reject()
                    QtCore.QTimer.singleShot(50,preview)
                    QtTest.QTest.mouseClick(dialog.repair,QtCore.Qt.LeftButton)
                    assert not preview_errors,preview_errors
                    assert dialog.edit.text() == (name if accept else broken)
            # Real Qt clipboard/paste path, including Chinese and punctuation.
            dialog.edit.selectAll()
            app.clipboard().setText(name)
            QtTest.QTest.keyClick(dialog.edit, QtCore.Qt.Key_V, QtCore.Qt.ControlModifier)
            assert dialog.edit.text() == name and dialog.save.isEnabled()
            dialog.grab().save(str(args.shots/"task_editor.png"))
            QtTest.QTest.mouseClick(dialog.save, QtCore.Qt.LeftButton)
        edit(window, save)
        wait(lambda:not worker.pending and worker.engine.task == name)
        window.update_state()
        assert window.task.currentText() == name
        assert load(args.root/"task_config.json")["current"] == name
        assert worker.engine.robot.writes + worker.engine.robot.torques == []
        checks.extend(["empty_rejected", "sequence_touch_preset", "chinese_clipboard_input", "gbk_utf8_repair_preview_cancel_and_apply", "explicit_keyboard_show_hide", "custom_name_applied_without_actuation"])
        window.grab().save(str(args.shots/"custom_task_ready.png"))

        # Returning to a preset uses the same validated, persisted worker path.
        preset = "关节小幅往返测试"
        window.task.setCurrentText(preset)
        window.task.activated[str].emit(preset)
        wait(lambda:not worker.pending and worker.engine.task == preset)
        window.update_state()
        assert window.task.currentText() == preset
        assert name in worker.engine.snapshot()["task_history"]
        worker.close(); window.hide()
        worker, window = start()
        assert worker.engine.task == preset and window.task.currentText() == preset
        checks.append("history_and_selection_survive_restart")

        old = teaching()
        old["task"] = "旧示教的独立任务名称"
        p = worker.engine.store.item("teachings",old["id"])
        p.mkdir(); worker.engine.store.write_teaching(old)
        before = (p/"trajectory.json").read_bytes()
        worker.engine.holding = "confirmed"
        assert worker.submit("select_teaching", item_id=old["id"])
        wait(lambda:not worker.pending and worker.engine.stage == "saved_teach")
        window.update_state()
        assert window.task.currentText() == old["task"]
        assert not window.task_edit.isEnabled() and not window.task.isEnabled()
        assert (p/"trajectory.json").read_bytes() == before
        worker.close(); window.hide()
        worker, window = start()
        assert window.task.currentText() == old["task"]
        assert not window.task_edit.isEnabled()
        checks.append("restored_old_task_displayed_and_locked")
        assert worker.submit("next_teaching")
        wait(lambda:not worker.pending and worker.engine.stage == "ready")
        window.update_state()
        assert window.task_edit.isEnabled()
        assert worker.submit("task", task=name)
        wait(lambda:not worker.pending and worker.engine.task == name)
        assert (p/"trajectory.json").read_bytes() == before
        assert worker.engine.robot.writes + worker.engine.robot.torques == []
        checks.append("next_episode_can_change_task_without_modifying_history")
        assert "Arm_Lib" not in sys.modules
        report = {"passed":True,"simulated":True,"checks":checks,"real_position_requests":0,"real_torque_requests":0}
        (args.shots/"task_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False))
    finally:
        for worker in workers:
            if worker.thread.is_alive():
                worker.close()
        for window in windows:
            window.timer.stop(); app.removeEventFilter(window.emergency_key)
            window.hide(); window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        app.quit()


if __name__ == "__main__":
    main()
