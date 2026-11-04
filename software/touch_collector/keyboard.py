"""Explicit, bounded Onboard D-Bus visibility; no robot or clipboard access."""
import shutil
from PyQt5 import QtCore, QtDBus


class OnboardKeyboard(QtCore.QObject):
    visibility = QtCore.pyqtSignal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.path = shutil.which('onboard')
        self.attempts = 0
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._show)
        self.active = False

    def _message(self, method):
        return QtDBus.QDBusMessage.createMethodCall('org.onboard.Onboard',
            '/org/onboard/Onboard/Keyboard', 'org.onboard.Onboard.Keyboard', method)

    def show(self):
        self.active = True
        self.attempts = 0
        self._show()

    def _show(self):
        if not self.active:
            return
        self.attempts += 1
        pending = QtDBus.QDBusConnection.sessionBus().asyncCall(self._message('Show'), 1000)
        watcher = QtDBus.QDBusPendingCallWatcher(pending, self)
        watcher.finished.connect(self._finished)

    def _finished(self, watcher):
        reply = QtDBus.QDBusPendingReply(watcher)
        watcher.deleteLater()
        if not self.active:
            return
        if not reply.isError():
            self.visibility.emit(True, '')
            return
        # Existing hidden instances need Show, not another detached process.
        if self.attempts == 1 and self.path:
            started = QtCore.QProcess.startDetached(self.path, [])
            if not (started[0] if isinstance(started, tuple) else started):
                self.visibility.emit(False, '屏幕键盘启动失败；可使用快捷任务或外接键盘。')
                return
        if self.path and self.attempts < 6:
            self.timer.start(250)
        else:
            self.visibility.emit(False, '无法显示系统键盘；可使用快捷任务或外接键盘。')

    def hide(self):
        self.active = False
        self.timer.stop()
        pending = QtDBus.QDBusConnection.sessionBus().asyncCall(self._message('Hide'), 1000)
        watcher = QtDBus.QDBusPendingCallWatcher(pending, self)
        watcher.finished.connect(self._hidden)

    def _hidden(self, watcher):
        reply = QtDBus.QDBusPendingReply(watcher)
        watcher.deleteLater()
        if not self.active:
            self.visibility.emit(reply.isError(), '未能收起系统键盘，请重试。' if reply.isError() else '')
