"""Use Qt's standard text field and the installed Onboard touchscreen keyboard."""
from PyQt5 import QtCore, QtWidgets as W
from .tasks import task_name, MAX_TASK_LENGTH, SEQUENCE_TASK, clipboard_repair_candidates
from .keyboard import OnboardKeyboard


class TaskDialog(W.QDialog):
    def __init__(self, current, parent=None):
        super().__init__(parent)
        self.setObjectName("taskEditor")
        self.setWindowTitle("编辑采集任务")
        self.setMinimumWidth(650)
        layout = W.QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 22)
        layout.setSpacing(14)
        heading = W.QLabel("编辑采集任务")
        heading.setObjectName("title")
        layout.addWidget(heading)
        note = W.QLabel("填写完整指令，说明抓哪个物体、放到哪里。仅用于之后开始的新示教，不修改已有数据。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.edit = W.QLineEdit(current)
        self.edit.setObjectName("taskNameInput")
        # A <=200-character Chinese task can arrive as up to 800 mojibake
        # characters. Preserve them until the user previews a repair.
        self.edit.setMaxLength(MAX_TASK_LENGTH*4)
        self.edit.setMinimumHeight(48)
        self.edit.setPlaceholderText("例如：从多个方块中抓起蓝色方块，放进固定盒子")
        layout.addWidget(self.edit)
        tools = W.QHBoxLayout()
        self.keyboard_controller = OnboardKeyboard(self)
        self.keyboard_path = self.keyboard_controller.path
        self.keyboard_visible = False
        self.keyboard = W.QPushButton("屏幕键盘")
        self.keyboard.setObjectName("taskKeyboard")
        self.keyboard.setEnabled(bool(self.keyboard_path))
        self.keyboard.setToolTip("显示或收起Onboard；中文输入使用系统输入法。")
        self.keyboard.clicked.connect(self.show_keyboard)
        self.keyboard_controller.visibility.connect(self.keyboard_status)
        tools.addWidget(self.keyboard)
        self.repair = W.QPushButton("修复粘贴乱码")
        self.repair.setObjectName("repairTaskClipboard")
        self.repair.clicked.connect(self.repair_text)
        tools.addWidget(self.repair)
        layout.addLayout(tools)
        presets = W.QHBoxLayout()
        for color in ("蓝", "红", "绿", "黄"):
            button = W.QPushButton(color + "色方块→盒子")
            button.clicked.connect(lambda checked=False, color=color: self.edit.setText(
                f"从多个方块中抓起{color}色方块，放进固定盒子"))
            presets.addWidget(button)
        layout.addLayout(presets)
        sequence = W.QPushButton("黄方块→盒子，再红圆柱→盒子")
        sequence.setObjectName("sequenceTaskPreset")
        sequence.clicked.connect(lambda:self.edit.setText(SEQUENCE_TASK))
        layout.addWidget(sequence)
        self.error = W.QLabel()
        self.error.setWordWrap(True)
        layout.addWidget(self.error)
        row = W.QHBoxLayout()
        cancel = W.QPushButton("取消")
        cancel.clicked.connect(self.reject)
        row.addWidget(cancel)
        self.save = W.QPushButton("保存任务名称", objectName="primary")
        self.save.setObjectName("saveTask")
        self.save.clicked.connect(self.accept)
        row.addWidget(self.save)
        layout.addLayout(row)
        self.edit.textChanged.connect(self.validate)
        self.edit.returnPressed.connect(lambda: self.accept() if self.save.isEnabled() else None)
        self.validate()
        self.edit.setFocus()
        self.edit.selectAll()

    def validate(self):
        candidates = clipboard_repair_candidates(self.edit.text())
        self.repair.setEnabled(bool(candidates))
        try:
            task_name(self.edit.text())
        except ValueError as error:
            self.error.setText(str(error))
            self.save.setEnabled(False)
        else:
            self.error.setText("疑似粘贴编码异常，可点“修复粘贴乱码”预览。" if candidates else
                               f"最多{MAX_TASK_LENGTH}字；历史任务可在主界面下拉选择。")
            self.save.setEnabled(True)

    def show_keyboard(self):
        if self.keyboard_visible:
            self.keyboard_controller.hide()
        else:
            self.keyboard_controller.show()
        self.edit.setFocus()

    def keyboard_status(self, visible, error):
        self.keyboard_visible = visible
        self.keyboard.setText("收起键盘" if visible else "屏幕键盘")
        if error:
            self.error.setText(error)

    def repair_text(self):
        candidates = clipboard_repair_candidates(self.edit.text())
        if not candidates:
            return
        preview = W.QDialog(self)
        preview.setObjectName('taskRepairPreview')
        preview.setWindowTitle('预览任务名称修复')
        preview.setMinimumWidth(620)
        box = W.QVBoxLayout(preview)
        note = W.QLabel('请核对修复后的文字；确认后仅替换输入框，仍需保存任务名称。')
        note.setWordWrap(True)
        box.addWidget(note)
        choices = W.QComboBox()
        choices.addItems(candidates)
        box.addWidget(choices)
        text = W.QPlainTextEdit(candidates[0])
        text.setReadOnly(True)
        text.setMinimumHeight(90)
        choices.currentTextChanged.connect(text.setPlainText)
        box.addWidget(text)
        buttons = W.QDialogButtonBox(W.QDialogButtonBox.Ok | W.QDialogButtonBox.Cancel)
        buttons.button(W.QDialogButtonBox.Ok).setText('采用修复结果')
        buttons.button(W.QDialogButtonBox.Cancel).setText('取消')
        buttons.accepted.connect(preview.accept)
        buttons.rejected.connect(preview.reject)
        box.addWidget(buttons)
        if preview.exec_() == W.QDialog.Accepted:
            self.edit.setText(choices.currentText())

    def done(self, result):
        if self.keyboard_controller.active:
            self.keyboard_controller.hide()
        super().done(result)

    def name(self):
        return task_name(self.edit.text())
