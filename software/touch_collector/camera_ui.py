"""Select named RGB streams; metadata nodes are filtered before this dialog."""
from PyQt5 import QtWidgets as W
from pathlib import Path

class CameraDialog(W.QDialog):
    def __init__(self,devices,config,parent=None,emergency=None):
        super().__init__(parent)
        self.setWindowTitle('单摄 / 双摄配置');self.resize(700,450)
        layout=W.QVBoxLayout(self)
        text=W.QLabel('双摄会在主界面同时显示两路彩色画面，并在回放采集时保存两路RGB。\n每路独立读取并记录时间戳，不是硬件触发同步；深度、红外和元数据节点不在此选择。')
        text.setWordWrap(True);layout.addWidget(text)
        self.mode=W.QComboBox();self.mode.addItems(['单摄','双摄']);self.mode.setCurrentIndex(int(config['mode']=='dual'));layout.addWidget(self.mode)
        self.choices={}
        for role,title in (('wrist','腕部 / 主相机'),('overview','外部 / 第二相机')):
            layout.addWidget(W.QLabel(title));combo=W.QComboBox()
            for device in devices:combo.addItem(device['card']+' · '+device['node'],device['path'])
            selected=config.get(role)
            index=next((i for i,d in enumerate(devices) if selected and Path(d['path']).resolve()==Path(selected).resolve()),-1)
            combo.setCurrentIndex(index if index>=0 else min(1 if role=='overview' else 0,combo.count()-1))
            self.choices[role]=combo;layout.addWidget(combo)
        self.support=W.QCheckBox('机械臂已抱紧或可靠固定，没有运动、示教或采集在进行')
        layout.addWidget(self.support)
        self.note=W.QLabel();self.note.setWordWrap(True);layout.addWidget(self.note)
        row=W.QHBoxLayout();cancel=W.QPushButton('取消');cancel.clicked.connect(self.reject)
        self.apply=W.QPushButton('应用并检查相机');self.apply.clicked.connect(self.accept)
        row.addWidget(cancel);row.addWidget(self.apply)
        if emergency:
            stop=W.QPushButton('软件急停');stop.setObjectName('danger');stop.clicked.connect(lambda:(emergency(),self.reject()));row.addWidget(stop)
        layout.addLayout(row)
        self.mode.currentIndexChanged.connect(self.update_options);self.support.stateChanged.connect(self.update_options)
        for combo in self.choices.values():combo.currentIndexChanged.connect(self.update_options)
        self.update_options()
    def update_options(self):
        dual=self.mode.currentIndex()==1
        self.choices['overview'].setEnabled(dual)
        enough=self.choices['wrist'].count()>=(2 if dual else 1)
        distinct=not dual or self.choices['wrist'].currentData()!=self.choices['overview'].currentData()
        self.apply.setEnabled(enough and distinct and self.support.isChecked())
        self.note.setText('双摄需要两台不同的RGB相机；当前设备不足或重复选择。' if not enough or not distinct else '应用失败会恢复原配置；采集中不能切换模式。')
    def configuration(self):
        return {'mode':'dual' if self.mode.currentIndex() else 'single',
                **{role:combo.currentData() for role,combo in self.choices.items()}}
