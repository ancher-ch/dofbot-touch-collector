#!/usr/bin/env python3
"""Native 1280x800 touchscreen UI. Startup is read-only. Run with --demo for no I2C."""
import argparse
import logging
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

from PyQt5 import QtCore, QtGui, QtWidgets as W, QtNetwork
from .core import Engine, Stopped, home_plan, teaching_plan
from .hardware import RealRobot, DemoRobot, DemoCamera
from . import __version__
from .safety import EmergencyLatch
from .multicamera import discover_devices,validate_config,make_rig
from .storage import load
from trajectory_teach import save_json


BASE = Path(__file__).resolve().parent.parent
REAL_ROOT = Path.home() / "dofbot-data/touch_collector"
STAGES = {"home_setup": "手动设置初始位置", "ready": "准备新示教", "prepare": "示教准备 · 松力", "teaching": "记录关节轨迹",
          "teach_review": "审核人工示教", "saved_teach": "人工示教已保存", "capture_review": "审核回放采集"}
HOLDING = {"unknown": "抱紧状态待确认", "free": "松力 · 请持续支撑", "pending": "抱紧指令已发 · 待现场确认", "confirmed": "已记录现场抱紧确认"}


def factories():
    found = []
    proc = Path("/proc")
    if not proc.exists():
        return found
    for p in proc.iterdir():
        if not p.name.isdigit():
            continue
        try:
            args = [x.decode(errors="replace") for x in (p / "cmdline").read_bytes().split(b"\0") if x]
            for name in ("YahboomArm.pyc", "joystick.py"):
                target = str(Path.home() / "dofbot_pro/APP_DOFBOT_PRO" / name)
                if target in args:
                    found.append((int(p.name), target))
        except OSError:
            pass
    return found


class Worker:
    def __init__(self, root, demo):
        self.root = root
        self.demo = demo
        self.engine = None
        self.error = "正在连接设备……"
        self.pending = False
        self.commands = queue.Queue(maxsize=1)
        self.quit = threading.Event()
        self.lockfile = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.audited = False
        self.viewing = threading.Event()
        self.safety=EmergencyLatch(root/'safety.json')
        self.camera_config={'mode':'single','wrist':'/dev/video0','overview':None}

    def camera_devices(self):
        return ([{'path':f'/dev/video{i*2}','node':f'/dev/video{i*2}','card':f'模拟RGB {i+1}'} for i in range(2)]
                if self.demo else discover_devices())

    def start(self):
        self.thread.start()

    def connect(self):
        if not self.demo:
            owners = factories()
            if owners:
                raise RuntimeError("出厂APP或手柄程序正在占用设备。可点“接管出厂控制”，关闭它们后重试。")
            import fcntl
            self.lockfile = (BASE / ".teleop_record.lock").open("a")
            try:
                fcntl.flock(self.lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self.lockfile.close()
                self.lockfile = None
                raise RuntimeError("另一个采集或控制程序正在运行，请先退出该程序，再点重新连接。")
        robot = None
        camera = None
        try:
            import cv2
            robot = DemoRobot() if self.demo else RealRobot()
            settings=self.root/'camera_config.json'
            if settings.exists():self.camera_config=validate_config(load(settings))
            else:
                devices=self.camera_devices()
                if devices:self.camera_config['wrist']=devices[0]['path']
            camera=make_rig(cv2,self.camera_config,self.demo)
            camera.start()
            session = self.root / "demo_session.json" if self.demo else BASE / ".learning_session.json"
            self.engine = Engine(robot, self.root, session, camera, cv2, self.demo,safety=self.safety)
            self.error = ""
        except BaseException:
            if camera:
                camera.close()
            if robot:
                robot.close()
            self.engine = None
            if self.lockfile:
                self.lockfile.close()
                self.lockfile = None
            raise

    def submit(self, name, **kwargs):
        if self.pending or self.quit.is_set() or (self.viewing.is_set() and name != "end_viewing"):
            return False
        if self.thread.ident is not None and not self.thread.is_alive():
            return False
        self.pending = True
        try:
            self.commands.put_nowait((name, kwargs))
            return True
        except queue.Full:
            self.pending = False
            return False

    def stop(self):
        if self.engine:
            self.engine.stop.set()

    def emergency_stop(self):
        if self.engine:self.engine.stop.set()
        self.safety.trip()
        if self.engine:self.engine.event('software_emergency_latched')

    def configure_camera(self,config,confirmed=False):
        if confirmed is not True:raise RuntimeError('切换前需要确认机械臂已固定。')
        e=self.engine;e._idle()
        if e.holding in ('free','pending'):raise RuntimeError('先结束松力操作并完成抱紧确认，再切换相机。')
        config=validate_config(config,self.camera_devices())
        old=self.camera_config.copy()
        if e.camera:e.camera.close()
        candidate=None
        try:
            candidate=make_rig(e.cv2,config,self.demo)
            e.camera=candidate  # retain ownership even if start/cleanup fails
            candidate.start()
            deadline=time.monotonic()+6
            while not candidate.status()['ready']:
                if self.quit.is_set() or e.stop.is_set():raise RuntimeError('已取消相机切换。')
                if candidate.status().get('error') or time.monotonic()>deadline:raise RuntimeError('新相机未通过帧率/连接检查。')
                time.sleep(.05)
            save_json(self.root/'camera_config.json',config)
            e.camera=candidate;self.camera_config=config
            e.preview=None;e.preview_frames={};e.preview_errors={};e.camera_error='';e.message='相机模式已切换；未发送机械臂指令。'
        except BaseException as error:
            if candidate:
                try:candidate.close()
                except Exception as close_error:
                    e.camera=candidate
                    e.camera_error='相机线程未释放，保持控制锁定；请退出软件后检查相机。'
                    raise RuntimeError(f'相机切换失败：{error}；资源未释放：{close_error}') from error
            e.camera=make_rig(e.cv2,old,self.demo);e.camera.start()
            self.camera_config=old
            raise

    def command(self, name, kwargs):
        if name == "end_viewing":
            self.viewing.clear()
            return
        if self.viewing.is_set():
            raise RuntimeError("正在观看视频，请先关闭观看窗口。")
        if name == "begin_viewing":
            if kwargs.get("confirmed") is not True:
                raise RuntimeError("观看前请先确认机械臂已固定。")
            if self.engine:
                self.engine._idle()
                if self.engine.stage in ("teaching","home_setup") or self.engine.holding in ("free","pending"):
                    raise RuntimeError("请先结束示教并恢复抱紧、完成现场确认，再观看视频。")
            self.viewing.set()
            return
        if name == "connect":
            if not self.engine:
                self.connect()
            return
        if name == "takeover":
            if self.demo or self.engine:
                return
            # Called only after an explicit touch confirmation. Never use a shell/pkill.
            for pid, script in factories():
                p = Path("/proc") / str(pid)
                args = (p / "cmdline").read_bytes().split(b"\0")
                if script.encode() not in args:
                    raise RuntimeError("出厂进程已改变，未关闭。")
                os.kill(pid, signal.SIGSTOP)
                os.kill(pid, signal.SIGKILL)
            time.sleep(.2)
            self.connect()
            return
        e = self.engine
        if e is None:
            raise RuntimeError("设备未连接。")
        if name == "task":
            # Metadata-only: don't clear a pending stop or alter actuator state.
            e.set_task(kwargs["task"])
            return
        if name not in ('check_feedback','reset_emergency','task'):
            self.safety.require_clear()
        e.stop.clear()
        e.error = ""
        e.feedback_command_failed = False
        if name == "demo_nudge":
            if self.demo and e.holding == "free":
                j = kwargs["joint"]
                maximum = 270 if j == 4 else 180
                e.robot.pose[j] = max(0, min(maximum, e.robot.pose[j] + kwargs["delta"]))
        elif name == "camera":
            self.configure_camera(self.camera_config,True)
        elif name=='configure_camera':
            self.configure_camera(**kwargs)
        else:
            getattr(e, name)(**kwargs)

    def run(self):
        try:
            self.connect()
        except Exception as error:
            self.error = str(error)
        next_tick = time.monotonic()
        while not self.quit.is_set():
            try:
                name, kwargs = self.commands.get(timeout=max(0, min(.05, next_tick-time.monotonic())))
            except queue.Empty:
                pass
            else:
                try:
                    self.command(name, kwargs)
                except Exception as error:
                    logging.exception("Touch collector command failed: %s",name)
                    if self.engine:
                        self.engine.fail(error)
                    else:
                        self.error = str(error)
                finally:
                    self.pending = False
                    next_tick = max(next_tick,time.monotonic())
            if self.engine and time.monotonic() >= next_tick:
                try:
                    if self.engine.stop.is_set():
                        if self.engine.stage == "teaching":
                            if self.safety.tripped.is_set():self.engine.fail(Stopped('软件急停，示教中断，草稿保留。'))
                            else:self.engine.end_teaching()
                        self.engine.message = "已停止后续运动。若仍松力，请保留支撑，使用当前位置抱紧。"
                        self.engine.stop.clear()
                    self.engine.poll()
                    if not self.audited and self.engine.valid:
                        from trajectory_teach import save_json
                        r = self.engine.robot
                        save_json(self.engine.store.root/"reports"/f"connection_{os.getpid()}.json",{
                            "simulated":self.demo,"pose":self.engine.pose,"holding":self.engine.holding,
                            "position_requests":getattr(r,"position_requests",len(getattr(r,"writes",[]))),
                            "torque_requests":getattr(r,"torque_requests",len(getattr(r,"torques",[]))),
                            "startup_policy":"read_only_no_automatic_motion"})
                        self.audited = True
                except Exception as error:
                    self.engine.fail(error)
                next_tick += .1
                if time.monotonic()-next_tick > .2:
                    if self.engine.stage == "teaching":
                        # Observation-only teaching uses actual read timestamps.
                        # Let teaching_tick enforce its 500 ms gap gate rather
                        # than applying the powered replay's tighter deadline.
                        self.engine.event("teaching_schedule_late",delay_ms=round(1000*(time.monotonic()-next_tick)))
                    next_tick = time.monotonic()+.1
        if self.engine:
            self.engine.close()
        if self.lockfile:
            self.lockfile.close()

    def close(self):
        self.stop()
        self.quit.set()
        self.thread.join(timeout=5)


STYLE = """
QWidget {font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif; font-size:14px; color:#172e3c;}
QMainWindow, #surface {background:#f2f5f4;}
#header {background:white; border-bottom:1px solid #dce4e3;}
#nav {background:#172f3e;} #nav QLabel {color:#a9c4cc;} #nav #brand {color:white;font-size:21px;font-weight:700;}
#title {font-size:24px;font-weight:700;} #subtitle {color:#637a84;font-size:13px;}
#badge {background:#e1eee9;color:#176c55;border-radius:12px;padding:8px 14px;}
#card, #joint {background:white;border:1px solid #dce5e4;border-radius:14px;}
#jointValue {font-size:29px;font-weight:700;color:#1f5261;}
#jointName {color:#607b87;font-size:12px;} #homeText {font-size:13px;color:#506a75;}
#warning {background:#fff0d8;color:#835512;border-radius:10px;padding:12px;font-size:13px;}
#message {color:#405e6a;font-size:14px;line-height:1.5;}
QPushButton {background:#fff;border:1px solid #cbdad8;border-radius:10px;padding:10px 14px;min-height:29px;font-weight:600;}
QPushButton:hover {background:#eaf3f0;} QPushButton:pressed {background:#d3e8df;}
QPushButton:disabled {color:#97a8ab;background:#edf1f1;border-color:#e0e6e5;}
QPushButton#primary {background:#176f5a;color:white;border-color:#176f5a;}
QPushButton#primary:hover {background:#125b49;} QPushButton#primary:disabled {background:#9db9b1;color:#eff5f2;border-color:#9db9b1;}
QPushButton#danger {background:#b54439;color:white;border:none;} QPushButton#danger:hover {background:#97382f;}
QPushButton#navButton {background:#223f50;color:#e9f1f1;border:none;text-align:left;}
QPushButton#navButton:disabled {background:#223f50;color:#a8bec5;}
QPushButton#navButton[active="true"] {background:#305b61;color:white;}
QListWidget {background:transparent;color:#c7d9de;border:none;outline:none;font-size:12px;}
QListWidget::item {padding:12px 6px;border-bottom:1px solid #304c5a;}
QListWidget::item:selected {background:#2a5a61;border-radius:7px;color:white;}
QComboBox {background:white;border:1px solid #ccdad7;border-radius:8px;padding:9px;min-height:25px;}
QComboBox QAbstractItemView {min-height:140px;selection-background-color:#d5e7e0;}
QComboBox QAbstractItemView::item {min-height:40px;}
QProgressBar {border:none;background:#e3ece8;border-radius:4px;height:7px;text-align:center;}
QProgressBar::chunk {background:#2a9277;border-radius:4px;}
QDialog {background:#f7f9f8;} QCheckBox {spacing:14px;font-size:16px;}
QCheckBox::indicator {width:28px;height:28px;}
#footer {font-size:12px;color:#647d87;}
"""


def label(text="", name=None, wrap=False):
    obj = W.QLabel(text)
    if name:
        obj.setObjectName(name)
    obj.setWordWrap(wrap)
    return obj

class EmergencyKeyFilter(QtCore.QObject):
    def __init__(self,worker,parent):super().__init__(parent);self.worker=worker
    def eventFilter(self,obj,event):
        if event.type()==QtCore.QEvent.KeyPress and event.key()==QtCore.Qt.Key_F12:
            self.parent().emergency_stop()
            modal=W.QApplication.activeModalWidget()
            if modal:modal.setWindowTitle('软件急停已锁定 · 故障排除后返回主界面复位')
            return True
        return False


class Window(W.QMainWindow):
    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self.last_library = None
        self.last_preview = None
        self.replay_gripper_extra = 1
        self.replay_joint3_offset = 0
        self.setWindowTitle("DOFBOT 数据采集台 v" + __version__ + (" · 模拟模式" if worker.demo else ""))
        self.resize(1280,800)
        self.setMinimumSize(1120,740)
        self.setStyleSheet(STYLE)
        self.emergency_key=EmergencyKeyFilter(worker,self)
        W.QApplication.instance().installEventFilter(self.emergency_key)
        surface = W.QWidget(objectName="surface")
        self.setCentralWidget(surface)
        root = W.QVBoxLayout(surface)
        root.setContentsMargins(0,0,0,0)
        root.setSpacing(0)
        head = W.QFrame(objectName="header")
        h = W.QHBoxLayout(head)
        h.setContentsMargins(24,12,24,12)
        h.addWidget(label("◈  DOFBOT  /  数据采集台  v" + __version__, "title"))
        h.addStretch()
        self.connection = label("正在连接", "badge")
        h.addWidget(self.connection)
        stop = W.QPushButton("■  停止动作", objectName="danger")
        stop.setMinimumWidth(145)
        stop.setToolTip("停止后续指令，不自动松力；已发出的100ms动作可能继续完成。")
        stop.clicked.connect(worker.stop)
        h.addWidget(stop)
        emergency=W.QPushButton('软件急停 · F12',objectName='danger')
        emergency.clicked.connect(self.emergency_stop);h.addWidget(emergency)
        self.reset_stop_btn=W.QPushButton('复位');self.reset_stop_btn.clicked.connect(self.reset_emergency);h.addWidget(self.reset_stop_btn)
        exit_button = W.QPushButton("退出")
        exit_button.clicked.connect(self.close)
        h.addWidget(exit_button)
        root.addWidget(head)
        body = W.QHBoxLayout()
        body.setSpacing(0)
        root.addLayout(body,1)
        nav = W.QFrame(objectName="nav")
        nav.setFixedWidth(210)
        n = W.QVBoxLayout(nav)
        n.setContentsMargins(18,24,18,16)
        n.setSpacing(13)
        n.addWidget(label("本机采集", "brand"))
        n.addWidget(label("先示教，再复现与采集", wrap=True))
        self.step_buttons = []
        for text in ("01   人工示教", "02   审核轨迹", "03   回放采集"):
            b = W.QPushButton(text, objectName="navButton")
            b.setEnabled(False)
            n.addWidget(b)
            self.step_buttons.append(b)
        self.saved_videos_btn = W.QPushButton("04   已保存视频", objectName="navButton")
        self.saved_videos_btn.clicked.connect(self.saved_videos)
        n.addWidget(self.saved_videos_btn)
        self.manager_btn=W.QPushButton('05   数据管理',objectName='navButton')
        self.manager_btn.clicked.connect(lambda:self.open_viewer(management=True));n.addWidget(self.manager_btn)
        n.addSpacing(12)
        n.addWidget(label("已保存的人工示教"))
        self.library = W.QListWidget()
        self.library.itemClicked.connect(self.choose)
        n.addWidget(self.library,1)
        self.counts = label("0 条完整数据", wrap=True)
        n.addWidget(self.counts)
        n.addWidget(label("离线运行 · 数据保存在 cyh", wrap=True))
        body.addWidget(nav)
        main = W.QWidget()
        m = W.QVBoxLayout(main)
        m.setContentsMargins(22,18,22,12)
        m.setSpacing(10)
        row = W.QHBoxLayout()
        titles = W.QVBoxLayout()
        self.title = label("准备新示教", "title")
        titles.addWidget(self.title)
        titles.addWidget(label("关节示教不存视频 · 自动回放同步记录视频与动作", "subtitle"))
        row.addLayout(titles)
        row.addStretch()
        self.mode = label("状态待确认", "badge")
        row.addWidget(self.mode)
        m.addLayout(row)
        joints = W.QHBoxLayout()
        joints.setSpacing(8)
        self.values, self.raw_labels = [], []
        for i, name in enumerate(("底座", "肩部", "肘部", "腕俯仰", "腕旋转", "夹爪")):
            card = W.QFrame(objectName="joint")
            v = W.QVBoxLayout(card)
            v.setContentsMargins(12,8,12,8)
            v.setSpacing(0)
            v.addWidget(label(f"J{i+1}  {name}", "jointName"))
            val = label("—", "jointValue")
            raw = label("等待读取", "jointName")
            v.addWidget(val)
            v.addWidget(raw)
            self.values.append(val)
            self.raw_labels.append(raw)
            joints.addWidget(card,1)
        m.addLayout(joints)
        self.warning = label("启动仅查看状态，不会自动改变扭矩或运动。", "warning", True)
        m.addWidget(self.warning)
        columns = W.QHBoxLayout()
        columns.setSpacing(16)
        left_card = W.QFrame(objectName="card")
        l = W.QVBoxLayout(left_card)
        l.setContentsMargins(14,12,14,12)
        camera_head = W.QHBoxLayout()
        camera_head.addWidget(label("相机预览", "subtitle"))
        camera_head.addStretch()
        reconnect = W.QPushButton("重连")
        self.camera_reconnect = reconnect
        reconnect.setMaximumWidth(80)
        reconnect.clicked.connect(lambda: self.send("camera"))
        camera_head.addWidget(reconnect)
        self.camera_settings=W.QPushButton('单/双摄')
        self.camera_settings.clicked.connect(self.camera_dialog);camera_head.addWidget(self.camera_settings)
        l.addLayout(camera_head)
        views=W.QHBoxLayout();views.setSpacing(8)
        self.preview_panels={};self.preview_labels={};self.preview_titles={}
        for role,title in (('wrist','主相机 / 腕部'),('overview','第二相机 / 外部')):
            panel=W.QWidget();layout=W.QVBoxLayout(panel);layout.setContentsMargins(0,0,0,0);layout.setSpacing(4)
            heading=label(title,'subtitle');layout.addWidget(heading)
            preview=label('等待相机画面',wrap=True);preview.setAlignment(QtCore.Qt.AlignCenter)
            preview.setMinimumSize(130,100)
            preview.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Ignored)
            preview.setStyleSheet('background:#e4ece9;border-radius:10px;color:#67818a;')
            layout.addWidget(preview,1);views.addWidget(panel,1)
            self.preview_panels[role]=panel;self.preview_labels[role]=preview;self.preview_titles[role]=heading
        self.preview=self.preview_labels['wrist']
        self.preview_panels['overview'].hide()
        l.addLayout(views,1)
        self.camera_note = label("预览不会保存；只在自动回放阶段采集RGB。", "subtitle", True)
        l.addWidget(self.camera_note)
        self.home_text = label("初始状态：尚未设置", "homeText", True)
        l.addWidget(self.home_text)
        home_buttons = W.QHBoxLayout()
        self.set_home_btn = W.QPushButton("手动设置初始位置")
        self.set_home_btn.clicked.connect(self.set_home)
        self.home_btn = W.QPushButton("快速回初始")
        self.home_btn.clicked.connect(self.home)
        home_buttons.addWidget(self.set_home_btn)
        home_buttons.addWidget(self.home_btn)
        l.addLayout(home_buttons)
        columns.addWidget(left_card, 10)
        controls = W.QFrame(objectName="card")
        c = W.QVBoxLayout(controls)
        c.setContentsMargins(18,15,18,15)
        c.setSpacing(8)
        self.task = W.QComboBox()
        from .tasks import DEFAULT_TASKS
        self.task.addItems(DEFAULT_TASKS)
        self.task.setMinimumWidth(0)
        self.task.setSizePolicy(W.QSizePolicy.Expanding, W.QSizePolicy.Fixed)
        self.task.setSizeAdjustPolicy(W.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.task.setMinimumContentsLength(10)
        self.task.activated[str].connect(lambda text: self.send("task", task=text))
        self.task_edit = W.QPushButton("编辑任务")
        self.task_edit.clicked.connect(self.edit_task)
        task_row = W.QHBoxLayout()
        task_row.addWidget(self.task, 1)
        task_row.addWidget(self.task_edit)
        c.addLayout(task_row)
        self.message = label("正在连接设备……", "message", True)
        self.message.setMinimumHeight(44)
        c.addWidget(self.message)
        self.stats = label("尚未开始记录", "subtitle")
        c.addWidget(self.stats)
        self.progress = W.QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setMaximumHeight(8)
        c.addWidget(self.progress)
        self.actions = {}
        action_grid = W.QGridLayout()
        action_grid.setSpacing(8)
        definitions = [
            ("finish_home", "保存初始并抱紧", self.finish_home, "primary"),
            ("cancel_home", "取消初始位置设置", self.cancel_home, None),
            ("release", "进入手动示教", self.release, "primary"),
            ("begin", "●  开始记录关节", lambda: self.send("begin_teaching"), "primary"),
            ("end", "■  结束示教", lambda: self.send("end_teaching"), "primary"),
            ("save_teach", "保存人工示教", self.save_teaching, "primary"),
            ("replay", "回本条起点并回放采集", self.replay, "primary"),
            ("save_capture", "保存此次数据", self.save_capture, "primary"),
            ("rerecord", "重新播放并记录", self.replay, None),
            ("delete", "删除此人工示教 / 重录", self.discard, None),
            ("next", "下一条人工示教", lambda: self.send("next_teaching"), None),
            ("preview_take", "预览本次数据", self.preview_take, None),
        ]
        for key, text, action, style in definitions:
            b = W.QPushButton(text)
            if style:
                b.setObjectName(style)
            b.clicked.connect(action)
            self.actions[key] = b
            positions = {"cancel_home":(1,0,2), "rerecord":(1,0,1), "delete":(1,1,1), "next":(2,0,1), "preview_take":(2,1,1)}
            row,col,span = positions.get(key,(0,0,2))
            action_grid.addWidget(b,row,col,1,span)
        c.addLayout(action_grid)
        c.addStretch()
        self.hold_button = W.QPushButton("当前位置抱紧")
        self.hold_button.clicked.connect(self.hold)
        self.feedback_button = W.QPushButton("重新检查反馈")
        self.feedback_button.clicked.connect(lambda:self.send("check_feedback"))
        self.confirm_button = W.QPushButton("确认已抱紧")
        self.confirm_button.clicked.connect(self.confirm_holding)
        recovery_row = W.QHBoxLayout()
        recovery_row.addWidget(self.hold_button)
        recovery_row.addWidget(self.feedback_button)
        c.addLayout(recovery_row)
        self.recover_button = W.QPushButton("支撑后重新松力调整")
        self.recover_button.clicked.connect(self.recover_manual)
        c.addWidget(self.recover_button)
        c.addWidget(self.confirm_button)
        self.connection_button = W.QPushButton("重新连接设备")
        self.connection_button.clicked.connect(lambda: self.send("connect"))
        self.takeover_button = W.QPushButton("接管出厂控制")
        self.takeover_button.clicked.connect(self.takeover)
        c.addWidget(self.connection_button)
        c.addWidget(self.takeover_button)
        columns.addWidget(controls, 11)
        m.addLayout(columns,1)
        if worker.demo:
            demo_row = W.QHBoxLayout()
            demo_row.addWidget(label("模拟拖动", "subtitle"))
            for j in range(6):
                for delta in (-3,3):
                    b = W.QPushButton(f"J{j+1}{'−' if delta<0 else '+'}")
                    b.setMaximumWidth(58)
                    b.clicked.connect(lambda _, j=j,d=delta:self.send("demo_nudge", joint=j,delta=d))
                    demo_row.addWidget(b)
            m.addLayout(demo_row)
        self.footer = label("数据保留在本机；删除移入回收站。", "footer")
        m.addWidget(self.footer)
        body.addWidget(main,1)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_state)
        self.timer.start(200)
        self.update_state()

    def state(self):
        return self.worker.engine.snapshot() if self.worker.engine else None

    def emergency_stop(self):
        self.worker.emergency_stop();self.update_state()

    def reset_emergency(self):
        if self.confirm('手动复位软件急停','软件急停不切断电源，也不自动松力或上力。请先排除故障；复位不会自动继续之前的动作。',
                        ['故障已排除，机械臂已抱紧或可靠支撑，手避开夹缝']):
            self.send('reset_emergency',confirmed=True)

    def camera_dialog(self):
        from .camera_ui import CameraDialog
        dialog=CameraDialog(self.worker.camera_devices(),self.worker.camera_config,self,self.emergency_stop)
        if dialog.exec_()==W.QDialog.Accepted:
            self.send('configure_camera',config=dialog.configuration(),confirmed=True)

    def send(self, name, **kwargs):
        if self.worker.submit(name, **kwargs):
            self.update_state()

    def edit_task(self):
        from .task_ui import TaskDialog
        state = self.state()
        if not state or not self.task_edit.isEnabled():
            return
        dialog = TaskDialog(state["task"], self)
        if dialog.exec_() == W.QDialog.Accepted:
            self.send("task", task=dialog.name())

    def confirm(self, title, text, checks, accept_text="确认执行"):
        dialog = W.QDialog(self)
        dialog.setWindowTitle(title)
        dialog.setMinimumWidth(580)
        layout = W.QVBoxLayout(dialog)
        layout.setContentsMargins(28,24,28,24)
        layout.setSpacing(18)
        layout.addWidget(label(title, "title"))
        layout.addWidget(label(text, wrap=True))
        boxes = []
        for check in checks:
            box = W.QCheckBox(check)
            boxes.append(box)
            layout.addWidget(box)
        buttons = W.QHBoxLayout()
        cancel = W.QPushButton("取消")
        accept = W.QPushButton(accept_text, objectName="primary")
        accept.setEnabled(not boxes)
        cancel.clicked.connect(dialog.reject)
        accept.clicked.connect(dialog.accept)
        for box in boxes:
            box.stateChanged.connect(lambda: accept.setEnabled(all(b.isChecked() for b in boxes)))
        buttons.addWidget(cancel)
        buttons.addWidget(accept)
        emergency=W.QPushButton('软件急停',objectName='danger')
        emergency.clicked.connect(lambda:(self.emergency_stop(),dialog.reject()))
        buttons.addWidget(emergency)
        layout.addLayout(buttons)
        return dialog.exec_() == W.QDialog.Accepted

    def set_home(self):
        if self.confirm("手动设置初始位置", "先松力，供你手动摆出合适的初始姿态。无需预先设置起点。\n全臂及夹爪将松力，没有重力补偿；此阶段不记录示教或视频。原初始位置在完成前保留。", ["肩肘和末端已可靠支撑，夹爪为空", "我将持续支撑，不强掰关节或拖过角度边界"]):
            self.send("begin_home_setup", confirmed=True)

    def finish_home(self):
        s = self.state()
        if s and self.confirm("保存初始并抱紧", f"准备保存当前姿态：\n{s['pose']}\n先在当前位置恢复扭矩；反馈检查通过、现场确认已抱紧后才保存。不会回到旧起点。", ["姿态已经摆好，支撑稳固、夹爪为空、手避开夹缝"]):
            self.send("finish_home_setup", confirmed=True)

    def cancel_home(self):
        if self.confirm("取消初始位置设置", "保留原初始位置，不自动上力或松力。若仍松力，取消后请继续支撑并点当前位置抱紧。", []):
            self.send("cancel_home_setup")

    def release(self):
        if self.confirm("进入手动示教", "全臂及夹爪将松力，没有重力补偿。相机仅预览，不保存视频。松力后先摆稳，再点开始记录。", ["肩肘和末端已可靠支撑，夹爪为空", "我将持续支撑，不强掰关节或拖过角度边界"]):
            self.send("release", confirmed=True)

    def hold(self):
        s = self.state()
        if s and (not s["valid"] or s["age_s"] >= .6):
            if self.confirm("反馈异常时尝试抱紧", "将只发送厂商的当前位置扭矩开启指令，不发送角度目标、不回原点。\n无法验证当前角度，可能出现位置调整；需现场判断是否抱紧。正在记录的示教会中断，初始位置设置会取消。读数恢复前不能回放。", ["机械臂已可靠支撑，夹爪为空", "手已避开夹缝，我能在现场观察并继续保留支撑"]):
                self.send("emergency_hold_here", confirmed=True)
            return
        if self.confirm("当前位置恢复抱紧", "将保持当前位置恢复扭矩，不回原点。可能有轻微调整；指令后仍需现场确认已抱紧。", ["支撑稳固、夹爪为空、手避开夹缝"]):
            self.send("hold_here", confirmed=True)

    def confirm_holding(self):
        if self.confirm("确认现场状态", "软件无法读取实际扭矩状态。请根据现场已有感受判断，保留支撑，不撤掉支撑来测试。", ["我已确认关节实际抱紧、姿态稳定"]):
            self.send("confirm_holding", confirmed=True)

    def recover_manual(self):
        if self.confirm("支撑后重新松力调整", "这会让全臂和夹爪重新松力，没有重力补偿。\n用于抱紧后角度越界、无法确认的恢复；不会发送位置目标或自动回位。\n松力后轻柔调整异常关节，看到有效角度并留出余量后，再点当前位置抱紧。若有机械阻挡，不要硬掰。", ["肩肘和末端已可靠支撑，夹爪为空", "我准备好持续承托，手避开夹缝，允许现在重新松力"]):
            self.send("recover_manual", confirmed=True)

    def save_teaching(self):
        s=self.state()
        if not s:
            return
        warnings=s["quality_warnings"]
        reviewed=False
        if warnings:
            largest=max(max(w["deltas"]) for w in warnings)
            if not self.confirm("审核快速手动变化",f"本条有{len(warnings)}次较快变化，最大相邻变化{largest:.0f}°。\n原始角度和时间保留。自动回放会逐段限速，速度不同于手拖。若是读数异常，请取消并删除重录。",["我已核实这些变化对应真实手动动作，允许保存待回放"]):
                return
            reviewed=True
        self.send("save_teaching",reviewed=reviewed)

    def motion_confirm(self, replay, gripper_extra_degrees=0, joint3_offset_degrees=0):
        s = self.state()
        target=s["teaching_start"] if s and replay else (s["home"] if s else None)
        if not s or not s["valid"] or target is None:
            return False
        try:
            if replay:
                plan,_=teaching_plan(self.worker.engine.teaching,gripper_extra_degrees,joint3_offset_degrees)
                target=plan[0]
            return_plan = home_plan(s["pose"], target)
        except (ValueError, RuntimeError) as error:
            self.confirm("回位计划不能执行", str(error), [])
            return False
        return_seconds = (len(return_plan)-1)/10
        text = f"当前：{s['pose']}\n{'本条示教起点' if replay else '初始位置'}：{target}\n同步平滑回位：预计运动约{return_seconds:.1f}秒，按距离自动调整；3秒倒计时另计。\n超过5秒会继续按计划回位，不因此取消；反馈异常或停止操作仍会中断。\n软件限速30°/s不是硬件额定值，沿途需清空；本软件不能避障。"
        if replay:
            text += f"\n夹爪夹紧补偿：{gripper_extra_degrees}°；最大张开角保持，最闭合处增加所选角度，中间平滑过渡。"
            text += f"\nJ3位置微调：{joint3_offset_degrees:+d}°，作用于本条回放及其起点；不修改全局初始位置。正负不是固定的上下方向，不是力矩重力补偿。"
            text += "\n回位后再次倒计时，再自动执行示教并录视频。回位路径应与方块、盒子留出空间。"
        return self.confirm("回初始并回放采集" if replay else "回到初始状态", text,
                            ["已抱紧，手和支撑已退出整条运动路径", "沿途无遮挡，线缆有余量；回位时夹爪为空"])

    def home(self):
        if self.motion_confirm(False):
            self.send("return_home", confirmed=True)

    def replay(self):
        dialog=W.QDialog(self)
        dialog.setWindowTitle("回放夹爪补偿")
        dialog.setMinimumWidth(580)
        layout=W.QVBoxLayout(dialog)
        layout.setContentsMargins(28,24,28,24)
        layout.setSpacing(18)
        layout.addWidget(label("回放夹爪补偿", "title"))
        description=label("建议先试1°。示教原始角度保留，补偿只用于本次回放；最大张开角不变，最闭合处增加所选角度，中间平滑过渡。\n这是位置补偿，没有力反馈；若持续嗡鸣、物体变形或异常发热，请停止，不要继续增加。",wrap=True)
        description.setObjectName("compensationDescription")
        description.setMinimumHeight(110)
        layout.addWidget(description)
        choice=W.QComboBox()
        choice.setObjectName("gripperCompensation")
        choice.addItems(["0° · 原始示教角度","1° · 夹紧多1°（建议先试）",*[f"{n}° · 夹紧多{n}°" for n in range(2,6)]])
        choice.setCurrentIndex(self.replay_gripper_extra)
        layout.addWidget(choice)
        layout.addWidget(label("J3位置微调（不是力矩重力补偿）",wrap=True))
        joint3=W.QComboBox()
        joint3.setObjectName("joint3Correction")
        for offset in range(-3,4):
            joint3.addItem("0° · 不微调（默认）" if not offset else f"{offset:+d}° · J3目标角度微调",offset)
        joint3.setCurrentIndex(joint3.findData(self.replay_joint3_offset))
        layout.addWidget(joint3)
        layout.addWidget(label("J3仅改变回放目标角度，原始示教保留；正负方向需要现场验证。不能靠微调消除舵机不响应或机构卡住。",wrap=True))
        buttons=W.QHBoxLayout()
        cancel=W.QPushButton("取消")
        accept=W.QPushButton("下一步：检查回放路径",objectName="primary")
        cancel.clicked.connect(dialog.reject);accept.clicked.connect(dialog.accept)
        buttons.addWidget(cancel);buttons.addWidget(accept);layout.addLayout(buttons)
        emergency=W.QPushButton('软件急停',objectName='danger');emergency.clicked.connect(lambda:(self.emergency_stop(),dialog.reject()));buttons.addWidget(emergency)
        if dialog.exec_()!=W.QDialog.Accepted:
            return
        extra=choice.currentIndex()
        self.replay_gripper_extra=extra
        offset=joint3.currentData()
        self.replay_joint3_offset=offset
        if self.motion_confirm(True,extra,offset):
            self.send("replay_capture", confirmed=True,gripper_extra_degrees=extra,joint3_offset_degrees=offset)

    def discard(self):
        if self.confirm("删除并重新示教", "所选人工示教及当前未保存采集将移入回收站。此前已保存的完整数据保留。此操作不改变扭矩。", []):
            self.send("discard_teaching")

    def save_capture(self):
        dialog = W.QDialog(self)
        dialog.setWindowTitle("保存此次数据")
        dialog.setMinimumWidth(550)
        v = W.QVBoxLayout(dialog)
        v.setContentsMargins(28,25,28,25)
        v.setSpacing(18)
        v.addWidget(label("现场任务结果", "title"))
        v.addWidget(label("视频与动作检查通过不代表抓取成功。请选择你实际观察到的任务结果。", wrap=True))
        options = W.QComboBox()
        options.addItems(["未标注 / 稍后检查", "成功：实际完成任务且无碰撞", "失败：未完成任务"])
        v.addWidget(options)
        cancel, save = W.QPushButton("取消"), W.QPushButton("保存到本机", objectName="primary")
        cancel.clicked.connect(dialog.reject)
        save.clicked.connect(dialog.accept)
        row = W.QHBoxLayout()
        row.addWidget(cancel)
        row.addWidget(save)
        emergency=W.QPushButton('软件急停',objectName='danger');emergency.clicked.connect(lambda:(self.emergency_stop(),dialog.reject()));row.addWidget(emergency)
        v.addLayout(row)
        if dialog.exec_() == W.QDialog.Accepted:
            self.send("save_capture", outcome=[None,True,False][options.currentIndex()])

    def choose(self, item):
        if self.worker.pending:
            return
        s = self.state()
        if s and s["stage"] in ("ready", "saved_teach", "capture_review"):
            self.send("select_teaching", item_id=item.data(QtCore.Qt.UserRole))

    def preview_take(self):
        engine = self.worker.engine
        if not engine or not engine.capture:
            return
        self.open_viewer(engine.capture.path)

    def saved_videos(self):
        self.open_viewer()

    def open_viewer(self, current=None,management=False):
        s = self.state()
        if self.worker.pending or self.worker.viewing.is_set():
            return
        if s and (s["activity"] != "idle" or s["stage"] in ("teaching","home_setup") or s["holding"] in ("free","pending")):
            W.QMessageBox.information(self,"先固定机械臂","请先结束示教并恢复抱紧、完成现场确认，再观看视频。")
            return
        if not self.confirm("管理前确认机械臂已固定" if management else "观看前确认机械臂已固定", "进入后锁定机械臂控制。此操作不会松力、抱紧、回位或执行轨迹。",
                            ["机械臂已抱紧或可靠固定，可自行保持姿态", "当前没有运动或采集，无需我持续手托"],accept_text="进入管理" if management else "开始观看"):
            return
        if not self.worker.submit("begin_viewing",confirmed=True):
            return
        self.view_wait = QtCore.QTimer(self)
        self.view_wait.setInterval(30)
        def ready():
            if self.worker.pending:
                return
            self.view_wait.stop()
            self.view_wait.deleteLater()
            if not self.worker.viewing.is_set():
                return
            try:
                import cv2
                if management:
                    from .manager_ui import ManagerDialog
                    dialog=ManagerDialog(self.worker.root,cv2,self,self.emergency_stop)
                else:
                    from .playback import VideoDialog
                    dialog=VideoDialog(self.worker.root,cv2,self,current=current,emergency=self.emergency_stop)
                dialog.exec_()
            except Exception as error:
                W.QMessageBox.warning(self,"无法观看视频",str(error))
            finally:
                self.worker.submit("end_viewing")
        self.view_wait.timeout.connect(ready)
        self.view_wait.start()
        self.update_state()

    def takeover(self):
        if self.confirm("接管出厂控制", "将结束出厂APP和手柄进程，释放相机及控制接口。不发送运动或扭矩指令。", ["当前没有用出厂APP或手柄执行任务"]):
            self.send("takeover")

    def update_state(self):
        s = self.state()
        viewing = self.worker.viewing.is_set()
        latched=self.worker.safety.tripped.is_set()
        self.reset_stop_btn.setVisible(latched)
        self.saved_videos_btn.setEnabled(not self.worker.pending and not viewing and
            (s is None or (s["activity"] == "idle" and s["stage"] not in ("teaching","home_setup") and s["holding"] not in ("free","pending"))))
        self.manager_btn.setEnabled(self.saved_videos_btn.isEnabled())
        self.camera_settings.setEnabled(bool(s) and not self.worker.pending and not viewing and not latched and s['activity']=='idle' and s['stage'] not in ('teaching','home_setup') and s['holding'] not in ('free','pending'))
        if s is None:
            self.task.setEnabled(False)
            self.task_edit.setEnabled(False)
            self.connection.setText("设备未连接")
            self.message.setText(self.worker.error)
            self.warning.setText("未连接时不能发送控制指令。软件不会自动接管或移动机械臂。")
            for b in list(self.actions.values())+[self.hold_button,self.feedback_button,self.confirm_button,self.recover_button,self.set_home_btn,self.home_btn]:
                b.setVisible(False)
            self.connection_button.setVisible(True)
            self.takeover_button.setVisible(not self.worker.demo)
            return
        self.connection_button.hide()
        self.takeover_button.hide()
        busy = self.worker.pending or s["activity"] != "idle" or viewing
        self.camera_reconnect.setEnabled(not busy and s["stage"] != "teaching")
        connected = s["valid"] and s["age_s"] < .6
        self.connection.setText("模拟模式 · 无真实连接" if s["demo"] else ("关节在线" if connected else "反馈异常 / 已过期"))
        self.mode.setText(HOLDING[s["holding"]])
        self.title.setText({"returning":"回到初始状态", "replaying":"执行与同步采集", "encoding":"正在合成视频"}.get(s["activity"],STAGES[s["stage"]]))
        if s["activity"]=="idle" and s["stage"] in ("ready","prepare"):
            self.title.setText("示教准备 · 松力" if s["holding"]=="free" else "准备新示教")
        if s["stage"] == "teach_review" and s["teaching_status"] in ("error","interrupted"):
            self.title.setText("示教已中断 · 草稿保留")
        phase = 2 if s["stage"] == "capture_review" or s["activity"] in ("replaying","encoding") else (1 if s["stage"] in ("teach_review","saved_teach") else 0)
        for i,b in enumerate(self.step_buttons):
            active = i == phase
            if b.property("active") != active:
                b.setProperty("active",active)
                b.style().unpolish(b)
                b.style().polish(b)
        message = s["message"]
        if s["feedback_error"]:
            message = s["feedback_error"]
            if s["holding"] == "pending":
                message += " 抱紧指令已发；无法调整时，可支撑后重新松力调整。"
            elif s["holding"] == "free":
                message += " 持续支撑，轻柔调整至有效范围；有阻挡不要硬掰。"
            elif s["stage"] == "teaching":
                message += " 正在重试，未将无效角度写入轨迹。"
            elif s["stage"] == "teach_review":
                message += " 本条示教已中断，恢复后请删除重录。"
            else:
                message += " 可重新检查，或现场确认后尝试抱紧。"
        self.message.setText(message)
        self.message.setStyleSheet("color:#a23d31;" if s["error"] or s["feedback_error"] else "")
        for j,val in enumerate(s["pose"]):
            self.values[j].setText(f"{val:.0f}°" if val is not None else "—")
            joint_valid = val is not None and s["age_s"] < .6
            self.values[j].setStyleSheet("color:#a23d31;" if not joint_valid else "")
            hint = s["hints"][j]
            self.raw_labels[j].setText(f"原始映射约{hint}°" if hint is not None else ("SDK反馈有效" if joint_valid else "读数无效 / 待更新"))
        if s["holding"] == "free":
            warning = "机械臂及夹爪已发松力指令，请持续支撑。读数异常时不要硬掰或盲目上力。"
        elif s["holding"] == "pending":
            warning = "抱紧指令已发，尚未完成现场确认。继续保留支撑，确认已抱紧后点下方按钮。"
            if not connected:
                warning = "角度异常，禁止自动回位。若已抱紧而无法手调，可点“支撑后重新松力调整”；不会自动松力。"
        elif s["holding"] == "unknown":
            warning = "抱紧状态待现场确认。角度稳定不能证明有扭矩；若仍松力，先恢复当前位置抱紧。"
        else:
            warning = "自动动作前检查整条路径。停止按钮只停止后续指令，不自动松力。"
        if s["valid"] and s["boundary_joints"] and s["holding"] == "free":
            warning += " "+"/".join(f"J{j}" for j in s["boundary_joints"])+"距SDK边界不足10°，手拖请留出余量。"
        self.warning.setText(warning)
        self.home_text.setText("初始状态："+("  /  ".join(f"{x:.0f}°" for x in s["home"]) if s["home"] else "尚未设置"))
        self.set_home_btn.setVisible(True)
        self.home_btn.setVisible(True)
        self.set_home_btn.setEnabled(not busy and connected and s["holding"] != "pending" and s["stage"] not in ("teaching","home_setup"))
        self.home_btn.setEnabled(not busy and connected and s["home"] is not None and s["holding"] == "confirmed" and s["stage"] not in ("teaching","home_setup"))
        visible = {
            "home_setup":{"finish_home","cancel_home"},
            "ready":{"release"}, "prepare":{"begin"}, "teaching":{"end"},
            "teach_review":{"save_teach","delete"}, "saved_teach":{"replay","next","delete"},
            "capture_review":{"save_capture","rerecord","delete","next","preview_take"},
        }[s["stage"]]
        if s["stage"] in ("ready","prepare"):
            visible={"begin"} if s["holding"]=="free" else {"release"}
        if s["stage"] == "teach_review" and s["teaching_status"] != "draft":
            visible.discard("save_teach")
        for key,b in self.actions.items():
            b.setVisible(key in visible)
            enable = not busy
            if key in ("begin","release"):
                enable = enable and connected and s["home"] is not None
            if key == "begin":
                enable = enable and s["holding"] == "free"
            if key == "finish_home":
                enable = enable and connected and s["holding"] == "free"
            if key in ("replay","rerecord"):
                enable = enable and connected and s["holding"] == "confirmed" and s["teaching_saved"]
            if key == "save_capture":
                enable = enable and s["capture_status"] == "draft"
            if key == "preview_take":
                enable = enable and s["capture_frames"] > 0
            if key == "save_teach":
                enable = enable and self.worker.engine.teaching and self.worker.engine.teaching["status"] == "draft"
            b.setEnabled(bool(enable))
        task_enabled = not busy and s["stage"] in ("ready", "prepare") and not self.worker.engine.teaching and not self.worker.engine.capture
        self.task.setEnabled(task_enabled)
        self.task_edit.setEnabled(task_enabled)
        # The engine owns the applied name, including restored historical work.
        choices = s["task_history"]
        if choices != [self.task.itemText(i) for i in range(self.task.count())]:
            blocker = QtCore.QSignalBlocker(self.task)
            self.task.clear()
            self.task.addItems(choices)
            self.task.setCurrentText(s["task"])
            del blocker
        elif self.task.currentText() != s["task"]:
            blocker = QtCore.QSignalBlocker(self.task)
            self.task.setCurrentText(s["task"])
            del blocker
        self.task.setToolTip(s["task"])
        self.task_edit.setToolTip("输入新任务名称，只用于之后的新示教。" if task_enabled else "已有数据请先保存，再点下一条人工示教后编辑任务。")
        self.hold_button.setVisible(s["holding"] in ("free","unknown") and (not connected or s["stage"] not in ("teaching","home_setup")))
        self.hold_button.setText("异常时尝试抱紧" if not connected else "当前位置抱紧")
        self.hold_button.setEnabled(not busy)
        self.feedback_button.setVisible(not connected)
        self.feedback_button.setEnabled(not busy)
        self.confirm_button.setVisible(s["holding"] in ("pending","unknown") and s["stage"] != "teaching" and connected)
        self.confirm_button.setEnabled(not busy and connected)
        self.recover_button.setVisible(s["holding"] != "free" and not connected and s["stage"] != "teaching")
        self.recover_button.setEnabled(not busy)
        self.stats.setText(f"{s['elapsed']:.1f} 秒   ·   {s['samples']} 个示教点   ·   {s['capture_frames']} 帧RGB")
        self.progress.setValue(int(100*(s["progress"] if s["activity"] in ("returning","replaying") else min(s["elapsed"]/120,1))))
        camera_status=s.get("camera_status")
        camera_note=(f'相机 {camera_status["fps"]:.1f}fps · '+('帧率稳定' if camera_status['ready'] else '预热中 / 帧率待稳定')) if camera_status else '相机仅预览；自动回放时保存RGB和逐帧动作。'
        if camera_status and camera_status.get('mode')=='dual':
            streams=camera_status['streams']
            camera_note=f'双摄 主{streams["wrist"]["fps"]:.1f} / 外{streams["overview"]["fps"]:.1f}fps · '+('稳定' if camera_status['ready'] else '预热 / 异常')
        self.camera_note.setText(s["camera_error"] or camera_note)
        dual=bool(camera_status and camera_status.get('mode')=='dual')
        self.preview_panels['overview'].setVisible(dual)
        frames=s.get('preview_frames') or ({'wrist':s['preview']} if s.get('preview') else {})
        errors=s.get('preview_errors',{})
        for role,preview in self.preview_labels.items():
            if role=='overview' and not dual:
                preview.clear();continue
            stream=(camera_status or {}).get('streams',{}).get(role,{})
            title='主相机 / 腕部' if role=='wrist' else '第二相机 / 外部'
            self.preview_titles[role].setText(title+(f' · {stream["fps"]:.1f} fps' if 'fps' in stream else ''))
            data=frames.get(role)
            pix=QtGui.QPixmap()
            if data and pix.loadFromData(data,'JPG'):
                preview.setPixmap(pix.scaled(preview.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
            else:
                preview.setText('相机暂不可用\n'+errors[role] if role in errors else '等待相机画面')
        items = [(x["id"],x["status"],x.get("samples_count",0)) for x in s["library"] if x["status"] == "recorded"]
        if items != self.last_library:
            self.library.clear()
            for item_id,status,count in items:
                try:
                    clock = datetime.strptime(item_id[:16],"%Y%m%dT%H%M%SZ").replace(tzinfo=timezone.utc).astimezone(ZoneInfo("Asia/Shanghai")).strftime("%H:%M:%S")
                except ValueError:
                    clock = item_id[:15]
                item = W.QListWidgetItem(f"{clock}  ·  {count}点\n{item_id[-8:]}")
                item.setData(QtCore.Qt.UserRole,item_id)
                self.library.addItem(item)
            self.last_library = items
        self.library.setEnabled(not busy and s["holding"] == "confirmed" and s["stage"] != "home_setup")
        self.counts.setText(f"{len(items)} 条人工示教\n{s['saved_episodes']} 条完整数据已保存")
        self.footer.setText(f"10 Hz · {s['root']}  |  删除移入回收站")
        if latched:
            self.warning.setText('软件急停已锁定：不再发送控制指令；已有动作可能继续，系统死机时此按钮无效。扭矩不自动改变，松力时保持支撑。')
            self.message.setText(self.worker.safety.reason+'；排除故障后点顶部“复位”，不会自动续动。'+self.worker.safety.persistence_error)
            for button in list(self.actions.values())+[self.hold_button,self.confirm_button,self.recover_button,self.set_home_btn,self.home_btn,self.camera_reconnect]:button.setEnabled(False)
            self.reset_stop_btn.setEnabled(not self.worker.pending and s['activity']=='idle' and s['stage']!='teaching')
        if not self.worker.thread.is_alive() and not self.worker.quit.is_set():
            self.connection.setText("设备线程已停止")
            self.message.setText("设备线程已停止：保持支撑，退出并重新打开软件。"+self.worker.error)
            for button in list(self.actions.values())+[self.hold_button,self.feedback_button,self.confirm_button,self.recover_button,self.set_home_btn,self.home_btn]:
                button.setEnabled(False)
            self.saved_videos_btn.setEnabled(False)

    def closeEvent(self, event):
        s = self.state()
        message = "关闭不会自动松力或上力。"
        if s and s["holding"] in ("free","pending"):
            message = "当前仍松力或抱紧未确认！先可靠支撑。关闭后保留恢复标记，不自动改变扭矩。"
        if not self.confirm("退出采集台",message,["已妥善支撑或确认抱紧，允许停止后续指令并退出"]):
            event.ignore()
            return
        self.timer.stop()
        self.worker.close()
        if self.worker.thread.is_alive():
            W.QMessageBox.warning(self,"正在停止","设备线程仍在结束，请稍后再退出。未强制关闭。")
            self.timer.start(200)
            event.ignore()
            return
        W.QApplication.instance().removeEventFilter(self.emergency_key)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo",action="store_true")
    parser.add_argument("--fullscreen",action="store_true")
    parser.add_argument("--root",type=Path)
    parser.add_argument("--screenshot",type=Path,help="只在模拟模式自动截图并退出，用于界面测试")
    args = parser.parse_args()
    if args.screenshot and not args.demo:
        parser.error("自动截图测试只能使用--demo。")
    root = args.root or (BASE / "touch_demo_data" if args.demo else REAL_ROOT)
    # OpenCV wheels may set incompatible Qt plugin paths. Use this PyQt installation.
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app = W.QApplication(sys.argv[:1])
    app.setApplicationName("DOFBOT 数据采集台")
    app.setFont(QtGui.QFont("Noto Sans CJK SC",11))
    import hashlib
    server_name = "cyh_dofbot_"+hashlib.sha256((str(root.resolve())+str(args.demo)).encode()).hexdigest()[:16]
    socket = QtNetwork.QLocalSocket()
    socket.connectToServer(server_name)
    if socket.waitForConnected(150):
        socket.write(b"activate")
        socket.flush()
        socket.waitForBytesWritten(150)
        return 0
    QtNetwork.QLocalServer.removeServer(server_name)
    server = QtNetwork.QLocalServer()
    if not server.listen(server_name):
        raise RuntimeError("无法创建本机单实例入口。")
    worker = Worker(root,args.demo)
    window = Window(worker)
    def activate():
        connection = server.nextPendingConnection()
        window.showFullScreen() if args.fullscreen else window.showNormal()
        window.raise_()
        window.activateWindow()
        if connection:
            connection.close()
    server.newConnection.connect(activate)
    worker.start()
    window.showFullScreen() if args.fullscreen else window.show()
    if args.screenshot:
        def shot():
            window.update_state()
            args.screenshot.parent.mkdir(parents=True,exist_ok=True)
            window.grab().save(str(args.screenshot))
            worker.close()
            app.quit()
        QtCore.QTimer.singleShot(2500,shot)
    try:
        return app.exec_()
    finally:
        window.timer.stop()
        app.removeEventFilter(window.emergency_key)
        if worker.thread.is_alive():worker.close()
        window.hide();window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)


if __name__ == "__main__":
    raise SystemExit(main())
