"""Read-only local episode viewer. No robot, worker or camera device imports."""
from bisect import bisect_right
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from zoneinfo import ZoneInfo

from PyQt5 import QtCore, QtGui, QtWidgets as W


def saved_items(root):
    result=[]
    for meta in (Path(root)/'episodes').glob('*/metadata.json'):
        try:
            d=json.loads(meta.read_text(encoding='utf-8'))
            if isinstance(d,dict) and d.get('status')=='saved':
                result.append((meta.parent,d))
        except (ValueError,OSError):
            continue
    return sorted(result,key=lambda x:x[0].name,reverse=True)


def timeline(rows, camera=True):
    if not rows:
        raise ValueError('没有可观看的数据帧。')
    if camera and all(isinstance(x.get('frame_receive_monotonic_ns'),int) for x in rows):
        values=[x['frame_receive_monotonic_ns']/1e9 for x in rows]
    else:
        values=[float(x['timestamp']) for x in rows]
    if any(not math.isfinite(x) for x in values) or any(b<=a for a,b in zip(values,values[1:])):
        raise ValueError('视频时间戳不连续，不能正确播放。')
    return [v-values[0] for v in values]


class Clip:
    def __init__(self,path,cv2):
        self.path=Path(path).resolve()
        self.cv2=cv2
        self.decoder=None
        self.decoder_index=-1
        source=self.path/'samples.jsonl'
        if source.stat().st_size>30_000_000:
            raise ValueError('本条数据过大，无法在触屏中加载。')
        self.rows=[json.loads(x) for x in source.read_text(encoding='utf-8').splitlines() if x.strip()]
        if len(self.rows)>12001:
            raise ValueError('本条数据帧数超出当前查看范围。')
        self.times=timeline(self.rows)
        self.roles=list(self.rows[0].get('cameras',{'wrist':{}})) if self.rows else ['wrist']
        if 'wrist' not in self.roles or not set(self.roles)<= {'wrist','overview'}:raise ValueError('相机角色无效。')
        self.role='wrist'
        for i,row in enumerate(self.rows):
            if row.get('frame_index')!=i:
                raise ValueError('帧索引缺失或顺序异常。')
            (self.path/row['image']).resolve().relative_to(self.path)
            for view in row.get('cameras',{}).values():(self.path/view['image']).resolve().relative_to(self.path)
            if list(row.get('cameras',{'wrist':{}}))!=self.roles:raise ValueError('同一条数据的相机角色不一致。')

    def pixmap(self,index):
        row=self.rows[index]
        image=self.path/(row['image'] if self.role=='wrist' else row['cameras'][self.role]['image'])
        if image.is_file():
            pix=QtGui.QPixmap(str(image))
            if not pix.isNull():return pix
        # Older archived datasets may retain MP4 but have no raw images.
        if self.decoder is None:
            self.decoder=self.cv2.VideoCapture(str(self.path/('video.mp4' if self.role=='wrist' else 'video_overview.mp4')))
            if not self.decoder.isOpened():
                raise ValueError('图像或视频文件无法打开，原文件未修改。')
        if index!=self.decoder_index+1:
            self.decoder.set(self.cv2.CAP_PROP_POS_FRAMES,index)
        ok,frame=self.decoder.read()
        if not ok:raise ValueError('视频解码失败，原文件未修改。')
        self.decoder_index=index
        rgb=self.cv2.cvtColor(frame,self.cv2.COLOR_BGR2RGB)
        h,w,_=rgb.shape
        return QtGui.QPixmap.fromImage(QtGui.QImage(rgb.data,w,h,rgb.strides[0],QtGui.QImage.Format_RGB888).copy())

    def close(self):
        if self.decoder is not None:
            self.decoder.release()
            self.decoder=None


class Player(W.QWidget):
    def __init__(self,cv2,parent=None):
        super().__init__(parent)
        self.cv2=cv2
        self.clip=None
        self.index=0
        self.anchor=0
        self.playing=False
        layout=W.QVBoxLayout(self)
        self.view_choice=W.QComboBox();self.view_choice.currentIndexChanged.connect(self.change_view)
        layout.addWidget(self.view_choice)
        self.image=W.QLabel('选择一条已保存的视频')
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Ignored)
        self.image.setMinimumSize(320,180)
        layout.addWidget(self.image,1)
        self.details=W.QLabel()
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.slider=W.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimumHeight(42)
        self.slider.valueChanged.connect(self.seek)
        layout.addWidget(self.slider)
        row=W.QHBoxLayout()
        self.previous=W.QPushButton('上一帧')
        self.play=W.QPushButton('播放')
        self.following=W.QPushButton('下一帧')
        self.previous.clicked.connect(lambda:self.step(-1))
        self.following.clicked.connect(lambda:self.step(1))
        self.play.clicked.connect(self.toggle)
        self.speed=W.QComboBox()
        self.speed.addItems(['0.5倍速','1倍速','2倍速'])
        self.speed.setCurrentIndex(1)
        self.speed.currentIndexChanged.connect(self.pause)
        for widget in (self.previous,self.play,self.following,self.speed):row.addWidget(widget)
        layout.addLayout(row)
        self.note=W.QLabel('按已记录的相机时间播放；不执行机械臂动作。')
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.timer=QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(12)
        self.timer.timeout.connect(self.advance)
        self._enabled(False)

    def _enabled(self,enabled):
        for widget in (self.previous,self.play,self.following,self.slider,self.speed):widget.setEnabled(enabled)

    def load(self,path):
        self.close_clip()
        try:
            self.clip=Clip(path,self.cv2)
            self.view_choice.blockSignals(True);self.view_choice.clear()
            for role in self.clip.roles:self.view_choice.addItem('腕部 / 主相机' if role=='wrist' else '外部 / 第二相机',role)
            self.view_choice.setEnabled(len(self.clip.roles)>1);self.view_choice.blockSignals(False)
            self.slider.blockSignals(True)
            self.slider.setRange(0,len(self.clip.rows)-1)
            self.slider.setValue(0)
            self.slider.blockSignals(False)
            self.index=0
            self.draw()
            self._enabled(True)
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.close_clip()
            self.image.clear()
            self.image.setText('无法观看本条视频')
            self.details.setText(str(error))

    def change_view(self):
        if not self.clip:return
        self.pause();self.clip.close();self.clip.role=self.view_choice.currentData() or 'wrist';self.clip.decoder_index=-1
        if self.clip.role=='overview':
            self.clip.times=timeline([dict(row,frame_receive_monotonic_ns=row['cameras']['overview']['frame_receive_monotonic_ns']) for row in self.clip.rows])
        else:self.clip.times=timeline(self.clip.rows)
        self.draw()

    def draw(self):
        if not self.clip:return
        try:
            pix=self.clip.pixmap(self.index)
            self.image.setPixmap(pix.scaled(self.image.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
            row=self.clip.rows[self.index]
            self.details.setText(f'第 {self.index+1}/{len(self.clip.rows)} 帧 · {self.clip.times[self.index]:.2f}/{self.clip.times[-1]:.2f} 秒\n'
                                 f'关节角度 {row.get("observation.state",[])}\n控制目标 {row.get("action",[])}')
        except (OSError,ValueError) as error:
            self.pause()
            self._enabled(False)
            self.details.setText(str(error))

    def pause(self,*unused):
        self.playing=False
        self.timer.stop()
        self.play.setText('播放')

    def seek(self,index):
        self.pause()
        self.index=index
        self.draw()

    def step(self,delta):
        if self.clip:self.slider.setValue(max(0,min(len(self.clip.rows)-1,self.index+delta)))

    def toggle(self):
        if not self.clip:return
        if self.playing:
            self.pause();return
        if self.index==len(self.clip.rows)-1:self.slider.setValue(0)
        speed=(.5,1.,2.)[self.speed.currentIndex()]
        self.anchor=time.monotonic()-self.clip.times[self.index]/speed
        self.playing=True
        self.play.setText('暂停')
        self.timer.start()

    def advance(self):
        if not self.clip or not self.playing:return
        elapsed=(time.monotonic()-self.anchor)*(.5,1.,2.)[self.speed.currentIndex()]
        index=max(0,min(len(self.clip.rows)-1,bisect_right(self.clip.times,elapsed)-1))
        if index!=self.index:
            self.index=index
            self.slider.blockSignals(True)
            self.slider.setValue(index)
            self.slider.blockSignals(False)
            self.draw()
        if index==len(self.clip.rows)-1:self.pause()

    def close_clip(self):
        self.pause()
        if self.clip:self.clip.close()
        self.clip=None
        self._enabled(False)


class VideoDialog(W.QDialog):
    def __init__(self,root,cv2,parent=None,current=None,emergency=None):
        super().__init__(parent)
        self.setWindowTitle('观看已保存视频 · 机械臂控制已锁定')
        self.resize(1100,710)
        outer=W.QVBoxLayout(self)
        title=W.QLabel('观看已保存视频' if current is None else '观看本次数据')
        title.setObjectName('title')
        outer.addWidget(title)
        if emergency:
            stop=W.QPushButton('软件急停 · F12',objectName='danger');stop.clicked.connect(lambda:(emergency(),title.setText('软件急停已锁定 · 返回主界面排除故障后手动复位')));outer.addWidget(stop)
        body=W.QHBoxLayout()
        self.items=W.QListWidget()
        self.items.setFixedWidth(245)
        self.items.setWordWrap(True)
        self.items.setStyleSheet('QListWidget {background:#e8eeed;color:#172e3c;border:1px solid #d7e3e2;border-radius:8px;} '
            'QListWidget::item {color:#172e3c;padding:8px;} QListWidget::item:selected {background:#2d6064;color:white;border-radius:6px;}')
        body.addWidget(self.items)
        self.player=Player(cv2,self)
        body.addWidget(self.player,1)
        outer.addLayout(body,1)
        close=W.QPushButton('关闭观看，返回采集')
        close.clicked.connect(self.accept)
        outer.addWidget(close)
        self.items.currentItemChanged.connect(self.selected)
        if current is not None:
            self.items.hide()
            self.player.load(current)
        else:
            for path,meta in saved_items(root):
                try:
                    date=datetime.strptime(path.name[:16],'%Y%m%dT%H%M%SZ').replace(tzinfo=timezone.utc).astimezone(ZoneInfo('Asia/Shanghai')).strftime('%m-%d %H:%M:%S')
                except ValueError:date=path.name[:16]
                result={True:'成功',False:'失败',None:'待标注'}.get(meta.get('operator_success'),'待标注')
                item=W.QListWidgetItem(f'{date}\n{meta.get("frames",0)} 帧 · {result}\n{meta.get("task","")}')
                item.setData(QtCore.Qt.UserRole,str(path))
                item.setSizeHint(QtCore.QSize(220,105))
                self.items.addItem(item)
            if self.items.count():self.items.setCurrentRow(0)
            else:self.player.image.setText('还没有已保存的视频')
        self.finished.connect(lambda _:self.player.close_clip())

    def selected(self,item,previous):
        if item:self.player.load(item.data(QtCore.Qt.UserRole))
