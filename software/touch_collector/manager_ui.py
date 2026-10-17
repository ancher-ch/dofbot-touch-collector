"""Touch-friendly local data management; mutations are reversible/audited."""
import threading
import re
from pathlib import Path
from PyQt5 import QtCore,QtWidgets as W
from .management import DatasetManager
from .storage import load

class ExportJob(QtCore.QThread):
    result=QtCore.pyqtSignal(str,bool)
    def __init__(self,manager,kind,item_id,path,parent=None):
        super().__init__(parent)
        self.manager=manager;self.kind=kind;self.item_id=item_id;self.path=path;self.cancel=threading.Event()
    def run(self):
        try:self.result.emit(str(self.manager.export(self.kind,self.item_id,self.path,self.cancel)),True)
        except Exception as error:self.result.emit(str(error),False)

class ManagerDialog(W.QDialog):
    def __init__(self,root,cv2,parent=None,emergency=None):
        super().__init__(parent)
        self.manager=DatasetManager(root);self.cv2=cv2;self.emergency=emergency
        self.job=None;self.records=[]
        self.setWindowTitle('数据管理 · 控制已锁定');self.resize(1140,720)
        layout=W.QVBoxLayout(self)
        top=W.QHBoxLayout();top.addWidget(W.QLabel('数据管理：筛选、标注、导出和回收站'))
        if emergency:
            stop=W.QPushButton('软件急停');stop.setObjectName('danger');stop.clicked.connect(lambda:(emergency(),self.message.setText('软件急停已锁定。扭矩不自动改变；故障排除后返回主界面手动复位。')));top.addWidget(stop)
        layout.addLayout(top)
        row=W.QHBoxLayout()
        self.kind=W.QComboBox();self.kind.addItems(['完整采集 / 异常采集','人工示教','回收站'])
        self.search=W.QLineEdit();self.search.setPlaceholderText('搜索编号、任务或备注')
        self.filter=W.QComboBox();self.filter.addItems(['全部','成功','失败','待标注','异常 / 中断'])
        for widget in (self.kind,self.search,self.filter):row.addWidget(widget)
        layout.addLayout(row)
        body=W.QHBoxLayout()
        self.table=W.QTableWidget(0,4)
        self.table.setHorizontalHeaderLabels(['编号 / 时间','状态 / 结果','帧数 / 点数','任务'])
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(W.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(58)
        self.table.setMinimumWidth(590)
        body.addWidget(self.table,3)
        panel=W.QVBoxLayout()
        self.info=W.QLabel('请选择数据');self.info.setWordWrap(True);self.info.setTextFormat(QtCore.Qt.PlainText)
        panel.addWidget(self.info)
        panel.addWidget(W.QLabel('任务标签（采集数据可改，原始示教保留）'))
        self.task=W.QLineEdit();self.task.setMaxLength(500);panel.addWidget(self.task)
        self.success=W.QComboBox();self.success.addItems(['待标注','成功','失败']);panel.addWidget(self.success)
        panel.addWidget(W.QLabel('备注'))
        self.notes=W.QPlainTextEdit();self.notes.setMaximumHeight(130);panel.addWidget(self.notes)
        self.buttons={}
        for key,text,callback in [('view','观看视频',self.view),('annotate','保存标签和备注',self.annotate),
                                  ('export','导出完整ZIP',self.export),('trash','移入回收站',self.trash),
                                  ('restore','从回收站恢复',self.restore)]:
            button=W.QPushButton(text);button.clicked.connect(callback);panel.addWidget(button);self.buttons[key]=button
        panel.addStretch();body.addLayout(panel,2);layout.addLayout(body,1)
        self.message=W.QLabel('删除移入回收站，可恢复；导出包含原始图像、视频、关节和校验清单。当前工作区数据需先点“下一条”再修改。')
        self.message.setWordWrap(True);layout.addWidget(self.message)
        bottom=W.QHBoxLayout();refresh=W.QPushButton('刷新');refresh.clicked.connect(self.reload)
        close=W.QPushButton('返回采集台');close.clicked.connect(self.accept)
        bottom.addWidget(refresh);bottom.addWidget(close);layout.addLayout(bottom)
        self.kind.currentIndexChanged.connect(self.reload);self.search.textChanged.connect(self.reload)
        self.filter.currentIndexChanged.connect(self.reload);self.table.itemSelectionChanged.connect(self.selected)
        self.reload()
    def current(self):
        row=self.table.currentRow()
        return self.records[row] if 0<=row<len(self.records) else None
    def reload(self):
        index=self.kind.currentIndex()
        kind='episodes' if index==0 else 'teachings'
        records=self.manager.trash_items() if index==2 else self.manager.store.list_items(kind)
        query=self.search.text().casefold();option=self.filter.currentIndex()
        self.records=[]
        for record in records:
            meta=record['metadata'] if index==2 else record
            if query and query not in ' '.join(str(meta.get(k,'')) for k in ('id','task','notes')).casefold():continue
            result=meta.get('operator_success');status=meta.get('status','')
            if option==1 and result is not True:continue
            if option==2 and result is not False:continue
            if option==3 and result is not None:continue
            if option==4 and status not in ('error','interrupted'):continue
            self.records.append(record)
        self.table.setRowCount(0)
        for record in self.records:
            meta=record['metadata'] if index==2 else record
            row=self.table.rowCount();self.table.insertRow(row)
            values=[meta.get('id',record.get('original_id','')),meta.get('status','')+' / '+{True:'成功',False:'失败',None:'待标注'}.get(meta.get('operator_success'),'待标注'),
                    str(meta.get('frames',meta.get('samples_count',0))),meta.get('task','')]
            for col,value in enumerate(values):self.table.setItem(row,col,W.QTableWidgetItem(str(value)))
        if self.records:self.table.selectRow(0)
        self.selected()
    def selected(self):
        record=self.current();trash=self.kind.currentIndex()==2;busy=self.job is not None
        for key,button in self.buttons.items():button.setEnabled(bool(record) and not busy)
        self.buttons['restore'].setVisible(trash)
        for key in ('view','annotate','export','trash'):self.buttons[key].setVisible(not trash)
        if not record:return
        meta=record['metadata'] if trash else record
        try:duration=f"{float(meta.get('sample_duration_s',0)):.1f}"
        except (ValueError,TypeError):duration='未知'
        policy=meta.get('gripper_compensation') or {}
        if not isinstance(policy,dict):policy={}
        self.info.setText(f"{meta.get('id',record.get('original_id',''))}\n{meta.get('status','')} · {duration} 秒\n相机：{meta.get('camera_mode','single')} · 补偿：{policy.get('max_extra_degrees',0)}°")
        self.task.setText(meta.get('task',''));self.task.setEnabled(not trash and self.kind.currentIndex()==0 and not busy)
        self.success.setCurrentIndex({None:0,True:1,False:2}.get(meta.get('operator_success'),0))
        self.success.setEnabled(not trash and self.kind.currentIndex()==0 and not busy)
        self.notes.setPlainText(meta.get('notes',''));self.notes.setEnabled(not trash and not busy)
        self.buttons['view'].setEnabled(not busy and self.kind.currentIndex()==0 and bool(meta.get('frames')))
    def run_action(self,callback):
        try:
            callback();self.message.setText('操作完成。');self.reload()
        except Exception as error:W.QMessageBox.warning(self,'未完成操作',str(error))
    def annotate(self):
        record=self.current()
        if record:self.run_action(lambda:self.manager.annotate('episodes' if self.kind.currentIndex()==0 else 'teachings',record['id'],self.task.text(),[None,True,False][self.success.currentIndex()],self.notes.toPlainText()))
    def trash(self):
        record=self.current()
        if record and W.QMessageBox.question(self,'移入回收站','仅移动所选数据，可以恢复；其他已保存采集不受影响。')==W.QMessageBox.Yes:
            self.run_action(lambda:self.manager.trash('episodes' if self.kind.currentIndex()==0 else 'teachings',record['id']))
    def restore(self):
        record=self.current()
        if record:self.run_action(lambda:self.manager.restore(record['trash_id']))
    def view(self):
        record=self.current()
        if not record:return
        from .playback import VideoDialog
        dialog=VideoDialog(self.manager.store.root,self.cv2,self,current=self.manager.store.item('episodes',record['id']),emergency=self.emergency)
        dialog.exec_()
    def export(self):
        record=self.current()
        if not record or self.job:return
        kind='episodes' if self.kind.currentIndex()==0 else 'teachings'
        default=self.manager.store.root/'exports'/f'{kind}_{record["id"]}.zip'
        path=self.choose_export_path(default)
        if not path:return
        self.job=ExportJob(self.manager,kind,record['id'],path,self)
        self.export_result=('',False)
        self.job.result.connect(lambda message,ok:setattr(self,'export_result',(message,ok)))
        self.job.finished.connect(self.export_finished)
        self.message.setText('正在后台导出，可随时使用软件急停。关闭窗口将取消导出。')
        self.selected();self.job.start()
    def choose_export_path(self,default):
        dialog=W.QDialog(self);dialog.setWindowTitle('导出到本机 exports 目录')
        layout=W.QVBoxLayout(dialog)
        note=W.QLabel('导出完整数据到：\n'+str(default.parent)+'\n可通过SSH下载ZIP；不会上传网络。');note.setWordWrap(True);layout.addWidget(note)
        name=W.QLineEdit(default.name);layout.addWidget(name)
        row=W.QHBoxLayout();cancel=W.QPushButton('取消');cancel.clicked.connect(dialog.reject)
        accept=W.QPushButton('开始导出');accept.clicked.connect(dialog.accept)
        row.addWidget(cancel);row.addWidget(accept)
        if self.emergency:
            stop=W.QPushButton('软件急停',objectName='danger');stop.clicked.connect(lambda:(self.emergency(),dialog.reject()));row.addWidget(stop)
        layout.addLayout(row)
        if dialog.exec_()!=W.QDialog.Accepted:return None
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,160}\.zip',name.text()):
            W.QMessageBox.warning(self,'文件名无效','文件名限英文字母、数字、下划线、点和短横线，以.zip结尾。');return None
        return default.parent/name.text()
    def export_finished(self):
        message,ok=self.export_result
        job=self.job;self.job=None
        if job:job.deleteLater()
        self.message.setText(('已导出：' if ok else '导出未完成：')+message);self.selected()
    def done(self,result):
        if self.job:
            self.job.cancel.set();self.message.setText('正在取消导出，请稍候…');return
        super().done(result)
