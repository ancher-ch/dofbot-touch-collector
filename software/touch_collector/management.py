"""Local dataset operations. Raw observations/actions remain immutable; no robot IO."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
import threading
import uuid
import zipfile
from .storage import Store,load
from trajectory_teach import save_json

class DatasetManager:
    def __init__(self,root):self.store=Store(root)
    def _available(self,kind,item_id):
        path=self.store.item(kind,item_id)
        if path.is_symlink():raise ValueError('不能管理符号链接数据目录。')
        work=self.store.root/'work.json'
        if work.exists() and load(work).get('teaching_id' if kind=='teachings' else 'capture_id')==item_id:
            raise ValueError('这条数据正在当前工作区使用。先返回采集台点“下一条”，再管理它。')
        if not path.is_dir():raise ValueError('数据不存在。')
        return path
    def annotate(self,kind,item_id,task,success,notes):
        path=self._available(kind,item_id)
        if type(notes) is not str or len(notes)>2000:raise ValueError('备注限2000字。')
        if type(task) is not str or not task.strip() or len(task)>500:raise ValueError('任务描述不能为空，最多500字。')
        if success is not None and type(success) is not bool:raise ValueError('任务结果无效。')
        meta=load(path/'metadata.json')
        before={key:meta.get(key) for key in ('task','operator_success','notes')}
        if kind=='episodes':
            meta.setdefault('original_task',meta.get('task'))
            meta.update(task=task.strip(),operator_success=success,train_ready=False)
        elif task!=meta.get('task'):raise ValueError('原始人工示教任务不在管理页改写；可修改采集条目的训练任务标签。')
        meta['notes']=notes
        meta.setdefault('annotation_history',[]).append({'utc':datetime.now(timezone.utc).isoformat(),'before':before})
        save_json(path/'metadata.json',meta)
    def trash(self,kind,item_id):
        self._available(kind,item_id)
        return self.store.trash(kind,item_id)
    def trash_items(self):
        result=[]
        for path in (self.store.root/'trash').iterdir():
            if not path.is_dir() or path.is_symlink():continue
            try:
                if (path/'trash_manifest.json').exists():manifest=load(path/'trash_manifest.json')
                else:
                    match=re.fullmatch(r'(episodes|teachings)_([A-Za-z0-9_-]{1,80})_[0-9a-f]{6}',path.name)
                    if not match:continue
                    manifest={'kind':match[1],'original_id':match[2],'deleted_utc':''}
                self.store.item(manifest['kind'],manifest['original_id'])
                result.append({'trash_id':path.name,**manifest,'metadata':load(path/'metadata.json')})
            except (ValueError,KeyError,OSError,TypeError):continue
        return sorted(result,key=lambda x:x.get('deleted_utc',''),reverse=True)
    def restore(self,trash_id):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,180}',trash_id):raise ValueError('回收站编号无效。')
        src=self.store.root/'trash'/trash_id
        if src.is_symlink():raise ValueError('不能恢复符号链接。')
        src.resolve().relative_to(self.store.root/'trash')
        manifest=next((x for x in self.trash_items() if x['trash_id']==trash_id),None)
        if manifest is None:raise ValueError('回收站项目无效。')
        dst=self.store.item(manifest['kind'],manifest['original_id'])
        if dst.exists():raise ValueError('原编号已存在，未覆盖任何数据。')
        src.rename(dst)
        return dst
    def export(self,kind,item_id,destination,cancel=None):
        src=self.store.item(kind,item_id)
        if not src.is_dir() or src.is_symlink():raise ValueError('数据目录无效。')
        destination=Path(destination).resolve()
        if destination.exists():raise ValueError('导出目标已存在，未覆盖。')
        if destination.is_relative_to(self.store.root):
            destination.relative_to(self.store.root/'exports')
        if destination.suffix.lower()!='.zip':raise ValueError('请选择.zip导出文件。')
        destination.parent.mkdir(parents=True,exist_ok=True)
        temporary=destination.with_name(destination.name+'.'+uuid.uuid4().hex+'.partial')
        cancel=cancel or threading.Event()
        entries={}
        try:
            with zipfile.ZipFile(temporary,'x',zipfile.ZIP_STORED) as archive:
                for file in sorted(src.rglob('*')):
                    if cancel.is_set():raise RuntimeError('已取消导出，原数据未改变。')
                    if file.is_symlink():raise ValueError('数据中含符号链接，拒绝导出。')
                    file.resolve().relative_to(src.resolve())
                    if not file.is_file():continue
                    digest=hashlib.sha256()
                    name=file.relative_to(src).as_posix()
                    with file.open('rb') as incoming,archive.open(name,'w',force_zip64=True) as outgoing:
                        while True:
                            if cancel.is_set():raise RuntimeError('已取消导出，原数据未改变。')
                            chunk=incoming.read(1024*1024)
                            if not chunk:break
                            digest.update(chunk);outgoing.write(chunk)
                    entries[name]=digest.hexdigest()
                archive.writestr('export_manifest.json',json.dumps({'kind':kind,'id':item_id,'files_sha256':entries,
                    'utc':datetime.now(timezone.utc).isoformat()},ensure_ascii=False,indent=2))
            # Hard link publishes atomically without overwriting a concurrent export.
            import os
            os.link(temporary,destination)
        finally:
            temporary.unlink(missing_ok=True)
        return destination
