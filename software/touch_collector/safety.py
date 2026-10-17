"""Persistent software stop latch. No actuator access; not a hardware E-stop."""
from datetime import datetime,timezone
import threading
from .storage import load
from trajectory_teach import save_json

class EmergencyLatch:
    def __init__(self,path):
        self.path=path
        self.tripped=threading.Event()
        self.lock=threading.RLock()
        self.reason=''
        self.persistence_error=''
        if path.exists():
            try:
                state=load(path)
                if state.get('latched',True):
                    self.tripped.set();self.reason=state.get('reason','上次软件急停尚未复位')
            except (OSError,ValueError,TypeError,AttributeError):
                self.tripped.set();self.reason='急停状态文件无效，按锁定处理'

    def trip(self,reason='触屏软件急停'):
        # Signal first; slow/failed storage must never delay the in-memory stop.
        self.tripped.set()
        with self.lock:
            self.tripped.set()
            self.reason=reason
            try:
                save_json(self.path,{'latched':True,'reason':reason,'utc':datetime.now(timezone.utc).isoformat()})
                self.persistence_error=''
            except OSError as error:self.persistence_error=str(error)

    def require_clear(self):
        if self.tripped.is_set():
            raise RuntimeError('软件急停已锁定。先排除故障、固定机械臂，再手动复位；不会自动松力或上力。')

    def reset(self):
        with self.lock:
            save_json(self.path,{'latched':False,'reset_utc':datetime.now(timezone.utc).isoformat()})
            self.tripped.clear();self.reason='';self.persistence_error=''

    def status(self):
        return {'latched':self.tripped.is_set(),'reason':self.reason,'persistence_error':self.persistence_error}
