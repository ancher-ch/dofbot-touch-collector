"""Named independently read RGB cameras; bounded host-time pairing, not hardware sync.

Design follows LeRobot camera API separation of background acquisition and fresh
consumer reads. Uses the existing tested camera backend; no LeRobot dependency.
"""
import os
from pathlib import Path
import struct
import time
from .camera import SynchronizedCamera,CameraReadinessError,CameraFrameError

RGB_FORMATS={b'MJPG',b'JPEG',b'YUYV',b'UYVY',b'RGB3',b'BGR3',b'NV12',b'NV21'}
SENSOR_ONLY_FORMATS={b'Z16 ',b'GREY',b'Y10 ',b'Y12 ',b'Y16 ',b'BA81',b'YV12',b'RW16'}

def rgb_capture_formats(fd,ioctl):
    """Enumerate pixel formats; capture capability alone includes depth and IR."""
    formats=[]
    for index in range(128):
        buf=bytearray(64)
        struct.pack_into('II',buf,0,index,1)  # VIDEO_CAPTURE
        try:ioctl(fd,0xc0405602,buf)  # VIDIOC_ENUM_FMT
        except OSError:break
        formats.append(bytes(buf[44:48]))
    return formats

def physical_camera_id(node):
    """Group all interfaces of one USB camera, without opening any sensor."""
    path=(Path('/sys/class/video4linux')/Path(node).name/'device').resolve()
    for parent in (path,*path.parents):
        if (parent/'idVendor').is_file() and (parent/'idProduct').is_file():
            return str(parent)
    return str(Path(node).resolve())

def discover_devices():
    import fcntl
    result=[]
    # Some multi-interface cameras reuse by-id index0. by-path includes the
    # interface number and avoids selecting depth when the RGB node renumbers.
    aliases=sorted(Path('/dev/v4l/by-path').glob('*'))+sorted(Path('/dev/v4l/by-id').glob('*'))
    for node in sorted(Path('/dev').glob('video*')):
        fd=None
        try:
            fd=os.open(node,os.O_RDONLY|os.O_NONBLOCK)
            buf=bytearray(104)
            fcntl.ioctl(fd,0x80685600,buf)
            caps=struct.unpack_from('I',buf,84)[0]
            if caps&0x80000000:caps=struct.unpack_from('I',buf,88)[0]
            if not caps&(1|0x1000):continue  # exclude metadata-only capture nodes
            formats=rgb_capture_formats(fd,fcntl.ioctl)
            # Gemini IR also advertises NV12/YV12: reject raw sensor interfaces
            # before accepting their optional converted YUV output.
            if set(formats)&SENSOR_ONLY_FORMATS or not set(formats)&RGB_FORMATS:continue
            stable=next((str(p) for p in aliases if p.resolve()==node.resolve()),str(node))
            result.append({'path':stable,'node':str(node),'card':bytes(buf[16:48]).split(b'\0')[0].decode(errors='replace'),
                           'identity':physical_camera_id(node),'formats':[f.decode('ascii',errors='replace') for f in formats]})
        except OSError:continue
        finally:
            if fd is not None:os.close(fd)
    return result

def validate_config(config,devices=None):
    if not isinstance(config,dict) or config.get('mode') not in ('single','dual'):
        raise ValueError('相机模式只能选单摄或双摄。')
    roles=['wrist']+(['overview'] if config['mode']=='dual' else [])
    paths=[]
    for role in roles:
        value=config.get(role)
        if not isinstance(value,str) or not value.startswith('/dev/'):
            raise ValueError('请选择有效RGB相机设备。')
        paths.append(str(Path(value).resolve()))
    if len(set(paths))!=len(paths):raise ValueError('双摄必须选择两台不同的RGB相机，不能重复选择同一设备。')
    if devices is not None:
        available={str(Path(x['path']).resolve()):x for x in devices}
        if not set(paths)<=set(available):raise ValueError('所选RGB相机未连接或不是RGB采集节点；当前模式未改变。')
        identities=[available[p].get('identity',p) for p in paths]
        if len(set(identities))!=len(identities):raise ValueError('双摄必须选择两台不同的实体RGB相机。')
    return {key:config.get(key) for key in ('mode','wrist','overview')}

class ViewSample(tuple):
    def __new__(cls,views):
        obj=super().__new__(cls,views['wrist'])
        obj.views=views
        return obj

class CameraRig:
    max_recording_age_ms=50
    max_recording_interval_error_ms=20
    max_pair_skew_ms=35
    def __init__(self,cv2,cameras,config):
        self.cv2=cv2;self.cameras=cameras;self.config=config
        self.last_sequences={}
        self.last_stamps={}
    def start(self):
        try:
            for camera in self.cameras.values():camera.start()
        except BaseException:
            self.close();raise
    def close(self):
        errors=[]
        for camera in self.cameras.values():
            try:
                camera.close()
                if hasattr(camera,'thread') and camera.thread.is_alive():raise RuntimeError('相机读取线程未退出，禁止重复打开。')
            except Exception as error:errors.append(str(error))
        if errors:raise RuntimeError('；'.join(errors))
    def status(self):
        streams={name:(camera.status() if hasattr(camera,'status') else {'ready':True,'fps':30.,'error':None}) for name,camera in self.cameras.items()}
        report=dict(streams['wrist'])
        report.update(mode=self.config['mode'],streams=streams,devices=dict(self.config),
                      ready=all(x['ready'] for x in streams.values()),
                      error='；'.join(f'{name}: {s["error"]}' for name,s in streams.items() if s.get('error')) or None)
        return report
    def get(self,after=-1):
        return ViewSample({name:camera.get(after if name=='wrist' else -1) for name,camera in self.cameras.items()})
    def _latest(self,camera,age):
        return camera.read_latest(age) if hasattr(camera,'read_latest') else camera.get()
    def preview_views(self):
        views={};errors={}
        for name,camera in self.cameras.items():
            try:views[name]=self._latest(camera,200)
            except Exception as error:errors[name]=str(error)
        return views,errors
    def preview_sample(self):
        views={name:self._latest(camera,200) for name,camera in self.cameras.items()}
        sample=views['wrist']
        if len(views)==1:return sample
        frames=[]
        for name,(_,_,frame) in views.items():
            image=self.cv2.resize(frame,(320,240))
            self.cv2.putText(image,name.upper(),(8,23),self.cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
            frames.append(image)
        return sample[0],sample[1],self.cv2.hconcat(frames)
    def prepare_recording(self):
        for role,camera in self.cameras.items():
            try:
                if hasattr(camera,'prepare_recording'):camera.prepare_recording()
            except CameraReadinessError as error:
                raise CameraReadinessError(error.diagnostics['status'],'启动预检',role) from error
        self.last_sequences={}
        self.last_stamps={}
        return self.status()
    def recording_anchor(self):
        self.prepare_recording()
        camera=self.cameras['wrist']
        return camera.recording_anchor() if hasattr(camera,'recording_anchor') else time.monotonic()
    def get_for_tick(self,after=-1):
        views={}
        for name,camera in self.cameras.items():
            try:views[name]=self._latest(camera,self.max_recording_age_ms)
            except CameraFrameError as error:
                raise CameraFrameError(error.reason,error.diagnostics,name) from error
        now=time.monotonic_ns()
        for name,(seq,stamp,_) in views.items():
            if seq<=max(self.last_sequences.get(name,-1),after if name=='wrist' else -1):
                raise RuntimeError(f'{name}相机没有新帧，停止本次采集。')
            if not 0<=(now-stamp)/1e6<=self.max_recording_age_ms:
                raise CameraFrameError('双摄取图后帧已过旧或时间异常',
                    {'observed_monotonic_ns':now,'sequence':seq,'frame_receive_monotonic_ns':stamp,
                     'frame_age_ms':(now-stamp)/1e6,'max_age_ms':self.max_recording_age_ms,
                     'streams':{role:{'sequence':v[0],'frame_receive_monotonic_ns':v[1]} for role,v in views.items()}},name)
            if name!='wrist' and name in self.last_stamps and not 50<=(stamp-self.last_stamps[name])/1e6<=150:
                raise RuntimeError('第二路采样间隔超出50～150ms，停止本次采集。')
        if (max(x[1] for x in views.values())-min(x[1] for x in views.values()))/1e6>self.max_pair_skew_ms:
            raise RuntimeError('双摄接收时间差超过35ms，停止本次采集；不是硬件同步相机。')
        self.last_sequences={name:x[0] for name,x in views.items()}
        self.last_stamps={name:x[1] for name,x in views.items()}
        return ViewSample(views)
    def next_recording_tick(self,sample):
        for role,camera in self.cameras.items():
            try:
                if hasattr(camera,'check_recording'):camera.check_recording()
                elif hasattr(camera,'prepare_recording'):camera.prepare_recording()
            except CameraReadinessError as error:
                raise CameraReadinessError(error.diagnostics['status'],'采集中',role) from error
        primary=self.cameras['wrist']
        try:
            return primary.next_recording_tick(sample) if hasattr(primary,'next_recording_tick') else sample[1]/1e9+.1
        except CameraReadinessError as error:
            raise CameraReadinessError(error.diagnostics['status'],'采集中','wrist') from error

def make_rig(cv2,config,demo=False):
    roles=['wrist']+(['overview'] if config['mode']=='dual' else [])
    if demo:
        from .hardware import DemoCamera
        cameras={role:DemoCamera(cv2) for role in roles}
    else:cameras={role:SynchronizedCamera(cv2,config[role]) for role in roles}
    return CameraRig(cv2,cameras,config)
