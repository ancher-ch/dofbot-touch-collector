"""Measured fixed-rate camera, freshness-aware reads, phase aligned 10 Hz capture.

Borrow the separation of latest-frame preview and bounded new-frame capture from
LeRobot's camera API. No LeRobot hardware/calibration code or dependencies.
"""
from collections import deque
import subprocess
import time

from capture_check import LatestCamera


def fixed_rate_control(device):
    """Change only the driver's dynamic-FPS permission, not exposure or gain."""
    result={'control':'exposure_dynamic_framerate','before':None,'after':None,'warning':''}
    try:
        command=['v4l2-ctl','-d',str(device)]
        before=subprocess.run(command+['--get-ctrl','exposure_dynamic_framerate'],capture_output=True,text=True,timeout=2)
        if before.returncode:
            result['warning']='驱动未提供动态帧率开关；以实测帧率检查为准。'
            return result
        result['before']=int(before.stdout.strip().rsplit(':',1)[1])
        if result['before']!=0:
            change=subprocess.run(command+['--set-ctrl','exposure_dynamic_framerate=0'],capture_output=True,text=True,timeout=2)
            if change.returncode:raise RuntimeError(change.stderr.strip())
        after=subprocess.run(command+['--get-ctrl','exposure_dynamic_framerate'],capture_output=True,text=True,timeout=2,check=True)
        result['after']=int(after.stdout.strip().rsplit(':',1)[1])
        if result['after']!=0:result['warning']='相机仍允许动态降帧；请检查驱动设置。'
    except (OSError,ValueError,IndexError,RuntimeError,subprocess.SubprocessError) as error:
        result['warning']='固定帧率设置未完成：'+str(error)
    return result


def interval_summary(stamps,requested=30):
    if len(stamps)<2:
        return {'ready':False,'fps':0.,'max_interval_ms':0.,'p95_interval_ms':0.,'samples':len(stamps)}
    intervals=sorted((b-a)/1e6 for a,b in zip(stamps,stamps[1:]))
    fps=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0])
    p95=intervals[min(len(intervals)-1,int(.95*(len(intervals)-1)))]
    return {'ready':len(stamps)>=46 and requested*.90<=fps<=requested*1.10 and p95<=40 and intervals[-1]<=55,
            'fps':round(fps,3),'max_interval_ms':round(intervals[-1],3),
            'p95_interval_ms':round(p95,3),'samples':len(stamps)}


class SynchronizedCamera(LatestCamera):
    max_recording_age_ms=50
    max_recording_interval_error_ms=20

    def __init__(self,cv2,device):
        super().__init__(cv2,device)
        self.stamps=deque(maxlen=91)
        self.controls={}
        self.configuration={'requested_fps':30,'width':640,'height':480,'format':'MJPG','requested_buffers':4}

    def _read(self):
        cap=self.cv2.VideoCapture(self.device,self.cv2.CAP_V4L2)
        try:
            cap.set(self.cv2.CAP_PROP_FOURCC,self.cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(self.cv2.CAP_PROP_FRAME_WIDTH,640)
            cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT,480)
            cap.set(self.cv2.CAP_PROP_FPS,30)
            # On this Orbbec/V4L2 device one mmap buffer halves delivery rate.
            # Four queued buffers plus continuous background draining measured
            # ~30Hz; a "latest" cache still keeps consumers off the decode path.
            cap.set(self.cv2.CAP_PROP_BUFFERSIZE,4)
            if not cap.isOpened():raise RuntimeError('无法打开相机，请检查连接及占用。')
            self.configuration['driver_reported_fps']=cap.get(self.cv2.CAP_PROP_FPS)
            self.configuration['driver_reported_buffers']=cap.get(self.cv2.CAP_PROP_BUFFERSIZE)
            self.controls=fixed_rate_control(self.device)
            sequence=0
            while not self.stopping.is_set():
                ok,frame=cap.read()
                received=time.monotonic_ns()
                if not ok:raise RuntimeError('相机读取失败，请检查连接。')
                with self.condition:
                    self.latest=(sequence,received,frame)
                    self.stamps.append(received)
                    sequence+=1
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error=str(error)
                self.condition.notify_all()
        finally:cap.release()

    def status(self):
        with self.condition:
            stamps=list(self.stamps)
            error=self.error
            controls=dict(self.controls)
        report=interval_summary(stamps)
        report.update(self.configuration)
        report.update(controls=controls,error=error)
        if error or not stamps or time.monotonic_ns()-stamps[-1]>100_000_000:report['ready']=False
        return report

    def read_latest(self,max_age_ms=200):
        with self.condition:
            if self.error:raise RuntimeError(self.error)
            if self.latest is None:raise RuntimeError('相机正在预热，尚无图像。')
            sequence,received,frame=self.latest
        if not 0<=time.monotonic_ns()-received<=max_age_ms*1e6:
            raise RuntimeError('相机最新图像过旧，请重连或检查帧率。')
        return sequence,received,frame.copy()

    def prepare_recording(self):
        s=self.status()
        if not s['ready']:
            raise RuntimeError(f'相机尚未稳定：实测{s["fps"]:.1f}fps，最大帧间隔{s["max_interval_ms"]:.1f}ms。请等待预热、改善照明或重连；未开始运动。')
        return s

    def recording_anchor(self):
        self.prepare_recording()
        previous=self.read_latest()[0]
        with self.condition:
            ready=self.condition.wait_for(lambda:self.error is not None or
                (self.latest is not None and self.latest[0]>previous),timeout=.12)
            if self.error:raise RuntimeError(self.error)
            if not ready:raise RuntimeError('等待相机新帧超时，未开始本段运动。')
            received=self.latest[1]
        # 12ms phase offset tolerates decode/OS jitter while selecting every 3rd
        # 30Hz frame. It is not a hardware trigger or exposure-time timestamp.
        anchor=received/1e9+.012
        if time.monotonic()>anchor+.01:raise RuntimeError('相机同步启动延迟，请重试。')
        return anchor

    def next_recording_tick(self,sample):
        # 30 nominal FPS measured ~29.7 on this device. Follow its clock every
        # third frame rather than accumulating phase error against perfect 10Hz.
        s=self.status()
        if not s['ready']:raise RuntimeError('相机帧率不再稳定，已停止后续采集指令。')
        return sample[1]/1e9+3/s['fps']+.012

    def get_for_tick(self,after=-1):
        sample=self.read_latest(self.max_recording_age_ms)
        if sample[0]<=after:raise RuntimeError('本次没有新的相机帧，已停止采集。')
        return sample
