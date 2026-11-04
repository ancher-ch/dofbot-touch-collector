"""Measured fixed-rate camera, freshness-aware reads, phase aligned 10 Hz capture.

Borrow the separation of latest-frame preview and bounded new-frame capture from
LeRobot's camera API. No LeRobot hardware/calibration code or dependencies.
"""
from collections import deque
import subprocess
import time

from capture_check import LatestCamera

class CameraReadinessError(RuntimeError):
    """Retain the exact failed snapshot for metadata and operator diagnostics."""
    def __init__(self,status,phase,role=None):
        self.diagnostics={'phase':phase,'role':role,'status':status}
        name={'wrist':'主/腕部相机','overview':'第二/外部相机'}.get(role,'相机')
        age=status.get('latest_frame_age_ms')
        age_text=f'{age:.1f}ms' if age is not None else '无图像'
        reasons='、'.join(status.get('not_ready_reasons',[])) or '状态异常'
        ending='未开始本段运动。' if phase=='启动预检' else '已停止后续采集指令，保留异常数据。'
        super().__init__(f'{phase}{name}检查失败：{reasons}；{status["fps"]:.1f}fps，'
                         f'P95 {status["p95_interval_ms"]:.1f}ms，最大间隔 {status["max_interval_ms"]:.1f}ms，'
                         f'最新帧年龄 {age_text}。{ending}')


class CameraFrameError(RuntimeError):
    """Exact rejected frame, with an optional physical camera role."""
    def __init__(self,reason,diagnostics,role=None):
        self.reason=reason
        self.diagnostics=dict(diagnostics,phase='取图',role=role,reason=reason)
        name={'wrist':'主/腕部相机','overview':'第二/外部相机'}.get(role,'相机')
        age=diagnostics.get('frame_age_ms')
        detail=f'，帧龄{age:.1f}ms / 上限{diagnostics["max_age_ms"]:.0f}ms' if age is not None else ''
        super().__init__(f'{name}{reason}{detail}；已停止后续采集指令，保留异常数据。')


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
            startup_deadline=time.monotonic()+2
            startup_failures=0
            while not self.stopping.is_set():
                try:
                    ok,frame=cap.read()
                    if not ok or frame is None or frame.size==0:raise RuntimeError('相机读取失败，请检查连接。')
                except (self.cv2.error,RuntimeError):
                    # Some UVC MJPEG devices return an empty first buffer after
                    # reopening. Retry only before the first accepted frame.
                    if sequence or startup_failures>=5 or time.monotonic()>=startup_deadline:raise
                    startup_failures+=1
                    self.configuration['startup_discarded_frames']=startup_failures
                    time.sleep(.02)
                    continue
                received=time.monotonic_ns()
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
            configuration=dict(self.configuration)
            # The age must use the SAME instant as the copied timestamps.
            # Scheduling while computing statistics must not age an old snapshot
            # and ignore newer frames already delivered by the reader thread.
            observed_ns=time.monotonic_ns()
        report=interval_summary(stamps)
        reasons=[]
        if len(stamps)<46:reasons.append('预热样本不足46帧')
        if len(stamps)>=2:
            if not 27<=report['fps']<=33:reasons.append('帧率超出27～33fps')
            if report['p95_interval_ms']>40:reasons.append('P95帧间隔超过40ms')
            if report['max_interval_ms']>55:reasons.append('最大帧间隔超过55ms')
        age=(observed_ns-stamps[-1])/1e6 if stamps else None
        if age is None:reasons.append('尚无图像')
        elif not 0<=age<=100:reasons.append('最新帧超过100ms或时间异常')
        if error:reasons.append('读帧错误：'+error)
        report.update(configuration)
        report.update(controls=controls,error=error)
        # Preserve the original unrounded interval gates; reason strings are
        # explanatory and must not turn rounded boundary values into passes.
        report['ready']=bool(report['ready'] and not error and age is not None and 0<=age<=100)
        report.update(observed_monotonic_ns=observed_ns,latest_frame_age_ms=age,not_ready_reasons=reasons)
        return report

    def read_latest(self,max_age_ms=200):
        # Validate at the same instant as the selected frame snapshot. Copying
        # happens outside the producer lock; if scheduling ages this selection,
        # reselect an ALREADY AVAILABLE newer frame once, never return stale data
        # or wait for a stalled camera. All original age/interval gates remain.
        for attempt in range(2):
            with self.condition:
                observed=time.monotonic_ns()
                if self.error:raise self._frame_error('读帧失败：'+self.error,None,observed,max_age_ms,attempt)
                if self.latest is None:raise self._frame_error('正在预热，尚无图像',None,observed,max_age_ms,attempt)
                sequence,received,frame=self.latest
                if not 0<=observed-received<=max_age_ms*1e6:
                    raise self._frame_error('最新图像过旧或时间异常',self.latest,observed,max_age_ms,attempt)
            copied=frame.copy()
            with self.condition:
                delivered=time.monotonic_ns()
                if self.error:raise self._frame_error('读帧失败：'+self.error,(sequence,received,frame),delivered,max_age_ms,attempt)
                if 0<=delivered-received<=max_age_ms*1e6:
                    return sequence,received,copied
                if attempt==0 and self.latest is not None and self.latest[0]>sequence:
                    continue
                raise self._frame_error('图像复制或线程调度后帧已过旧',(sequence,received,frame),delivered,max_age_ms,attempt)

    def _frame_error(self,reason,sample,observed,max_age_ms,attempt):
        # Called under condition; Condition uses an RLock so status() can retain
        # the same reader snapshot. These fields explain the failed sample,
        # rather than a later, recovered preview state.
        diagnostics={'observed_monotonic_ns':observed,'max_age_ms':max_age_ms,
                     'selection_attempt':attempt+1,'status':self.status()}
        if sample is not None:
            diagnostics.update(sequence=sample[0],frame_receive_monotonic_ns=sample[1],
                               frame_age_ms=(observed-sample[1])/1e6)
        if self.latest is not None:
            diagnostics['latest_sequence_at_rejection']=self.latest[0]
            diagnostics['latest_receive_monotonic_ns_at_rejection']=self.latest[1]
        return CameraFrameError(reason,diagnostics)

    def prepare_recording(self):
        return self._require_ready('启动预检')

    def check_recording(self):
        return self._require_ready('采集中')

    def _require_ready(self,phase):
        s=self.status()
        if not s['ready']:
            raise CameraReadinessError(s,phase)
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
        s=self.check_recording()
        return sample[1]/1e9+3/s['fps']+.012

    def get_for_tick(self,after=-1):
        sample=self.read_latest(self.max_recording_age_ms)
        if sample[0]<=after:raise RuntimeError('本次没有新的相机帧，已停止采集。')
        return sample
