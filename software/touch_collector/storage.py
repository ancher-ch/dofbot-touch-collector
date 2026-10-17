"""Self-contained episodes, atomic metadata, reversible deletion. No robot access."""
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import uuid

from trajectory_teach import save_json


def identifier():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


class Store:
    def __init__(self, root):
        self.root = Path(root).resolve()
        for name in ("teachings", "episodes", "trash", "reports"):
            (self.root / name).mkdir(parents=True, exist_ok=True)

    def item(self, kind, item_id):
        if kind not in ("teachings", "episodes") or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", item_id):
            raise ValueError("数据编号无效。")
        p = self.root / kind / item_id
        p.resolve().relative_to(self.root / kind)
        return p

    def list_items(self, kind):
        result = []
        for p in (self.root / kind).glob("*/metadata.json"):
            try:
                d = load(p)
                if not isinstance(d,dict):continue
                d["id"] = p.parent.name
                result.append(d)
            except (ValueError, OSError, TypeError):
                continue
        return sorted(result, key=lambda x: x["id"], reverse=True)

    def new_teaching(self, task, home):
        item_id = identifier()
        p = self.item("teachings", item_id)
        p.mkdir()
        d = {"schema": "dofbot_joint_teaching_v1", "id": item_id, "task": task,
             "status": "draft", "camera_recorded": False, "contains_motor_actions": False,
             "teaching_method": "direct_torque_switch", "reference_pose": list(home),
             "joint_order": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
             "angle_unit": "degree", "samples": []}
        self.write_teaching(d)
        return d

    def write_teaching(self, d):
        p = self.item("teachings", d["id"])
        save_json(p / "trajectory.json", d)
        save_json(p / "metadata.json", {k: v for k, v in d.items() if k != "samples"} | {"samples_count": len(d["samples"])})

    def trash(self, kind, item_id):
        src = self.item(kind, item_id)
        if not src.exists():
            return
        # Resolve both sides within this store; never recursively delete user data.
        src.resolve().relative_to(self.root)
        dst = self.root / "trash" / f"{kind}_{item_id}_{uuid.uuid4().hex[:6]}"
        dst.resolve().relative_to(self.root / "trash")
        save_json(src/'trash_manifest.json',{'kind':kind,'original_id':item_id,
                  'deleted_utc':datetime.now(timezone.utc).isoformat()})
        src.rename(dst)
        return dst


class Capture:
    @classmethod
    def reopen(cls, store, item_id, cv2):
        obj = cls.__new__(cls)
        obj.cv2 = cv2
        obj.id = item_id
        obj.path = store.item("episodes", item_id)
        obj.meta = load(obj.path / "metadata.json")
        obj.rows = io.StringIO()
        obj.rows.close()
        obj.timestamps = load(obj.path / "frame_timestamps.json") if (obj.path / "frame_timestamps.json").exists() else []
        obj.last_sequence = -1
        obj.origin = None
        return obj

    def __init__(self, store, teaching, cv2):
        self.cv2 = cv2
        self.id = identifier()
        self.path = store.item("episodes", self.id)
        self.path.mkdir()
        (self.path / "images").mkdir()
        save_json(self.path / "teaching.json", teaching)
        source = (self.path / "teaching.json").read_bytes()
        self.meta = {"schema": "dofbot_touch_episode_v1", "id": self.id, "status": "recording",
                     "task": teaching["task"], "teaching_id": teaching["id"],
                     "source_sha256": hashlib.sha256(source).hexdigest(),
                     "operator_success": None, "train_ready": False, "capture_checks_passed": False,
                     "fps": 10, "frames": 0, "video": "video.mp4",
                     "state_unit": "degree", "action_type": "absolute_joint_position_target",
                     "joint_order": teaching["joint_order"], "camera_device": "/dev/video0",
                     "timestamp_note": "Host frame receive time is not exposure time. State precedes NEW command writes; held targets reference the previous active command. Video frame n maps to JSONL frame_index n; use recorded timestamps for timing.",
                     "created_utc": datetime.now(timezone.utc).isoformat()}
        self.rows = (self.path / "samples.jsonl").open("w", encoding="utf-8")
        self.last_sequence = -1
        self.view_sequences = {}
        self.origin = None
        self.timestamps = []
        self.write_meta()

    def write_meta(self):
        save_json(self.path / "metadata.json", self.meta)

    def add(self, camera_sample, state, action, read_start, read_end, write_start, write_end, tick, command_sent=True):
        if command_sent:
            if write_start is None or write_end is None or not read_end<=write_start<=write_end:
                raise RuntimeError("实际指令时间顺序无效。")
            self.last_command = (list(action),write_start,write_end)
        elif write_start is not None or write_end is not None or not hasattr(self,"last_command") or list(action)!=self.last_command[0]:
            raise RuntimeError("保持目标帧必须引用已经发送的同一目标，不能伪造新指令。")
        sequence, frame_ns, frame = camera_sample
        if sequence <= self.last_sequence:
            raise RuntimeError("相机帧重复，停止采集。")
        age = (read_start - frame_ns) / 1e6
        if not 0 <= age <= 200:
            raise RuntimeError("相机帧与状态时间差超过200ms，停止采集。")
        index = self.meta["frames"]
        filename = f"images/{index:06d}.jpg"
        views=getattr(camera_sample,'views',{'wrist':camera_sample})
        if set(views) not in ({'wrist'},{'wrist','overview'}):raise RuntimeError('相机角色无效。')
        if hasattr(self,'recorded_roles') and set(views)!=self.recorded_roles:raise RuntimeError('采集中不能切换相机数量。')
        self.recorded_roles=set(views)
        if (max(x[1] for x in views.values())-min(x[1] for x in views.values()))/1e6>35:
            raise RuntimeError('双摄配对时间差超过35ms。')
        if not hasattr(self,'view_sequences'):self.view_sequences={}
        camera_rows={}
        for name,(seq,received,image) in views.items():
            if seq<=self.view_sequences.get(name,-1):raise RuntimeError(f'{name}相机帧重复。')
            view_age=(read_start-received)/1e6
            if not 0<=view_age<=50 and len(views)>1:raise RuntimeError('双摄图像过旧，不能保存为完整采集。')
            directory='images' if name=='wrist' else 'images_overview'
            path=self.path/directory
            path.mkdir(exist_ok=True)
            if name!='wrist' and not self.cv2.imwrite(str(path/f'{index:06d}.jpg'),image,[self.cv2.IMWRITE_JPEG_QUALITY,90]):
                raise RuntimeError('第二路相机图像保存失败。')
            camera_rows[name]={'image':f'{directory}/{index:06d}.jpg','camera_sequence':seq,
                               'frame_receive_monotonic_ns':received,'frame_age_at_state_start_ms':view_age}
        self.view_sequences={name:sample[0] for name,sample in views.items()}
        self.meta['camera_mode']='dual' if len(views)==2 else 'single'
        self.meta['camera_streams']={name:{'images':'images' if name=='wrist' else 'images_overview',
            'video':'video.mp4' if name=='wrist' else 'video_overview.mp4'} for name in views}
        if not self.cv2.imwrite(str(self.path / filename), frame, [self.cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise RuntimeError("图像保存失败。")
        if self.origin is None:
            self.origin = tick
        stamp = tick - self.origin
        row = {"frame_index": index, "timestamp": stamp, "image": filename,
               "camera_sequence": sequence, "frame_receive_monotonic_ns": frame_ns,"cameras":camera_rows,
               "camera_pair_skew_ms":(max(x[1] for x in views.values())-min(x[1] for x in views.values()))/1e6,
               "state_read_start_monotonic_ns": read_start, "state_read_end_monotonic_ns": read_end,
               "frame_age_at_state_start_ms": age, "observation.state": list(state),
               "action": list(action), "action_write_start_monotonic_ns": write_start,
               "action_write_end_monotonic_ns": write_end, "action_duration_ms": 100 if command_sent else None,
               "command_sent":command_sent,"active_command_write_start_monotonic_ns":self.last_command[1],
               "active_command_write_end_monotonic_ns":self.last_command[2]}
        self.rows.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.rows.flush()
        self.timestamps.append(stamp)
        self.meta["frames"] += 1
        self.meta["commands_sent"] = self.meta.get("commands_sent",0)+int(command_sent)
        self.meta["action_semantics"] = "active_joint_position_target; command_sent indicates a new bus write"
        self.last_sequence = sequence

    def finish(self, status, reason=None):
        if not self.rows.closed:
            self.rows.close()
        self.meta["status"] = status
        self.meta["reason"] = reason
        self.meta["sample_duration_s"] = self.timestamps[-1] if self.timestamps else 0
        if len(self.timestamps)>1 and self.timestamps[-1]>self.timestamps[0]:
            self.meta["actual_observation_hz"]=(len(self.timestamps)-1)/(self.timestamps[-1]-self.timestamps[0])
        save_json(self.path / "frame_timestamps.json", self.timestamps)
        self.write_meta()

    def encode(self, stop):
        for role in self.meta.get('camera_streams',{'wrist':{}}):
            self._encode_stream(stop,role)
        self.meta['capture_checks_passed']=True
        self.meta['status']='draft'
        self.write_meta()

    def _encode_stream(self,stop,role):
        if self.meta["frames"] < 2:
            raise RuntimeError("有效帧不足，不能生成视频。")
        if not shutil.which("ffmpeg"):
            raise RuntimeError("缺少ffmpeg，原始图像和动作已保留。")
        directory='images' if role=='wrist' else 'images_overview'
        video='video.mp4' if role=='wrist' else 'video_overview.mp4'
        command = ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                   "-framerate", "10", "-start_number", "0", "-i", str(self.path / directory / "%06d.jpg"),
                   "-frames:v", str(self.meta["frames"]), "-c:v", "libx264", "-threads", "2",
                   "-preset", "ultrafast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                   str(self.path / video)]
        deadline = time.monotonic() + 120
        with (self.path / f"encode_{role}.log").open("wb") as log:
            process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=log)
            while process.poll() is None:
                if stop.is_set() or time.monotonic() > deadline:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    raise RuntimeError("视频生成已停止；原始图像和动作仍保留。")
                time.sleep(0.1)
            if process.returncode:
                raise RuntimeError("视频编码失败；原始图像和动作仍保留，查看encode.log。")
        probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames",
                                "-show_entries", "stream=nb_read_frames", "-of", "json", str(self.path / video)],
                               capture_output=True, text=True, timeout=20)
        if probe.returncode or int(json.loads(probe.stdout)["streams"][0]["nb_read_frames"]) != self.meta["frames"]:
            raise RuntimeError("视频帧数与动作行数不一致，不能保存为完整数据。")
