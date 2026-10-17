#!/usr/bin/env python3
"""Record RGB + joint feedback for diagnostics only; never send motion commands."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import threading
import time
import uuid

from arm_control import read_pose


class LatestCamera:
    def __init__(self, cv2, device):
        self.cv2 = cv2
        self.device = device
        self.condition = threading.Condition()
        self.stopping = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._read, daemon=True)

    def _read(self):
        cap = self.cv2.VideoCapture(self.device, self.cv2.CAP_V4L2)
        try:
            cap.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(self.cv2.CAP_PROP_FPS, 30)
            cap.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                raise RuntimeError("无法打开相机，检查设备路径以及是否被其他程序占用。")
            sequence = 0
            while not self.stopping.is_set():
                ok, frame = cap.read()
                received = time.monotonic_ns()
                if not ok:
                    raise RuntimeError("相机读取失败，检查 USB 连接。")
                with self.condition:
                    self.latest = (sequence, received, frame)
                    sequence += 1
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = str(error)
                self.condition.notify_all()
        finally:
            cap.release()

    def start(self):
        self.thread.start()

    def get(self, after=-1):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.error is not None or
                (self.latest is not None and self.latest[0] > after), timeout=5)
            if self.error:
                raise RuntimeError(self.error)
            if not ready:
                raise RuntimeError("5 秒内未收到新的相机画面。")
            sequence, received, frame = self.latest
            if time.monotonic_ns() - received > 200_000_000:
                raise RuntimeError("最新相机帧已经超过 200 ms，停止检查录制。")
            return sequence, received, frame.copy()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=10, choices=range(1, 61))
    parser.add_argument("--fps", type=int, default=10, choices=range(1, 16))
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--task", default="检查方块搬运场景与图像、关节时间对应")
    parser.add_argument("--root", type=Path, default=Path.home() / "dofbot-data/_checks")
    args = parser.parse_args()
    import cv2
    from Arm_Lib import Arm_Device

    args.root.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(args.root).free < 200 * 1024 * 1024:
        raise RuntimeError("剩余空间不足 200 MB，取消录制。")
    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    output = args.root / name
    output.mkdir()
    (output / "images").mkdir()
    metadata = {
        "schema": "dofbot_capture_check_v1", "purpose": "diagnostic_only",
        "train_ready": False, "contains_actions": False,
        "task": args.task, "camera_device": args.camera,
        "joint_order": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
        "angle_unit": "degree", "image_format": "JPEG; decoded OpenCV images are BGR",
        "requested_sample_hz": args.fps, "requested_seconds": args.seconds,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "clock_anchor": {"unix_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns()},
        "timestamp_meaning": "Camera timestamp is host read completion, not sensor exposure time. Joint readings are sequential within their recorded interval.",
        "status": "in_progress", "frames": 0,
    }
    meta_path = output / "metadata.json"
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    camera = LatestCamera(cv2, args.camera)
    arm = None
    count = 0
    ages = []
    overruns = 0
    first_sample = last_sample = None
    failure = None
    try:
        arm = Arm_Device(i2c_bus=7)
        camera.start()
        camera.get()
        # Let automatic exposure settle while the camera worker drains frames.
        time.sleep(1)
        print(f"开始 {args.seconds} 秒检查录制，不发送运动指令。\n保存位置：{output}", flush=True)
        start = time.monotonic()
        previous_sequence = -1
        with (output / "samples.jsonl").open("w", encoding="utf-8") as records:
            for index in range(args.seconds * args.fps):
                deadline = start + index / args.fps
                time.sleep(max(0, deadline - time.monotonic()))
                sample_started = time.monotonic()
                if sample_started - deadline > 1 / args.fps:
                    overruns += 1
                sequence, frame_time, frame = camera.get(previous_sequence)
                previous_sequence = sequence
                state_start = time.monotonic_ns()
                angles = read_pose(arm)
                state_end = time.monotonic_ns()
                image_path = f"images/{index:06d}.jpg"
                if not cv2.imwrite(str(output / image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                    raise RuntimeError("图像保存失败。")
                age = (state_start - frame_time) / 1e6
                row = {
                    "frame_index": index, "timestamp": sample_started - start,
                    "image": image_path, "image_shape_hwc": list(frame.shape),
                    "camera_sequence": sequence, "frame_receive_monotonic_ns": frame_time,
                    "state_read_start_monotonic_ns": state_start,
                    "state_read_end_monotonic_ns": state_end,
                    "frame_age_at_state_start_ms": age,
                    "observation.state": angles, "action": None,
                }
                records.write(json.dumps(row, ensure_ascii=False) + "\n")
                records.flush()
                count += 1
                ages.append(age)
                if first_sample is None:
                    first_sample = sample_started
                last_sample = sample_started
                if count % args.fps == 0:
                    print(f"已记录 {count} 帧；最近角度 {angles}", flush=True)
        metadata["status"] = "complete"
    except (Exception, KeyboardInterrupt) as error:
        failure = error
        metadata["status"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        metadata["error"] = str(error) or type(error).__name__
    finally:
        if camera.thread.ident is not None:
            camera.close()
        if arm is not None:
            arm.bus.close()
        metadata.update({
            "frames": count, "overrun_count": overruns,
            "effective_sample_hz": ((count - 1) / (last_sample - first_sample)
                                    if count > 1 and last_sample > first_sample else None),
            "max_frame_age_at_state_start_ms": max(ages) if ages else None,
            "finished_utc": datetime.now(timezone.utc).isoformat(),
        })
        meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"录制状态：{metadata['status']}，{count} 帧。\n{output}", flush=True)
        print("这是图像与角度检查数据，缺少操作动作，不能直接当作 VLA 示范训练。", flush=True)
    if failure is not None:
        raise failure


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ValueError) as error:
        print(f"错误：{error}")
        raise SystemExit(1)
