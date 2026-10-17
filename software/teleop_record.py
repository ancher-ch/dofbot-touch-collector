#!/usr/bin/env python3
"""Keyboard demonstration recorder for DOFBOT PRO; raw episodes, not LeRobot export."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import select
import shutil
import sys
import threading
import time
import uuid

from arm_control import read_pose, write_pose
from capture_check import LatestCamera


FPS = 10
DURATION_MS = 200


def apply_key(target, joint, direction):
    """One key requests at most one degree on one joint, within SDK ranges."""
    if joint not in range(1, 7) or direction not in (-1, 1) or len(target) != 6:
        raise ValueError("无效的关节微调请求。")
    result = list(target)
    for j, angle in enumerate(result, 1):
        if not math.isfinite(angle) or not 0 <= angle <= (270 if j == 5 else 180):
            raise ValueError("目标角度超出 SDK 范围。")
    value = result[joint - 1] + direction
    if not 0 <= value <= (270 if joint == 5 else 180):
        raise ValueError("已到达 SDK 角度边界，未继续移动。")
    result[joint - 1] = value
    return result


def check_tracking(current, target):
    if len(current) != 6 or len(target) != 6 or any(
            not isinstance(a, (int, float)) or not math.isfinite(a)
            for a in list(current) + list(target)):
        raise RuntimeError("关节反馈或目标无效。")
    if any(abs(a - b) > 5 for a, b in zip(current, target)):
        raise RuntimeError("反馈与已下发目标相差超过 5°，停止指令；检查是否碰到物体、限位或受到其他程序控制。")


class Episode:
    def __init__(self, root, task, camera_device, cv2):
        self.cv2 = cv2
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.path = root / name
        self.path.mkdir()
        (self.path / "images").mkdir()
        self.started = time.monotonic()
        self.count = 0
        self.sample_origin = None
        self.last_time = None
        self.metadata = {
            "schema": "dofbot_raw_demonstration_v1", "task": task,
            "status": "recording", "operator_success": None,
            "train_ready": False, "format_note": "Raw capture; audit and convert to pinned LeRobot version before training.",
            "camera_device": camera_device, "sample_hz_requested": FPS,
            "joint_order": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
            "state_unit": "degree", "action_unit": "degree", "action_type": "absolute_joint_position_target",
            "control_duration_ms": DURATION_MS,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "clock_anchor": {"unix_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns()},
            "timestamp_note": "Frame host receive time is not sensor exposure time; joint feedback is sequential within its recorded interval. Observation precedes action write.",
        }
        self._write_meta()
        self.rows = (self.path / "samples.jsonl").open("w", encoding="utf-8")

    def _write_meta(self):
        (self.path / "metadata.json").write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    def add(self, frame, sequence, frame_ns, state, state_start, state_end,
            action, action_start, action_end, tick_time, action_duration_ms=DURATION_MS):
        filename = f"images/{self.count:06d}.jpg"
        if not self.cv2.imwrite(str(self.path / filename), frame, [self.cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise RuntimeError("图像写入失败，停止录制。")
        if self.sample_origin is None:
            self.sample_origin = tick_time
        relative_time = tick_time - self.sample_origin
        row = {
            "frame_index": self.count, "timestamp": relative_time,
            "image": filename, "image_shape_hwc": list(frame.shape),
            "camera_sequence": sequence, "frame_receive_monotonic_ns": frame_ns,
            "state_read_start_monotonic_ns": state_start, "state_read_end_monotonic_ns": state_end,
            "frame_age_at_state_start_ms": (state_start - frame_ns) / 1e6,
            "observation.state": list(state), "action": list(action),
            "action_write_start_monotonic_ns": action_start, "action_write_end_monotonic_ns": action_end,
            "action_duration_ms": action_duration_ms,
        }
        self.rows.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.rows.flush()
        self.count += 1
        self.last_time = relative_time

    def finish(self, outcome, reason=None):
        self.rows.close()
        self.metadata.update({
            "status": outcome, "operator_success": (outcome == "success" if outcome in ("success", "failed") else None),
            "frames": self.count, "duration_s": time.monotonic() - self.started,
            "finished_utc": datetime.now(timezone.utc).isoformat(),
        })
        if reason:
            self.metadata["reason"] = reason
        self._write_meta()
        print(f"\n本条已保存：{outcome}，{self.count} 帧\n{self.path}", flush=True)


def start_preview(camera, cv2, port):
    class PreviewHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                body = ('<!doctype html><meta charset="utf-8"><title>DOFBOT相机预览</title>'
                        '<body style="background:#161616;color:white;font-family:sans-serif;text-align:center">'
                        '<h2>DOFBOT 相机预览</h2><p>此页面只显示相机；在 SSH 终端操作和录制。</p>'
                        '<img id="view" style="max-width:96vw;max-height:75vh">'
                        '<script>const v=document.getElementById("view");'
                        'function next(){v.src="/frame.jpg?t="+Date.now()}'
                        'v.onload=()=>setTimeout(next,200);v.onerror=()=>setTimeout(next,1000);next();</script>').encode("utf-8")
                content_type = "text/html; charset=utf-8"
            elif path == "/frame.jpg":
                try:
                    _, _, frame = camera.get()
                    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        raise RuntimeError("JPEG encoding failed")
                    body = encoded.tobytes()
                    content_type = "image/jpeg"
                except Exception:
                    self.send_error(503, "Camera unavailable")
                    return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), PreviewHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def show_help():
    print("\n按 1～6 选择关节（6为夹爪）；a 减少1°，d 增加1°。")
    print("r 开始一条录制；s 结束并标记成功；f 结束并标记失败；q 退出；p 保存当前预览；h 帮助。")
    print("先在未录制时熟悉动作并摆到起点，再按 r；复位和重新摆物体时不要录制。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default="把方块抓起放进固定盒子")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--root", type=Path, default=Path.home() / "dofbot-data/pick_place_raw")
    parser.add_argument("--preview-port", type=int, default=8765)
    args = parser.parse_args()
    if not 1024 <= args.preview_port <= 65535:
        parser.error("预览端口必须在 1024～65535。")
    if not sys.stdin.isatty():
        parser.error("需要交互终端；从电脑直接启动时请使用 ssh -t。")
    learning_state = Path(__file__).with_name(".learning_session.json")
    if learning_state.exists() and json.loads(learning_state.read_text(encoding="utf-8")).get("learning_may_be_active", True):
        raise RuntimeError("上次拖动示教可能仍在松力，先运行 trajectory_teach.py restore。")
    import cv2
    import fcntl
    import termios
    import tty
    from Arm_Lib import Arm_Device

    args.root.mkdir(parents=True, exist_ok=True)
    # Prevent two copies of this recorder from controlling the same arm.
    lock = Path(__file__).with_name(".teleop_record.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("已有示教采集程序运行，不能重复启动。")
    camera = LatestCamera(cv2, args.camera)
    arm = None
    episode = None
    original_terminal = None
    preview_server = None
    terminal_fd = sys.stdin.fileno()
    try:
        camera.start()
        _, _, frame = camera.get()
        preview = args.root / "latest_preview.jpg"
        if not cv2.imwrite(str(preview), frame):
            raise RuntimeError("预览图保存失败。")
        preview_server = start_preview(camera, cv2, args.preview_port)
        print(f"电脑浏览器打开 http://127.0.0.1:{args.preview_port}/ 看实时画面（仅预览）。", flush=True)
        arm = Arm_Device(i2c_bus=7)
        target = [round(a) for a in read_pose(arm)]
        print(f"已连接，相机预览保存于：{preview}\n当前角度：{target}")
        show_help()
        print("SDK角度范围不代表无碰撞。先清空夹爪、留出空间，并退出其他控制程序。")
        print("本程序不会自动抓取；只有 a/d 微调或录制中的目标保持会发送指令。")
        if input("输入 MOVE 进入手动示教；其他输入退出：").strip() != "MOVE":
            return
        original_terminal = termios.tcgetattr(terminal_fd)
        tty.setcbreak(terminal_fd)
        joint = 1
        previous_sequence = -1
        next_tick = time.monotonic()
        last_display = 0
        while True:
            time.sleep(max(0, next_tick - time.monotonic()))
            tick = time.monotonic()
            if episode and tick - next_tick > 0.2:
                raise RuntimeError("采样循环延迟超过 200 ms，停止本条录制和后续指令。")
            next_tick = tick + 1 / FPS
            keys = ""
            while select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(terminal_fd, 128)
                if not chunk:
                    raise RuntimeError("控制终端断开。")
                keys += chunk.decode("utf-8", errors="ignore")
            if "q" in keys:
                break
            if "h" in keys:
                show_help()
            # End this episode before allowing a reset or other commands.
            if episode and ("s" in keys or "f" in keys):
                episode.finish("failed" if "f" in keys else "success")
                episode = None
                keys = ""
            sequence, frame_ns, frame = camera.get(previous_sequence)
            previous_sequence = sequence
            state_start = time.monotonic_ns()
            state = read_pose(arm)
            state_end = time.monotonic_ns()
            check_tracking(state, target)
            if "p" in keys:
                if not cv2.imwrite(str(preview), frame):
                    raise RuntimeError("预览图保存失败。")
                print(f"\n预览已更新：{preview}", flush=True)
            if "r" in keys and episode is None:
                if shutil.disk_usage(args.root).free < 500 * 1024 * 1024:
                    raise RuntimeError("剩余空间不足 500 MB，不能开始新录制。")
                episode = Episode(args.root, args.task, args.camera, cv2)
                print(f"\n开始录制：{episode.path.name}", flush=True)
                # Avoid carrying preparation keypresses into the first recorded action.
                keys = ""
            moved = False
            for key in keys:
                if key in "123456":
                    joint = int(key)
                elif key in "ad" and not moved:
                    try:
                        target = apply_key(target, joint, -1 if key == "a" else 1)
                        moved = True
                    except ValueError as error:
                        print(f"\n{error}", flush=True)
            if moved or episode:
                action_start = time.monotonic_ns()
                write_pose(arm, target, DURATION_MS)
                action_end = time.monotonic_ns()
                if episode:
                    episode.add(frame, sequence, frame_ns, state, state_start, state_end,
                                target, action_start, action_end, tick)
                    if time.monotonic() - episode.started > 120:
                        episode.finish("failed", "episode_time_limit_120s")
                        episode = None
            if tick - last_display >= 1:
                mode = f"录制 {episode.count}帧" if episode else "未录制"
                print(f"\r[{mode}] 选中 {joint}号；角度 {[round(a) for a in state]}    ", end="", flush=True)
                last_display = tick
    except Exception as error:
        if episode is not None:
            episode.finish("error", str(error))
            episode = None
        raise
    finally:
        if original_terminal is not None:
            termios.tcsetattr(terminal_fd, termios.TCSADRAIN, original_terminal)
        if episode is not None:
            episode.finish("interrupted", "program exited before operator marked outcome")
        if preview_server is not None:
            preview_server.shutdown()
            preview_server.server_close()
        if camera.thread.ident is not None:
            camera.close()
        if arm is not None:
            arm.bus.close()
        lock.close()
        print("\n程序退出，不再发送指令；已发出的舵机动作可能继续完成。", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (RuntimeError, OSError, ValueError, EOFError) as error:
        print(f"\n错误：{error}")
        raise SystemExit(1)
