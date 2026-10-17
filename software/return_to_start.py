#!/usr/bin/env python3
"""Supervised, bounded return to the saved teaching start; no torque changes."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import time

from arm_control import read_pose, write_angle
from trajectory_teach import SESSION, session_active, save_json, valid_pose, near_pose
import json


ORDER = (2, 3, 4, 1, 5, 6)
STEP = 3
DURATION_MS = 1500


def return_steps(arm, reference, start, events, should_stop=lambda: False):
    reference = valid_pose(reference)
    expected = valid_pose(start)
    if any(value != round(value) for value in reference):
        raise ValueError("起点必须为SDK整数角度。")
    deadline = time.monotonic() + 180
    count = 0
    for joint in ORDER:
        while True:
            if should_stop() or time.monotonic() > deadline:
                raise RuntimeError("已停止后续回位指令（停止请求或超时）。")
            actual = read_pose(arm)
            if not near_pose(actual, expected, 2):
                raise RuntimeError("姿态发生意外变化，停止后续指令。")
            difference = reference[joint - 1] - actual[joint - 1]
            if abs(difference) <= 1:
                expected = actual
                break
            count += 1
            if count > 120:
                raise RuntimeError("回位步数异常，停止后续指令。")
            destination = round(actual[joint - 1] + max(-STEP, min(STEP, difference)))
            if abs(destination - actual[joint - 1]) > STEP:
                raise RuntimeError("单次目标变化超出3°，未发送。")
            event = {"joint": joint, "before": actual, "target_deg": destination,
                     "duration_ms": DURATION_MS, "time": time.monotonic()}
            events.append(event)
            write_angle(arm, joint, destination, DURATION_MS)
            time.sleep(DURATION_MS / 1000 + 0.4)
            after = read_pose(arm)
            event["after"] = after
            print(f"{count}: {joint}号 {actual[joint-1]:.0f}° → 目标{destination}°，反馈{after[joint-1]:.0f}°；起点目标{reference[joint-1]:.0f}°", flush=True)
            requested = destination - actual[joint - 1]
            progress = (after[joint - 1] - actual[joint - 1]) * (1 if requested > 0 else -1)
            if abs(after[joint - 1] - destination) > 2 or progress < 1:
                raise RuntimeError("未按目标移动，停止且不重发指令。")
            if any(abs(after[k] - actual[k]) > 2 for k in range(6) if k != joint - 1):
                raise RuntimeError("其他关节发生意外变化，停止后续指令。")
            expected = after
    final = read_pose(arm)
    if not near_pose(final, reference, 2):
        raise RuntimeError("最终姿态未通过起点误差检查。")
    return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="显示计划并等待现场输入RETURN才执行")
    args = parser.parse_args()
    if args.apply and not sys.stdin.isatty():
        parser.error("执行需要交互终端。")
    if session_active():
        raise RuntimeError("现场抱紧尚未确认，禁止回位；先完成hold-current/HOLDING。")
    state = json.loads(SESSION.read_text(encoding="utf-8"))
    reference = valid_pose(state["reference_pose"])
    import fcntl
    from Arm_Lib import Arm_Device
    with Path(__file__).with_name(".teleop_record.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = Arm_Device(i2c_bus=7)
        report = {"status": "plan_only", "reference": reference, "events": [],
                  "torque_commands_sent": False, "source_session": str(SESSION)}
        try:
            start = read_pose(arm)
            for _ in range(2):
                time.sleep(0.2)
                if not near_pose(read_pose(arm), start, 1):
                    raise RuntimeError("当前角度不稳定，未发送运动指令。")
            report["start"] = start
            print(f"当前：{start}\n本次录制起点：{reference}", flush=True)
            print("顺序：2号→3号→4号→1号→5号→6号；每步最多3°，用时1.5秒。", flush=True)
            print("小步不等于避障。确认已抱紧、夹爪为空、手和支撑已离开路径，沿途及线缆有余量。", flush=True)
            if not args.apply:
                print("只显示计划，未发送运动指令。加--apply进入现场确认。", flush=True)
                return
            if input("路径确认无障碍，输入 RETURN 执行；其他输入取消：").strip() != "RETURN":
                return
            for seconds in range(5, 0, -1):
                print(f"{seconds}秒后开始；Ctrl+C取消。", flush=True)
                time.sleep(1)
            if session_active():
                raise RuntimeError("状态标记改变，取消回位。")
            report["status"] = "moving"
            report["final"] = return_steps(arm, reference, start, report["events"])
            report["status"] = "complete"
            print(f"回位完成：{report['final']}；没有启动录制或回放。", flush=True)
        except BaseException as error:
            report["status"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
            report["error"] = str(error)
            raise
        finally:
            arm.bus.close()
            name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            save_json(Path.home() / "dofbot-data/reports" / f"return_to_start_{name}.json", report)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止后续指令；已发出的短时运动可能继续完成。")
        raise SystemExit(130)
    except (RuntimeError, ValueError, OSError, EOFError) as error:
        print(f"停止：{error}")
        raise SystemExit(1)
