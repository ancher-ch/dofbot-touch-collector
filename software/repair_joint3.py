#!/usr/bin/env python3
"""Inspect joint 3 raw feedback; optionally make one supervised <=5 degree move to 0."""
import argparse
import json
from pathlib import Path
import time

from arm_control import read_angle, read_pose, write_angle
from trajectory_teach import SESSION, save_json


def boundary_plan(raw_position):
    # Same linear mapping as installed SDK; use continuous angle for the step guard.
    if isinstance(raw_position, bool) or not isinstance(raw_position, int) or raw_position <= 0:
        raise ValueError("无效原始位置，不能计划修复。")
    continuous_angle = 180 - 180 * (raw_position - 900) / 2200
    sdk_angle = 180 - int(180 * (raw_position - 900) / 2200)
    if not -5 <= continuous_angle < 0 or sdk_angle >= 0:
        raise ValueError("此工具只处理3号略低于0°、且到0°变化不超过5°的情况。")
    return {"raw_position": raw_position, "sdk_mapped_angle": sdk_angle,
            "continuous_mapped_angle": continuous_angle, "target": 0,
            "maximum_change_degrees": abs(continuous_angle)}


def read_raw_position(arm):
    for _ in range(3):
        arm.bus.write_byte_data(0x15, 0x33, 0)
        time.sleep(0.03)
        word = arm.bus.read_word_data(0x15, 0x33)
        if word:
            return ((word >> 8) & 255) | ((word << 8) & 65280)
    raise RuntimeError("3号原始读数持续无效，不发送修复动作。")


def verify_feedback(arm, timeout=5):
    """Wait for three stable valid feedback sets; never issue a motion command."""
    deadline = time.monotonic() + timeout
    valid_sets = []
    last_error = "未取得稳定反馈"
    while time.monotonic() < deadline:
        try:
            pose = read_pose(arm)
            if not 0 <= pose[2] <= 2:
                raise RuntimeError("3号不在0～2°的目标附近。")
            if valid_sets and max(abs(a-b) for a,b in zip(pose,valid_sets[-1])) > 2:
                valid_sets = []
            valid_sets.append(pose)
            if len(valid_sets) >= 3:
                return pose
        except RuntimeError as error:
            valid_sets = []
            last_error = str(error)
        time.sleep(.2)
    raise RuntimeError(f"反馈未在等待时间内稳定：{last_error}；没有追加运动指令。")


def repair(arm, apply=False, session_path=SESSION):
    raw = read_raw_position(arm)
    sdk_angle = 180 - int(180 * (raw - 900) / 2200)
    if 0 <= sdk_angle <= 2:
        actual = verify_feedback(arm)
        print(f"当前六路反馈已有效：{actual}；无需再次修复，没有发送运动指令。")
        return True
    plan = boundary_plan(raw)
    others = {joint: read_angle(arm, joint) for joint in (1, 2, 4, 5, 6)}
    print(f"3号原始位置：{raw}，按SDK连续公式约 {plan['continuous_mapped_angle']:.2f}°。")
    print(f"计划：只给3号发送0°目标，用时1500ms；变化约 {plan['maximum_change_degrees']:.2f}°。")
    print(f"其他关节当前反馈：{others}")
    if not apply:
        print("只读检查结束，未发送目标角度。需要现场执行时加 --apply。")
        return False
    print("此动作需要3号关节周围有空间。确认关节已抱紧、机械臂稳定，手避开夹缝。")
    print("不会切换扭矩、不会修改零位，也不会移动其他关节。")
    if input("准备好后输入 MOVE3 并回车；其他输入取消：").strip() != "MOVE3":
        print("已取消，未发送角度目标。")
        return False
    fresh = boundary_plan(read_raw_position(arm))
    if abs(fresh["continuous_mapped_angle"] - plan["continuous_mapped_angle"]) > 0.5:
        raise RuntimeError("确认期间3号位置变化，取消修复。")
    for joint, angle in others.items():
        if abs(read_angle(arm, joint) - angle) > 2:
            raise RuntimeError("其他关节位置变化，取消修复。")
    write_angle(arm, 3, 0, 1500)
    time.sleep(1.5)
    actual = verify_feedback(arm)
    print(f"六路反馈已恢复：{actual}")
    if session_path.exists():
        state = json.loads(session_path.read_text(encoding="utf-8"))
        state.update({"learning_may_be_active": False,
                      "operator_confirmed_holding_before_boundary_move": True,
                      "boundary_repair_joint": 3, "boundary_repair_feedback": actual})
        save_json(session_path, state)
    print("完成。本次只修复了反馈边界，不是有效示教数据；先不自动录制或回放。")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="显示计划并等待现场输入MOVE3后执行一次动作")
    args = parser.parse_args()
    import fcntl
    from Arm_Lib import Arm_Device
    with Path(__file__).with_name(".teleop_record.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("其他采集程序仍在运行，请先退出。")
        arm = Arm_Device(i2c_bus=7)
        try:
            repair(arm, args.apply)
        finally:
            arm.bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止后续动作，已发出的短时动作可能继续。")
        raise SystemExit(130)
    except (RuntimeError, ValueError, OSError, EOFError) as error:
        print(f"错误：{error}")
        raise SystemExit(1)
