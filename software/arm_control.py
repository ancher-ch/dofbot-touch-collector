#!/usr/bin/env python3
"""DOFBOT PRO: read angles, jog a joint, or run bounded movement demos."""
import argparse
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import time


MAX_STEP = 5.0
GROUP_OFFSETS = (10, 0, 0, 0, 15, 15)


def acquire_control_lock(path=None):
    import fcntl
    path = path or Path(__file__).with_name(".teleop_record.lock")
    lock = Path(path).open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("触屏采集软件或其他控制程序正在独占机械臂，请在该界面查看状态或先退出。")
    return lock


def read_angle(arm, joint):
    for _ in range(3):
        value = arm.Arm_serial_servo_read(joint)
        if isinstance(value, (int, float)) and math.isfinite(value):
            if 0 <= value <= (270 if joint == 5 else 180):
                return float(value)
        time.sleep(0.05)
    raise RuntimeError(f"关节 {joint} 读数未通过SDK校验，停止后续运动；可能是通信失败或角度映射超范围，先诊断。")


def target_angle(joint, current, delta):
    if joint not in range(1, 7):
        raise ValueError("关节编号必须为 1～6。")
    if not math.isfinite(delta) or not 0 < abs(delta) <= MAX_STEP:
        raise ValueError("单次相对变化必须大于 0 且不超过 5°。")
    maximum = 270 if joint == 5 else 180
    if not math.isfinite(current) or not 0 <= current <= maximum:
        raise ValueError("当前角度无效，未发送运动指令。")
    # The vendor SDK uses integer position units. Request an integer angle.
    target = round(current + delta)
    if target == round(current):
        raise ValueError("变化小于舵机整数角度分辨率。")
    if not 0 <= target <= maximum or abs(target - current) > MAX_STEP:
        raise ValueError("目标超出 SDK 角度范围或单次 5° 限制，未发送运动指令。")
    return target


def write_angle(arm, joint, target, duration):
    # This SDK prints and swallows I2C write exceptions; surface them to the user.
    output = io.StringIO()
    with redirect_stdout(output):
        arm.Arm_serial_servo_write(joint, target, duration)
    message = output.getvalue().strip()
    if message:
        raise RuntimeError(f"厂商库返回信息，动作结果未知：{message}")


def run_jog(arm, joint, start, target, duration):
    """Send once and require measured progress; never resend on failure."""
    target_angle(joint, start, target - start)
    if not 800 <= duration <= 5000:
        raise ValueError("运动时间必须在 800～5000 ms。")
    latest = read_angle(arm, joint)
    if abs(latest - start) > 1 or abs(target - latest) > MAX_STEP:
        raise RuntimeError("执行前位置发生变化，未发送运动指令。")
    write_angle(arm, joint, target, duration)
    time.sleep(duration / 1000 + 0.5)
    actual = read_angle(arm, joint)
    print(f"目标 {target}°，反馈 {latest:.0f}° → {actual:.0f}°。", flush=True)
    change = target - latest
    if abs(change) >= 2 and (actual - latest) * (1 if change > 0 else -1) < 1:
        raise RuntimeError("反馈未显示朝目标移动，本次测试未通过；不会重发指令，请先诊断。")
    if abs(actual - target) > 2:
        raise RuntimeError("反馈未到达目标附近，本次测试未通过；不会重发指令。")
    return actual


def run_demo(arm, joint, start, target, duration, cycles):
    """Finite out-and-back motion; abort without further writes on bad feedback."""
    if not isinstance(cycles, int) or not 1 <= cycles <= 10:
        raise ValueError("往返次数必须在 1～10 次。")
    if not 800 <= duration <= 5000:
        raise ValueError("运动时间必须在 800～5000 ms。")
    target_angle(joint, start, target - start)
    expected = start
    for cycle in range(1, cycles + 1):
        for destination in (target, round(start)):
            latest = read_angle(arm, joint)
            if abs(latest - expected) > 2:
                raise RuntimeError("关节反馈偏离预期，已停止发送后续动作。")
            if abs(destination - latest) > MAX_STEP:
                raise RuntimeError("当前位置到下一目标超过 5°，已停止发送后续动作。")
            write_angle(arm, joint, destination, duration)
            time.sleep(duration / 1000 + 0.5)
            actual = read_angle(arm, joint)
            print(f"目标 {destination}°，反馈 {latest:.0f}° → {actual:.0f}°。", flush=True)
            requested_change = destination - latest
            if abs(requested_change) >= 2 and (actual - latest) * (1 if requested_change > 0 else -1) < 1:
                raise RuntimeError("反馈没有显示朝目标移动，已停止发送后续动作；请检查现场状态。")
            if abs(actual - destination) > 2:
                raise RuntimeError("关节未到达目标附近，已停止发送后续动作。")
            expected = destination
        print(f"已完成 {cycle}/{cycles} 次往返，反馈 {actual:.0f}°。", flush=True)


def read_pose(arm):
    return [read_angle(arm, joint) for joint in range(1, 7)]


def group_target(start):
    if len(start) != 6:
        raise ValueError("必须读取六个关节角度。")
    result = []
    for joint, (angle, delta) in enumerate(zip(start, GROUP_OFFSETS), 1):
        maximum = 270 if joint == 5 else 180
        if not math.isfinite(angle) or not 0 <= angle <= maximum:
            raise ValueError(f"关节 {joint} 起始角度无效。")
        target = round(angle) + delta
        if not 0 <= target <= maximum:
            raise ValueError(f"关节 {joint} 的演示目标超出 SDK 范围，取消整组动作。")
        result.append(target)
    return result


def write_pose(arm, pose, duration):
    output = io.StringIO()
    with redirect_stdout(output):
        arm.Arm_serial_servo_write6_array(list(pose), duration)
    message = output.getvalue().strip()
    if message:
        raise RuntimeError(f"厂商库返回信息，整组动作结果未知：{message}")


def run_group(arm, start, cycles):
    if not isinstance(cycles, int) or not 1 <= cycles <= 5:
        raise ValueError("同步演示次数必须在 1～5 次。")
    target = group_target(start)
    origin = [round(angle) for angle in start]
    expected = list(start)
    for cycle in range(1, cycles + 1):
        for destination in (target, origin):
            latest = read_pose(arm)
            if any(abs(a - b) > 2 for a, b in zip(latest, expected)):
                raise RuntimeError("关节位置偏离预期，停止发送整组动作。")
            write_pose(arm, destination, 2500)
            time.sleep(3)
            actual = read_pose(arm)
            print("反馈：" + "，".join(f"{j}号 {a:.0f}°→{b:.0f}°"
                  for j, a, b in zip(range(1, 7), latest, actual)), flush=True)
            if any(abs(a - b) > 2 for a, b in zip(actual, destination)):
                raise RuntimeError("存在关节未到达目标附近，停止发送后续动作。")
            expected = destination
        print(f"已完成 {cycle}/{cycles} 次同步往返。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=7, help="I2C bus; installed vendor SDK defaults to 7")
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("status", help="读取六个关节角度，不发送运动指令")
    jog = commands.add_parser("jog", help="单关节相对微调，执行前需在终端输入 MOVE")
    jog.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    jog.add_argument("--delta", type=float, required=True, help="角度变化，单次最多 ±5°")
    jog.add_argument("--duration-ms", type=int, default=1500, help="运动时间，范围 800～5000 ms")
    demo = commands.add_parser("demo", help="单关节小幅往返，有限次数，输入 MOVE 后倒计时 5 秒")
    demo.add_argument("--joint", type=int, choices=range(1, 7), default=6)
    demo.add_argument("--delta", type=float, default=-3, help="相对起点的另一端角度，最多 ±5°")
    demo.add_argument("--cycles", type=int, choices=range(1, 11), default=5)
    demo.add_argument("--duration-ms", type=int, default=1500, help="每程运动时间，范围 800～5000 ms")
    group = commands.add_parser("group-demo", help="底座 +10°、手腕 +15°、夹爪 +15°同步往返")
    group.add_argument("--cycles", type=int, choices=range(1, 6), default=3)
    args = parser.parse_args()
    learning_state = Path(__file__).with_name(".learning_session.json")
    if args.command in ("jog", "demo", "group-demo") and learning_state.exists():
        if json.loads(learning_state.read_text(encoding="utf-8")).get("learning_may_be_active", True):
            parser.error("拖动示教可能仍在松力，先运行 trajectory_teach.py restore。")
    if args.command in ("jog", "demo"):
        if not 800 <= args.duration_ms <= 5000:
            parser.error("运动时间必须在 800～5000 ms。")
        if not math.isfinite(args.delta) or not 0 < abs(args.delta) <= MAX_STEP:
            parser.error("单次变化必须大于 0 且不超过 ±5°。")
    # Import after parsing so --help and argument validation do not open I2C.
    control_lock = acquire_control_lock()
    arm = None
    try:
        from Arm_Lib import Arm_Device
        arm = Arm_Device(i2c_bus=args.bus)
        if args.command == "group-demo":
            start = read_pose(arm)
            target = group_target(start)
            print("计划：底座、手腕、夹爪同时往返；2、3、4号保持起始角度目标。", flush=True)
            for joint in (1, 5, 6):
                print(f"关节 {joint}：{start[joint - 1]:.0f}° ↔ {target[joint - 1]}°。", flush=True)
            print(f"每程 2.5 秒，往返 {args.cycles} 次，约 {6 * args.cycles} 秒。", flush=True)
            print("请清空夹爪和底座转动扫过的空间，确认线缆不受拉扯，停止其他控制程序。", flush=True)
            if input("输入 MOVE 并回车执行；其他输入取消：").strip() != "MOVE":
                print("已取消，未发送运动指令。")
                return
            for seconds in range(5, 0, -1):
                print(f"{seconds} 秒后开始……", flush=True)
                time.sleep(1)
            run_group(arm, start, args.cycles)
            print("同步演示结束，不再发送动作。", flush=True)
            return
        if args.command not in ("jog", "demo"):
            for joint in range(1, 7):
                suffix = "（夹爪）" if joint == 6 else ""
                print(f"关节 {joint}{suffix}: {read_angle(arm, joint):.0f}°", flush=True)
            return
        current = read_angle(arm, args.joint)
        target = target_angle(args.joint, current, args.delta)
        if args.command == "demo":
            seconds = 2 * args.cycles * (args.duration_ms / 1000 + 0.5)
            print(f"计划：关节 {args.joint}，{current:.0f}° ↔ {target}°，往返 {args.cycles} 次，约 {seconds:.0f} 秒。", flush=True)
        else:
            print(f"计划：关节 {args.joint}，{current:.0f}° → {target}°，用时 {args.duration_ms} ms。", flush=True)
        print("确认运动路径无遮挡、手已远离；若操作夹爪，确认夹爪为空。", flush=True)
        if input("输入 MOVE 并回车执行；其他输入取消：").strip() != "MOVE":
            print("已取消，未发送运动指令。")
            return
        if args.command == "demo":
            for seconds in range(5, 0, -1):
                print(f"{seconds} 秒后开始……", flush=True)
                time.sleep(1)
        latest = read_angle(arm, args.joint)
        if abs(latest - current) > 1:
            raise RuntimeError("确认期间关节位置发生变化，已取消。请停止其他控制程序后重试。")
        if abs(target - latest) > MAX_STEP:
            raise RuntimeError("当前位置到目标超过 5°，已取消。")
        if args.command == "demo":
            run_demo(arm, args.joint, current, target, args.duration_ms, args.cycles)
            print("演示结束，不再发送动作。", flush=True)
            return
        run_jog(arm, args.joint, current, target, args.duration_ms)
        print("反馈检查通过；实际机械运动仍需现场观察。", flush=True)
    finally:
        if arm is not None:
            arm.bus.close()
        control_lock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序退出。已发送的舵机运动可能继续，Ctrl+C 不等同于硬件急停。")
        raise SystemExit(130)
    except (RuntimeError, ValueError, OSError, EOFError) as error:
        print(f"错误：{error}")
        raise SystemExit(1)
