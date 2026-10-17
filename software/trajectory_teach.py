#!/usr/bin/env python3
"""Kinesthetic joint teaching, then a separate supervised camera-recorded replay."""
import argparse
from bisect import bisect_right
from contextlib import redirect_stdout
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import select
import shutil
import sys
import time
import uuid

from arm_control import read_pose, write_pose
from capture_check import LatestCamera
from teleop_record import Episode, check_tracking, start_preview


HOME = Path.home() / "dofbot-data"
TRAJECTORIES = HOME / "datasets/joint_trajectories"
SESSION = Path(__file__).with_name(".learning_session.json")
HZ = 10
REPLAY_DURATION_MS = 100


def save_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def valid_pose(pose):
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError("轨迹必须包含六个关节角度。")
    for joint, value in enumerate(pose, 1):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("轨迹角度必须为有限数值。")
        if not 0 <= value <= (270 if joint == 5 else 180):
            raise ValueError(f"关节 {joint} 的角度超出 SDK 范围。")
    return list(pose)


def near_pose(actual, expected, tolerance=2):
    return max(abs(a - b) for a, b in zip(valid_pose(actual), valid_pose(expected))) <= tolerance


def learning_command(arm, enable):
    if enable not in (0, 1):
        raise ValueError("学习模式参数无效。")
    output = io.StringIO()
    with redirect_stdout(output):
        arm.Arm_Button_Mode(enable)
    if output.getvalue().strip():
        raise RuntimeError("学习模式指令结果未知：" + output.getvalue().strip())


def torque_command(arm, enable):
    if enable not in (0, 1):
        raise ValueError("扭矩开关参数无效。")
    output = io.StringIO()
    with redirect_stdout(output):
        arm.Arm_serial_set_torque(enable)
    if output.getvalue().strip():
        raise RuntimeError("扭矩开关指令结果未知：" + output.getvalue().strip())


def close_release(arm, session):
    if session.get("release_method", "learning_mode") == "torque_switch":
        torque_command(arm, 1)
    else:
        learning_command(arm, 0)


def session_active(path=None):
    path = path or SESSION
    if not path.exists():
        return False
    return json.loads(path.read_text(encoding="utf-8")).get("learning_may_be_active", True)


def recovery_pose_hint(arm):
    """Read raw feedback for hand-guiding only; never authorize power from it."""
    result = []
    for joint in range(1, 7):
        arm.bus.write_byte_data(0x15, 0x30 + joint, 0)
        time.sleep(0.03)
        word = arm.bus.read_word_data(0x15, 0x30 + joint)
        raw = ((word >> 8) & 255) | ((word << 8) & 65280)
        if not 0 < raw <= 4095:
            raise RuntimeError(f"关节{joint}原始反馈无效。")
        angle = 270 * (raw - 380) / 3320 if joint == 5 else 180 * (raw - 900) / 2200
        if joint in (2, 3, 4):
            angle = 180 - angle
        maximum = 270 if joint == 5 else 180
        if not -10 <= angle <= maximum + 10:
            raise RuntimeError(f"关节{joint}原始映射明显超范围。")
        result.append(round(angle, 1))
    return result


def hold_current(arm, session_path=None):
    """Enable direct torque once at a stable measured pose; await human confirmation."""
    session_path = session_path or SESSION
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not session.get("learning_may_be_active", True):
        raise RuntimeError("没有待恢复的松力记录。")
    if session.get("release_method") != "torque_switch":
        raise RuntimeError("仅适用于本程序直接扭矩开关产生的松力状态。")
    if session.get("hold_current_attempt"):
        raise RuntimeError("当前位置恢复已尝试过，禁止自动重发；先检查反馈和现场抱紧状态。")
    before = valid_pose(read_pose(arm))
    for _ in range(2):
        time.sleep(0.2)
        if not near_pose(read_pose(arm), before, 1):
            raise RuntimeError("当前姿态仍在变化，未发送恢复扭矩指令。")
    attempt = {"before_pose": before, "status": "command_result_unknown",
               "requested_utc": datetime.now(timezone.utc).isoformat(),
               "position_commands_sent": False, "operator_confirmation_pending": True}
    session["hold_current_attempt"] = attempt
    save_json(session_path, session)
    torque_command(arm, 1)
    time.sleep(0.5)
    for _ in range(3):
        after = valid_pose(read_pose(arm))
        if not near_pose(after, before, 2):
            attempt.update({"status": "feedback_drift", "after_pose": after})
            save_json(session_path, session)
            raise RuntimeError("恢复后角度变化超过2°，停止后续操作；请保持支撑。")
        time.sleep(0.2)
    attempt.update({"status": "feedback_verified", "after_pose": after})
    save_json(session_path, session)
    print(f"当前位置恢复扭矩指令已发送，反馈稳定：{after}。", flush=True)
    print("没有发送归位角度。保留支撑，待现场确认抱紧后再清除待恢复标记。", flush=True)
    return after


def confirm_hold_current(arm, session_path=None, operator_confirmed=False):
    if not operator_confirmed:
        raise RuntimeError("需要真实的现场抱紧确认。")
    session_path = session_path or SESSION
    state = json.loads(session_path.read_text(encoding="utf-8"))
    attempt = state.get("hold_current_attempt", {})
    if attempt.get("status") != "feedback_verified":
        raise RuntimeError("恢复后的反馈尚未通过检查，保留状态标记。")
    current = read_pose(arm)
    if not near_pose(current, attempt["after_pose"], 2):
        raise RuntimeError("反馈已明显变化，保留状态标记。")
    state["learning_may_be_active"] = False
    attempt["operator_confirmation_pending"] = False
    attempt["operator_confirmed_utc"] = datetime.now(timezone.utc).isoformat()
    save_json(session_path, state)
    return current


def restore_prompt(arm, session_path=None):
    """Never re-enable from an arbitrary moved pose or automatically on Ctrl+C."""
    session_path = session_path or SESSION
    session = json.loads(session_path.read_text(encoding="utf-8"))
    reference = valid_pose(session["reference_pose"])
    if not session.get("learning_may_be_active", True):
        return True
    print("\n关节仍处于松力状态，请一直托稳，不要松手。", flush=True)
    print(f"录制已停止。请手动回到松力前的起始角度：{reference}", flush=True)
    print("各关节接近起点（误差不超过2°）且已托稳后，输入 HOLD 并回车恢复扭矩。", flush=True)
    if session.get("release_method") == "torque_switch":
        print("推荐：支撑稳固、夹爪为空时，输入 HOLDHERE 在当前位置抱紧；不要求回到原起点。", flush=True)
        print("指令后继续保持支撑，现场确认已抱紧，再输入 HOLDING 完成。", flush=True)
    print("必须离开时先给机械臂可靠支撑，再输入 LEAVE；程序将保留松力状态标记。", flush=True)
    print("手动回位困难时，可输入 LEAVE 退出，再运行完整命令：", flush=True)
    print("python3 trajectory_teach.py hold-current", flush=True)
    failed_reads = 0
    while True:
        try:
            try:
                current = read_pose(arm)
                failed_reads = 0
                if session.get("hold_current_attempt", {}).get("status") == "feedback_verified":
                    print(f"\r[待现场确认抱紧] 当前 {[round(a) for a in current]}；已抱紧请输入HOLDING    ", end="", flush=True)
                else:
                    print(f"\r当前 {[round(a) for a in current]}；起点 {reference}    ", end="", flush=True)
            except RuntimeError as error:
                current = None
                try:
                    hint = recovery_pose_hint(arm)
                except (RuntimeError, OSError, AttributeError):
                    failed_reads += 1
                    print(f"\n读取失败：{error} 请继续支撑，检查连接后重试。", flush=True)
                    if failed_reads >= 3:
                        print("\n连续三次读取失败，停止循环。机械臂可能仍在松力，请保持可靠支撑。", flush=True)
                        print("不要回放或盲目恢复扭矩；先只读诊断。", flush=True)
                        return False
                else:
                    failed_reads = 0
                    print(f"\r[仅手动回位参考，不能据此上力] 原始映射 {hint}；起点 {reference}    ", end="", flush=True)
            if not select.select([sys.stdin], [], [], 1)[0]:
                continue
            command = sys.stdin.readline()
            if command == "" or command.strip() == "LEAVE":
                print("\n恢复流程尚未完成现场确认。保持可靠支撑；重连后运行 trajectory_teach.py restore。", flush=True)
                return False
            if command.strip() == "HOLDHERE":
                hold_current(arm, session_path)
                session = json.loads(session_path.read_text(encoding="utf-8"))
                print("\n现场确认已抱紧后，输入 HOLDING；不要重复HOLDHERE。", flush=True)
                continue
            if command.strip() == "HOLDING":
                confirm_hold_current(arm, session_path, operator_confirmed=True)
                print("\n已记录现场抱紧确认；可缓慢移开支撑。", flush=True)
                return True
            if command.strip() != "HOLD":
                continue
            if session.get("hold_current_attempt"):
                print("\n已有当前位置恢复记录，请先确认实际状态；已抱紧请输入HOLDING。", flush=True)
                continue
            latest = read_pose(arm)
            if not near_pose(latest, reference):
                print("\n还未回到起点，未发送恢复扭矩指令。", flush=True)
                continue
            close_release(arm, session)
            session["learning_may_be_active"] = False
            session["exit_command_sent_utc"] = datetime.now(timezone.utc).isoformat()
            save_json(session_path, session)
            print("\n恢复扭矩指令已发送，可能出现位置调整；现场确认机械臂已经抱紧后，再慢慢移开支撑。", flush=True)
            return True
        except KeyboardInterrupt:
            print("\n恢复流程已退出，未自动改变扭矩。请保持可靠支撑，之后运行 restore 或 recover。", flush=True)
            return False
        except (RuntimeError, OSError) as error:
            print(f"\n恢复未确认：{error}。继续托稳，可输入 HOLD 重试。", flush=True)


def recover(arm, session_path=None):
    """Explicit on-site recovery when feedback cannot satisfy the normal guard."""
    session_path = session_path or SESSION
    if not session_path.exists() or not session_active(session_path):
        print("没有本程序遗留的松力状态记录。")
        return False
    session = json.loads(session_path.read_text(encoding="utf-8"))
    reference = valid_pose(session["reference_pose"])
    method = session.get("release_method", "learning_mode")
    print(f"松力前参考角度：{reference}", flush=True)
    print("读数失败，无法核实当前位置；这不是回放，也不会发送目标角度。", flush=True)
    print("先托稳/可靠支撑机械臂，尽量接近原姿态，移开夹持物，手避开关节夹缝。", flush=True)
    print("接下来会恢复扭矩，可能出现抱紧或位置调整；只有你确认准备好才执行。", flush=True)
    if input("准备好后输入 RECOVER 并回车；其他输入取消：").strip() != "RECOVER":
        print("取消，未发送模式或扭矩指令。")
        return False
    session["recovery_requested_utc"] = datetime.now(timezone.utc).isoformat()
    save_json(session_path, session)
    close_release(arm, session)
    print("恢复指令已发送，继续保持支撑，正在重新查询反馈。", flush=True)
    time.sleep(0.5)
    try:
        current = read_pose(arm)
    except RuntimeError as error:
        session["recovery_feedback_error"] = str(error)
        save_json(session_path, session)
        print("一项或多项角度仍未通过SDK校验；可能是通信或角度映射超范围，需只读诊断。", flush=True)
        print("状态标记未清除。请保持可靠支撑，不要重复恢复指令或开始回放。", flush=True)
        return False
    print(f"当前反馈恢复：{[round(a) for a in current]}", flush=True)
    session["recovery_feedback_pose"] = current
    save_json(session_path, session)
    if input("现场确认关节已抱紧，输入 HOLDING 并回车；其他输入保留待确认状态：").strip() != "HOLDING":
        print("尚未确认抱紧，状态标记保留；不要开始回放。")
        return False
    session["learning_may_be_active"] = False
    session["recovery_operator_confirmed_utc"] = datetime.now(timezone.utc).isoformat()
    save_json(session_path, session)
    print("已记录反馈恢复和现场抱紧确认。请缓慢移开支撑。", flush=True)
    return True


def prepare_plan(document, max_samples=600, max_replay_seconds=180, max_sample_step=10, segment_retime=False):
    if document.get("schema") != "dofbot_joint_teaching_v1" or document.get("status") != "recorded":
        raise ValueError("只允许回放完整的关节示教文件。")
    rows = document.get("samples", [])
    if not isinstance(rows, list) or not 10 <= len(rows) <= max_samples:
        raise ValueError(f"轨迹需要10～{max_samples}个有效采样点。")
    times, poses = [], []
    maximum_rate = 0.0
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("轨迹采样点格式无效。")
        stamp = row.get("t")
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError("轨迹时间无效。")
        pose = valid_pose(row.get("q"))
        if times:
            gap = stamp - times[-1]
            if gap < 0.02 or gap > 0.5:
                raise ValueError("轨迹采样时间重复、倒退或间隔异常，请重新示教。")
            step = max(abs(a - b) for a, b in zip(pose, poses[-1]))
            if max_sample_step is not None and step > max_sample_step:
                raise ValueError("轨迹相邻点跳变超过10°，请更慢地重新示教。")
            maximum_rate = max(maximum_rate, step / gap)
        elif stamp != 0:
            raise ValueError("轨迹起点时间必须为0。")
        times.append(float(stamp))
        poses.append(pose)
    if times[-1] < 0.5:
        raise ValueError("示教轨迹太短。")
    if segment_retime:
        # Observation-only hand guidance can legitimately be faster than powered
        # replay. Preserve measured data; stretch each execution segment instead
        # of discarding it or slowing the entire episode for one gripper closure.
        execution = [0.0]
        for i in range(1,len(times)):
            step = max(abs(a-b) for a,b in zip(poses[i-1],poses[i]))
            execution.append(execution[-1]+max(2*(times[i]-times[i-1]),step/10.0))
        scale = execution[-1]/times[-1]
        times = execution
        duration = times[-1]
    else:
        scale = max(2.0, maximum_rate / 10.0)
        duration = times[-1] * scale
    # Start at half recorded speed; slow further if required to cap 10 deg/s.
    if duration > max_replay_seconds:
        raise ValueError(f"限速后的回放超过{max_replay_seconds}秒，请缩短轨迹并更慢示教。")
    plan = []
    for index in range(math.ceil(duration * HZ) + 1):
        source_t = min(index / HZ if segment_retime else index / HZ / scale, times[-1])
        right = min(max(1, bisect_right(times, source_t)), len(times) - 1)
        left = right - 1
        fraction = (source_t - times[left]) / (times[right] - times[left])
        pose = [round(a + (b - a) * fraction) for a, b in zip(poses[left], poses[right])]
        if plan and max(abs(a - b) for a, b in zip(pose, plan[-1])) > 2:
            raise ValueError("重采样后单步超过2°，拒绝回放。")
        plan.append(pose)
    return plan, scale


def wait_for_teaching_start(arm, reference):
    """Read only after release; START requires stable feedback near the old target."""
    print("准备阶段：只显示角度，尚未录制。轻柔确认可拖动，不要硬掰或靠近机械限位。", flush=True)
    print(f"先回到松力前姿态 {reference}（每关节误差不超过2°）。", flush=True)
    print("准备好输入 START 并回车开始；输入 END 取消并进入恢复流程。", flush=True)
    previous = None
    stable = 0
    failed = 0
    start_requested = False
    while True:
        try:
            current = read_pose(arm)
            failed = 0
            stable = stable + 1 if previous is not None and near_pose(current, previous, 1) else 1
            previous = current
            print(f"\r[准备，未录制] 当前 {[round(a) for a in current]}；稳定检查 {min(stable, 3)}/3    ", end="", flush=True)
        except RuntimeError as error:
            failed += 1
            stable = 0
            previous = None
            current = None
            print(f"\n准备读取失败：{error}", flush=True)
            if failed >= 3:
                raise RuntimeError("松力后连续三次读取失败，未开始录制。请保持支撑，按恢复提示操作。")
        if start_requested:
            if stable >= 3:
                if near_pose(current, reference):
                    print("\n开始记录关节轨迹。", flush=True)
                    return True
                print("\n尚未回到松力前姿态，未开始；回到起点后重新输入 START。", flush=True)
                start_requested = False
            else:
                time.sleep(0.2)
                continue
        if select.select([sys.stdin], [], [], 0.3)[0]:
            command = sys.stdin.readline()
            if command == "":
                raise KeyboardInterrupt
            if command.strip() == "START":
                start_requested = True
            elif command.strip() == "END":
                print("\n已取消录制，进入恢复扭矩流程。", flush=True)
                return False


def teach(arm, args):
    if session_active():
        raise RuntimeError("上次学习模式尚未安全结束。先运行 trajectory_teach.py restore。")
    reference = [round(a) for a in read_pose(arm)]
    boundary_joints = []
    for joint, angle in enumerate(reference[:5], 1):
        maximum = 270 if joint == 5 else 180
        if angle < 5 or angle > maximum - 5:
            boundary_joints.append(joint)
    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    path = TRAJECTORIES / name / "trajectory.json"
    document = {
        "schema": "dofbot_joint_teaching_v1", "status": "preparing", "task": args.task,
        "camera_recorded": False, "contains_motor_actions": False, "teaching_method": "direct_torque_switch",
        "joint_order": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"],
        "angle_unit": "degree", "reference_pose": reference, "samples": [],
        "near_sdk_boundary_joints": boundary_joints,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    save_json(path, document)
    print(f"起始角度：{reference}\n只录关节，时长 {args.seconds} 秒，不启动相机。", flush=True)
    print("先取出夹持物，托稳机械臂（包括肩肘和末端），松力后各关节和夹爪都不再主动保持。", flush=True)
    print("首次功能测试只移动一个关节5～10°。完整任务需持续支撑、缓慢示教，并在计时结束前完成夹爪开合。", flush=True)
    print("录完可直接输入HOLDHERE在当前位置抱紧，再输入HOLDING确认；不自动回放。", flush=True)
    if boundary_joints:
        print(f"注意：{boundary_joints}号接近SDK边界。不要继续往边界外拖动；读数异常即停止采集。", flush=True)
    if input("已托稳，输入 RELEASE 并回车关闭扭矩进入准备；其他输入取消：").strip() != "RELEASE":
        document["status"] = "cancelled"
        save_json(path, document)
        return
    if not near_pose(read_pose(arm), reference):
        raise RuntimeError("确认期间起始位置改变，未松力。")
    save_json(SESSION, {"learning_may_be_active": True, "release_method": "torque_switch", "reference_pose": reference,
                        "trajectory": str(path), "created_utc": datetime.now(timezone.utc).isoformat()})
    interrupted = False
    restored = False
    try:
        torque_command(arm, 0)
        time.sleep(0.2)
        print("已发送关闭扭矩指令（不进入板载学习模式）。确认能够轻柔移动再示教，不要硬掰。", flush=True)
        if not wait_for_teaching_start(arm, reference):
            document["status"] = "cancelled"
            return
        start = time.monotonic()
        first_ns = None
        document["status"] = "recording"
        for index in range(args.seconds * HZ):
            time.sleep(max(0, start + index / HZ - time.monotonic()))
            read_start = time.monotonic_ns()
            pose = read_pose(arm)
            read_end = time.monotonic_ns()
            if first_ns is None:
                first_ns = read_start
                if not near_pose(pose, reference, 3):
                    raise RuntimeError("刚松力的位置已偏离起点，请托稳后重新示教。")
            document["samples"].append({"t": (read_start - first_ns) / 1e9, "q": pose,
                                         "read_duration_ms": (read_end - read_start) / 1e6})
            if (index + 1) % HZ == 0:
                save_json(path, document)
                print(f"已录 {(index + 1) // HZ}/{args.seconds} 秒：{[round(a) for a in pose]}", flush=True)
        document["status"] = "recorded"
    except KeyboardInterrupt:
        interrupted = True
        document["status"] = "interrupted"
        print("\n关节录制中断，停止追加轨迹。", flush=True)
    except Exception as error:
        document["status"] = "error"
        document["error"] = str(error)
        print(f"\n录制失败：{error}", flush=True)
    finally:
        # Preserve the measured path before asking the operator to return to start.
        try:
            save_json(path, document)
        finally:
            if interrupted:
                print("录制已退出，机械臂可能仍松力。保持可靠支撑；之后运行 restore 或 recover。", flush=True)
            else:
                restored = restore_prompt(arm)
    if document["status"] == "recorded":
        save_json(TRAJECTORIES / "latest.json", {"trajectory": str(path)})
        print(f"关节轨迹已保存：{path}", flush=True)
        if not restored or session_active():
            print("恢复扭矩尚未完成，禁止回放；保持支撑，先运行：", flush=True)
            print("python3 trajectory_teach.py restore", flush=True)
            print("手动回位困难时，可改用 trajectory_teach.py hold-current（当前位置抱紧；回放前另需回到轨迹起点）。", flush=True)
            return
        try:
            plan, scale = prepare_plan(document)
            print(f"校验通过，限速回放约 {(len(plan)-1)/HZ:.1f} 秒（时长倍率 {scale:.2f}）。", flush=True)
        except ValueError as error:
            print(f"这条轨迹暂不能回放：{error}", flush=True)
        print("下一步运行：python3 trajectory_teach.py replay", flush=True)


def replay(arm, args):
    if session_active():
        raise RuntimeError("学习模式可能仍在松力，先运行 trajectory_teach.py restore。")
    if args.trajectory:
        path = args.trajectory
    else:
        pointer = json.loads((TRAJECTORIES / "latest.json").read_text(encoding="utf-8"))
        path = Path(pointer["trajectory"])
    source_bytes = path.read_bytes()
    document = json.loads(source_bytes)
    plan, scale = prepare_plan(document)
    if not near_pose(read_pose(arm), plan[0]):
        raise RuntimeError(f"当前姿态不在轨迹起点附近，未发送运动指令。轨迹起点：{plan[0]}；请先安全回到起点。")
    import cv2
    camera = LatestCamera(cv2, args.camera)
    server = None
    episode = None
    try:
        camera.start()
        camera.get()
        server = start_preview(camera, cv2, args.preview_port)
        print(f"预览：http://127.0.0.1:{args.preview_port}/\n轨迹：{path}", flush=True)
        print(f"将按限速轨迹自动运动约 {(len(plan)-1)/HZ:.1f} 秒，同时录RGB、反馈角度和真实指令。", flush=True)
        print("先移开手、清空路径。首次空夹爪测试；抓取测试需将方块和盒子重新摆好。", flush=True)
        if input("输入 REPLAY 并回车；其他输入取消：").strip() != "REPLAY":
            return
        for seconds in range(5, 0, -1):
            print(f"{seconds} 秒后自动开始……", flush=True)
            time.sleep(1)
        if not near_pose(read_pose(arm), plan[0]):
            raise RuntimeError("倒计时期间位置改变，取消回放。")
        if shutil.disk_usage(args.root).free < 500 * 1024 * 1024:
            raise RuntimeError("磁盘剩余不足500MB，取消回放。")
        args.root.mkdir(parents=True, exist_ok=True)
        episode = Episode(args.root, document.get("task", "关节轨迹测试"), args.camera, cv2)
        episode.metadata.update({
            "episode_type": "powered_replay_of_kinesthetic_teaching", "source_trajectory": str(path),
            "source_sha256": hashlib.sha256(source_bytes).hexdigest(), "replay_time_scale": scale,
            "control_duration_ms": REPLAY_DURATION_MS, "capture_checks_passed": False,
            "recorded_modalities": ["rgb", "joint_feedback", "commanded_joint_targets"],
        })
        episode._write_meta()
        previous_sequence = -1
        previous_target = plan[0]
        started = time.monotonic()
        for index, target in enumerate(plan):
            time.sleep(max(0, started + index / HZ - time.monotonic()))
            tick = time.monotonic()
            if tick - (started + index / HZ) > 0.2:
                raise RuntimeError("回放采集延迟超过200ms，停止后续指令。")
            if select.select([sys.stdin], [], [], 0)[0]:
                command = sys.stdin.readline()
                if command == "" or command.strip().lower() == "q":
                    raise KeyboardInterrupt
            sequence, frame_ns, frame = camera.get(previous_sequence)
            previous_sequence = sequence
            read_start = time.monotonic_ns()
            actual = read_pose(arm)
            read_end = time.monotonic_ns()
            check_tracking(actual, previous_target)
            if max(abs(a - b) for a, b in zip(actual, target)) > 6:
                raise RuntimeError("反馈落后于下一目标超过6°，停止回放。")
            write_start = time.monotonic_ns()
            write_pose(arm, target, REPLAY_DURATION_MS)
            write_end = time.monotonic_ns()
            episode.add(frame, sequence, frame_ns, actual, read_start, read_end,
                        target, write_start, write_end, tick, action_duration_ms=REPLAY_DURATION_MS)
            previous_target = target
            if index % HZ == 0:
                print(f"回放 {index/HZ:.0f}/{(len(plan)-1)/HZ:.1f} 秒，反馈 {[round(a) for a in actual]}", flush=True)
        time.sleep(0.35)
        final_pose = read_pose(arm)
        if not near_pose(final_pose, plan[-1], 3):
            raise RuntimeError("回放结束后反馈未到末点附近，保存为失败。")
        episode.metadata.update({"capture_checks_passed": True, "final_observation_state": final_pose})
        episode.finish("unlabeled")
        completed = episode
        episode = None
        try:
            outcome = input("现场任务确实完成且无碰撞，输入 SUCCESS 标记成功；失败输入 FAIL；其他输入保留待检查：").strip()
        except (EOFError, KeyboardInterrupt):
            outcome = ""
        if outcome in ("SUCCESS", "FAIL"):
            completed.metadata["status"] = "success" if outcome == "SUCCESS" else "failed"
            completed.metadata["operator_success"] = outcome == "SUCCESS"
            completed._write_meta()
        print(f"保存结果：{completed.metadata['status']}\n{completed.path}", flush=True)
        print("这是一条待检查、待转换格式的原始数据。回放结束保持末点，不自动返回起点。", flush=True)
    except KeyboardInterrupt:
        if episode:
            episode.finish("interrupted", "operator stopped replay")
            episode = None
        print("\n停止后续指令；已经发出的短时动作可能继续。", flush=True)
    except Exception as error:
        if episode:
            episode.finish("error", str(error))
            episode = None
        raise
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if camera.thread.ident is not None:
            camera.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    teach_parser = commands.add_parser("teach", help="支撑机械臂后直接松力，输入START只录关节")
    teach_parser.add_argument("--seconds", type=int, default=15, choices=range(2, 61))
    teach_parser.add_argument("--task", default="关节小幅往返测试")
    commands.add_parser("restore", help="恢复上次未结束的学习模式，必须先手动回到起点")
    commands.add_parser("recover", help="角度读取失效时的现场确认恢复；可能恢复扭矩，不发送角度")
    commands.add_parser("hold-current", help="支撑稳固且读数有效时，在当前位置恢复扭矩，不归位")
    replay_parser = commands.add_parser("replay", help="限速回放最近轨迹并录RGB、角度和真实指令")
    replay_parser.add_argument("--trajectory", type=Path)
    replay_parser.add_argument("--camera", default="/dev/video0")
    replay_parser.add_argument("--preview-port", type=int, default=8765)
    replay_parser.add_argument("--root", type=Path, default=HOME / "datasets/pick_place_replay_raw")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("需要交互终端；从电脑直接运行请使用 ssh -t。")
    if args.command == "replay":
        if not 1024 <= args.preview_port <= 65535:
            parser.error("预览端口必须在1024～65535。")
        args.root.mkdir(parents=True, exist_ok=True)
    import fcntl
    from Arm_Lib import Arm_Device
    with Path(__file__).with_name(".teleop_record.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("另一个采集程序仍在运行，请先在其终端按q退出。")
        arm = Arm_Device(i2c_bus=7)
        try:
            if args.command == "teach":
                teach(arm, args)
            elif args.command == "restore":
                if not SESSION.exists() or not session_active():
                    print("没有本程序遗留的松力状态记录。")
                else:
                    restore_prompt(arm)
            elif args.command == "recover":
                recover(arm)
            elif args.command == "hold-current":
                print("保持可靠支撑、夹爪为空、手避开夹缝；即将在当前位置恢复扭矩。", flush=True)
                if input("输入 HOLDHERE 执行；其他输入取消：").strip() == "HOLDHERE":
                    after = hold_current(arm)
                    if input("现场确认已抱紧，输入 HOLDING；其他输入保留待确认标记：").strip() == "HOLDING":
                        confirm_hold_current(arm, operator_confirmed=True)
                        print("已记录现场抱紧确认。回放仍要求单独确认轨迹起点。", flush=True)
            else:
                replay(arm, args)
        finally:
            arm.bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已退出。已发送动作可能继续；若仍在学习模式，请托稳并运行 restore。")
        raise SystemExit(130)
    except (RuntimeError, ValueError, OSError, KeyError, EOFError) as error:
        print(f"错误：{error}")
        raise SystemExit(1)
