"""Single-owner control engine. No GUI dependencies; all motor IO stays in one worker."""
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import threading
import time

from trajectory_teach import valid_pose, near_pose, prepare_plan, save_json
from .hardware import FeedbackError
from .storage import Store, Capture, load, identifier
from .safety import EmergencyLatch
from .tasks import DEFAULT_TASKS, task_name, task_choices
from .tracking import command_bounds, failed_joints, failure_text
from .camera import CameraFrameError


def home_plan(start, target, duration=None, max_speed=30.0, max_acceleration=90.0):
    """Time-scale the existing quintic to the span, never to a five-second cap.

    ``duration`` optionally requests a minimum duration; limits may lengthen it.
    The one-second floor avoids abrupt short moves. Limits are software policy,
    not motor ratings or collision avoidance. Execution remains at 10 Hz.
    """
    start, target = valid_pose(start), valid_pose(target)
    for name, value in (("速度", max_speed), ("加速度", max_acceleration)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"回位{name}限制必须为有限正数，未执行。")
    if duration is not None and (isinstance(duration, bool) or not isinstance(duration, (int, float)) or
                                 not math.isfinite(duration) or duration <= 0):
        raise ValueError("回位请求时间必须为有限正数，未执行。")
    span = max(abs(a-b) for a,b in zip(start,target))
    required = max(1.875 * span / max_speed, math.sqrt(5.774 * span / max_acceleration))
    count = math.ceil(max(1.0, duration or 0.0, required) * 10)
    result = []
    for i in range(count + 1):
        u = i / count
        s = 10*u**3 - 15*u**4 + 6*u**5
        result.append([round(a+(b-a)*s) for a,b in zip(start,target)])
    if any(max(abs(a-b) for a,b in zip(x,y)) > 4 for x,y in zip(result,result[1:])):
        raise ValueError("回位计划单步变化过大，未执行。")
    return result


class Stopped(RuntimeError):
    pass


def compensated_teaching(document, extra_degrees=0):
    """Stretch J6 closure relative to demonstrated widest opening; never edit source.

    For this manufacturer's gripper, increasing the angle closes the fingers.
    This is a position correction, not a measurement or guarantee of grip force.
    """
    if type(extra_degrees) is not int or extra_degrees not in range(6):
        raise ValueError("夹爪夹紧补偿只能选0°到5°的整数。")
    if not extra_degrees:
        return document, {"strategy":"disabled","max_extra_degrees":0}
    rows=document.get("samples",[])
    if not isinstance(rows,list) or not rows or any(not isinstance(r,dict) for r in rows):
        raise ValueError("示教采样点无效，不能计算夹爪补偿。")
    angles=[valid_pose(r.get("q"))[5] for r in rows]
    opened,closed=min(angles),max(angles)
    if closed-opened < 5:
        raise ValueError("本条夹爪开合幅度不足5°，无法可靠区分张开与夹紧；请选择0°补偿，或重新示教开合动作。")
    if closed+extra_degrees>180:
        raise ValueError("夹爪补偿后超过180°，未执行；请选择更小补偿。")
    result=copy.deepcopy(document)
    for row,angle in zip(result["samples"],angles):
        row["q"]=list(row["q"])
        row["q"][5]=angle+extra_degrees*(angle-opened)/(closed-opened)
    policy={"strategy":"linear_closure_span","joint":6,"unit":"degree",
            "increasing_angle_closes":True,"max_extra_degrees":extra_degrees,
            "source_open_angle":opened,"source_closed_angle":closed,
            "target_open_angle":opened,"target_closed_angle":closed+extra_degrees,
            "formula":"target = source + max_extra_degrees * (source - source_open_angle) / (source_closed_angle - source_open_angle)",
            "integer_target_rounding":"nearest, after interpolation and segment retiming",
            "force_control":False}
    return result,policy


def joint3_corrected_teaching(document, offset_degrees=0):
    """Optional constant position correction, not torque/gravity compensation."""
    if type(offset_degrees) is not int or not -3 <= offset_degrees <= 3:
        raise ValueError("J3位置微调只能选-3°到+3°的整数。")
    policy = {"joint":3, "offset_degrees":offset_degrees, "unit":"degree",
              "strategy":"constant_position_offset" if offset_degrees else "disabled",
              "gravity_compensation":False,"force_control":False,
              "scope":"replay_and_its_first_pose_only; saved_global_home_unchanged"}
    if not offset_degrees:
        return document, policy
    result = copy.deepcopy(document)
    for row in result["samples"]:
        pose = valid_pose(row["q"])
        if not 0 <= pose[2]+offset_degrees <= 180:
            raise ValueError("J3位置微调后超出0°～180°，未执行；请减小微调，不会自动裁切。")
        row["q"] = pose[:]
        row["q"][2] += offset_degrees
    return result, policy


def teaching_plan(document, gripper_extra_degrees=0, joint3_offset_degrees=0):
    if document.get("quality_warnings") and not document.get("warnings_reviewed"):
        raise ValueError("本条包含快速变化，需先审核并保存人工示教。")
    # Validate the untouched trajectory first; a correction cannot mask bad data.
    plan=prepare_plan(document,max_samples=1201,max_replay_seconds=600,
                        max_sample_step=None,segment_retime=True)
    positioned,_=joint3_corrected_teaching(document,joint3_offset_degrees)
    adjusted,_=compensated_teaching(positioned,gripper_extra_degrees)
    return plan if adjusted is document else prepare_plan(adjusted,max_samples=1201,max_replay_seconds=600,
                                                         max_sample_step=None,segment_retime=True)


class Engine:
    def __init__(self, robot, root, session_path, camera=None, cv2=None, demo=False, safety=None):
        self.robot = robot
        self.store = Store(root)
        self.safety=safety or EmergencyLatch(self.store.root/'safety.json')
        self.robot.safety_guard=self.safety.require_clear
        self.session_path = Path(session_path)
        self.camera = camera
        self.cv2 = cv2
        self.demo = demo
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.stage = "ready"
        self.activity = "idle"
        self.holding = "unknown"
        self.pose = [None]*6
        self.hints = [None]*6
        self.valid = False
        self.read_at = 0
        self.feedback_error = ""
        self.feedback_command_failed = False
        self.feedback_reasons = [None]*6
        self.feedback_failures = 0
        self.feedback_since = None
        self.last_feedback_event = 0
        self.message = "先查看关节，设置初始状态。启动不会自动运动。"
        self.error = ""
        self.task = DEFAULT_TASKS[0]
        self.task_history = []
        task_config = self.store.root / "task_config.json"
        if task_config.exists():
            try:
                config = load(task_config)
                self.task = task_name(config["current"])
                recent = config.get("recent", [])
                if not isinstance(recent, list):
                    raise ValueError("任务历史格式无效。")
                self.task_history = task_choices(self.task, recent)
            except (OSError, ValueError, TypeError, KeyError):
                self.task = DEFAULT_TASKS[0]
                self.task_history = []
        self.home = None
        self.home_setup_previous = None
        self.home_candidate = None
        self.teaching = None
        self.capture = None
        self.teach_started = None
        self.first_read_ns = None
        self.elapsed = 0.0
        self.progress = 0.0
        self.preview = None
        self.preview_frames = {}
        self.preview_errors = {}
        self.camera_error = ""
        self.last_preview = 0
        config = self.store.root / "home.json"
        if config.exists():
            self.home = valid_pose(load(config)["pose"])
        if self.session_path.exists() and load(self.session_path).get("learning_may_be_active", True):
            attempt=load(self.session_path).get("hold_current_attempt")
            if isinstance(attempt,dict) and attempt.get("status") in ("command_result_unknown","feedback_verified","torque_command_sent"):
                self.holding="pending"
                self.message="上次已发送抱紧指令但未完成现场确认。保持支撑，核实反馈和现场状态，不重复上力。"
            else:
                self.holding = "free"
                self.message = "检测到上次未完成的松力记录。保持支撑，优先恢复当前位置抱紧。"
        # Interrupted captures remain inspectable; they never become accepted automatically.
        for meta in self.store.list_items("episodes"):
            if meta.get("status") in ("recording", "encoding"):
                meta.update(status="interrupted", reason="Application ended before capture completion")
                save_json(self.store.item("episodes", meta["id"]) / "metadata.json", meta)
        work = self.store.root / "work.json"
        if work.exists():
            try:
                last = load(work)
                if last.get("teaching_id"):
                    self.teaching = load(self.store.item("teachings", last["teaching_id"]) / "trajectory.json")
                    if self.teaching["status"] == "recording":
                        self.teaching.update(status="interrupted", error="Software ended during manual teaching")
                        self.store.write_teaching(self.teaching)
                    self.task = self.teaching["task"]
                    self.stage = "saved_teach" if self.teaching["status"] == "recorded" else "teach_review"
                if last.get("capture_id") and self.teaching:
                    self.capture = Capture.reopen(self.store,last["capture_id"],self.cv2)
                    self.stage = "capture_review"
            except (OSError,ValueError,KeyError):
                self.message = "上次工作草稿无法完整恢复；原文件仍保留。请检查数据目录。"

    def persist(self):
        save_json(self.store.root / "work.json", {
            "teaching_id": self.teaching["id"] if self.teaching else None,
            "capture_id": self.capture.id if self.capture else None})

    def snapshot(self):
        with self.lock:
            return {"stage": self.stage, "activity": self.activity, "holding": self.holding,"emergency":self.safety.status(),
                    "pose": self.pose[:], "hints": self.hints[:], "valid": self.valid,
                    "age_s": time.monotonic()-self.read_at if self.read_at else 999,
                    "feedback_error": self.feedback_error, "feedback_reasons": self.feedback_reasons[:],
                    "feedback_failures": self.feedback_failures,
                    "boundary_joints": [i+1 for i,x in enumerate(self.pose) if x is not None and
                                        min(x,(270 if i==4 else 180)-x)<10],
                    "home": copy.deepcopy(self.home), "message": self.message, "error": self.error,
                    "task": self.task, "task_history": task_choices(self.task, self.task_history),
                    "elapsed": self.elapsed, "progress": self.progress,
                    "samples": len(self.teaching["samples"]) if self.teaching else 0,
                    "teaching_id": self.teaching["id"] if self.teaching else None,
                    "teaching_saved": bool(self.teaching and self.teaching["status"] == "recorded"),
                    "teaching_status": self.teaching["status"] if self.teaching else None,
                    "teaching_start": self.teaching["samples"][0]["q"] if self.teaching and self.teaching["samples"] else None,
                    "quality_warnings": copy.deepcopy(self.teaching.get("quality_warnings",[])) if self.teaching else [],
                    "capture_id": self.capture.id if self.capture else None,
                    "capture_frames": self.capture.meta["frames"] if self.capture else 0,
                    "capture_status": self.capture.meta["status"] if self.capture else None,
                    "capture_success": self.capture.meta.get("operator_success") if self.capture else None,
                    "preview": self.preview, "camera_error": self.camera_error,
                    "preview_frames":dict(self.preview_frames),"preview_errors":dict(self.preview_errors),
                    "camera_status":self.camera.status() if self.camera and hasattr(self.camera,"status") else None,
                    "library": self.store.list_items("teachings"),
                    "saved_episodes": len([x for x in self.store.list_items("episodes") if x["status"] == "saved"]),
                    "root": str(self.store.root), "demo": self.demo}

    def refresh(self, bounds=None):
        try:
            p = valid_pose(self.robot.read_with_bounds(bounds) if bounds is not None and
                           hasattr(self.robot, "read_with_bounds") else self.robot.read())
        except FeedbackError as error:
            with self.lock:
                self.pose = error.pose
                self.hints = error.hints
                self.valid = False
                self.read_at = time.monotonic()
                self.feedback_error = str(error)
                self.feedback_reasons = error.reasons
                self.feedback_failures += 1
                if self.feedback_since is None:
                    self.feedback_since = self.read_at
                if self.read_at-self.last_feedback_event >= 5 or self.feedback_failures == 1:
                    self.event("feedback_invalid", detail=str(error), reasons=error.reasons,
                               sdk_io=getattr(self.robot,"last_feedback_io",[]))
                    self.last_feedback_event = self.read_at
            raise
        except Exception:
            with self.lock:
                self.valid = False
            raise
        with self.lock:
            if self.feedback_failures:
                self.event("feedback_recovered", missed_reads=self.feedback_failures)
            if self.feedback_command_failed and self.stage != "teach_review":
                self.error = ""
                self.message = ("反馈已恢复；抱紧指令已发，继续支撑并完成现场确认。" if self.holding == "pending" else
                                "反馈已恢复。上次操作未完成，请重新选择操作；软件没有自动续录或切换扭矩。")
                self.feedback_command_failed = False
            self.pose = p
            self.hints = [None]*6
            self.valid = True
            self.read_at = time.monotonic()
            self.feedback_error = ""
            self.feedback_reasons = [None]*6
            self.feedback_failures = 0
            self.feedback_since = None
            waited=[r for r in getattr(self.robot,"last_feedback_io",[]) if r.get("requested_wait_ms",3)>3]
            if waited:
                self.event("feedback_reply_retry", queries=waited)
        return p

    def event(self, name, **details):
        """Best-effort diagnostics; a log failure must not hide the actuator state."""
        row = {"utc":datetime.now(timezone.utc).isoformat(),"event":name,
               "stage":self.stage,"holding":self.holding,"pose":self.pose[:],
               "hints":self.hints[:],"teaching_id":self.teaching.get("id") if self.teaching else None,
               **details}
        try:
            path = self.store.root/"reports"/"events.jsonl"
            if path.exists() and path.stat().st_size>5_000_000:
                path.replace(path.with_suffix(".previous.jsonl"))
            with path.open("a",encoding="utf-8") as out:
                out.write(json.dumps(row,ensure_ascii=False)+"\n")
        except OSError:
            pass

    def stable_pose(self):
        pose = self.refresh()
        for _ in range(2):
            self.check_stop()
            time.sleep(.1)
            if not near_pose(self.refresh(), pose, 1):
                raise RuntimeError("当前姿态不稳定，未执行操作；先支撑稳固。")
        return pose

    def check_stop(self):
        if self.safety.tripped.is_set():raise Stopped('软件急停已锁定，停止后续指令；扭矩未自动改变。')
        if self.stop.is_set():
            raise Stopped("已停止后续指令，保持最后目标；停止按钮不会自动松力。")

    def _idle(self, allow_home_setup=False):
        if self.activity != "idle" or self.stage == "teaching":
            raise RuntimeError("当前操作尚未结束。")
        if self.stage == "home_setup" and not allow_home_setup:
            raise RuntimeError("先完成或取消初始位置设置。")

    def set_task(self, task):
        self._idle()
        if self.stage not in ("ready", "prepare") or self.teaching is not None or self.capture is not None:
            raise RuntimeError("已有示教或采集数据时不能改任务；请先保存/删除当前数据，再选择下一条。")
        value = task_name(task)
        history = task_choices(value, self.task_history)
        # Persist first: a disk error must not display a name that wasn't saved.
        save_json(self.store.root / "task_config.json", {"current": value, "recent": history})
        self.task = value
        self.task_history = history
        self.message = "任务名称已保存：" + value
        self.event("task_changed", task=value)

    @staticmethod
    def require(confirmed):
        if confirmed is not True:
            raise RuntimeError("需要触屏上的现场状态确认。")

    def _session(self, **changes):
        data = load(self.session_path) if self.session_path.exists() else {}
        data.update(changes)
        save_json(self.session_path, data)

    def _power_required(self):
        self.safety.require_clear()
        if self.holding != "confirmed":
            raise RuntimeError("先恢复当前位置抱紧，并在现场确认已抱紧。")
        if self.session_path.exists() and load(self.session_path).get("learning_may_be_active", True):
            raise RuntimeError("仍有待确认的松力状态，禁止自动运动。")

    def set_home(self, confirmed=False):
        self.require(confirmed)
        self._idle()
        p = [round(x) for x in self.stable_pose()]
        self._save_home(p)
        self.message = "已将当前姿态设为初始状态；没有发送运动指令。"

    def _save_home(self, p):
        save_json(self.store.root / "home.json", {"pose": p, "set_utc": datetime.now(timezone.utc).isoformat(),
                                                   "joint_order": list(range(1,7)), "source": "measured_pose"})
        self.home = p[:]

    def begin_home_setup(self, confirmed=False):
        self.require(confirmed)
        self._idle()
        previous = self.stage
        self._release_here()
        self.home_setup_previous = previous
        self.home_candidate = None
        self.stage = "home_setup"
        self.message = "初始位置设置：保持支撑，手动摆好六个关节，再点保存初始并抱紧。不记录示教或视频。"

    def finish_home_setup(self, confirmed=False):
        self.require(confirmed)
        self._idle(allow_home_setup=True)
        if self.stage != "home_setup" or self.holding != "free":
            raise RuntimeError("先进入手动设置初始位置；抱紧已发出时请完成现场确认。")
        self.hold_here(True)
        self.home_candidate = [round(x) for x in self.pose]
        self.message = "当前位置抱紧指令已发送。保持支撑，现场确认已抱紧后，新初始位置才会保存。"

    def _leave_home_setup(self):
        previous = self.home_setup_previous or "ready"
        self.stage = "ready" if previous == "prepare" and self.holding != "free" else previous
        self.home_setup_previous = None
        self.home_candidate = None

    def cancel_home_setup(self):
        self._idle(allow_home_setup=True)
        if self.stage != "home_setup":
            raise RuntimeError("当前不在设置初始位置。")
        self._leave_home_setup()
        self.message = "已取消设置，原初始位置保留。扭矩状态未改变；仍松力时持续支撑，并点当前位置抱紧。"

    def _release_here(self):
        if self.holding == "pending":
            raise RuntimeError("先完成现场抱紧确认。")
        session = load(self.session_path) if self.session_path.exists() else {}
        if self.holding == "free" and session.get("release_method") != "torque_switch":
            raise RuntimeError("遗留板载学习模式需先使用原恢复流程，不能混用扭矩接口。")
        p = self.stable_pose()
        if self.holding != "free":
            self._session(learning_may_be_active=True, release_method="torque_switch",
                          reference_pose=p, ui_owner="touch_collector", created_utc=datetime.now(timezone.utc).isoformat(),
                          hold_current_attempt=None)
            self.holding = "free"
            self.robot.torque(0)

    def release(self, confirmed=False):
        self.require(confirmed)
        self._idle()
        if self.home is None:
            raise RuntimeError("先设置初始状态。")
        self._release_here()
        self.stage = "prepare"
        self.elapsed = self.progress = 0
        self.message = "已发送松力指令。保持支撑；摆好起点，再点开始记录。相机仅预览，不保存。"

    def check_feedback(self):
        self.refresh()
        self.event("operator_feedback_check")
        if self.stage in ("ready","prepare") and self.holding != "pending":
            self.message = "六路反馈已恢复有效。仍松力时持续支撑；摆好本条起点后可记录，或先在当前位置抱紧。"

    def recover_manual(self, confirmed=False):
        """Explicit supported re-release, including when SDK angles are invalid.

        Torque-off needs no position target. Never infer a valid angle from raw
        hints, clear an error draft, or silently restart teaching. This is an
        operator-controlled escape from pending-hold + invalid-feedback deadlock.
        """
        self.require(confirmed)
        self._idle(allow_home_setup=True)
        session = load(self.session_path) if self.session_path.exists() else {}
        if session.get("release_method") != "torque_switch":
            raise RuntimeError("没有本软件的扭矩松力记录，不能使用此恢复入口；先诊断控制模式。")
        if self.holding == "free":
            raise RuntimeError("已经发过松力指令，持续支撑并手动调整；不要反复切换扭矩。")
        self.check_stop()
        # Durable uncertainty must precede torque-off; disk failure prevents IO.
        self._session(learning_may_be_active=True, release_method="torque_switch",
                      hold_current_attempt=None,
                      manual_recovery={"status":"command_result_unknown",
                                       "operator_support_confirmed":True,
                                       "requested_utc":datetime.now(timezone.utc).isoformat(),
                                       "previous_holding":self.holding})
        self.holding = "free"
        if self.stage == "home_setup":
            self._leave_home_setup()
        if self.stage in ("ready","prepare"):
            self.stage = "prepare"
        self.event("supported_manual_recovery_requested")
        self.robot.torque(0)
        self._session(manual_recovery={"status":"torque_off_sent",
                                       "operator_support_confirmed":True})
        self.message = "已重新发送松力指令，请持续支撑。轻柔调整异常关节至有效范围，不能硬掰；读数有效后可当前位置抱紧。异常草稿需删除重录，不自动续录或回位。"
        self.event("supported_manual_recovery_sent")

    def confirm_holding(self, confirmed=False):
        self.require(confirmed)
        self._idle(allow_home_setup=True)
        pose = self.stable_pose()
        if self.holding == "free":
            raise RuntimeError("本软件仍记录为松力，请先点当前位置抱紧，不能仅凭角度稳定确认。")
        completing_home = self.stage == "home_setup"
        if completing_home:
            if self.home_candidate is None or self.holding != "pending":
                raise RuntimeError("先点保存初始并抱紧，完成反馈检查后再确认。")
            if not near_pose(pose, self.home_candidate, 2):
                raise RuntimeError("当前姿态偏离待保存位置，旧初始位置未修改；保持支撑，取消设置后检查。")
            self._save_home(self.home_candidate)
        self.holding = "confirmed"
        stamp = datetime.now(timezone.utc).isoformat()
        session = load(self.session_path) if self.session_path.exists() else {}
        attempt = session.get("hold_current_attempt")
        if isinstance(attempt,dict):
            attempt.update(operator_confirmation_pending=False,operator_confirmed_utc=stamp,
                           feedback_verified=True,after_pose=pose[:])
        self._session(learning_may_be_active=False, operator_confirmed_utc=stamp,hold_current_attempt=attempt)
        self.message = "已记录现场抱紧确认；可以缓慢移开支撑。"
        self.event("holding_operator_confirmed")
        if completing_home:
            self._leave_home_setup()
            self.message = "新初始位置已保存，现场抱紧已确认。可进入手动示教；没有发送回位或关节位置指令。"
        elif self.stage == "prepare":
            self.stage = "ready"

    def hold_here(self, confirmed=False):
        self.require(confirmed)
        self._idle(allow_home_setup=True)
        if self.holding == "pending":
            raise RuntimeError("指令已发出，请现场确认抱紧，不重复上力。")
        session = load(self.session_path) if self.session_path.exists() else {}
        if session.get("learning_may_be_active") and session.get("release_method") != "torque_switch":
            raise RuntimeError("遗留板载学习模式需使用原恢复流程，不能混用扭矩接口。")
        before = self.stable_pose()
        # Persist uncertainty BEFORE the actuator command, including on an SDK error.
        self.holding = "pending"
        self._session(learning_may_be_active=True, release_method="torque_switch",
                      hold_current_attempt={"before_pose": before, "status": "command_result_unknown"})
        self.robot.torque(1)
        time.sleep(.5)
        for _ in range(3):
            if not near_pose(self.refresh(), before, 2):
                raise RuntimeError("上力后姿态变化超过2°；保持支撑，停止后续操作。")
            time.sleep(.1)
        self._session(hold_current_attempt={"before_pose": before, "after_pose": self.pose,
                                           "status": "feedback_verified", "operator_confirmation_pending": True})
        self.message = "抱紧指令已发送、反馈稳定。继续保留支撑，确认实际抱紧后点确认已抱紧。"

    def emergency_hold_here(self, confirmed=False):
        self.safety.require_clear()
        """Explicit operator recovery: torque only, even with invalid host feedback.

        Manufacturer torque-on holds the current position. This does NOT authorize
        position commands, claim valid angles, or clear the unresolved marker.
        """
        self.require(confirmed)
        if self.activity != "idle":
            raise RuntimeError("先停止当前自动动作。")
        if self.holding == "pending":
            raise RuntimeError("抱紧指令已发出，不重复上力。继续支撑并检查现场状态。")
        session = load(self.session_path) if self.session_path.exists() else {}
        if session.get("learning_may_be_active") and session.get("release_method") != "torque_switch":
            raise RuntimeError("遗留板载学习模式不能使用此恢复入口。")
        if self.stage == "teaching":
            self.fail(Stopped("现场选择停止示教并尝试当前位置抱紧；本条保留为中断草稿。"))
        if self.stage == "home_setup":
            self.cancel_home_setup()
        self.holding = "pending"
        self._session(learning_may_be_active=True,release_method="torque_switch",
                      hold_current_attempt={"status":"command_result_unknown","feedback_verified":False,
                                            "operator_confirmation_pending":True,"recovery_without_feedback":True})
        self.event("emergency_hold_requested")
        self.robot.torque(1)
        self._session(hold_current_attempt={"status":"torque_command_sent","feedback_verified":False,
                                            "operator_confirmation_pending":True,"recovery_without_feedback":True})
        self.message = "已发送当前位置抱紧指令，未发关节目标。继续支撑并现场检查；反馈恢复后再确认，异常期间禁止回放。"

    def begin_teaching(self):
        self.safety.require_clear()
        self._idle()
        if self.stage != "prepare" or self.holding != "free":
            raise RuntimeError("先进入手动示教准备。")
        # This is observation-only: recording may start while the supported arm
        # is already moving. Stability belongs to power/motion transitions.
        p = self.refresh()
        if self.teaching and self.teaching["status"] not in ("recorded",):
            raise RuntimeError("先保存或删除上一条示教，再开始新记录。")
        self.teaching = self.store.new_teaching(self.task, p)
        self.teaching["simulated"] = self.demo
        self.teaching["status"] = "recording"
        self.teaching["quality_warnings"] = []
        self.store.write_teaching(self.teaching)
        self.teach_started = time.monotonic()
        self.first_read_ns = None
        self.elapsed = 0
        self.capture = None
        self.stage = "teaching"
        self.persist()
        self.message = "正在记录关节，不保存视频。完成开合和搬运路径后点结束示教；最长120秒。"
        self.event("teaching_started")

    def teaching_tick(self):
        read_start = time.monotonic_ns()
        p = self.refresh()
        read_end = time.monotonic_ns()
        if self.first_read_ns is None:
            self.first_read_ns = read_start
        t = (read_start-self.first_read_ns)/1e9
        rows = self.teaching["samples"]
        if rows:
            gap=t-rows[-1]["t"]
            if 0 <= gap < .02:
                # A UI command can wake the worker between scheduled samples.
                # Keep its fresh feedback, without appending duplicate-time data.
                return
            if gap > .5 or gap < 0:
                self.event("teaching_bad_interval",gap_s=gap,previous=rows[-1]["q"],actual=p)
                raise RuntimeError(f"示教采样间隔{gap:.3f}秒超出允许范围，本条保留为异常草稿。")
            deltas=[abs(a-b) for a,b in zip(p,rows[-1]["q"])]
            if max(deltas)>10:
                warning={"sample_index":len(rows),"t":t,"gap_s":gap,"deltas":deltas}
                self.teaching["quality_warnings"].append(warning)
                self.event("teaching_fast_observation",**warning)
                self.message="正在记录。检测到较快手动变化，数据保留；保存时需审核，自动回放将逐段限速。"
        rows.append({"t": t, "q": p, "state_read_start_monotonic_ns": read_start,
                     "state_read_end_monotonic_ns": read_end})
        if len(rows)==1:
            self.teaching["reference_pose"]=p[:]
        self.elapsed = t
        if len(rows) % 10 == 0:
            self.store.write_teaching(self.teaching)
        if t >= 119.9:
            self.end_teaching()

    def end_teaching(self):
        if self.stage != "teaching":
            raise RuntimeError("当前不在记录示教。")
        self.teaching["status"] = "draft"
        self.store.write_teaching(self.teaching)
        self.stage = "teach_review"
        self.persist()
        self.message = "示教已结束，尚未保存。机械臂仍松力：先当前位置抱紧，再确认；可保存或删除重录。"
        self.event("teaching_ended",samples=len(self.teaching["samples"]))

    def save_teaching(self, reviewed=False):
        self._idle()
        if not self.teaching or self.teaching["status"] != "draft":
            raise RuntimeError("没有可保存的完整示教草稿。")
        d = copy.deepcopy(self.teaching)
        if d.get("quality_warnings"):
            self.require(reviewed)
            d["warnings_reviewed"]=True
        d["status"] = "recorded"
        _, scale = teaching_plan(d)
        self.teaching = d
        self.store.write_teaching(d)
        self.stage = "saved_teach"
        self.persist()
        self.message = f"人工示教已保存。回放按约{scale:.1f}倍时长限速；也可采下一条。"

    def select_teaching(self, item_id):
        self._idle()
        if self.capture and self.capture.meta["status"] == "draft":
            raise RuntimeError("先保存当前采集，或选择重录/删除；不会静默丢弃草稿。")
        if self.holding in ("free", "pending"):
            raise RuntimeError("先完成当前位置抱紧及现场确认。")
        d = load(self.store.item("teachings", item_id) / "trajectory.json")
        teaching_plan(d)
        self.teaching = d
        self.capture = None
        self.stage = "saved_teach"
        self.persist()
        self.task = d["task"]
        self.message = "已选择人工示教；回放返回本条实际起点，全局初始位置保持不变。没有移动机械臂。"

    def discard_teaching(self):
        self._idle()
        if self.capture and self.capture.meta["status"] != "saved":
            self.store.trash("episodes", self.capture.id)
        if self.teaching:
            self.store.trash("teachings", self.teaching["id"])
        self.teaching = None
        self.capture = None
        self.stage = "prepare" if self.holding == "free" else "ready"
        self.persist()
        self.message = "该人工示教及未保存的当前采集已移入回收站。硬件状态未改变。"

    def next_teaching(self):
        self._idle()
        if self.teaching and self.teaching["status"] not in ("recorded", "error", "interrupted"):
            raise RuntimeError("先保存或删除当前示教。")
        if self.capture and self.capture.meta["status"] == "draft":
            raise RuntimeError("先保存此次采集，或选择重录。")
        self.teaching = None
        self.capture = None
        self.stage = "prepare" if self.holding == "free" else "ready"
        self.persist()
        self.message = "已准备下一条。没有切换扭矩或移动机械臂。"

    def _motion(self, targets, recording=None):
        previous = self.refresh()
        if not near_pose(previous, targets[0], 2):
            raise RuntimeError("当前位置与计划起点不一致，未执行。")
        begun = self.camera.recording_anchor() if recording and hasattr(self.camera,"recording_anchor") else time.monotonic()
        # Compare observed motion with commands that have had time to execute.
        # The real controller has measured following lag; an instantaneous ideal
        # plant plus fixed 5/6-degree gates incorrectly rejected normal motion.
        history=[(begun,previous[:])]
        last_sent = None
        last_seq = -1
        last_frame_ns = None
        scheduled = begun
        for i, target in enumerate(targets):
            self.check_stop()
            time.sleep(max(0, scheduled-time.monotonic()))
            self.check_stop()
            tick = time.monotonic()
            if tick-scheduled > .2:
                raise RuntimeError("控制循环延迟超过200ms，停止后续运动。")
            camera_sample = None
            if recording:
                camera_sample = self.camera.get_for_tick(last_seq) if hasattr(self.camera,"get_for_tick") else self.camera.get(last_seq)
                last_seq = camera_sample[0]
                if last_frame_ns is not None and hasattr(self.camera,"max_recording_interval_error_ms"):
                    gap=(camera_sample[1]-last_frame_ns)/1e6
                    if abs(gap-100)>self.camera.max_recording_interval_error_ms:
                        raise RuntimeError(f'相机采样间隔异常（{gap:.1f}ms），停止后续指令并保留异常采集。')
                last_frame_ns=camera_sample[1]
                self.check_stop()
                if time.monotonic()-scheduled > .2:
                    raise RuntimeError("等待相机导致延迟超过200ms，未发送本次目标。")
            read_start = time.monotonic_ns()
            bounds, recent, issued = command_bounds(history, time.monotonic())
            actual = self.refresh(bounds)
            read_end = time.monotonic_ns()
            rechecks = getattr(self.robot, "last_read_rechecks", [])
            if rechecks:
                self.event("motion_feedback_rechecked", step=i, checks=rechecks,
                           motion_kind="replay_capture" if recording is not None else "return_home")
            envelope_ok=all(min(q[j] for q in recent)-3 <= actual[j] <= max(q[j] for q in recent)+3 for j in range(6))
            # State is read BEFORE this tick's command. Comparing it with the
            # pending target adds one step to measured lag and causes false trips.
            relative_ok=near_pose(actual,issued,12)
            if not envelope_ok or not relative_ok:
                axes = failed_joints(actual, bounds)
                self.event("motion_tracking_failed",step=i,actual=actual,previous_target=previous,
                           target=target,elapsed=tick-begun,recent_commands=recent,
                           envelope_ok=envelope_ok,relative_ok=relative_ok,issued_target=issued,
                           failed_axes=[j+1 for j in axes],feedback_bounds=bounds,
                           motion_kind="replay_capture" if recording is not None else "return_home")
                raise RuntimeError(failure_text(actual, issued, target, axes))
            max_camera_age=getattr(self.camera,"max_recording_age_ms",200)
            if recording:
                for role,sample in getattr(camera_sample,'views',{'wrist':camera_sample}).items():
                    if not 0 <= (read_start-sample[1])/1e6 <= max_camera_age:
                        raise CameraFrameError('相机帧过旧，未发送本次目标',
                            {'observed_monotonic_ns':read_start,'sequence':sample[0],
                             'frame_receive_monotonic_ns':sample[1],
                             'frame_age_ms':(read_start-sample[1])/1e6,'max_age_ms':max_camera_age},role)
            if time.monotonic()-scheduled > .2:
                raise RuntimeError("关节读取导致控制延迟超过200ms，未发送本次目标。")
            self.check_stop()
            command_sent = last_sent != target
            write_start = write_end = None
            if command_sent:
                write_start = time.monotonic_ns()
                self.robot.write(target, 100)
                write_end = time.monotonic_ns()
                last_sent = target[:]
                history.append((write_end/1e9,target[:]))
            if recording:
                recording.add(camera_sample, actual, last_sent, read_start, read_end, write_start, write_end, tick,
                              command_sent=command_sent)
            previous = target
            history=[row for row in history if tick-row[0]<=.6 or row is history[-1]]
            self.event("motion_sample",step=i,actual=actual,target=last_sent,command_sent=command_sent,
                       duration_ms=100 if command_sent else None)
            self.progress = (i+1)/len(targets)
            self.elapsed = tick-begun
            scheduled = self.camera.next_recording_tick(camera_sample) if recording and hasattr(self.camera,"next_recording_tick") else begun+(i+1)/10
        goal = targets[-1]
        goal_bounds = [(q-2, q+2) for q in goal]
        if recording is None:
            # No new position command during settling. Bound the observation
            # window and continue checking stop/emergency every <=100 ms.
            stable = 0
            for attempt in range(16):
                self.check_stop()
                actual = self.refresh(goal_bounds)
                stable = stable+1 if near_pose(actual, goal, 2) else 0
                if stable >= 2:
                    return
                if attempt < 15:
                    time.sleep(.1)
            self.event("motion_goal_failed", actual=actual, target=goal,
                       failed_axes=[j+1 for j in failed_joints(actual, goal_bounds)],
                       motion_kind="return_home", observation_window_s=1.5)
        else:
            time.sleep(.25)
            self.check_stop()
            actual = self.refresh(goal_bounds)
            if near_pose(actual, goal, 2):
                return
        raise RuntimeError("动作结束后仍未到达目标附近，保留为异常。"+
                           failure_text(actual, goal, goal, failed_joints(actual, goal_bounds)))

    def _countdown(self):
        for n in range(3,0,-1):
            self.check_stop()
            self.message = f"{n} 秒后开始运动。请移开手，停止按钮可取消。"
            for substep in range(10):
                self.check_stop()
                if substep in (0,5):
                    self.refresh()
                time.sleep(.1)

    def return_home(self, confirmed=False, countdown=True, target_pose=None):
        self.require(confirmed)
        self._power_required()
        destination = valid_pose(target_pose) if target_pose is not None else self.home
        if destination is None:
            raise RuntimeError("尚未设定初始状态。")
        self._idle()
        start = self.stable_pose()
        if self.stage == "prepare":
            self.stage = "ready"
        if near_pose(start, destination, 2):
            self.message = "当前已经在初始状态附近，没有发送运动指令。"
            return
        plan = home_plan(start, destination)
        planned_seconds = (len(plan)-1)/10
        destination_name = "本条示教起点" if target_pose is not None else "初始状态"
        self.event("return_home_plan", start=start, target=destination, planned_duration_s=planned_seconds,
                   max_speed_deg_s=30.0, max_acceleration_deg_s2=90.0, fixed_time_limit=False)
        self.activity = "returning"
        try:
            if countdown:
                self._countdown()
            # Human could have moved the arm during the countdown; never replan silently.
            self.message = f"正在平滑回位，计划运动约{planned_seconds:.1f}秒；按距离自动定时，不因超过5秒取消。"
            self._motion(plan)
            self.message = f"已回到{destination_name}。计划运动{planned_seconds:.1f}秒，稳定检查完成；未开始采集。"
        finally:
            self.activity = "idle"

    def replay_capture(self, confirmed=False, gripper_extra_degrees=0, joint3_offset_degrees=0):
        self.require(confirmed)
        self._idle()
        self._power_required()
        if not self.teaching or self.teaching["status"] != "recorded":
            raise RuntimeError("先保存或选择一条完整人工示教。")
        plan, scale = teaching_plan(self.teaching,gripper_extra_degrees,joint3_offset_degrees)
        _,gripper_policy=compensated_teaching(self.teaching,gripper_extra_degrees)
        _,joint3_policy=joint3_corrected_teaching(self.teaching,joint3_offset_degrees)
        if self.camera is None:
            raise RuntimeError("相机未连接。")
        self.camera.get()
        if hasattr(self.camera,"prepare_recording"):
            self.camera.prepare_recording()
        if shutil.disk_usage(self.store.root).free < 500*1024*1024:
            raise RuntimeError("磁盘剩余不足500MB。")
        # Re-record archives only the unsaved take. Accepted episodes remain untouched.
        self.return_home(confirmed=True, countdown=True,target_pose=plan[0])
        if self.capture and self.capture.meta["status"] != "saved":
            self.store.trash("episodes", self.capture.id)
            self.capture = None
        self.activity = "replaying"
        try:
            self._countdown()
            self.capture = Capture(self.store, self.teaching, self.cv2)
            self.persist()
            self.capture.meta.update(replay_time_scale=scale, simulated=self.demo,
                                     gripper_compensation=gripper_policy,
                                     joint3_position_correction=joint3_policy)
            if hasattr(self.camera,"status"):
                camera_status=self.camera.status()
                self.capture.meta.update(camera_configuration=camera_status,
                    camera_sync_policy="Every third frame of measured ~30Hz receive clock; nominal ~10Hz control; host receive timestamps, not hardware exposure sync",
                    max_image_age_ms=self.camera.max_recording_age_ms,
                    max_image_interval_error_ms=self.camera.max_recording_interval_error_ms)
                if camera_status.get('devices'):
                    self.capture.meta['camera_device']=camera_status['devices']['wrist']
                if camera_status.get('mode')=='dual':
                    self.capture.meta.update(max_camera_pair_skew_ms=self.camera.max_pair_skew_ms,
                        secondary_sample_interval_bounds_ms=[50,150],
                        camera_sync_policy='Primary receive-clock ~10Hz; independent secondary latest fresh frame paired within35ms; secondary interval50..150ms; host timestamps, not hardware exposure sync')
            self.capture.write_meta()
            self.message = "正在自动执行并同步采集RGB、实际关节反馈和真实控制指令。"
            self._motion(plan, self.capture)
            self.capture.finish("encoding")
            self.activity = "encoding"
            self.message = "动作结束，正在合成视频；原始帧与动作已写入。"
            self.capture.encode(self.stop)
            self.stage = "capture_review"
            self.message = "完整采集草稿已生成。可重放重录、删除人工示教，或保存此次数据。"
        except BaseException as error:
            if self.capture and self.capture.meta["status"] not in ("saved", "draft"):
                if hasattr(error,'diagnostics'):
                    self.capture.meta['camera_failure_diagnostics']=error.diagnostics
                self.capture.finish("interrupted" if isinstance(error,Stopped) else "error", str(error))
            self.stage = "capture_review" if self.capture else "saved_teach"
            raise
        finally:
            self.activity = "idle"

    def save_capture(self, outcome=None):
        self._idle()
        if not self.capture or self.capture.meta["status"] != "draft" or not self.capture.meta["capture_checks_passed"]:
            raise RuntimeError("当前采集未通过完整性检查，不能保存为完整数据。")
        if outcome is not None and not isinstance(outcome,bool):
            raise ValueError("任务结果无效。")
        self.capture.meta.update(status="saved", operator_success=outcome,
                                 saved_utc=datetime.now(timezone.utc).isoformat())
        self.capture.write_meta()
        self.persist()
        self.message = "此次数据已保存：视频、逐帧状态/动作和人工示教在同一目录。仍需审核、转换训练格式。"

    def poll(self):
        try:
            if self.stage == "teaching":
                self.teaching_tick()
            else:
                self.refresh()
        except FeedbackError as error:
            # Skip failed observations, retaining the real monotonic timeline. A
            # recovered read still passes the existing <=500 ms gap/jump gates.
            if self.stage == "teaching" and (self.feedback_failures>=3 or
                    (self.feedback_since is not None and time.monotonic()-self.feedback_since>=.35)):
                self.fail(RuntimeError("示教已中断：连续关节反馈异常。本条保留为异常草稿，不能回放。"+str(error)))
        finally:
            # A failed joint read must not freeze the independent camera preview.
            self.preview_tick()

    def preview_tick(self):
        if not self.camera or not self.cv2 or time.monotonic()-self.last_preview < .2:
            return
        try:
            # LatestCamera's background thread owns VideoCapture. No UI thread reads a device.
            if hasattr(self.camera,'preview_views'):
                views,errors=self.camera.preview_views()
                frames={}
                for role,sample in views.items():
                    image=sample[2]
                    # Encode only the display resolution. Recording retains the
                    # original full-resolution frames and timestamps.
                    dual=getattr(self.camera,'config',{}).get('mode')=='dual'
                    max_width,max_height=(320,240) if dual else (480,360)
                    height,width=image.shape[:2]
                    ratio=min(1,max_width/width,max_height/height)
                    if ratio<1:image=self.cv2.resize(image,(round(width*ratio),round(height*ratio)))
                    ok,data=self.cv2.imencode('.jpg',image,[self.cv2.IMWRITE_JPEG_QUALITY,75])
                    if ok:frames[role]=data.tobytes()
                    else:errors[role]='相机预览编码失败。'
                self.preview_frames=frames;self.preview_errors=errors
                self.preview=frames.get('wrist')
                self.camera_error='；'.join(f'{role}: {message}' for role,message in errors.items())
                return
            if hasattr(self.camera,'preview_sample'):
                sample=self.camera.preview_sample()
            elif hasattr(self.camera, "condition"):
                with self.camera.condition:
                    sample = self.camera.latest
                    error = self.camera.error
                if error:
                    raise RuntimeError(error)
                if sample is None:
                    return
                if time.monotonic_ns()-sample[1] > 500_000_000:
                    raise RuntimeError("相机画面已过期。")
            else:
                sample = self.camera.get()
            ok, data = self.cv2.imencode(".jpg", sample[2], [self.cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                self.preview = data.tobytes()
                self.camera_error = ""
        except Exception as error:
            self.preview = None
            self.preview_frames={};self.preview_errors={'wrist':str(error)}
            self.camera_error = str(error)
        finally:
            self.last_preview = time.monotonic()

    def reset_emergency(self,confirmed=False):
        self.require(confirmed)
        self._idle(allow_home_setup=True)
        self.safety.reset()
        self.stop.clear()
        self.message='软件急停已手动复位；操作未自动续接，扭矩状态未改变。'
        self.event('emergency_reset')

    def fail(self, error):
        self.error = str(error)
        self.feedback_command_failed = isinstance(error,FeedbackError)
        self.message = str(error)
        if self.stage == "teaching":
            self.teaching.update(status="interrupted" if isinstance(error,Stopped) else "error", error=str(error))
            self.stage = "teach_review"
            try:
                self.store.write_teaching(self.teaching)
            except OSError as storage_error:
                self.message += "；异常草稿写入失败："+str(storage_error)
        self.activity = "idle"
        details={'detail':str(error)}
        if hasattr(error,'diagnostics'):details['camera_failure_diagnostics']=error.diagnostics
        self.event("operation_failed", **details)

    def close(self):
        if self.stage == "teaching":
            self.fail(Stopped("软件关闭，示教已中断；扭矩未自动改变。"))
        if self.camera:
            self.camera.close()
        self.robot.close()
