"""Hardware boundary. Demo mode never imports Arm_Lib or opens I2C."""
import time

from arm_control import write_pose
from trajectory_teach import torque_command


class PacedBus:
    """Space host I2C transactions for the MCU's asynchronous servo bridge.

    The single worker remains the only owner. This is host-side pacing, not a
    claim that a servo replied or that a stalled controller has been reset.
    """
    def __init__(self, bus, gap=.003):
        self.bus=bus
        self.gap=gap
        self.last_end=0.
        self.transactions=0

    def call(self, name, *args):
        time.sleep(max(0,self.gap-(time.monotonic()-self.last_end)))
        try:
            return getattr(self.bus,name)(*args)
        finally:
            self.last_end=time.monotonic()
            self.transactions+=1

    def write_byte_data(self,*args): return self.call("write_byte_data",*args)
    def write_i2c_block_data(self,*args): return self.call("write_i2c_block_data",*args)
    def read_word_data(self,*args): return self.call("read_word_data",*args)
    def read_byte_data(self,*args): return self.call("read_byte_data",*args)
    def close(self): self.bus.close()


class FeedbackError(RuntimeError):
    def __init__(self, pose, hints, reasons=None):
        self.reasons = reasons or [None]*6
        parts = [f"J{i+1}: {self.reasons[i] or '读数无效'}" for i,x in enumerate(pose) if x is None]
        super().__init__("；".join(parts) + "。保持支撑；勿硬掰关节。")
        self.pose = pose
        self.hints = hints


class RealRobot:
    def __init__(self):
        from Arm_Lib import Arm_Device
        self.arm = Arm_Device(i2c_bus=7)
        self.arm.bus = PacedBus(self.arm.bus)
        self.position_requests = 0
        self.torque_requests = 0

    def read(self):
        pose, hints, reasons = [], [], []
        for j in range(1, 7):
            value = self.arm.Arm_serial_servo_read(j)
            # A single transient reply must not destroy an entire teaching. Bound the
            # retry to one per joint; never substitute a cached/clamped angle.
            if value is None:
                time.sleep(.003)
                value = self.arm.Arm_serial_servo_read(j)
            pose.append(value)
            hint = None
            reason = None
            if value is None:
                reason = "SDK无有效应答"
                try:
                    address = getattr(self.arm,"addr",0x15)
                    self.arm.bus.write_byte_data(address, 0x30 + j, 0)
                    # Same turnaround as the installed manufacturer's read method.
                    time.sleep(0.003)
                    w = self.arm.bus.read_word_data(address, 0x30 + j)
                    raw = ((w >> 8) & 255) | ((w << 8) & 65280)
                    if 0 < raw <= 4095:
                        hint = 270 * (raw - 380) / 3320 if j == 5 else 180 * (raw - 900) / 2200
                        if j in (2, 3, 4):
                            hint = 180 - hint
                        hint = round(hint, 1)
                        maximum = 270 if j == 5 else 180
                        reason = f"原始映射约{hint}°，超出SDK范围" if not 0 <= hint <= maximum else "原始映射在范围内，SDK应答不稳定"
                    else:
                        reason = f"无效原始应答({raw})"
                except OSError:
                    reason = "I2C读取失败"
            hints.append(hint)
            reasons.append(reason)
        if any(x is None for x in pose):
            raise FeedbackError(pose, hints, reasons)
        return pose

    def write(self, pose, duration):
        if hasattr(self,'safety_guard'):self.safety_guard()
        self.position_requests += 1
        write_pose(self.arm, pose, duration)

    def torque(self, enable):
        if hasattr(self,'safety_guard'):self.safety_guard()
        self.torque_requests += 1
        torque_command(self.arm, enable)

    def close(self):
        self.arm.bus.close()


class DemoRobot:
    def __init__(self):
        self.pose = [90, 120, 35, 25, 90, 30]
        self.writes = []
        self.torques = []

    def read(self):
        return self.pose[:]

    def write(self, pose, duration):
        if hasattr(self,'safety_guard'):self.safety_guard()
        self.writes.append((list(pose), duration))
        self.pose = list(pose)

    def torque(self, enable):
        if hasattr(self,'safety_guard'):self.safety_guard()
        self.torques.append(enable)

    def close(self):
        pass


class DemoCamera:
    error = None

    def __init__(self, cv2):
        self.cv2 = cv2
        self.latest = None
        self.sequence = 0

    def start(self):
        pass

    def get(self, after=-1):
        import numpy as np
        f = np.full((480, 640, 3), (210, 220, 224), dtype=np.uint8)
        self.cv2.rectangle(f, (0, 305), (640, 480), (155, 176, 183), -1)
        self.cv2.rectangle(f, (365, 260), (555, 390), (93, 126, 140), 3)
        self.cv2.rectangle(f, (145 + self.sequence % 80, 290), (205 + self.sequence % 80, 350), (65, 169, 218), -1)
        self.cv2.putText(f, "SIMULATION / NO ROBOT", (35, 60), self.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 65, 70), 2)
        self.sequence += 1
        self.latest = (self.sequence, time.monotonic_ns(), f)
        return self.latest

    def close(self):
        pass
