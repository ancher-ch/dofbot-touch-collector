"""Tracking is relative to issued commands; a pending target has not moved the arm."""
JOINT_NAMES = ("底座", "肩部", "肘部", "腕俯仰", "腕旋转", "夹爪")


def command_bounds(history, now):
    recent = [pose for stamp, pose in history if 0 <= now-stamp <= .45]
    issued = history[-1][1]
    if not recent:
        recent = [issued]
    bounds = [(max(min(q[j] for q in recent)-3, issued[j]-12),
               min(max(q[j] for q in recent)+3, issued[j]+12)) for j in range(6)]
    return bounds, recent, issued


def failed_joints(actual, bounds):
    return [j for j, (value, (low, high)) in enumerate(zip(actual, bounds))
            if not low <= value <= high]


def failure_text(actual, issued, pending, axes):
    detail = "；".join(f"J{j+1}{JOINT_NAMES[j]} 实际{actual[j]}°/已发{issued[j]}°/待发{pending[j]}°" for j in axes)
    text = "反馈偏离已发送目标，已停止后续指令。" + detail + "。"
    if 5 in axes:
        text += "夹爪未跟上不等于夹紧成功；检查物体、机构和舵机状态，勿靠增加补偿强行通过。"
    return text
