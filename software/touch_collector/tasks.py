"""Task text validation and a small local history; no hardware access."""
import re

DEFAULT_TASKS = ("把方块抓起放进固定盒子", "空载抓取路径测试", "关节小幅往返测试")
MAX_TASK_LENGTH = 200
SEQUENCE_TASK = "把黄色方块放入盒子，再把红色圆柱放入盒子"


def clipboard_repair_candidates(value):
    """Preview reversible VNC byte-decoding errors; never change text implicitly."""
    if not isinstance(value, str) or any('\u3400' <= c <= '\u9fff' for c in value):
        return []
    raw_versions = []
    for transport in ('latin1', 'cp1252'):
        try:
            raw = value.encode(transport)
        except UnicodeError:
            continue
        if raw not in raw_versions:
            raw_versions.append(raw)
    results = []
    for raw in raw_versions:
        for source in ('utf-8', 'gb18030'):
            try:
                repaired = task_name(raw.decode(source))
            except (UnicodeError, ValueError):
                continue
            if sum('\u3400' <= c <= '\u9fff' for c in repaired) >= 2 and repaired not in results:
                results.append(repaired)
                # Valid UTF-8 is unambiguous here; do not offer GBK gibberish too.
                break
    return results


def task_name(value):
    if not isinstance(value, str):
        raise ValueError("任务名称必须是文字。")
    if any(ord(char) < 32 and char not in "\t\r\n" or ord(char) == 127 for char in value):
        raise ValueError("任务名称含有不可见控制字符。")
    value = re.sub(r"\s+", " ", value).strip()
    if not value:
        raise ValueError("请填写任务名称。")
    if len(value) > MAX_TASK_LENGTH:
        raise ValueError(f"任务名称不能超过{MAX_TASK_LENGTH}字。")
    return value


def task_choices(current, recent=()):
    result = []
    for value in [current, *recent, *DEFAULT_TASKS]:
        try:
            value = task_name(value)
        except ValueError:
            continue
        if value not in result:
            result.append(value)
    return result[:30]
