# 软件模块与旧CLI

主入口、安装和操作说明见仓库根 README.md 与 docs/使用与安装说明.md。

此目录必须完整保留：触屏后端复用 arm_control.py、capture_check.py、teleop_record.py、trajectory_teach.py 等模块。repair_joint3.py 与 return_to_start.py 是开发辅助工具，不是触屏采集流程，不要按文件名直接运行校准或回位。

旧CLI需要现场交互确认，不应与触屏应用或其他控制程序并行。GitHub分发已将旧CLI相机HTTP预览限制在127.0.0.1；电脑预览用正常认证的SSH隧道，不开放局域网/公网监听。位置、扭矩、抱紧与松力都需要现场判断；软件停止不是硬件急停。

在本目录执行 python3 -m unittest discover 运行模拟硬件测试，不使用真实机器人、相机或I2C。含 readonly_check 名称的脚本可能访问硬件，不属于纯模拟测试。
