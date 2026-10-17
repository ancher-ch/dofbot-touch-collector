# GitHub 分发说明

当前版本为 v0.1.7.post1。旧记录仅供开发参考；安装与路径以仓库根 README 和 docs/使用与安装说明.md 为准。旧 CLI 的 HTTP 预览仅监听本机，外部观看须使用 SSH 隧道。

# 调研与复用记录

调研日期：2026-10-17。

v0.1.4对照[LeRobot相机访问模式](https://huggingface.co/docs/lerobot/main/en/cameras)及[OpenCVCamera实现](https://github.com/huggingface/lerobot/blob/main/src/lerobot/cameras/opencv/camera_opencv.py)，借鉴后台取帧、最新帧预览与有界新帧采集分离；继续适配本机现有OpenCV，不引入LeRobot完整依赖。缓冲数使用本机A/B测量选择四缓冲，并未照搬其他相机的缓冲配置。自动曝光帧率权限语义核对[Linux V4L2相机控制文档](https://www.kernel.org/doc/html/v5.7/media/uapi/v4l/ext-ctrls-camera.html)；实测关闭此权限本身没有修复本机低帧率，不能把它写成已确认根因。

| 来源 | 借鉴或复用 | 适配本机的决定 |
|---|---|---|
| [Hugging Face LeLab](https://github.com/huggingface/leLab) | 图形化校准/示教/记录/回放、数据浏览的工作流 | 上游pyproject要求Python≥3.12，并依赖LeRobot完整栈；这台Jetson是Python3.10，DOFBOT使用I2C Arm_Lib。因此借鉴工作流，采用已安装的PyQt5做本机触屏界面 |
| [LeRobot录制脚本](https://github.com/huggingface/lerobot/blob/main/src/lerobot/scripts/lerobot_record.py) | 明确区分观测、实际发送动作、episode暂存、重录、保存 | 在本机实现相同语义；不直接套用其他机械臂的驱动、断开时松力行为或电机标定 |
| [LeRobotDataset v3](https://huggingface.co/docs/lerobot/lerobot-dataset-v3) | 独立episode、视频与状态/动作对应、任务和来源元数据 | 当前保存MP4+JSONL原始数据；不把它标称为原生LeRobot v3数据集，后续导出需单独验证 |
| [Yahboom DOFBOT-PRO](https://github.com/YahboomTechnology/DOFBOT-Pro) | 本机已经安装的Arm_Lib接口 | 只调用现有角度读取、同步位置写入、直接扭矩开关；不修改零位、不清除厂家动作组 |
| [厂商扭矩接口](https://www.yahboom.net/study/DOFBOT-Pro/12504/) | 开启扭矩维持当前位置 | 将“当前位置抱紧”和“回到初始姿态”分成两个独立操作；记录指令与现场确认，避免靠角度稳定推断扭矩状态 |
| [Qt线程说明](https://doc.qt.io/archives/qt-5.15/qthread.html) | GUI线程与设备工作分离 | 单个设备线程拥有I2C，UI只提交命令、显示快照；停止信号不排在长任务队列之后 |
| [FFmpeg image2](https://ffmpeg.org/ffmpeg-formats.html#image2-1) | 由有序RGB帧生成可播放视频 | 视频帧数与JSONL行数检查一致，保留真实采样时间戳与原始帧 |

应用核心复用本工程已验证的`valid_pose`、`near_pose`、`prepare_plan`、`save_json`、`write_pose`、`torque_command`和`LatestCamera`。LeLab/LeRobot仅作为流程与格式参考，未声称本应用已获得上游DOFBOT适配或认证。

2026-10-17 v0.1.3再次对照LeRobot的`record_loop`、`lerobot_replay.py`与[SO follower send_action](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)：观测记录与动作执行分离，执行端单独约束目标相对当前角度的距离。本机手拖记录不因相邻真实角度变化超过10°而丢弃整条；较快变化必须人工审核，执行轨迹逐段限速、记录实际发送动作。未复制SO硬件的零位、PID、断电或标定流程。

另查阅[jleppert/yahboom的旧款DOFBOT使用记录](https://github.com/jleppert/yahboom)：作者记录快速I2C操作或多控制进程导致舵机总线失效、需要完整控制板断电的现象。该项目为Jetson Nano、Arm_Lib 0.0.5，与本机DOFBOT-PRO、Arm_Lib 1.0.3不同；其无效反馈/无应答症状也不完全符合本次仍可读角度的现象。因此仅借鉴单一总线所有者、减少重复指令和完整断电实验，不据此断言本机故障原因，也不复制复位、标定或厂商服务禁用命令。驱动增加3ms主机事务间隔；完整断电后连续两次真实空载回放通过，但各措施对恢复的独立贡献未分离验证。
