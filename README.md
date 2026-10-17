# DOFBOT Touch Collector

DOFBOT-PRO 本机触屏数据采集台，Python / PyQt5，GitHub分发修订 **v0.1.7.post1**。

手动示教只记录关节；自动回放时记录 RGB、真实关节状态和实际执行目标，人工审核后保存原始 episode。支持初始姿态、单/双RGB配置、夹爪0/1/2°补偿、视频浏览、数据管理、软删除/恢复、ZIP导出和持久化软件急停。

## 先读

- [详细安装与使用说明](docs/使用与安装说明.md)
- [测试与兼容性](docs/测试与兼容性.md)
- [发布检查](docs/发布检查.md)
- [安全说明](SECURITY.md)

**适配环境：** DOFBOT-PRO、Jetson Ubuntu22.04、Python3.10、厂家Arm_Lib、I2C bus7。不是通用6轴驱动，不是Windows安装程序。厂家SDK和系统依赖不随仓库分发。

**实体双摄仍待验证。** 现有单摄硬件是带深度能力的Orbbec，但软件只保存RGB，不保存深度/IR。原始MP4+JSONL不是直接可训练的LeRobot v3数据集。

## 从模拟开始

完整下载/解压仓库，在机器人Ubuntu图形桌面终端进入仓库根目录：

```bash
/usr/bin/python3 verify-package.py
/usr/bin/python3 check-environment.py --demo
bash launch-demo.sh
```

模拟不打开I2C、不实例化厂家SDK，生成数据不能混入真实训练。真机配置、支撑和路径检查完成后才使用：

```bash
/usr/bin/python3 check-environment.py --real
bash launch-robot.sh
```

真机数据默认当前用户 ~/dofbot-data/touch_collector；关闭程序不自动改变扭矩。关节读数有效不代表已经抱紧。松力没有重力补偿，必须持续可靠支撑。

## 安全边界

软件急停只拦截本软件后续指令，不切电，不能撤销已经发往控制板的目标，不能替代硬件急停。复位不自动续动。不要将机器人控制接口、SSH或预览端口暴露到公网。

原有CLI仅保留开发兼容；HTTP相机预览默认只监听127.0.0.1。电脑端需要预览时，在电脑使用正常SSH认证建立隧道：

```bash
ssh -N -L 8765:127.0.0.1:8765 robot_user@ROBOT_IP
```

再在电脑浏览器访问 http://127.0.0.1:8765/。该HTTP预览本身没有认证/TLS，仍依赖本机信任与SSH隧道；触屏应用不需要此HTTP服务。

## 隐私与开发

本仓库只包含代码、合成测试和说明，没有真实数据、凭据或运行状态。不要添加 .env、密钥、视频、日志或 home/work/safety/session 状态文件。默认忽略规则不是秘密扫描的替代品。

```bash
python3 scripts/check_publication.py
cd software
python3 -m unittest discover
```

建议安装官方Gitleaks后扫描历史与工作区。Ubuntu/Linux首次启用本地提交检查时，先授予钩子可执行权限，再配置钩子目录：

```bash
chmod +x .githooks/pre-commit
git config core.hooksPath .githooks
```

尤其是网页上传或ZIP解压得到的副本，不要假设已有可执行权限。启用后，缺少Gitleaks会拒绝提交。GitHub CI使用只读权限、固定提交版本的Actions，并校验扫描工具SHA256；只运行模拟测试，不连接机械臂。参考 [Gitleaks](https://github.com/gitleaks/gitleaks) 和 [GitHub防泄漏说明](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)。

工程借鉴LeRobot的观测/动作分离、后台取帧和episode管理思路，复用本机Qt/FFmpeg/Arm_Lib适配；未声称获得LeRobot或厂家认证，详见 software/touch_collector/REFERENCES.md。未捆绑厂家SDK。仓库尚未指定开源许可证，公开或进一步再分发时应由维护者确定许可。
