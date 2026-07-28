# Robot 306 视觉抓取交接

## 已交付链路

`RealSense D435i RGB + 对齐深度 -> YOLO11 分割/ByteTrack -> 掩膜内稳健深度
-> 相机三维点 -> TF 到 Link_Zero_Point -> GraspNet 6D 候选 -> TCP 换算
-> MoveIt 2 / 厂商 IK、FK、RRT -> 双臂轨迹 -> 平行夹爪 -> 抬升`

自然语言入口为 OpenClaw workspace：
`agent_workspace/skills/visual-grasping/SKILL.md`。它把“抓一瓶水”等 prompt
映射到检测类别，并且默认只做 dry-run。

## 关键硬件

- 主机：AutoLife Robot 306，Ubuntu 24.04，ROS 2 Jazzy
- GPU：RTX 4090 Laptop 16 GB
- RGB-D：Intel RealSense D435i，序列号 `261822074480`
- 机械臂：AutoLife robot_v2_2，左右各 7 DoF，厂商 ROS 服务后缀 `0_306`
- 夹爪：左右平行夹爪，厂商位置范围约 0（开）到 360（闭）

## 来源与固定版本

- GraspNet baseline：`https://github.com/graspnet/graspnet-baseline`，
  commit `280c215129f759ed8649cb4e89fc5dfee55f4f80`
- OpenClaw：`https://github.com/openclaw/openclaw`，
  commit `d0669429d857e4cd2cfc39b030095c5424f0b94f`
- Ultralytics YOLO：包内独立 venv，模型 `models/yolo/yolo11n-seg.pt`
- GraspNet RealSense 权重：
  `models/graspnet/checkpoint-rs.tar`

厂商机械臂 SDK 是机器人预装的 `autolife_robot_arm 2.2.3+build3`，控制代码
使用其官方话题、IK、FK 和全身 RRT 服务。

## 构建与测试

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
bash tools/build_project.sh
```

启动、检查、列出对象、规划瓶子抓取：

```bash
tools/graspctl.sh start
tools/graspctl.sh status
tools/graspctl.sh list
tools/graspctl.sh plan bottle auto
tools/graspctl.sh logs 200
```

停止仅停止本项目 launch，不停止厂商 arm/vision 服务：

```bash
tools/graspctl.sh stop
```

## OpenClaw

源码及依赖全部在项目目录。首次或更新源码后：

```bash
bash tools/install_openclaw.sh
```

配置模型凭据后运行自然语言任务，例如：

```bash
tools/openclaw.sh "请识别桌面上的水瓶并完成抓取规划，不要让机械臂运动"
```

OpenClaw 未包含任何 API key。请按 `third_party/openclaw/docs` 配置所选模型。

## 真机解锁（必须由现场人员完成）

交付配置保持：

```yaml
dry_run: true
allow_hardware_execution: false
enable_pbvs_hardware_follow: false
```

真机动作前必须逐项完成：

1. 用标定板复核头部相机外参，不可只依赖名义安装位姿。
2. 复核左右 TCP 位于两指实际夹持中心。
3. 清空桌面碰撞区；先用空夹爪和软质轻物体。
4. RViz 检查预抓取、抓取、抬升三段轨迹。
5. 现场一人守急停，速度保持当前 0.25 或更低。
6. 手工把 `config/motion.yaml` 的 `dry_run` 改为 `false`、
   `allow_hardware_execution` 改为 `true`，重启本项目栈。
7. 先执行一次 `tools/graspctl.sh plan bottle auto`，确认成功后才执行：

```bash
tools/graspctl.sh execute bottle auto I_HAVE_CHECKED_ESTOP_AND_WORKSPACE
```

首次实机不要开启 PBVS 硬件跟随。任一相机、TF、关节、EEF、IK/FK/RRT
检查失败时不得绕过。

## 已知限制

- 通用性取决于检测模型类别；默认 COCO 模型能识别 bottle/cup 等，未知物体
  需替换兼容的实例分割权重或接入开放词汇检测器。
- 干运行 IK 已通过，但 MoveIt 日志仍提示没有配置
  `moveit_controller_manager`。在将厂商轨迹控制器完整映射到 MoveIt 并做现场
  空载验证前，不得开启真实轨迹执行。
- GraspNet 权重限其上游许可范围使用。
- 当前外参/TCP 是名义或既有实测值，不能替代每台机器人现场标定。
- OpenClaw 是决策入口，不进入实时控制环；确定性的 ROS 节点承担感知、规划
  和执行，避免 LLM 直接输出关节命令。

## 2026-07-28 验收记录

- 目标目录独立 `colcon build` 成功，30 tests / 0 failures。
- 在线硬件检查通过：RGB-D、内参、关节、EEF、相机 TF、左右 TCP 均在线。
- 画面识别到 `bottle`：置信度约 0.835，深度约 0.604 m。
- 完整 `execute:false` 规划成功：GraspNet 分数约 0.305，选择右臂，
  夹爪需求宽度约 0.0888 m；IK dry-run 通过，未发送硬件命令。
- OpenClaw 2026.7.2 构建成功，`visual-grasping` skill 状态为
  `eligible=true`、`modelVisible=true`。
- 二次在线复测修复了厂商 SDK `utils` 初始化时误加载不兼容
  `torchaudio`、导致 RGB-D 共享内存桥停发的问题。修复后 RGB 与对齐深度以
  1280x720、帧差 0 持续发布，硬件健康检查全部通过。
- 二次复测画面识别到 `bottle`：置信度约 0.929，深度约 0.531 m；
  单一 action server 下完整规划成功，GraspNet 分数约 0.267，选择左臂，
  夹爪需求宽度约 0.0739 m；IK dry-run 通过，未发送硬件命令。
- `graspctl.sh` 现在按整个 `setsid` 进程组检查和停止服务，避免 launch
  进程提前退出后残留 ROS 节点、继而产生重复 action server。
- 验收结束后已停止本项目栈；厂商 arm/vision 基础服务保持原状。
