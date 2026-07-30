# Dual-Arm RGB-D Visual Grasping for AutoLife S2

面向 AutoLife S2 双臂机器人的 ROS 2 Jazzy 视觉抓取系统：实时 RGB-D 感知、
YOLO11 实例分割与跟踪、GraspNet 6D 抓取候选、同目标新帧复核、MoveIt 规划，
以及受强制安全门保护的厂商 18 关节轨迹适配。

[详细中文说明](README_ZH.md) ·
[V2 架构](docs/ARCHITECTURE_V2_ZH.md) ·
[部署交接](HANDOFF.md) ·
[高 Star 参考](docs/HIGH_STAR_DUAL_ARM_REFERENCES_ZH.md)

> [!WARNING]
> 当前仓库默认且必须保持 `dry_run`。现场 PlanningScene、相机与双 TCP 标定、
> 夹爪映射、实体急停测试和签名验收尚未全部完成。探针 `ready=true` 只表示输入
> 在线，不代表允许机器人运动。

## 当前状态

- 机器人模型：左臂 7 DoF + 右臂 7 DoF。
- 厂商轨迹契约：腰腿 4 + 左臂 7 + 右臂 7，共 18 个值。
- 当前任务语义：选择一只主动臂抓取，另一只臂保持；不是实机双臂协同搬运。
- 唯一硬件规划入口：`moveit_py_vendor_execution`。
- PBVS 直接硬件跟随不受支持；厂商 RRT/末端位姿后端仅用于 dry-run。
- 干净源码构建验证：`72 tests, 0 errors, 0 failures, 0 skipped`。
- robosuite 双 Panda 验证：两路 7 轴状态、14 维 action、10 个有限物理步，
  `hardware_connected=false`。

## V2 数据链

```text
RealSense RGB-D
  -> YOLO11-seg + track id
  -> 原子 observation（RGB/depth/mask/intrinsics）
  -> opaque token + payload size + SHA-256
  -> GraspNet + 目标邻域点云碰撞过滤
  -> 同一 track 的新帧目标复核
  -> MoveIt dry-run
  -> MotionExecutor 安全边界
  -> AutoLife 18 关节适配器（当前锁定）
```

V2 不再把约 5 MB 的 RGB-D 数据复制进多个 DDS 服务，也不再拼接相互独立的
RGB、深度和内参缓存。选中观察仅存放在项目自己的
`runtime/selected_observations`，由 GraspNet 校验路径、文件类型、大小、
SHA-256、track、时间戳、frame、shape 和 dtype 后单次消费。

## 主要安全不变量

真实命令只有同时满足下列条件才可能到达厂商适配器：

```text
dry_run=false
AND allow_hardware_execution=true
AND backend=moveit_py_vendor_execution
AND enable_pbvs_hardware_follow=false
AND 精确操作员确认令牌
AND site_acceptance 全部通过
AND hardware probe 新鲜且保护状态正常
AND MotionExecutor 获得唯一执行锁
```

任一硬件请求发布后若发生超时或传输异常，系统会锁存
`unknown-motion fault`，进程内不能清除。项目没有经过验证的软件停止接口；
紧急停止必须使用现场实体急停。

## 目录

```text
adaptive_object_grasping_nodes/   ROS 节点与纯逻辑核心
action/ msg/ srv/                 ROS 2 接口
config/                           感知、规划、硬件与 MoveIt 配置
launch/ scripts/ tools/           启动、安装、检查和操作入口
test/                             无硬件单元测试
experiments/robosuite_dual_panda/ 双 Panda 2×7-DoF 仿真冒烟
docs/                             架构、迁移、硬件与验证文档
agent_workspace/                  OpenClaw 自然语言入口
```

模型、虚拟环境、CUDA/Python 依赖、运行日志和现场配置不会提交到 Git；它们由
安装脚本在项目目录内重建。

## 构建与测试

要求 Ubuntu 24.04、ROS 2 Jazzy 和 `colcon`：

```bash
git clone https://github.com/andyzeng0921/zeng-Visual-Grasping.git
cd zeng-Visual-Grasping

source /opt/ros/jazzy/setup.bash
bash tools/build_project.sh
python3 tools/check_dual_arm_profile.py
```

AI 环境单独安装在 `third_party/`，不会污染厂商机器人环境：

```bash
bash tools/install_ai_environment.sh
```

GraspNet baseline 和权重需要先接受其上游学术/非商业许可，再按
[迁移说明](docs/MIGRATION_GUIDE_ZH.md) 安装。

## 安全 dry-run

首次使用先创建本地现场验收文件。真实文件已被 `.gitignore` 排除：

```bash
cp config/site_acceptance.example.yaml config/site_acceptance.yaml

./tools/graspctl.sh start
./tools/graspctl.sh status
./tools/graspctl.sh list
./tools/graspctl.sh plan bottle auto
./tools/graspctl.sh logs 200
./tools/graspctl.sh stop
```

`plan` 的安全失败是正常结果：目标移动、遮挡、深度无效、候选碰撞、夹爪宽度或
方向不合格、重验证失败时都会 fail closed。不要为获得成功结果而放宽阈值。

`graspctl.sh stop` 只停止本项目 ROS 进程，不是机器人停止命令。

## 双七轴仿真

无 ROS、无 AutoLife SDK、无真实硬件的双 Panda 冒烟测试位于
[`experiments/robosuite_dual_panda`](experiments/robosuite_dual_panda)。
依赖固定为 robosuite 1.5.2、MuJoCo 3.3.7 和 NumPy 1.26.4。

已验证结果 SHA256：

```text
6a73312d2b654eeae872176659c684d40ab617d552bae10cd07acb2b6b826600
```

## 许可证与公开发布

- `package.xml` 当前声明 Apache-2.0，但仓库尚未加入项目级 `LICENSE`。
- AutoLife S2 的 URDF/STL 再发布权需要在把仓库改为公开前单独确认。
- GraspNet baseline 与模型权重不包含在仓库内，并受其上游许可约束。
- 当前建议保持仓库私有，发布前按
  [`GITHUB_UPLOAD_CHECKLIST_ZH.md`](GITHUB_UPLOAD_CHECKLIST_ZH.md) 完成检查。
