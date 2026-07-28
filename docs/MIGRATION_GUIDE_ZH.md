# adaptive_object_grasping 简易迁移手册

## 1. 迁移内容

必须完整复制整个 `adaptive_object_grasping` 源码目录，不能只复制 ROS
源码和配置。下列目录包含运行所需的大文件，并且可能被 Git 忽略：

```text
models/yolo/
models/graspnet/
third_party/venv/
third_party/graspnet-baseline/
```

Python 虚拟环境与 CPU 架构、Ubuntu、Python 和 CUDA 版本相关。目标机器人
环境不一致时，不要直接复用 `third_party/venv`，应在目标机器人重新执行：

```bash
bash tools/install_ai_environment.sh
```

同时确认 GraspNet checkpoint、YOLO 权重和 PointNet2 扩展均已安装。

IK 虚影和 MoveIt 可达性验证还需要：

```bash
sudo apt install ros-jazzy-moveit-py ros-jazzy-moveit-planners-ompl
```

## 2. 收集目标机器人参数

迁移前确认：

- ROS 2 版本，当前为 Jazzy。
- `ROS_DOMAIN_ID`，当前为 `0`。
- 机器人编号，当前为 `306`。
- 真实工作区绝对路径。当前目录名末尾有空格：
  `/home/ubuntu/ros2_ws `。
- 厂商 `robot_env`、机器人 SDK URDF 和 RGB-D 共享内存是否存在。
- CUDA、PyTorch、Ultralytics、Open3D 与 GPU 是否匹配。

不要凭视觉忽略路径末尾空格。所有 shell 命令都应给工作区路径加引号。

## 3. 修改机器人编号

把源机器人的话题后缀（例如 `0_283`）改成目标机器人后缀（例如
`0_306`）。至少检查：

```text
config/hardware.yaml
config/motion.yaml
config/pbvs.yaml
adaptive_object_grasping_nodes/hardware_probe.py
adaptive_object_grasping_nodes/robot_tf_broadcaster.py
adaptive_object_grasping_nodes/pbvs_servo.py
adaptive_object_grasping_nodes/motion_executor.py
```

可用以下命令检查活动代码和配置，不要误改历史验证文档：

```bash
grep -RIn "0_283" adaptive_object_grasping_nodes config scripts launch \
  --exclude="*.bak*"
```

还要核对 `config/hardware.yaml` 中的厂商 Python 路径和 SDK URDF 路径。

## 4. 编译

```bash
cd "/home/ubuntu/ros2_ws "
source /opt/ros/jazzy/setup.bash

colcon build \
  --base-paths "src/adaptive_object_grasping" \
  --packages-select adaptive_object_grasping \
  --symlink-install
```

如果功能包曾在另一个绝对路径编译，CMake 会报告
`CMakeCache.txt directory is different` 或 source path mismatch。只清理该包
的 CMake 缓存后重建：

```bash
colcon build \
  --base-paths "src/adaptive_object_grasping" \
  --packages-select adaptive_object_grasping \
  --symlink-install \
  --cmake-clean-cache
```

## 5. 单元测试

```bash
cd "/home/ubuntu/ros2_ws "
source /opt/ros/jazzy/setup.bash
source install/adaptive_object_grasping/share/adaptive_object_grasping/local_setup.bash

colcon test \
  --base-paths "src/adaptive_object_grasping" \
  --packages-select adaptive_object_grasping

colcon test-result \
  --test-result-base build/adaptive_object_grasping \
  --verbose
```

306 机器人本次结果为 22 个测试全部通过。

## 6. DDS 环境

启动端和命令端必须使用相同的 Domain、RMW 和 CycloneDDS 配置：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

如果出现：

```text
Failed to find a free participant index for domain 0
```

先确认没有重复启动多套功能，再停止用其他 RMW 启动的 ROS CLI daemon：

```bash
ros2 daemon stop
ros2 node list
```

不要用模糊的 `pkill python` 清理机器人。应通过完整命令行确认 PID，只停止
重复的 launch 及其子进程。

## 7. 启动与只读检查

```bash
cd "/home/ubuntu/ros2_ws "
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch adaptive_object_grasping bringup.launch.py
```

检查 Action、识别目标和硬件门：

```bash
ros2 action list -t | grep pick_object

ros2 service call /list_grasp_objects \
  adaptive_object_grasping/srv/ListObjects '{}'

ros2 service call /check_grasp_hardware \
  std_srvs/srv/Trigger '{}'
```

只有硬件检查返回 `ready: true` 才能继续。相机、关节状态、末端位姿、
相机 TF、左右 TCP TF 都必须为 `ok: true`。

## 8. 安全 dry-run

先用 `execute: false` 验证完整感知和抓取估计链路：

```bash
ros2 action send_goal /pick_object \
  adaptive_object_grasping/action/PickObject \
  "{track_id: 1, label: bottle, preferred_arm: auto, execute: false, maximum_tracking_time: 12.0, required_stable_duration: 0.0}" \
  --feedback
```

把 `track_id` 和 `label` 换成 `/list_grasp_objects` 的实时结果。

当前版本的 `execute: false` 不仅生成 GraspNet 候选，还会调用 MoveItPy/OMPL
运动规划服务
进行 IK/可达性验证，但不会发送硬件运动命令。规划成功后发布：

```text
/adaptive_grasp/target_joint_states
/adaptive_grasp/target_robot_description
```

默认 RViz 配置中的 `IK Target Ghost` RobotModel 会把抓取目标关节状态显示为
半透明机械臂虚影；`Current Robot State` 显示当前真实关节状态。虚影 TF 使用
`grasp_target/` 前缀，与真实机器人 TF 隔离。只有 Action 反馈进入
`planning` 并成功返回 `dry_run` 时，虚影才代表通过 IK/规划验证的目标状态。

306 机器人实测：目标 1（`bottle`）返回 `SUCCEEDED`，选择左臂，候选评分
`0.584`；目标关节话题发布 26 个关节，目标腕部 TF 可持续查询，且没有发送
任何硬件运动命令。

MoveIt 在纯规划模式下可能打印未配置 OctoMap 3D sensor 或 controller manager
的消息；只要 OMPL 规划成功、Action 返回 `dry_run` 且目标关节话题存在，这些
消息不影响虚影验证。它们不能作为允许真机执行的依据。

本次 306 机器人验证已完成以下链路：

```text
RGB-D -> YOLO -> 目标选择 -> GraspNet 推理 -> 候选筛选
```

测试画面最终返回 `no GraspNet candidate passed score filter`。这表示推理调用
已经运行，但当前物体姿态、点云或筛选条件下没有可用候选，不是编译或 Action
通信失败。应先调整物体位置、保证目标完整可见并重新测试，再根据日志谨慎检查
`config/graspnet.yaml` 的目标点数、深度带、碰撞和评分参数。

## 9. 真机执行安全要求

`execute: true` 会真实控制机械臂。执行前必须：

1. 确认只运行一套抓取节点。
2. 确认硬件探针 `ready: true`。
3. 在 RViz 检查抓取和预抓取姿态。
4. 使用轻质软物体、低速度，操作员守在急停旁。
5. 确认候选筛选成功后再允许真实执行。

不要为了“跑通”而直接关闭碰撞过滤或大幅降低评分阈值。

## 10. 迁移验收

- 单包编译成功。
- 22 个单元测试无失败。
- YOLO 和 GraspNet 模型可加载，GPU 可用。
- RGB-D 首帧正常发布。
- `/pick_object` Action 可发现。
- 可列出实时目标。
- 硬件探针所有关键项通过。
- `execute: false` 能进入 GraspNet 推理。
- 无重复 launch、无遗留测试进程。
- 真机执行保持人工安全确认。
