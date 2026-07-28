# adaptive_object_grasping 迁移速查

本文只总结本次将该包迁移到 306 机器人后实际做过的工作。

## 环境

- ROS 2 Jazzy，`ROS_DOMAIN_ID=0`
- 机器人编号：`306`
- 工作区：`/home/ubuntu/ros2_ws `（目录名末尾有空格）
- 包路径：`/home/ubuntu/ros2_ws /src/adaptive_object_grasping`

## 已做修改

- 活动配置和节点默认话题后缀由 `0_283` 改为 `0_306`。
- 删除 `motion_executor_robot_env.sh` 中写死的旧工作区路径。
- 补齐 MoveItPy 和 OMPL 规划依赖，并完成 IK 可达性验证。

## 必装依赖

```bash
sudo apt install ros-jazzy-moveit-py ros-jazzy-moveit-planners-ompl
```

整包迁移时还要保留可能被 Git 忽略的 `models/` 和 `third_party/`。
跨系统、CPU 或 CUDA 版本时应重建包内 Python 虚拟环境。

## 编译

```bash
cd "/home/ubuntu/ros2_ws "
source /opt/ros/jazzy/setup.bash
colcon build --base-paths "src/adaptive_object_grasping" \
  --packages-select adaptive_object_grasping --symlink-install
```

若 CMake 报旧工作区路径不一致，在命令末尾加 `--cmake-clean-cache`。

## 启动和安全测试

```bash
source "/home/ubuntu/ros2_ws /install/setup.bash"
ros2 launch adaptive_object_grasping bringup.launch.py
```

```bash
ros2 action send_goal /pick_object \
  adaptive_object_grasping/action/PickObject \
  "{track_id: 1, label: bottle, preferred_arm: auto, execute: false, maximum_tracking_time: 12.0, required_stable_duration: 0.0}" \
  --feedback
```

`execute:false` 不会驱动机械臂；`execute:true` 会真实控制机械臂，本次未做
真实抓取。

## 本次结果和问题记录

- 编译成功，22 个单元测试全部通过。
- RGB-D、YOLO、GraspNet 和 MoveIt IK 链路均已验证。
- 目标 1 `bottle` dry-run 成功，选择左臂，机械臂未运动。
- DDS participant 耗尽是重复启动两套 launch 导致；只保留一套，并在
  RMW 不一致时执行 `ros2 daemon stop`。
- `~/.bashrc` 的 CycloneDDS 配置使用 `lo`、`ParticipantIndex=auto` 和
  `MaxAutoParticipantIndex=200`，否则节点较多时新 ROS 命令可能无法启动。
- 所有包含工作区的 shell 路径必须加引号，以保留目录名末尾空格。
