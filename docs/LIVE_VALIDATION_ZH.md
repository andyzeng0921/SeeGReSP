# 当前机器人实机接入与验收结果

## 已接通的链路

2026-07-16 已在机器人 283 上完成以下在线验证：

- 头部 RGB-D 共享内存桥接到标准 ROS 2 图像、对齐深度和 `CameraInfo`。
- 基于实时腰部、颈部关节反馈发布 `Link_Zero_Point -> rgbd_head_color_optical_frame` 动态 TF。
- 发布左右腕部动态 TF，以及 `left_grasp_tcp`、`right_grasp_tcp` 静态 TCP。
- 平行夹爪最大指间距离配置为 `0.095 m`。
- YOLO11 在真实画面识别并持续跟踪两个 `bottle`。
- PBVS 读取厂商 `left_eef_pose/right_eef_pose` 反馈并完成稳定性判断。
- GraspNet baseline、PointNet2 CUDA、KNN CUDA、`grasp_nms` 和 RealSense 权重均已安装在功能包目录内。
- 厂商 IK、FK、全身 RRT 服务及关节轨迹消息格式均已接入。
- 真实 `bottle` 完整 dry-run 已依次通过选物、PBVS、GraspNet、候选筛选、预抓取、抓取和抬升三段 IK/FK/RRT。

本次成功候选示例：GraspNet 分数 `0.455`，要求夹爪宽度 `0.0873 m`。三段规划全部成功，且没有发布轨迹或夹爪命令。

## 安装 GraspNet

仅限接受官方非商业研究许可证后执行：

```bash
cd /home/ubuntu/ros2_ws/src/adaptive_object_grasping
bash tools/install_ai_environment.sh
GRASPNET_LICENSE_ACCEPTED=1 CUDA_HOME=/usr/local/cuda-13.0 \
  bash tools/install_graspnet.sh
```

脚本固定官方 baseline 提交 `280c215129f759ed8649cb4e89fc5dfee55f4f80`，并校验权重 SHA256：

```text
60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
```

安装成功时会真实加载网络并在 GPU 上完成一次点云推理，输出 `output=(5, 17)`。

## 编译与启动

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select adaptive_object_grasping --symlink-install
source install/setup.bash
ros2 launch adaptive_object_grasping bringup.launch.py
```

机器人现有终端需要保持厂商的 Domain 0、CycloneDDS loopback 配置；正常交互式终端已在 `~/.bashrc` 配置。

## 列出物体并执行无运动规划

```bash
ros2 service call /list_grasp_objects \
  adaptive_object_grasping/srv/ListObjects '{}'

ros2 action send_goal /pick_object \
  adaptive_object_grasping/action/PickObject \
  "{track_id: -1, label: bottle, preferred_arm: auto, execute: false,
    maximum_tracking_time: 15.0, required_stable_duration: 0.3}" --feedback
```

成功结果应包含：

```text
success: true
vendor IK/FK/RRT validated pregrasp, grasp and lift; no hardware command sent
```

协调器会按 GraspNet 分数依次尝试候选，自动跳过 FK 或 RRT 不可达的姿态。GraspNet 在碰撞过滤无结果时最多重新采样三次。

## 实体运动前必须完成

当前默认配置仍为：

```yaml
dry_run: true
allow_hardware_execution: false
enable_pbvs_hardware_follow: false
```

因此当前可以稳定完成感知和完整规划，但不会让机械臂运动。首次实体抓取前仍必须：

1. 用标定板或已知尺寸工具复核相机光学外参。
2. 用实体夹爪复核左右 TCP 是否位于两指实际夹持区中心。
3. 在 RViz 或厂商可视化中检查规划轨迹不穿过机器人与环境。
4. 使用轻质软物体、最低速度、单次动作，并安排人员在急停旁。
5. 完成上述检查后才将 `dry_run` 改为 `false`、`allow_hardware_execution` 改为 `true`。

当前 `position_tolerance=0.04 m` 是针对名义 TCP 和厂商 IK 精度的实测规划验收值。完成精确 TCP 与手眼标定后，应重新测试并尽量收紧该阈值。
