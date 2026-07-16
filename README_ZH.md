# 自适应任意物体抓取功能包

包名：`adaptive_object_grasping`

该包将高速 YOLO 多目标识别与跟踪、用户目标选择、受限距离 PBVS 跟随、
目标稳定性判断、GraspNet 6D 抓取位姿估计，以及 MoveIt 2/机器人厂商执行
接口串成一个可取消的 ROS 2 抓取 Action。

## 当前实现边界

- YOLO 只能识别模型训练类别。默认 `yolo11n-seg.pt` 识别 COCO 类别；需要
  新类别时应换成自训练分割权重或兼容的开放词汇 YOLO 模型。
- PBVS 当前使用头部 RGB-D 的目标三维位置，保持夹爪与目标的初始相对偏移，
  主要增强目标被短距离挪动时的观赏性和鲁棒性。它不是无边界追逐算法。
- GraspNet 使用用户另行安装的官方 baseline 和 RealSense 权重。本包不复制
  其受限许可证源码或模型。
- MoveIt 是推荐执行后端；当前机器人还没有完整 MoveIt 配置。厂商末端位姿
  后端可用于后续联调，但默认被硬件锁禁止。

## 已查到的真机参数

详细报告见 `docs/HARDWARE_DISCOVERY.md`。当前已实测头部 RealSense 共享内存
为 640 x 480、60 FPS、RGB/深度对齐，内参为：

```text
fx=606.8763427734375  fy=606.8387451171875
cx=329.44561767578125 cy=253.8282928466797
depth_scale=0.001
```

桥接后的标准话题：

```text
/head_camera/color/image_raw
/head_camera/aligned_depth_to_color/image_raw
/head_camera/color/camera_info
```

2026-07-16 实机验证结果：包内 PyTorch 2.11.0+cu128 已识别 RTX 4090，
Ultralytics 8.4.96 与 Open3D 0.19.0 可正常导入；真实头部画面中 YOLO11 分割跟踪
识别到两只 `bottle`，跟踪 ID 为 1、2，深度约为 0.56 m、0.54 m。

## 编译与测试

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/adaptive_object_grasping \
  --packages-select adaptive_object_grasping --symlink-install
source install/setup.bash
colcon test --base-paths src/adaptive_object_grasping \
  --packages-select adaptive_object_grasping
colcon test-result --test-result-base build/adaptive_object_grasping --verbose
```

## 安装独立 AI 环境

不要往厂商 `robot_env` 中安装 YOLO/GraspNet，它承载机器人基础服务。执行：

```bash
bash /home/ubuntu/ros2_ws/src/adaptive_object_grasping/tools/install_ai_environment.sh
```

安装内容统一保存在功能包内部：`third_party/venv`、`models/yolo`、
`third_party/graspnet-baseline` 和 `models/graspnet`。复制整个功能包后重新编译即可迁移；
不同 CPU、系统或 CUDA 版本的机器人应在目标机器上重新运行安装脚本。详细目录说明见
`docs/PORTABLE_RUNTIME.md`。

然后按脚本末尾提示，在接受官方非商业研究许可证后，单独安装
`graspnet-baseline`、编译 PointNet2 扩展并放置 checkpoint。

## 安全启动

完整启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch adaptive_object_grasping bringup.launch.py
```

默认不会控制机械臂。先检查硬件门：

```bash
ros2 service call /check_grasp_hardware std_srvs/srv/Trigger '{}'
```

只有响应中 `ready=true` 才说明相机、关节反馈、末端反馈和关键 TF 都在线。

## 查看和选择物体

```bash
ros2 service call /list_grasp_objects \
  adaptive_object_grasping/srv/ListObjects '{}'
```

按跟踪编号选择：

```bash
ros2 service call /select_grasp_object \
  adaptive_object_grasping/srv/SelectObject \
  "{track_id: 3, label: '', preferred_arm: auto}"
```

也可以将 `track_id` 设为 `-1`，按类别名称选择当前画面中置信度最高的目标。

## 完整 dry-run

```bash
ros2 action send_goal /pick_object \
  adaptive_object_grasping/action/PickObject \
  "{track_id: 3, label: '', preferred_arm: auto, execute: false,
    maximum_tracking_time: 12.0, required_stable_duration: 0.0}" --feedback
```

状态依次为：`selecting -> tracking -> stable -> estimating -> dry_run -> completed`。

## 真机执行解锁顺序

1. 标定并发布 `Link_Zero_Point -> rgbd_head_color_optical_frame`。
2. 建立左右真实夹爪 TCP 帧，不直接把腕部安装链接当作指尖中心。
3. 让关节状态和末端位姿反馈持续在线，执行 `tools/probe_hardware.sh`。
4. 在 RViz 检查 GraspNet 候选、预抓取位姿和接近方向。
5. 优先完成 MoveIt 2 的 URDF/SRDF、kinematics、controllers、joint limits 和
   planning scene 配置。
6. 首次真机只用轻质软物体、低速度、单次尝试，并在急停旁操作。
7. 最后才修改 `config/motion.yaml`：

```yaml
dry_run: false
allow_hardware_execution: true
```

PBVS 真实机械臂跟随是独立高风险开关，初次抓取仍应保持：

```yaml
enable_pbvs_hardware_follow: false
```

## 参考实现

- Ultralytics tracking: https://docs.ultralytics.com/modes/track/
- Official GraspNet baseline: https://github.com/graspnet/graspnet-baseline
- MoveIt realtime servo: https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
- MoveIt Python planning API: https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
