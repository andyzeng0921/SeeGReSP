# Hardware discovery report

Discovery date: 2026-07-16

## Confirmed on the robot

- ROS distribution: ROS 2 Jazzy on x86_64.
- GPU: NVIDIA GeForce RTX 4090 Laptop GPU, 16376 MiB.
- Head RGB-D device: Intel RealSense configured by the Autolife SDK as
  `mod_camera_rgbd_head`.
- RGB-D mode: 640 x 480 at 60 FPS, depth aligned to color.
- Color and depth shared-memory streams are live and use the same frame counter.
- Depth encoding: unsigned 16-bit millimetres (`depth_scale=0.001`).
- Measured color/aligned-depth intrinsics:

  - `fx = 606.8763427734375`
  - `fy = 606.8387451171875`
  - `cx = 329.44561767578125`
  - `cy = 253.8282928466797`
  - distortion coefficients are all zero in the SDK stream

- Shared-memory names:

  - `/camera_image_buffer_rgbd_head_color`
  - `/camera_image_buffer_rgbd_head_depth`
  - `/camera_intrinsics_struct_rgbd_head_color`
  - `/camera_intrinsics_struct_rgbd_head_depth`

- Package-local AI runtime verified on the robot:

  - PyTorch `2.11.0+cu128`, CUDA available on the RTX 4090
  - torchvision `0.26.0+cu128`
  - Ultralytics `8.4.96`
  - Open3D `0.19.0`
  - graspnetAPI `1.2.11` with Python-3.12-compatible dependency overrides
  - YOLO11 segmentation/tracking detected two bottles in a live RGB-D frame,
    both with valid median depth and camera-frame 3D positions

- Robot model: Autolife S1 v2.2. The installed URDF/SRDF defines two 7-DoF
  groups named `Left_Arm` and `Right_Arm`.
- Model base and arm-tip links:

  - `Link_Zero_Point`
  - `Link_Left_Wrist_Lower_to_Gripper`
  - `Link_Right_Wrist_Lower_to_Gripper`

- Vendor ROS interfaces documented/captured for robot suffix `0_283`:

  - feedback: `/topic_arm_whole_body_and_gripper_current_joints_status_0_283`
  - feedback: `/topic_arm_current_robot_eef_pose_0_283`
  - EEF command: `/topic_arm_move_eef_pose_in_robot_frame_0_283`
  - gripper position: `/topic_arm_gripper_target_joints_position_0_283`
  - gripper torque: `/topic_arm_gripper_target_joints_torque_0_283`

- The vendor processes use ROS domain 0, Cyclone DDS, and a loopback-only network
  interface. `camera_and_probe.launch.py` now applies the matching DDS settings
  and raises the automatic participant-index search limit for the robot's dense
  local ROS graph.
- Live joint and gripper feedback was captured. It is a `std_msgs/msg/String`
  JSON object whose arm arrays contain seven joint positions in degrees and whose
  gripper states contain `position`, `speed`, `torque`, and `temperature`.
- Live EEF feedback was captured. Its JSON fields are `left_eef_pose`,
  `right_eef_pose`, and `head_pose`; each EEF pose contains a position in metres
  and an `[x, y, z, w]` quaternion in the robot frame.
- The vendor EEF command is a JSON string with `pos_left_in_robot`,
  `quat_left_in_robot`, `pos_right_in_robot`, and `quat_right_in_robot` fields.
  The gripper position command uses `left_gripper_target_joints_position` and
  `right_gripper_target_joints_position` arrays.
- The configured gripper range is 10 through 360 (open through closed). The
  package uses 10 as open and a conservative 330 as closed.
- Vendor IK, FK, and whole-body RRT planning services are active as
  `autolife_robot_srvs/srv/SetString` services. They provide an initial planning
  route while a complete MoveIt configuration is being prepared.
- The installed URDF locates `Link_Camera_Head_Forehead` at translation
  `[0.0830747060, 0.0019568093, -0.1056439492]` and RPY
  `[0, 1.1868238914, 0.0000819235]` from its neck parent. The SDK additionally
  applies a `[0, -1, 4]` degree head encoder offset and a nominal zero-translation
  optical-axis rotation.

## Missing or not yet safe to infer

- The URDF contains `Link_Camera_Head_Forehead`, which is the likely physical
  mounting link for the RGB-D camera. It and the SDK's nominal axis conversion
  allow a provisional transform to be reconstructed, but neither defines a
  measured RealSense color optical extrinsic. The live TF tree currently contains
  navigation and lidar frames only. The transform to
  `rgbd_head_color_optical_frame` therefore still requires hand-eye calibration
  or a vendor calibration record; the package deliberately does not guess it.
- The URDF tip links are wrist/gripper mounting links, not a calibrated grasp TCP.
  The SRDF has no end-effector entries and the live TF tree has no gripper frames.
  Finger-centre `left_grasp_tcp` and `right_grasp_tcp` frames must be added after
  TCP calibration. Their intentionally absent transforms keep the hardware gate
  locked.
- MoveIt 2 and `moveit_py` were not installed. The SRDF exists, but controller,
  kinematics, joint-state bridge and planning-pipeline configuration are still
  required before selecting `planning_backend=moveit_py`.
- No `ros2_control` controller interface was found. A MoveIt deployment will also
  need adapters between standard joint/trajectory messages and the vendor JSON
  string interfaces.
- The package-local environment contains `graspnetAPI`, but the official
  `graspnet-baseline` source and RealSense checkpoint are not present.
- The official GraspNet baseline license permits noncommercial research use only.
  Its repository and checkpoint are intentionally not copied into this package
  until the user explicitly accepts that license.

## Safety consequence

The default configuration keeps `dry_run=true`,
`allow_hardware_execution=false`, and PBVS hardware follow disabled. The hardware
probe requires fresh RGB-D intrinsics, arm joint feedback, EEF feedback, and all
three base transforms. A failed check keeps the execution adapter locked.

A live read-only probe confirmed that camera info, joint feedback, EEF feedback,
and camera intrinsics all pass. It correctly remains locked because
`rgbd_head_color_optical_frame`, `left_grasp_tcp`, and `right_grasp_tcp` do not yet
have transforms to `Link_Zero_Point`.
