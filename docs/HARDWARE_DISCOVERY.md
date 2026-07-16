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

## Missing or not yet safe to infer

- The active robot processes did not expose arm feedback or a complete TF tree to
  the isolated SSH ROS CLI during discovery. Topic names are known, but live
  payloads still need to be captured while the arm ROS bridge is discoverable.
- The URDF contains `Link_Camera_Head_Forehead`, which is the likely physical
  mounting link for the RGB-D camera. It does not define the RealSense optical
  frame. The transform from that physical link to
  `rgbd_head_color_optical_frame` must be calibrated or obtained from the vendor;
  the package deliberately does not guess it.
- The URDF tip links are wrist/gripper mounting links, not a calibrated grasp TCP.
  Finger-centre TCP frames must be added after hand-eye/TCP calibration.
- MoveIt 2 and `moveit_py` were not installed. The SRDF exists, but controller,
  kinematics, joint-state bridge and planning-pipeline configuration are still
  required before selecting `planning_backend=moveit_py`.
- The vendor `robot_env` currently has an XPU PyTorch build and must not be
  modified. Install `ultralytics`, `graspnetAPI`, Open3D and the NVIDIA CUDA
  PyTorch build into this package's `third_party/venv` by running
  `tools/install_ai_environment.sh`.
- The official GraspNet baseline license permits noncommercial research use only.
  Its repository and checkpoint are intentionally not copied into this package.

## Safety consequence

The default configuration keeps `dry_run=true`,
`allow_hardware_execution=false`, and PBVS hardware follow disabled. The hardware
probe requires fresh RGB-D intrinsics, arm joint feedback, EEF feedback, and all
three base transforms. A failed check keeps the execution adapter locked.
