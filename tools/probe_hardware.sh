#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

echo "=== Standard grasp topics ==="
ros2 topic list -t | grep -E 'head_camera|topic_arm_.*(joint|eef|gripper)|/tf' || true

echo "=== RGB-D rates ==="
timeout 6 ros2 topic hz /head_camera/color/image_raw || true
timeout 6 ros2 topic hz /head_camera/aligned_depth_to_color/image_raw || true

echo "=== Camera intrinsics ==="
timeout 5 ros2 topic echo /head_camera/color/camera_info \
  --qos-profile sensor_data --once --timeout 4 || true

echo "=== Required transforms ==="
for target in rgbd_head_color_optical_frame \
  Link_Left_Wrist_Lower_to_Gripper Link_Right_Wrist_Lower_to_Gripper; do
  echo "Link_Zero_Point <- $target"
  timeout 3 ros2 run tf2_ros tf2_echo Link_Zero_Point "$target" || true
done

echo "=== Package hardware gate ==="
ros2 service call /check_grasp_hardware std_srvs/srv/Trigger '{}'
