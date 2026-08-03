cmake_minimum_required(VERSION 3.8)
project(adaptive_object_grasping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickObject.action"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/SelectedObjectObservation.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "srv/EstimateGrasps.srv"
  "srv/ExecuteCandidate.srv"
  "srv/ListObjects.srv"
  "srv/SelectObject.srv"
  DEPENDENCIES action_msgs geometry_msgs sensor_msgs std_msgs
)

ament_python_install_package(${PROJECT_NAME}_nodes)

install(PROGRAMS
  scripts/hardware_probe_node.py
  scripts/graspnet_server_node.py
  scripts/graspnet_server_venv.sh
  scripts/motion_executor_node.py
  scripts/motion_executor_robot_env.sh
  scripts/grasp_coordinator_node.py
  scripts/rgbd_shm_bridge_robot_env.sh
  scripts/rgbd_shm_bridge_node.py
  scripts/robot_tf_broadcaster_node.py
  scripts/pbvs_servo_node.py
  scripts/yolo_tracker_node.py
  scripts/yolo_tracker_venv.sh
  scripts/grasp_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/install_ai_environment.sh
  tools/install_graspnet.sh
  tools/probe_hardware.sh
  tools/check_dual_arm_profile.py
  tools/check_execution_readiness.py
  tools/calibrate_environment.py
  tools/calibrate_visible_reset.py
  tools/install_moveit_overlay.sh
  DESTINATION share/${PROJECT_NAME}/tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(perception_core_test test/test_perception_core.py)
  ament_add_pytest_test(perception_sync_core_test test/test_perception_sync_core.py)
  ament_add_pytest_test(observation_store_core_test test/test_observation_store_core.py)
  ament_add_pytest_test(target_validation_core_test test/test_target_validation_core.py)
  ament_add_pytest_test(pbvs_core_test test/test_pbvs_core.py)
  ament_add_pytest_test(graspnet_core_test test/test_graspnet_core.py)
  ament_add_pytest_test(motion_core_test test/test_motion_core.py)
  ament_add_pytest_test(robot_geometry_core_test test/test_robot_geometry_core.py)
  ament_add_pytest_test(dual_arm_core_test test/test_dual_arm_core.py)
  ament_add_pytest_test(hardware_health_core_test test/test_hardware_health_core.py)
  ament_add_pytest_test(safety_core_test test/test_safety_core.py)
  ament_add_pytest_test(robot_profile_test test/test_robot_profile.py)
  ament_add_pytest_test(environment_scene_core_test test/test_environment_scene_core.py)
  ament_add_pytest_test(moveit_config_test test/test_moveit_config.py)
  ament_add_pytest_test(visible_reset_core_test test/test_visible_reset_core.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
