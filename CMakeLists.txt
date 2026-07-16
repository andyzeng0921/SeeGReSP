cmake_minimum_required(VERSION 3.8)
project(adaptive_object_grasping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickObject.action"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "srv/EstimateGrasps.srv"
  "srv/ListObjects.srv"
  "srv/SelectObject.srv"
  DEPENDENCIES action_msgs geometry_msgs sensor_msgs std_msgs
)

ament_python_install_package(${PROJECT_NAME}_nodes)

install(PROGRAMS
  scripts/hardware_probe_node.py
  scripts/rgbd_shm_bridge_robot_env.sh
  scripts/rgbd_shm_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
