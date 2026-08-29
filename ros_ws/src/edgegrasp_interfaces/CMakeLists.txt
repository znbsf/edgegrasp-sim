cmake_minimum_required(VERSION 3.8)
project(edgegrasp_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedTarget.msg"
  "msg/GraspSequenceTerminal.msg"
  "action/PlanTarget.action"
  "action/ExecuteTrajectory.action"
  "action/GraspSequence.action"
  "action/GraspPhysicsEvidence.action"
  DEPENDENCIES builtin_interfaces geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
