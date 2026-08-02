cmake_minimum_required(VERSION 3.8)
project(alfa_task_interfaces)

find_package(ament_cmake REQUIRED)
find_package(alfa_system_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoxGeometry.msg"
  "msg/PickOutcome.msg"
  "msg/PickTarget.msg"
  "msg/PlaceOutcome.msg"
  "msg/PlaceTarget.msg"
  "action/PlanAndExecutePick.action"
  "action/PlanAndExecutePlace.action"
  DEPENDENCIES alfa_system_interfaces builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
