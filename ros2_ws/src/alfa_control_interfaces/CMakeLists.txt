cmake_minimum_required(VERSION 3.8)
project(alfa_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(alfa_system_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VacuumChannelResult.msg"
  "msg/VacuumChannelSample.msg"
  "action/VacuumGrip.action"
  DEPENDENCIES alfa_system_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
