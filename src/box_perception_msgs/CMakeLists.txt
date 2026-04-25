cmake_minimum_required(VERSION 3.8)
project(box_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoxResult.msg"
  "msg/BoxPerceptionResult.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()
