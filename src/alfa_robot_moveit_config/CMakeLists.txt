cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Dual arm planner node
add_executable(dual_arm_planner_node src/dual_arm_planner_node.cpp)
ament_target_dependencies(dual_arm_planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  std_srvs
)
install(TARGETS dual_arm_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})