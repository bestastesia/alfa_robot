cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# dual_arm_planner_node: 双末端位姿规划，走 MoveGroupInterface::setPoseTarget
# 让 bio_ik 通过 setFromIK(poses, tips) 路径统一协调双臂+updown
add_executable(dual_arm_planner_node src/dual_arm_planner_node.cpp)
ament_target_dependencies(dual_arm_planner_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
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

install(PROGRAMS
  scripts/reachability_tester.py
  scripts/nine_orient_reachability.py
  scripts/x_edge_refine_reachability.py
  scripts/rviz_dual_goal_pose_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
