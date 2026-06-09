cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticCargo.msg"
  "msg/SemanticScene.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# dual_arm_planner_node: 双末端位姿规划，走 MoveGroupInterface::setPoseTarget
# 让 bio_ik 通过 setFromIK(poses, tips) 路径统一协调双臂+updown
set(IK_BENCHMARK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/ik_benchmark)

add_executable(dual_arm_planner_node
  src/dual_arm_planner_node.cpp
  ${IK_BENCHMARK_ROOT}/src/ik_solver.cpp
  ${IK_BENCHMARK_ROOT}/src/parallel_updown_aware_ik_solver.cpp
)

target_compile_features(dual_arm_planner_node PUBLIC cxx_std_17)

target_include_directories(dual_arm_planner_node PRIVATE
  ${IK_BENCHMARK_ROOT}/include
)

ament_target_dependencies(dual_arm_planner_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  shape_msgs
  pluginlib
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
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
  scripts/set_joints.py
  scripts/mujoco_joint_demo_commander.py
  scripts/mujoco_digital_twin.py
  scripts/mujoco_sync_bridge.py
  scripts/mujoco_planning_scene_bridge.py
  scripts/semantic_scene_to_planning_scene.py
  scripts/semantic_scene_utils.py
  scripts/visualize_moveit_box_stack_flow.py
  DESTINATION lib/${PROJECT_NAME}
)
