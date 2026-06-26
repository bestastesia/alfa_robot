cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
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
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticCargo.msg"
  "msg/SemanticScene.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# dual_arm_planner_node: 双末端位姿规划，走 MoveGroupInterface::setPoseTarget
# 让 bio_ik 通过 setFromIK(poses, tips) 路径统一协调双臂+updown
set(IK_BENCHMARK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/ik_benchmark)

add_library(alfa_robot_motion_core
  src/motion_core/pose_math.cpp
  src/motion_core/scene_geometry.cpp
  src/motion_core/task_geometry.cpp
)

target_compile_features(alfa_robot_motion_core PUBLIC cxx_std_17)

target_include_directories(alfa_robot_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(alfa_robot_motion_core
  geometry_msgs
  nlohmann_json
)

target_link_libraries(alfa_robot_motion_core
  Eigen3::Eigen
)

add_library(alfa_robot_motion_scene_adapter
  src/box_stack_flow_orchestrator.cpp
  src/extract_demo_orchestrator.cpp
  src/extract_planning_pipeline.cpp
  src/loaded_pose_planning.cpp
  src/motion_flow_recorder.cpp
  src/motion_scene_adapter.cpp
  src/optimized_ik_pipeline.cpp
  ${IK_BENCHMARK_ROOT}/src/ik_solver.cpp
  ${IK_BENCHMARK_ROOT}/src/parallel_updown_aware_ik_solver.cpp
)

target_compile_features(alfa_robot_motion_scene_adapter PUBLIC cxx_std_17)

target_include_directories(alfa_robot_motion_scene_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${IK_BENCHMARK_ROOT}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(alfa_robot_motion_scene_adapter
  alfa_robot_motion_core
)

ament_target_dependencies(alfa_robot_motion_scene_adapter
  ament_index_cpp
  geometry_msgs
  kdl_parser
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  nlohmann_json
  orocos_kdl_vendor
  pluginlib
  random_numbers
  rclcpp
  control_msgs
  shape_msgs
  srdfdom
  urdf
)

add_executable(dual_arm_planner_node
  src/dual_arm_planner_node.cpp
)

target_compile_features(dual_arm_planner_node PUBLIC cxx_std_17)

target_include_directories(dual_arm_planner_node PRIVATE
  include
  ${IK_BENCHMARK_ROOT}/include
)

target_link_libraries(dual_arm_planner_node
  alfa_robot_motion_core
  alfa_robot_motion_scene_adapter
)

ament_target_dependencies(dual_arm_planner_node
  rclcpp
  rclcpp_action
  control_msgs
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
  kdl_parser
  orocos_kdl_vendor
)

install(TARGETS
  alfa_robot_motion_core
  alfa_robot_motion_scene_adapter
  dual_arm_planner_node
  EXPORT export_alfa_robot_motion_core
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${IK_BENCHMARK_ROOT}/include/ DESTINATION include)

ament_export_targets(export_alfa_robot_motion_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  ament_index_cpp
  geometry_msgs
  kdl_parser
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  nlohmann_json
  orocos_kdl_vendor
  pluginlib
  random_numbers
  rclcpp
  control_msgs
  shape_msgs
  srdfdom
  urdf
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
  scripts/extract_stage_monitor_console.py
  scripts/extract_sequence_rerun.py
  scripts/extract_failed_attempts_rerun.py
  scripts/execute_l6_r8_mock_live.py
  scripts/execute_l6_r8_real_live.py
  scripts/run_extract_live_benchmark.py
  scripts/visualize_moveit_box_stack_flow.py
  DESTINATION lib/${PROJECT_NAME}
)
