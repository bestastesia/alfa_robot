cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
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
find_package(interactive_markers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_motion_scene_service REQUIRED)
find_package(robot_motion_interfaces REQUIRED)
find_package(robot_motion_core REQUIRED)
find_package(alfa_robot_analytic_ik REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfigureExtractMonitor.srv"
  "srv/PlanWallBoxDemo.srv"
  DEPENDENCIES geometry_msgs robot_motion_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# dual_arm_planner_node: 双末端全流程复现；抓取/抽离 IK 走当前三平行结构解析解。

add_library(alfa_robot_motion_core
  src/motion_core/pose_math.cpp
)

target_compile_features(alfa_robot_motion_core PUBLIC cxx_std_17)

target_include_directories(alfa_robot_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(alfa_robot_motion_core
  geometry_msgs
  nlohmann_json
  robot_motion_scene_service
)

target_link_libraries(alfa_robot_motion_core
  Eigen3::Eigen
  robot_motion_scene_service::robot_motion_scene_core
)

add_library(alfa_robot_motion_scene_adapter
  src/box_stack_flow_orchestrator.cpp
  src/box_pose_rrt_extract_planner.cpp
  src/execution_trajectory_adapter.cpp
  src/extract_monitor_json.cpp
  src/extract_monitor_replay_builder.cpp
  src/extract_monitor_snapshot_writer.cpp
  src/extract_monitor_state.cpp
  src/extract_monitor_transition_planning.cpp
  src/extract_demo_orchestrator.cpp
  src/extract_planning_pipeline.cpp
  src/loaded_pose_planning.cpp
  src/motion_flow_recorder.cpp
  src/optimized_ik_pipeline.cpp
  src/planning_diagnostics.cpp
  src/trajectory_plan_utils.cpp
)

target_compile_features(alfa_robot_motion_scene_adapter PUBLIC cxx_std_17)

target_include_directories(alfa_robot_motion_scene_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(alfa_robot_motion_scene_adapter
  alfa_robot_motion_core
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
  robot_motion_scene_service::robot_motion_scene_core
  robot_motion_scene_service::robot_motion_scene_adapter
  robot_motion_core::robot_motion_core
)

ament_target_dependencies(alfa_robot_motion_scene_adapter
  ament_index_cpp
  alfa_robot_analytic_ik
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  nlohmann_json
  pluginlib
  random_numbers
  rclcpp
  control_msgs
  trajectory_msgs
  shape_msgs
  robot_motion_scene_service
  robot_motion_core
  srdfdom
  urdf
)

add_executable(dual_arm_planner_node
  src/dual_arm_planner_node.cpp
)

target_compile_features(dual_arm_planner_node PUBLIC cxx_std_17)

target_include_directories(dual_arm_planner_node PRIVATE
  include
)

target_link_libraries(dual_arm_planner_node
  alfa_robot_motion_core
  alfa_robot_motion_scene_adapter
  robot_motion_scene_service::robot_motion_scene_core
  robot_motion_scene_service::robot_motion_scene_adapter
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dual_arm_planner_node "${cpp_typesupport_target}")

ament_target_dependencies(dual_arm_planner_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
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
  alfa_robot_analytic_ik
  robot_motion_interfaces
  robot_motion_core
)

add_executable(analytic_arm_ik_service_node
  src/analytic_arm_ik_service_node.cpp
)

target_compile_features(analytic_arm_ik_service_node PUBLIC cxx_std_17)

target_include_directories(analytic_arm_ik_service_node PRIVATE
  include
)

target_link_libraries(analytic_arm_ik_service_node
  alfa_robot_motion_core
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
)

ament_target_dependencies(analytic_arm_ik_service_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  robot_motion_interfaces
  alfa_robot_analytic_ik
)

add_executable(motion_collision_service_node
  src/motion_collision_service_node.cpp
)

target_compile_features(motion_collision_service_node PUBLIC cxx_std_17)

target_include_directories(motion_collision_service_node PRIVATE
  include
)

target_link_libraries(motion_collision_service_node
  alfa_robot_motion_core
  alfa_robot_motion_scene_adapter
)

ament_target_dependencies(motion_collision_service_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_msgs
  robot_motion_interfaces
)

add_executable(v3_continuous_reachability
  src/v3_continuous_reachability.cpp
)

target_compile_features(v3_continuous_reachability PUBLIC cxx_std_17)

target_link_libraries(v3_continuous_reachability
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
)

ament_target_dependencies(v3_continuous_reachability
  rclcpp
  moveit_core
  moveit_ros_planning
  nlohmann_json
  alfa_robot_analytic_ik
)

add_executable(v3_redundant_ik_interactive_demo
  src/v3_redundant_ik_interactive_demo.cpp
)

target_compile_features(v3_redundant_ik_interactive_demo PUBLIC cxx_std_17)

target_link_libraries(v3_redundant_ik_interactive_demo
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
)

ament_target_dependencies(v3_redundant_ik_interactive_demo
  alfa_robot_analytic_ik
  geometry_msgs
  interactive_markers
  nlohmann_json
  rclcpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
)

add_executable(v3_single_arm_box_extract_demo
  src/v3_single_arm_box_extract_demo.cpp
)

target_compile_features(v3_single_arm_box_extract_demo PUBLIC cxx_std_17)

target_include_directories(v3_single_arm_box_extract_demo PRIVATE
  ${OMPL_INCLUDE_DIRS}
  include
)

target_link_libraries(v3_single_arm_box_extract_demo
  ${OMPL_LIBRARIES}
  "${cpp_typesupport_target}"
  alfa_robot_motion_scene_adapter
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
)

ament_target_dependencies(v3_single_arm_box_extract_demo
  alfa_robot_analytic_ik
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  nlohmann_json
  pluginlib
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

add_executable(v3_dual_arm_cartesian_box_demo
  src/v3_dual_arm_cartesian_box_demo.cpp
)

target_compile_features(v3_dual_arm_cartesian_box_demo PUBLIC cxx_std_17)

target_include_directories(v3_dual_arm_cartesian_box_demo PRIVATE
  include
)

target_link_libraries(v3_dual_arm_cartesian_box_demo
  alfa_robot_motion_scene_adapter
  alfa_robot_analytic_ik::alfa_robot_analytic_ik
)

ament_target_dependencies(v3_dual_arm_cartesian_box_demo
  alfa_robot_analytic_ik
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_ros_planning
  nlohmann_json
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

install(TARGETS
  alfa_robot_motion_core
  alfa_robot_motion_scene_adapter
  dual_arm_planner_node
  analytic_arm_ik_service_node
  motion_collision_service_node
  v3_continuous_reachability
  v3_redundant_ik_interactive_demo
  v3_single_arm_box_extract_demo
  v3_dual_arm_cartesian_box_demo
  EXPORT export_alfa_robot_motion_core
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_alfa_robot_motion_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(robot_motion_core)
ament_export_dependencies(
  Eigen3
  alfa_robot_analytic_ik
  ament_index_cpp
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  nlohmann_json
  pluginlib
  random_numbers
  rclcpp
  control_msgs
  trajectory_msgs
  shape_msgs
  robot_motion_scene_service
  robot_motion_interfaces
  srdfdom
  urdf
)

if(BUILD_TESTING)
  enable_testing()
  add_test(
    NAME check_l6_r8_real_safety
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/safety/check_l6_r8_real_safety.py
  )

  add_executable(test_execution_trajectory_adapter test/test_execution_trajectory_adapter.cpp)
  target_link_libraries(test_execution_trajectory_adapter alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_execution_trajectory_adapter control_msgs trajectory_msgs)
  add_test(NAME test_execution_trajectory_adapter COMMAND test_execution_trajectory_adapter)

  add_executable(test_pose_math test/test_pose_math.cpp)
  target_link_libraries(test_pose_math alfa_robot_motion_scene_adapter)
  add_test(NAME test_pose_math COMMAND test_pose_math)

  add_executable(test_trajectory_plan_utils test/test_trajectory_plan_utils.cpp)
  target_link_libraries(test_trajectory_plan_utils alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_trajectory_plan_utils srdfdom urdf)
  add_test(NAME test_trajectory_plan_utils COMMAND test_trajectory_plan_utils)

  add_executable(test_motion_flow_recorder test/test_motion_flow_recorder.cpp)
  target_link_libraries(test_motion_flow_recorder alfa_robot_motion_scene_adapter)
  add_test(NAME test_motion_flow_recorder COMMAND test_motion_flow_recorder)

  add_executable(test_extract_monitor_snapshot_writer test/test_extract_monitor_snapshot_writer.cpp)
  target_link_libraries(test_extract_monitor_snapshot_writer alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_extract_monitor_snapshot_writer nlohmann_json)
  add_test(NAME test_extract_monitor_snapshot_writer COMMAND test_extract_monitor_snapshot_writer)

  add_executable(test_extract_monitor_json test/test_extract_monitor_json.cpp)
  target_link_libraries(test_extract_monitor_json alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_extract_monitor_json nlohmann_json srdfdom urdf)
  add_test(NAME test_extract_monitor_json COMMAND test_extract_monitor_json)

  add_executable(test_extract_monitor_replay_builder test/test_extract_monitor_replay_builder.cpp)
  target_link_libraries(test_extract_monitor_replay_builder alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_extract_monitor_replay_builder nlohmann_json srdfdom urdf)
  add_test(NAME test_extract_monitor_replay_builder COMMAND test_extract_monitor_replay_builder)

  add_executable(test_extract_monitor_state test/test_extract_monitor_state.cpp)
  target_link_libraries(test_extract_monitor_state alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_extract_monitor_state srdfdom urdf)
  add_test(NAME test_extract_monitor_state COMMAND test_extract_monitor_state)

  add_executable(test_extract_monitor_transition_planning test/test_extract_monitor_transition_planning.cpp)
  target_link_libraries(test_extract_monitor_transition_planning alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_extract_monitor_transition_planning srdfdom urdf)
  add_test(NAME test_extract_monitor_transition_planning COMMAND test_extract_monitor_transition_planning)

  add_executable(test_ik_candidate_selector test/test_ik_candidate_selector.cpp)
  target_link_libraries(test_ik_candidate_selector alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_ik_candidate_selector srdfdom urdf)
  add_test(NAME test_ik_candidate_selector COMMAND test_ik_candidate_selector)

  add_executable(test_analytic_moveit_fk test/test_analytic_moveit_fk.cpp)
  target_link_libraries(test_analytic_moveit_fk alfa_robot_analytic_ik::alfa_robot_analytic_ik)
  ament_target_dependencies(test_analytic_moveit_fk alfa_robot_analytic_ik moveit_core srdfdom urdf)
  add_test(NAME test_analytic_moveit_fk COMMAND test_analytic_moveit_fk)

  add_executable(test_v311_analytic_moveit_fk test/test_v311_analytic_moveit_fk.cpp)
  target_link_libraries(test_v311_analytic_moveit_fk alfa_robot_analytic_ik::alfa_robot_analytic_ik)
  ament_target_dependencies(
    test_v311_analytic_moveit_fk
    alfa_robot_analytic_ik
    ament_index_cpp
    moveit_core
    srdfdom
    urdf
  )
  add_test(NAME test_v311_analytic_moveit_fk COMMAND test_v311_analytic_moveit_fk)

  add_executable(validate_v3_wall_replay test/validate_v3_wall_replay.cpp)
  ament_target_dependencies(validate_v3_wall_replay moveit_core moveit_msgs moveit_ros_planning nlohmann_json srdfdom urdf)

  add_executable(test_demo_failure test/test_demo_failure.cpp)
  target_link_libraries(test_demo_failure alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_demo_failure visualization_msgs)
  target_compile_options(test_demo_failure PRIVATE -UNDEBUG)
  add_test(NAME test_demo_failure COMMAND test_demo_failure)

  add_executable(test_wall_sequence test/test_wall_sequence.cpp)
  target_compile_features(test_wall_sequence PRIVATE cxx_std_17)
  target_include_directories(test_wall_sequence PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  target_compile_options(test_wall_sequence PRIVATE -UNDEBUG)
  add_test(NAME test_wall_sequence COMMAND test_wall_sequence)

  add_executable(test_comfort_height test/test_comfort_height.cpp)
  target_compile_features(test_comfort_height PRIVATE cxx_std_17)
  target_include_directories(test_comfort_height PRIVATE include)
  target_compile_options(test_comfort_height PRIVATE -UNDEBUG)
  add_test(NAME test_comfort_height COMMAND test_comfort_height)

  add_executable(test_loaded_pose_selector test/test_loaded_pose_selector.cpp)
  target_link_libraries(test_loaded_pose_selector alfa_robot_motion_scene_adapter)
  ament_target_dependencies(test_loaded_pose_selector srdfdom urdf)
  add_test(NAME test_loaded_pose_selector COMMAND test_loaded_pose_selector)

  add_test(
    NAME test_process_lifecycle
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_process_lifecycle.py
  )

  add_test(
    NAME test_live_execution_phase_split
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_live_execution_phase_split.py
  )

  add_test(
    NAME test_extract_sequence_definition
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_extract_sequence_definition.py
  )

  add_test(
    NAME test_package_boundaries
    COMMAND /usr/bin/python3
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_package_boundaries.py
      ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  )

  add_test(
    NAME test_v311_demo_contract
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_v311_demo_contract.py
  )
endif()

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
  scripts/process_lifecycle.py
  scripts/extract_startup_stability_smoke.py
  scripts/extract_stage_monitor_console.py
  scripts/extract_sequence_rerun.py
  scripts/verify_extract_sequence_snapshot.py
  scripts/extract_failed_attempts_rerun.py
  scripts/execute_l6_r8_mock_live.py
  scripts/execute_l6_r8_real_live.py
  scripts/run_extract_live_benchmark.py
  scripts/robot_motion_state_source.py
  scripts/motion_contracts/check_joint_contract.py
  scripts/motion_contracts/generate_motion_baseline.py
  scripts/visualize_moveit_box_stack_flow.py
  scripts/visualize_v3_continuous_reachability.py
  scripts/visualize_v3_reachability_scan_volume.py
  DESTINATION lib/${PROJECT_NAME}
)
