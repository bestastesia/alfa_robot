cmake_minimum_required(VERSION 3.8)
project(alfa_robot_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskCommand.msg"
  "msg/TaskStatus.msg"
  "srv/PlanJointTarget.srv"
  "srv/SolveDualIk.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs trajectory_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(ik_solver_lib SHARED src/ik_solver.cpp)
target_include_directories(ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ik_solver_lib PUBLIC
  moveit_core moveit_ros_planning moveit_msgs pluginlib rclcpp geometry_msgs
  ament_index_cpp random_numbers srdfdom urdf nlohmann_json
)

add_library(parallel_updown_aware_ik_solver_lib SHARED src/parallel_updown_aware_ik_solver.cpp)
target_include_directories(parallel_updown_aware_ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parallel_updown_aware_ik_solver_lib PUBLIC
  moveit_core moveit_ros_planning moveit_msgs pluginlib rclcpp geometry_msgs
  ament_index_cpp random_numbers srdfdom urdf nlohmann_json
)
target_link_libraries(parallel_updown_aware_ik_solver_lib PUBLIC ik_solver_lib)

add_executable(fixed_platform_dual_ik_service src/fixed_platform_dual_ik_service.cpp)
target_include_directories(fixed_platform_dual_ik_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fixed_platform_dual_ik_service PUBLIC
  moveit_core moveit_ros_planning moveit_msgs pluginlib rclcpp geometry_msgs sensor_msgs
  std_msgs ament_index_cpp random_numbers srdfdom urdf nlohmann_json
)
target_link_libraries(fixed_platform_dual_ik_service PRIVATE parallel_updown_aware_ik_solver_lib ${cpp_typesupport_target})

add_executable(mock_box_task_publisher src/mock_box_task_publisher.cpp)
ament_target_dependencies(mock_box_task_publisher PUBLIC rclcpp geometry_msgs std_msgs)
target_link_libraries(mock_box_task_publisher PRIVATE ${cpp_typesupport_target})

add_executable(fixed_platform_task_orchestrator src/fixed_platform_task_orchestrator.cpp)
ament_target_dependencies(fixed_platform_task_orchestrator PUBLIC
  rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs nlohmann_json
)
target_link_libraries(fixed_platform_task_orchestrator PRIVATE ${cpp_typesupport_target})

add_executable(temporary_moveit_joint_planner src/temporary_moveit_joint_planner.cpp)
ament_target_dependencies(temporary_moveit_joint_planner PUBLIC
  moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_msgs
  rclcpp geometry_msgs sensor_msgs trajectory_msgs shape_msgs nlohmann_json
)
target_link_libraries(temporary_moveit_joint_planner PRIVATE ${cpp_typesupport_target})

install(TARGETS
  fixed_platform_dual_ik_service
  mock_box_task_publisher
  fixed_platform_task_orchestrator
  temporary_moveit_joint_planner
  ik_solver_lib
  parallel_updown_aware_ik_solver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS
  scripts/fake_plc_cli_task_publisher.py
  scripts/fixed_platform_kdl_ik_service.py
  scripts/monitor_ik_planner_chain.py
  scripts/plc_acceptance_supervisor.py
  scripts/planning_scene_visualizer.py
  scripts/visualize_fixed_platform_task_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
