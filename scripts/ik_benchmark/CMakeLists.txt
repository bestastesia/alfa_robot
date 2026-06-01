cmake_minimum_required(VERSION 3.8)
project(alfa_robot_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(nlohmann_json REQUIRED)

# ── IkSolver library ───────────────────────────────────────────────────────

add_library(ik_solver_lib SHARED
  src/ik_solver.cpp
)

target_include_directories(ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_solver_lib PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

# ── ParallelUpdownAwareIkSolver library ───────────────────────────────────

add_library(parallel_updown_aware_ik_solver_lib SHARED
  src/parallel_updown_aware_ik_solver.cpp
)

target_include_directories(parallel_updown_aware_ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(parallel_updown_aware_ik_solver_lib PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(parallel_updown_aware_ik_solver_lib PUBLIC ik_solver_lib)

# ── ik_benchmark executable ────────────────────────────────────────────────

add_executable(ik_benchmark src/benchmark_main.cpp)

target_include_directories(ik_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_benchmark PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(ik_benchmark PRIVATE ik_solver_lib)

# ── ik_demo executable ─────────────────────────────────────────────────────

add_executable(ik_demo src/demo_single.cpp)

target_include_directories(ik_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_demo PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(ik_demo PRIVATE ik_solver_lib)

# ── ik_range_grid executable ──────────────────────────────────────────────

add_executable(ik_range_grid src/range_grid_main.cpp)

target_include_directories(ik_range_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_range_grid PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(ik_range_grid PRIVATE ik_solver_lib)

# ── pick_place_baseline executable ────────────────────────────────────────

add_executable(pick_place_baseline src/pick_place_baseline_main.cpp)

target_include_directories(pick_place_baseline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pick_place_baseline PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(pick_place_baseline PRIVATE ik_solver_lib)

# ── pick_place_updown_lookup executable ───────────────────────────────────

add_executable(pick_place_updown_lookup src/pick_place_updown_lookup_main.cpp)

target_include_directories(pick_place_updown_lookup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pick_place_updown_lookup PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(pick_place_updown_lookup PRIVATE ik_solver_lib)

# ── parallel_ik_benchmark executable ─────────────────────────────────────

add_executable(parallel_ik_benchmark src/parallel_ik_benchmark_main.cpp)

target_include_directories(parallel_ik_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(parallel_ik_benchmark PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(parallel_ik_benchmark PRIVATE ik_solver_lib)

# ── box_stack_kdl_benchmark executable ────────────────────────────────────

add_executable(box_stack_kdl_benchmark src/box_stack_kdl_benchmark_main.cpp)

target_include_directories(box_stack_kdl_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(box_stack_kdl_benchmark PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(box_stack_kdl_benchmark PRIVATE ik_solver_lib)

# ── parallel_updown_ik_demo executable ────────────────────────────────────

add_executable(parallel_updown_ik_demo src/parallel_updown_ik_demo_main.cpp)

target_include_directories(parallel_updown_ik_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(parallel_updown_ik_demo PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(parallel_updown_ik_demo PRIVATE parallel_updown_aware_ik_solver_lib)

# ── updown_solver_comparison executable ───────────────────────────────────

add_executable(updown_solver_comparison src/updown_solver_comparison_benchmark.cpp)

target_include_directories(updown_solver_comparison PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(updown_solver_comparison PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  srdfdom
  urdf
  nlohmann_json
)

target_link_libraries(updown_solver_comparison PRIVATE parallel_updown_aware_ik_solver_lib)

# ── collision_state_monitor executable ────────────────────────────────────

add_executable(collision_state_monitor src/collision_state_monitor.cpp)

target_include_directories(collision_state_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(collision_state_monitor PUBLIC
  moveit_core
  moveit_ros_planning
  moveit_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
  ament_index_cpp
  srdfdom
  urdf
)

# ── Install ────────────────────────────────────────────────────────────────

install(TARGETS ik_benchmark ik_demo ik_range_grid pick_place_baseline pick_place_updown_lookup parallel_ik_benchmark box_stack_kdl_benchmark parallel_updown_ik_demo updown_solver_comparison collision_state_monitor ik_solver_lib parallel_updown_aware_ik_solver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/benchmark.py
  scripts/demo.py
  scripts/ik_range_grid.py
  scripts/run_parallel_ik_speed_limit_benchmark.py
  scripts/plot_parallel_ik_speed_limit.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
