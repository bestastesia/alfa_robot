cmake_minimum_required(VERSION 3.8)
project(alfa_robot_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
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

# ── ik_benchmark executable ────────────────────────────────────────────────

add_executable(ik_benchmark src/benchmark_main.cpp)

target_include_directories(ik_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_benchmark PUBLIC
  moveit_core
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

# ── Install ────────────────────────────────────────────────────────────────

install(TARGETS ik_benchmark ik_demo ik_range_grid ik_solver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/benchmark.py
  scripts/demo.py
  scripts/ik_range_grid.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
