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
find_package(pick_ik REQUIRED)
find_package(bio_ik REQUIRED)

add_executable(ik_benchmark src/ik_benchmark.cpp)

target_include_directories(ik_benchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament_target_dependencies handles moveit_core's individual component targets
# and all transitive dependencies correctly — do NOT use target_link_libraries
# with moveit_core::moveit_core (that target does not exist).
ament_target_dependencies(ik_benchmark PUBLIC
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp
  geometry_msgs
  ament_index_cpp
  random_numbers
  pick_ik
  bio_ik
)

install(TARGETS ik_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
