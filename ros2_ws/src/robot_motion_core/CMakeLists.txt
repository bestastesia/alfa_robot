cmake_minimum_required(VERSION 3.22)
project(robot_motion_core)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(robot_motion_core
  src/box_pose_extract_rrt.cpp
)
target_compile_features(robot_motion_core PUBLIC cxx_std_17)
target_include_directories(robot_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_motion_core PUBLIC Eigen3::Eigen)

add_library(robot_motion_core::robot_motion_core ALIAS robot_motion_core)

install(TARGETS robot_motion_core
  EXPORT export_robot_motion_core
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_robot_motion_core)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  add_executable(test_ik_candidate_types test/test_ik_candidate_types.cpp)
  target_link_libraries(test_ik_candidate_types robot_motion_core)
  add_test(NAME test_ik_candidate_types COMMAND test_ik_candidate_types)

  add_executable(test_box_pose_extract_rrt test/test_box_pose_extract_rrt.cpp)
  target_link_libraries(test_box_pose_extract_rrt robot_motion_core)
  add_test(NAME test_box_pose_extract_rrt COMMAND test_box_pose_extract_rrt)
endif()

ament_package()
