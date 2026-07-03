cmake_minimum_required(VERSION 3.22)
project(robot_motion_scene_service)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(robot_motion_scene_core
  src/motion_core/scene_pose.cpp
  src/motion_core/scene_geometry.cpp
  src/motion_core/task_geometry.cpp
)

target_compile_features(robot_motion_scene_core PUBLIC cxx_std_17)
set_target_properties(robot_motion_scene_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(robot_motion_scene_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_motion_scene_core
  Eigen3::Eigen
)

ament_target_dependencies(robot_motion_scene_core
  geometry_msgs
)

add_library(robot_motion_scene_adapter
  src/motion_scene_adapter.cpp
)

target_compile_features(robot_motion_scene_adapter PUBLIC cxx_std_17)
set_target_properties(robot_motion_scene_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(robot_motion_scene_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_motion_scene_adapter
  robot_motion_scene_core
)

ament_target_dependencies(robot_motion_scene_adapter
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  shape_msgs
)

install(TARGETS
  robot_motion_scene_core
  robot_motion_scene_adapter
  EXPORT export_robot_motion_scene_service
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_robot_motion_scene_service HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  shape_msgs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_scene_geometry test/test_scene_geometry.cpp)
  target_link_libraries(test_scene_geometry robot_motion_scene_core)
  ament_target_dependencies(test_scene_geometry geometry_msgs)
  add_test(NAME test_scene_geometry COMMAND test_scene_geometry)

  add_executable(test_task_geometry test/test_task_geometry.cpp)
  target_link_libraries(test_task_geometry robot_motion_scene_core)
  add_test(NAME test_task_geometry COMMAND test_task_geometry)
endif()

ament_package()
