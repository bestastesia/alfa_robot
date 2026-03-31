cmake_minimum_required(VERSION 3.22)
project(alfa_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

# 编译服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SurfaceApproach.srv
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(surface_approach_node src/surface_approach_node.cpp)
add_executable(path src/path.cpp)
add_executable(trajectory_executor src/trajectory_executor.cpp)
ament_target_dependencies(surface_approach_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2_eigen
 )
 ament_target_dependencies(path
  rclcpp
  moveit_ros_planning_interface
  tf2_eigen
  trajectory_msgs
)
ament_target_dependencies(trajectory_executor
  rclcpp
  trajectory_msgs
  sensor_msgs
  std_msgs
  control_msgs
  rclcpp_action
)

# 链接服务接口
target_link_libraries(surface_approach_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
target_include_directories(path PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS surface_approach_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  path
  trajectory_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()
