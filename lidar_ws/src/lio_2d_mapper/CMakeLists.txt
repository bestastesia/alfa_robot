cmake_minimum_required(VERSION 3.8)
project(lio_2d_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(mapper_node src/mapper_node.cpp)
add_executable(lidar_tf_node src/lidar_tf_node.cpp)
add_executable(lidar_init_pose_node src/lidar_init_pose_node.cpp)
add_executable(pcd_to_grid_map_node src/pcd_to_grid_map_node.cpp)
add_executable(offline_map_publisher src/offline_map_publisher.cpp)

ament_target_dependencies(mapper_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(lidar_tf_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(lidar_init_pose_node
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(pcd_to_grid_map_node
  rclcpp
  nav_msgs
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(offline_map_publisher
  rclcpp
  nav_msgs
)

install(TARGETS
  mapper_node
  lidar_tf_node
  lidar_init_pose_node
  pcd_to_grid_map_node
  offline_map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
