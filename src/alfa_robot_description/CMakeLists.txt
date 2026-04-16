cmake_minimum_required(VERSION 3.8)
project(alfa_robot_description)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

# 显式安装 collision 网格，确保 RViz/Gazebo 能解析 package://.../collision/*.STL
install(
  DIRECTORY meshes/alfa_robot/collision
  DESTINATION share/${PROJECT_NAME}/meshes/alfa_robot/collision
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_alfa_robot_urdf_xacro test/alfa_robot_test_urdf_xacro.py)
find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()
