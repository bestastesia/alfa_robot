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
  DIRECTORY config launch rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes/current_robot
  DESTINATION share/${PROJECT_NAME}/meshes
)

# install 后用 xacro 生成 URDF（供 ik_benchmark 等离线工具使用）
find_package(xacro REQUIRED)
xacro_add_files(
  urdf/alfa_robot.urdf.xacro
  INSTALL DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_alfa_robot_urdf_xacro test/alfa_robot_test_urdf_xacro.py)
find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()
