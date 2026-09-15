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
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# 根 xacro 已随 urdf/ 目录安装；运行时按需展开，避免首次构建时自查找本包失败。
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_alfa_robot_urdf_xacro test/alfa_robot_test_urdf_xacro.py)
  ament_add_pytest_test(test_v3_description_semantics test/test_v3_description_semantics.py)
  ament_add_pytest_test(test_v3_chassis_semantics test/test_v3_chassis_semantics.py)
  ament_add_pytest_test(test_v3_head_semantics test/test_v3_head_semantics.py)
  ament_add_pytest_test(test_v311_consumer_contract test/test_v311_consumer_contract.py)
endif()

ament_package()
