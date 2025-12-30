cmake_minimum_required(VERSION 3.8)
project(base_description)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找 ROS 2 的构建工具
find_package(ament_cmake REQUIRED)

# 安装 meshes, urdf, launch 文件夹到 ROS 2 的共享目录
install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
