cmake_minimum_required(VERSION 3.8)
project(alfa_robot_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(alfa_robot_hardware SHARED
  src/alfa_robot_hardware.cpp
  src/joint/rmd_joint.cpp
  src/joint/canopen_joint.cpp
  src/joint/placeholder_joint.cpp
  src/joint/zeroerr_joint.cpp
  src/joint/cylinder_joint.cpp
  src/driver/rmd_driver.cpp
  src/driver/canopen_driver.cpp
  src/driver/zeroerr_driver.cpp
  src/driver/cylinder_driver.cpp
)

target_include_directories(alfa_robot_hardware PUBLIC include)

ament_target_dependencies(alfa_robot_hardware
  hardware_interface rclcpp rclcpp_lifecycle)

target_compile_definitions(alfa_robot_hardware
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface alfa_robot_hardware.xml)

install(TARGETS alfa_robot_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_alfa_robot_hardware test/test_alfa_robot_hardware.cpp)
  target_include_directories(test_alfa_robot_hardware PRIVATE include)
  target_link_libraries(test_alfa_robot_hardware alfa_robot_hardware)
  ament_target_dependencies(test_alfa_robot_hardware
    hardware_interface pluginlib rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(alfa_robot_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
