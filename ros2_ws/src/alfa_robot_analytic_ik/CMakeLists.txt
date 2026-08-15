cmake_minimum_required(VERSION 3.22)
project(alfa_robot_analytic_ik)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(alfa_robot_analytic_ik
  src/analytic_ik.cpp
  src/v3_redundant_analytic_ik.cpp
)
set_target_properties(alfa_robot_analytic_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(alfa_robot_analytic_ik PUBLIC cxx_std_17)

target_include_directories(alfa_robot_analytic_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(alfa_robot_analytic_ik PUBLIC Eigen3::Eigen)

add_library(alfa_robot_analytic_ik::alfa_robot_analytic_ik ALIAS alfa_robot_analytic_ik)

install(TARGETS alfa_robot_analytic_ik
  EXPORT export_alfa_robot_analytic_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_alfa_robot_analytic_ik HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  add_executable(test_analytic_ik test/test_analytic_ik.cpp)
  target_link_libraries(test_analytic_ik alfa_robot_analytic_ik)
  add_test(NAME test_analytic_ik COMMAND test_analytic_ik)

  add_executable(test_v3_redundant_analytic_ik test/test_v3_redundant_analytic_ik.cpp)
  target_link_libraries(test_v3_redundant_analytic_ik alfa_robot_analytic_ik)
  add_test(NAME test_v3_redundant_analytic_ik COMMAND test_v3_redundant_analytic_ik)
endif()

ament_package()
