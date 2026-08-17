cmake_minimum_required(VERSION 3.22)
project(motion_internal_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttachedBox.msg"
  "msg/ArmExtractPolicy.msg"
  "msg/DualGraspStrategy.msg"
  "msg/GraspTarget.msg"
  "msg/MotionContext.msg"
  "msg/MotionPlanCandidate.msg"
  "msg/Pose6D.msg"
  "msg/RobotMotionScene.msg"
  "msg/RobotMotionState.msg"
  "msg/TaskReceipt.msg"
  "srv/CheckCollision.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/PlanDualArmIk.srv"
  "srv/PlanExtract.srv"
  "srv/PlanLoaded.srv"
  "srv/RunDualArmPoseTask.srv"
  "srv/RunBoxPairTask.srv"
  "srv/RunDualGraspTask.srv"
  "srv/RunMotionTask.srv"
  "srv/SetRobotMotionScene.srv"
  "srv/SolveArmIk.srv"
  "srv/SetRobotMotionState.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs trajectory_msgs moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interface_generation test/test_interface_generation.cpp)
  rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
  )
  target_link_libraries(test_interface_generation "${cpp_typesupport_target}")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
