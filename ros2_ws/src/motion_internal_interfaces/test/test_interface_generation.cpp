#include <gtest/gtest.h>

#include <type_traits>

#include "motion_internal_interfaces/msg/arm_extract_policy.hpp"
#include "motion_internal_interfaces/msg/attached_box.hpp"
#include "motion_internal_interfaces/msg/dual_grasp_strategy.hpp"
#include "motion_internal_interfaces/msg/grasp_target.hpp"
#include "motion_internal_interfaces/msg/motion_context.hpp"
#include "motion_internal_interfaces/msg/motion_plan_candidate.hpp"
#include "motion_internal_interfaces/msg/pose6_d.hpp"
#include "motion_internal_interfaces/msg/robot_motion_scene.hpp"
#include "motion_internal_interfaces/msg/robot_motion_state.hpp"
#include "motion_internal_interfaces/msg/task_receipt.hpp"
#include "motion_internal_interfaces/srv/check_collision.hpp"
#include "motion_internal_interfaces/srv/execute_trajectory.hpp"
#include "motion_internal_interfaces/srv/plan_dual_arm_ik.hpp"
#include "motion_internal_interfaces/srv/plan_extract.hpp"
#include "motion_internal_interfaces/srv/plan_loaded.hpp"
#include "motion_internal_interfaces/srv/run_box_pair_task.hpp"
#include "motion_internal_interfaces/srv/run_dual_arm_pose_task.hpp"
#include "motion_internal_interfaces/srv/run_dual_grasp_task.hpp"
#include "motion_internal_interfaces/srv/run_motion_task.hpp"
#include "motion_internal_interfaces/srv/set_robot_motion_scene.hpp"
#include "motion_internal_interfaces/srv/set_robot_motion_state.hpp"
#include "motion_internal_interfaces/srv/solve_arm_ik.hpp"

TEST(MotionInternalInterfaces, AllIdlsGenerateCppTypes)
{
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::ArmExtractPolicy>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::AttachedBox>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::DualGraspStrategy>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::GraspTarget>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::MotionContext>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::MotionPlanCandidate>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::Pose6D>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::RobotMotionScene>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::RobotMotionState>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::msg::TaskReceipt>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::CheckCollision::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::ExecuteTrajectory::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::PlanDualArmIk::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::PlanExtract::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::PlanLoaded::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::RunBoxPairTask::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::RunDualArmPoseTask::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::RunDualGraspTask::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::RunMotionTask::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::SetRobotMotionScene::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::SetRobotMotionState::Request>);
  static_assert(std::is_default_constructible_v<motion_internal_interfaces::srv::SolveArmIk::Request>);
  SUCCEED();
}
