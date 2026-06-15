#pragma once

#include "alfa_robot_moveit_config/extract_candidate_scorer.hpp"
#include "alfa_robot_moveit_config/extract_candidate_solver.hpp"
#include "alfa_robot_moveit_config/extract_motion_planner.hpp"
#include "alfa_robot_moveit_config/extract_planner_types.hpp"
#include "alfa_robot_moveit_config/motion_core/task_geometry.hpp"

#include <moveit/robot_state/robot_state.h>

#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace moveit::core
{
class JointModelGroup;
}

namespace alfa_robot::motion
{

using ExtractRecordStepCallback = std::function<void(
  size_t,
  const moveit::core::RobotState&,
  const nlohmann::json&)>;

using ExtractSingleClearCallback = std::function<bool(
  const moveit::core::RobotState&,
  const AttachedBoxSpec&,
  int,
  bool*,
  std::string*)>;

using ExtractDualClearCallback = std::function<bool(
  const moveit::core::RobotState&,
  const AttachedBoxSpec&,
  int,
  const AttachedBoxSpec&,
  int,
  bool*,
  bool*,
  std::string*)>;

struct ExtractRolloutPlannerConfig
{
  ExtractMotionPlanner* motion_planner = nullptr;
  ExtractCandidateSolver* candidate_solver = nullptr;
  ExtractCandidateScorer* candidate_scorer = nullptr;
  const moveit::core::JointModelGroup* joint_group = nullptr;
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_v5_tool0";
  std::string right_tip = "right_v5_tool0";
  bool fail_fast = true;
  bool dual_async = false;
  size_t success_extra_steps = 3;
  size_t top_valid_limit = 6;
  ExtractSingleClearCallback single_clear_callback;
  ExtractDualClearCallback dual_clear_callback;
};

class ExtractRolloutPlanner
{
public:
  explicit ExtractRolloutPlanner(ExtractRolloutPlannerConfig config);

  ExtractRolloutTiming rolloutLeft(
    const moveit::core::RobotState& start_state,
    const BoxSpec& source_box,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    size_t candidate_order,
    size_t h_index,
    size_t seed_index,
    double h,
    double ik_score,
    double ik_solve_ms,
    const ExtractRecordStepCallback& record_step = {}) const;

  ExtractRolloutTiming rolloutDual(
    const moveit::core::RobotState& start_state,
    const BoxSpec& left_source_box,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const BoxSpec& right_source_box,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t candidate_order,
    size_t h_index,
    size_t seed_index,
    double h,
    double ik_score,
    double ik_solve_ms,
    const ExtractRecordStepCallback& record_step = {}) const;

private:
  double currentUpdown(const moveit::core::RobotState& state) const;
  const std::string& tipForSide(const std::string& side) const;
  const moveit::core::JointModelGroup* groupForSide(const std::string& side) const;

  double currentPitchUpRad(const std::string& side, const moveit::core::RobotState& state) const;

  bool solveCandidate(
    const std::string& side,
    const moveit::core::RobotState& current_state,
    const geometry_msgs::msg::Pose& target_pose,
    size_t step_index,
    size_t candidate_index,
    double retreat_x,
    double retreat_delta_x,
    double lift_z,
    double lift_delta_z,
    double pitch_up_rad,
    double pitch_delta_rad,
    double min_allowed_tip_z,
    const AttachedBoxSpec& carried_box,
    int box_id,
    ExtractCandidate* out) const;

  std::vector<ExtractCandidate> makeCandidatesForSide(
    const std::string& side,
    const moveit::core::RobotState& current_state,
    const BoxSpec& source_box,
    const AttachedBoxSpec& carried_box,
    int box_id,
    size_t step,
    double last_retreat_x,
    double current_lift_z,
    double min_allowed_tip_z) const;

  std::vector<ExtractCandidate> topValidCandidates(
    const std::string& side,
    const std::vector<ExtractCandidate>& candidates,
    const moveit::core::RobotState& current_state,
    double last_retreat_x) const;

  void copyArmState(
    const std::string& side,
    const moveit::core::RobotState& from,
    moveit::core::RobotState& to) const;

  ArmExtractPath rolloutArm(
    const std::string& side,
    const moveit::core::RobotState& start_state,
    const BoxSpec& source_box,
    const AttachedBoxSpec& carried_box,
    int box_id) const;

  moveit::core::RobotState combineAsyncArmStates(
    const moveit::core::RobotState& base_state,
    const ArmExtractPath& left_path,
    const ArmExtractPath& right_path,
    size_t left_index,
    size_t right_index) const;

  bool validateAsyncPath(
    const moveit::core::RobotState& start_state,
    const ArmExtractPath& left_path,
    const ArmExtractPath& right_path,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    std::vector<moveit::core::RobotStatePtr>* combined_states,
    std::string* reason) const;

  DualExtractStepCandidate selectDualStepCandidate(
    const moveit::core::RobotState& current_state,
    const std::vector<ExtractCandidate>& left_candidates,
    const std::vector<ExtractCandidate>& right_candidates,
    double left_last_retreat_x,
    double right_last_retreat_x,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id) const;

  ExtractRolloutPlannerConfig config_;
};

}  // namespace alfa_robot::motion
