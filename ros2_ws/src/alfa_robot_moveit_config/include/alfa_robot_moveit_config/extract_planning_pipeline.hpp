#pragma once

#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"
#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"
#include "robot_motion_scene_service/motion_core/task_geometry.hpp"
#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"
#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace moveit::core
{
class JointModelGroup;
}  // namespace moveit::core

namespace alfa_robot::motion
{

class MotionSceneAdapter;

struct ExtractCandidate
{
  size_t step_index = 0;
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  geometry_msgs::msg::Pose target_pose;
  bool ik_success = false;
  bool state_valid = false;
  bool carried_clear = false;
  bool detached_from_neighbors = false;
  std::string rejection_reason;
  moveit::core::RobotStatePtr state;
};

struct DualExtractStepCandidate
{
  ExtractCandidate left;
  ExtractCandidate right;
  moveit::core::RobotStatePtr state;
  bool state_valid = false;
  bool left_detached = false;
  bool right_detached = false;
  double score = std::numeric_limits<double>::infinity();
  std::string rejection_reason;
};

struct ArmExtractPath
{
  bool success = false;
  std::string failure_reason;
  std::vector<moveit::core::RobotStatePtr> states;
  std::vector<ExtractCandidate> selected_candidates;
  double final_retreat_x = 0.0;
  double final_lift_z = 0.0;
  double final_pitch_deg = 0.0;
  size_t accepted_steps = 0;
  size_t failed_steps = 0;
};

struct ExtractRolloutTiming
{
  size_t candidate_order = 0;
  size_t h_index = 0;
  size_t seed_index = 0;
  double h = 0.0;
  double ik_score = 0.0;
  double ik_solve_ms = 0.0;
  double rollout_ms = 0.0;
  double interval_ms = 0.0;
  bool success = false;
  bool loaded_plan_attempted = false;
  bool loaded_plan_success = false;
  bool lateral_shift_attempted = false;
  bool lateral_shift_success = false;
  double lateral_shift_ms = 0.0;
  double lateral_shift_reached_distance = 0.0;
  size_t lateral_shift_points = 0;
  double loaded_plan_ms = 0.0;
  size_t loaded_plan_points = 0;
  double loaded_plan_trajectory_distance = 0.0;
  bool loaded_plan_selected = false;
  size_t selected_left_loaded_pose_index = 0;
  size_t selected_right_loaded_pose_index = 0;
  double selected_left_loaded_pose_distance = 0.0;
  double selected_right_loaded_pose_distance = 0.0;
  double loaded_pose_distance_sum = 0.0;
  double loaded_pose_distance_l2 = 0.0;
  double loaded_pose_max_joint_delta = 0.0;
  size_t loaded_plan_rank = 0;
  size_t accepted_steps = 0;
  size_t failed_steps = 0;
  double final_retreat_x = 0.0;
  double final_lift_z = 0.0;
  double final_pitch_deg = 0.0;
  double right_final_retreat_x = 0.0;
  double right_final_lift_z = 0.0;
  double right_final_pitch_deg = 0.0;
  std::string failure_reason;
  std::string loaded_plan_failure_reason;
  moveit::core::RobotStatePtr final_state;
  std::vector<nlohmann::json> rollout_records;
  moveit::planning_interface::MoveGroupInterface::Plan loaded_plan;
  moveit::core::RobotStatePtr loaded_start_state;
  moveit::core::RobotStatePtr loaded_goal_state;
  std::vector<LoadedPoseReplayStage> lateral_shift_replay_stages;
};

struct ExtractMotionDelta
{
  double retreat_ratio = 1.0;
  double lift_ratio = 0.0;
  double pitch_delta_deg = 0.0;
};

struct ExtractMotionCommand
{
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  BoxSpec shifted_box;
};

struct ExtractMotionLayer
{
  double pitch_delta_deg = 0.0;
  std::vector<ExtractMotionCommand> commands;
};

struct ExtractMotionPlannerConfig
{
  double step_x = 0.03;
  double max_x = 0.36;
};

class ExtractMotionPlanner
{
public:
  explicit ExtractMotionPlanner(ExtractMotionPlannerConfig config = {});

  const ExtractMotionPlannerConfig& config() const { return config_; }

  std::vector<ExtractMotionDelta> motionDeltas() const;

  std::vector<double> pitchDeltaDegrees(double current_pitch_rad) const;

  std::vector<ExtractMotionLayer> layers(
    const BoxSpec& source_box,
    int box_id,
    double current_pitch_rad,
    double last_retreat_x,
    double current_lift_z) const;

  size_t maxStepCount() const;

private:
  ExtractMotionPlannerConfig config_;
};

struct ExtractCandidateSolverConfig
{
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup* joint_group = nullptr;
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_tool0";
  std::string right_tip = "right_tool0";
  bool use_independent_kdl = false;
  double kdl_timeout = 0.01;
  double position_tolerance = 0.01;
  double orientation_tolerance = 0.05;
  double max_tip_z_drop = 0.002;
  double min_tool_normal_z = -1e-4;
  bool enforce_tool_normal_not_down = true;
  bool top_suction = false;
  double top_suction_orientation_tolerance = 0.12217304763960307;
  double max_joint_delta = 0.0;
  int independent_kdl_max_iterations = 120;
  double independent_kdl_eps = 1e-5;
  int independent_kdl_seed_attempts = 1;
  double independent_kdl_seed_jitter = 8.0 * 3.14159265358979323846 / 180.0;
};

struct ExtractCandidateSolveRequest
{
  std::string side = "left";
  const moveit::core::RobotState* current_state = nullptr;
  geometry_msgs::msg::Pose target_pose;
  size_t step_index = 0;
  size_t candidate_index = 0;
  double retreat_x = 0.0;
  double retreat_delta_x = 0.0;
  double lift_z = 0.0;
  double lift_delta_z = 0.0;
  double pitch_up_rad = 0.0;
  double pitch_delta_rad = 0.0;
  double min_allowed_tip_z = 0.0;
  double fixed_updown = 0.0;
  double min_tool_normal_z = std::numeric_limits<double>::quiet_NaN();
};

class ExtractCandidateSolver
{
public:
  explicit ExtractCandidateSolver(ExtractCandidateSolverConfig config);
  ~ExtractCandidateSolver();

  bool initialize(std::string* error = nullptr);

  bool solve(const ExtractCandidateSolveRequest& request, ExtractCandidate* out) const;

private:
  struct ArmKdlChain;

  bool initArmKdlChain(const std::string& side, ArmKdlChain* out, std::string* error) const;
  bool solveIndependentKdl(
    const std::string& side,
    const ArmKdlChain& chain,
    const moveit::core::RobotState& current_state,
    const Eigen::Isometry3d& target_world,
    double fixed_updown,
    moveit::core::RobotState& state) const;

  const moveit::core::JointModelGroup* groupForSide(const std::string& side) const;
  const std::string& tipForSide(const std::string& side) const;
  double armJointDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  ExtractCandidateSolverConfig config_;
  std::unique_ptr<ArmKdlChain> left_kdl_chain_;
  std::unique_ptr<ArmKdlChain> right_kdl_chain_;
  mutable std::mutex moveit_kdl_mutex_;
};

struct ExtractCandidateScorerConfig
{
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_tool0";
  std::string right_tip = "right_tool0";
  double step_x = 0.03;
  double lift_weight = 10.0;
  double pitch_weight = 0.02;
  double retreat_continuity_weight = 0.2;
  double joint_delta_weight = 0.6;
  double tip_position_delta_weight = 2.0;
  double tip_orientation_delta_weight = 0.05;
};

class ExtractCandidateScorer
{
public:
  explicit ExtractCandidateScorer(ExtractCandidateScorerConfig config);

  double armJointDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double tipPositionDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double tipOrientationDelta(
    const std::string& side,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to) const;

  double score(
    const std::string& side,
    const ExtractCandidate& candidate,
    const moveit::core::RobotState& current_state,
    double last_retreat_x) const;

private:
  const moveit::core::JointModelGroup* groupForSide(const std::string& side) const;
  const std::string& tipForSide(const std::string& side) const;

  ExtractCandidateScorerConfig config_;
};

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

using ExtractTrajectoryClearCallback = std::function<bool(
  const moveit::planning_interface::MoveGroupInterface::Plan&,
  const moveit::core::RobotState&,
  const std::vector<AttachedBoxSpec>&,
  std::string*)>;

struct ExtractRolloutPlannerConfig
{
  ExtractMotionPlanner* motion_planner = nullptr;
  ExtractCandidateSolver* candidate_solver = nullptr;
  ExtractCandidateScorer* candidate_scorer = nullptr;
  const moveit::core::JointModelGroup* joint_group = nullptr;
  const moveit::core::JointModelGroup* left_arm_group = nullptr;
  const moveit::core::JointModelGroup* right_arm_group = nullptr;
  std::string left_tip = "left_tool0";
  std::string right_tip = "right_tool0";
  bool fail_fast = true;
  bool dual_async = false;
  bool top_suction = false;
  double top_suction_updown_step = 0.01;
  double top_suction_max_lift = 0.5;
  size_t success_extra_steps = 3;
  size_t top_valid_limit = 6;
  ExtractSingleClearCallback single_clear_callback;
  ExtractDualClearCallback dual_clear_callback;
  ExtractTrajectoryClearCallback trajectory_clear_callback;
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

struct ExtractBenchmarkSummary
{
  double total_interval_ms = 0.0;
  double total_rollout_ms = 0.0;
  double total_loaded_plan_ms = 0.0;
  double mean_interval_ms = 0.0;
  double mean_rollout_ms = 0.0;
  double mean_loaded_plan_ms = 0.0;
  size_t loaded_plan_attempted_count = 0;
  size_t loaded_plan_success_count = 0;
  size_t loaded_plan_first_success_rank = 0;
  size_t loaded_plan_first_success_candidate_order = 0;
  bool any_success = false;
};

ExtractBenchmarkSummary summarize_extract_timings(
  const std::vector<ExtractRolloutTiming>& timings);

class ExtractBenchmarkCsvWriter
{
public:
  static bool write(
    const std::string& path,
    const std::vector<ExtractRolloutTiming>& timings,
    const rclcpp::Logger& logger);
};

struct ExtractBenchmarkRunnerConfig
{
  IkCandidateSelector* candidate_selector = nullptr;
  LoadedPosePlanner* loaded_pose_planner = nullptr;
  rclcpp::Logger logger = rclcpp::get_logger("extract_benchmark_runner");
  bool record_tip_error_ik_candidates = false;
  bool record_rollouts = false;
  bool dual_async = false;
  bool plan_loaded_after_success = false;
  size_t candidate_limit = 0;
  size_t extract_workers = 1;
  double dedup_joint_threshold_rad = 0.0;
  double dedup_h_threshold = 0.0;
  std::string csv_path;
  LoadedPoseBatchPlanOptions loaded_options;
};

struct ExtractBenchmarkRunnerCallbacks
{
  std::function<moveit::core::RobotState(
    const moveit::core::RobotState&,
    const ik_benchmark::UpdownAwareIkCandidate&)> state_from_candidate;
  std::function<ExtractRolloutTiming(
    const moveit::core::RobotState&,
    const AttachedBoxSpec&,
    int,
    size_t,
    const ik_benchmark::UpdownAwareIkCandidate&,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>&)> rollout_left;
  std::function<ExtractRolloutTiming(
    const moveit::core::RobotState&,
    const AttachedBoxSpec&,
    int,
    const AttachedBoxSpec&,
    int,
    size_t,
    const ik_benchmark::UpdownAwareIkCandidate&,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>&)> rollout_dual;
  std::function<void(ExtractRolloutTiming&)> fill_loaded_metrics;
  std::function<void(
    const std::string&,
    const moveit::core::RobotState&,
    const std::vector<AttachedBoxSpec>&,
    const nlohmann::json&)> record_keyframe;
  std::function<void(const std::string&, const moveit::core::RobotState&, const ik_benchmark::UpdownAwareIkResult&, const AttachedBoxSpec&)> record_tip_errors;
  std::function<void(const nlohmann::json&)> record_summary;
  std::function<nlohmann::json(const ik_benchmark::UpdownAwareIkResult&)> rejection_counts_json;
  std::function<bool(const std::string&)> fail;
  std::function<void(const std::string&)> set_last_error;
};

class ExtractBenchmarkRunner
{
public:
  ExtractBenchmarkRunner(
    ExtractBenchmarkRunnerConfig config,
    ExtractBenchmarkRunnerCallbacks callbacks);

  bool runLeft(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id);

  bool runDual(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id);

private:
  std::vector<ik_benchmark::UpdownAwareIkCandidate> sortedLegalCandidates(
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    size_t* original_legal_count,
    IkCandidateSelectionStats* dedup_stats) const;

  bool writeCsv(const std::vector<ExtractRolloutTiming>& timings) const;

  ExtractRolloutTiming runDualCandidate(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const std::vector<ik_benchmark::UpdownAwareIkCandidate>& legal_candidates,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t index,
    bool record_rollout) const;

  ExtractBenchmarkRunnerConfig config_;
  ExtractBenchmarkRunnerCallbacks callbacks_;
};

}  // namespace alfa_robot::motion
