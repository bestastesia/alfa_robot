#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace robot_motion::core
{

struct IkSolverOptions
{
  std::string urdf_path;
  std::string srdf_path;
  std::string base_frame;
  std::string tip_link;
  std::string tip_link2;
  bool reject_collisions = true;
  bool enforce_arm_base_collisions = false;
};

struct ReachSphereConfig
{
  double cx = 0.015;
  double cy = 0.3125;
  double cz = 0.6625;
  double radius = 0.815;
};

struct UpdownAwareIkConfig
{
  enum class HSearchMode
  {
    FixedDiscrete,
    ContinuousRange,
  };

  std::string fixed_group = "dual_arm";
  std::string free_group = "dual_arm_with_base";
  std::string solver_plugin = "bio_ik/BioIKKinematicsPlugin";
  std::string base_frame = "base_link";
  std::string left_tip = "left_tool0";
  std::string right_tip = "right_tool0";

  ReachSphereConfig left_reach_sphere;
  ReachSphereConfig right_reach_sphere = {0.015, -0.3125, 0.6625, 0.815};
  double tool0_offset = 0.1;
  double sphere_margin = 0.0;
  double gripper_z_reach_lower = 0.45;
  double gripper_z_reach_upper = 1.25;
  double top_suction_z_reach_lower = 0.3;
  double top_suction_z_reach_upper = 0.45;
  // updown(h) 采样范围默认值：逻辑/URDF 与电机物理空间均为 [0, 0.7]。
  // 采样阶段即限死，不生成越界候选。
  double h_lower = 0.0;
  double h_upper = 0.7;
  bool full_h_range_scan = true;

  HSearchMode h_search_mode = HSearchMode::FixedDiscrete;
  double h_search_margin = 0.1;
  double h_step = 0.01;
  std::size_t h_candidate_count = 5;
  double max_updown_delta = std::numeric_limits<double>::infinity();

  std::size_t seed_count = 4;
  std::size_t continuous_seed_multiplier = 1;
  double seed_noise = 0.35;
  bool try_target_orders = true;
  bool use_reversed_target_order = true;

  std::size_t workers = 4;
  double timeout = 0.5;

  bool check_tip_error = true;
  double position_tolerance = 0.02;
  double top_suction_position_tolerance = 0.04;
  double orientation_tolerance = 0.05;
  double top_suction_orientation_tolerance = 0.1221730476;
  bool check_collision = false;
  bool enforce_arm_base_collisions = false;
  bool reject_swapped_tips = true;

  bool fallback_enabled = true;
  double fallback_timeout = 2.0;
  std::size_t fallback_seed_count = 12;
  std::size_t fallback_rounds = 1;
  std::size_t fallback_random_family_count = 2;
  std::size_t fallback_random_per_family = 4;
  double fallback_seed_noise = 0.6;
  double fallback_updown_noise = 0.5;

  double cost_updown_static_bonus = 1.0;
  double cost_updown_within_0p1_bonus = 0.3;
  double cost_updown_over_0p1_distance = 1.0;
  bool cost_updown_enabled = false;
  double cost_joint_limit_margin = 1.0;
  std::vector<double> joint_limit_weights = {0.5, 3.0, 0.7, 0.5, 1.5, 1.2};
  double joint_limit_free_ratio = 0.6;
  double cost_joint2_torque = 2.0;
  double cost_joint3_torque = 0.5;
  double cost_loaded_family_distance = 0.0;
  double cost_loaded_preferred_distance = 0.0;
  double cost_solve_ms = 0.0;

  std::vector<std::vector<double>> left_loaded_pose_family;
  std::vector<std::vector<double>> right_loaded_pose_family;
  std::size_t left_preferred_loaded_pose_index = 0;
  std::size_t right_preferred_loaded_pose_index = 0;

  double updown_static_epsilon = 0.005;
  double updown_small_motion_threshold = 0.1;

  double left_joint2_horizontal_angle = 0.0;
  double left_joint3_horizontal_angle = 0.0;
  double right_joint2_horizontal_angle = 0.0;
  double right_joint3_horizontal_angle = 0.0;
  double link2_length = 0.65;
  double link3_length = 0.65;
  double link2_mass_proxy = 1.0;
  double link3_mass_proxy = 1.0;
  double payload_mass_proxy = 1.0;

  IkSolverOptions solver_options;
};

struct UpdownAwareIkRequest
{
  enum class GraspMode
  {
    Front,
    TopSuction,
  };

  Eigen::Isometry3d left_target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right_target = Eigen::Isometry3d::Identity();
  double current_h = 0.0;
  GraspMode grasp_mode = GraspMode::Front;
  std::vector<double> current_arm_joints;
  std::vector<double> current_full_joints;
};

struct UpdownAwareIkCandidate
{
  bool legal = false;
  bool collision_free = true;
  bool swapped = false;
  bool timeout_like = false;
  std::string solver_path;
  std::string target_order;
  std::string rejection_reason;

  double h = 0.0;
  double h_center = 0.0;
  double h_range_lower = 0.0;
  double h_range_upper = 0.0;
  double score = std::numeric_limits<double>::infinity();
  double solve_ms = 0.0;
  double direct_pos_error = 0.0;
  double direct_ori_error = 0.0;
  double swapped_pos_error = 0.0;
  double updown_delta = 0.0;
  double joint_delta = 0.0;
  double joint_limit_margin_cost = 0.0;
  std::size_t h_index = 0;
  std::size_t seed_index = 0;

  std::vector<std::string> joint_names;
  std::vector<double> joint_values;
  std::vector<std::string> full_joint_names;
  std::vector<double> full_joint_values;
  std::vector<std::string> collision_pairs;
};

struct UpdownAwareIkResult
{
  bool success = false;
  bool fallback_used = false;
  bool range_reachable = false;
  std::string solver_path;
  std::string failure_reason;

  double h_interval_lower = 0.0;
  double h_interval_upper = 0.0;
  double h_center = 0.0;
  std::vector<double> h_candidates;

  std::size_t trial_count = 0;
  std::size_t legal_count = 0;
  std::size_t timeout_like_count = 0;
  std::size_t swapped_rejected_count = 0;
  double wall_ms = 0.0;
  double sum_solve_ms = 0.0;

  UpdownAwareIkCandidate selected;
  std::vector<UpdownAwareIkCandidate> candidates;
  std::vector<UpdownAwareIkCandidate> pre_score_candidates;
};

using UpdownAwareCostFn =
  std::function<double(const UpdownAwareIkCandidate&, const UpdownAwareIkRequest&)>;

}  // namespace robot_motion::core
