/**
 * dual_arm_planner_node.cpp
 *
 * MoveIt-backed dual-arm box-stack flow reproducer.
 *
 * Grasp IK uses the project benchmark solver:
 *   fixed discrete h candidates × multiple BioIK seeds × cost scoring.
 * MoveIt is only used for joint-space trajectory planning/execution.
 */

#include "ik_benchmark/parallel_updown_aware_ik_solver.h"
#include "alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp"
#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"
#include "alfa_robot_moveit_config/extract_demo_orchestrator.hpp"
#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"
#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"
#include "alfa_robot_moveit_config/motion_flow_recorder.hpp"
#include "alfa_robot_moveit_config/motion_scene_adapter.hpp"
#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"
#include "alfa_robot_moveit_config/motion_core/scene_geometry.hpp"
#include "alfa_robot_moveit_config/motion_core/task_geometry.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometric_shapes/shapes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

using alfa_robot::motion::AttachedBoxSpec;
using alfa_robot::motion::ArmExtractPath;
using alfa_robot::motion::ExtractBenchmarkRunner;
using alfa_robot::motion::ExtractBenchmarkRunnerCallbacks;
using alfa_robot::motion::ExtractBenchmarkRunnerConfig;
using alfa_robot::motion::ExtractCandidateScorer;
using alfa_robot::motion::ExtractCandidateScorerConfig;
using alfa_robot::motion::ExtractCandidateSolver;
using alfa_robot::motion::ExtractCandidateSolverConfig;
using alfa_robot::motion::ExtractCandidateSolveRequest;
using alfa_robot::motion::ExtractDemoCallbacks;
using alfa_robot::motion::ExtractDemoConfig;
using alfa_robot::motion::ExtractDemoOrchestrator;
using alfa_robot::motion::ExtractMotionPlanner;
using alfa_robot::motion::ExtractMotionPlannerConfig;
using alfa_robot::motion::ExtractRolloutPlanner;
using alfa_robot::motion::ExtractRolloutPlannerConfig;
using alfa_robot::motion::DualExtractStepCandidate;
using alfa_robot::motion::ExtractCandidate;
using alfa_robot::motion::ExtractRolloutTiming;
using alfa_robot::motion::AxisAlignedBox;
using alfa_robot::motion::BoxStackFlowCallbacks;
using alfa_robot::motion::BoxStackFlowConfig;
using alfa_robot::motion::BoxStackFlowOrchestrator;
using alfa_robot::motion::BoxSpec;
using alfa_robot::motion::BoxWallGeometryConfig;
using alfa_robot::motion::CarriedBoxGeometryConfig;
using alfa_robot::motion::ContainerGeometryConfig;
using alfa_robot::motion::ContainerPanel;
using alfa_robot::motion::LoadedPoseBatchPlanOptions;
using alfa_robot::motion::LoadedPoseBatchPlanResult;
using alfa_robot::motion::IkCandidateSelectionStats;
using alfa_robot::motion::IkCandidateSelector;
using alfa_robot::motion::IkCandidateSelectorConfig;
using alfa_robot::motion::LoadedPosePlanner;
using alfa_robot::motion::LoadedPosePlannerConfig;
using alfa_robot::motion::LoadedPoseSelection;
using alfa_robot::motion::LoadedPoseSelector;
using alfa_robot::motion::LoadedPoseSelectorConfig;
using alfa_robot::motion::MotionSceneAdapter;
using alfa_robot::motion::MotionSceneAdapterConfig;
using alfa_robot::motion::MotionFlowRecorder;
using alfa_robot::motion::OptimizedDualIkSolver;
using alfa_robot::motion::OptimizedDualIkSolverConfig;
using alfa_robot::motion::OptimizedDualIkSolveRequest;
using alfa_robot::motion::PickPair;
using alfa_robot::motion::StaticBoxObstacle;
using alfa_robot::motion::aabb_from_attached_box_transform;
using alfa_robot::motion::aabb_overlaps;
using alfa_robot::motion::carried_box_detached_from_neighbors;
using alfa_robot::motion::deg_to_rad;
using alfa_robot::motion::format_degrees;
using alfa_robot::motion::forward_x_orientation;
using alfa_robot::motion::make_attached_box_spec;
using alfa_robot::motion::make_boxes;
using alfa_robot::motion::make_box_wall_obstacles_for_opening;
using alfa_robot::motion::make_container_panels;
using alfa_robot::motion::make_identity_pose;
using alfa_robot::motion::make_pick_pairs;
using alfa_robot::motion::make_pose;
using alfa_robot::motion::names_values_json;
using alfa_robot::motion::parse_box_pair_list;
using alfa_robot::motion::parse_pose_family_degrees;
using alfa_robot::motion::pitch_up_orientation;
using alfa_robot::motion::pose_degrees_json;
using alfa_robot::motion::pose_family_degrees_json;
using alfa_robot::motion::pose_json;
using alfa_robot::motion::pose_orientation_error;
using alfa_robot::motion::pose_position_error;
using alfa_robot::motion::pose_to_eigen;
using alfa_robot::motion::shortest_angular_distance;
using alfa_robot::motion::top_suction_orientation;
using alfa_robot::motion::vector_json;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJointTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

std::vector<std::string> touch_links_for_attached_box(const AttachedBoxSpec& box)
{
  std::vector<std::string> links{box.link_name};
  if (box.link_name.rfind("left_", 0) == 0) {
    links.push_back("left_v5_link6");
    links.push_back("left_v5_link5");
  } else if (box.link_name.rfind("right_", 0) == 0) {
    links.push_back("right_v5_link6");
    links.push_back("right_v5_link5");
  }
  return links;
}

void attach_boxes_to_robot_state(
  moveit::core::RobotState& state,
  const std::vector<AttachedBoxSpec>& boxes,
  double collision_padding)
{
  for (const auto& box : boxes) {
    const double size_x = std::max(0.001, box.size[0] + 2.0 * collision_padding);
    const double size_y = std::max(0.001, box.size[1] + 2.0 * collision_padding);
    const double size_z = std::max(0.001, box.size[2] + 2.0 * collision_padding);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(std::make_shared<shapes::Box>(size_x, size_y, size_z));

    EigenSTL::vector_Isometry3d shape_poses;
    Eigen::Isometry3d shape_pose = Eigen::Isometry3d::Identity();
    shape_pose.translation() = Eigen::Vector3d(
      box.center_in_link[0],
      box.center_in_link[1],
      box.center_in_link[2]);
    shape_poses.push_back(shape_pose);

    state.attachBody(
      box.id,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
      touch_links_for_attached_box(box),
      box.link_name);
  }
  state.update(true);
}

}  // namespace

class DualArmPlannerNode : public rclcpp::Node
{
public:
  explicit DualArmPlannerNode(const rclcpp::NodeOptions& options)
  : Node("dual_arm_planner", options)
  {}

  void init()
  {
    planning_group_ = get_or_declare_parameter<std::string>("planning_group", "dual_v5_arm_with_base");
    left_tip_ = get_or_declare_parameter<std::string>("left_tip", "left_v5_tool0");
    right_tip_ = get_or_declare_parameter<std::string>("right_tip", "right_v5_tool0");
    execute_ = get_or_declare_parameter<bool>("execute", true);
    execution_backend_ = get_or_declare_parameter<std::string>("execution_backend", "moveit");
    execution_action_name_ = get_or_declare_parameter<std::string>(
      "execution_action_name", "/alfa_execution/execute_joint_trajectory");
    execution_action_wait_timeout_s_ = get_or_declare_parameter<double>("execution_action_wait_timeout_s", 5.0);
    execution_result_timeout_s_ = get_or_declare_parameter<double>("execution_result_timeout_s", 0.0);
    execution_include_turn_ = get_or_declare_parameter<bool>("execution_include_turn", true);
    execution_allow_hold_missing_target_joints_ =
      get_or_declare_parameter<bool>("execution_allow_hold_missing_target_joints", true);
    execution_reject_unmapped_planned_joints_ =
      get_or_declare_parameter<bool>("execution_reject_unmapped_planned_joints", true);
    reject_ik_collisions_ = get_or_declare_parameter<bool>("reject_ik_collisions", false);
    check_goal_collision_ = get_or_declare_parameter<bool>("check_goal_collision", false);
    prefer_commanded_state_ = get_or_declare_parameter<bool>("prefer_commanded_state", true);
    fixed_updown_ = get_or_declare_parameter<double>("fixed_updown", 0.45);
    box_front_x_ = get_or_declare_parameter<double>("box_front_x", 0.625);
    scene_y_shift_ = get_or_declare_parameter<double>("scene_y_shift", 0.0);
    world_to_base_z_ = get_or_declare_parameter<double>("world_to_base_z", 0.202094);
    top_suction_x_offset_ = get_or_declare_parameter<double>("top_suction_x_offset", 0.15);
    top_suction_z_offset_ = get_or_declare_parameter<double>("top_suction_z_offset", 0.2);
    max_rounds_ = get_or_declare_parameter<int>("max_rounds", 10);
    include_top_suction_ = get_or_declare_parameter<bool>("include_top_suction", true);
    ik_timeout_ = get_or_declare_parameter<double>("ik_timeout", 2.0);
    planning_time_ = get_or_declare_parameter<double>("planning_time", 8.0);
    planning_attempts_ = get_or_declare_parameter<int>("planning_attempts", 8);
    velocity_scale_ = get_or_declare_parameter<double>("velocity_scale", 1.0);
    acceleration_scale_ = get_or_declare_parameter<double>("acceleration_scale", 1.0);
    joint_goal_tolerance_rad_ = get_or_declare_parameter<double>("joint_goal_tolerance_rad", 0.02);
    state_wait_timeout_s_ = get_or_declare_parameter<double>("state_wait_timeout_s", 2.0);
    record_jsonl_path_ = get_or_declare_parameter<std::string>(
      "record_jsonl_path", "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/moveit_box_stack_flow/moveit_box_stack_flow.jsonl");
    record_trajectories_ = get_or_declare_parameter<bool>("record_trajectories", true);
    extract_monitor_snapshot_path_ = get_or_declare_parameter<std::string>(
      "extract_monitor_snapshot_path",
      "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/extract_stage_monitor/latest_snapshot.json");

    ik_config_.fixed_group = get_or_declare_parameter<std::string>("ik_fixed_group", "dual_v5_arm");
    ik_config_.free_group = get_or_declare_parameter<std::string>("ik_free_group", "dual_v5_arm_with_base");
    ik_config_.solver_plugin = get_or_declare_parameter<std::string>("ik_solver_plugin", "bio_ik/BioIKKinematicsPlugin");
    ik_config_.base_frame = get_or_declare_parameter<std::string>("ik_base_frame", "base_link");
    ik_config_.left_tip = left_tip_;
    ik_config_.right_tip = right_tip_;
    ik_config_.tool0_offset = get_or_declare_parameter<double>("ik_tool0_offset", 0.0);
    ik_config_.gripper_z_reach_lower = get_or_declare_parameter<double>("front_z_reach_lower", 0.9) - world_to_base_z_;
    ik_config_.gripper_z_reach_upper = get_or_declare_parameter<double>("front_z_reach_upper", 1.3) - world_to_base_z_;
    ik_config_.top_suction_z_reach_lower = get_or_declare_parameter<double>("top_z_reach_lower", 0.3) - world_to_base_z_;
    ik_config_.top_suction_z_reach_upper = get_or_declare_parameter<double>("top_z_reach_upper", 0.45) - world_to_base_z_;
    ik_config_.h_lower = get_or_declare_parameter<double>("ik_h_lower", 0.0);
    ik_config_.h_upper = get_or_declare_parameter<double>("ik_h_upper", 0.99);
    ik_config_.h_search_mode = ik_benchmark::UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
    ik_config_.h_search_margin = get_or_declare_parameter<double>("ik_h_search_margin", 0.2);
    ik_config_.h_step = get_or_declare_parameter<double>("ik_h_step", 0.1);
    ik_config_.h_candidate_count = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_h_candidate_count", 16)));
    ik_config_.seed_count = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_seed_count", 32)));
    ik_config_.cost_loaded_family_distance =
      get_or_declare_parameter<double>("ik_loaded_family_distance_weight", 0.2);
    ik_config_.cost_loaded_preferred_distance =
      get_or_declare_parameter<double>("ik_loaded_preferred_distance_weight", 0.1);
    ik_config_.workers = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_workers", 16)));
    ik_config_.timeout = get_or_declare_parameter<double>("ik_candidate_timeout", 0.01);
    ik_config_.try_target_orders = get_or_declare_parameter<bool>("ik_try_target_orders", false);
    ik_config_.use_reversed_target_order = get_or_declare_parameter<bool>("ik_use_reversed_target_order", true);
    ik_config_.check_tip_error = true;
    ik_config_.position_tolerance = get_or_declare_parameter<double>("ik_position_tolerance", 0.02);
    ik_config_.top_suction_position_tolerance = get_or_declare_parameter<double>("ik_top_position_tolerance", 0.04);
    ik_config_.orientation_tolerance = get_or_declare_parameter<double>("ik_orientation_tolerance", 0.05);
    ik_config_.top_suction_orientation_tolerance =
      get_or_declare_parameter<double>("ik_top_orientation_tolerance_deg", 7.0) * M_PI / 180.0;
    ik_config_.check_collision = get_or_declare_parameter<bool>("optimized_ik_check_collision", reject_ik_collisions_);
    ik_config_.enforce_arm_base_collisions = get_or_declare_parameter<bool>("ik_enforce_arm_base_collisions", true);
    ik_config_.reject_swapped_tips = get_or_declare_parameter<bool>("ik_reject_swapped_tips", true);
    ik_config_.fallback_enabled = get_or_declare_parameter<bool>("ik_fallback_enabled", false);
    ik_config_.fallback_seed_count = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_fallback_seed_count", 64)));
    ik_config_.fallback_rounds = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_fallback_rounds", 1)));
    ik_config_.fallback_timeout = get_or_declare_parameter<double>("ik_fallback_timeout", ik_config_.timeout);

    enable_container_obstacle_ = get_or_declare_parameter<bool>("enable_container_obstacle", true);
    container_frame_ = get_or_declare_parameter<std::string>("container_frame", "world");
    container_length_ = get_or_declare_parameter<double>("container_length", 4.0);
    container_width_ = get_or_declare_parameter<double>("container_width", 2.2);
    container_height_ = get_or_declare_parameter<double>("container_height", 2.4);
    container_center_x_ = get_or_declare_parameter<double>("container_center_x", 0.8);
    container_center_y_ = get_or_declare_parameter<double>("container_center_y", 0.0);
    container_floor_z_ = get_or_declare_parameter<double>("container_floor_z", 0.0);
    container_wall_thickness_ = get_or_declare_parameter<double>("container_wall_thickness", 0.02);
    enable_attached_box_collision_ = get_or_declare_parameter<bool>("enable_attached_box_collision", true);
    carried_box_depth_ = get_or_declare_parameter<double>("carried_box_depth", 0.3);
    carried_box_width_ = get_or_declare_parameter<double>("carried_box_width", 0.4);
    carried_box_height_ = get_or_declare_parameter<double>("carried_box_height", 0.4);
    attached_box_collision_padding_ = get_or_declare_parameter<double>("attached_box_collision_padding", -0.002);
    enable_static_box_obstacles_ = get_or_declare_parameter<bool>("enable_static_box_obstacles", true);
    static_box_obstacle_inset_ = get_or_declare_parameter<double>("static_box_obstacle_inset", 0.002);

    extract_demo_left_box_id_ = get_or_declare_parameter<int>("extract_demo_left_box_id", 2);
    extract_demo_right_box_id_ = get_or_declare_parameter<int>("extract_demo_right_box_id", 4);
    extract_demo_pair_sequence_ = parse_box_pair_list(
      get_or_declare_parameter<std::string>("extract_demo_pair_sequence", "2,4;7,9;12,14;17,19"));
    if (extract_demo_pair_sequence_.empty()) {
      extract_demo_pair_sequence_ = {{2, 4}, {7, 9}, {12, 14}, {17, 19}};
    }
    extract_demo_all_rows_ = get_or_declare_parameter<bool>("extract_demo_all_rows", false);
    extract_step_x_ = get_or_declare_parameter<double>("extract_step_x", 0.03);
    extract_max_x_ = get_or_declare_parameter<double>("extract_max_x", 0.36);
    extract_lift_candidates_ = get_or_declare_parameter<std::vector<double>>("extract_lift_candidates", std::vector<double>{0.0, 0.02, 0.05, 0.08});
    extract_pitch_candidates_deg_ = get_or_declare_parameter<std::vector<double>>("extract_pitch_candidates_deg", std::vector<double>{0.0, 5.0, 10.0, 15.0});
    extract_neighbor_margin_ = get_or_declare_parameter<double>("extract_neighbor_margin", 0.02);
    extract_fail_fast_ = get_or_declare_parameter<bool>("extract_fail_fast", false);
    extract_success_extra_steps_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_success_extra_steps", 3)));
    extract_kdl_timeout_ = get_or_declare_parameter<double>("extract_kdl_timeout", 0.003);
    extract_position_tolerance_ = get_or_declare_parameter<double>("extract_position_tolerance", 0.01);
    extract_orientation_tolerance_ = get_or_declare_parameter<double>("extract_orientation_tolerance", 0.05);
    extract_max_tip_z_drop_ = get_or_declare_parameter<double>("extract_max_tip_z_drop", 0.002);
    extract_min_tool_normal_z_ = get_or_declare_parameter<double>("extract_min_tool_normal_z", -1e-4);
    extract_score_lift_weight_ = get_or_declare_parameter<double>("extract_score_lift_weight", 10.0);
    extract_score_pitch_weight_ = get_or_declare_parameter<double>("extract_score_pitch_weight", 0.02);
    extract_score_retreat_continuity_weight_ = get_or_declare_parameter<double>("extract_score_retreat_continuity_weight", 0.2);
    extract_score_joint_delta_weight_ = get_or_declare_parameter<double>("extract_score_joint_delta_weight", 0.6);
    extract_score_tip_position_delta_weight_ = get_or_declare_parameter<double>("extract_score_tip_position_delta_weight", 2.0);
    extract_score_tip_orientation_delta_weight_ = get_or_declare_parameter<double>("extract_score_tip_orientation_delta_weight", 0.05);
    extract_max_joint_delta_ = get_or_declare_parameter<double>("extract_max_joint_delta", 0.0);
    extract_demo_direct_grasp_start_ = get_or_declare_parameter<bool>("extract_demo_direct_grasp_start", false);
    extract_grasp_ik_home_updown_ = get_or_declare_parameter<double>("extract_grasp_ik_home_updown", 0.3);
    extract_benchmark_all_legal_ik_ = get_or_declare_parameter<bool>("extract_benchmark_all_legal_ik", false);
    extract_benchmark_dual_arm_ = get_or_declare_parameter<bool>("extract_benchmark_dual_arm", false);
    extract_benchmark_dual_async_ = get_or_declare_parameter<bool>("extract_benchmark_dual_async", false);
    extract_benchmark_csv_path_ = get_or_declare_parameter<std::string>(
      "extract_benchmark_csv_path",
      "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/motion51_extract_replay/extract_all_legal_ik_timing.csv");
    extract_benchmark_record_rollouts_ = get_or_declare_parameter<bool>("extract_benchmark_record_rollouts", false);
    extract_benchmark_candidate_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_benchmark_candidate_limit", 0)));
    extract_benchmark_extract_workers_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_benchmark_extract_workers", 1)));
    extract_ik_dedup_enabled_ = get_or_declare_parameter<bool>("extract_ik_dedup_enabled", false);
    extract_ik_dedup_joint_threshold_ =
      get_or_declare_parameter<double>("extract_ik_dedup_joint_threshold_deg", 1.0) * M_PI / 180.0;
    extract_ik_dedup_h_threshold_ = get_or_declare_parameter<double>("extract_ik_dedup_h_threshold", 0.005);
    extract_benchmark_plan_loaded_after_success_ =
      get_or_declare_parameter<bool>("extract_benchmark_plan_loaded_after_success", false);
    extract_loaded_planning_group_ =
      get_or_declare_parameter<std::string>("extract_loaded_planning_group", "dual_v5_arm_with_base");
    extract_loaded_planning_time_ = get_or_declare_parameter<double>("extract_loaded_planning_time", 1.0);
    extract_loaded_planning_attempts_ =
      std::max(1, get_or_declare_parameter<int>("extract_loaded_planning_attempts", 8));
    extract_loaded_use_direct_pipeline_ =
      get_or_declare_parameter<bool>("extract_loaded_use_direct_pipeline", false);
    extract_loaded_parallel_workers_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_loaded_parallel_workers", 1)));
    extract_loaded_candidate_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_loaded_candidate_limit", 0)));
    extract_loaded_sort_by_pose_distance_ =
      get_or_declare_parameter<bool>("extract_loaded_sort_by_pose_distance", false);
    extract_loaded_stop_on_first_success_ =
      get_or_declare_parameter<bool>("extract_loaded_stop_on_first_success", false);
    extract_loaded_target_updown_ = get_or_declare_parameter<double>("extract_loaded_target_updown", 0.3);
    extract_loaded_lateral_shift_enabled_ =
      get_or_declare_parameter<bool>("extract_loaded_lateral_shift_enabled", false);
    extract_loaded_lateral_shift_distance_ =
      get_or_declare_parameter<double>("extract_loaded_lateral_shift_distance", 0.4);
    extract_loaded_lateral_shift_step_ =
      get_or_declare_parameter<double>("extract_loaded_lateral_shift_step", 0.0);
    extract_loaded_lateral_shift_column_ =
      get_or_declare_parameter<int>("extract_loaded_lateral_shift_column", 3);
    extract_loaded_pre_lower_left_box_id_ =
      get_or_declare_parameter<int>("extract_loaded_pre_lower_left_box_id", 0);
    extract_loaded_pre_lower_right_box_id_ =
      get_or_declare_parameter<int>("extract_loaded_pre_lower_right_box_id", 0);
    extract_loaded_pre_lower_updown_delta_ =
      get_or_declare_parameter<double>("extract_loaded_pre_lower_updown_delta", 0.0);
    enforce_loaded_plan_aabb_clearance_ =
      get_or_declare_parameter<bool>("enforce_loaded_plan_aabb_clearance", true);
    extract_use_independent_kdl_ = get_or_declare_parameter<bool>("extract_use_independent_kdl", true);
    extract_independent_kdl_max_iterations_ =
      std::max(1, get_or_declare_parameter<int>("extract_independent_kdl_max_iterations", 120));
    extract_independent_kdl_eps_ =
      get_or_declare_parameter<double>("extract_independent_kdl_eps", 1e-5);
    extract_independent_kdl_seed_attempts_ =
      std::max(1, get_or_declare_parameter<int>("extract_independent_kdl_seed_attempts", 1));
    extract_independent_kdl_seed_jitter_ =
      get_or_declare_parameter<double>("extract_independent_kdl_seed_jitter_deg", 8.0) * M_PI / 180.0;
    record_tip_error_ik_candidates_ = get_or_declare_parameter<bool>("record_tip_error_ik_candidates", false);
    record_tip_error_ik_candidate_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("record_tip_error_ik_candidate_limit", 80)));

    left_loaded_pose_family_ = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "loaded_left_pose_family_deg",
      "[-0.0,59.04,-135.16,0.0,-76.13,0.0];[0.0,-75.0,135.0,0.0,60.0,0.0];[33.87,75.82,-135.08,0.0,-59.25,-33.87]"));
    right_loaded_pose_family_ = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "loaded_right_pose_family_deg",
      "[0.0,58.88,-134.84,0.0,-75.96,0.0];[0.0,-75.0,135.0,0.0,60.0,0.0];[-30.93,74.17,-134.92,0.0,-60.74,30.93]"));
    if (left_loaded_pose_family_.empty()) {
      left_loaded_pose_family_.push_back(deg_to_rad({0, -75, 135, 0, 60, 0}));
    }
    if (right_loaded_pose_family_.empty()) {
      right_loaded_pose_family_.push_back(deg_to_rad({0, -75, 135, 0, 60, 0}));
    }
    const size_t loaded_preferred_index = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("loaded_preferred_pose_index", 1)));
    left_preferred_loaded_pose_index_ = std::min(loaded_preferred_index, left_loaded_pose_family_.size() - 1);
    right_preferred_loaded_pose_index_ = std::min(loaded_preferred_index, right_loaded_pose_family_.size() - 1);
    ik_config_.left_loaded_pose_family = left_loaded_pose_family_;
    ik_config_.right_loaded_pose_family = right_loaded_pose_family_;
    ik_config_.left_preferred_loaded_pose_index = left_preferred_loaded_pose_index_;
    ik_config_.right_preferred_loaded_pose_index = right_preferred_loaded_pose_index_;

    left_pregrasp_arm_ = deg_to_rad({0, -90, 135, -45, 0, 0});
    right_pregrasp_arm_ = deg_to_rad({0, -90, 135, 45, 0, 0});
    left_loaded_arm_ = left_loaded_pose_family_[left_preferred_loaded_pose_index_];
    right_loaded_arm_ = right_loaded_pose_family_[right_preferred_loaded_pose_index_];

    loaded_pose_selector_config_.left_pose_family = left_loaded_pose_family_;
    loaded_pose_selector_config_.right_pose_family = right_loaded_pose_family_;
    loaded_pose_selector_config_.left_preferred_index = left_preferred_loaded_pose_index_;
    loaded_pose_selector_config_.right_preferred_index = right_preferred_loaded_pose_index_;
    loaded_pose_selector_config_.target_updown = extract_loaded_target_updown_;

    joint_state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions joint_state_sub_options;
    joint_state_sub_options.callback_group = joint_state_callback_group_;
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = msg;
      },
      joint_state_sub_options);

    if (execution_backend_ == "alfa_execution_bridge") {
      execution_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        shared_from_this(), execution_action_name_);
    } else if (execution_backend_ != "moveit") {
      throw std::runtime_error(
        "Unsupported execution_backend '" + execution_backend_ +
        "'; expected 'moveit' or 'alfa_execution_bridge'");
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group_->setGoalJointTolerance(joint_goal_tolerance_rad_);

    loaded_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), extract_loaded_planning_group_);
    loaded_move_group_->setPlanningTime(extract_loaded_planning_time_);
    loaded_move_group_->setNumPlanningAttempts(extract_loaded_planning_attempts_);
    loaded_move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    loaded_move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    loaded_move_group_->setGoalJointTolerance(joint_goal_tolerance_rad_);

    robot_model_ = move_group_->getRobotModel();
    joint_group_ = robot_model_->getJointModelGroup(planning_group_);
    if (!joint_group_) {
      throw std::runtime_error("No JointModelGroup named " + planning_group_);
    }
    left_arm_group_ = robot_model_->getJointModelGroup("left_v5_arm");
    right_arm_group_ = robot_model_->getJointModelGroup("right_v5_arm");
    if (!left_arm_group_ || !right_arm_group_) {
      throw std::runtime_error("Missing single-arm JointModelGroup left_v5_arm/right_v5_arm");
    }

    loaded_pose_selector_config_.enforce_bounds_group = joint_group_;
    loaded_pose_selector_ = std::make_unique<LoadedPoseSelector>(loaded_pose_selector_config_);

    optimized_ik_solver_ = std::make_unique<ik_benchmark::ParallelUpdownAwareIkSolver>(ik_config_);
    optimized_dual_ik_solver_ = std::make_unique<OptimizedDualIkSolver>(optimized_dual_ik_solver_config());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    if (!planning_scene_monitor_->getPlanningScene()) {
      RCLCPP_WARN(get_logger(), "PlanningSceneMonitor init failed; collision checks disabled");
    } else {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->requestPlanningSceneState();
    }

    if (extract_loaded_use_direct_pipeline_) {
      const std::vector<std::string> request_adapters = {
        "default_planner_request_adapters/AddTimeOptimalParameterization",
        "default_planner_request_adapters/ResolveConstraintFrames",
        "default_planner_request_adapters/FixWorkspaceBounds",
        "default_planner_request_adapters/FixStartStateBounds",
        "default_planner_request_adapters/FixStartStateCollision",
        "default_planner_request_adapters/FixStartStatePathConstraints",
      };
      loaded_planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
        robot_model_, shared_from_this(), "ompl", "ompl_interface/OMPLPlanner", request_adapters);
      loaded_planning_pipeline_->displayComputedMotionPlans(false);
      loaded_planning_pipeline_->publishReceivedRequests(false);
      loaded_planning_pipeline_->checkSolutionPaths(true);
      RCLCPP_INFO(
        get_logger(),
        "Loaded pose direct planning pipeline ready: plugin=%s workers=%zu attempts=%d time=%.3fs",
        loaded_planning_pipeline_->getPlannerPluginName().c_str(),
        extract_loaded_parallel_workers_,
        extract_loaded_planning_attempts_,
        extract_loaded_planning_time_);
    }

    scene_adapter_ = std::make_unique<MotionSceneAdapter>(motion_scene_adapter_config());
    extract_motion_planner_ = std::make_unique<ExtractMotionPlanner>(extract_motion_planner_config());
    extract_candidate_scorer_ = std::make_unique<ExtractCandidateScorer>(extract_candidate_scorer_config());
    extract_candidate_solver_ = std::make_unique<ExtractCandidateSolver>(extract_candidate_solver_config());
    std::string extract_solver_error;
    if (!extract_candidate_solver_->initialize(&extract_solver_error)) {
      throw std::runtime_error("Failed to initialize extract candidate solver: " + extract_solver_error);
    }
    loaded_pose_planner_ = std::make_unique<LoadedPosePlanner>(loaded_pose_planner_config());
    extract_rollout_planner_ = std::make_unique<ExtractRolloutPlanner>(extract_rollout_planner_config());
    ik_candidate_selector_ = std::make_unique<IkCandidateSelector>(ik_candidate_selector_config());
    apply_container_obstacles();
    set_static_box_wall_opening(extract_demo_left_box_id_, extract_demo_right_box_id_, "initial");

    demo_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const bool ok = run_one_pair_flow(5, 6, false, 1);
        response->success = ok;
        response->message = ok ? "one-pair MoveIt flow finished" : last_error_;
      });

    box_stack_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/run_box_stack_flow",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(get_logger(), "Received /%s/run_box_stack_flow request", get_name());
        const bool ok = run_box_stack_flow();
        response->success = ok;
        response->message = ok ? "box-stack MoveIt flow finished" : last_error_;
      });

    extract_demo_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/run_left_extract_demo",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(get_logger(), "Received /%s/run_left_extract_demo request", get_name());
        const bool ok = run_left_extract_demo();
        response->success = ok;
        response->message = ok ? "left extract primitive finished" : last_error_;
      });

    extract_monitor_next_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/run_extract_monitor_next",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(get_logger(), "Received /%s/run_extract_monitor_next request", get_name());
        std::string message;
        const bool ok = run_extract_monitor_next(&message);
        response->success = ok;
        response->message = ok ? message : (message.empty() ? last_error_ : message);
      });

    extract_monitor_full_selected_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/run_extract_monitor_full_selected",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(get_logger(), "Received /%s/run_extract_monitor_full_selected request", get_name());
        std::string message;
        const bool ok = run_extract_monitor_full_selected(&message);
        response->success = ok;
        response->message = ok ? message : (message.empty() ? last_error_ : message);
      });

    RCLCPP_INFO(get_logger(), "DualArmPlannerNode ready");
    RCLCPP_INFO(get_logger(), "  group=%s execute=%s backend=%s box_front_x=%.3f max_rounds=%d include_top=%s",
                planning_group_.c_str(), execute_ ? "true" : "false", execution_backend_.c_str(),
                box_front_x_, max_rounds_,
                include_top_suction_ ? "true" : "false");
    RCLCPP_INFO(get_logger(),
                "  Services: /%s/plan_and_execute, /%s/run_box_stack_flow, /%s/run_left_extract_demo, /%s/run_extract_monitor_next, /%s/run_extract_monitor_full_selected",
                get_name(), get_name(), get_name(), get_name(), get_name());
    RCLCPP_INFO(get_logger(),
                "  IK strategy=fixed_discrete h=%zu seed=%zu workers=%zu timeout=%.3fs collision=%s",
                ik_config_.h_candidate_count, ik_config_.seed_count, ik_config_.workers, ik_config_.timeout,
                ik_config_.check_collision ? "true" : "false");
    RCLCPP_INFO(get_logger(),
                "  loaded pose prior left=%zu right=%zu preferred=(%zu,%zu) weights=(family %.3f, preferred %.3f)",
                left_loaded_pose_family_.size(), right_loaded_pose_family_.size(),
                left_preferred_loaded_pose_index_, right_preferred_loaded_pose_index_,
                ik_config_.cost_loaded_family_distance, ik_config_.cost_loaded_preferred_distance);
    RCLCPP_INFO(get_logger(),
                "  Extract primitive IK=%s fixed-updown timeout=%.3fs pos_tol=%.3fm ori_tol=%.3frad",
                extract_use_independent_kdl_ ? "independent Orocos KDL chains" : "MoveIt setFromIK/KDL plugin",
                extract_kdl_timeout_, extract_position_tolerance_, extract_orientation_tolerance_);
    if (!extract_use_independent_kdl_ && extract_benchmark_extract_workers_ > 1) {
      RCLCPP_WARN(
        get_logger(),
        "  Extract primitive IK is using MoveIt setFromIK with %zu workers; this path is mutex-guarded and can serialize rollout IK. "
        "Use extract_use_independent_kdl:=true for the benchmarked parallel extract path.",
        extract_benchmark_extract_workers_);
    }
    if (extract_use_independent_kdl_) {
      RCLCPP_INFO(get_logger(), "  Independent KDL settings: max_iterations=%d eps=%.2e",
                  extract_independent_kdl_max_iterations_, extract_independent_kdl_eps_);
      RCLCPP_INFO(get_logger(), "  Independent KDL seeds: attempts=%d jitter=%.2fdeg",
                  extract_independent_kdl_seed_attempts_, extract_independent_kdl_seed_jitter_ * 180.0 / M_PI);
    }
    RCLCPP_INFO(get_logger(), "  speed scale velocity=%.2f acceleration=%.2f",
                velocity_scale_, acceleration_scale_);
    RCLCPP_INFO(get_logger(), "  attached carried-box collision=%s size=(%.2f, %.2f, %.2f)",
                enable_attached_box_collision_ ? "true" : "false",
                carried_box_depth_, carried_box_width_, carried_box_height_);

    open_record_file();
  }

private:
  template<typename T>
  T get_or_declare_parameter(const std::string& name, const T& default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    if (!get_parameter(name, value)) {
      return default_value;
    }
    return value;
  }

  moveit::core::RobotStatePtr get_current_robot_state()
  {
    if (prefer_commanded_state_ && last_commanded_state_) {
      return std::make_shared<moveit::core::RobotState>(*last_commanded_state_);
    }

    if (execute_ && move_group_) {
      auto current_state = move_group_->getCurrentState(1.0);
      if (current_state) {
        current_state->update();
        return current_state;
      }
    }

    sensor_msgs::msg::JointState::SharedPtr joint_state_msg;
    for (int retry = 0; retry < 30; ++retry) {
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        joint_state_msg = latest_joint_state_;
      }
      if (joint_state_msg) break;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    auto state = std::make_shared<moveit::core::RobotState>(robot_model_);
    state->setToDefaultValues();

    if (!joint_state_msg) {
      RCLCPP_WARN(get_logger(), "No /joint_states received; using model default state");
      state->update();
      return state;
    }

    for (size_t i = 0; i < joint_state_msg->name.size() && i < joint_state_msg->position.size(); ++i) {
      if (robot_model_->hasJointModel(joint_state_msg->name[i])) {
        state->setJointPositions(joint_state_msg->name[i], &joint_state_msg->position[i]);
      }
    }
    state->update();
    return state;
  }

  bool is_state_valid(const moveit::core::RobotState& state, bool check_collision) const
  {
    if (!state.satisfiesBounds(joint_group_)) return false;
    if (!check_collision) return true;
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) return true;
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    return !scene->isStateColliding(state, joint_group_->getName());
  }

  static bool robot_state_has_attached_body(const moveit::core::RobotState& state)
  {
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    state.getAttachedBodies(attached_bodies);
    return !attached_bodies.empty();
  }

  bool is_state_valid_with_attached_boxes(
    const moveit::core::RobotState& state,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    bool check_collision,
    std::string* reason = nullptr) const
  {
    if (!state.satisfiesBounds(joint_group_)) {
      if (reason) *reason = "robot state out of bounds";
      return false;
    }
    if (!check_collision) return true;
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) return true;

    moveit::core::RobotState collision_state(state);
    if (enable_attached_box_collision_ && !attached_boxes.empty()) {
      std::vector<AttachedBoxSpec> missing_boxes;
      missing_boxes.reserve(attached_boxes.size());
      for (const auto& box : attached_boxes) {
        if (!collision_state.hasAttachedBody(box.id)) {
          missing_boxes.push_back(box);
        }
      }
      attach_boxes_to_robot_state(collision_state, missing_boxes, attached_box_collision_padding_);
    }
    collision_state.update(true);

    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = true;
    request.max_contacts = 10;
    scene->checkCollision(request, result, collision_state);
    if (!result.collision) return true;

    if (reason) {
      *reason = "robot/carried box state colliding";
      if (!result.contacts.empty()) {
        const auto& pair = result.contacts.begin()->first;
        *reason += ": " + pair.first + " <-> " + pair.second;
      }
    }
    return false;
  }

  std::string scene_collision_reason(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group) const
  {
    if (!scene) return "scene_missing";
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = true;
    request.max_contacts = 5;
    request.max_contacts_per_pair = 1;
    if (group) {
      request.group_name = group->getName();
    }
    scene->checkCollision(request, result, state);
    if (!result.collision) return "";
    std::ostringstream out;
    out << "collision";
    size_t count = 0;
    for (const auto& entry : result.contacts) {
      if (count == 0) {
        out << ":";
      } else {
        out << ",";
      }
      out << entry.first.first << "<->" << entry.first.second;
      ++count;
      if (count >= 3) break;
    }
    return out.str();
  }

  std::string group_bounds_reason(
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group) const
  {
    if (!group) return "bounds_missing_group";
    if (state.satisfiesBounds(group)) return "";
    std::ostringstream out;
    out << "bounds";
    size_t count = 0;
    for (const auto& name : group->getVariableNames()) {
      const auto& bounds = robot_model_->getVariableBounds(name);
      const double value = state.getVariablePosition(name);
      bool bad = false;
      if (bounds.position_bounded_) {
        bad = value < bounds.min_position_ - 1e-9 || value > bounds.max_position_ + 1e-9;
      }
      if (!bad) continue;
      out << (count == 0 ? ":" : ",")
          << name << "=" << value
          << "[" << bounds.min_position_ << "," << bounds.max_position_ << "]";
      ++count;
      if (count >= 4) break;
    }
    return out.str();
  }

  std::string direct_pipeline_failure_diagnostic(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const moveit::core::JointModelGroup* group) const
  {
    if (!group) return "diagnostic=missing_group";

    auto state_status = [&](const char* label, const moveit::core::RobotState& state) {
      std::ostringstream out;
      const std::string bounds = group_bounds_reason(state, group);
      out << label << "_bounds=" << (bounds.empty() ? "ok" : bounds);
      const std::string collision = scene_collision_reason(scene, state, group);
      out << "," << label << "_collision=" << (collision.empty() ? "clear" : collision);
      return out.str();
    };

    std::ostringstream out;
    out << "diagnostic{"
        << state_status("start", start_state) << ";"
        << state_status("goal", goal_state);

    if (!start_state.satisfiesBounds(group) || !goal_state.satisfiesBounds(group)) {
      out << ";line=skipped_bounds}";
      return out.str();
    }
    const std::string start_collision = scene_collision_reason(scene, start_state, group);
    const std::string goal_collision = scene_collision_reason(scene, goal_state, group);
    if (!start_collision.empty() || !goal_collision.empty()) {
      out << ";line=skipped_endpoint_collision}";
      return out.str();
    }

    moveit::core::RobotState probe(start_state);
    const auto& variable_names = group->getVariableNames();
    constexpr int kInterpolationSteps = 50;
    for (int step = 1; step < kInterpolationSteps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(kInterpolationSteps);
      for (const auto& name : variable_names) {
        const double start_value = start_state.getVariablePosition(name);
        const double goal_value = goal_state.getVariablePosition(name);
        const auto* variable_joint = robot_model_->getJointOfVariable(name);
        const bool angular_variable =
          variable_joint && variable_joint->getType() != moveit::core::JointModel::PRISMATIC;
        const double delta = angular_variable ?
          shortest_angular_distance(start_value, goal_value) :
          (goal_value - start_value);
        probe.setVariablePosition(name, start_value + delta * t);
      }
      probe.update(true);
      const std::string bounds = group_bounds_reason(probe, group);
      if (!bounds.empty()) {
        out << ";line=first_" << bounds << "@" << step << "/" << kInterpolationSteps << "}";
        return out.str();
      }
      const std::string collision = scene_collision_reason(scene, probe, group);
      if (!collision.empty()) {
        out << ";line=first_" << collision << "@" << step << "/" << kInterpolationSteps << "}";
        return out.str();
      }
    }

    out << ";line=straight_joint_interpolation_clear}";
    return out.str();
  }

  ContainerGeometryConfig container_geometry_config() const
  {
    return {
      container_center_x_,
      container_center_y_ + scene_y_shift_,
      container_width_,
      container_height_,
      container_length_,
      container_wall_thickness_,
      container_floor_z_,
    };
  }

  BoxWallGeometryConfig box_wall_geometry_config() const
  {
    return {
      box_front_x_,
      scene_y_shift_,
      container_center_y_ + scene_y_shift_,
      container_width_,
      container_floor_z_,
      carried_box_width_,
      carried_box_height_,
      carried_box_depth_,
      static_box_obstacle_inset_,
    };
  }

  CarriedBoxGeometryConfig carried_box_geometry_config() const
  {
    return {
      carried_box_width_,
      carried_box_height_,
      carried_box_depth_,
    };
  }

  ExtractMotionPlannerConfig extract_motion_planner_config() const
  {
    return {
      extract_step_x_,
      extract_max_x_,
    };
  }

  ExtractCandidateScorerConfig extract_candidate_scorer_config() const
  {
    return {
      left_arm_group_,
      right_arm_group_,
      left_tip_,
      right_tip_,
      extract_step_x_,
      extract_score_lift_weight_,
      extract_score_pitch_weight_,
      extract_score_retreat_continuity_weight_,
      extract_score_joint_delta_weight_,
      extract_score_tip_position_delta_weight_,
      extract_score_tip_orientation_delta_weight_,
    };
  }

  ExtractCandidateSolverConfig extract_candidate_solver_config() const
  {
    return {
      robot_model_,
      joint_group_,
      left_arm_group_,
      right_arm_group_,
      left_tip_,
      right_tip_,
      extract_use_independent_kdl_,
      extract_kdl_timeout_,
      extract_position_tolerance_,
      extract_orientation_tolerance_,
      extract_max_tip_z_drop_,
      extract_min_tool_normal_z_,
      extract_max_joint_delta_,
      extract_independent_kdl_max_iterations_,
      extract_independent_kdl_eps_,
      extract_independent_kdl_seed_attempts_,
      extract_independent_kdl_seed_jitter_,
    };
  }

  ExtractRolloutPlannerConfig extract_rollout_planner_config()
  {
    ExtractRolloutPlannerConfig config;
    config.motion_planner = extract_motion_planner_.get();
    config.candidate_solver = extract_candidate_solver_.get();
    config.candidate_scorer = extract_candidate_scorer_.get();
    config.joint_group = joint_group_;
    config.left_arm_group = left_arm_group_;
    config.right_arm_group = right_arm_group_;
    config.left_tip = left_tip_;
    config.right_tip = right_tip_;
    config.fail_fast = extract_fail_fast_;
    config.dual_async = extract_benchmark_dual_async_;
    config.success_extra_steps = extract_success_extra_steps_;
    config.single_clear_callback =
      [this](
        const moveit::core::RobotState& state,
        const AttachedBoxSpec& box,
        int box_id,
        bool* detached,
        std::string* reason) {
        return state_clear_for_single_extract(state, box, box_id, detached, reason);
      };
    config.dual_clear_callback =
      [this](
        const moveit::core::RobotState& state,
        const AttachedBoxSpec& left_box,
        int left_box_id,
        const AttachedBoxSpec& right_box,
        int right_box_id,
        bool* left_detached,
        bool* right_detached,
        std::string* reason) {
        return state_clear_for_dual_extract(
          state, left_box, left_box_id, right_box, right_box_id,
          left_detached, right_detached, reason);
      };
    config.trajectory_clear_callback =
      [this](
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const moveit::core::RobotState& start_state,
        const std::vector<AttachedBoxSpec>& attached_boxes,
        std::string* reason) {
        return planned_trajectory_clear_in_full_scene(plan, start_state, attached_boxes, reason);
      };
    return config;
  }

  IkCandidateSelectorConfig ik_candidate_selector_config() const
  {
    return {
      extract_ik_dedup_enabled_,
      extract_ik_dedup_joint_threshold_,
      extract_ik_dedup_h_threshold_,
      extract_benchmark_candidate_limit_,
    };
  }

  OptimizedDualIkSolverConfig optimized_dual_ik_solver_config() const
  {
    return {
      optimized_ik_solver_.get(),
      robot_model_.get(),
      joint_group_,
      fixed_updown_,
    };
  }

  MotionSceneAdapterConfig motion_scene_adapter_config() const
  {
    MotionSceneAdapterConfig config;
    config.enable_container_obstacle = enable_container_obstacle_;
    config.frame = container_frame_;
    config.container = container_geometry_config();
    config.enable_static_box_obstacles = enable_static_box_obstacles_;
    config.box_wall = box_wall_geometry_config();
    config.enable_attached_box_collision = enable_attached_box_collision_;
    config.carried_box = carried_box_geometry_config();
    config.left_tip = left_tip_;
    config.right_tip = right_tip_;
    return config;
  }

  LoadedPosePlannerConfig loaded_pose_planner_config()
  {
    LoadedPosePlannerConfig config;
    config.move_group = loaded_move_group_.get();
    config.selector = loaded_pose_selector_.get();
    config.scene_adapter = scene_adapter_.get();
    config.target_joint_names = arm_joint_target_names();
    config.attached_box_collision_padding = attached_box_collision_padding_;
    config.lateral_shift_enabled = extract_loaded_lateral_shift_enabled_;
    config.lateral_shift_distance = extract_loaded_lateral_shift_distance_;
    config.lateral_shift_step = extract_loaded_lateral_shift_step_ > 0.0
      ? extract_loaded_lateral_shift_step_
      : extract_step_x_;
    config.lateral_shift_column = extract_loaded_lateral_shift_column_;
    config.pre_loaded_lower_left_box_id = extract_loaded_pre_lower_left_box_id_;
    config.pre_loaded_lower_right_box_id = extract_loaded_pre_lower_right_box_id_;
    config.pre_loaded_lower_updown_delta = extract_loaded_pre_lower_updown_delta_;
    config.fixed_updown = extract_loaded_target_updown_;
    config.min_tool_normal_z = extract_min_tool_normal_z_;
    config.max_joint_delta = extract_max_joint_delta_;
    config.lateral_shift_solver = extract_candidate_solver_.get();
    if (extract_loaded_use_direct_pipeline_) {
      config.direct_plan_callback = [this](
        const std::string& stage_name,
        const moveit::core::RobotState& start_state,
        const moveit::core::RobotState& goal_state,
        moveit::planning_interface::MoveGroupInterface::Plan* plan,
        std::string* reason) {
        return plan_loaded_pose_with_direct_pipeline(stage_name, start_state, goal_state, plan, reason);
      };
    }
    config.clearance_callback = [this](
      const moveit::planning_interface::MoveGroupInterface::Plan& plan,
      const moveit::core::RobotState& start_state,
      std::string* reason) {
      return planned_trajectory_clear_in_full_scene(plan, start_state, active_attached_boxes(), reason);
    };
    config.record_callback = [this](
      const std::string& stage_name,
      const moveit::planning_interface::MoveGroupInterface::Plan& plan,
      const moveit::core::RobotState& start_state,
      const moveit::core::RobotState& goal_state,
      const std::vector<std::string>& target_names,
      const nlohmann::json& extra) {
      if (recording_enabled()) {
        record_stage(stage_name, plan, start_state, goal_state, target_names, extra);
      }
    };
    return config;
  }

  LoadedPoseBatchPlanOptions loaded_pose_batch_plan_options() const
  {
    LoadedPoseBatchPlanOptions options;
    options.enabled = extract_benchmark_plan_loaded_after_success_;
    options.sort_by_pose_distance = extract_loaded_sort_by_pose_distance_;
    options.stop_on_first_success = extract_loaded_stop_on_first_success_;
    options.candidate_limit = extract_loaded_candidate_limit_;
    options.parallel_workers = extract_loaded_use_direct_pipeline_ ? extract_loaded_parallel_workers_ : 1;
    return options;
  }

  bool plan_loaded_pose_with_direct_pipeline(
    const std::string& stage_name,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    moveit::planning_interface::MoveGroupInterface::Plan* plan,
    std::string* reason) const
  {
    if (!plan) {
      if (reason) *reason = "direct_pipeline_plan_output_null";
      return false;
    }
    if (!loaded_planning_pipeline_) {
      if (reason) *reason = "direct_pipeline_not_initialized";
      return false;
    }
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
      if (reason) *reason = "direct_pipeline_planning_scene_not_initialized";
      return false;
    }
    const auto* loaded_group = robot_model_->getJointModelGroup(extract_loaded_planning_group_);
    if (!loaded_group) {
      if (reason) *reason = "direct_pipeline_missing_group_" + extract_loaded_planning_group_;
      return false;
    }

    planning_scene::PlanningScenePtr scene_snapshot;
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
      if (!locked_scene) {
        if (reason) *reason = "direct_pipeline_planning_scene_lock_failed";
        return false;
      }
      scene_snapshot = planning_scene::PlanningScene::clone(
        static_cast<const planning_scene::PlanningSceneConstPtr&>(locked_scene));
    }
    scene_snapshot->setCurrentState(start_state);
    if (scene_adapter_) {
      scene_adapter_->applyToPlanningSceneSnapshot(*scene_snapshot, active_attached_boxes());
      scene_snapshot->setCurrentState(start_state);
    }

    planning_interface::MotionPlanRequest request;
    request.group_name = extract_loaded_planning_group_;
    request.allowed_planning_time = extract_loaded_planning_time_;
    request.num_planning_attempts = extract_loaded_planning_attempts_;
    request.max_velocity_scaling_factor = velocity_scale_;
    request.max_acceleration_scaling_factor = acceleration_scale_;
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state, true);
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal_state, loaded_group, joint_goal_tolerance_rad_));

    planning_interface::MotionPlanResponse response;
    const bool generated = loaded_planning_pipeline_->generatePlan(scene_snapshot, request, response);
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        !response.trajectory_) {
      if (reason) {
        *reason = "direct_pipeline_planning_failed_code_" +
          std::to_string(response.error_code_.val) + " " +
          direct_pipeline_failure_diagnostic(scene_snapshot, start_state, goal_state, loaded_group);
      }
      return false;
    }

    moveit::core::robotStateToRobotStateMsg(start_state, plan->start_state_, true);
    response.trajectory_->getRobotTrajectoryMsg(plan->trajectory_);
    plan->planning_time_ = response.planning_time_;
    if (plan->trajectory_.joint_trajectory.points.empty()) {
      if (reason) *reason = "direct_pipeline_empty_trajectory";
      return false;
    }
    (void)stage_name;
    return true;
  }

  BoxStackFlowConfig box_stack_flow_config() const
  {
    return {
      box_front_x_,
      scene_y_shift_,
      fixed_updown_,
      include_top_suction_,
      max_rounds_,
      left_pregrasp_arm_,
      right_pregrasp_arm_,
      left_loaded_arm_,
      right_loaded_arm_,
      extract_demo_pair_sequence_,
    };
  }

  BoxStackFlowCallbacks box_stack_flow_callbacks()
  {
    BoxStackFlowCallbacks callbacks;
    callbacks.clear_scene = [this]() {
      clear_carried_boxes_from_scene();
    };
    callbacks.set_wall_opening = [this](int left_box_id, int right_box_id, const std::string& reason) {
      return set_static_box_wall_opening(left_box_id, right_box_id, reason);
    };
    callbacks.plan_joint_target = [this](
      const std::string& stage_name,
      double updown,
      const std::vector<double>& left_arm,
      const std::vector<double>& right_arm) {
      return plan_to_joint_target(stage_name, make_dual_arm_joint_target(updown, left_arm, right_arm));
    };
    callbacks.plan_grasp_ik = [this](
      const std::string& stage_name,
      const BoxSpec& left_box,
      const BoxSpec& right_box,
      bool top_suction) {
      const auto left_pose = top_suction ? top_suction_pose(left_box) : front_grasp_pose(left_box);
      const auto right_pose = top_suction ? top_suction_pose(right_box) : front_grasp_pose(right_box);
      return plan_dual_tip_ik(stage_name, left_pose, right_pose, top_suction);
    };
    callbacks.attach_boxes = [this](int left_box_id, int right_box_id, bool top_suction) {
      return attach_carried_boxes(left_box_id, right_box_id, top_suction);
    };
    callbacks.validate_attached_boxes = [this](const std::string& stage_name) {
      if (auto attached_state = get_current_robot_state()) {
        std::string moveit_collision_reason;
        if (!is_state_valid_with_attached_boxes(
              *attached_state,
              active_attached_boxes(),
              true,
              &moveit_collision_reason)) {
          return fail(stage_name + ": carried box collides in MoveIt scene (" +
                      moveit_collision_reason + ")");
        }
        std::string carried_collision_reason;
        if (!carried_boxes_clear_static_obstacles(*attached_state, &carried_collision_reason)) {
          return fail(stage_name + ": carried box collides with static box obstacle (" +
                      carried_collision_reason + ")");
        }
      }
      return true;
    };
    callbacks.detach_boxes = [this]() {
      return detach_carried_boxes();
    };
    callbacks.info = [this](const std::string& message) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    };
    callbacks.fail = [this](const std::string& message) {
      return fail(message);
    };
    return callbacks;
  }

  ExtractDemoConfig extract_demo_config() const
  {
    return {
      extract_demo_all_rows_,
      extract_demo_left_box_id_,
      extract_demo_right_box_id_,
      extract_demo_pair_sequence_,
    };
  }

  ExtractDemoCallbacks extract_demo_callbacks()
  {
    ExtractDemoCallbacks callbacks;
    callbacks.clear_scene = [this]() {
      clear_carried_boxes_from_scene();
    };
    callbacks.reset_commanded_state = [this]() {
      last_commanded_state_.reset();
    };
    callbacks.run_pair = [this](int left_box_id, int right_box_id) {
      return run_left_extract_pair(left_box_id, right_box_id);
    };
    callbacks.record_summary = [this](
      bool success,
      const std::string& error,
      const std::vector<std::pair<int, int>>& pairs) {
      if (!recording_enabled()) return;
      nlohmann::json pair_json = nlohmann::json::array();
      for (const auto& [left_box_id, right_box_id] : pairs) {
        pair_json.push_back({left_box_id, right_box_id});
      }
      recorder_->write(nlohmann::json({
        {"type", "summary"},
        {"success", success},
        {"error", error},
        {"stages", recorded_stage_count()},
        {"pairs", pair_json}
      }));
    };
    callbacks.last_error = [this]() {
      return last_error_;
    };
    callbacks.set_last_error = [this](const std::string& error) {
      last_error_ = error;
    };
    return callbacks;
  }

  ExtractBenchmarkRunnerConfig extract_benchmark_runner_config() const
  {
    ExtractBenchmarkRunnerConfig config;
    config.candidate_selector = ik_candidate_selector_.get();
    config.loaded_pose_planner = loaded_pose_planner_.get();
    config.logger = get_logger();
    config.record_tip_error_ik_candidates = record_tip_error_ik_candidates_;
    config.record_rollouts = extract_benchmark_record_rollouts_;
    config.dual_async = extract_benchmark_dual_async_;
    config.plan_loaded_after_success = extract_benchmark_plan_loaded_after_success_;
    config.candidate_limit = extract_benchmark_candidate_limit_;
    config.extract_workers = extract_benchmark_extract_workers_;
    config.dedup_joint_threshold_rad = extract_ik_dedup_joint_threshold_;
    config.dedup_h_threshold = extract_ik_dedup_h_threshold_;
    config.csv_path = extract_benchmark_csv_path_;
    config.loaded_options = loaded_pose_batch_plan_options();
    return config;
  }

  ExtractBenchmarkRunnerCallbacks extract_benchmark_runner_callbacks()
  {
    ExtractBenchmarkRunnerCallbacks callbacks;
    callbacks.state_from_candidate = [this](
      const moveit::core::RobotState& seed_state,
      const ik_benchmark::UpdownAwareIkCandidate& candidate) {
      return state_from_ik_candidate(seed_state, candidate);
    };
    callbacks.rollout_left = [this](
      const moveit::core::RobotState& start_state,
      const AttachedBoxSpec& left_box,
      int left_box_id,
      size_t candidate_order,
      const ik_benchmark::UpdownAwareIkCandidate& ik_candidate,
      const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step) {
      return rollout_left_extract_from_state(start_state, left_box, left_box_id, candidate_order, ik_candidate, record_step);
    };
    callbacks.rollout_dual = [this](
      const moveit::core::RobotState& start_state,
      const AttachedBoxSpec& left_box,
      int left_box_id,
      const AttachedBoxSpec& right_box,
      int right_box_id,
      size_t candidate_order,
      const ik_benchmark::UpdownAwareIkCandidate& ik_candidate,
      const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step) {
      return rollout_dual_extract_from_state(
        start_state, left_box, left_box_id, right_box, right_box_id,
        candidate_order, ik_candidate, record_step);
    };
    callbacks.fill_loaded_metrics = [this](ExtractRolloutTiming& timing) {
      fill_loaded_pose_distance_metrics(timing);
    };
    callbacks.record_keyframe = [this](
      const std::string& stage_name,
      const moveit::core::RobotState& state,
      const std::vector<AttachedBoxSpec>& boxes,
      const nlohmann::json& extra) {
      record_extract_keyframe(stage_name, state, boxes, extra);
    };
    callbacks.record_tip_errors = [this](
      const std::string& prefix,
      const moveit::core::RobotState& seed_state,
      const ik_benchmark::UpdownAwareIkResult& ik_result,
      const AttachedBoxSpec& left_box) {
      record_tip_error_ik_candidates(prefix, seed_state, ik_result, left_box);
    };
    callbacks.record_summary = [this](const nlohmann::json& summary) {
      if (recording_enabled()) {
        recorder_->write(summary);
      }
    };
    callbacks.rejection_counts_json = [this](const ik_benchmark::UpdownAwareIkResult& result) {
      return ik_candidate_rejection_counts_json(result);
    };
    callbacks.fail = [this](const std::string& message) {
      return fail(message);
    };
    callbacks.set_last_error = [this](const std::string& message) {
      last_error_ = message;
    };
    return callbacks;
  }

  std::vector<ContainerPanel> container_panels() const
  {
    return scene_adapter_ ? scene_adapter_->containerPanels() : make_container_panels(container_geometry_config());
  }

  void apply_container_obstacles()
  {
    if (!enable_container_obstacle_) {
      RCLCPP_INFO(get_logger(), "Container obstacle disabled");
      return;
    }
    if (!scene_adapter_) return;

    if (scene_adapter_->applyContainerObstacles()) {
      RCLCPP_INFO(get_logger(),
                  "Applied container obstacle: frame=%s length=%.2f width=%.2f height=%.2f panels=%zu",
                  container_frame_.c_str(), container_length_, container_width_, container_height_,
                  container_panels().size());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to apply container obstacle collision objects");
    }
  }

  std::vector<StaticBoxObstacle> make_static_box_wall_obstacles_for_opening(int left_box_id, int right_box_id) const
  {
    if (!enable_static_box_obstacles_) return {};
    return make_box_wall_obstacles_for_opening(left_box_id, right_box_id, box_wall_geometry_config());
  }

  std::vector<StaticBoxObstacle> static_box_obstacles() const
  {
    if (!enable_static_box_obstacles_ || !scene_adapter_) return {};
    return scene_adapter_->staticBoxObstacles();
  }

  const std::vector<AttachedBoxSpec>& active_attached_boxes() const
  {
    static const std::vector<AttachedBoxSpec> empty;
    return scene_adapter_ ? scene_adapter_->activeAttachedBoxes() : empty;
  }

  bool set_static_box_wall_opening(int left_box_id, int right_box_id, const std::string& reason)
  {
    if (!enable_static_box_obstacles_) {
      if (scene_adapter_) scene_adapter_->setStaticBoxWallOpening(left_box_id, right_box_id);
      RCLCPP_INFO(get_logger(), "Static box-wall obstacles disabled");
      return true;
    }

    const bool applied = scene_adapter_ && scene_adapter_->setStaticBoxWallOpening(left_box_id, right_box_id);
    if (!applied || static_box_obstacles().empty()) {
      RCLCPP_WARN(get_logger(), "No dynamic box wall for opening L%d/R%d (%s)",
                  left_box_id, right_box_id, reason.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Using dynamic box wall opening L%d/R%d for %s",
                left_box_id, right_box_id, reason.c_str());
    return true;
  }

  AttachedBoxSpec make_carried_box_spec(const std::string& side, int box_id, bool top_suction) const
  {
    return scene_adapter_
      ? scene_adapter_->makeCarriedBoxSpec(side, box_id, top_suction)
      : make_attached_box_spec(side, box_id, top_suction, carried_box_geometry_config());
  }

  AxisAlignedBox attached_box_world_aabb(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& box) const
  {
    return aabb_from_attached_box_transform(state.getGlobalLinkTransform(box.link_name), box);
  }

  bool apply_attached_box_state(
    const std::vector<AttachedBoxSpec>& specs, int operation, const std::string& action_name)
  {
    if (!enable_attached_box_collision_) return true;
    if (specs.empty()) return true;
    if (!scene_adapter_) return true;
    if (!scene_adapter_->applyAttachedBoxState(specs, operation)) {
      return fail(action_name + ": failed to update attached carried boxes");
    }
    return true;
  }

  bool carried_box_detached_from_neighbors(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int box_id,
    std::string* reason) const
  {
    return alfa_robot::motion::carried_box_detached_from_neighbors(
      attached_box_world_aabb(state, carried_box),
      box_id,
      box_front_x_,
      scene_y_shift_,
      carried_box_width_,
      carried_box_height_,
      carried_box_depth_,
      extract_neighbor_margin_,
      carried_box.id,
      reason);
  }

  bool left_carried_box_detached_from_neighbors(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int left_box_id,
    std::string* reason) const
  {
    return carried_box_detached_from_neighbors(state, carried_box, left_box_id, reason);
  }

  bool carried_boxes_clear_static_obstacles(
    const moveit::core::RobotState& state,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_ || active_attached_boxes().empty()) return true;

    for (const auto& carried_box : active_attached_boxes()) {
      if (!carried_box_clear_scene_obstacles(state, carried_box, reason)) {
        return false;
      }
    }
    return true;
  }

  bool carried_box_clear_scene_obstacles(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_) return true;
    const auto carried_aabb = attached_box_world_aabb(state, carried_box);

    if (enable_static_box_obstacles_) {
      for (const auto& obstacle : static_box_obstacles()) {
        const AxisAlignedBox obstacle_aabb{obstacle.center, obstacle.size};
        if (aabb_overlaps(carried_aabb, obstacle_aabb)) {
          if (reason) *reason = carried_box.id + " overlaps " + obstacle.id;
          return false;
        }
      }
    }

    if (enable_container_obstacle_) {
      for (const auto& panel : container_panels()) {
        const AxisAlignedBox panel_aabb{panel.center, panel.size};
        if (aabb_overlaps(carried_aabb, panel_aabb)) {
          if (reason) *reason = carried_box.id + " overlaps " + panel.id;
          return false;
        }
      }
    }

    return true;
  }

  bool state_clear_for_extract(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& left_carried_box,
    int left_box_id,
    bool* detached,
    std::string* reason) const
  {
    if (!is_state_valid_with_attached_boxes(state, {left_carried_box}, true, reason)) {
      if (reason && reason->empty()) *reason = "robot/carried box state colliding or out of bounds";
      return false;
    }

    std::string carried_reason;
    if (!carried_box_clear_scene_obstacles(state, left_carried_box, &carried_reason)) {
      if (reason) *reason = carried_reason;
      return false;
    }

    std::string detached_reason;
    const bool detached_now = left_carried_box_detached_from_neighbors(state, left_carried_box, left_box_id, &detached_reason);
    if (detached) *detached = detached_now;
    if (!detached_now && reason) *reason = detached_reason;
    return true;
  }

  bool state_clear_for_single_extract(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int box_id,
    bool* detached,
    std::string* reason) const
  {
    if (!is_state_valid_with_attached_boxes(state, {carried_box}, true, reason)) {
      if (reason && reason->empty()) *reason = "robot/carried box state colliding or out of bounds";
      return false;
    }

    std::string carried_reason;
    if (!carried_box_clear_scene_obstacles(state, carried_box, &carried_reason)) {
      if (reason) *reason = carried_reason;
      return false;
    }

    std::string detached_reason;
    const bool detached_now = carried_box_detached_from_neighbors(state, carried_box, box_id, &detached_reason);
    if (detached) *detached = detached_now;
    if (!detached_now && reason) *reason = detached_reason;
    return true;
  }

  bool state_clear_for_dual_extract(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    bool* left_detached,
    bool* right_detached,
    std::string* reason) const
  {
    if (!is_state_valid_with_attached_boxes(state, {left_box, right_box}, true, reason)) {
      if (reason && reason->empty()) *reason = "robot/carried box state colliding or out of bounds";
      return false;
    }

    for (const auto& box : {left_box, right_box}) {
      std::string carried_reason;
      if (!carried_box_clear_scene_obstacles(state, box, &carried_reason)) {
        if (reason) *reason = carried_reason;
        return false;
      }
    }

    std::string left_reason;
    std::string right_reason;
    const bool left_ok = carried_box_detached_from_neighbors(state, left_box, left_box_id, &left_reason);
    const bool right_ok = carried_box_detached_from_neighbors(state, right_box, right_box_id, &right_reason);
    if (left_detached) *left_detached = left_ok;
    if (right_detached) *right_detached = right_ok;
    if ((!left_ok || !right_ok) && reason) {
      *reason = !left_ok ? left_reason : right_reason;
    }
    return true;
  }

  planning_scene::PlanningScenePtr make_full_scene_snapshot(
    const moveit::core::RobotState& start_state,
    const std::vector<AttachedBoxSpec>& attached_boxes) const
  {
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
      return nullptr;
    }
    planning_scene::PlanningScenePtr scene_snapshot;
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
      if (!locked_scene) return nullptr;
      scene_snapshot = planning_scene::PlanningScene::clone(
        static_cast<const planning_scene::PlanningSceneConstPtr&>(locked_scene));
    }
    scene_snapshot->setCurrentState(start_state);
    if (scene_adapter_) {
      scene_adapter_->applyToPlanningSceneSnapshot(*scene_snapshot, attached_boxes);
      scene_snapshot->setCurrentState(start_state);
    }
    return scene_snapshot;
  }

  bool state_clear_in_full_scene(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& state,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    std::string* reason) const
  {
    if (!joint_group_) return true;
    const std::string bounds = group_bounds_reason(state, joint_group_);
    if (!bounds.empty()) {
      if (reason) *reason = bounds;
      return false;
    }
    moveit::core::RobotState collision_state(state);
    if (enable_attached_box_collision_) {
      std::vector<AttachedBoxSpec> missing_boxes;
      missing_boxes.reserve(attached_boxes.size());
      for (const auto& box : attached_boxes) {
        if (!collision_state.hasAttachedBody(box.id)) {
          missing_boxes.push_back(box);
        }
      }
      attach_boxes_to_robot_state(collision_state, missing_boxes, attached_box_collision_padding_);
    }
    collision_state.update(true);
    const std::string collision = scene_collision_reason(scene, collision_state, joint_group_);
    if (!collision.empty()) {
      if (reason) *reason = collision;
      return false;
    }
    if (enforce_loaded_plan_aabb_clearance_) {
      for (const auto& box : attached_boxes) {
        std::string aabb_reason;
        if (!carried_box_clear_scene_obstacles(collision_state, box, &aabb_reason)) {
          if (reason) *reason = "conservative AABB check failed (" + aabb_reason + ")";
          return false;
        }
      }
    }
    return true;
  }

  bool planned_trajectory_clear_in_full_scene(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    std::string* reason) const
  {
    auto scene_snapshot = make_full_scene_snapshot(start_state, attached_boxes);
    if (!scene_snapshot) return true;

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      moveit::core::RobotState state(start_state);
      const auto& point = trajectory.points[point_index];
      for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
        if (is_robot_variable(trajectory.joint_names[i])) {
          state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
        }
      }
      state.update(true);
      std::string point_reason;
      if (!state_clear_in_full_scene(scene_snapshot, state, attached_boxes, &point_reason)) {
        if (reason) {
          *reason = "trajectory point " + std::to_string(point_index) +
                    ": full scene check failed";
          if (!point_reason.empty()) *reason += " (" + point_reason + ")";
        }
        return false;
      }
    }
    return true;
  }

  moveit::planning_interface::MoveGroupInterface::Plan make_interpolated_joint_plan(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    double duration_s) const
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto& trajectory = plan.trajectory_.joint_trajectory;
    trajectory.joint_names = arm_joint_target_names();

    double max_delta = 0.0;
    for (const auto& name : trajectory.joint_names) {
      const double delta = std::abs(joint_variable_delta(name, start_state, goal_state));
      max_delta = std::max(max_delta, delta);
    }
    const size_t steps = std::max<size_t>(2, static_cast<size_t>(std::ceil(max_delta / (5.0 * M_PI / 180.0))) + 1);
    trajectory.points.reserve(steps);
    for (size_t step = 0; step < steps; ++step) {
      const double ratio = steps <= 1 ? 1.0 : static_cast<double>(step) / static_cast<double>(steps - 1);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start = rclcpp::Duration::from_seconds(duration_s * ratio);
      point.positions.reserve(trajectory.joint_names.size());
      for (const auto& name : trajectory.joint_names) {
        const double start = start_state.getVariablePosition(name);
        point.positions.push_back(start + joint_variable_delta(name, start_state, goal_state) * ratio);
      }
      trajectory.points.push_back(std::move(point));
    }
    moveit::core::robotStateToRobotStateMsg(start_state, plan.start_state_, true);
    plan.planning_time_ = 0.0;
    return plan;
  }

  double joint_variable_delta(
    const std::string& name,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state) const
  {
    const double start = start_state.getVariablePosition(name);
    const double goal = goal_state.getVariablePosition(name);
    const auto* variable_joint = robot_model_ ? robot_model_->getJointOfVariable(name) : nullptr;
    const bool angular_variable =
      variable_joint && variable_joint->getType() != moveit::core::JointModel::PRISMATIC;
    return angular_variable ? shortest_angular_distance(start, goal) : (goal - start);
  }

  moveit::planning_interface::MoveGroupInterface::Plan shortcut_joint_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    std::string* reason) const
  {
    const auto& trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.points.size() <= 2) {
      return plan;
    }

    std::vector<moveit::core::RobotState> states;
    states.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) {
      moveit::core::RobotState state(start_state);
      for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
        if (is_robot_variable(trajectory.joint_names[i])) {
          state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
        }
      }
      state.update(true);
      states.push_back(std::move(state));
    }

    std::vector<size_t> kept;
    kept.push_back(0);
    size_t from = 0;
    while (from + 1 < states.size()) {
      size_t best = from + 1;
      for (size_t to = states.size() - 1; to > from + 1; --to) {
        auto candidate = make_interpolated_joint_plan(states[from], states[to], 0.1);
        std::string segment_reason;
        if (planned_trajectory_clear_in_full_scene(candidate, states[from], attached_boxes, &segment_reason)) {
          best = to;
          break;
        }
      }
      kept.push_back(best);
      from = best;
    }

    if (kept.size() >= states.size()) {
      return plan;
    }

    moveit::planning_interface::MoveGroupInterface::Plan out;
    out.start_state_ = plan.start_state_;
    out.planning_time_ = plan.planning_time_;
    auto& out_traj = out.trajectory_.joint_trajectory;
    out_traj.joint_names = trajectory.joint_names;
    const double duration = trajectory.points.empty()
      ? 1.0
      : rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    for (size_t i = 0; i < kept.size(); ++i) {
      const auto& state = states[kept[i]];
      trajectory_msgs::msg::JointTrajectoryPoint point;
      const double ratio = kept.size() <= 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(kept.size() - 1);
      point.time_from_start = rclcpp::Duration::from_seconds(duration * ratio);
      point.positions.reserve(out_traj.joint_names.size());
      for (const auto& name : out_traj.joint_names) {
        point.positions.push_back(state.getVariablePosition(name));
      }
      out_traj.points.push_back(std::move(point));
    }

    if (reason) {
      *reason = "shortcut " + std::to_string(trajectory.points.size()) + " -> " +
                std::to_string(out_traj.points.size()) + " points";
    }
    return out;
  }

  moveit::planning_interface::MoveGroupInterface::Plan densify_joint_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    double max_joint_step_rad,
    double max_updown_step_m) const
  {
    const auto& trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
      return plan;
    }

    moveit::planning_interface::MoveGroupInterface::Plan out;
    out.start_state_ = plan.start_state_;
    out.planning_time_ = plan.planning_time_;
    auto& out_traj = out.trajectory_.joint_trajectory;
    out_traj.joint_names = trajectory.joint_names;

    auto point_time = [](const trajectory_msgs::msg::JointTrajectoryPoint& point) {
      return rclcpp::Duration(point.time_from_start).seconds();
    };
    out_traj.points.push_back(trajectory.points.front());
    for (size_t point_index = 1; point_index < trajectory.points.size(); ++point_index) {
      const auto& previous = trajectory.points[point_index - 1];
      const auto& current = trajectory.points[point_index];
      if (previous.positions.size() != trajectory.joint_names.size() ||
          current.positions.size() != trajectory.joint_names.size()) {
        out_traj.points.push_back(current);
        continue;
      }

      double max_ratio = 0.0;
      for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
        const double delta = std::abs(current.positions[i] - previous.positions[i]);
        const double limit = trajectory.joint_names[i] == "updown"
          ? std::max(1e-4, max_updown_step_m)
          : std::max(1e-4, max_joint_step_rad);
        max_ratio = std::max(max_ratio, delta / limit);
      }
      const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(max_ratio)));
      const double start_time = point_time(previous);
      const double end_time = point_time(current);
      for (size_t step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(start_time + (end_time - start_time) * ratio);
        point.positions.reserve(trajectory.joint_names.size());
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
          point.positions.push_back(previous.positions[i] + (current.positions[i] - previous.positions[i]) * ratio);
        }
        out_traj.points.push_back(std::move(point));
      }
    }
    return out;
  }

  bool plan_joint_space_with_direct_pipeline(
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    moveit::planning_interface::MoveGroupInterface::Plan* plan,
    std::string* reason) const
  {
    if (!plan) {
      if (reason) *reason = "direct_joint_plan_output_null";
      return false;
    }
    if (!loaded_planning_pipeline_) {
      if (reason) *reason = "direct_pipeline_not_initialized";
      return false;
    }
    const auto* loaded_group = robot_model_->getJointModelGroup(extract_loaded_planning_group_);
    if (!loaded_group) {
      if (reason) *reason = "direct_pipeline_missing_group_" + extract_loaded_planning_group_;
      return false;
    }
    auto scene_snapshot = make_full_scene_snapshot(start_state, {});
    if (!scene_snapshot) {
      if (reason) *reason = "direct_pipeline_planning_scene_not_initialized";
      return false;
    }

    planning_interface::MotionPlanRequest request;
    request.group_name = extract_loaded_planning_group_;
    request.allowed_planning_time = extract_loaded_planning_time_;
    request.num_planning_attempts = extract_loaded_planning_attempts_;
    request.max_velocity_scaling_factor = velocity_scale_;
    request.max_acceleration_scaling_factor = acceleration_scale_;
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state, true);
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal_state, loaded_group, joint_goal_tolerance_rad_));

    planning_interface::MotionPlanResponse response;
    const bool generated = loaded_planning_pipeline_->generatePlan(scene_snapshot, request, response);
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        !response.trajectory_) {
      if (reason) {
        *reason = "direct_pipeline_planning_failed_code_" +
          std::to_string(response.error_code_.val) + " " +
          direct_pipeline_failure_diagnostic(scene_snapshot, start_state, goal_state, loaded_group);
      }
      return false;
    }

    moveit::core::robotStateToRobotStateMsg(start_state, plan->start_state_, true);
    response.trajectory_->getRobotTrajectoryMsg(plan->trajectory_);
    plan->planning_time_ = response.planning_time_;
    if (plan->trajectory_.joint_trajectory.points.empty()) {
      if (reason) *reason = "direct_pipeline_empty_trajectory";
      return false;
    }
    return true;
  }

  bool planned_carried_boxes_clear_static_obstacles(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_) return true;
    if (active_attached_boxes().empty() && !robot_state_has_attached_body(start_state)) return true;

    std::string moveit_scene_reason;
    if (!planned_trajectory_clear_in_moveit_scene(plan, start_state, &moveit_scene_reason)) {
      if (reason) *reason = moveit_scene_reason;
      return false;
    }

    if (!enforce_loaded_plan_aabb_clearance_) {
      std::string aabb_reason;
      if (!planned_trajectory_clear_by_aabb(plan, start_state, &aabb_reason)) {
        RCLCPP_WARN(
          get_logger(),
          "Loaded plan passed MoveIt collision check but failed conservative AABB post-check: %s",
          aabb_reason.c_str());
      }
      return true;
    }

    return planned_trajectory_clear_by_aabb(plan, start_state, reason);
  }

  bool planned_trajectory_clear_in_moveit_scene(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_) return true;
    if (active_attached_boxes().empty() && !robot_state_has_attached_body(start_state)) return true;

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      moveit::core::RobotState state(start_state);
      const auto& point = trajectory.points[point_index];
      for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
        if (is_robot_variable(trajectory.joint_names[i])) {
          state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
        }
      }
      state.update(true);
      std::string point_reason;
      if (!is_state_valid_with_attached_boxes(state, active_attached_boxes(), true, &point_reason)) {
        if (reason) {
          *reason = "trajectory point " + std::to_string(point_index) +
                    ": MoveIt scene collision check failed";
          if (!point_reason.empty()) {
            *reason += " (" + point_reason + ")";
          }
        }
        return false;
      }
    }
    return true;
  }

  bool planned_trajectory_clear_by_aabb(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_ || active_attached_boxes().empty()) return true;

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      moveit::core::RobotState state(start_state);
      const auto& point = trajectory.points[point_index];
      for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
        if (is_robot_variable(trajectory.joint_names[i])) {
          state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
        }
      }
      state.update();
      std::string point_reason;
      if (!carried_boxes_clear_static_obstacles(state, &point_reason)) {
        if (reason) {
          *reason = "trajectory point " + std::to_string(point_index) +
                    ": conservative AABB post-check failed (" + point_reason + ")";
        }
        return false;
      }
    }
    return true;
  }

  bool remove_carried_box_ids(const std::vector<std::string>& ids, const std::string& action_name)
  {
    if (!enable_attached_box_collision_) return true;
    if (ids.empty()) return true;
    if (!scene_adapter_) return true;
    if (!scene_adapter_->removeCarriedBoxIds(ids)) {
      return fail(action_name + ": failed to remove carried boxes from planning scene");
    }
    return true;
  }

  bool clear_carried_boxes_from_scene()
  {
    if (!enable_attached_box_collision_) return true;
    if (!scene_adapter_ || scene_adapter_->activeAttachedBoxes().empty()) return true;
    return detach_carried_boxes();
  }

  bool attach_carried_boxes(int left_box_id, int right_box_id, bool top_suction)
  {
    if (!scene_adapter_) return true;
    if (!scene_adapter_->attachCarriedBoxes(left_box_id, right_box_id, top_suction)) {
      return fail("attach_carried_boxes: failed to update attached carried boxes");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Attached carried boxes: left=%d right=%d mode=%s",
                left_box_id, right_box_id, top_suction ? "top_suction" : "front");
    return true;
  }

  bool detach_carried_boxes()
  {
    if (!scene_adapter_ || scene_adapter_->activeAttachedBoxes().empty()) return true;
    if (!scene_adapter_->detachCarriedBoxes()) {
      return fail("detach_carried_boxes: failed to remove carried boxes from planning scene");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Detached carried boxes");
    return true;
  }

  sensor_msgs::msg::JointState make_dual_arm_joint_target(
    double updown, const std::vector<double>& left_arm, const std::vector<double>& right_arm) const
  {
    sensor_msgs::msg::JointState target;
    target.name = {
      "updown",
      "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
      "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
      "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
      "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
    };
    target.position.reserve(target.name.size());
    target.position.push_back(updown);
    target.position.insert(target.position.end(), left_arm.begin(), left_arm.end());
    target.position.insert(target.position.end(), right_arm.begin(), right_arm.end());
    return target;
  }

  bool plan_to_joint_target(const std::string& stage_name, const sensor_msgs::msg::JointState& target)
  {
    auto start_state = get_current_robot_state();
    if (!start_state) {
      return fail(stage_name + ": cannot get start state");
    }

    moveit::core::RobotState goal_state(*start_state);
    for (size_t i = 0; i < target.name.size() && i < target.position.size(); ++i) {
      if (is_robot_variable(target.name[i])) {
        goal_state.setVariablePosition(target.name[i], target.position[i]);
      }
    }
    goal_state.enforceBounds(joint_group_);
    goal_state.update();

    if (!is_state_valid(goal_state, check_goal_collision_)) {
      return fail(stage_name + ": joint target out of bounds or colliding");
    }

    RCLCPP_INFO(get_logger(), "[%s] planning joint target", stage_name.c_str());
    return plan_to_goal_state(stage_name, *start_state, goal_state, target.name);
  }

  bool plan_dual_tip_ik(
    const std::string& stage_name,
    const geometry_msgs::msg::Pose& left_pose,
    const geometry_msgs::msg::Pose& right_pose,
    bool top_suction)
  {
    auto start_state = get_current_robot_state();
    if (!start_state) {
      return fail(stage_name + ": cannot get start state");
    }
    if (!optimized_dual_ik_solver_ || !optimized_dual_ik_solver_->ready()) {
      return fail(stage_name + ": optimized IK solver is not initialized");
    }

    RCLCPP_INFO(get_logger(), "[%s] optimized IK L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f) current_h=%.3f",
                stage_name.c_str(),
                left_pose.position.x, left_pose.position.y, left_pose.position.z,
                right_pose.position.x, right_pose.position.y, right_pose.position.z, current_updown(*start_state));

    const auto solved = optimized_dual_ik_solver_->solve(
      OptimizedDualIkSolveRequest{stage_name, left_pose, right_pose, top_suction, start_state.get()},
      "optimized_dual_tip_ik");
    if (!solved.success) {
      return fail(stage_name + ": " + solved.failure_reason);
    }

    if (!is_state_valid(*solved.goal_state, check_goal_collision_)) {
      return fail(stage_name + ": selected IK state out of bounds or colliding");
    }
    const auto& result = solved.ik_result;

    RCLCPP_INFO(get_logger(),
                "[%s] IK selected h=%.3f score=%.3f path=%s h_index=%zu seed_index=%zu trials=%zu legal=%zu wall=%.1fms",
                stage_name.c_str(), result.selected.h, result.selected.score, result.selected.solver_path.c_str(),
                result.selected.h_index, result.selected.seed_index, result.trial_count, result.legal_count,
                result.wall_ms);

    return plan_to_goal_state(stage_name, *start_state, *solved.goal_state,
                              result.selected.full_joint_names, solved.extra);
  }

  bool solve_dual_tip_ik_state(
    const std::string& stage_name,
    const geometry_msgs::msg::Pose& left_pose,
    const geometry_msgs::msg::Pose& right_pose,
    bool top_suction,
    const moveit::core::RobotState& seed_state,
    moveit::core::RobotState* goal_state,
    nlohmann::json* extra_out,
    ik_benchmark::UpdownAwareIkResult* result_out = nullptr)
  {
    if (!optimized_dual_ik_solver_ || !optimized_dual_ik_solver_->ready()) {
      return fail(stage_name + ": optimized IK solver is not initialized");
    }
    if (!goal_state) {
      return fail(stage_name + ": output goal_state is null");
    }

    RCLCPP_INFO(get_logger(), "[%s] direct optimized IK L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f) current_h=%.3f",
                stage_name.c_str(),
                left_pose.position.x, left_pose.position.y, left_pose.position.z,
                right_pose.position.x, right_pose.position.y, right_pose.position.z, current_updown(seed_state));

    const auto solved = optimized_dual_ik_solver_->solve(
      OptimizedDualIkSolveRequest{stage_name, left_pose, right_pose, top_suction, &seed_state},
      "optimized_dual_tip_ik_direct_seed");
    const auto& result = solved.ik_result;
    if (result_out) {
      *result_out = result;
    }
    if (!solved.success) {
      return fail(stage_name + ": " + solved.failure_reason);
    }

    *goal_state = *solved.goal_state;

    if (!is_state_valid(*goal_state, check_goal_collision_)) {
      return fail(stage_name + ": selected IK state out of bounds or colliding");
    }

    RCLCPP_INFO(get_logger(),
                "[%s] direct IK selected h=%.3f score=%.3f path=%s h_index=%zu seed_index=%zu trials=%zu legal=%zu wall=%.1fms",
                stage_name.c_str(), result.selected.h, result.selected.score, result.selected.solver_path.c_str(),
                result.selected.h_index, result.selected.seed_index, result.trial_count, result.legal_count,
                result.wall_ms);

    if (extra_out) {
      *extra_out = solved.extra;
    }
    return true;
  }

  geometry_msgs::msg::Pose link_pose(const moveit::core::RobotState& state, const std::string& link_name) const
  {
    const Eigen::Isometry3d& tf = state.getGlobalLinkTransform(link_name);
    Eigen::Quaterniond q(tf.linear());
    q.normalize();
    return make_pose(tf.translation().x(), tf.translation().y(), tf.translation().z(), q);
  }

  nlohmann::json ik_candidate_rejection_counts_json(const ik_benchmark::UpdownAwareIkResult& result) const
  {
    if (optimized_dual_ik_solver_) {
      return optimized_dual_ik_solver_->candidateRejectionCountsJson(result);
    }
    std::map<std::string, size_t> counts;
    for (const auto& candidate : result.candidates) {
      if (candidate.legal) {
        counts["legal"]++;
      } else if (!candidate.rejection_reason.empty()) {
        counts[candidate.rejection_reason]++;
      } else {
        counts["unknown"]++;
      }
    }
    nlohmann::json out = nlohmann::json::object();
    for (const auto& [reason, count] : counts) {
      out[reason] = count;
    }
    return out;
  }

  moveit::core::RobotState state_from_ik_candidate(
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkCandidate& candidate) const
  {
    moveit::core::RobotState state(seed_state);
    for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
      const auto& name = candidate.full_joint_names[i];
      if (is_robot_variable(name)) {
        state.setVariablePosition(name, candidate.full_joint_values[i]);
      }
    }
    state.enforceBounds(joint_group_);
    state.update();
    return state;
  }

  std::vector<std::string> arm_joint_target_names() const
  {
    return {
      "updown",
      "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
      "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
      "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
      "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
    };
  }

  void fill_loaded_pose_distance_metrics(ExtractRolloutTiming& timing) const
  {
    if (!timing.final_state) {
      return;
    }
    if (!loaded_pose_selector_) {
      return;
    }
    const auto selection = loaded_pose_selector_->select(*timing.final_state);
    timing.selected_left_loaded_pose_index = selection.left_index;
    timing.selected_right_loaded_pose_index = selection.right_index;
    timing.selected_left_loaded_pose_distance = selection.left_distance;
    timing.selected_right_loaded_pose_distance = selection.right_distance;
    timing.loaded_pose_distance_sum = selection.distance_sum;
    timing.loaded_pose_distance_l2 = selection.distance_l2;
    timing.loaded_pose_max_joint_delta = selection.max_joint_delta;
  }

  ExtractRolloutTiming rollout_left_extract_from_state(
    const moveit::core::RobotState& start_state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    size_t candidate_order,
    const ik_benchmark::UpdownAwareIkCandidate& ik_candidate,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step = {}) const
  {
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    if (left_it == boxes.end()) {
      ExtractRolloutTiming timing;
      timing.candidate_order = candidate_order;
      timing.h_index = ik_candidate.h_index;
      timing.seed_index = ik_candidate.seed_index;
      timing.h = ik_candidate.h;
      timing.ik_score = ik_candidate.score;
      timing.ik_solve_ms = ik_candidate.solve_ms;
      timing.failure_reason = "unknown_left_box_id";
      return timing;
    }
    return extract_rollout_planner_->rolloutLeft(
      start_state,
      left_it->second,
      left_box,
      left_box_id,
      candidate_order,
      ik_candidate.h_index,
      ik_candidate.seed_index,
      ik_candidate.h,
      ik_candidate.score,
      ik_candidate.solve_ms,
      record_step);
  }

  ExtractRolloutTiming rollout_dual_extract_from_state(
    const moveit::core::RobotState& start_state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t candidate_order,
    const ik_benchmark::UpdownAwareIkCandidate& ik_candidate,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step = {}) const
  {
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    const auto right_it = boxes.find(right_box_id);
    if (left_it == boxes.end() || right_it == boxes.end()) {
      ExtractRolloutTiming timing;
      timing.candidate_order = candidate_order;
      timing.h_index = ik_candidate.h_index;
      timing.seed_index = ik_candidate.seed_index;
      timing.h = ik_candidate.h;
      timing.ik_score = ik_candidate.score;
      timing.ik_solve_ms = ik_candidate.solve_ms;
      timing.failure_reason = "unknown_box_id";
      return timing;
    }
    return extract_rollout_planner_->rolloutDual(
      start_state,
      left_it->second,
      left_box,
      left_box_id,
      right_it->second,
      right_box,
      right_box_id,
      candidate_order,
      ik_candidate.h_index,
      ik_candidate.seed_index,
      ik_candidate.h,
      ik_candidate.score,
      ik_candidate.solve_ms,
      record_step);
  }

  bool benchmark_all_legal_ik_extract(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id)
  {
    ExtractBenchmarkRunner runner(extract_benchmark_runner_config(), extract_benchmark_runner_callbacks());
    return runner.runLeft(prefix, seed_state, ik_result, left_box, left_box_id);
  }

  bool benchmark_all_legal_ik_dual_extract(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id)
  {
    ExtractBenchmarkRunner runner(extract_benchmark_runner_config(), extract_benchmark_runner_callbacks());
    return runner.runDual(prefix, seed_state, ik_result, left_box, left_box_id, right_box, right_box_id);
  }

  bool record_extract_keyframe(
    const std::string& stage_name,
    const moveit::core::RobotState& state,
    const std::vector<AttachedBoxSpec>& boxes,
    const nlohmann::json& extra)
  {
    if (!recording_enabled()) return true;
    const auto saved_boxes = active_attached_boxes();
    if (scene_adapter_) scene_adapter_->setActiveAttachedBoxesForRecordOnly(boxes);

    trajectory_msgs::msg::JointTrajectory traj;
    const auto names = optimized_ik_solver_ ? optimized_ik_solver_->freeVariableNames() : robot_model_->getVariableNames();
    traj.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point.positions.reserve(names.size());
    for (const auto& name : names) {
      point.positions.push_back(is_robot_variable(name) ? state.getVariablePosition(name) : 0.0);
    }
    traj.points.push_back(point);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_.joint_trajectory = traj;
    record_stage(stage_name, plan, state, state, names, extra);

    if (scene_adapter_) scene_adapter_->setActiveAttachedBoxesForRecordOnly(saved_boxes);
    return true;
  }

  bool record_extract_keyframe(
    const std::string& stage_name,
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& left_box,
    const nlohmann::json& extra)
  {
    return record_extract_keyframe(stage_name, state, std::vector<AttachedBoxSpec>{left_box}, extra);
  }

  void record_tip_error_ik_candidates(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box)
  {
    if (!recording_enabled() || record_tip_error_ik_candidate_limit_ == 0) return;
    size_t recorded = 0;
    for (size_t i = 0; i < ik_result.candidates.size(); ++i) {
      const auto& candidate = ik_result.candidates[i];
      if (candidate.rejection_reason != "tip_error_too_large" || candidate.full_joint_values.empty()) {
        continue;
      }
      auto state = std::make_shared<moveit::core::RobotState>(seed_state);
      for (size_t j = 0; j < candidate.full_joint_names.size() && j < candidate.full_joint_values.size(); ++j) {
        const auto& name = candidate.full_joint_names[j];
        if (is_robot_variable(name)) {
          state->setVariablePosition(name, candidate.full_joint_values[j]);
        }
      }
      state->enforceBounds(joint_group_);
      state->update();
      nlohmann::json extra = {
        {"stage_kind", "tip_error_ik_candidate"},
        {"candidate_index", i},
        {"h_index", candidate.h_index},
        {"seed_index", candidate.seed_index},
        {"h", candidate.h},
        {"score", candidate.score},
        {"solve_ms", candidate.solve_ms},
        {"direct_pos_error", candidate.direct_pos_error},
        {"direct_ori_error", candidate.direct_ori_error},
        {"swapped_pos_error", candidate.swapped_pos_error},
        {"target_order", candidate.target_order},
        {"rejection_reason", candidate.rejection_reason},
        {"collision_free", candidate.collision_free},
        {"collision_pairs", candidate.collision_pairs}
      };
      record_extract_keyframe(
        prefix + "/tip_error_candidate_" + std::to_string(recorded),
        *state,
        left_box,
        extra);
      ++recorded;
      if (recorded >= record_tip_error_ik_candidate_limit_) break;
    }
  }

  bool plan_left_extract_primitive(int left_box_id, const std::string& prefix)
  {
    auto start_state = last_commanded_state_
      ? std::make_shared<moveit::core::RobotState>(*last_commanded_state_)
      : get_current_robot_state();
    if (!start_state) return fail(prefix + "/extract: cannot get start state");

    const AttachedBoxSpec left_box = make_carried_box_spec("left", left_box_id, false);
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    if (left_it == boxes.end()) return fail(prefix + "/extract: unknown left box id");

    auto record_step = [&](size_t step, const moveit::core::RobotState& state, const nlohmann::json& extra) {
      const bool accepted = extra.value("accepted", false);
      const std::string stage = step == 0
        ? prefix + "/extract_start"
        : prefix + (accepted ? "/extract_step_" : "/extract_failed_step_") + std::to_string(step);
      nlohmann::json enriched = extra;
      enriched["stage_kind"] = accepted ? "left_extract_primitive" : "left_extract_primitive_candidates";
      enriched["extract_ik"] = "left_v5_arm_kdl_fixed_updown";
      enriched["left_box_id"] = left_box_id;
      record_extract_keyframe(stage, state, left_box, enriched);
    };

    const ExtractRolloutTiming timing = extract_rollout_planner_->rolloutLeft(
      *start_state,
      left_it->second,
      left_box,
      left_box_id,
      0,
      0,
      0,
      current_updown(*start_state),
      0.0,
      0.0,
      record_step);

    if (timing.success && timing.final_state) {
      last_commanded_state_ = timing.final_state;
      RCLCPP_INFO(get_logger(), "[%s/extract] detached retreat=%.3f lift=%.3f pitch=%.1fdeg",
                  prefix.c_str(), timing.final_retreat_x, timing.final_lift_z, timing.final_pitch_deg);
      return true;
    }
    return fail(prefix + "/extract: " + (timing.failure_reason.empty()
      ? std::string("reached max retreat without neighbor detachment")
      : timing.failure_reason));
  }

  bool plan_to_goal_state(
    const std::string& stage_name,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::vector<std::string>& target_names,
    const nlohmann::json& extra = nlohmann::json::object())
  {
    moveit::core::RobotState planning_start_state(start_state);
    moveit::core::RobotState planning_goal_state(goal_state);
    const auto carried_boxes = active_attached_boxes();
    if (enable_attached_box_collision_ && !carried_boxes.empty()) {
      attach_boxes_to_robot_state(planning_start_state, carried_boxes, attached_box_collision_padding_);
      attach_boxes_to_robot_state(planning_goal_state, carried_boxes, attached_box_collision_padding_);
    }

    move_group_->setStartState(planning_start_state);
    move_group_->setJointValueTarget(planning_goal_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      return fail(stage_name + ": MoveIt planning failed, code=" + std::to_string(plan_result.val));
    }

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    RCLCPP_INFO(get_logger(), "[%s] planned points=%zu execute=%s", stage_name.c_str(),
                trajectory.points.size(), execute_ ? "true" : "false");

    std::string carried_collision_reason;
    if (!planned_carried_boxes_clear_static_obstacles(plan, planning_start_state, &carried_collision_reason)) {
      return fail(stage_name + ": carried box collides with static box obstacle (" + carried_collision_reason + ")");
    }

    record_stage(stage_name, plan, planning_start_state, planning_goal_state, target_names, extra);

    if (execute_) {
      if (execution_backend_ == "alfa_execution_bridge") {
        if (!execute_with_alfa_execution_bridge(stage_name, plan, planning_start_state)) {
          return false;
        }
      } else {
        const auto exec_result = move_group_->execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
          return fail(stage_name + ": MoveIt execute failed, code=" + std::to_string(exec_result.val));
        }
      }
    }

    last_commanded_state_ = std::make_shared<moveit::core::RobotState>(goal_state);
    wait_for_joint_state_near(goal_state, target_names);
    return true;
  }

  bool wait_for_joint_state_near(
    const moveit::core::RobotState& goal_state,
    const std::vector<std::string>& target_names)
  {
    if (!execute_ || state_wait_timeout_s_ <= 0.0) return true;

    const auto deadline = now() + rclcpp::Duration::from_seconds(state_wait_timeout_s_);
    while (rclcpp::ok() && now() < deadline) {
      if (move_group_) {
        auto current_state = move_group_->getCurrentState(0.1);
        if (current_state && robot_state_matches(goal_state, *current_state, target_names)) {
          return true;
        }
      }

      sensor_msgs::msg::JointState::SharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        msg = latest_joint_state_;
      }
      if (msg && joint_state_matches(goal_state, *msg, target_names)) {
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_WARN(get_logger(),
                "Executed trajectory but /joint_states did not reach target within %.2fs. "
                "Continuing with last commanded state as planning seed. If RViz snaps back, check duplicate /joint_states publishers.",
                state_wait_timeout_s_);
    return false;
  }

  bool execute_with_alfa_execution_bridge(
    const std::string& stage_name,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& planning_start_state)
  {
    if (!execution_action_client_) {
      return fail(stage_name + ": execution action client is not initialized");
    }
    if (!execution_action_client_->wait_for_action_server(
          std::chrono::duration<double>(execution_action_wait_timeout_s_))) {
      return fail(stage_name + ": execution action server unavailable: " + execution_action_name_);
    }

    auto goal = FollowJointTrajectory::Goal();
    std::string reason;
    if (!build_alfa_execution_goal(plan.trajectory_.joint_trajectory, planning_start_state, &goal, &reason)) {
      return fail(stage_name + ": cannot build alfa execution goal (" + reason + ")");
    }

    RCLCPP_INFO(get_logger(), "[%s] sending trajectory to %s: points=%zu joints=%zu",
                stage_name.c_str(), execution_action_name_.c_str(),
                goal.trajectory.points.size(), goal.trajectory.joint_names.size());

    auto send_future = execution_action_client_->async_send_goal(goal);
    if (!wait_for_future(send_future, execution_action_wait_timeout_s_)) {
      return fail(stage_name + ": failed to send alfa execution goal");
    }

    auto goal_handle = send_future.get();
    if (!goal_handle) {
      return fail(stage_name + ": alfa execution goal rejected");
    }

    auto result_future = execution_action_client_->async_get_result(goal_handle);
    if (!wait_for_future(result_future, execution_result_timeout_s_)) {
      return fail(stage_name + ": failed waiting alfa execution result");
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      std::ostringstream oss;
      oss << stage_name << ": alfa execution failed code="
          << static_cast<int>(wrapped_result.code)
          << " action_error=" << wrapped_result.result->error_code
          << " message=" << wrapped_result.result->error_string;
      return fail(oss.str());
    }
    return true;
  }

  template<typename FutureT>
  bool wait_for_future(FutureT& future, double timeout_s) const
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
        return true;
      }
      if (timeout_s > 0.0) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_s) {
          return false;
        }
      }
    }
    return false;
  }

  bool build_alfa_execution_goal(
    const trajectory_msgs::msg::JointTrajectory& source,
    const moveit::core::RobotState& planning_start_state,
    FollowJointTrajectory::Goal* goal,
    std::string* reason) const
  {
    if (!goal) return false;
    if (source.points.empty()) {
      if (reason) *reason = "source trajectory is empty";
      return false;
    }

    const auto target_names = alfa_execution_joint_names();
    std::vector<int> source_indices;
    source_indices.reserve(target_names.size());
    for (const auto& target_name : target_names) {
      const auto moveit_name = alfa_to_moveit_joint_name(target_name);
      const auto it = std::find(source.joint_names.begin(), source.joint_names.end(), moveit_name);
      if (it == source.joint_names.end()) {
        if (!execution_allow_hold_missing_target_joints_) {
          if (reason) *reason = "missing planned joint " + moveit_name + " for target " + target_name;
          return false;
        }
        source_indices.push_back(-1);
      } else {
        source_indices.push_back(static_cast<int>(std::distance(source.joint_names.begin(), it)));
      }
    }

    if (execution_reject_unmapped_planned_joints_) {
      const auto mapped_target_names = alfa_execution_joint_names();
      for (const auto& planned_name : source.joint_names) {
        if (std::find(mapped_target_names.begin(), mapped_target_names.end(),
                      moveit_to_alfa_joint_name(planned_name)) != mapped_target_names.end()) {
          continue;
        }
        if (planned_joint_changes(source, planned_name)) {
          if (reason) {
            *reason = "planned joint " + planned_name +
              " changes but is not mapped to alfa execution target joints";
          }
          return false;
        }
      }
    }

    goal->trajectory = trajectory_msgs::msg::JointTrajectory();
    goal->trajectory.header = source.header;
    goal->trajectory.joint_names = target_names;
    goal->trajectory.points.reserve(source.points.size());

    for (const auto& source_point : source.points) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start = source_point.time_from_start;
      point.positions.reserve(target_names.size());
      if (!source_point.velocities.empty()) point.velocities.reserve(target_names.size());
      if (!source_point.accelerations.empty()) point.accelerations.reserve(target_names.size());
      if (!source_point.effort.empty()) point.effort.reserve(target_names.size());

      for (size_t i = 0; i < target_names.size(); ++i) {
        const int source_index = source_indices[i];
        if (source_index >= 0) {
          const auto index = static_cast<size_t>(source_index);
          point.positions.push_back(index < source_point.positions.size() ? source_point.positions[index] : 0.0);
          if (!source_point.velocities.empty()) {
            point.velocities.push_back(index < source_point.velocities.size() ? source_point.velocities[index] : 0.0);
          }
          if (!source_point.accelerations.empty()) {
            point.accelerations.push_back(index < source_point.accelerations.size() ? source_point.accelerations[index] : 0.0);
          }
          if (!source_point.effort.empty()) {
            point.effort.push_back(index < source_point.effort.size() ? source_point.effort[index] : 0.0);
          }
        } else {
          const auto moveit_name = alfa_to_moveit_joint_name(target_names[i]);
          const double hold_position =
            is_robot_variable(moveit_name) ? planning_start_state.getVariablePosition(moveit_name) : 0.0;
          point.positions.push_back(hold_position);
          if (!source_point.velocities.empty()) point.velocities.push_back(0.0);
          if (!source_point.accelerations.empty()) point.accelerations.push_back(0.0);
          if (!source_point.effort.empty()) point.effort.push_back(0.0);
        }
      }
      goal->trajectory.points.push_back(std::move(point));
    }
    return true;
  }

  bool planned_joint_changes(
    const trajectory_msgs::msg::JointTrajectory& source,
    const std::string& joint_name) const
  {
    const auto it = std::find(source.joint_names.begin(), source.joint_names.end(), joint_name);
    if (it == source.joint_names.end()) return false;
    const auto index = static_cast<size_t>(std::distance(source.joint_names.begin(), it));
    std::optional<double> first_value;
    for (const auto& point : source.points) {
      if (index >= point.positions.size()) continue;
      if (!first_value) {
        first_value = point.positions[index];
        continue;
      }
      if (std::abs(point.positions[index] - *first_value) > 1e-6) {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> alfa_execution_joint_names() const
  {
    std::vector<std::string> names = {
      "left_joint1",
      "left_joint2",
      "left_joint3",
      "left_joint4",
      "left_joint5",
      "left_joint6",
      "right_joint1",
      "right_joint2",
      "right_joint3",
      "right_joint4",
      "right_joint5",
      "right_joint6",
    };
    if (execution_include_turn_) {
      names.push_back("turn");
    }
    return names;
  }

  std::string alfa_to_moveit_joint_name(const std::string& name) const
  {
    if (name.rfind("left_joint", 0) == 0) {
      return "left_v5_joint" + name.substr(std::string("left_joint").size());
    }
    if (name.rfind("right_joint", 0) == 0) {
      return "right_v5_joint" + name.substr(std::string("right_joint").size());
    }
    return name;
  }

  std::string moveit_to_alfa_joint_name(const std::string& name) const
  {
    if (name.rfind("left_v5_joint", 0) == 0) {
      return "left_joint" + name.substr(std::string("left_v5_joint").size());
    }
    if (name.rfind("right_v5_joint", 0) == 0) {
      return "right_joint" + name.substr(std::string("right_v5_joint").size());
    }
    return name;
  }

  bool robot_state_matches(
    const moveit::core::RobotState& goal_state,
    const moveit::core::RobotState& current_state,
    const std::vector<std::string>& target_names) const
  {
    for (const auto& name : target_names) {
      if (!is_robot_variable(name)) continue;
      const double error = std::abs(current_state.getVariablePosition(name) - goal_state.getVariablePosition(name));
      if (error > joint_goal_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  bool joint_state_matches(
    const moveit::core::RobotState& goal_state,
    const sensor_msgs::msg::JointState& msg,
    const std::vector<std::string>& target_names) const
  {
    for (const auto& name : target_names) {
      auto it = std::find(msg.name.begin(), msg.name.end(), name);
      if (it == msg.name.end()) {
        const auto alfa_name = moveit_to_alfa_joint_name(name);
        it = std::find(msg.name.begin(), msg.name.end(), alfa_name);
      }
      if (it == msg.name.end()) continue;
      const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (index >= msg.position.size()) continue;
      if (!is_robot_variable(name)) continue;
      const double error = std::abs(msg.position[index] - goal_state.getVariablePosition(name));
      if (error > joint_goal_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  bool is_robot_variable(const std::string& name) const
  {
    const auto& variable_names = robot_model_->getVariableNames();
    return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
  }

  std::vector<double> state_values(
    const moveit::core::RobotState& state, const std::vector<std::string>& names) const
  {
    if (optimized_dual_ik_solver_) {
      return optimized_dual_ik_solver_->stateValues(state, names);
    }
    std::vector<double> values;
    values.reserve(names.size());
    for (const auto& name : names) {
      values.push_back(is_robot_variable(name) ? state.getVariablePosition(name) : 0.0);
    }
    return values;
  }

  double current_updown(const moveit::core::RobotState& state) const
  {
    if (optimized_dual_ik_solver_) {
      return optimized_dual_ik_solver_->currentUpdown(state);
    }
    return is_robot_variable("updown") ? state.getVariablePosition("updown") : fixed_updown_;
  }

  bool recording_enabled() const
  {
    return recorder_ && recorder_->enabled();
  }

  size_t recorded_stage_count() const
  {
    return recorder_ ? recorder_->stageCount() : 0;
  }

  void open_record_file()
  {
    if (!record_trajectories_ || record_jsonl_path_.empty()) return;
    nlohmann::json header = {
      {"type", "header"},
      {"schema", "moveit_box_stack_flow_v1"},
      {"ik_strategy", "fixed_discrete_h_multi_seed_cost_scorer"},
      {"planning_group", planning_group_},
      {"box_front_x", box_front_x_},
      {"scene_y_shift", scene_y_shift_},
      {"world_to_base_z", world_to_base_z_},
      {"fixed_updown", fixed_updown_},
      {"velocity_scale", velocity_scale_},
      {"acceleration_scale", acceleration_scale_},
      {"max_rounds", max_rounds_},
      {"include_top_suction", include_top_suction_},
      {"execute", execute_},
      {"container_obstacle", container_obstacle_json()},
      {"static_box_obstacles", static_box_obstacles_json()},
      {"attached_box_collision", attached_box_config_json()},
      {"loaded_pose_family", {
        {"left_candidates_deg", pose_family_degrees_json(left_loaded_pose_family_)},
        {"right_candidates_deg", pose_family_degrees_json(right_loaded_pose_family_)},
        {"left_preferred_index", left_preferred_loaded_pose_index_},
        {"right_preferred_index", right_preferred_loaded_pose_index_},
        {"family_distance_weight", ik_config_.cost_loaded_family_distance},
        {"preferred_distance_weight", ik_config_.cost_loaded_preferred_distance}
      }},
      {"ik_config", {
        {"fixed_group", ik_config_.fixed_group},
        {"free_group", ik_config_.free_group},
        {"solver_plugin", ik_config_.solver_plugin},
        {"h_search_mode", "fixed_discrete"},
        {"h_candidate_count", ik_config_.h_candidate_count},
        {"seed_count", ik_config_.seed_count},
        {"workers", ik_config_.workers},
        {"timeout", ik_config_.timeout},
        {"front_z_reach_window", {ik_config_.gripper_z_reach_lower, ik_config_.gripper_z_reach_upper}},
        {"top_z_reach_window", {ik_config_.top_suction_z_reach_lower, ik_config_.top_suction_z_reach_upper}},
        {"h_limits", {ik_config_.h_lower, ik_config_.h_upper}},
        {"check_collision", ik_config_.check_collision},
        {"cost_loaded_family_distance", ik_config_.cost_loaded_family_distance},
        {"cost_loaded_preferred_distance", ik_config_.cost_loaded_preferred_distance}
      }}
    };
    recorder_ = std::make_unique<MotionFlowRecorder>();
    std::string error;
    if (!recorder_->open(record_jsonl_path_, header, &error)) {
      RCLCPP_WARN(get_logger(), "Failed to open trajectory record JSONL: %s", record_jsonl_path_.c_str());
      recorder_.reset();
      return;
    }
    RCLCPP_INFO(get_logger(), "Recording MoveIt flow JSONL: %s", record_jsonl_path_.c_str());
  }

  nlohmann::json container_obstacle_json() const
  {
    nlohmann::json panels = nlohmann::json::array();
    for (const auto& panel : container_panels()) {
      panels.push_back({
        {"id", panel.id},
        {"center", {panel.center[0], panel.center[1], panel.center[2]}},
        {"size", {panel.size[0], panel.size[1], panel.size[2]}},
      });
    }
    return {
      {"enabled", enable_container_obstacle_},
      {"frame", container_frame_},
      {"length", container_length_},
      {"width", container_width_},
      {"height", container_height_},
      {"center_x", container_center_x_},
      {"center_y", container_center_y_ + scene_y_shift_},
      {"nominal_center_y", container_center_y_},
      {"scene_y_shift", scene_y_shift_},
      {"floor_z", container_floor_z_},
      {"wall_thickness", container_wall_thickness_},
      {"panels", panels},
    };
  }

  nlohmann::json attached_box_config_json() const
  {
    return {
      {"enabled", enable_attached_box_collision_},
      {"depth", carried_box_depth_},
      {"width", carried_box_width_},
      {"height", carried_box_height_},
    };
  }

  nlohmann::json static_box_obstacles_json() const
  {
    nlohmann::json boxes = nlohmann::json::array();
    for (const auto& box : static_box_obstacles()) {
      boxes.push_back({
        {"id", box.id},
        {"center", {box.center[0], box.center[1], box.center[2]}},
        {"size", {box.size[0], box.size[1], box.size[2]}},
      });
    }
    return {
      {"enabled", enable_static_box_obstacles_},
      {"mode", "dynamic_box_wall_with_pair_opening"},
      {"opening_left_box_id", scene_adapter_ ? scene_adapter_->activeStaticLeftBoxId() : 0},
      {"opening_right_box_id", scene_adapter_ ? scene_adapter_->activeStaticRightBoxId() : 0},
      {"inset", static_box_obstacle_inset_},
      {"boxes", boxes},
    };
  }

  nlohmann::json active_attached_boxes_json() const
  {
    nlohmann::json boxes = nlohmann::json::array();
    for (const auto& box : active_attached_boxes()) {
      boxes.push_back({
        {"id", box.id},
        {"link_name", box.link_name},
        {"center_in_link", {box.center_in_link[0], box.center_in_link[1], box.center_in_link[2]}},
        {"size", {box.size[0], box.size[1], box.size[2]}},
      });
    }
    return boxes;
  }

  nlohmann::json attached_boxes_json(const std::vector<AttachedBoxSpec>& specs) const
  {
    nlohmann::json boxes = nlohmann::json::array();
    for (const auto& box : specs) {
      boxes.push_back({
        {"id", box.id},
        {"link_name", box.link_name},
        {"center_in_link", {box.center_in_link[0], box.center_in_link[1], box.center_in_link[2]}},
        {"size", {box.size[0], box.size[1], box.size[2]}},
      });
    }
    return boxes;
  }

  nlohmann::json robot_state_json(const moveit::core::RobotState& state) const
  {
    const auto& names = robot_model_->getVariableNames();
    std::vector<double> values;
    values.reserve(names.size());
    for (const auto& name : names) values.push_back(state.getVariablePosition(name));
    return {{"joint_names", names}, {"joint_values", values}, {"joint_map", names_values_json(names, values)}};
  }

  nlohmann::json trajectory_json(const moveit::planning_interface::MoveGroupInterface::Plan& plan) const
  {
    const auto& traj = plan.trajectory_.joint_trajectory;
    nlohmann::json points = nlohmann::json::array();
    for (const auto& point : traj.points) {
      points.push_back({
        {"time_from_start_sec", rclcpp::Duration(point.time_from_start).seconds()},
        {"positions", point.positions},
        {"velocities", point.velocities}
      });
    }
    return {
      {"joint_names", traj.joint_names},
      {"point_count", traj.points.size()},
      {"points", points}
    };
  }

  nlohmann::json monitor_stage_json(
    const std::string& stage_name,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::vector<std::string>& target_names,
    const std::vector<AttachedBoxSpec>& attached_boxes,
    const nlohmann::json& extra) const
  {
    return {
      {"type", "stage"},
      {"stage", stage_name},
      {"trajectory", trajectory_json(plan)},
      {"target_names", target_names},
      {"start_state", robot_state_json(start_state)},
      {"goal_state", robot_state_json(goal_state)},
      {"attached_boxes", attached_boxes_json(attached_boxes)},
      {"static_box_obstacles", static_box_obstacles_json()},
      {"extra", extra}
    };
  }

  nlohmann::json monitor_candidate_json(
    const ik_benchmark::UpdownAwareIkCandidate& candidate,
    size_t display_index,
    const moveit::core::RobotState& state) const
  {
    return {
      {"display_index", display_index},
      {"h", candidate.h},
      {"h_index", candidate.h_index},
      {"seed_index", candidate.seed_index},
      {"score", candidate.score},
      {"solve_ms", candidate.solve_ms},
      {"solver_path", candidate.solver_path},
      {"target_order", candidate.target_order},
      {"updown_delta", candidate.updown_delta},
      {"joint_delta", candidate.joint_delta},
      {"state", robot_state_json(state)}
    };
  }

  nlohmann::json monitor_timing_json(
    const ExtractRolloutTiming& timing,
    size_t display_index,
    const moveit::core::RobotState& state) const
  {
    const std::vector<AttachedBoxSpec> carried_boxes{
      extract_monitor_state_.left_box,
      extract_monitor_state_.right_box,
    };
    nlohmann::json replay_stages = nlohmann::json::array();
    for (const auto& stage : timing.rollout_records) {
      replay_stages.push_back(stage);
    }
    for (const auto& shift_stage : timing.lateral_shift_replay_stages) {
      if (!shift_stage.start_state || !shift_stage.goal_state) {
        continue;
      }
      nlohmann::json extra = shift_stage.extra;
      extra["candidate_order"] = timing.candidate_order;
      extra["loaded_plan_rank"] = timing.loaded_plan_rank;
      extra["loaded_plan_success"] = timing.loaded_plan_success;
      extra["loaded_plan_failure_reason"] = timing.loaded_plan_failure_reason;
      extra["left_box_id"] = extract_monitor_state_.left_box_id;
      extra["right_box_id"] = extract_monitor_state_.right_box_id;
      replay_stages.push_back(monitor_stage_json(
        shift_stage.stage_name,
        shift_stage.plan,
        *shift_stage.start_state,
        *shift_stage.goal_state,
        arm_joint_target_names(),
        carried_boxes,
        extra));
    }
    if (timing.loaded_start_state && timing.loaded_goal_state &&
        !timing.loaded_plan.trajectory_.joint_trajectory.points.empty()) {
      nlohmann::json extra = {
        {"stage_kind", "monitor_loaded_plan_attempt_replay"},
        {"valid", timing.loaded_plan_success},
        {"candidate_order", timing.candidate_order},
        {"loaded_plan_rank", timing.loaded_plan_rank},
        {"loaded_plan_success", timing.loaded_plan_success},
        {"loaded_plan_failure_reason", timing.loaded_plan_failure_reason},
        {"loaded_plan_ms", timing.loaded_plan_ms},
        {"loaded_plan_points", timing.loaded_plan_points},
        {"loaded_plan_trajectory_distance", timing.loaded_plan_trajectory_distance},
        {"left_box_id", extract_monitor_state_.left_box_id},
        {"right_box_id", extract_monitor_state_.right_box_id}
      };
      replay_stages.push_back(monitor_stage_json(
        extract_monitor_state_.prefix + "/candidate_" + std::to_string(timing.candidate_order) + "/loaded_plan_attempt",
        timing.loaded_plan,
        *timing.loaded_start_state,
        *timing.loaded_goal_state,
        arm_joint_target_names(),
        carried_boxes,
        extra));
    }
    return {
      {"display_index", display_index},
      {"candidate_order", timing.candidate_order},
      {"h", timing.h},
      {"h_index", timing.h_index},
      {"seed_index", timing.seed_index},
      {"ik_score", timing.ik_score},
      {"ik_solve_ms", timing.ik_solve_ms},
      {"rollout_ms", timing.rollout_ms},
      {"interval_ms", timing.interval_ms},
      {"accepted_steps", timing.accepted_steps},
      {"failed_steps", timing.failed_steps},
      {"final_retreat_x", timing.final_retreat_x},
      {"final_lift_z", timing.final_lift_z},
      {"final_pitch_deg", timing.final_pitch_deg},
      {"right_final_retreat_x", timing.right_final_retreat_x},
      {"right_final_lift_z", timing.right_final_lift_z},
      {"right_final_pitch_deg", timing.right_final_pitch_deg},
      {"success", timing.success},
      {"failure_reason", timing.failure_reason},
      {"loaded_plan_attempted", timing.loaded_plan_attempted},
      {"loaded_plan_success", timing.loaded_plan_success},
      {"lateral_shift_attempted", timing.lateral_shift_attempted},
      {"lateral_shift_success", timing.lateral_shift_success},
      {"lateral_shift_ms", timing.lateral_shift_ms},
      {"lateral_shift_reached_distance", timing.lateral_shift_reached_distance},
      {"lateral_shift_points", timing.lateral_shift_points},
      {"loaded_plan_rank", timing.loaded_plan_rank},
      {"loaded_plan_ms", timing.loaded_plan_ms},
      {"loaded_plan_points", timing.loaded_plan_points},
      {"loaded_plan_trajectory_distance", timing.loaded_plan_trajectory_distance},
      {"loaded_plan_failure_reason", timing.loaded_plan_failure_reason},
      {"loaded_pose_distance_sum", timing.loaded_pose_distance_sum},
      {"loaded_pose_distance_l2", timing.loaded_pose_distance_l2},
      {"loaded_pose_max_joint_delta", timing.loaded_pose_max_joint_delta},
      {"loaded_plan_selected", timing.loaded_plan_selected},
      {"state", robot_state_json(state)},
      {"replay_stage_count", replay_stages.size()},
      {"replay_stages", replay_stages}
    };
  }

  bool write_extract_monitor_snapshot(const nlohmann::json& snapshot) const
  {
    try {
      const std::filesystem::path path(extract_monitor_snapshot_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      std::ofstream out(path);
      if (!out) {
        RCLCPP_ERROR(get_logger(), "Failed to open extract monitor snapshot: %s",
                     extract_monitor_snapshot_path_.c_str());
        return false;
      }
      out << snapshot.dump(2) << '\n';
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to write extract monitor snapshot %s: %s",
                   extract_monitor_snapshot_path_.c_str(), e.what());
      return false;
    }
  }

  std::vector<ik_benchmark::UpdownAwareIkCandidate> selected_monitor_ik_candidates(
    const ik_benchmark::UpdownAwareIkResult& ik_result,
    IkCandidateSelectionStats* stats) const
  {
    std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
    legal_candidates.reserve(ik_result.candidates.size());
    for (const auto& candidate : ik_result.candidates) {
      if (candidate.legal) {
        legal_candidates.push_back(candidate);
      }
    }
    std::sort(legal_candidates.begin(), legal_candidates.end(),
              [](const auto& lhs, const auto& rhs) {
                if (lhs.score != rhs.score) return lhs.score < rhs.score;
                if (lhs.h_index != rhs.h_index) return lhs.h_index < rhs.h_index;
                return lhs.seed_index < rhs.seed_index;
              });
    if (ik_candidate_selector_) {
      return ik_candidate_selector_->select(legal_candidates, stats);
    }
    if (stats) {
      stats->input_count = legal_candidates.size();
      stats->unique_count = legal_candidates.size();
      stats->selected_count = legal_candidates.size();
    }
    return legal_candidates;
  }

  moveit::core::RobotState make_extract_monitor_seed_state() const
  {
    moveit::core::RobotState seed_state(robot_model_);
    seed_state.setToDefaultValues();
    for (size_t i = 0; i < left_pregrasp_arm_.size(); ++i) {
      seed_state.setVariablePosition("left_v5_joint" + std::to_string(i + 1), left_pregrasp_arm_[i]);
    }
    for (size_t i = 0; i < right_pregrasp_arm_.size(); ++i) {
      seed_state.setVariablePosition("right_v5_joint" + std::to_string(i + 1), right_pregrasp_arm_[i]);
    }
    seed_state.setVariablePosition("updown", extract_grasp_ik_home_updown_);
    seed_state.enforceBounds(joint_group_);
    seed_state.update();
    return seed_state;
  }

  moveit::core::RobotState make_extract_monitor_loaded_start_state() const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (size_t i = 0; i < left_loaded_arm_.size(); ++i) {
      state.setVariablePosition("left_v5_joint" + std::to_string(i + 1), left_loaded_arm_[i]);
    }
    for (size_t i = 0; i < right_loaded_arm_.size(); ++i) {
      state.setVariablePosition("right_v5_joint" + std::to_string(i + 1), right_loaded_arm_[i]);
    }
    state.setVariablePosition("updown", extract_grasp_ik_home_updown_);
    state.enforceBounds(joint_group_);
    state.update();
    return state;
  }

  void record_stage(
    const std::string& stage_name,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::vector<std::string>& target_names,
    const nlohmann::json& extra)
  {
    if (!recording_enabled()) return;
    recorder_->recordStage(
      stage_name,
      plan,
      robot_state_json(start_state),
      robot_state_json(goal_state),
      target_names,
      active_attached_boxes_json(),
      static_box_obstacles_json(),
      extra);
  }

  geometry_msgs::msg::Pose front_grasp_pose(const BoxSpec& box) const
  {
    return make_pose(box.x, box.y, box.z - world_to_base_z_, forward_x_orientation());
  }

  geometry_msgs::msg::Pose top_suction_pose(const BoxSpec& box) const
  {
    return make_pose(
      box.x + top_suction_x_offset_, box.y, box.z + top_suction_z_offset_ - world_to_base_z_, top_suction_orientation());
  }

  bool run_extract_monitor_next(std::string* message)
  {
    std::lock_guard<std::mutex> lock(extract_monitor_mutex_);
    if (!message) {
      return fail("extract monitor: output message is null");
    }

    switch (extract_monitor_phase_) {
      case ExtractMonitorPhase::ReadyForIk:
        return run_extract_monitor_ik_stage(message);
      case ExtractMonitorPhase::ReadyForExtract:
        return run_extract_monitor_extract_stage(message);
      case ExtractMonitorPhase::ReadyForLoaded:
        return run_extract_monitor_loaded_stage(message);
      case ExtractMonitorPhase::ReadyForFinal:
        return run_extract_monitor_final_stage(message);
      case ExtractMonitorPhase::Done:
        extract_monitor_phase_ = ExtractMonitorPhase::ReadyForIk;
        return run_extract_monitor_ik_stage(message);
    }
    return fail("extract monitor: unknown phase");
  }

  bool run_extract_monitor_full_selected(std::string* message)
  {
    std::lock_guard<std::mutex> lock(extract_monitor_mutex_);
    if (!message) {
      return fail("extract monitor full: output message is null");
    }

    const auto total_start = std::chrono::steady_clock::now();
    extract_monitor_phase_ = ExtractMonitorPhase::ReadyForIk;

    std::string ik_message;
    if (!run_extract_monitor_ik_stage(&ik_message)) {
      *message = "完整流程失败在IK阶段: " + ik_message;
      return false;
    }
    const double ik_elapsed_ms = extract_monitor_last_stage_ms_;

    std::string extract_message;
    if (!run_extract_monitor_extract_stage(&extract_message)) {
      *message = "完整流程失败在抽离阶段: " + extract_message;
      return false;
    }
    const double extract_elapsed_ms = extract_monitor_last_stage_ms_;

    std::string loaded_message;
    if (!run_extract_monitor_loaded_stage(&loaded_message)) {
      *message = "完整流程失败在负重规划阶段: " + loaded_message;
      return false;
    }
    const double loaded_elapsed_ms = extract_monitor_last_stage_ms_;

    std::string final_message;
    if (!run_extract_monitor_final_stage(&final_message)) {
      *message = "完整流程失败在最终选择阶段: " + final_message;
      return false;
    }
    const double final_elapsed_ms = extract_monitor_last_stage_ms_;

    const double total_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - total_start).count();

    nlohmann::json snapshot;
    try {
      snapshot = nlohmann::json::parse(std::ifstream(extract_monitor_snapshot_path_));
    } catch (const std::exception&) {
      snapshot = nlohmann::json::object();
    }
    if (snapshot.is_object()) {
      snapshot["phase"] = "full_selected";
      snapshot["phase_label"] = "完整流程最终采用方案";
      snapshot["box_front_x"] = box_front_x_;
      snapshot["scene_y_shift"] = scene_y_shift_;
      snapshot["elapsed_ms"] = total_elapsed_ms;
      snapshot["ik_elapsed_ms"] = ik_elapsed_ms;
      snapshot["extract_elapsed_ms"] = extract_elapsed_ms;
      snapshot["loaded_elapsed_ms"] = loaded_elapsed_ms;
      snapshot["final_elapsed_ms"] = final_elapsed_ms;
      write_extract_monitor_snapshot(snapshot);
    }

    extract_monitor_phase_ = ExtractMonitorPhase::ReadyForIk;
    std::ostringstream out;
    out << "完整流程完成: total=" << total_elapsed_ms << "ms"
        << " ik=" << ik_elapsed_ms << "ms"
        << " extract=" << extract_elapsed_ms << "ms"
        << " loaded=" << loaded_elapsed_ms << "ms"
        << " final=" << final_elapsed_ms << "ms"
        << " snapshot=" << extract_monitor_snapshot_path_;
    *message = out.str();
    return true;
  }

  bool run_extract_monitor_ik_stage(std::string* message)
  {
    const auto stage_start = std::chrono::steady_clock::now();
    clear_carried_boxes_from_scene();
    last_error_.clear();
    last_commanded_state_.reset();
    extract_monitor_state_ = ExtractMonitorState{};

    const int left_box_id = extract_demo_left_box_id_;
    const int right_box_id = extract_demo_right_box_id_;
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    const auto right_it = boxes.find(right_box_id);
    if (left_it == boxes.end() || right_it == boxes.end()) {
      return fail("extract monitor IK: unknown box id");
    }
    set_static_box_wall_opening(left_box_id, right_box_id, "extract_monitor");

    extract_monitor_state_.left_box_id = left_box_id;
    extract_monitor_state_.right_box_id = right_box_id;
    extract_monitor_state_.left_box = make_carried_box_spec("left", left_box_id, false);
    extract_monitor_state_.right_box = make_carried_box_spec("right", right_box_id, false);
    extract_monitor_state_.prefix = "extract_monitor_L" + std::to_string(left_box_id) +
                                    "_R" + std::to_string(right_box_id);
    extract_monitor_state_.seed_state =
      std::make_shared<moveit::core::RobotState>(make_extract_monitor_seed_state());
    extract_monitor_state_.loaded_start_state =
      std::make_shared<moveit::core::RobotState>(make_extract_monitor_loaded_start_state());

    moveit::core::RobotState selected_state(*extract_monitor_state_.seed_state);
    nlohmann::json ik_extra;
    ik_benchmark::UpdownAwareIkResult ik_result;
    if (!solve_dual_tip_ik_state(
          extract_monitor_state_.prefix + "/monitor_ik",
          front_grasp_pose(left_it->second),
          front_grasp_pose(right_it->second),
          false,
          *extract_monitor_state_.seed_state,
          &selected_state,
          &ik_extra,
          &ik_result)) {
      *message = "IK阶段失败: " + last_error_;
      return false;
    }

    IkCandidateSelectionStats dedup_stats;
    extract_monitor_state_.ik_result = ik_result;
    extract_monitor_state_.legal_candidates = selected_monitor_ik_candidates(ik_result, &dedup_stats);
    extract_monitor_state_.candidate_states.clear();
    extract_monitor_state_.candidate_states.reserve(extract_monitor_state_.legal_candidates.size());

    nlohmann::json records = nlohmann::json::array();
    for (size_t i = 0; i < extract_monitor_state_.legal_candidates.size(); ++i) {
      auto state = std::make_shared<moveit::core::RobotState>(
        state_from_ik_candidate(*extract_monitor_state_.seed_state, extract_monitor_state_.legal_candidates[i]));
      extract_monitor_state_.candidate_states.push_back(state);
      records.push_back(monitor_candidate_json(extract_monitor_state_.legal_candidates[i], i, *state));
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    nlohmann::json snapshot = {
      {"type", "extract_monitor_snapshot"},
      {"phase", "ik_candidates"},
      {"phase_label", "不重复 IK 候选"},
      {"elapsed_ms", elapsed_ms},
      {"left_box_id", left_box_id},
      {"right_box_id", right_box_id},
      {"box_front_x", box_front_x_},
      {"scene_y_shift", scene_y_shift_},
      {"snapshot_path", extract_monitor_snapshot_path_},
      {"ik_trial_count", ik_result.trial_count},
      {"ik_legal_count", ik_result.legal_count},
      {"ik_wall_ms", ik_result.wall_ms},
      {"ik_dedup_enabled", dedup_stats.enabled},
      {"ik_dedup_input_count", dedup_stats.input_count},
      {"ik_dedup_unique_count", dedup_stats.unique_count},
      {"ik_dedup_removed_count", dedup_stats.removed_count},
      {"ik_dedup_selected_count", dedup_stats.selected_count},
      {"ik_dedup_ms", dedup_stats.elapsed_ms},
      {"rejection_counts", ik_candidate_rejection_counts_json(ik_result)},
      {"records", records}
    };
    if (!write_extract_monitor_snapshot(snapshot)) {
      return fail("extract monitor IK: failed to write snapshot");
    }

    extract_monitor_phase_ = ExtractMonitorPhase::ReadyForExtract;
    std::ostringstream out;
    out << "IK阶段完成: unique=" << extract_monitor_state_.legal_candidates.size()
        << " legal=" << ik_result.legal_count
        << " trials=" << ik_result.trial_count
        << " elapsed=" << elapsed_ms << "ms snapshot=" << extract_monitor_snapshot_path_;
    *message = out.str();
    return true;
  }

  bool run_extract_monitor_extract_stage(std::string* message)
  {
    if (!extract_monitor_state_.seed_state || extract_monitor_state_.legal_candidates.empty()) {
      return fail("extract monitor extract: IK stage has no candidates");
    }

    const auto stage_start = std::chrono::steady_clock::now();
    const size_t count = extract_monitor_state_.legal_candidates.size();
    extract_monitor_state_.timings.clear();
    extract_monitor_state_.timings.resize(count);

    const size_t worker_count = std::max<size_t>(1, std::min(extract_benchmark_extract_workers_, count));
    std::atomic<size_t> next_index{0};
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (size_t worker = 0; worker < worker_count; ++worker) {
      workers.emplace_back([&, worker]() {
        (void)worker;
        while (true) {
          const size_t index = next_index.fetch_add(1);
          if (index >= count) {
            break;
          }
          const auto state = state_from_ik_candidate(
            *extract_monitor_state_.seed_state,
            extract_monitor_state_.legal_candidates[index]);
          std::vector<nlohmann::json> rollout_records;
          auto record_step = [&](size_t step, const moveit::core::RobotState& step_state, const nlohmann::json& extra) {
            trajectory_msgs::msg::JointTrajectory traj;
            const auto names = arm_joint_target_names();
            traj.joint_names = names;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(step));
            point.positions.reserve(names.size());
            for (const auto& name : names) {
              point.positions.push_back(step_state.getVariablePosition(name));
            }
            traj.points.push_back(point);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_.joint_trajectory = traj;

            nlohmann::json enriched = extra;
            enriched["stage_kind"] = "monitor_selected_extract_replay";
            enriched["candidate_order"] = index;
            enriched["left_box_id"] = extract_monitor_state_.left_box_id;
            enriched["right_box_id"] = extract_monitor_state_.right_box_id;
            rollout_records.push_back(monitor_stage_json(
              extract_monitor_state_.prefix + "/selected_extract_step_" + std::to_string(step),
              plan,
              step_state,
              step_state,
              names,
              {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
              enriched));
          };
          auto timing = rollout_dual_extract_from_state(
            state,
            extract_monitor_state_.left_box,
            extract_monitor_state_.left_box_id,
            extract_monitor_state_.right_box,
            extract_monitor_state_.right_box_id,
            index,
            extract_monitor_state_.legal_candidates[index],
            record_step);
          if (timing.success) {
            timing.rollout_records = std::move(rollout_records);
          }
          fill_loaded_pose_distance_metrics(timing);
          extract_monitor_state_.timings[index] = std::move(timing);
        }
      });
    }
    for (auto& worker : workers) {
      worker.join();
    }

    nlohmann::json records = nlohmann::json::array();
    size_t success_count = 0;
    std::map<std::string, size_t> failure_counts;
    for (size_t i = 0; i < extract_monitor_state_.timings.size(); ++i) {
      const auto& timing = extract_monitor_state_.timings[i];
      if (timing.success && timing.final_state) {
        records.push_back(monitor_timing_json(timing, success_count, *timing.final_state));
        ++success_count;
      } else {
        failure_counts[timing.failure_reason.empty() ? "unknown" : timing.failure_reason]++;
      }
    }

    nlohmann::json failure_json = nlohmann::json::object();
    for (const auto& [reason, count_value] : failure_counts) {
      failure_json[reason] = count_value;
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    nlohmann::json snapshot = {
      {"type", "extract_monitor_snapshot"},
      {"phase", "extract_successes"},
      {"phase_label", "抽离成功候选"},
      {"elapsed_ms", elapsed_ms},
      {"left_box_id", extract_monitor_state_.left_box_id},
      {"right_box_id", extract_monitor_state_.right_box_id},
      {"box_front_x", box_front_x_},
      {"scene_y_shift", scene_y_shift_},
      {"input_candidate_count", count},
      {"success_count", success_count},
      {"worker_count", worker_count},
      {"failure_counts", failure_json},
      {"records", records}
    };
    if (!write_extract_monitor_snapshot(snapshot)) {
      return fail("extract monitor extract: failed to write snapshot");
    }

    extract_monitor_phase_ = ExtractMonitorPhase::ReadyForLoaded;
    std::ostringstream out;
    out << "抽离阶段完成: success=" << success_count << "/" << count
        << " workers=" << worker_count
        << " elapsed=" << elapsed_ms << "ms snapshot=" << extract_monitor_snapshot_path_;
    *message = out.str();
    return true;
  }

  bool run_extract_monitor_loaded_stage(std::string* message)
  {
    if (extract_monitor_state_.timings.empty()) {
      return fail("extract monitor loaded: extract stage has no timings");
    }

    const auto stage_start = std::chrono::steady_clock::now();
    auto options = loaded_pose_batch_plan_options();
    options.enabled = true;
    const std::vector<AttachedBoxSpec> boxes{
      extract_monitor_state_.left_box,
      extract_monitor_state_.right_box,
    };
    const LoadedPoseBatchPlanResult batch = loaded_pose_planner_
      ? loaded_pose_planner_->planBatch(
          extract_monitor_state_.prefix + "/monitor_loaded",
          extract_monitor_state_.timings,
          boxes,
          options)
      : LoadedPoseBatchPlanResult{};

    nlohmann::json records = nlohmann::json::array();
    size_t success_count = 0;
    size_t attempted_count = 0;
    std::map<std::string, size_t> failure_counts;
    for (const auto index : batch.plan_indices) {
      if (index >= extract_monitor_state_.timings.size()) {
        continue;
      }
      const auto& timing = extract_monitor_state_.timings[index];
      if (timing.loaded_plan_attempted) {
        ++attempted_count;
      }
      if (timing.loaded_plan_attempted && timing.final_state && loaded_pose_selector_) {
        moveit::core::RobotState record_state = timing.loaded_goal_state
          ? *timing.loaded_goal_state
          : loaded_pose_selector_->makeGoalState(*timing.final_state);
        records.push_back(monitor_timing_json(timing, records.size(), record_state));
      }
      if (timing.loaded_plan_success && timing.final_state && loaded_pose_selector_) {
        ++success_count;
      } else if (timing.loaded_plan_attempted) {
        failure_counts[timing.loaded_plan_failure_reason.empty() ? "unknown" : timing.loaded_plan_failure_reason]++;
      }
    }

    nlohmann::json failure_json = nlohmann::json::object();
    for (const auto& [reason, count_value] : failure_counts) {
      failure_json[reason] = count_value;
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    nlohmann::json snapshot = {
      {"type", "extract_monitor_snapshot"},
      {"phase", "loaded_plan_successes"},
      {"phase_label", "负重规划成功候选"},
      {"elapsed_ms", elapsed_ms},
      {"left_box_id", extract_monitor_state_.left_box_id},
      {"right_box_id", extract_monitor_state_.right_box_id},
      {"box_front_x", box_front_x_},
      {"scene_y_shift", scene_y_shift_},
      {"extract_success_count", batch.plan_indices.size()},
      {"attempted_count", attempted_count},
      {"success_count", success_count},
      {"loaded_plan_batch_wall_ms", batch.wall_ms},
      {"loaded_parallel_workers", options.parallel_workers},
      {"loaded_candidate_limit", options.candidate_limit},
      {"failure_counts", failure_json},
      {"records", records}
    };
    if (!write_extract_monitor_snapshot(snapshot)) {
      return fail("extract monitor loaded: failed to write snapshot");
    }

    extract_monitor_phase_ = ExtractMonitorPhase::ReadyForFinal;
    std::ostringstream out;
    out << "负重规划阶段完成: success=" << success_count
        << " attempted=" << attempted_count
        << " candidates=" << batch.plan_indices.size()
        << " elapsed=" << elapsed_ms << "ms snapshot=" << extract_monitor_snapshot_path_;
    *message = out.str();
    return true;
  }

  bool run_extract_monitor_final_stage(std::string* message)
  {
    const auto stage_start = std::chrono::steady_clock::now();
    ExtractRolloutTiming* selected = nullptr;
    auto pre_attach_is_smooth = [&](const ExtractRolloutTiming& timing) {
      if (!extract_monitor_state_.loaded_start_state ||
          timing.candidate_order >= extract_monitor_state_.legal_candidates.size() ||
          !extract_monitor_state_.seed_state) {
        return false;
      }
      const auto ik_state = state_from_ik_candidate(
        *extract_monitor_state_.seed_state,
        extract_monitor_state_.legal_candidates[timing.candidate_order]);
      auto plan = make_interpolated_joint_plan(*extract_monitor_state_.loaded_start_state, ik_state, 1.0);
      std::string reason;
      return planned_trajectory_clear_in_full_scene(plan, *extract_monitor_state_.loaded_start_state, {}, &reason);
    };

    for (auto& timing : extract_monitor_state_.timings) {
      if (timing.loaded_plan_success && pre_attach_is_smooth(timing)) {
        selected = &timing;
        break;
      }
    }
    if (!selected) {
      for (auto& timing : extract_monitor_state_.timings) {
        if (timing.loaded_plan_success) {
          selected = &timing;
          break;
        }
      }
    }
    if (!selected || !selected->final_state || !loaded_pose_selector_) {
      return fail("extract monitor final: no loaded-plan success to select");
    }

    moveit::core::RobotState goal_state = selected->loaded_goal_state
      ? *selected->loaded_goal_state
      : loaded_pose_selector_->makeGoalState(*selected->final_state);
    moveit::core::RobotState ik_goal_state = selected->candidate_order < extract_monitor_state_.legal_candidates.size() &&
        extract_monitor_state_.seed_state
      ? state_from_ik_candidate(
          *extract_monitor_state_.seed_state,
          extract_monitor_state_.legal_candidates[selected->candidate_order])
      : *selected->final_state;

    if (selected->rollout_records.empty() &&
        selected->candidate_order < extract_monitor_state_.legal_candidates.size() &&
        extract_monitor_state_.seed_state) {
      std::vector<nlohmann::json> rollout_records;
      auto record_step = [&](size_t step, const moveit::core::RobotState& state, const nlohmann::json& extra) {
        trajectory_msgs::msg::JointTrajectory traj;
        const auto names = arm_joint_target_names();
        traj.joint_names = names;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(step));
        point.positions.reserve(names.size());
        for (const auto& name : names) {
          point.positions.push_back(state.getVariablePosition(name));
        }
        traj.points.push_back(point);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = traj;

        nlohmann::json enriched = extra;
        enriched["stage_kind"] = "monitor_selected_extract_replay";
        enriched["candidate_order"] = selected->candidate_order;
        enriched["left_box_id"] = extract_monitor_state_.left_box_id;
        enriched["right_box_id"] = extract_monitor_state_.right_box_id;
        rollout_records.push_back(monitor_stage_json(
          extract_monitor_state_.prefix + "/selected_extract_step_" + std::to_string(step),
          plan,
          state,
          state,
          names,
          {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
          enriched));
      };
      const auto& candidate = extract_monitor_state_.legal_candidates[selected->candidate_order];
      auto replay_timing = rollout_dual_extract_from_state(
        state_from_ik_candidate(*extract_monitor_state_.seed_state, candidate),
        extract_monitor_state_.left_box,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box,
        extract_monitor_state_.right_box_id,
        selected->candidate_order,
        candidate,
        record_step);
      if (replay_timing.success) {
        selected->rollout_records = std::move(rollout_records);
      }
    }

    nlohmann::json replay_stages = nlohmann::json::array();
    if (extract_monitor_state_.loaded_start_state) {
      const auto transition_t0 = std::chrono::steady_clock::now();
      moveit::planning_interface::MoveGroupInterface::Plan transition_plan =
        make_interpolated_joint_plan(*extract_monitor_state_.loaded_start_state, ik_goal_state, 1.0);
      transition_plan = densify_joint_plan(transition_plan, 5.0 * M_PI / 180.0, 0.01);
      std::string transition_reason;
      bool transition_ok =
        planned_trajectory_clear_in_full_scene(
          transition_plan, *extract_monitor_state_.loaded_start_state, {}, &transition_reason);
      std::string transition_method = "joint_interpolation";
      if (!transition_ok) {
        transition_method = "rrt";
        transition_reason.clear();
        transition_ok = plan_joint_space_with_direct_pipeline(
          *extract_monitor_state_.loaded_start_state, ik_goal_state, &transition_plan, &transition_reason);
        if (transition_ok) {
          std::string shortcut_reason;
          transition_plan = shortcut_joint_plan(
            transition_plan, *extract_monitor_state_.loaded_start_state, {}, &shortcut_reason);
          transition_plan = densify_joint_plan(transition_plan, 5.0 * M_PI / 180.0, 0.01);
          transition_ok = planned_trajectory_clear_in_full_scene(
            transition_plan, *extract_monitor_state_.loaded_start_state, {}, &transition_reason);
          if (!shortcut_reason.empty()) {
            transition_reason = shortcut_reason + (transition_reason.empty() ? "" : "; " + transition_reason);
          }
        }
      }
      const double transition_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - transition_t0).count();
      nlohmann::json extra = {
        {"stage_kind", "monitor_selected_pre_attach_loaded_to_ik_replay"},
        {"valid", transition_ok},
        {"method", transition_method},
        {"transition_ms", transition_ms},
        {"failure_reason", transition_ok ? "" : transition_reason},
        {"candidate_order", selected->candidate_order},
        {"loaded_plan_rank", selected->loaded_plan_rank},
        {"left_box_id", extract_monitor_state_.left_box_id},
        {"right_box_id", extract_monitor_state_.right_box_id}
      };
      replay_stages.push_back(monitor_stage_json(
        extract_monitor_state_.prefix + "/selected_pre_attach_loaded_to_ik",
        transition_plan,
        *extract_monitor_state_.loaded_start_state,
        ik_goal_state,
        arm_joint_target_names(),
        {},
        extra));
    }
    for (const auto& stage : selected->rollout_records) {
      replay_stages.push_back(stage);
    }
    for (const auto& shift_stage : selected->lateral_shift_replay_stages) {
      if (!shift_stage.start_state || !shift_stage.goal_state) {
        continue;
      }
      nlohmann::json extra = shift_stage.extra;
      extra["candidate_order"] = selected->candidate_order;
      extra["loaded_plan_rank"] = selected->loaded_plan_rank;
      extra["left_box_id"] = extract_monitor_state_.left_box_id;
      extra["right_box_id"] = extract_monitor_state_.right_box_id;
      replay_stages.push_back(monitor_stage_json(
        shift_stage.stage_name,
        shift_stage.plan,
        *shift_stage.start_state,
        *shift_stage.goal_state,
        arm_joint_target_names(),
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        extra));
    }
    if (selected->loaded_start_state && selected->loaded_goal_state &&
        !selected->loaded_plan.trajectory_.joint_trajectory.points.empty()) {
      nlohmann::json extra = {
        {"stage_kind", "monitor_selected_loaded_plan_replay"},
        {"valid", true},
        {"candidate_order", selected->candidate_order},
        {"loaded_plan_rank", selected->loaded_plan_rank},
        {"loaded_plan_ms", selected->loaded_plan_ms},
        {"loaded_plan_points", selected->loaded_plan_points},
        {"loaded_plan_trajectory_distance", selected->loaded_plan_trajectory_distance},
        {"moveit_attached_box_count", 2},
        {"left_box_id", extract_monitor_state_.left_box_id},
        {"right_box_id", extract_monitor_state_.right_box_id}
      };
      replay_stages.push_back(monitor_stage_json(
        extract_monitor_state_.prefix + "/selected_loaded_plan",
        selected->loaded_plan,
        *selected->loaded_start_state,
        *selected->loaded_goal_state,
        arm_joint_target_names(),
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        extra));
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    nlohmann::json snapshot = {
      {"type", "extract_monitor_snapshot"},
      {"phase", "final_selected"},
      {"phase_label", "最终采用方案"},
      {"elapsed_ms", elapsed_ms},
      {"left_box_id", extract_monitor_state_.left_box_id},
      {"right_box_id", extract_monitor_state_.right_box_id},
      {"box_front_x", box_front_x_},
      {"scene_y_shift", scene_y_shift_},
      {"records", nlohmann::json::array({monitor_timing_json(*selected, 0, goal_state)})},
      {"replay_stages", replay_stages}
    };
    if (!write_extract_monitor_snapshot(snapshot)) {
      return fail("extract monitor final: failed to write snapshot");
    }

    extract_monitor_phase_ = ExtractMonitorPhase::Done;
    std::ostringstream out;
    out << "最终方案已选择: candidate_order=" << selected->candidate_order
        << " loaded_rank=" << selected->loaded_plan_rank
        << " trajectory_distance=" << selected->loaded_plan_trajectory_distance
        << " elapsed=" << elapsed_ms << "ms snapshot=" << extract_monitor_snapshot_path_;
    *message = out.str();
    return true;
  }

  bool run_left_extract_demo()
  {
    ExtractDemoOrchestrator orchestrator(extract_demo_config(), extract_demo_callbacks());
    return orchestrator.run();
  }

  bool run_left_extract_pair(int left_box_id, int right_box_id)
  {
    clear_carried_boxes_from_scene();
    last_error_.clear();
    last_commanded_state_.reset();

    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    const auto right_it = boxes.find(right_box_id);
    if (left_it == boxes.end() || right_it == boxes.end()) {
      return fail("left extract demo: unknown box id");
    }
    set_static_box_wall_opening(left_box_id, right_box_id, "left_extract_pair");

    const std::string prefix = "left_extract_demo_L" + std::to_string(left_box_id) +
                               "_R" + std::to_string(right_box_id);

    const auto left_pose = front_grasp_pose(left_it->second);
    const auto right_pose = front_grasp_pose(right_it->second);

    if (extract_demo_direct_grasp_start_) {
      auto seed_state = std::make_shared<moveit::core::RobotState>(robot_model_);
      seed_state->setToDefaultValues();
      for (size_t i = 0; i < left_pregrasp_arm_.size(); ++i) {
        seed_state->setVariablePosition("left_v5_joint" + std::to_string(i + 1), left_pregrasp_arm_[i]);
      }
      for (size_t i = 0; i < right_pregrasp_arm_.size(); ++i) {
        seed_state->setVariablePosition("right_v5_joint" + std::to_string(i + 1), right_pregrasp_arm_[i]);
      }
      seed_state->setVariablePosition("updown", extract_grasp_ik_home_updown_);
      seed_state->enforceBounds(joint_group_);
      seed_state->update();

      moveit::core::RobotState grasp_state(*seed_state);
      nlohmann::json grasp_extra;
      ik_benchmark::UpdownAwareIkResult direct_ik_result;
      if (!solve_dual_tip_ik_state(prefix + "/grasp_ik_direct_start", left_pose, right_pose, false,
                                   *seed_state, &grasp_state, &grasp_extra, &direct_ik_result)) {
        return false;
      }
      if (extract_benchmark_all_legal_ik_) {
        const AttachedBoxSpec left_box = make_carried_box_spec("left", left_box_id, false);
        const AttachedBoxSpec right_box = make_carried_box_spec("right", right_box_id, false);
        const std::string original_csv_path = extract_benchmark_csv_path_;
        if (extract_demo_all_rows_ && !original_csv_path.empty()) {
          const std::filesystem::path csv_path(original_csv_path);
          const std::string stem = csv_path.stem().string() + "_L" + std::to_string(left_box_id) +
                                   "_R" + std::to_string(right_box_id);
          extract_benchmark_csv_path_ = (csv_path.parent_path() / (stem + csv_path.extension().string())).string();
        }
        const bool ok = extract_benchmark_dual_arm_
          ? benchmark_all_legal_ik_dual_extract(prefix, *seed_state, direct_ik_result,
                                                left_box, left_box_id, right_box, right_box_id)
          : benchmark_all_legal_ik_extract(prefix, *seed_state, direct_ik_result,
                                           left_box, left_box_id);
        extract_benchmark_csv_path_ = original_csv_path;
        if (recording_enabled() && !extract_demo_all_rows_) {
          recorder_->write(nlohmann::json({
            {"type", "summary"},
            {"success", ok},
            {"error", ok ? "" : last_error_},
            {"stages", recorded_stage_count()}
          }));
        }
        return ok;
      }
      last_commanded_state_ = std::make_shared<moveit::core::RobotState>(grasp_state);
      record_extract_keyframe(prefix + "/grasp_ik_direct_start", grasp_state,
                              make_carried_box_spec("left", left_box_id, false), grasp_extra);
    } else {
      if (!plan_to_joint_target(prefix + "/pregrasp",
                                make_dual_arm_joint_target(fixed_updown_, left_pregrasp_arm_, right_pregrasp_arm_))) {
        return false;
      }

      if (!plan_dual_tip_ik(prefix + "/grasp_ik", left_pose, right_pose, false)) {
        return false;
      }
    }

    if (scene_adapter_) {
      scene_adapter_->setActiveAttachedBoxesForRecordOnly({make_carried_box_spec("left", left_box_id, false)});
    }
    if (!apply_attached_box_state(active_attached_boxes(), moveit_msgs::msg::CollisionObject::ADD,
                                  prefix + "/attach_left_carried_box")) {
      if (scene_adapter_) scene_adapter_->clearActiveAttachedBoxesForRecordOnly();
      return false;
    }

    if (last_commanded_state_) {
      record_extract_keyframe(
        prefix + "/attach_hold",
        *last_commanded_state_,
        active_attached_boxes().front(),
        {
          {"stage_kind", "attach_hold"},
          {"left_box_id", left_box_id},
          {"note", "attached box added without changing robot joint state"}
        });
    }

    const bool ok = plan_left_extract_primitive(left_box_id, prefix);
    detach_carried_boxes();
    if (recording_enabled() && !extract_demo_all_rows_) {
      recorder_->write(nlohmann::json({
        {"type", "summary"},
        {"success", ok},
        {"error", ok ? "" : last_error_},
        {"stages", recorded_stage_count()}
      }));
    }
    return ok;
  }

  bool run_one_pair_flow(int left_box_id, int right_box_id, bool top_suction, int round)
  {
    BoxStackFlowOrchestrator orchestrator(box_stack_flow_config(), box_stack_flow_callbacks());
    return orchestrator.runOnePair(left_box_id, right_box_id, top_suction, round);
  }

  bool run_box_stack_flow()
  {
    last_error_.clear();
    last_commanded_state_.reset();

    BoxStackFlowOrchestrator orchestrator(box_stack_flow_config(), box_stack_flow_callbacks());
    const bool ok = orchestrator.run();
    if (!ok) {
      return false;
    }
    if (recording_enabled()) {
      recorder_->write(nlohmann::json({
        {"type", "summary"},
        {"success", true},
        {"stages", recorded_stage_count()}
      }));
    }
    return true;
  }

  bool fail(const std::string& message)
  {
    last_error_ = message;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    if (recording_enabled()) {
      recorder_->write(nlohmann::json({
        {"type", "summary"},
        {"success", false},
        {"error", message},
        {"stages", recorded_stage_count()}
      }));
    }
    return false;
  }

  std::string planning_group_;
  std::string left_tip_;
  std::string right_tip_;
  bool execute_ = true;
  std::string execution_backend_ = "moveit";
  std::string execution_action_name_ = "/alfa_execution/execute_joint_trajectory";
  double execution_action_wait_timeout_s_ = 5.0;
  double execution_result_timeout_s_ = 0.0;
  bool execution_include_turn_ = true;
  bool execution_allow_hold_missing_target_joints_ = true;
  bool execution_reject_unmapped_planned_joints_ = true;
  bool reject_ik_collisions_ = false;
  bool check_goal_collision_ = false;
  bool prefer_commanded_state_ = true;
  bool include_top_suction_ = true;
  double fixed_updown_ = 0.45;
  double box_front_x_ = 0.625;
  double scene_y_shift_ = 0.0;
  double world_to_base_z_ = 0.202094;
  double top_suction_x_offset_ = 0.15;
  double top_suction_z_offset_ = 0.2;
  double ik_timeout_ = 2.0;
  double planning_time_ = 8.0;
  double velocity_scale_ = 1.0;
  double acceleration_scale_ = 1.0;
  double joint_goal_tolerance_rad_ = 0.02;
  double state_wait_timeout_s_ = 2.0;
  bool enable_container_obstacle_ = true;
  std::string container_frame_ = "world";
  double container_length_ = 4.0;
  double container_width_ = 2.2;
  double container_height_ = 2.4;
  double container_center_x_ = 0.8;
  double container_center_y_ = 0.0;
  double container_floor_z_ = 0.0;
  double container_wall_thickness_ = 0.02;
  bool enable_attached_box_collision_ = true;
  double carried_box_depth_ = 0.3;
  double carried_box_width_ = 0.4;
  double carried_box_height_ = 0.4;
  double attached_box_collision_padding_ = -0.002;
  bool enforce_loaded_plan_aabb_clearance_ = true;
  bool enable_static_box_obstacles_ = true;
  double static_box_obstacle_inset_ = 0.002;
  int extract_demo_left_box_id_ = 2;
  int extract_demo_right_box_id_ = 4;
  std::vector<std::pair<int, int>> extract_demo_pair_sequence_{{2, 4}, {7, 9}, {12, 14}, {17, 19}};
  bool extract_demo_all_rows_ = false;
  double extract_step_x_ = 0.03;
  double extract_max_x_ = 0.36;
  std::vector<double> extract_lift_candidates_;
  std::vector<double> extract_pitch_candidates_deg_;
  double extract_neighbor_margin_ = 0.02;
  bool extract_fail_fast_ = false;
  size_t extract_success_extra_steps_ = 3;
  double extract_kdl_timeout_ = 0.01;
  double extract_position_tolerance_ = 0.01;
  double extract_orientation_tolerance_ = 0.05;
  double extract_max_tip_z_drop_ = 0.002;
  double extract_min_tool_normal_z_ = -1e-4;
  double extract_score_lift_weight_ = 10.0;
  double extract_score_pitch_weight_ = 0.02;
  double extract_score_retreat_continuity_weight_ = 0.2;
  double extract_score_joint_delta_weight_ = 0.6;
  double extract_score_tip_position_delta_weight_ = 2.0;
  double extract_score_tip_orientation_delta_weight_ = 0.05;
  double extract_max_joint_delta_ = 0.0;
  bool extract_demo_direct_grasp_start_ = false;
  double extract_grasp_ik_home_updown_ = 0.3;
  bool extract_benchmark_all_legal_ik_ = false;
  bool extract_benchmark_dual_arm_ = false;
  bool extract_benchmark_dual_async_ = false;
  std::string extract_benchmark_csv_path_;
  bool extract_benchmark_record_rollouts_ = false;
  size_t extract_benchmark_candidate_limit_ = 0;
  size_t extract_benchmark_extract_workers_ = 1;
  bool extract_ik_dedup_enabled_ = false;
  double extract_ik_dedup_joint_threshold_ = 1.0 * M_PI / 180.0;
  double extract_ik_dedup_h_threshold_ = 0.005;
  bool extract_benchmark_plan_loaded_after_success_ = false;
  std::string extract_loaded_planning_group_ = "dual_v5_arm_with_base";
  double extract_loaded_planning_time_ = 1.0;
  int extract_loaded_planning_attempts_ = 8;
  bool extract_loaded_use_direct_pipeline_ = false;
  size_t extract_loaded_parallel_workers_ = 1;
  size_t extract_loaded_candidate_limit_ = 0;
  bool extract_loaded_sort_by_pose_distance_ = false;
  bool extract_loaded_stop_on_first_success_ = false;
  double extract_loaded_target_updown_ = 0.3;
  bool extract_loaded_lateral_shift_enabled_ = false;
  double extract_loaded_lateral_shift_distance_ = 0.4;
  double extract_loaded_lateral_shift_step_ = 0.0;
  int extract_loaded_lateral_shift_column_ = 3;
  int extract_loaded_pre_lower_left_box_id_ = 0;
  int extract_loaded_pre_lower_right_box_id_ = 0;
  double extract_loaded_pre_lower_updown_delta_ = 0.0;
  bool extract_use_independent_kdl_ = false;
  int extract_independent_kdl_max_iterations_ = 120;
  double extract_independent_kdl_eps_ = 1e-5;
  int extract_independent_kdl_seed_attempts_ = 1;
  double extract_independent_kdl_seed_jitter_ = 8.0 * M_PI / 180.0;
  bool record_tip_error_ik_candidates_ = false;
  size_t record_tip_error_ik_candidate_limit_ = 80;
  std::string record_jsonl_path_;
  bool record_trajectories_ = true;
  std::string extract_monitor_snapshot_path_;
  int max_rounds_ = 10;
  int planning_attempts_ = 8;
  std::vector<double> left_pregrasp_arm_;
  std::vector<double> right_pregrasp_arm_;
  std::vector<double> left_loaded_arm_;
  std::vector<double> right_loaded_arm_;
  std::vector<std::vector<double>> left_loaded_pose_family_;
  std::vector<std::vector<double>> right_loaded_pose_family_;
  size_t left_preferred_loaded_pose_index_ = 0;
  size_t right_preferred_loaded_pose_index_ = 0;
  std::string last_error_;
  ik_benchmark::UpdownAwareIkConfig ik_config_;
  LoadedPoseSelectorConfig loaded_pose_selector_config_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> loaded_move_group_;
  std::unique_ptr<LoadedPoseSelector> loaded_pose_selector_;
  std::unique_ptr<LoadedPosePlanner> loaded_pose_planner_;
  std::unique_ptr<ExtractMotionPlanner> extract_motion_planner_;
  std::unique_ptr<ExtractCandidateScorer> extract_candidate_scorer_;
  std::unique_ptr<ExtractCandidateSolver> extract_candidate_solver_;
  std::unique_ptr<ExtractRolloutPlanner> extract_rollout_planner_;
  std::unique_ptr<IkCandidateSelector> ik_candidate_selector_;
  std::unique_ptr<MotionSceneAdapter> scene_adapter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_pipeline::PlanningPipelinePtr loaded_planning_pipeline_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  const moveit::core::JointModelGroup* left_arm_group_ = nullptr;
  const moveit::core::JointModelGroup* right_arm_group_ = nullptr;
  moveit::core::RobotStatePtr last_commanded_state_;
  std::unique_ptr<ik_benchmark::ParallelUpdownAwareIkSolver> optimized_ik_solver_;
  std::unique_ptr<OptimizedDualIkSolver> optimized_dual_ik_solver_;
  std::unique_ptr<MotionFlowRecorder> recorder_;

  enum class ExtractMonitorPhase
  {
    ReadyForIk,
    ReadyForExtract,
    ReadyForLoaded,
    ReadyForFinal,
    Done,
  };

  struct ExtractMonitorState
  {
    int left_box_id = 0;
    int right_box_id = 0;
    std::string prefix;
    AttachedBoxSpec left_box;
    AttachedBoxSpec right_box;
    moveit::core::RobotStatePtr seed_state;
    moveit::core::RobotStatePtr loaded_start_state;
    ik_benchmark::UpdownAwareIkResult ik_result;
    std::vector<ik_benchmark::UpdownAwareIkCandidate> legal_candidates;
    std::vector<moveit::core::RobotStatePtr> candidate_states;
    std::vector<ExtractRolloutTiming> timings;
  };

  ExtractMonitorPhase extract_monitor_phase_ = ExtractMonitorPhase::ReadyForIk;
  ExtractMonitorState extract_monitor_state_;
  std::mutex extract_monitor_mutex_;
  double extract_monitor_last_stage_ms_ = 0.0;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr demo_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr box_stack_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_demo_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_monitor_next_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_monitor_full_selected_srv_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr execution_action_client_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joint_state_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<DualArmPlannerNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  try {
    node->init();
    spin_thread.join();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "dual_arm_planner init failed: %s", error.what());
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
