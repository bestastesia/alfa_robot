/**
 * dual_arm_planner_node.cpp
 *
 * MoveIt-backed dual-arm box-stack flow reproducer.
 *
 * Grasp IK uses the current deterministic analytic solver:
 *   fixed discrete h candidates × closed-form arm branches × cost scoring.
 * MoveIt is only used for joint-space trajectory planning/execution.
 */

#include "alfa_robot_moveit_config/box_stack_flow_orchestrator.hpp"
#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"
#include "alfa_robot_moveit_config/extract_demo_orchestrator.hpp"
#include "alfa_robot_moveit_config/extract_monitor_json.hpp"
#include "alfa_robot_moveit_config/extract_monitor_replay_builder.hpp"
#include "alfa_robot_moveit_config/extract_monitor_snapshot_writer.hpp"
#include "alfa_robot_moveit_config/extract_monitor_state.hpp"
#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"
#include "alfa_robot_moveit_config/execution_trajectory_adapter.hpp"
#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"
#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"
#include "alfa_robot_moveit_config/motion_flow_recorder.hpp"
#include "alfa_robot_moveit_config/planning_diagnostics.hpp"
#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"
#include "robot_motion_scene_service/motion_scene_adapter.hpp"
#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"
#include "robot_motion_scene_service/motion_core/scene_geometry.hpp"
#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

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
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "alfa_robot_moveit_config/srv/configure_extract_monitor.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <array>
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
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace
{

using alfa_robot::motion::AttachedBoxSpec;
using alfa_robot::motion::ArmExtractPath;
using alfa_robot::motion::ExtractBenchmarkRunner;
using alfa_robot::motion::ExtractBenchmarkRunnerCallbacks;
using alfa_robot::motion::ExecutionTrajectoryAdapter;
using alfa_robot::motion::ExecutionTrajectoryAdapterConfig;
using alfa_robot::motion::ExecutionJointStateMatchRequest;
using alfa_robot::motion::ExecutionStateMatchRequest;
using alfa_robot::motion::attached_boxes_json;
using alfa_robot::motion::ExecutionTrajectoryBuildRequest;
using alfa_robot::motion::ExtractMonitorArmSeed;
using alfa_robot::motion::ExtractMonitorController;
using alfa_robot::motion::ExtractMonitorExtractSnapshotRequest;
using alfa_robot::motion::ExtractMonitorExtractStageMessageRequest;
using alfa_robot::motion::ExtractMonitorFinalSnapshotRequest;
using alfa_robot::motion::ExtractMonitorFinalStageMessageRequest;
using alfa_robot::motion::ExtractMonitorFullSelectedSnapshotRequest;
using alfa_robot::motion::ExtractMonitorInitialStateRequest;
using alfa_robot::motion::ExtractMonitorIkSnapshotRequest;
using alfa_robot::motion::ExtractMonitorIkStageMessageRequest;
using alfa_robot::motion::ExtractMonitorLoadedSnapshotRequest;
using alfa_robot::motion::ExtractMonitorLoadedStageMessageRequest;
using alfa_robot::motion::ExtractMonitorSnapshotWriter;
using alfa_robot::motion::ExtractMonitorState;
using alfa_robot::motion::ExtractMonitorStageSnapshotWriteRequest;
using alfa_robot::motion::ExtractMonitorStageCallbacks;
using alfa_robot::motion::ExtractMonitorReplayBuilder;
using alfa_robot::motion::ExtractMonitorSelectedExtractReplayStateRequest;
using alfa_robot::motion::ExtractMonitorTransitionPlanner;
using alfa_robot::motion::ExtractMonitorTimingRecordsRequest;
using alfa_robot::motion::extract_monitor_candidate_records_json;
using alfa_robot::motion::extract_monitor_extract_snapshot;
using alfa_robot::motion::extract_monitor_final_snapshot;
using alfa_robot::motion::extract_monitor_final_stage_message;
using alfa_robot::motion::extract_monitor_ik_snapshot;
using alfa_robot::motion::extract_monitor_ik_stage_message;
using alfa_robot::motion::extract_monitor_loaded_snapshot;
using alfa_robot::motion::extract_monitor_loaded_stage_message;
using alfa_robot::motion::failure_counts_json;
using alfa_robot::motion::extract_monitor_candidate_state_for_timing;
using alfa_robot::motion::extract_monitor_candidate_for_timing;
using alfa_robot::motion::extract_monitor_extract_stage_message;
using alfa_robot::motion::extract_monitor_snapshot_base;
using alfa_robot::motion::extract_monitor_selected_extract_replay_state_stage;
using alfa_robot::motion::extract_monitor_timing_records_json;
using alfa_robot::motion::ik_candidate_rejection_counts_json;
using alfa_robot::motion::dual_arm_with_updown_joint_names;
using alfa_robot::motion::populate_extract_monitor_candidate_states;
using alfa_robot::motion::run_extract_monitor_candidate_tasks;
using alfa_robot::motion::robot_state_from_ik_candidate;
using alfa_robot::motion::select_extract_monitor_final_timing;
using alfa_robot::motion::single_state_plan;
using alfa_robot::motion::summarize_extract_monitor_timings;
using alfa_robot::motion::summarize_loaded_plan_timings;
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
using alfa_robot::motion::BoxPoseRrtExtractPlanner;
using alfa_robot::motion::BoxPoseRrtExtractPlannerConfig;
using alfa_robot::motion::BoxPoseRrtArmPolicy;
using alfa_robot::motion::BoxSpec;
using alfa_robot::motion::BoxWallGeometryConfig;
using alfa_robot::motion::CarriedBoxGeometryConfig;
using alfa_robot::motion::ContainerGeometryConfig;
using alfa_robot::motion::ContainerPanel;
using alfa_robot::motion::ContainerRelativePose;
using alfa_robot::motion::compute_container_pose_relative_to_vehicle;
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
using alfa_robot::motion::MotionFlowHeaderRequest;
using alfa_robot::motion::MotionFlowRecorder;
using alfa_robot::motion::OptimizedDualIkSolver;
using alfa_robot::motion::OptimizedDualIkSolverConfig;
using alfa_robot::motion::OptimizedDualIkSolveRequest;
using alfa_robot::motion::PickPair;
using alfa_robot::motion::StaticBoxObstacle;
using alfa_robot::motion::aabb_from_attached_box_transform;
using alfa_robot::motion::carried_box_detached_from_neighbors;
using alfa_robot::motion::carried_box_clear_rear_guards;
using alfa_robot::motion::deg_to_rad;
using alfa_robot::motion::direct_pipeline_failure_diagnostic;
using alfa_robot::motion::format_degrees;
using alfa_robot::motion::forward_x_orientation;
using alfa_robot::motion::group_bounds_reason;
using alfa_robot::motion::make_attached_box_spec;
using alfa_robot::motion::make_boxes;
using alfa_robot::motion::make_box_wall_obstacles_for_opening;
using alfa_robot::motion::make_container_panels;
using alfa_robot::motion::make_extract_monitor_joint_state;
using alfa_robot::motion::make_extract_monitor_initial_state;
using alfa_robot::motion::make_extract_monitor_replay_request;
using alfa_robot::motion::make_identity_pose;
using alfa_robot::motion::motion_flow_header_json;
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
using alfa_robot::motion::robot_state_json;
using alfa_robot::motion::scene_collision_reason;
using alfa_robot::motion::shortest_angular_distance;
using alfa_robot::motion::top_suction_orientation;
using alfa_robot::motion::vector_json;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJointTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

std::vector<std::string> touch_links_for_attached_box(const AttachedBoxSpec& box)
{
  std::vector<std::string> links{box.link_name};
  if (box.link_name.rfind("left_", 0) == 0) {
    links.push_back("left_joint6");
    links.push_back("left_joint5");
    links.push_back("left_joint4");
    links.push_back("left_joint3");
  } else if (box.link_name.rfind("right_", 0) == 0) {
    links.push_back("right_joint6");
    links.push_back("right_joint5");
    links.push_back("right_joint4");
    links.push_back("right_joint3");
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
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    planning_group_ = get_or_declare_parameter<std::string>("planning_group", "dual_arm_with_base");
    left_tip_ = get_or_declare_parameter<std::string>("left_tip", "left_tool0");
    right_tip_ = get_or_declare_parameter<std::string>("right_tip", "right_tool0");
    execute_ = get_or_declare_parameter<bool>("execute", true);
    execution_backend_ = get_or_declare_parameter<std::string>("execution_backend", "moveit");
    execution_action_name_ = get_or_declare_parameter<std::string>(
      "execution_action_name", "/dual_arm_trajectory_controller/follow_joint_trajectory");
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
    extract_monitor_snapshot_writer_.setPath(extract_monitor_snapshot_path_);
    extract_monitor_top_suction_ = get_or_declare_parameter<bool>("extract_monitor_top_suction", false);
    extract_monitor_left_top_suction_ =
      get_or_declare_parameter<bool>("extract_monitor_left_top_suction", extract_monitor_top_suction_);
    extract_monitor_right_top_suction_ =
      get_or_declare_parameter<bool>("extract_monitor_right_top_suction", extract_monitor_top_suction_);

    ik_config_.fixed_group = get_or_declare_parameter<std::string>("ik_fixed_group", "dual_arm");
    ik_config_.free_group = get_or_declare_parameter<std::string>("ik_free_group", "dual_arm_with_base");
    ik_config_.solver_plugin = get_or_declare_parameter<std::string>("ik_solver_plugin", "analytic_three_parallel");
    ik_config_.base_frame = get_or_declare_parameter<std::string>("ik_base_frame", "base_link");
    ik_config_.left_tip = left_tip_;
    ik_config_.right_tip = right_tip_;
    ik_config_.tool0_offset = get_or_declare_parameter<double>("ik_tool0_offset", 0.0);
    ik_config_.gripper_z_reach_lower = get_or_declare_parameter<double>("front_z_reach_lower", 0.45) - world_to_base_z_;
    ik_config_.gripper_z_reach_upper = get_or_declare_parameter<double>("front_z_reach_upper", 1.25) - world_to_base_z_;
    ik_config_.top_suction_z_reach_lower = get_or_declare_parameter<double>("top_z_reach_lower", 0.0) - world_to_base_z_;
    ik_config_.top_suction_z_reach_upper = get_or_declare_parameter<double>("top_z_reach_upper", 0.6) - world_to_base_z_;
    ik_config_.h_lower = get_or_declare_parameter<double>("ik_h_lower", 0.0);
    ik_config_.h_upper = get_or_declare_parameter<double>("ik_h_upper", 0.7);
    ik_config_.full_h_range_scan = get_or_declare_parameter<bool>("ik_full_h_range_scan", true);
    ik_config_.h_search_mode = robot_motion::core::UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
    ik_config_.h_search_margin = get_or_declare_parameter<double>("ik_h_search_margin", 0.2);
    ik_config_.h_step = get_or_declare_parameter<double>("ik_h_step", 0.01);
    ik_config_.h_candidate_count = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_h_candidate_count", 64)));
    ik_config_.seed_count = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_seed_count", 32)));
    ik_config_.cost_loaded_family_distance =
      get_or_declare_parameter<double>("ik_loaded_family_distance_weight", 0.2);
    ik_config_.cost_loaded_preferred_distance =
      get_or_declare_parameter<double>("ik_loaded_preferred_distance_weight", 0.1);
    ik_config_.cost_updown_enabled = get_or_declare_parameter<bool>("ik_updown_cost_enabled", false);
    ik_config_.cost_joint_limit_margin =
      get_or_declare_parameter<double>("ik_joint_limit_margin_weight", 1.0);
    ik_config_.joint_limit_weights = {
      get_or_declare_parameter<double>("ik_joint1_limit_weight", 0.5),
      get_or_declare_parameter<double>("ik_joint2_limit_weight", 3.0),
      get_or_declare_parameter<double>("ik_joint3_limit_weight", 0.7),
      get_or_declare_parameter<double>("ik_joint4_limit_weight", 0.5),
      get_or_declare_parameter<double>("ik_joint5_limit_weight", 1.5),
      get_or_declare_parameter<double>("ik_joint6_limit_weight", 1.2),
    };
    ik_config_.joint_limit_free_ratio =
      get_or_declare_parameter<double>("ik_joint_limit_free_ratio", 0.6);
    ik_config_.workers = static_cast<size_t>(std::max(1, get_or_declare_parameter<int>("ik_workers", 1)));
    ik_analytic_root_samples_ = static_cast<size_t>(
      std::max(32, get_or_declare_parameter<int>("ik_analytic_root_samples", 360)));
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
    container_track_vehicle_drift_ = get_or_declare_parameter<bool>("container_track_vehicle_drift", false);
    vehicle_drift_global_frame_ = get_or_declare_parameter<std::string>("vehicle_drift_global_frame", "map");
    vehicle_drift_translation_threshold_m_ =
      get_or_declare_parameter<double>("vehicle_drift_translation_threshold_m", 0.05);
    vehicle_drift_rotation_threshold_rad_ =
      get_or_declare_parameter<double>("vehicle_drift_rotation_threshold_rad", 0.02);
    container_length_ = get_or_declare_parameter<double>("container_length", 4.0);
    container_width_ = get_or_declare_parameter<double>("container_width", 1.8);
    container_height_ = get_or_declare_parameter<double>("container_height", 2.4);
    container_center_x_ = get_or_declare_parameter<double>("container_center_x", 0.8);
    container_center_y_ = get_or_declare_parameter<double>("container_center_y", 0.0);
    container_pose_dynamic_ = get_or_declare_parameter<bool>("container_pose_dynamic", false);
    container_pose_map_x_ = get_or_declare_parameter<double>("container_pose_map_x", container_center_x_);
    container_pose_map_y_ = get_or_declare_parameter<double>("container_pose_map_y", container_center_y_);
    container_pose_map_yaw_ = get_or_declare_parameter<double>("container_pose_map_yaw", 0.0);
    container_floor_z_ = get_or_declare_parameter<double>("container_floor_z", 0.0);
    container_wall_thickness_ = get_or_declare_parameter<double>("container_wall_thickness", 0.02);
    enable_attached_box_collision_ = get_or_declare_parameter<bool>("enable_attached_box_collision", true);
    carried_box_depth_ = get_or_declare_parameter<double>("carried_box_depth", 0.3);
    carried_box_width_ = get_or_declare_parameter<double>("carried_box_width", 0.5);
    carried_box_height_ = get_or_declare_parameter<double>("carried_box_height", 0.4);
    carried_box_grasp_lateral_offset_ =
      get_or_declare_parameter<double>("carried_box_grasp_lateral_offset", 0.0);
    attached_box_collision_padding_ = get_or_declare_parameter<double>("attached_box_collision_padding", -0.002);
    enable_static_box_obstacles_ = get_or_declare_parameter<bool>("enable_static_box_obstacles", true);
    static_box_obstacle_inset_ = get_or_declare_parameter<double>("static_box_obstacle_inset", 0.002);

    extract_demo_left_box_id_ = get_or_declare_parameter<int>("extract_demo_left_box_id", 1);
    extract_demo_right_box_id_ = get_or_declare_parameter<int>("extract_demo_right_box_id", 3);
    extract_demo_pair_sequence_ = parse_box_pair_list(
      get_or_declare_parameter<std::string>(
        "extract_demo_pair_sequence", "1,3;4,6;7,9;10,12;13,15"));
    if (extract_demo_pair_sequence_.empty()) {
      extract_demo_pair_sequence_ = {{1, 3}, {4, 6}, {7, 9}, {10, 12}, {13, 15}};
    }
    extract_demo_all_rows_ = get_or_declare_parameter<bool>("extract_demo_all_rows", false);
    extract_monitor_top_suction_ = get_or_declare_parameter<bool>("extract_monitor_top_suction", false);
    extract_monitor_left_top_suction_ =
      get_or_declare_parameter<bool>("extract_monitor_left_top_suction", extract_monitor_top_suction_);
    extract_monitor_right_top_suction_ =
      get_or_declare_parameter<bool>("extract_monitor_right_top_suction", extract_monitor_top_suction_);
    extract_step_x_ = get_or_declare_parameter<double>("extract_step_x", 0.03);
    extract_max_x_ = get_or_declare_parameter<double>("extract_max_x", 0.36);
    extract_lift_candidates_ = get_or_declare_parameter<std::vector<double>>("extract_lift_candidates", std::vector<double>{0.0, 0.02, 0.05, 0.08});
    extract_pitch_candidates_deg_ = get_or_declare_parameter<std::vector<double>>("extract_pitch_candidates_deg", std::vector<double>{0.0, 5.0, 10.0, 15.0});
    extract_neighbor_margin_ = get_or_declare_parameter<double>("extract_neighbor_margin", 0.02);
    extract_fail_fast_ = get_or_declare_parameter<bool>("extract_fail_fast", false);
    extract_success_extra_steps_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_success_extra_steps", 3)));
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
    extract_max_joint_delta_ = get_or_declare_parameter<double>("extract_max_joint_delta", 10.0 * M_PI / 180.0);
    extract_demo_direct_grasp_start_ = get_or_declare_parameter<bool>("extract_demo_direct_grasp_start", false);
    extract_grasp_ik_home_updown_ = get_or_declare_parameter<double>("extract_grasp_ik_home_updown", 0.3);
    extract_monitor_turn_ = get_or_declare_parameter<double>("extract_monitor_turn", 0.0);
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
    extract_benchmark_extract_success_quorum_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_benchmark_extract_success_quorum", 3)));
    extract_benchmark_extract_quality_success_quorum_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_benchmark_extract_quality_success_quorum", 0)));
    extract_benchmark_extract_quality_loaded_distance_sum_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_benchmark_extract_quality_loaded_distance_sum", 0.0));
    extract_ik_dedup_enabled_ = get_or_declare_parameter<bool>("extract_ik_dedup_enabled", true);
    extract_ik_dedup_joint_threshold_ =
      get_or_declare_parameter<double>("extract_ik_dedup_joint_threshold_deg", 1.0) * M_PI / 180.0;
    extract_ik_dedup_h_threshold_ = get_or_declare_parameter<double>("extract_ik_dedup_h_threshold", 0.005);
    extract_ik_stratified_limit_enabled_ =
      get_or_declare_parameter<bool>("extract_ik_stratified_limit_enabled", false);
    extract_ik_stratified_h_bucket_ =
      std::max(1e-4, get_or_declare_parameter<double>("extract_ik_stratified_h_bucket", 0.05));
    extract_ik_stratified_top_score_count_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_ik_stratified_top_score_count", 12)));
    extract_ik_candidate_reserve_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_ik_candidate_reserve_limit", 64)));
    extract_ik_candidate_reserve_stratified_ =
      get_or_declare_parameter<bool>("extract_ik_candidate_reserve_stratified", true);
    extract_ik_candidate_reserve_interleave_stride_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_ik_candidate_reserve_interleave_stride", 4)));
    extract_ik_loaded_distance_order_weight_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_ik_loaded_distance_order_weight", 0.0));
    extract_monitor_capture_raw_ik_ = get_or_declare_parameter<bool>("extract_monitor_capture_raw_ik", false);
    extract_monitor_build_final_replay_ =
      get_or_declare_parameter<bool>("extract_monitor_build_final_replay", true);
    extract_monitor_place_cycle_enabled_ =
      get_or_declare_parameter<bool>("extract_monitor_place_cycle_enabled", false);
    extract_monitor_place_updown_ =
      get_or_declare_parameter<double>("extract_monitor_place_updown", 0.10);
    extract_monitor_place_transition_updown_ =
      get_or_declare_parameter<double>("extract_monitor_place_transition_updown", 0.10);
    auto left_place_family = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "extract_monitor_place_left_pose_deg",
      "[0.0,-55.0,-50.0,-60.0,0.0,0.0]"));
    auto right_place_family = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "extract_monitor_place_right_pose_deg",
      "[0.0,-55.0,-50.0,-60.0,0.0,0.0]"));
    if (left_place_family.size() != 1 || right_place_family.size() != 1 ||
        left_place_family.front().size() != 6 || right_place_family.front().size() != 6) {
      throw std::runtime_error(
        "extract monitor place pose must contain exactly one six-axis pose per arm");
    }
    extract_monitor_place_left_arm_ = std::move(left_place_family.front());
    extract_monitor_place_right_arm_ = std::move(right_place_family.front());
    extract_rollout_mode_ = get_or_declare_parameter<std::string>("extract_rollout_mode", "greedy");
    extract_top_updown_lift_distance_ = std::max(
      0.0, get_or_declare_parameter<double>("extract_top_updown_lift_distance", 0.40));
    extract_rrt_rollout_enabled_ = get_or_declare_parameter<bool>("extract_rrt_rollout_enabled", false);
    extract_box_pose_rrt_max_iterations_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_box_pose_rrt_max_iterations", 160)));
    extract_box_pose_rrt_paths_per_arm_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_box_pose_rrt_paths_per_arm", 8)));
    extract_box_pose_rrt_path_pair_limit_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_box_pose_rrt_path_pair_limit", 64)));
    extract_box_pose_rrt_max_retreat_ =
      get_or_declare_parameter<double>("extract_box_pose_rrt_max_retreat", 0.55);
    extract_box_pose_rrt_max_lift_ =
      get_or_declare_parameter<double>("extract_box_pose_rrt_max_lift", 0.55);
    extract_box_pose_rrt_separation_margin_ =
      get_or_declare_parameter<double>("extract_box_pose_rrt_separation_margin", 0.03);
    extract_box_pose_rrt_analytic_root_samples_ = static_cast<size_t>(
      std::max(8, get_or_declare_parameter<int>("extract_box_pose_rrt_analytic_root_samples", 12)));
    extract_box_pose_rrt_diagnostics_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_diagnostics", false);
    extract_box_pose_rrt_edge_scene_collision_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_edge_scene_collision", true);
    extract_box_pose_rrt_parent_candidates_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_box_pose_rrt_parent_candidates", 8)));
    extract_box_pose_rrt_parent_diverse_candidates_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_box_pose_rrt_parent_diverse_candidates", 0)));
    extract_box_pose_rrt_parent_endpoint_score_weight_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_parent_endpoint_score_weight", 0.05));
    extract_box_pose_rrt_parent_node_score_weight_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_parent_node_score_weight", 0.0));
    extract_box_pose_rrt_parent_density_weight_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_parent_density_weight", 0.0));
    extract_box_pose_rrt_max_lateral_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_max_lateral", 0.0));
    extract_box_pose_rrt_step_lateral_ =
      std::max(1e-4, get_or_declare_parameter<double>("extract_box_pose_rrt_step_lateral", 0.02));
    extract_box_pose_rrt_front_free_motion_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_front_free_motion", true);
    extract_box_pose_rrt_front_goal_requires_max_pitch_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_front_goal_requires_max_pitch", false);
    extract_box_pose_rrt_best_first_fallback_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_best_first_fallback", true);
    extract_box_pose_rrt_best_first_first_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_best_first_first", false);
    extract_box_pose_rrt_top_best_first_first_ =
      get_or_declare_parameter<bool>("extract_box_pose_rrt_top_best_first_first", false);
    extract_box_pose_rrt_top_goal_min_pitch_deg_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_top_goal_min_pitch_deg", 5.0));
    extract_box_pose_rrt_best_first_max_expansions_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_box_pose_rrt_best_first_max_expansions", 800)));
    extract_box_pose_rrt_best_first_heuristic_weight_ =
      std::max(0.0, get_or_declare_parameter<double>("extract_box_pose_rrt_best_first_heuristic_weight", 1.0));
    extract_rrt_planning_group_ = get_or_declare_parameter<std::string>("extract_rrt_planning_group", "dual_arm");
    extract_rrt_planning_time_ = get_or_declare_parameter<double>("extract_rrt_planning_time", 0.35);
    extract_rrt_planning_attempts_ = std::max(1, get_or_declare_parameter<int>("extract_rrt_planning_attempts", 1));
    extract_rrt_endpoint_per_arm_limit_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_rrt_endpoint_per_arm_limit", 8)));
    extract_rrt_goal_limit_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_rrt_goal_limit", 8)));
    extract_benchmark_plan_loaded_after_success_ =
      get_or_declare_parameter<bool>("extract_benchmark_plan_loaded_after_success", false);
    extract_loaded_planning_group_ =
      get_or_declare_parameter<std::string>("extract_loaded_planning_group", "dual_arm_with_base");
    extract_loaded_planner_id_ =
      get_or_declare_parameter<std::string>("extract_loaded_planner_id", "");
    extract_loaded_planning_time_ = get_or_declare_parameter<double>("extract_loaded_planning_time", 1.0);
    extract_loaded_planning_attempts_ =
      std::max(1, get_or_declare_parameter<int>("extract_loaded_planning_attempts", 8));
    extract_loaded_use_direct_pipeline_ =
      get_or_declare_parameter<bool>("extract_loaded_use_direct_pipeline", false);
    extract_loaded_planning_mode_ =
      get_or_declare_parameter<std::string>("extract_loaded_planning_mode", "rrt");
    extract_loaded_parallel_workers_ = static_cast<size_t>(
      std::max(1, get_or_declare_parameter<int>("extract_loaded_parallel_workers", 1)));
    extract_loaded_candidate_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("extract_loaded_candidate_limit", 0)));
    extract_loaded_sort_by_pose_distance_ =
      get_or_declare_parameter<bool>("extract_loaded_sort_by_pose_distance", false);
    extract_loaded_stop_on_first_success_ =
      get_or_declare_parameter<bool>("extract_loaded_stop_on_first_success", true);
    extract_loaded_target_updown_ = get_or_declare_parameter<double>("extract_loaded_target_updown", 0.3);
    extract_loaded_preserve_lower_updown_ =
      get_or_declare_parameter<bool>("extract_loaded_preserve_lower_updown", false);
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
      get_or_declare_parameter<bool>("enforce_loaded_plan_aabb_clearance", false);
    enforce_loaded_static_box_wall_aabb_clearance_ =
      get_or_declare_parameter<bool>("enforce_loaded_static_box_wall_aabb_clearance", true);
    record_tip_error_ik_candidates_ = get_or_declare_parameter<bool>("record_tip_error_ik_candidates", false);
    record_tip_error_ik_candidate_limit_ = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("record_tip_error_ik_candidate_limit", 80)));

    left_loaded_pose_family_ = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "loaded_left_pose_family_deg",
      "[0.0,0.0,0.0,0.0,0.0,0.0]"));
    right_loaded_pose_family_ = parse_pose_family_degrees(get_or_declare_parameter<std::string>(
      "loaded_right_pose_family_deg",
      "[0.0,0.0,0.0,0.0,0.0,0.0]"));
    if (left_loaded_pose_family_.empty()) {
      left_loaded_pose_family_.push_back(deg_to_rad({0, 0, 0, 0, 0, 0}));
    }
    if (right_loaded_pose_family_.empty()) {
      right_loaded_pose_family_.push_back(deg_to_rad({0, 0, 0, 0, 0, 0}));
    }
    const size_t loaded_preferred_index = static_cast<size_t>(
      std::max(0, get_or_declare_parameter<int>("loaded_preferred_pose_index", 0)));
    left_preferred_loaded_pose_index_ = std::min(loaded_preferred_index, left_loaded_pose_family_.size() - 1);
    right_preferred_loaded_pose_index_ = std::min(loaded_preferred_index, right_loaded_pose_family_.size() - 1);
    ik_config_.left_loaded_pose_family = left_loaded_pose_family_;
    ik_config_.right_loaded_pose_family = right_loaded_pose_family_;
    ik_config_.left_preferred_loaded_pose_index = left_preferred_loaded_pose_index_;
    ik_config_.right_preferred_loaded_pose_index = right_preferred_loaded_pose_index_;

    left_pregrasp_arm_ = deg_to_rad({0, -45, 120, -75, 0, 0});
    right_pregrasp_arm_ = deg_to_rad({0, -45, 120, -75, 0, 0});
    left_loaded_arm_ = left_loaded_pose_family_[left_preferred_loaded_pose_index_];
    right_loaded_arm_ = right_loaded_pose_family_[right_preferred_loaded_pose_index_];

    loaded_pose_selector_config_.left_pose_family = left_loaded_pose_family_;
    loaded_pose_selector_config_.right_pose_family = right_loaded_pose_family_;
    loaded_pose_selector_config_.left_preferred_index = left_preferred_loaded_pose_index_;
    loaded_pose_selector_config_.right_preferred_index = right_preferred_loaded_pose_index_;
    loaded_pose_selector_config_.target_updown = extract_loaded_target_updown_;
    loaded_pose_selector_config_.preserve_lower_updown = extract_loaded_preserve_lower_updown_;

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
    left_arm_group_ = robot_model_->getJointModelGroup("left_arm");
    right_arm_group_ = robot_model_->getJointModelGroup("right_arm");
    if (!left_arm_group_ || !right_arm_group_) {
      throw std::runtime_error("Missing single-arm JointModelGroup left_arm/right_arm");
    }

    loaded_pose_selector_config_.enforce_bounds_group = joint_group_;
    loaded_pose_selector_ = std::make_unique<LoadedPoseSelector>(loaded_pose_selector_config_);

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
      declare_ompl_planner_config_parameters();
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
        "Loaded pose direct planning pipeline ready: plugin=%s planner_id=%s workers=%zu attempts=%d time=%.3fs",
        loaded_planning_pipeline_->getPlannerPluginName().c_str(),
        extract_loaded_planner_id_.empty() ? "(default)" : extract_loaded_planner_id_.c_str(),
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
    auto box_pose_solver_config = extract_candidate_solver_config();
    box_pose_solver_config.analytic_root_samples = extract_box_pose_rrt_analytic_root_samples_;
    box_pose_solver_config.profile = box_pose_solver_profile_;
    box_pose_rrt_candidate_solver_ = std::make_unique<ExtractCandidateSolver>(box_pose_solver_config);
    if (!box_pose_rrt_candidate_solver_->initialize(&extract_solver_error)) {
      throw std::runtime_error("box-pose RRT analytic solver init failed: " + extract_solver_error);
    }
    auto pre_contact_solver_config = extract_candidate_solver_config();
    pre_contact_solver_config.max_joint_delta = 0.0;
    pre_contact_candidate_solver_ = std::make_unique<ExtractCandidateSolver>(pre_contact_solver_config);
    if (!pre_contact_candidate_solver_->initialize(&extract_solver_error)) {
      throw std::runtime_error("pre-contact analytic solver init failed: " + extract_solver_error);
    }
    loaded_pose_planner_ = std::make_unique<LoadedPosePlanner>(loaded_pose_planner_config());
    extract_rollout_planner_ = std::make_unique<ExtractRolloutPlanner>(extract_rollout_planner_config());
    box_pose_rrt_extract_planner_ =
      std::make_unique<BoxPoseRrtExtractPlanner>(box_pose_rrt_extract_planner_config());
    ik_candidate_selector_ = std::make_unique<IkCandidateSelector>(ik_candidate_selector_config());
    apply_container_obstacles();
    set_static_box_wall_opening(extract_demo_left_box_id_, extract_demo_right_box_id_, "initial");

    if (vehicle_pose_tracking_active()) {
      vehicle_drift_check_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0),
        [this]() { check_static_obstacles_drift(); });
      RCLCPP_INFO(
        get_logger(),
        "车体位姿跟踪已启动(1Hz)：track_drift=%s dynamic_pose=%s（动态位姿模式会自动"
        "启用漂移刷新，车体挪动超阈值时重算集装箱几何）。",
        container_track_vehicle_drift_ ? "true" : "false",
        container_pose_dynamic_ ? "true" : "false");
    }

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

    extract_monitor_config_srv_ = create_service<alfa_robot_moveit_config::srv::ConfigureExtractMonitor>(
      "~/configure_extract_monitor",
      [this](
        const std::shared_ptr<alfa_robot_moveit_config::srv::ConfigureExtractMonitor::Request> request,
        std::shared_ptr<alfa_robot_moveit_config::srv::ConfigureExtractMonitor::Response> response) {
        std::string message;
        const bool ok = configure_extract_monitor_task(*request, &message);
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
                "  IK strategy=analytic_three_parallel_fixed_h h=%zu root_samples=%zu collision=%s",
                ik_config_.h_candidate_count, ik_analytic_root_samples_,
                ik_config_.check_collision ? "true" : "false");
    RCLCPP_INFO(get_logger(),
                "  loaded pose prior left=%zu right=%zu preferred=(%zu,%zu) weights=(family %.3f, preferred %.3f)",
                left_loaded_pose_family_.size(), right_loaded_pose_family_.size(),
                left_preferred_loaded_pose_index_, right_preferred_loaded_pose_index_,
                ik_config_.cost_loaded_family_distance, ik_config_.cost_loaded_preferred_distance);
    RCLCPP_INFO(get_logger(),
                "  IK cost updown=%s joint_limit=(weight %.3f free_ratio %.2f weights=[%.2f %.2f %.2f %.2f %.2f %.2f])",
                ik_config_.cost_updown_enabled ? "enabled" : "disabled",
                ik_config_.cost_joint_limit_margin,
                ik_config_.joint_limit_free_ratio,
                ik_config_.joint_limit_weights.size() > 0 ? ik_config_.joint_limit_weights[0] : 1.0,
                ik_config_.joint_limit_weights.size() > 1 ? ik_config_.joint_limit_weights[1] : 1.0,
                ik_config_.joint_limit_weights.size() > 2 ? ik_config_.joint_limit_weights[2] : 1.0,
                ik_config_.joint_limit_weights.size() > 3 ? ik_config_.joint_limit_weights[3] : 1.0,
                ik_config_.joint_limit_weights.size() > 4 ? ik_config_.joint_limit_weights[4] : 1.0,
                ik_config_.joint_limit_weights.size() > 5 ? ik_config_.joint_limit_weights[5] : 1.0);
    RCLCPP_INFO(get_logger(),
                "  Extract primitive IK=analytic_three_parallel fixed-updown root_samples=%zu pos_tol=%.3fm ori_tol=%.3frad",
                ik_analytic_root_samples_, extract_position_tolerance_, extract_orientation_tolerance_);
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

    auto scene = extract_collision_scene_for_thread();
    if (!scene) {
      if (reason) *reason = "extract_collision_scene_unavailable";
      return false;
    }
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = true;
    request.max_contacts = 10;
    const auto collision_started = std::chrono::steady_clock::now();
    scene->checkCollision(request, result, collision_state);
    const auto collision_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - collision_started).count());
    extract_collision_check_count_.fetch_add(1, std::memory_order_relaxed);
    extract_collision_check_total_ns_.fetch_add(collision_ns, std::memory_order_relaxed);
    auto previous_max = extract_collision_check_max_ns_.load(std::memory_order_relaxed);
    while (collision_ns > previous_max &&
           !extract_collision_check_max_ns_.compare_exchange_weak(
             previous_max, collision_ns, std::memory_order_relaxed)) {}
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

  planning_scene::PlanningScenePtr make_extract_collision_scene_snapshot() const
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
    if (scene_adapter_) {
      scene_adapter_->applyToPlanningSceneSnapshot(*scene_snapshot, {});
    }
    return scene_snapshot;
  }

  planning_scene::PlanningScenePtr extract_collision_scene_for_thread() const
  {
    struct ThreadLocalScene
    {
      const void* owner = nullptr;
      uint64_t epoch = 0;
      planning_scene::PlanningScenePtr scene;
    };
    thread_local ThreadLocalScene cache;
    const uint64_t epoch = extract_collision_scene_epoch_.load(std::memory_order_acquire);
    if (cache.owner != this || cache.epoch != epoch || !cache.scene) {
      cache.owner = this;
      cache.epoch = epoch;
      cache.scene = make_extract_collision_scene_snapshot();
    }
    return cache.scene;
  }

  // container_pose_dynamic_ 为真时，用车体最近一次查询到的位姿刷新集装箱相对车体的
  // 位姿缓存。查询 TF 有副作用（节流日志的时间戳），因此这个函数不是 const；真正的碰撞
  // 几何计算 (container_geometry_config()) 只读这份缓存，保持 const，不牵连调用它的一大
  // 批 const 碰撞检测热路径函数（carried_box_clear_scene_obstacles 等）。
  void refresh_dynamic_container_geometry()
  {
    if (!container_pose_dynamic_) return;
    bool tf_ok = false;
    const Eigen::Isometry3d vehicle_pose = lookup_vehicle_pose(&tf_ok);
    if (!tf_ok) {
      // TF 查询失败时绝不用 identity 兜底去覆盖已有缓存——那会把集装箱悄悄挪回"车体在
      // 原点"的错误假设位置。保留上一次成功算出的相对位姿；若从未成功过，则维持空缓存，
      // container_geometry_config() 会退回静态 container_center_x/y（明确的保守回退），
      // 而不是一个基于错误车体位姿算出的几何。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "container_pose_dynamic 已启用，但 %s -> %s 车体位姿 TF 不可用；"
        "%s，不用 identity 兜底覆盖，避免集装箱几何跳到错误位置。",
        vehicle_drift_global_frame_.c_str(), container_frame_.c_str(),
        container_pose_dynamic_cache_.has_value() ? "沿用上一次成功的相对位姿缓存"
                                                  : "尚无成功缓存，退回静态 container_center_x/y");
      return;
    }
    container_pose_dynamic_cache_ = compute_container_pose_relative_to_vehicle(
      vehicle_pose, container_pose_map_x_, container_pose_map_y_, container_pose_map_yaw_);
  }

  ContainerGeometryConfig container_geometry_config() const
  {
    ContainerGeometryConfig config;
    config.width = container_width_;
    config.height = container_height_;
    config.length = container_length_;
    config.wall_thickness = container_wall_thickness_;
    config.floor_z = container_floor_z_;
    if (container_pose_dynamic_ && container_pose_dynamic_cache_.has_value()) {
      config.center_x = container_pose_dynamic_cache_->x;
      config.center_y = container_pose_dynamic_cache_->y + scene_y_shift_;
      config.yaw = container_pose_dynamic_cache_->yaw;
    } else {
      config.center_x = container_center_x_;
      config.center_y = container_center_y_ + scene_y_shift_;
      config.yaw = 0.0;
    }
    return config;
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
      carried_box_grasp_lateral_offset_,
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
      extract_position_tolerance_,
      extract_orientation_tolerance_,
      extract_max_tip_z_drop_,
      extract_min_tool_normal_z_,
      true,
      extract_monitor_left_top_suction_ && extract_monitor_right_top_suction_,
      ik_config_.top_suction_orientation_tolerance,
      extract_max_joint_delta_,
      ik_analytic_root_samples_,
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
    config.top_suction = extract_monitor_left_top_suction_ && extract_monitor_right_top_suction_;
    config.left_top_suction = extract_monitor_left_top_suction_;
    config.right_top_suction = extract_monitor_right_top_suction_;
    config.top_suction_updown_step = extract_step_x_;
    config.top_suction_max_lift = extract_max_x_;
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

  BoxPoseRrtExtractPlannerConfig box_pose_rrt_extract_planner_config()
  {
    BoxPoseRrtExtractPlannerConfig config;
    config.logger = get_logger();
    config.candidate_solver = box_pose_rrt_candidate_solver_.get();
    config.candidate_scorer = extract_candidate_scorer_.get();
    config.joint_group = joint_group_;
    config.left_arm_group = left_arm_group_;
    config.right_arm_group = right_arm_group_;
    config.left_tip = left_tip_;
    config.right_tip = right_tip_;
    config.max_paths_per_arm = extract_box_pose_rrt_paths_per_arm_;
    config.max_path_pairs_to_validate = extract_box_pose_rrt_path_pair_limit_;
    config.diagnose_isolated_arm_paths = extract_box_pose_rrt_diagnostics_;
    config.front_rrt.mode = robot_motion::core::BoxPoseExtractMode::FrontPivot;
    config.front_rrt.box_depth = carried_box_depth_;
    config.front_rrt.box_height = carried_box_height_;
    config.front_rrt.separation_margin = extract_box_pose_rrt_separation_margin_;
    config.front_rrt.max_retreat = extract_box_pose_rrt_max_retreat_;
    config.front_rrt.max_lift = extract_box_pose_rrt_max_lift_;
    config.front_rrt.max_pitch = M_PI_2;
    config.front_rrt.max_lateral = extract_box_pose_rrt_max_lateral_;
    config.front_rrt.step_lateral = extract_box_pose_rrt_step_lateral_;
    config.front_rrt.edge_resolution_lateral = extract_box_pose_rrt_step_lateral_;
    config.front_rrt.front_free_motion = extract_box_pose_rrt_front_free_motion_;
    config.front_rrt.front_goal_requires_max_pitch = extract_box_pose_rrt_front_goal_requires_max_pitch_;
    config.front_rrt.endpoint_only_edges = true;
    config.front_rrt.max_iterations = extract_box_pose_rrt_max_iterations_;
    config.front_rrt.max_solution_count = extract_box_pose_rrt_paths_per_arm_;
    config.front_rrt.parent_candidate_count = extract_box_pose_rrt_parent_candidates_;
    config.front_rrt.parent_diverse_candidate_count = extract_box_pose_rrt_parent_diverse_candidates_;
    config.front_rrt.parent_endpoint_score_weight = extract_box_pose_rrt_parent_endpoint_score_weight_;
    config.front_rrt.parent_node_score_weight = extract_box_pose_rrt_parent_node_score_weight_;
    config.front_rrt.parent_density_weight = extract_box_pose_rrt_parent_density_weight_;
    config.front_rrt.best_first_fallback = extract_box_pose_rrt_best_first_fallback_;
    config.front_rrt.best_first_first = extract_box_pose_rrt_best_first_first_;
    config.front_rrt.best_first_max_expansions = extract_box_pose_rrt_best_first_max_expansions_;
    config.front_rrt.best_first_heuristic_weight = extract_box_pose_rrt_best_first_heuristic_weight_;
    config.front_rrt.random_seed = 17;
    config.top_rrt = config.front_rrt;
    config.top_rrt.mode = robot_motion::core::BoxPoseExtractMode::TopTranslate;
    config.top_rrt.best_first_first = extract_box_pose_rrt_top_best_first_first_;
    config.top_rrt.max_pitch = M_PI_2;
    config.top_rrt.endpoint_only_edges = true;
    config.top_rrt.max_lift = extract_box_pose_rrt_max_lift_;
    config.top_rrt.min_top_retreat = extract_box_pose_rrt_separation_margin_;
    config.top_rrt.min_top_lift = extract_box_pose_rrt_separation_margin_;
    config.top_rrt.top_goal_min_pitch = extract_box_pose_rrt_top_goal_min_pitch_deg_ * M_PI / 180.0;
    config.top_rrt.top_goal_requires_retreat = false;
    config.top_rrt.top_goal_requires_max_pitch = false;
    config.top_rrt.retreat_distance_weight = 2.0;
    config.top_rrt.lift_distance_weight = 0.5;
    config.top_rrt.random_seed = 29;
    if (extract_box_pose_rrt_edge_scene_collision_) {
      config.single_clear_callback =
        [this](
          const moveit::core::RobotState& state,
          const AttachedBoxSpec& carried_box,
          int box_id,
          bool* detached,
          std::string* reason) {
          return state_clear_for_single_extract(state, carried_box, box_id, detached, reason);
        };
    }
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
    config.profile = box_pose_rrt_profile_;
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
      &ik_config_,
      robot_model_.get(),
      joint_group_,
      fixed_updown_,
      ik_analytic_root_samples_,
    };
  }

  bool ensure_optimized_ik_solver()
  {
    if (optimized_dual_ik_solver_ && optimized_dual_ik_solver_->ready()) {
      return true;
    }
    if (!optimized_dual_ik_solver_) {
      RCLCPP_INFO(
        get_logger(),
        "Initializing analytic optimized IK solver on first use: h=%zu root_samples=%zu",
        ik_config_.h_candidate_count,
        ik_analytic_root_samples_);
      optimized_dual_ik_solver_ = std::make_unique<OptimizedDualIkSolver>(optimized_dual_ik_solver_config());
    }
    return optimized_dual_ik_solver_->ready();
  }

  MotionSceneAdapterConfig motion_scene_adapter_config()
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
    if (vehicle_pose_tracking_active()) {
      config.vehicle_pose_provider = [this]() { return lookup_vehicle_pose(); };
    }
    return config;
  }

  // 查询 vehicle_drift_global_frame_ -> container_frame_（通常是 map -> world）的当前
  // 变换。碰撞几何本身始终以 container_frame_（world）写入 planning scene 不变——world
  // 跟车体刚性绑定，车体怎么动它天然跟着动，MoveIt 的规划假设不受影响。这个查询只用来
  // 判断"车体在全局系下挪动了多少"，供 MotionSceneAdapter::staticObstaclesStale 检测
  // container_center_x/y 这类相对偏移参数是否已经对不上车体新停靠的位置。
  // 查询失败（例如车体位姿源节点未启动）时退化为 identity，不会导致规划节点崩溃。
  // 车体位姿跟踪机制（1Hz TF 查询定时器 + scene adapter 的 vehicle_pose_provider +
  // 漂移检测）在两种情况下都需要启动：
  //   1. container_track_vehicle_drift_：显式的"车体漂移告警"开关（历史行为）；
  //   2. container_pose_dynamic_：动态集装箱位姿建模——车体一动，集装箱相对车体的几何
  //      就必须重新计算并刷新，否则集装箱几何会停在构造时算出的那一帧再也不更新。
  // 早期版本只 gate 在 (1) 上，导致只开 (2) 时刷新机制完全不启动、集装箱几何静默冻结。
  bool vehicle_pose_tracking_active() const
  {
    return container_track_vehicle_drift_ || container_pose_dynamic_;
  }

  Eigen::Isometry3d lookup_vehicle_pose(bool* tf_ok = nullptr)
  {
    if (!tf_buffer_) {
      if (tf_ok) *tf_ok = false;
      return Eigen::Isometry3d::Identity();
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        vehicle_drift_global_frame_, container_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      if (tf_ok) *tf_ok = true;
      return tf2::transformToEigen(transform);
    } catch (const tf2::TransformException& ex) {
      if (tf_ok) *tf_ok = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "lookup_vehicle_pose: %s -> %s unavailable (%s), falling back to identity",
        vehicle_drift_global_frame_.c_str(), container_frame_.c_str(), ex.what());
      return Eigen::Isometry3d::Identity();
    }
  }

  void check_static_obstacles_drift()
  {
    if (!vehicle_pose_tracking_active() || !scene_adapter_) return;
    if (!scene_adapter_->staticObstaclesStale(
          vehicle_drift_translation_threshold_m_, vehicle_drift_rotation_threshold_rad_)) {
      return;
    }
    if (container_pose_dynamic_) {
      RCLCPP_WARN(
        get_logger(),
        "静态碰撞几何(container)已过期：车体在 %s 下的位姿相对上次写入时已超出阈值"
        "(平移>%.3fm 或 旋转>%.3frad)，正在按 container_pose_map_x/y/yaw 重新计算"
        "集装箱相对车体的位姿并自动刷新碰撞场景。",
        vehicle_drift_global_frame_.c_str(), vehicle_drift_translation_threshold_m_,
        vehicle_drift_rotation_threshold_rad_);
      apply_container_obstacles();
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "静态碰撞几何(container/box wall)可能已过期：车体在 %s 下的位姿相对上次写入时"
      "已超出阈值(平移>%.3fm 或 旋转>%.3frad)，container_center_x/y 等相对偏移参数"
      "描述的可能已经不是车体当前停靠位置。请重新调用 apply_container_obstacles 之类"
      "的写入接口刷新碰撞场景。",
      vehicle_drift_global_frame_.c_str(), vehicle_drift_translation_threshold_m_,
      vehicle_drift_rotation_threshold_rad_);
  }

  LoadedPosePlannerConfig loaded_pose_planner_config()
  {
    LoadedPosePlannerConfig config;
    config.move_group = loaded_move_group_.get();
    config.selector = loaded_pose_selector_.get();
    config.scene_adapter = scene_adapter_.get();
    config.target_joint_names = dual_arm_with_updown_joint_names();
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
    config.planning_mode = extract_loaded_planning_mode_;
    config.min_tool_normal_z = extract_min_tool_normal_z_;
    config.max_joint_delta = extract_max_joint_delta_;
    config.lateral_shift_solver = extract_candidate_solver_.get();
    if (extract_loaded_use_direct_pipeline_) {
      config.direct_plan_callback = [this](
        const std::string& stage_name,
        const moveit::core::RobotState& start_state,
        const moveit::core::RobotState& goal_state,
        const std::vector<AttachedBoxSpec>& carried_boxes,
        moveit::planning_interface::MoveGroupInterface::Plan* plan,
        std::string* reason) {
        return plan_loaded_pose_with_direct_pipeline(stage_name, start_state, goal_state, carried_boxes, plan, reason);
      };
    }
    config.clearance_callback = [this](
      const moveit::planning_interface::MoveGroupInterface::Plan& plan,
      const moveit::core::RobotState& start_state,
      const std::vector<AttachedBoxSpec>& carried_boxes,
      std::string* reason) {
      return planned_trajectory_clear_in_full_scene(plan, start_state, carried_boxes, reason);
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
    config.top_suction_height_mismatch_callback = [this]() {
        return extract_monitor_top_height_mismatch_;
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

  void declare_ompl_planner_config_parameters()
  {
    auto declare_string = [this](const std::string& name, const std::string& value) {
      if (!has_parameter(name)) {
        declare_parameter<std::string>(name, value);
      }
    };
    auto declare_double = [this](const std::string& name, double value) {
      if (!has_parameter(name)) {
        declare_parameter<double>(name, value);
      }
    };
    auto declare_bool = [this](const std::string& name, bool value) {
      if (!has_parameter(name)) {
        declare_parameter<bool>(name, value);
      }
    };
    auto declare_string_array = [this](const std::string& name, const std::vector<std::string>& value) {
      if (!has_parameter(name)) {
        declare_parameter<std::vector<std::string>>(name, value);
      }
    };

    declare_string("ompl.planner_configs.RRTConnectkConfigDefault.type", "geometric::RRTConnect");
    declare_double("ompl.planner_configs.RRTConnectkConfigDefault.range", 0.0);
    declare_string("ompl.planner_configs.RRTstarkConfigDefault.type", "geometric::RRTstar");
    declare_double("ompl.planner_configs.RRTstarkConfigDefault.range", 0.0);
    declare_double("ompl.planner_configs.RRTstarkConfigDefault.goal_bias", 0.05);
    declare_bool("ompl.planner_configs.RRTstarkConfigDefault.delay_collision_checking", true);

    declare_string("ompl.dual_arm.default_planner_config", "RRTConnectkConfigDefault");
    declare_string_array(
      "ompl.dual_arm.planner_configs",
      std::vector<std::string>{"RRTConnectkConfigDefault", "RRTstarkConfigDefault"});
    declare_string("ompl.dual_arm_with_base.default_planner_config", "RRTConnectkConfigDefault");
    declare_string_array(
      "ompl.dual_arm_with_base.planner_configs",
      std::vector<std::string>{"RRTConnectkConfigDefault", "RRTstarkConfigDefault"});
  }

  bool plan_loaded_pose_with_direct_pipeline(
    const std::string& stage_name,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state,
    const std::vector<AttachedBoxSpec>& carried_boxes,
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

    moveit::core::RobotState planning_start_state(start_state);
    moveit::core::RobotState planning_goal_state(goal_state);
    if (enable_attached_box_collision_) {
      attach_boxes_to_robot_state(
        planning_start_state, carried_boxes, attached_box_collision_padding_);
      attach_boxes_to_robot_state(
        planning_goal_state, carried_boxes, attached_box_collision_padding_);
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
    scene_snapshot->setCurrentState(planning_start_state);
    if (scene_adapter_) {
      scene_adapter_->applyToPlanningSceneSnapshot(*scene_snapshot, carried_boxes);
      scene_snapshot->setCurrentState(planning_start_state);
    }

    planning_interface::MotionPlanRequest request;
    request.group_name = extract_loaded_planning_group_;
    if (!extract_loaded_planner_id_.empty()) {
      request.planner_id = extract_loaded_planner_id_;
    }
    request.allowed_planning_time = extract_loaded_planning_time_;
    request.num_planning_attempts = extract_loaded_planning_attempts_;
    request.max_velocity_scaling_factor = velocity_scale_;
    request.max_acceleration_scaling_factor = acceleration_scale_;
    moveit::core::robotStateToRobotStateMsg(planning_start_state, request.start_state, true);
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(
        planning_goal_state, loaded_group, joint_goal_tolerance_rad_));
    planning_interface::MotionPlanResponse response;
    const bool generated = loaded_planning_pipeline_->generatePlan(scene_snapshot, request, response);
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        !response.trajectory_) {
      if (reason) {
        *reason = "direct_pipeline_planning_failed_code_" +
          std::to_string(response.error_code_.val) + " " +
          direct_pipeline_failure_diagnostic(
            scene_snapshot, planning_start_state, planning_goal_state, loaded_group);
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
      const auto left_pose = make_outer_box_grasp_pose(left_box, top_suction, true);
      const auto right_pose = make_outer_box_grasp_pose(right_box, top_suction, false);
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
      const robot_motion::core::UpdownAwareIkCandidate& candidate) {
      return robot_state_from_ik_candidate(seed_state, candidate, joint_group_);
    };
    callbacks.rollout_left = [this](
      const moveit::core::RobotState& start_state,
      const AttachedBoxSpec& left_box,
      int left_box_id,
      size_t candidate_order,
      const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
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
      const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
      const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step) {
      return rollout_dual_extract_from_state(
        start_state, left_box, left_box_id, right_box, right_box_id,
        candidate_order, ik_candidate, record_step);
    };
    callbacks.fill_loaded_metrics = [this](ExtractRolloutTiming& timing) {
      if (loaded_pose_selector_) {
        loaded_pose_selector_->fillTimingDistanceMetrics(timing);
      }
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
      const robot_motion::core::UpdownAwareIkResult& ik_result,
      const AttachedBoxSpec& left_box) {
      record_tip_error_ik_candidates(prefix, seed_state, ik_result, left_box);
    };
    callbacks.record_summary = [this](const nlohmann::json& summary) {
      if (recording_enabled()) {
        recorder_->write(summary);
      }
    };
    callbacks.rejection_counts_json = [this](const robot_motion::core::UpdownAwareIkResult& result) {
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

    if (container_pose_dynamic_) {
      refresh_dynamic_container_geometry();
      scene_adapter_->updateContainerGeometry(container_geometry_config());
    }
    if (scene_adapter_->applyContainerObstacles()) {
      extract_collision_scene_epoch_.fetch_add(1, std::memory_order_acq_rel);
      const auto& applied = scene_adapter_->config().container;
      RCLCPP_INFO(get_logger(),
                  "Applied container obstacle: frame=%s length=%.2f width=%.2f height=%.2f panels=%zu "
                  "center_x=%.3f center_y=%.3f yaw_deg=%.2f dynamic=%s",
                  container_frame_.c_str(), container_length_, container_width_, container_height_,
                  container_panels().size(),
                  applied.center_x, applied.center_y, applied.yaw * 180.0 / M_PI,
                  container_pose_dynamic_ ? "true" : "false");
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
    extract_collision_scene_epoch_.fetch_add(1, std::memory_order_acq_rel);
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

  geometry_msgs::msg::Pose make_outer_box_grasp_pose(
    const BoxSpec& box,
    bool top_suction,
    bool left_side) const
  {
    auto pose = top_suction
      ? make_top_suction_pose(
          box, world_to_base_z_, top_suction_x_offset_, top_suction_z_offset_)
      : make_front_grasp_pose(box, world_to_base_z_);
    pose.position.y = scene_y_shift_ +
      (left_side
        ? alfa_robot::motion::kOuterBoxGraspTargetY
        : -alfa_robot::motion::kOuterBoxGraspTargetY);
    return pose;
  }

  bool is_top_suction_box_spec(const AttachedBoxSpec& box) const
  {
    return std::abs(box.size[0] - carried_box_depth_) < 1e-6 &&
           std::abs(box.size[1] - carried_box_width_) < 1e-6 &&
           std::abs(box.size[2] - carried_box_height_) < 1e-6 &&
           std::abs(box.center_in_link[2] - carried_box_height_ * 0.5) < 1e-6;
  }

  bool extract_monitor_both_top_suction() const
  {
    return extract_monitor_left_top_suction_ && extract_monitor_right_top_suction_;
  }

  std::string grasp_mode_label(bool top_suction) const
  {
    return top_suction ? "top_suction" : "front";
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

  bool top_suction_box_detached_from_stack(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int box_id,
    std::string* reason) const
  {
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto it = boxes.find(box_id);
    if (it == boxes.end()) {
      if (reason) *reason = "unknown_box_id";
      return false;
    }
    const auto carried = expanded_aabb(attached_box_world_aabb(state, carried_box), extract_neighbor_margin_);
    const double carried_min_z = carried.center[2] - 0.5 * carried.size[2];
    const double source_top_z = it->second.z + 0.5 * carried_box_height_;
    if (carried_min_z >= source_top_z + extract_neighbor_margin_) {
      return true;
    }
    if (reason) {
      std::ostringstream oss;
      oss << carried_box.id << " bottom still below source top z="
          << carried_min_z << " < " << (source_top_z + extract_neighbor_margin_);
      *reason = oss.str();
    }
    return false;
  }

  bool left_carried_box_detached_from_neighbors(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int left_box_id,
    std::string* reason) const
  {
    return carried_box_detached_from_neighbors(state, carried_box, left_box_id, reason);
  }

  bool carried_box_detached_for_extract_mode(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    int box_id,
    std::string* reason) const
  {
    if (!is_top_suction_box_spec(carried_box)) {
      if (!carried_box_detached_from_neighbors(state, carried_box, box_id, reason)) {
        return false;
      }
      size_t clearance_levels = 1;
      if (box_id == extract_demo_left_box_id_) {
        clearance_levels = extract_monitor_left_front_clearance_levels_;
      } else if (box_id == extract_demo_right_box_id_) {
        clearance_levels = extract_monitor_right_front_clearance_levels_;
      }
      if (clearance_levels <= 1) {
        if (reason) reason->clear();
        return true;
      }
      return alfa_robot::motion::carried_box_detached_from_source_layers_xz(
        attached_box_world_aabb(state, carried_box),
        box_id,
        box_front_x_,
        scene_y_shift_,
        carried_box_width_,
        carried_box_height_,
        carried_box_depth_,
        extract_neighbor_margin_,
        clearance_levels,
        carried_box.id,
        reason);
    }
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto it = boxes.find(box_id);
    if (it == boxes.end()) {
      if (reason) *reason = "unknown_box_id_for_top_detachment";
      return false;
    }
    const AxisAlignedBox source_box{{
      it->second.x + 0.5 * carried_box_depth_,
      it->second.y,
      it->second.z,
    }, {
      carried_box_depth_,
      carried_box_width_,
      carried_box_height_,
    }};
    const double detachment_margin = std::max(
      extract_neighbor_margin_, extract_box_pose_rrt_separation_margin_);
    return alfa_robot::motion::carried_box_detached_from_source_xz(
      attached_box_world_aabb(state, carried_box),
      source_box,
      detachment_margin,
      carried_box.id,
      reason);
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
    return carried_box_clear_obstacles(
      carried_aabb,
      carried_box.id,
      enable_static_box_obstacles_ ? static_box_obstacles() : std::vector<StaticBoxObstacle>{},
      enable_container_obstacle_ ? container_panels() : std::vector<ContainerPanel>{},
      reason);
  }

  bool carried_box_clear_rear_guard(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_ || !enable_static_box_obstacles_) return true;
    return carried_box_clear_rear_guards(
      attached_box_world_aabb(state, carried_box),
      carried_box.id,
      static_box_obstacles(),
      reason);
  }

  bool carried_box_clear_static_box_wall_obstacles(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_ || !enable_static_box_obstacles_) return true;
    const auto carried_aabb = attached_box_world_aabb(state, carried_box);
    return carried_box_clear_obstacles(
      carried_aabb,
      carried_box.id,
      static_box_obstacles(),
      {},
      reason);
  }

  bool carried_box_clear_container_obstacles(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& carried_box,
    std::string* reason) const
  {
    if (!enable_attached_box_collision_ || !enable_container_obstacle_) return true;
    const auto carried_aabb = attached_box_world_aabb(state, carried_box);
    return carried_box_clear_obstacles(
      carried_aabb,
      carried_box.id,
      {},
      container_panels(),
      reason);
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
    const bool detached_now = carried_box_detached_for_extract_mode(state, left_carried_box, left_box_id, &detached_reason);
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
    const bool detached_now = carried_box_detached_for_extract_mode(state, carried_box, box_id, &detached_reason);
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
    const bool left_ok = carried_box_detached_for_extract_mode(state, left_box, left_box_id, &left_reason);
    const bool right_ok = carried_box_detached_for_extract_mode(state, right_box, right_box_id, &right_reason);
    if (left_detached) *left_detached = left_ok;
    if (right_detached) *right_detached = right_ok;
    if ((!left_ok || !right_ok) && reason) {
      *reason = !left_ok ? left_reason : right_reason;
    }
    return true;
  }

  bool state_clear_for_dual_grasp_start(
    const moveit::core::RobotState& state,
    const AttachedBoxSpec& left_box,
    const AttachedBoxSpec& right_box,
    std::string* reason) const
  {
    if (!is_state_valid_with_attached_boxes(state, {left_box, right_box}, true, reason)) {
      if (reason && reason->empty()) *reason = "robot/carried box grasp state colliding or out of bounds";
      return false;
    }

    for (const auto& box : {left_box, right_box}) {
      std::string carried_reason;
      if (!carried_box_clear_scene_obstacles(state, box, &carried_reason)) {
        if (reason) *reason = carried_reason;
        return false;
      }
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
    moveit::core::RobotState planning_start_state(start_state);
    if (enable_attached_box_collision_) {
      attach_boxes_to_robot_state(
        planning_start_state, attached_boxes, attached_box_collision_padding_);
    }
    scene_snapshot->setCurrentState(planning_start_state);
    if (scene_adapter_) {
      scene_adapter_->applyToPlanningSceneSnapshot(*scene_snapshot, attached_boxes);
      scene_snapshot->setCurrentState(planning_start_state);
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
    for (const auto& box : attached_boxes) {
      std::string rear_guard_reason;
      if (!carried_box_clear_rear_guard(collision_state, box, &rear_guard_reason)) {
        if (reason) *reason = "rear guard check failed (" + rear_guard_reason + ")";
        return false;
      }
    }
    if (enforce_loaded_static_box_wall_aabb_clearance_) {
      for (const auto& box : attached_boxes) {
        std::string static_wall_reason;
        if (!carried_box_clear_static_box_wall_obstacles(collision_state, box, &static_wall_reason)) {
          if (reason) *reason = "static box-wall AABB check failed (" + static_wall_reason + ")";
          return false;
        }
      }
    }
    for (const auto& box : attached_boxes) {
      std::string container_reason;
      if (!carried_box_clear_container_obstacles(collision_state, box, &container_reason)) {
        if (reason) *reason = "container AABB check failed (" + container_reason + ")";
        return false;
      }
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
    trajectory.joint_names = dual_arm_with_updown_joint_names();

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
    const auto* revolute_joint =
      dynamic_cast<const moveit::core::RevoluteJointModel*>(variable_joint);
    const bool continuous_variable = revolute_joint && revolute_joint->isContinuous();
    return continuous_variable
      ? std::atan2(std::sin(goal - start), std::cos(goal - start))
      : (goal - start);
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
    std::string* reason,
    const std::vector<AttachedBoxSpec>& attached_boxes = {},
    const std::string& group_name = "",
    double planning_time_override = -1.0,
    int planning_attempts_override = -1) const
  {
    if (!plan) {
      if (reason) *reason = "direct_joint_plan_output_null";
      return false;
    }
    if (!loaded_planning_pipeline_) {
      if (reason) *reason = "direct_pipeline_not_initialized";
      return false;
    }
    const std::string effective_group_name = group_name.empty() ? extract_loaded_planning_group_ : group_name;
    const auto* loaded_group = robot_model_->getJointModelGroup(effective_group_name);
    if (!loaded_group) {
      if (reason) *reason = "direct_pipeline_missing_group_" + effective_group_name;
      return false;
    }
    auto scene_snapshot = make_full_scene_snapshot(start_state, attached_boxes);
    if (!scene_snapshot) {
      if (reason) *reason = "direct_pipeline_planning_scene_not_initialized";
      return false;
    }

    moveit::core::RobotState planning_start_state(start_state);
    moveit::core::RobotState planning_goal_state(goal_state);
    if (enable_attached_box_collision_) {
      attach_boxes_to_robot_state(
        planning_start_state, attached_boxes, attached_box_collision_padding_);
      attach_boxes_to_robot_state(
        planning_goal_state, attached_boxes, attached_box_collision_padding_);
    }

    planning_interface::MotionPlanRequest request;
    request.group_name = effective_group_name;
    request.allowed_planning_time = planning_time_override > 0.0
      ? planning_time_override
      : extract_loaded_planning_time_;
    request.num_planning_attempts = planning_attempts_override > 0
      ? planning_attempts_override
      : extract_loaded_planning_attempts_;
    request.max_velocity_scaling_factor = velocity_scale_;
    request.max_acceleration_scaling_factor = acceleration_scale_;
    moveit::core::robotStateToRobotStateMsg(planning_start_state, request.start_state, true);
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(
        planning_goal_state, loaded_group, joint_goal_tolerance_rad_));

    planning_interface::MotionPlanResponse response;
    const bool generated = loaded_planning_pipeline_->generatePlan(scene_snapshot, request, response);
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        !response.trajectory_) {
      if (reason) {
        *reason = "direct_pipeline_planning_failed_code_" +
          std::to_string(response.error_code_.val) + " " +
          direct_pipeline_failure_diagnostic(
            scene_snapshot, planning_start_state, planning_goal_state, loaded_group);
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
      "left_joint1", "left_joint2", "left_joint3",
      "left_joint4", "left_joint5", "left_joint6",
      "right_joint1", "right_joint2", "right_joint3",
      "right_joint4", "right_joint5", "right_joint6",
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
    return plan_dual_tip_ik(stage_name, left_pose, right_pose, top_suction, top_suction);
  }

  bool plan_dual_tip_ik(
    const std::string& stage_name,
    const geometry_msgs::msg::Pose& left_pose,
    const geometry_msgs::msg::Pose& right_pose,
    bool left_top_suction,
    bool right_top_suction)
  {
    auto start_state = get_current_robot_state();
    if (!start_state) {
      return fail(stage_name + ": cannot get start state");
    }
    if (!ensure_optimized_ik_solver()) {
      return fail(stage_name + ": optimized IK solver is not initialized");
    }

    RCLCPP_INFO(get_logger(), "[%s] optimized IK L=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f) current_h=%.3f",
                stage_name.c_str(),
                left_pose.position.x, left_pose.position.y, left_pose.position.z,
                right_pose.position.x, right_pose.position.y, right_pose.position.z, current_updown(*start_state));

    const auto solved = optimized_dual_ik_solver_->solve(
      OptimizedDualIkSolveRequest{
        stage_name,
        left_pose,
        right_pose,
        left_top_suction && right_top_suction,
        start_state.get(),
        left_top_suction,
        right_top_suction},
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
    robot_motion::core::UpdownAwareIkResult* result_out = nullptr,
    bool capture_pre_score_candidates = false)
  {
    return solve_dual_tip_ik_state(
      stage_name,
      left_pose,
      right_pose,
      top_suction,
      top_suction,
      seed_state,
      goal_state,
      extra_out,
      result_out,
      capture_pre_score_candidates);
  }

  bool solve_dual_tip_ik_state(
    const std::string& stage_name,
    const geometry_msgs::msg::Pose& left_pose,
    const geometry_msgs::msg::Pose& right_pose,
    bool left_top_suction,
    bool right_top_suction,
    const moveit::core::RobotState& seed_state,
    moveit::core::RobotState* goal_state,
    nlohmann::json* extra_out,
    robot_motion::core::UpdownAwareIkResult* result_out = nullptr,
    bool capture_pre_score_candidates = false)
  {
    if (!ensure_optimized_ik_solver()) {
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
      OptimizedDualIkSolveRequest{
        stage_name,
        left_pose,
        right_pose,
        left_top_suction && right_top_suction,
        &seed_state,
        left_top_suction,
        right_top_suction,
        capture_pre_score_candidates},
      "optimized_dual_tip_ik_direct_seed");
    const auto& result = solved.ik_result;
    if (result_out) {
      *result_out = result;
    }
    if (!solved.success) {
      return fail(stage_name + ": " + solved.failure_reason);
    }

    *goal_state = *solved.goal_state;

    if (!capture_pre_score_candidates && !is_state_valid(*goal_state, check_goal_collision_)) {
      return fail(stage_name + ": selected IK state out of bounds or colliding");
    }

    RCLCPP_INFO(get_logger(),
                "[%s] direct IK selected h=%.3f score=%.3f limit_cost=%.3f path=%s h_index=%zu seed_index=%zu trials=%zu legal=%zu wall=%.1fms",
                stage_name.c_str(), result.selected.h, result.selected.score,
                result.selected.joint_limit_margin_cost, result.selected.solver_path.c_str(),
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

  BoxSpec extract_source_box_for_mode(
    const BoxSpec& box,
    const moveit::core::RobotState& state,
    const std::string& tip_link,
    bool top_suction) const
  {
    if (!top_suction) {
      return box;
    }
    const Eigen::Isometry3d& tip_tf = state.getGlobalLinkTransform(tip_link);
    return BoxSpec{
      box.id,
      tip_tf.translation().x(),
      tip_tf.translation().y(),
      tip_tf.translation().z(),
    };
  }

  std::vector<ExtractCandidate> extract_rrt_endpoint_candidates_for_side(
    const std::string& side,
    const moveit::core::RobotState& start_state,
    const BoxSpec& source_box,
    const AttachedBoxSpec& carried_box,
    int box_id) const
  {
    std::vector<ExtractCandidate> candidates;
    if (!extract_candidate_solver_ || !extract_candidate_scorer_) {
      return candidates;
    }

    const std::string& tip = side == "left" ? left_tip_ : right_tip_;
    const double step_x = std::max(1e-4, extract_step_x_);
    const double max_x = std::max(step_x, extract_max_x_);
    const size_t retreat_steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(max_x / step_x)));
    const size_t lift_steps = retreat_steps;
    const double min_allowed_tip_z = start_state.getGlobalLinkTransform(tip).translation().z();
    const double fixed_updown = current_updown(start_state);
    const std::vector<double> pitch_degrees{0.0, 1.0, 3.0, 5.0, 10.0, 15.0};

    size_t candidate_index = 0;
    for (size_t retreat_step = 1; retreat_step <= retreat_steps; ++retreat_step) {
      const double retreat_x = std::min(max_x, static_cast<double>(retreat_step) * step_x);
      for (size_t lift_step = 0; lift_step <= lift_steps; ++lift_step) {
        const double lift_z = std::min(max_x, static_cast<double>(lift_step) * step_x);
        for (const double pitch_deg : pitch_degrees) {
          const double pitch_rad = pitch_deg * M_PI / 180.0;
          ExtractCandidate candidate;
          const auto target_pose = make_pose(
            source_box.x - retreat_x,
            source_box.y,
            source_box.z + lift_z,
            pitch_up_orientation(pitch_rad));
          ExtractCandidateSolveRequest request;
          request.side = side;
          request.current_state = &start_state;
          request.target_pose = target_pose;
          request.step_index = retreat_step;
          request.candidate_index = candidate_index++;
          request.retreat_x = retreat_x;
          request.retreat_delta_x = retreat_x;
          request.lift_z = lift_z;
          request.lift_delta_z = lift_z;
          request.pitch_up_rad = pitch_rad;
          request.pitch_delta_rad = pitch_rad;
          request.min_allowed_tip_z = min_allowed_tip_z;
          request.fixed_updown = fixed_updown;
          request.min_tool_normal_z = extract_min_tool_normal_z_;
          if (!extract_candidate_solver_->solve(request, &candidate) || !candidate.state) {
            continue;
          }

          bool detached = false;
          std::string clear_reason;
          if (!state_clear_for_single_extract(*candidate.state, carried_box, box_id, &detached, &clear_reason) ||
              !detached) {
            continue;
          }
          candidate.state_valid = true;
          candidate.carried_clear = true;
          candidate.detached_from_neighbors = true;
          candidates.push_back(std::move(candidate));
        }
      }
    }

    std::sort(candidates.begin(), candidates.end(),
              [&](const ExtractCandidate& lhs, const ExtractCandidate& rhs) {
                return extract_candidate_scorer_->score(side, lhs, start_state, 0.0) <
                       extract_candidate_scorer_->score(side, rhs, start_state, 0.0);
              });
    if (candidates.size() > extract_rrt_endpoint_per_arm_limit_) {
      candidates.resize(extract_rrt_endpoint_per_arm_limit_);
    }
    return candidates;
  }

  ExtractRolloutTiming rollout_dual_extract_rrt_from_state(
    const moveit::core::RobotState& start_state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t candidate_order,
    const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step = {}) const
  {
    struct RrtGoalCandidate
    {
      ExtractCandidate left;
      ExtractCandidate right;
      moveit::core::RobotStatePtr state;
      double score = std::numeric_limits<double>::infinity();
    };

    ExtractRolloutTiming timing;
    timing.candidate_order = candidate_order;
    timing.h_index = ik_candidate.h_index;
    timing.seed_index = ik_candidate.seed_index;
    timing.h = ik_candidate.h;
    timing.ik_score = ik_candidate.score;
    timing.ik_solve_ms = ik_candidate.solve_ms;

    const auto t0 = std::chrono::steady_clock::now();
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    const auto left_it = boxes.find(left_box_id);
    const auto right_it = boxes.find(right_box_id);
    if (left_it == boxes.end() || right_it == boxes.end()) {
      timing.failure_reason = "extract_rrt_unknown_box_id";
      return timing;
    }

    if (record_step) {
      nlohmann::json extra = {
        {"stage_kind", "dual_extract_rrt_start"},
        {"candidate_order", candidate_order},
        {"h_index", ik_candidate.h_index},
        {"seed_index", ik_candidate.seed_index},
        {"h", ik_candidate.h},
        {"ik_score", ik_candidate.score},
        {"ik_solve_ms", ik_candidate.solve_ms},
        {"accepted", true},
        {"step", 0}
      };
      record_step(0, start_state, extra);
    }

    const auto left_source = extract_source_box_for_mode(
      left_it->second, start_state, left_tip_, is_top_suction_box_spec(left_box));
    const auto right_source = extract_source_box_for_mode(
      right_it->second, start_state, right_tip_, is_top_suction_box_spec(right_box));
    const auto left_endpoints = extract_rrt_endpoint_candidates_for_side(
      "left", start_state, left_source, left_box, left_box_id);
    const auto right_endpoints = extract_rrt_endpoint_candidates_for_side(
      "right", start_state, right_source, right_box, right_box_id);
    if (left_endpoints.empty() || right_endpoints.empty()) {
      timing.failure_reason = left_endpoints.empty()
        ? "extract_rrt_no_left_endpoint"
        : "extract_rrt_no_right_endpoint";
      timing.rollout_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return timing;
    }

    std::vector<RrtGoalCandidate> goals;
    for (const auto& left : left_endpoints) {
      for (const auto& right : right_endpoints) {
        auto goal = std::make_shared<moveit::core::RobotState>(start_state);
        for (const auto& name : left_arm_group_->getVariableNames()) {
          goal->setVariablePosition(name, left.state->getVariablePosition(name));
        }
        for (const auto& name : right_arm_group_->getVariableNames()) {
          goal->setVariablePosition(name, right.state->getVariablePosition(name));
        }
        goal->setVariablePosition("updown", current_updown(start_state));
        goal->enforceBounds(joint_group_);
        goal->update(true);

        bool left_detached = false;
        bool right_detached = false;
        std::string clear_reason;
        if (!state_clear_for_dual_extract(
              *goal, left_box, left_box_id, right_box, right_box_id,
              &left_detached, &right_detached, &clear_reason) ||
            !left_detached || !right_detached) {
          continue;
        }

        const double score =
          extract_candidate_scorer_->score("left", left, start_state, 0.0) +
          extract_candidate_scorer_->score("right", right, start_state, 0.0) +
          0.2 * std::abs(left.retreat_x - right.retreat_x) +
          0.2 * std::abs(left.lift_z - right.lift_z);
        goals.push_back(RrtGoalCandidate{left, right, goal, score});
      }
    }
    std::sort(goals.begin(), goals.end(),
              [](const RrtGoalCandidate& lhs, const RrtGoalCandidate& rhs) {
                return lhs.score < rhs.score;
              });
    if (goals.size() > extract_rrt_goal_limit_) {
      goals.resize(extract_rrt_goal_limit_);
    }
    if (goals.empty()) {
      timing.failure_reason = "extract_rrt_no_collision_free_dual_endpoint";
      timing.rollout_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return timing;
    }

    std::map<std::string, size_t> failure_counts;
    const std::vector<AttachedBoxSpec> attached_boxes{left_box, right_box};
    for (size_t goal_index = 0; goal_index < goals.size(); ++goal_index) {
      const auto& goal = goals[goal_index];
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      std::string plan_reason;
      const bool planned = plan_joint_space_with_direct_pipeline(
        start_state,
        *goal.state,
        &plan,
        &plan_reason,
        attached_boxes,
        extract_rrt_planning_group_,
        extract_rrt_planning_time_,
        extract_rrt_planning_attempts_);
      if (!planned) {
        failure_counts[plan_reason.empty() ? "extract_rrt_plan_failed" : plan_reason]++;
        continue;
      }

      std::string shortcut_reason;
      auto shortened = shortcut_joint_plan(plan, start_state, attached_boxes, &shortcut_reason);
      auto dense = densify_joint_plan(shortened, 5.0 * M_PI / 180.0, extract_step_x_);
      std::string clear_reason;
      if (!planned_trajectory_clear_in_full_scene(dense, start_state, attached_boxes, &clear_reason)) {
        failure_counts["extract_rrt_path_collision: " + clear_reason]++;
        continue;
      }

      const auto& trajectory = dense.trajectory_.joint_trajectory;
      moveit::core::RobotState final_state(start_state);
      for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
        moveit::core::RobotState state(start_state);
        const auto& point = trajectory.points[point_index];
        for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i) {
          if (is_robot_variable(trajectory.joint_names[i])) {
            state.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
          }
        }
        state.update(true);
        final_state = state;

        bool left_detached = false;
        bool right_detached = false;
        std::string state_reason;
        const bool clear = state_clear_for_dual_extract(
          state, left_box, left_box_id, right_box, right_box_id,
          &left_detached, &right_detached, &state_reason);
        if (record_step) {
          nlohmann::json extra = {
            {"stage_kind", "dual_extract_rrt_step"},
            {"candidate_order", candidate_order},
            {"h_index", ik_candidate.h_index},
            {"seed_index", ik_candidate.seed_index},
            {"h", ik_candidate.h},
            {"ik_score", ik_candidate.score},
            {"ik_solve_ms", ik_candidate.solve_ms},
            {"step", point_index},
            {"goal_index", goal_index},
            {"accepted", clear},
            {"left_retreat_x", goal.left.retreat_x},
            {"right_retreat_x", goal.right.retreat_x},
            {"left_lift_z", goal.left.lift_z},
            {"right_lift_z", goal.right.lift_z},
            {"left_pitch_up_deg", goal.left.pitch_up_rad * 180.0 / M_PI},
            {"right_pitch_up_deg", goal.right.pitch_up_rad * 180.0 / M_PI},
            {"left_detached_from_neighbors", left_detached},
            {"right_detached_from_neighbors", right_detached},
            {"failure_reason", state_reason}
          };
          record_step(point_index, state, extra);
        }
      }

      bool left_detached = false;
      bool right_detached = false;
      std::string final_reason;
      if (!state_clear_for_dual_extract(
            final_state, left_box, left_box_id, right_box, right_box_id,
            &left_detached, &right_detached, &final_reason) ||
          !left_detached || !right_detached) {
        failure_counts["extract_rrt_final_not_detached: " + final_reason]++;
        continue;
      }

      timing.success = true;
      timing.failure_reason.clear();
      timing.final_state = std::make_shared<moveit::core::RobotState>(final_state);
      timing.accepted_steps = trajectory.points.size();
      timing.final_retreat_x = goal.left.retreat_x;
      timing.final_lift_z = goal.left.lift_z;
      timing.final_pitch_deg = goal.left.pitch_up_rad * 180.0 / M_PI;
      timing.right_final_retreat_x = goal.right.retreat_x;
      timing.right_final_lift_z = goal.right.lift_z;
      timing.right_final_pitch_deg = goal.right.pitch_up_rad * 180.0 / M_PI;
      break;
    }

    if (!timing.success) {
      size_t best_count = 0;
      timing.failure_reason = "extract_rrt_no_valid_plan";
      for (const auto& [reason, count] : failure_counts) {
        if (count > best_count) {
          best_count = count;
          timing.failure_reason = reason;
        }
      }
    }
    timing.rollout_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return timing;
  }

  ExtractRolloutTiming rollout_left_extract_from_state(
    const moveit::core::RobotState& start_state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    size_t candidate_order,
    const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
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
      extract_source_box_for_mode(left_it->second, start_state, left_tip_, is_top_suction_box_spec(left_box)),
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
    const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
    const std::function<void(size_t, const moveit::core::RobotState&, const nlohmann::json&)>& record_step = {}) const
  {
    if (extract_rollout_mode_ == "box_pose_rrt") {
      if (!box_pose_rrt_extract_planner_) {
        ExtractRolloutTiming timing;
        timing.candidate_order = candidate_order;
        timing.h_index = ik_candidate.h_index;
        timing.seed_index = ik_candidate.seed_index;
        timing.h = ik_candidate.h;
        timing.ik_score = ik_candidate.score;
        timing.ik_solve_ms = ik_candidate.solve_ms;
        timing.failure_reason = "box_pose_rrt_planner_not_initialized";
        return timing;
      }
      return box_pose_rrt_extract_planner_->rolloutDual(
        start_state,
        left_box,
        left_box_id,
        right_box,
        right_box_id,
        candidate_order,
        ik_candidate.h_index,
        ik_candidate.seed_index,
        ik_candidate.h,
        ik_candidate.score,
        ik_candidate.solve_ms,
        is_top_suction_box_spec(left_box),
        is_top_suction_box_spec(right_box),
        BoxPoseRrtArmPolicy{
          extract_monitor_require_left_detached_,
          extract_monitor_left_front_clearance_levels_,
          extract_monitor_left_retreat_priority_,
          extract_monitor_left_lift_priority_,
          extract_monitor_left_pitch_priority_},
        BoxPoseRrtArmPolicy{
          extract_monitor_require_right_detached_,
          extract_monitor_right_front_clearance_levels_,
          extract_monitor_right_retreat_priority_,
          extract_monitor_right_lift_priority_,
          extract_monitor_right_pitch_priority_},
        record_step);
    }
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
      extract_source_box_for_mode(left_it->second, start_state, left_tip_, is_top_suction_box_spec(left_box)),
      left_box,
      left_box_id,
      extract_source_box_for_mode(right_it->second, start_state, right_tip_, is_top_suction_box_spec(right_box)),
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

  bool solve_top_lift_arm_step(
    const std::string& side,
    const moveit::core::RobotState& current_state,
    double lift_delta_z,
    size_t step_index,
    moveit::core::RobotState* out_state,
    std::string* reason) const
  {
    if (!extract_candidate_solver_ || !out_state) {
      if (reason) *reason = "top_lift_solver_not_initialized";
      return false;
    }
    const std::string& tip = side == "left" ? left_tip_ : right_tip_;
    const Eigen::Isometry3d current_tip = current_state.getGlobalLinkTransform(tip);
    Eigen::Isometry3d target_tip = current_tip;
    target_tip.translation().z() += lift_delta_z;

    Eigen::Quaterniond q(target_tip.linear());
    q.normalize();
    geometry_msgs::msg::Pose target_pose =
      make_pose(target_tip.translation().x(), target_tip.translation().y(), target_tip.translation().z(), q);

    ExtractCandidateSolveRequest request;
    request.side = side;
    request.current_state = &current_state;
    request.target_pose = target_pose;
    request.step_index = step_index;
    request.candidate_index = 0;
    request.retreat_x = 0.0;
    request.retreat_delta_x = 0.0;
    request.lift_z = lift_delta_z * static_cast<double>(step_index);
    request.lift_delta_z = lift_delta_z;
    request.pitch_up_rad = 0.0;
    request.pitch_delta_rad = 0.0;
    request.min_allowed_tip_z = current_tip.translation().z();
    request.fixed_updown = current_updown(current_state);
    request.min_tool_normal_z = -1.1;

    ExtractCandidate candidate;
    if (!extract_candidate_solver_->solve(request, &candidate) || !candidate.state) {
      if (reason) {
        *reason = candidate.rejection_reason.empty()
          ? side + "_top_lift_analytic_failed"
          : candidate.rejection_reason;
      }
      return false;
    }

    *out_state = *candidate.state;
    out_state->setVariablePosition("updown", current_updown(current_state));
    out_state->enforceBounds(joint_group_);
    out_state->update();
    return true;
  }

  ExtractRolloutTiming rollout_dual_updown_lift_from_state(
    const moveit::core::RobotState& start_state,
    const AttachedBoxSpec& left_box,
    int left_box_id,
    const AttachedBoxSpec& right_box,
    int right_box_id,
    size_t candidate_order,
    const robot_motion::core::UpdownAwareIkCandidate& ik_candidate,
    bool require_full_updown_lift)
  {
    ExtractRolloutTiming timing;
    timing.candidate_order = candidate_order;
    timing.h_index = ik_candidate.h_index;
    timing.seed_index = ik_candidate.seed_index;
    timing.h = ik_candidate.h;
    timing.ik_score = ik_candidate.score;
    timing.ik_solve_ms = ik_candidate.solve_ms;

    moveit::core::RobotState current_state(start_state);
    current_state.setVariablePosition("updown", current_updown(start_state));
    current_state.update();
    const double target_lift = require_full_updown_lift
      ? extract_top_updown_lift_distance_
      : std::min(0.45, extract_max_x_);
    const std::string lift_stage = require_full_updown_lift
      ? "direct_updown_lift"
      : "top_suction_lift";

    auto record_state = [&](
      size_t step,
      const moveit::core::RobotState& state,
      bool accepted,
      bool left_detached,
      bool right_detached,
      const std::string& reason) {
      const double lift_z = std::min(
        target_lift, extract_step_x_ * static_cast<double>(step));
      const double previous_lift_z = step == 0 ? 0.0 : std::min(
        target_lift, extract_step_x_ * static_cast<double>(step - 1));
      nlohmann::json extra = {
        {"stage_kind", lift_stage + "_extract"},
        {"accepted", accepted},
        {"candidate_order", candidate_order},
        {"step", step},
        {"lift_z", lift_z},
        {"lift_delta_z", lift_z - previous_lift_z},
        {"left_detached", left_detached},
        {"right_detached", right_detached},
        {"left_box_id", left_box_id},
        {"right_box_id", right_box_id},
        {"grasp_mode", extract_monitor_both_top_suction() ? "top_suction" : "front"},
        {"required_lift", require_full_updown_lift ? extract_top_updown_lift_distance_ : 0.0},
        {"rejection_reason", reason}
      };
      timing.rollout_records.push_back(extract_monitor_selected_extract_replay_state_stage(
        ExtractMonitorSelectedExtractReplayStateRequest{
          extract_monitor_state_.prefix,
          step,
          candidate_order,
          &state,
          left_box_id,
          right_box_id,
          dual_arm_with_updown_joint_names(),
          {left_box, right_box},
          static_box_obstacles_json(),
          extra,
          0.0}));
    };

    bool left_detached = false;
    bool right_detached = false;
    std::string clear_reason;
    if (!is_state_valid_with_attached_boxes(current_state, {left_box, right_box}, true, &clear_reason)) {
      timing.failure_reason = lift_stage + "_start_invalid: " + clear_reason;
      record_state(0, current_state, false, left_detached, right_detached, clear_reason);
      return timing;
    }
    if (!carried_box_clear_scene_obstacles(current_state, left_box, &clear_reason) ||
        !carried_box_clear_scene_obstacles(current_state, right_box, &clear_reason)) {
      timing.failure_reason = lift_stage + "_start_invalid: " + clear_reason;
      record_state(0, current_state, false, left_detached, right_detached, clear_reason);
      return timing;
    }
    std::string left_detach_reason;
    std::string right_detach_reason;
    left_detached = is_top_suction_box_spec(left_box)
      ? top_suction_box_detached_from_stack(current_state, left_box, left_box_id, &left_detach_reason)
      : carried_box_detached_for_extract_mode(current_state, left_box, left_box_id, &left_detach_reason);
    right_detached = is_top_suction_box_spec(right_box)
      ? top_suction_box_detached_from_stack(current_state, right_box, right_box_id, &right_detach_reason)
      : carried_box_detached_for_extract_mode(current_state, right_box, right_box_id, &right_detach_reason);
    clear_reason = left_detached ? right_detach_reason : left_detach_reason;
    record_state(0, current_state, true, left_detached, right_detached, clear_reason);
    if (!require_full_updown_lift && left_detached && right_detached) {
      timing.success = true;
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      timing.accepted_steps = 0;
      return timing;
    }

    const size_t max_steps = std::max<size_t>(
      1,
      static_cast<size_t>(std::ceil(target_lift / std::max(0.001, extract_step_x_))));
    const double initial_updown = current_updown(current_state);
    for (size_t step = 1; step <= max_steps; ++step) {
      moveit::core::RobotState next_state(current_state);
      const double achieved_lift = std::min(
        target_lift, extract_step_x_ * static_cast<double>(step));
      next_state.setVariablePosition("updown", initial_updown + achieved_lift);
      next_state.update();

      std::string step_reason;
      left_detached = false;
      right_detached = false;
      if (!next_state.satisfiesBounds(joint_group_)) {
        timing.failure_reason = lift_stage + "_target_out_of_bounds_step_" +
          std::to_string(step);
        record_state(step, next_state, false, left_detached, right_detached, timing.failure_reason);
        return timing;
      }
      if (!is_state_valid_with_attached_boxes(next_state, {left_box, right_box}, true, &step_reason)) {
        timing.failure_reason = lift_stage + "_collision_step_" +
          std::to_string(step) + ": " + step_reason;
        record_state(step, next_state, false, left_detached, right_detached, step_reason);
        return timing;
      }
      if (!carried_box_clear_scene_obstacles(next_state, left_box, &step_reason) ||
          !carried_box_clear_scene_obstacles(next_state, right_box, &step_reason)) {
        timing.failure_reason = lift_stage + "_collision_step_" +
          std::to_string(step) + ": " + step_reason;
        record_state(step, next_state, false, left_detached, right_detached, step_reason);
        return timing;
      }
      std::string left_step_detach_reason;
      std::string right_step_detach_reason;
      left_detached = is_top_suction_box_spec(left_box)
        ? top_suction_box_detached_from_stack(
            next_state, left_box, left_box_id, &left_step_detach_reason)
        : carried_box_detached_for_extract_mode(
            next_state, left_box, left_box_id, &left_step_detach_reason);
      right_detached = is_top_suction_box_spec(right_box)
        ? top_suction_box_detached_from_stack(
            next_state, right_box, right_box_id, &right_step_detach_reason)
        : carried_box_detached_for_extract_mode(
            next_state, right_box, right_box_id, &right_step_detach_reason);
      step_reason = left_detached ? right_step_detach_reason : left_step_detach_reason;

      current_state = next_state;
      timing.accepted_steps = step;
      timing.final_lift_z = achieved_lift;
      timing.right_final_lift_z = timing.final_lift_z;
      record_state(step, current_state, true, left_detached, right_detached, step_reason);
      if (!require_full_updown_lift && left_detached && right_detached) {
        timing.success = true;
        timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
        timing.failure_reason.clear();
        return timing;
      }
    }

    if (require_full_updown_lift) {
      timing.success = true;
      timing.final_state = std::make_shared<moveit::core::RobotState>(current_state);
      timing.failure_reason.clear();
      return timing;
    }

    timing.failure_reason = "top_suction_lift_reached_max_without_detachment";
    return timing;
  }

  bool benchmark_all_legal_ik_extract(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const robot_motion::core::UpdownAwareIkResult& ik_result,
    const AttachedBoxSpec& left_box,
    int left_box_id)
  {
    ExtractBenchmarkRunner runner(extract_benchmark_runner_config(), extract_benchmark_runner_callbacks());
    return runner.runLeft(prefix, seed_state, ik_result, left_box, left_box_id);
  }

  bool benchmark_all_legal_ik_dual_extract(
    const std::string& prefix,
    const moveit::core::RobotState& seed_state,
    const robot_motion::core::UpdownAwareIkResult& ik_result,
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

    const auto names = dual_arm_with_updown_joint_names();
    const auto plan = single_state_plan(state, names, 0.0);
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
    const robot_motion::core::UpdownAwareIkResult& ik_result,
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
      enriched["extract_ik"] = "left_arm_analytic_fixed_updown";
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

  ExecutionTrajectoryAdapterConfig execution_trajectory_adapter_config() const
  {
    ExecutionTrajectoryAdapterConfig config;
    config.include_turn = execution_include_turn_;
    config.allow_hold_missing_target_joints = execution_allow_hold_missing_target_joints_;
    config.reject_unmapped_planned_joints = execution_reject_unmapped_planned_joints_;
    return config;
  }

  bool build_alfa_execution_goal(
    const trajectory_msgs::msg::JointTrajectory& source,
    const moveit::core::RobotState& planning_start_state,
    FollowJointTrajectory::Goal* goal,
    std::string* reason) const
  {
    const ExecutionTrajectoryAdapter adapter(execution_trajectory_adapter_config());
    ExecutionTrajectoryBuildRequest request;
    request.source = &source;
    request.is_robot_variable = [this](const std::string& name) { return is_robot_variable(name); };
    request.hold_position = [&planning_start_state](const std::string& name) {
      return planning_start_state.getVariablePosition(name);
    };
    return adapter.buildGoal(
      request,
      goal,
      reason);
  }

  std::string moveit_to_alfa_joint_name(const std::string& name) const
  {
    return ExecutionTrajectoryAdapter(execution_trajectory_adapter_config()).moveItToAlfaJointName(name);
  }

  bool robot_state_matches(
    const moveit::core::RobotState& goal_state,
    const moveit::core::RobotState& current_state,
    const std::vector<std::string>& target_names) const
  {
    const ExecutionTrajectoryAdapter adapter(execution_trajectory_adapter_config());
    ExecutionStateMatchRequest request;
    request.target_names = target_names;
    request.tolerance = joint_goal_tolerance_rad_;
    request.is_robot_variable = [this](const std::string& name) { return is_robot_variable(name); };
    request.goal_position = [&goal_state](const std::string& name) {
      return goal_state.getVariablePosition(name);
    };
    request.current_position = [&current_state](const std::string& name) {
      return current_state.getVariablePosition(name);
    };
    return adapter.robotStateMatches(request);
  }

  bool joint_state_matches(
    const moveit::core::RobotState& goal_state,
    const sensor_msgs::msg::JointState& msg,
    const std::vector<std::string>& target_names) const
  {
    const ExecutionTrajectoryAdapter adapter(execution_trajectory_adapter_config());
    ExecutionJointStateMatchRequest request;
    request.current = &msg;
    request.target_names = target_names;
    request.tolerance = joint_goal_tolerance_rad_;
    request.is_robot_variable = [this](const std::string& name) { return is_robot_variable(name); };
    request.goal_position = [&goal_state](const std::string& name) {
      return goal_state.getVariablePosition(name);
    };
    return adapter.jointStateMatches(request);
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
    const auto header = motion_flow_header_json(MotionFlowHeaderRequest{
      planning_group_,
      box_front_x_,
      scene_y_shift_,
      world_to_base_z_,
      fixed_updown_,
      velocity_scale_,
      acceleration_scale_,
      max_rounds_,
      include_top_suction_,
      execute_,
      container_obstacle_json(),
      static_box_obstacles_json(),
      attached_box_config_json(),
      {
        {"left_candidates_deg", pose_family_degrees_json(left_loaded_pose_family_)},
        {"right_candidates_deg", pose_family_degrees_json(right_loaded_pose_family_)},
        {"left_preferred_index", left_preferred_loaded_pose_index_},
        {"right_preferred_index", right_preferred_loaded_pose_index_},
        {"family_distance_weight", ik_config_.cost_loaded_family_distance},
        {"preferred_distance_weight", ik_config_.cost_loaded_preferred_distance}
      },
      {
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
      }});
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
    return alfa_robot::motion::container_obstacle_json(
      enable_container_obstacle_,
      container_frame_,
      container_length_,
      container_width_,
      container_height_,
      container_center_x_,
      container_center_y_ + scene_y_shift_,
      container_center_y_,
      scene_y_shift_,
      container_floor_z_,
      container_wall_thickness_,
      container_panels());
  }

  nlohmann::json attached_box_config_json() const
  {
    return alfa_robot::motion::attached_box_config_json(
      enable_attached_box_collision_,
      carried_box_depth_,
      carried_box_width_,
      carried_box_height_);
  }

  nlohmann::json static_box_obstacles_json() const
  {
    return alfa_robot::motion::static_box_obstacles_json(
      enable_static_box_obstacles_,
      scene_adapter_ ? scene_adapter_->activeStaticLeftBoxId() : 0,
      scene_adapter_ ? scene_adapter_->activeStaticRightBoxId() : 0,
      static_box_obstacle_inset_,
      static_box_obstacles());
  }

  nlohmann::json active_attached_boxes_json() const
  {
    return attached_boxes_json(active_attached_boxes());
  }

  void record_monitor_extract_replay_step(
    size_t step,
    size_t candidate_order,
    const moveit::core::RobotState& state,
    const nlohmann::json& extra,
    std::vector<nlohmann::json>* rollout_records) const
  {
    if (!rollout_records) {
      return;
    }
    const auto names = dual_arm_with_updown_joint_names();
    rollout_records->push_back(extract_monitor_selected_extract_replay_state_stage(
      ExtractMonitorSelectedExtractReplayStateRequest{
        extract_monitor_state_.prefix,
        step,
        candidate_order,
        &state,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        names,
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        static_box_obstacles_json(),
        extra,
        0.1 * static_cast<double>(step)}));
  }

  bool finish_extract_monitor_stage(
    nlohmann::json snapshot,
    const std::string& context,
    const std::string& success_message,
    std::string* message)
  {
    // Rerun 等下游可视化必须与 MoveIt 规划场景共用同一份碰撞几何：这里注入的
    // container_panels 就是 MotionSceneAdapter 实际写入 PlanningScene 的同一份数据，
    // 不是重新计算的另一套。
    snapshot["container_panels"] = alfa_robot::motion::container_panels_json(container_panels());
    const auto result = extract_monitor_snapshot_writer_.writeStageSnapshot(
      ExtractMonitorStageSnapshotWriteRequest{
        &snapshot,
        context,
        success_message});
    if (result.success) {
      if (message) {
        *message = result.message;
      }
      return true;
    }
    if (!result.error_log_message.empty()) {
      RCLCPP_ERROR(get_logger(), "%s", result.error_log_message.c_str());
    }
    return fail(result.message);
  }

  std::vector<robot_motion::core::UpdownAwareIkCandidate> selected_monitor_ik_candidates(
    const robot_motion::core::UpdownAwareIkResult& ik_result,
    IkCandidateSelectionStats* stats,
    bool defer_candidate_limit = false) const
  {
    if (defer_candidate_limit) {
      auto config = ik_candidate_selector_config();
      config.candidate_limit = 0;
      return IkCandidateSelector(config).selectLegalFromResult(ik_result, stats);
    }
    if (ik_candidate_selector_) {
      return ik_candidate_selector_->selectLegalFromResult(ik_result, stats);
    }
    if (stats) {
      *stats = IkCandidateSelectionStats{};
    }
    return {};
  }

  void apply_extract_ik_candidate_limit(
    std::vector<robot_motion::core::UpdownAwareIkCandidate>* candidates,
    std::vector<moveit::core::RobotStatePtr>* states) const
  {
    if (!candidates || !states || candidates->size() != states->size()) return;
    const size_t primary_limit = extract_benchmark_candidate_limit_;
    if (primary_limit == 0) return;
    const size_t total_limit = std::min(
      candidates->size(),
      primary_limit + extract_ik_candidate_reserve_limit_);
    if (candidates->size() <= total_limit && !extract_ik_stratified_limit_enabled_) return;
    if (extract_ik_candidate_reserve_limit_ == 0 && !extract_ik_stratified_limit_enabled_) {
      candidates->resize(std::min(primary_limit, candidates->size()));
      states->resize(std::min(primary_limit, states->size()));
      return;
    }

    std::vector<size_t> primary_indices;
    primary_indices.reserve(std::min(primary_limit, total_limit));
    std::vector<size_t> reserve_indices;
    reserve_indices.reserve(extract_ik_candidate_reserve_limit_);
    std::vector<size_t> filler_indices;
    filler_indices.reserve(total_limit);
    std::vector<bool> selected(candidates->size(), false);
    std::vector<long long> used_h_buckets;
    used_h_buckets.reserve(total_limit);
    const auto h_bucket = [this](double h) {
      return static_cast<long long>(std::llround(h / extract_ik_stratified_h_bucket_));
    };
    const auto h_bucket_used = [&](long long bucket) {
      return std::find(used_h_buckets.begin(), used_h_buckets.end(), bucket) != used_h_buckets.end();
    };

    const size_t top_score_count = extract_ik_stratified_limit_enabled_
      ? std::min(total_limit, extract_ik_stratified_top_score_count_)
      : std::min(total_limit, primary_limit);
    for (size_t index = 0; index < candidates->size() && primary_indices.size() < top_score_count; ++index) {
      selected[index] = true;
      primary_indices.push_back(index);
      used_h_buckets.push_back(h_bucket((*candidates)[index].h));
    }

    if (extract_ik_stratified_limit_enabled_ || extract_ik_candidate_reserve_stratified_) {
      for (size_t index = top_score_count; index < candidates->size() &&
           primary_indices.size() + reserve_indices.size() < total_limit; ++index) {
        const auto bucket = h_bucket((*candidates)[index].h);
        if (h_bucket_used(bucket)) continue;
        selected[index] = true;
        reserve_indices.push_back(index);
        used_h_buckets.push_back(bucket);
      }
    }
    for (size_t index = 0; index < candidates->size() &&
         primary_indices.size() + reserve_indices.size() + filler_indices.size() < total_limit; ++index) {
      if (selected[index]) continue;
      selected[index] = true;
      filler_indices.push_back(index);
    }

    std::vector<size_t> selected_indices;
    selected_indices.reserve(primary_indices.size() + reserve_indices.size() + filler_indices.size());
    if (extract_ik_candidate_reserve_interleave_stride_ == 0 || reserve_indices.empty()) {
      selected_indices.insert(selected_indices.end(), primary_indices.begin(), primary_indices.end());
      selected_indices.insert(selected_indices.end(), reserve_indices.begin(), reserve_indices.end());
    } else {
      size_t primary_cursor = 0;
      size_t reserve_cursor = 0;
      while (primary_cursor < primary_indices.size() || reserve_cursor < reserve_indices.size()) {
        for (size_t i = 0;
             i < extract_ik_candidate_reserve_interleave_stride_ && primary_cursor < primary_indices.size();
             ++i) {
          selected_indices.push_back(primary_indices[primary_cursor++]);
        }
        if (reserve_cursor < reserve_indices.size()) {
          selected_indices.push_back(reserve_indices[reserve_cursor++]);
        }
      }
    }
    selected_indices.insert(selected_indices.end(), filler_indices.begin(), filler_indices.end());
    std::stable_sort(selected_indices.begin(), selected_indices.end(), [&](size_t lhs, size_t rhs) {
      const double lhs_score = std::isfinite((*candidates)[lhs].score)
        ? (*candidates)[lhs].score
        : std::numeric_limits<double>::infinity();
      const double rhs_score = std::isfinite((*candidates)[rhs].score)
        ? (*candidates)[rhs].score
        : std::numeric_limits<double>::infinity();
      if (lhs_score != rhs_score) return lhs_score < rhs_score;
      return lhs < rhs;
    });

    std::vector<robot_motion::core::UpdownAwareIkCandidate> limited_candidates;
    std::vector<moveit::core::RobotStatePtr> limited_states;
    limited_candidates.reserve(selected_indices.size());
    limited_states.reserve(selected_indices.size());
    for (const auto index : selected_indices) {
      limited_candidates.push_back((*candidates)[index]);
      limited_states.push_back((*states)[index]);
    }
    *candidates = std::move(limited_candidates);
    *states = std::move(limited_states);
  }

  void reorder_extract_ik_candidates_by_loaded_distance(
    std::vector<robot_motion::core::UpdownAwareIkCandidate>* candidates,
    std::vector<moveit::core::RobotStatePtr>* states) const
  {
    if (!candidates || !states || candidates->size() != states->size() ||
        candidates->empty() || !loaded_pose_selector_ ||
        extract_ik_loaded_distance_order_weight_ <= 0.0) {
      return;
    }

    struct RankedIndex
    {
      size_t index = 0;
      double rank = 0.0;
      double loaded_distance = 0.0;
    };
    std::vector<RankedIndex> order;
    order.reserve(candidates->size());
    for (size_t index = 0; index < candidates->size(); ++index) {
      const auto& state = (*states)[index];
      const double loaded_distance = state
        ? loaded_pose_selector_->select(*state).distance_sum
        : std::numeric_limits<double>::infinity();
      order.push_back({
        index,
        (*candidates)[index].score + extract_ik_loaded_distance_order_weight_ * loaded_distance,
        loaded_distance});
    }
    std::stable_sort(order.begin(), order.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.rank != rhs.rank) return lhs.rank < rhs.rank;
      return lhs.loaded_distance < rhs.loaded_distance;
    });

    auto old_candidates = std::move(*candidates);
    auto old_states = std::move(*states);
    candidates->clear();
    states->clear();
    candidates->reserve(order.size());
    states->reserve(order.size());
    for (const auto& item : order) {
      candidates->push_back(std::move(old_candidates[item.index]));
      states->push_back(std::move(old_states[item.index]));
    }
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

  ExtractMonitorStageCallbacks extract_monitor_stage_callbacks()
  {
    ExtractMonitorStageCallbacks callbacks;
    callbacks.ik = [this](std::string* message) { return run_extract_monitor_ik_stage(message); };
    callbacks.extract = [this](std::string* message) { return run_extract_monitor_extract_stage(message); };
    callbacks.loaded = [this](std::string* message) { return run_extract_monitor_loaded_stage(message); };
    callbacks.final = [this](std::string* message) { return run_extract_monitor_final_stage(message); };
    return callbacks;
  }

  bool run_extract_monitor_next(std::string* message)
  {
    std::lock_guard<std::mutex> lock(extract_monitor_mutex_);
    if (!message) {
      return fail("extract monitor: output message is null");
    }

    return extract_monitor_controller_.runNext(
      extract_monitor_stage_callbacks(),
      [this] { return extract_monitor_last_stage_ms_; },
      message);
  }

  bool configure_extract_monitor_task(
    const alfa_robot_moveit_config::srv::ConfigureExtractMonitor::Request& request,
    std::string* message)
  {
    std::lock_guard<std::mutex> lock(extract_monitor_mutex_);
    const auto configure_start = std::chrono::steady_clock::now();
    if (request.update_runtime_config) {
      if (!std::isfinite(request.box_front_x) || request.box_front_x <= 0.0) {
        if (message) *message = "box_front_x must be finite and positive";
        return fail("configure extract monitor: invalid box_front_x");
      }
      if (!std::isfinite(request.scene_y_shift)) {
        if (message) *message = "scene_y_shift must be finite";
        return fail("configure extract monitor: invalid scene_y_shift");
      }
      static const std::unordered_set<std::string> supported_rollout_modes{
        "greedy",
        "box_pose_rrt",
        "moveit_rrt_legacy",
        "top_lift_legacy",
        "top_updown_lift",
        "direct_updown_lift",
      };
      if (supported_rollout_modes.find(request.extract_rollout_mode) ==
          supported_rollout_modes.end())
      {
        if (message) *message = "unsupported extract_rollout_mode: " + request.extract_rollout_mode;
        return fail("configure extract monitor: unsupported extract_rollout_mode");
      }
      if (request.extract_box_pose_rrt_max_iterations <= 0) {
        if (message) *message = "extract_box_pose_rrt_max_iterations must be positive";
        return fail("configure extract monitor: invalid RRT iteration limit");
      }

      box_front_x_ = request.box_front_x;
      scene_y_shift_ = request.scene_y_shift;
      extract_rollout_mode_ = request.extract_rollout_mode;
      extract_loaded_lateral_shift_enabled_ = request.loaded_lateral_shift_enabled;
      extract_box_pose_rrt_max_iterations_ =
        static_cast<size_t>(request.extract_box_pose_rrt_max_iterations);
      if (scene_adapter_) {
        scene_adapter_->updateContainerGeometry(container_geometry_config());
        scene_adapter_->updateBoxWallGeometry(box_wall_geometry_config());
      }
      apply_container_obstacles();
    }
    const auto boxes = make_boxes(box_front_x_, scene_y_shift_);
    if (boxes.find(request.left_box_id) == boxes.end() ||
        boxes.find(request.right_box_id) == boxes.end())
    {
      const std::string error = "unknown box id: L" + std::to_string(request.left_box_id) +
                                "/R" + std::to_string(request.right_box_id);
      if (message) {
        *message = error;
      }
      return fail("configure extract monitor: " + error);
    }
    if (request.snapshot_path.empty()) {
      if (message) {
        *message = "snapshot_path is empty";
      }
      return fail("configure extract monitor: snapshot_path is empty");
    }

    extract_demo_left_box_id_ = request.left_box_id;
    extract_demo_right_box_id_ = request.right_box_id;
    extract_monitor_left_top_suction_ = request.left_top_suction;
    extract_monitor_right_top_suction_ = request.right_top_suction;
    extract_monitor_use_explicit_targets_ = request.use_explicit_targets;
    if (extract_monitor_use_explicit_targets_) {
      const auto valid_target_frame = [this](const geometry_msgs::msg::PoseStamped& target) {
          return target.header.frame_id.empty() || target.header.frame_id == ik_config_.base_frame;
        };
      if (!valid_target_frame(request.left_target) || !valid_target_frame(request.right_target)) {
        if (message) *message = "explicit target frame must be " + ik_config_.base_frame;
        return fail("configure extract monitor: explicit target frame mismatch");
      }
      extract_monitor_left_target_ = request.left_target.pose;
      extract_monitor_right_target_ = request.right_target.pose;
    }
    extract_monitor_require_left_detached_ = true;
    extract_monitor_require_right_detached_ = true;
    extract_monitor_left_front_clearance_levels_ = extract_monitor_left_top_suction_ ? 0 : 1;
    extract_monitor_right_front_clearance_levels_ = extract_monitor_right_top_suction_ ? 0 : 1;
    extract_monitor_left_retreat_priority_ = extract_monitor_left_top_suction_ ? 1.0 : 3.0;
    extract_monitor_left_lift_priority_ = extract_monitor_left_top_suction_ ? 3.0 : 1.0;
    extract_monitor_left_pitch_priority_ = extract_monitor_left_top_suction_ ? 2.0 : 1.0;
    extract_monitor_right_retreat_priority_ = extract_monitor_right_top_suction_ ? 1.0 : 3.0;
    extract_monitor_right_lift_priority_ = extract_monitor_right_top_suction_ ? 3.0 : 1.0;
    extract_monitor_right_pitch_priority_ = extract_monitor_right_top_suction_ ? 2.0 : 1.0;
    if (request.strategy.task_type != 0) {
      extract_monitor_require_left_detached_ = request.strategy.left.require_full_detachment;
      extract_monitor_require_right_detached_ = request.strategy.right.require_full_detachment;
      extract_monitor_left_front_clearance_levels_ = extract_monitor_left_top_suction_ ? 0 :
        std::max<size_t>(1, request.strategy.left.front_clearance_levels);
      extract_monitor_right_front_clearance_levels_ = extract_monitor_right_top_suction_ ? 0 :
        std::max<size_t>(1, request.strategy.right.front_clearance_levels);
      extract_monitor_left_retreat_priority_ = request.strategy.left.retreat_priority;
      extract_monitor_left_lift_priority_ = request.strategy.left.lift_priority;
      extract_monitor_left_pitch_priority_ = request.strategy.left.pitch_priority;
      extract_monitor_right_retreat_priority_ = request.strategy.right.retreat_priority;
      extract_monitor_right_lift_priority_ = request.strategy.right.lift_priority;
      extract_monitor_right_pitch_priority_ = request.strategy.right.pitch_priority;
    } else if (extract_monitor_both_top_suction()) {
      const double left_z = extract_monitor_use_explicit_targets_
        ? extract_monitor_left_target_.position.z
        : boxes.at(request.left_box_id).z;
      const double right_z = extract_monitor_use_explicit_targets_
        ? extract_monitor_right_target_.position.z
        : boxes.at(request.right_box_id).z;
      constexpr double equal_height_tolerance = 0.02;
      if (left_z > right_z + equal_height_tolerance) {
        extract_monitor_require_left_detached_ = false;
      } else if (right_z > left_z + equal_height_tolerance) {
        extract_monitor_require_right_detached_ = false;
      }
    } else if (!extract_monitor_left_top_suction_ && !extract_monitor_right_top_suction_) {
      const double left_z = extract_monitor_use_explicit_targets_
        ? extract_monitor_left_target_.position.z
        : boxes.at(request.left_box_id).z;
      const double right_z = extract_monitor_use_explicit_targets_
        ? extract_monitor_right_target_.position.z
        : boxes.at(request.right_box_id).z;
      constexpr double equal_height_tolerance = 0.02;
      if (left_z > right_z + equal_height_tolerance) {
        extract_monitor_right_front_clearance_levels_ = 2;
      } else if (right_z > left_z + equal_height_tolerance) {
        extract_monitor_left_front_clearance_levels_ = 2;
      }
    }
    extract_monitor_top_height_mismatch_ =
      extract_monitor_both_top_suction() &&
      extract_monitor_require_left_detached_ != extract_monitor_require_right_detached_;
    extract_monitor_top_suction_ = extract_monitor_both_top_suction();
    extract_monitor_snapshot_path_ = request.snapshot_path;
    extract_monitor_snapshot_writer_.setPath(extract_monitor_snapshot_path_);
    extract_monitor_controller_.reset();
    extract_monitor_state_ = ExtractMonitorState{};
    extract_monitor_last_stage_ms_ = 0.0;
    if (!ensure_optimized_ik_solver()) {
      if (message) {
        *message = "optimized IK solver is not initialized";
      }
      return fail("configure extract monitor: optimized IK solver is not initialized");
    }
    clear_carried_boxes_from_scene();
    set_static_box_wall_opening(extract_demo_left_box_id_, extract_demo_right_box_id_, "configure_extract_monitor");

    const double configure_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - configure_start).count();
    if (message) {
      *message = "configured extract monitor: L" + std::to_string(extract_demo_left_box_id_) +
                 "/R" + std::to_string(extract_demo_right_box_id_) +
                 " modes=(" + grasp_mode_label(extract_monitor_left_top_suction_) +
                 "," + grasp_mode_label(extract_monitor_right_top_suction_) + ")" +
                 " box_front_x=" + std::to_string(box_front_x_) +
                 " scene_y_shift=" + std::to_string(scene_y_shift_) +
                 " rollout=" + extract_rollout_mode_ +
                 " front_clearance_levels=(" +
                 std::to_string(extract_monitor_left_front_clearance_levels_) + "," +
                 std::to_string(extract_monitor_right_front_clearance_levels_) + ")" +
                 " snapshot=" + extract_monitor_snapshot_path_ +
                 " configure_ms=" + std::to_string(configure_ms);
    }
    RCLCPP_INFO(
      get_logger(),
      "Configured extract monitor task: L%d/R%d modes=(%s,%s) front_clearance_levels=(%zu,%zu) snapshot=%s",
      extract_demo_left_box_id_,
      extract_demo_right_box_id_,
      grasp_mode_label(extract_monitor_left_top_suction_).c_str(),
      grasp_mode_label(extract_monitor_right_top_suction_).c_str(),
      extract_monitor_left_front_clearance_levels_,
      extract_monitor_right_front_clearance_levels_,
      extract_monitor_snapshot_path_.c_str());
    return true;
  }

  bool run_extract_monitor_full_selected(std::string* message)
  {
    std::lock_guard<std::mutex> lock(extract_monitor_mutex_);
    if (!message) {
      return fail("extract monitor full: output message is null");
    }

    const auto result = extract_monitor_controller_.runFull(
      extract_monitor_stage_callbacks(),
      [this] { return extract_monitor_last_stage_ms_; });
    if (!result.success) {
      nlohmann::json failure_snapshot = alfa_robot::motion::extract_monitor_full_selected_snapshot(
        extract_monitor_snapshot_writer_.readOrEmpty(),
        box_front_x_,
        scene_y_shift_,
        result.total_elapsed_ms,
        result.stage_elapsed_ms);
      failure_snapshot["success"] = false;
      failure_snapshot["failure_reason"] = result.message;
      std::string snapshot_error;
      if (!extract_monitor_snapshot_writer_.write(failure_snapshot, &snapshot_error)) {
        RCLCPP_ERROR(
          get_logger(), "%s",
          extract_monitor_snapshot_writer_.writeError(snapshot_error).c_str());
      }
      *message = result.message;
      return false;
    }

    std::string error;
    if (!extract_monitor_snapshot_writer_.writeFullSelectedSnapshot(
      ExtractMonitorFullSelectedSnapshotRequest{
        box_front_x_,
        scene_y_shift_,
        result.total_elapsed_ms,
        result.stage_elapsed_ms},
      &error))
    {
      RCLCPP_ERROR(get_logger(), "%s", extract_monitor_snapshot_writer_.writeError(error).c_str());
      return fail("extract monitor full: failed to write snapshot");
    }

    *message = extract_monitor_snapshot_writer_.appendSnapshotPath(result.message);
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

    extract_monitor_state_ = make_extract_monitor_initial_state(
      ExtractMonitorInitialStateRequest{
        left_box_id,
        right_box_id,
        make_carried_box_spec("left", left_box_id, extract_monitor_left_top_suction_),
        make_carried_box_spec("right", right_box_id, extract_monitor_right_top_suction_),
        robot_model_,
        joint_group_,
        ExtractMonitorArmSeed{left_pregrasp_arm_, right_pregrasp_arm_, extract_grasp_ik_home_updown_, extract_monitor_turn_},
        ExtractMonitorArmSeed{left_loaded_arm_, right_loaded_arm_, extract_grasp_ik_home_updown_, extract_monitor_turn_}});

    moveit::core::RobotState selected_state(*extract_monitor_state_.seed_state);
    nlohmann::json ik_extra;
    robot_motion::core::UpdownAwareIkResult ik_result;
    const auto left_pose = extract_monitor_use_explicit_targets_
      ? extract_monitor_left_target_
      : make_outer_box_grasp_pose(left_it->second, extract_monitor_left_top_suction_, true);
    const auto right_pose = extract_monitor_use_explicit_targets_
      ? extract_monitor_right_target_
      : make_outer_box_grasp_pose(right_it->second, extract_monitor_right_top_suction_, false);
    if (!solve_dual_tip_ik_state(
          extract_monitor_state_.prefix + "/monitor_ik",
          left_pose,
          right_pose,
          extract_monitor_left_top_suction_,
          extract_monitor_right_top_suction_,
          *extract_monitor_state_.seed_state,
          &selected_state,
          &ik_extra,
          &ik_result,
          extract_monitor_capture_raw_ik_)) {
      *message = "IK阶段失败: " + last_error_;
      return false;
    }

    if (extract_monitor_capture_raw_ik_) {
      std::vector<moveit::core::RobotStatePtr> raw_states;
      raw_states.reserve(ik_result.pre_score_candidates.size());
      for (const auto& candidate : ik_result.pre_score_candidates) {
        raw_states.push_back(std::make_shared<moveit::core::RobotState>(
          robot_state_from_ik_candidate(*extract_monitor_state_.seed_state, candidate, joint_group_)));
      }
      nlohmann::json raw_records = extract_monitor_candidate_records_json(
        ik_result.pre_score_candidates, raw_states);
      for (size_t index = 0; index < raw_records.size(); ++index) {
        raw_records[index]["generation_index"] = index;
        raw_records[index]["pre_score"] = true;
        raw_records[index]["score"] = nullptr;
      }
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - stage_start).count();
      extract_monitor_last_stage_ms_ = elapsed_ms;
      nlohmann::json snapshot = extract_monitor_ik_snapshot(
        ExtractMonitorIkSnapshotRequest{
          extract_monitor_snapshot_path_,
          elapsed_ms,
          left_box_id,
          right_box_id,
          box_front_x_,
          scene_y_shift_,
          &ik_result,
          IkCandidateSelectionStats{},
          ik_candidate_rejection_counts_json(ik_result),
          raw_records});
      snapshot["type"] = "ik_pre_score_candidates";
      snapshot["phase"] = "ik_pre_score_candidates";
      snapshot["phase_label"] = "代价函数前原始合法 IK 解";
      snapshot["raw_legal_count"] = raw_records.size();
      snapshot["cost_scored"] = false;
      snapshot["deduplicated"] = false;
      snapshot["scene_filtered"] = false;
      return finish_extract_monitor_stage(
        snapshot,
        "extract monitor raw IK",
        "原始 IK 阶段完成: raw_legal=" + std::to_string(raw_records.size()) +
          " elapsed=" + std::to_string(elapsed_ms) + "ms snapshot=" + extract_monitor_snapshot_path_,
        message);
    }

    IkCandidateSelectionStats dedup_stats;
    extract_monitor_state_.ik_result = ik_result;
    const auto selected_candidates = selected_monitor_ik_candidates(ik_result, &dedup_stats, true);
    extract_monitor_state_.legal_candidates.clear();
    extract_monitor_state_.candidate_states.clear();
    extract_monitor_state_.legal_candidates.reserve(selected_candidates.size());
    extract_monitor_state_.candidate_states.reserve(selected_candidates.size());
    std::map<std::string, size_t> scene_filter_rejections;
    nlohmann::json scene_rejected_records = nlohmann::json::array();
    size_t scene_filter_input_count = 0;
    for (const auto& candidate : selected_candidates) {
      ++scene_filter_input_count;
      auto state = std::make_shared<moveit::core::RobotState>(
        robot_state_from_ik_candidate(*extract_monitor_state_.seed_state, candidate, joint_group_));
      std::string collision_reason;
      if (!state_clear_for_dual_grasp_start(
            *state,
            extract_monitor_state_.left_box,
            extract_monitor_state_.right_box,
            &collision_reason))
      {
        const std::string rejection_reason = collision_reason.empty() ?
          "ik_candidate_scene_rejected" : collision_reason;
        scene_filter_rejections[rejection_reason]++;
        auto rejected_record = alfa_robot::motion::extract_monitor_candidate_json(
          candidate, scene_rejected_records.size(), *state);
        rejected_record["scene_rejection_reason"] = rejection_reason;
        rejected_record["scene_rejected"] = true;
        scene_rejected_records.push_back(std::move(rejected_record));
        continue;
      }
      extract_monitor_state_.legal_candidates.push_back(candidate);
      extract_monitor_state_.candidate_states.push_back(std::move(state));
      const size_t accepted_limit = extract_benchmark_candidate_limit_ == 0
        ? 0
        : extract_benchmark_candidate_limit_ + extract_ik_candidate_reserve_limit_;
      if (!extract_ik_stratified_limit_enabled_ &&
          accepted_limit > 0 &&
          extract_monitor_state_.legal_candidates.size() >= accepted_limit) {
        break;
      }
    }
    const size_t scene_filter_accepted_before_limit = extract_monitor_state_.legal_candidates.size();
    apply_extract_ik_candidate_limit(
      &extract_monitor_state_.legal_candidates,
      &extract_monitor_state_.candidate_states);
    reorder_extract_ik_candidates_by_loaded_distance(
      &extract_monitor_state_.legal_candidates,
      &extract_monitor_state_.candidate_states);
    dedup_stats.selected_count = extract_monitor_state_.legal_candidates.size();

    const nlohmann::json records = extract_monitor_candidate_records_json(
      extract_monitor_state_.legal_candidates,
      extract_monitor_state_.candidate_states);

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    nlohmann::json snapshot = extract_monitor_ik_snapshot(
      ExtractMonitorIkSnapshotRequest{
        extract_monitor_snapshot_path_,
        elapsed_ms,
        left_box_id,
        right_box_id,
        box_front_x_,
        scene_y_shift_,
        &ik_result,
        dedup_stats,
        ik_candidate_rejection_counts_json(ik_result),
        records});
    nlohmann::json all_ik_candidate_records = nlohmann::json::array();
    for (const auto& candidate : ik_result.candidates) {
      const auto candidate_state = robot_state_from_ik_candidate(
        *extract_monitor_state_.seed_state, candidate, joint_group_);
      all_ik_candidate_records.push_back(alfa_robot::motion::extract_monitor_candidate_json(
        candidate, all_ik_candidate_records.size(), candidate_state));
    }
    snapshot["all_ik_candidate_records"] = std::move(all_ik_candidate_records);
    snapshot["scene_filter_input_count"] = scene_filter_input_count;
    snapshot["scene_filter_accepted_before_limit_count"] = scene_filter_accepted_before_limit;
    snapshot["scene_filter_accepted_count"] = extract_monitor_state_.legal_candidates.size();
    snapshot["extract_ik_primary_candidate_limit"] = extract_benchmark_candidate_limit_;
    snapshot["extract_ik_candidate_reserve_limit"] = extract_ik_candidate_reserve_limit_;
    snapshot["extract_ik_candidate_reserve_stratified"] = extract_ik_candidate_reserve_stratified_;
    snapshot["extract_ik_candidate_reserve_interleave_stride"] = extract_ik_candidate_reserve_interleave_stride_;
    snapshot["extract_ik_loaded_distance_order_weight"] = extract_ik_loaded_distance_order_weight_;
    snapshot["scene_filter_rejections"] = failure_counts_json(scene_filter_rejections);
    snapshot["scene_rejected_records"] = std::move(scene_rejected_records);
    snapshot["attached_boxes"] = attached_boxes_json(
      {extract_monitor_state_.left_box, extract_monitor_state_.right_box});
    snapshot["static_box_obstacles"] = static_box_obstacles_json();
    return finish_extract_monitor_stage(
      snapshot,
      "extract monitor IK",
      extract_monitor_ik_stage_message(
      ExtractMonitorIkStageMessageRequest{
        extract_monitor_state_.legal_candidates.size(),
        ik_result.legal_count,
        ik_result.trial_count,
        elapsed_ms,
        extract_monitor_snapshot_path_}),
      message);
  }

  bool run_extract_monitor_extract_stage(std::string* message)
  {
    if (!extract_monitor_state_.seed_state || extract_monitor_state_.legal_candidates.empty()) {
      extract_monitor_last_stage_ms_ = 0.0;
      const std::string reason = "extract monitor extract: IK stage has no candidates after scene filtering";
      if (message) *message = reason;
      return fail(reason);
    }

    const auto stage_start = std::chrono::steady_clock::now();
    extract_collision_scene_epoch_.fetch_add(1, std::memory_order_acq_rel);
    extract_collision_check_count_.store(0, std::memory_order_relaxed);
    extract_collision_check_total_ns_.store(0, std::memory_order_relaxed);
    extract_collision_check_max_ns_.store(0, std::memory_order_relaxed);
    if (box_pose_rrt_profile_) box_pose_rrt_profile_->reset();
    if (box_pose_solver_profile_) box_pose_solver_profile_->reset();
    const size_t count = extract_monitor_state_.legal_candidates.size();
    size_t worker_count = 1;
    const bool direct_updown_lift =
      extract_rollout_mode_ == "top_updown_lift" ||
      extract_rollout_mode_ == "direct_updown_lift";
    if ((extract_monitor_both_top_suction() && extract_rollout_mode_ == "top_lift_legacy") ||
        direct_updown_lift) {
      extract_monitor_state_.timings.clear();
      extract_monitor_state_.timings.reserve(extract_monitor_state_.legal_candidates.size());
      worker_count = 1;
      for (size_t index = 0; index < extract_monitor_state_.legal_candidates.size(); ++index) {
        const auto& candidate = extract_monitor_state_.legal_candidates[index];
        auto state = robot_state_from_ik_candidate(*extract_monitor_state_.seed_state, candidate, joint_group_);
        auto timing = rollout_dual_updown_lift_from_state(
          state,
          extract_monitor_state_.left_box,
          extract_monitor_state_.left_box_id,
          extract_monitor_state_.right_box,
          extract_monitor_state_.right_box_id,
          index,
          candidate,
          direct_updown_lift);
        if (loaded_pose_selector_) {
          loaded_pose_selector_->fillTimingDistanceMetrics(timing);
        }
        extract_monitor_state_.timings.push_back(std::move(timing));
      }
    } else {
      auto quality_stop_condition = [this](
        size_t success_count,
        const ExtractRolloutTiming& timing)
      {
        if (extract_benchmark_extract_quality_success_quorum_ == 0 ||
            extract_benchmark_extract_quality_loaded_distance_sum_ <= 0.0) {
          return success_count >= extract_benchmark_extract_success_quorum_;
        }
        if (success_count < extract_benchmark_extract_quality_success_quorum_) {
          return false;
        }
        if (timing.loaded_pose_distance_sum <= extract_benchmark_extract_quality_loaded_distance_sum_) {
          return true;
        }
        return extract_benchmark_extract_success_quorum_ > 0 &&
               success_count >= extract_benchmark_extract_success_quorum_;
      };
      worker_count = run_extract_monitor_candidate_tasks(
        extract_monitor_state_,
        extract_benchmark_extract_workers_,
        [&](size_t index, const robot_motion::core::UpdownAwareIkCandidate& candidate) {
          const auto state = robot_state_from_ik_candidate(*extract_monitor_state_.seed_state, candidate, joint_group_);
          std::vector<nlohmann::json> rollout_records;
          auto record_step = [&](size_t step, const moveit::core::RobotState& step_state, const nlohmann::json& extra) {
            record_monitor_extract_replay_step(step, index, step_state, extra, &rollout_records);
          };
          auto timing = extract_rollout_mode_ == "moveit_rrt_legacy"
            ? rollout_dual_extract_rrt_from_state(
                state,
                extract_monitor_state_.left_box,
                extract_monitor_state_.left_box_id,
                extract_monitor_state_.right_box,
                extract_monitor_state_.right_box_id,
                index,
                candidate,
                record_step)
            : rollout_dual_extract_from_state(
                state,
                extract_monitor_state_.left_box,
                extract_monitor_state_.left_box_id,
                extract_monitor_state_.right_box,
                extract_monitor_state_.right_box_id,
                index,
                candidate,
                record_step);
          if (!rollout_records.empty()) {
            timing.rollout_records = std::move(rollout_records);
          }
          if (loaded_pose_selector_) {
            loaded_pose_selector_->fillTimingDistanceMetrics(timing);
          }
          return timing;
        },
        extract_benchmark_extract_success_quorum_,
        quality_stop_condition);
    }

    const auto summary = summarize_extract_monitor_timings(extract_monitor_state_.timings);
    std::vector<size_t> record_indices = summary.success_indices;
    if (record_indices.empty()) {
      for (size_t index = 0; index < extract_monitor_state_.timings.size(); ++index) {
        const auto& timing = extract_monitor_state_.timings[index];
        if (!timing.rollout_records.empty()) {
          record_indices.push_back(index);
        }
      }
    }
    const nlohmann::json records = extract_monitor_timing_records_json(
      ExtractMonitorTimingRecordsRequest{
        &extract_monitor_state_.timings,
        record_indices,
        extract_monitor_state_.prefix,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        dual_arm_with_updown_joint_names(),
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        static_box_obstacles_json(),
        [this](const ExtractRolloutTiming& timing) {
          if (timing.final_state) {
            return timing.final_state;
          }
          return extract_monitor_candidate_state_for_timing(extract_monitor_state_, timing);
        }});

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    const uint64_t collision_count = extract_collision_check_count_.load(std::memory_order_relaxed);
    const uint64_t collision_total_ns = extract_collision_check_total_ns_.load(std::memory_order_relaxed);
    const uint64_t collision_max_ns = extract_collision_check_max_ns_.load(std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(),
      "extract collision timing: checks=%llu total_ms=%.3f mean_us=%.3f max_us=%.3f stage_ms=%.3f",
      static_cast<unsigned long long>(collision_count),
      static_cast<double>(collision_total_ns) / 1.0e6,
      collision_count == 0 ? 0.0 : static_cast<double>(collision_total_ns) / collision_count / 1.0e3,
      static_cast<double>(collision_max_ns) / 1.0e3,
      elapsed_ms);
    if (box_pose_rrt_profile_) {
      const uint64_t nodes = box_pose_rrt_profile_->node_evaluations.load(std::memory_order_relaxed);
      const uint64_t target_ns = box_pose_rrt_profile_->target_pose_ns.load(std::memory_order_relaxed);
      const uint64_t ik_ns = box_pose_rrt_profile_->ik_ns.load(std::memory_order_relaxed);
      const uint64_t clear_ns = box_pose_rrt_profile_->clear_ns.load(std::memory_order_relaxed);
      const uint64_t rrt_ns = box_pose_rrt_profile_->rrt_plan_ns.load(std::memory_order_relaxed);
      const uint64_t pair_ns = box_pose_rrt_profile_->pair_validation_ns.load(std::memory_order_relaxed);
      const uint64_t clear_non_fcl_ns = clear_ns > collision_total_ns ? clear_ns - collision_total_ns : 0;
      RCLCPP_INFO(
        get_logger(),
        "extract profile: nodes=%llu target_ms=%.3f ik_ms=%.3f clear_ms=%.3f fcl_ms=%.3f clear_non_fcl_ms=%.3f rrt_total_cpu_ms=%.3f pair_validation_cpu_ms=%.3f wall_ms=%.3f",
        static_cast<unsigned long long>(nodes),
        static_cast<double>(target_ns) / 1.0e6,
        static_cast<double>(ik_ns) / 1.0e6,
        static_cast<double>(clear_ns) / 1.0e6,
        static_cast<double>(collision_total_ns) / 1.0e6,
        static_cast<double>(clear_non_fcl_ns) / 1.0e6,
        static_cast<double>(rrt_ns) / 1.0e6,
        static_cast<double>(pair_ns) / 1.0e6,
        elapsed_ms);
    }
    if (box_pose_solver_profile_) {
      RCLCPP_INFO(
        get_logger(),
        "extract solver profile: state_copy_ms=%.3f pre_analytic_state_ms=%.3f analytic_core_ms=%.3f post_state_update_ms=%.3f validation_ms=%.3f",
        static_cast<double>(box_pose_solver_profile_->state_copy_ns.load(std::memory_order_relaxed)) / 1.0e6,
        static_cast<double>(box_pose_solver_profile_->pre_analytic_state_ns.load(std::memory_order_relaxed)) / 1.0e6,
        static_cast<double>(box_pose_solver_profile_->analytic_core_ns.load(std::memory_order_relaxed)) / 1.0e6,
        static_cast<double>(box_pose_solver_profile_->post_state_update_ns.load(std::memory_order_relaxed)) / 1.0e6,
        static_cast<double>(box_pose_solver_profile_->validation_ns.load(std::memory_order_relaxed)) / 1.0e6);
    }
    nlohmann::json snapshot = extract_monitor_extract_snapshot(
      ExtractMonitorExtractSnapshotRequest{
        elapsed_ms,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        box_front_x_,
        scene_y_shift_,
        count,
        summary.success_count,
        worker_count,
        summary.failure_counts,
        records});
    snapshot["extract_quality_success_quorum"] = extract_benchmark_extract_quality_success_quorum_;
    snapshot["extract_quality_loaded_distance_sum"] =
      extract_benchmark_extract_quality_loaded_distance_sum_;
    const bool snapshot_written = finish_extract_monitor_stage(
      snapshot,
      "extract monitor extract",
      extract_monitor_extract_stage_message(
      ExtractMonitorExtractStageMessageRequest{
        summary.success_count,
        count,
        worker_count,
        elapsed_ms,
        extract_monitor_snapshot_path_}),
      message);
    if (!snapshot_written) {
      return false;
    }
    if (summary.success_count == 0) {
      const auto dominant = std::max_element(
        summary.failure_counts.begin(), summary.failure_counts.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
      std::ostringstream reason;
      reason << "extract monitor extract: no successful rollout among " << count << " candidates";
      if (dominant != summary.failure_counts.end()) {
        reason << "; dominant=" << dominant->first << " count=" << dominant->second;
      }
      if (message) *message = reason.str();
      return fail(reason.str());
    }
    return true;
  }

  bool run_extract_monitor_loaded_stage(std::string* message)
  {
    if (extract_monitor_state_.timings.empty()) {
      extract_monitor_last_stage_ms_ = 0.0;
      return fail("extract monitor loaded: extract stage has no timings");
    }

    const auto stage_start = std::chrono::steady_clock::now();
    auto options = loaded_pose_batch_plan_options();
    options.enabled = true;
    const std::vector<AttachedBoxSpec> boxes{
      extract_monitor_state_.left_box,
      extract_monitor_state_.right_box,
    };
    LoadedPoseBatchPlanResult batch = loaded_pose_planner_
      ? loaded_pose_planner_->planBatch(
          extract_monitor_state_.prefix + "/monitor_loaded",
          extract_monitor_state_.timings,
          boxes,
          options)
      : LoadedPoseBatchPlanResult{};

    const auto summary = summarize_loaded_plan_timings(extract_monitor_state_.timings, batch.plan_indices);
    extract_monitor_state_.loaded_plan_batch_wall_ms = batch.wall_ms;
    extract_monitor_state_.loaded_plan_candidate_count = batch.plan_indices.size();
    extract_monitor_state_.loaded_plan_attempted_count = summary.attempted_count;
    extract_monitor_state_.loaded_plan_success_count = summary.success_count;
    extract_monitor_state_.loaded_parallel_workers = options.parallel_workers;
    extract_monitor_state_.loaded_candidate_limit = options.candidate_limit;
    extract_monitor_state_.loaded_plan_failure_counts = summary.failure_counts;
    const nlohmann::json records = extract_monitor_timing_records_json(
      ExtractMonitorTimingRecordsRequest{
        &extract_monitor_state_.timings,
        summary.attempted_indices,
        extract_monitor_state_.prefix,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        dual_arm_with_updown_joint_names(),
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        static_box_obstacles_json(),
        [this](const ExtractRolloutTiming& timing) -> moveit::core::RobotStatePtr {
          if (!timing.loaded_plan_attempted || !timing.final_state || !loaded_pose_selector_) {
            return {};
          }
          if (timing.loaded_goal_state) {
            return std::make_shared<moveit::core::RobotState>(*timing.loaded_goal_state);
          }
          return std::make_shared<moveit::core::RobotState>(
            loaded_pose_selector_->makeGoalState(*timing.final_state));
        }});

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    const nlohmann::json snapshot = extract_monitor_loaded_snapshot(
      ExtractMonitorLoadedSnapshotRequest{
        elapsed_ms,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        box_front_x_,
        scene_y_shift_,
        batch.plan_indices.size(),
        summary.attempted_count,
        summary.success_count,
        batch.wall_ms,
        options.parallel_workers,
        options.candidate_limit,
        summary.failure_counts,
        records});
    const bool snapshot_written = finish_extract_monitor_stage(
      snapshot,
      "extract monitor loaded",
      extract_monitor_loaded_stage_message(
      ExtractMonitorLoadedStageMessageRequest{
        summary.success_count,
        summary.attempted_count,
        batch.plan_indices.size(),
        elapsed_ms,
        extract_monitor_snapshot_path_}),
      message);
    if (!snapshot_written) {
      return false;
    }
    if (summary.success_count == 0) {
      const auto dominant = std::max_element(
        summary.failure_counts.begin(), summary.failure_counts.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
      std::ostringstream reason;
      reason << "extract monitor loaded: no successful loaded plan among "
             << summary.attempted_count << " attempts";
      if (dominant != summary.failure_counts.end()) {
        reason << "; dominant=" << dominant->first << " count=" << dominant->second;
      }
      if (message) *message = reason.str();
      return fail(reason.str());
    }
    return true;
  }

  bool extract_monitor_pre_attach_transition_is_smooth(const ExtractRolloutTiming& timing)
  {
    if (!extract_monitor_state_.loaded_start_state ||
        extract_monitor_state_.candidate_states.empty()) {
      return false;
    }
    const auto ik_state = extract_monitor_candidate_state_for_timing(extract_monitor_state_, timing);
    if (!ik_state) {
      return false;
    }
    std::string pre_contact_reason;
    const auto pre_contact_state = build_extract_monitor_pre_contact_state(
      *extract_monitor_state_.loaded_start_state, *ik_state, &pre_contact_reason);
    const auto transition_clear = [this](
      const moveit::core::RobotState& start,
      const moveit::core::RobotState& goal) {
      auto plan = make_interpolated_joint_plan(start, goal, 1.0);
      std::string reason;
      return planned_trajectory_clear_in_full_scene(plan, start, {}, &reason);
    };
    if (pre_contact_state) {
      return transition_clear(*extract_monitor_state_.loaded_start_state, *pre_contact_state) &&
        transition_clear(*pre_contact_state, *ik_state);
    }

    auto plan = make_interpolated_joint_plan(*extract_monitor_state_.loaded_start_state, *ik_state, 1.0);
    std::string reason;
    return planned_trajectory_clear_in_full_scene(plan, *extract_monitor_state_.loaded_start_state, {}, &reason);
  }

  moveit::core::RobotStatePtr build_extract_monitor_pre_contact_state(
    const moveit::core::RobotState& loaded_start_state,
    const moveit::core::RobotState& ik_goal_state,
    std::string* reason) const
  {
    (void)loaded_start_state;
    if (!pre_contact_candidate_solver_) {
      if (reason) *reason = "pre_contact_solver_not_initialized";
      return nullptr;
    }
    constexpr double pre_contact_offset = 0.05;
    auto pre_contact_state = std::make_shared<moveit::core::RobotState>(ik_goal_state);
    const double fixed_updown = current_updown(ik_goal_state);

    const auto solve_side = [&](const std::string& side, const std::string& tip, bool top_suction) {
      const Eigen::Isometry3d ik_tip = ik_goal_state.getGlobalLinkTransform(tip);
      Eigen::Isometry3d pre_contact_tip = ik_tip;
      pre_contact_tip.translation() -= ik_tip.linear() * Eigen::Vector3d::UnitZ() * pre_contact_offset;
      Eigen::Quaterniond orientation(pre_contact_tip.linear());
      orientation.normalize();

      ExtractCandidateSolveRequest request;
      request.side = side;
      request.current_state = &ik_goal_state;
      request.target_pose = make_pose(
        pre_contact_tip.translation().x(),
        pre_contact_tip.translation().y(),
        pre_contact_tip.translation().z(),
        orientation);
      request.step_index = 0;
      request.candidate_index = 0;
      request.min_allowed_tip_z = -std::numeric_limits<double>::infinity();
      request.fixed_updown = fixed_updown;
      request.min_tool_normal_z = -1.1;
      request.top_suction = top_suction;

      ExtractCandidate candidate;
      if (!pre_contact_candidate_solver_->solve(request, &candidate) || !candidate.state) {
        if (reason) {
          *reason = candidate.rejection_reason.empty()
            ? side + "_pre_contact_ik_failed"
            : candidate.rejection_reason;
        }
        return false;
      }

      const auto* group = side == "left" ? left_arm_group_ : right_arm_group_;
      if (!group) {
        if (reason) *reason = side + "_pre_contact_missing_group";
        return false;
      }
      for (const auto& name : group->getVariableNames()) {
        pre_contact_state->setVariablePosition(name, candidate.state->getVariablePosition(name));
      }
      return true;
    };

    if (!solve_side("left", left_tip_, extract_monitor_left_top_suction_)) {
      return nullptr;
    }
    if (!solve_side("right", right_tip_, extract_monitor_right_top_suction_)) {
      return nullptr;
    }

    pre_contact_state->setVariablePosition("updown", fixed_updown);
    pre_contact_state->enforceBounds(joint_group_);
    pre_contact_state->update(true);
    if (reason) reason->clear();
    return pre_contact_state;
  }

  ExtractRolloutTiming* select_extract_monitor_final_timing()
  {
    return alfa_robot::motion::select_extract_monitor_final_timing(
      extract_monitor_state_.timings,
      [this](const ExtractRolloutTiming& timing) {
        return extract_monitor_pre_attach_transition_is_smooth(timing);
      });
  }

  void ensure_selected_extract_replay_records(ExtractRolloutTiming& selected)
  {
    if (!selected.rollout_records.empty()) {
      return;
    }
    const auto* candidate = extract_monitor_candidate_for_timing(extract_monitor_state_, selected);
    auto start_state = extract_monitor_candidate_state_for_timing(extract_monitor_state_, selected);
    if (!candidate || !start_state) {
      return;
    }

    std::vector<nlohmann::json> rollout_records;
    auto record_step = [&](size_t step, const moveit::core::RobotState& state, const nlohmann::json& extra) {
      record_monitor_extract_replay_step(step, selected.candidate_order, state, extra, &rollout_records);
    };
    auto replay_timing = rollout_dual_extract_from_state(
      *start_state,
      extract_monitor_state_.left_box,
      extract_monitor_state_.left_box_id,
      extract_monitor_state_.right_box,
      extract_monitor_state_.right_box_id,
      selected.candidate_order,
      *candidate,
      record_step);
    if (replay_timing.success) {
      selected.rollout_records = std::move(rollout_records);
    }
  }

  ExtractMonitorTransitionPlanner extract_monitor_transition_planner(
    const std::vector<AttachedBoxSpec>& carried_boxes = {},
    const std::string& stage_name = "extract_monitor_transition")
  {
    ExtractMonitorTransitionPlanner transition_planner;
    transition_planner.make_interpolated_plan =
      [this](const auto& start, const auto& goal, double duration_s) {
        return make_interpolated_joint_plan(start, goal, duration_s);
      };
    transition_planner.densify_plan = [this](const auto& plan) {
      return densify_joint_plan(plan, 5.0 * M_PI / 180.0, 0.01);
    };
    transition_planner.validate_plan = [this, carried_boxes](
      const auto& plan, const auto& start, std::string* reason) {
      return planned_trajectory_clear_in_full_scene(plan, start, carried_boxes, reason);
    };
    transition_planner.direct_plan = [this, carried_boxes, stage_name](
      const auto& start, const auto& goal, auto* plan, std::string* reason) {
      if (!carried_boxes.empty()) {
        return plan_loaded_pose_with_direct_pipeline(
          stage_name, start, goal, carried_boxes, plan, reason);
      }
      return plan_joint_space_with_direct_pipeline(start, goal, plan, reason);
    };
    transition_planner.shortcut_plan = [this, carried_boxes](
      const auto& plan, const auto& start, std::string* reason) {
      return shortcut_joint_plan(plan, start, carried_boxes, reason);
    };
    return transition_planner;
  }

  moveit::core::RobotState make_extract_monitor_place_goal_state(
    const moveit::core::RobotState& loaded_state) const
  {
    moveit::core::RobotState goal_state(loaded_state);
    const std::array<std::string, 6> left_names{
      "left_joint1", "left_joint2", "left_joint3",
      "left_joint4", "left_joint5", "left_joint6"};
    const std::array<std::string, 6> right_names{
      "right_joint1", "right_joint2", "right_joint3",
      "right_joint4", "right_joint5", "right_joint6"};
    for (size_t index = 0; index < left_names.size(); ++index) {
      goal_state.setVariablePosition(left_names[index], extract_monitor_place_left_arm_[index]);
      goal_state.setVariablePosition(right_names[index], extract_monitor_place_right_arm_[index]);
    }
    goal_state.setVariablePosition("updown", extract_monitor_place_updown_);
    goal_state.update();
    return goal_state;
  }

  bool append_extract_monitor_place_cycle(
    const ExtractRolloutTiming& selected,
    nlohmann::json* replay_stages,
    nlohmann::json* metrics,
    std::string* reason)
  {
    if (!extract_monitor_place_cycle_enabled_) {
      return true;
    }
    if (!replay_stages || !replay_stages->is_array()) {
      if (reason) *reason = "place_cycle_replay_stages_missing";
      return false;
    }
    if (!selected.loaded_goal_state) {
      if (reason) *reason = "place_cycle_loaded_goal_state_missing";
      return false;
    }

    const moveit::core::RobotState loaded_state(*selected.loaded_goal_state);
    const moveit::core::RobotState place_state =
      make_extract_monitor_place_goal_state(loaded_state);
    const std::vector<AttachedBoxSpec> carried_boxes{
      extract_monitor_state_.left_box, extract_monitor_state_.right_box};
    const auto target_names = dual_arm_with_updown_joint_names();
    const auto static_obstacles = static_box_obstacles_json();

    const auto outbound_start = std::chrono::steady_clock::now();
    const double place_transition_updown = extract_monitor_both_top_suction()
      ? std::min(extract_monitor_place_updown_, extract_monitor_place_transition_updown_)
      : extract_monitor_place_updown_;
    moveit::core::RobotState place_transition_state(loaded_state);
    place_transition_state.setVariablePosition("updown", place_transition_updown);
    place_transition_state.enforceBounds();
    place_transition_state.update(true);
    moveit::core::RobotState place_arm_state(place_state);
    place_arm_state.setVariablePosition("updown", place_transition_updown);
    place_arm_state.enforceBounds();
    place_arm_state.update(true);

    const bool needs_place_height_transition = std::abs(
      loaded_state.getVariablePosition("updown") -
      place_transition_state.getVariablePosition("updown")) > 1e-6;
    alfa_robot::motion::ExtractMonitorTransitionPlanResult height_transition;
    if (needs_place_height_transition) {
      height_transition = extract_monitor_transition_planner(
        carried_boxes, extract_monitor_state_.prefix + "/selected_loaded_to_place_height").plan(
        loaded_state, place_transition_state);
      if (!height_transition.valid) {
        if (reason) {
          *reason = "place_cycle_loaded_to_place_height_failed: " +
            height_transition.failure_reason;
        }
        return false;
      }
      replay_stages->push_back(alfa_robot::motion::extract_monitor_stage_json(
        extract_monitor_state_.prefix + "/selected_loaded_to_place_height",
        height_transition.plan,
        loaded_state,
        place_transition_state,
        target_names,
        carried_boxes,
        static_obstacles,
        {
          {"stage_kind", "monitor_selected_loaded_to_place_height_replay"},
          {"valid", true},
          {"method", height_transition.method},
          {"release_after_stage", false},
          {"place_transition_updown", place_transition_updown},
        }));
    }

    const auto outbound = extract_monitor_transition_planner(
      carried_boxes, extract_monitor_state_.prefix + "/selected_loaded_to_place_arms").plan(
      place_transition_state, place_arm_state);
    if (!outbound.valid) {
      if (reason) {
        *reason = "place_cycle_loaded_to_place_arms_failed: " + outbound.failure_reason;
      }
      return false;
    }
    replay_stages->push_back(alfa_robot::motion::extract_monitor_stage_json(
      extract_monitor_state_.prefix + "/selected_loaded_to_place_arms",
      outbound.plan,
      place_transition_state,
      place_arm_state,
      target_names,
      carried_boxes,
      static_obstacles,
      {
        {"stage_kind", "monitor_selected_loaded_to_place_replay"},
        {"valid", true},
        {"method", outbound.method},
        {"release_after_stage", false},
        {"place_transition_updown", place_transition_updown},
        {"place_updown", extract_monitor_place_updown_},
      }));

    const bool needs_final_place_height_transition = std::abs(
      place_arm_state.getVariablePosition("updown") -
      place_state.getVariablePosition("updown")) > 1e-6;
    alfa_robot::motion::ExtractMonitorTransitionPlanResult final_height_transition;
    if (needs_final_place_height_transition) {
      final_height_transition = extract_monitor_transition_planner(
        carried_boxes, extract_monitor_state_.prefix + "/selected_place_final_height").plan(
        place_arm_state, place_state);
      if (!final_height_transition.valid) {
        if (reason) {
          *reason = "place_cycle_final_place_height_failed: " +
            final_height_transition.failure_reason;
        }
        return false;
      }
      replay_stages->push_back(alfa_robot::motion::extract_monitor_stage_json(
        extract_monitor_state_.prefix + "/selected_place_final_height",
        final_height_transition.plan,
        place_arm_state,
        place_state,
        target_names,
        carried_boxes,
        static_obstacles,
        {
          {"stage_kind", "monitor_selected_place_final_height_replay"},
          {"valid", true},
          {"method", final_height_transition.method},
          {"release_after_stage", true},
          {"place_transition_updown", place_transition_updown},
          {"place_updown", extract_monitor_place_updown_},
        }));
    } else {
      (*replay_stages)[replay_stages->size() - 1]["extra"]["release_after_stage"] = true;
    }
    const double outbound_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - outbound_start).count();

    moveit::core::RobotState return_loaded_state(loaded_state);
    return_loaded_state.setVariablePosition("updown", fixed_updown_);
    return_loaded_state.enforceBounds();
    return_loaded_state.update(true);
    const auto return_start = std::chrono::steady_clock::now();
    const auto return_plan = extract_monitor_transition_planner(
      {}, extract_monitor_state_.prefix + "/selected_place_to_loaded").plan(
      place_state, return_loaded_state);
    const double return_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - return_start).count();
    if (!return_plan.valid) {
      if (reason) {
        *reason = "place_cycle_place_to_loaded_failed: " + return_plan.failure_reason;
      }
      return false;
    }
    replay_stages->push_back(alfa_robot::motion::extract_monitor_stage_json(
      extract_monitor_state_.prefix + "/selected_place_to_loaded",
      return_plan.plan,
      place_state,
      return_loaded_state,
      target_names,
      {},
      static_obstacles,
      {
        {"stage_kind", "monitor_selected_place_to_loaded_replay"},
        {"valid", true},
        {"method", return_plan.method},
        {"transition_ms", return_ms},
        {"boxes_released", true},
        {"loaded_updown", fixed_updown_},
      }));
    if (metrics) {
      *metrics = {
        {"enabled", true},
        {"loaded_to_place_ms", outbound_ms},
        {"loaded_to_place_method", needs_place_height_transition
          ? height_transition.method + "+" + outbound.method +
            (needs_final_place_height_transition ? "+" + final_height_transition.method : "")
          : outbound.method},
        {"place_to_loaded_ms", return_ms},
        {"place_to_loaded_method", return_plan.method},
        {"place_updown", extract_monitor_place_updown_},
        {"place_transition_updown", place_transition_updown},
      };
    }
    return true;
  }

  nlohmann::json build_final_replay_stages(
    ExtractRolloutTiming& selected,
    const moveit::core::RobotState& ik_goal_state)
  {
    ExtractMonitorReplayBuilder builder;
    builder.transition_planner = extract_monitor_transition_planner();
    builder.ensure_extract_replay = [this](ExtractRolloutTiming& timing) {
      ensure_selected_extract_replay_records(timing);
    };
    builder.build_pre_contact_state =
      [this](
        const moveit::core::RobotState& loaded_start_state,
        const moveit::core::RobotState& ik_goal_state,
        std::string* reason) {
        return build_extract_monitor_pre_contact_state(loaded_start_state, ik_goal_state, reason);
      };

    return builder.build(selected, make_extract_monitor_replay_request(
      extract_monitor_state_,
      dual_arm_with_updown_joint_names(),
      static_box_obstacles_json(),
      std::make_shared<moveit::core::RobotState>(ik_goal_state)));
  }

  bool run_extract_monitor_final_stage(std::string* message)
  {
    const auto stage_start = std::chrono::steady_clock::now();
    ExtractRolloutTiming* selected = select_extract_monitor_final_timing();
    if (!selected || !selected->final_state || !loaded_pose_selector_) {
      const std::string reason = "extract monitor final: no loaded-plan success to select";
      if (message) *message = reason;
      return fail(reason);
    }

    moveit::core::RobotState goal_state = selected->loaded_goal_state
      ? *selected->loaded_goal_state
      : loaded_pose_selector_->makeGoalState(*selected->final_state);
    const auto ik_candidate_state = extract_monitor_candidate_state_for_timing(extract_monitor_state_, *selected);
    moveit::core::RobotState ik_goal_state = ik_candidate_state ? *ik_candidate_state : *selected->final_state;

    nlohmann::json replay_stages = extract_monitor_build_final_replay_
      ? build_final_replay_stages(*selected, ik_goal_state)
      : nlohmann::json::array();
    nlohmann::json place_cycle_metrics = {
      {"enabled", extract_monitor_place_cycle_enabled_}
    };
    if (extract_monitor_place_cycle_enabled_) {
      if (!extract_monitor_build_final_replay_) {
        const std::string reason = "extract monitor place cycle requires final replay";
        if (message) *message = reason;
        return fail(reason);
      }
      std::string place_cycle_reason;
      if (!append_extract_monitor_place_cycle(
          *selected, &replay_stages, &place_cycle_metrics, &place_cycle_reason)) {
        const std::string failure = "extract monitor final: " + place_cycle_reason;
        if (message) *message = failure;
        return fail(failure);
      }
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stage_start).count();
    extract_monitor_last_stage_ms_ = elapsed_ms;
    const std::vector<ExtractRolloutTiming> selected_records{*selected};
    const nlohmann::json final_records = extract_monitor_timing_records_json(
      ExtractMonitorTimingRecordsRequest{
        &selected_records,
        {0},
        extract_monitor_state_.prefix,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        dual_arm_with_updown_joint_names(),
        {extract_monitor_state_.left_box, extract_monitor_state_.right_box},
        static_box_obstacles_json(),
        [&goal_state](const ExtractRolloutTiming&) {
          return std::make_shared<moveit::core::RobotState>(goal_state);
        }});
    const nlohmann::json snapshot = extract_monitor_final_snapshot(
      ExtractMonitorFinalSnapshotRequest{
        elapsed_ms,
        extract_monitor_state_.left_box_id,
        extract_monitor_state_.right_box_id,
        box_front_x_,
        scene_y_shift_,
        final_records.empty() ? nlohmann::json::object() : final_records[0],
        replay_stages});
    nlohmann::json enriched_snapshot = snapshot;
    enriched_snapshot["loaded_plan_batch_wall_ms"] = extract_monitor_state_.loaded_plan_batch_wall_ms;
    enriched_snapshot["loaded_plan_candidate_count"] = extract_monitor_state_.loaded_plan_candidate_count;
    enriched_snapshot["loaded_plan_attempted_count"] = extract_monitor_state_.loaded_plan_attempted_count;
    enriched_snapshot["loaded_plan_success_count"] = extract_monitor_state_.loaded_plan_success_count;
    enriched_snapshot["loaded_parallel_workers"] = extract_monitor_state_.loaded_parallel_workers;
    enriched_snapshot["loaded_candidate_limit"] = extract_monitor_state_.loaded_candidate_limit;
    enriched_snapshot["loaded_plan_failure_counts"] =
      failure_counts_json(extract_monitor_state_.loaded_plan_failure_counts);
    enriched_snapshot["place_cycle"] = place_cycle_metrics;
    return finish_extract_monitor_stage(
      enriched_snapshot,
      "extract monitor final",
      extract_monitor_final_stage_message(
      ExtractMonitorFinalStageMessageRequest{
        selected,
        elapsed_ms,
        extract_monitor_snapshot_path_}),
      message);
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

    const auto left_pose = make_outer_box_grasp_pose(left_it->second, false, true);
    const auto right_pose = make_outer_box_grasp_pose(right_it->second, false, false);

    if (extract_demo_direct_grasp_start_) {
      auto seed_state = std::make_shared<moveit::core::RobotState>(robot_model_);
      seed_state->setToDefaultValues();
      for (size_t i = 0; i < left_pregrasp_arm_.size(); ++i) {
        seed_state->setVariablePosition("left_joint" + std::to_string(i + 1), left_pregrasp_arm_[i]);
      }
      for (size_t i = 0; i < right_pregrasp_arm_.size(); ++i) {
        seed_state->setVariablePosition("right_joint" + std::to_string(i + 1), right_pregrasp_arm_[i]);
      }
      seed_state->setVariablePosition("updown", extract_grasp_ik_home_updown_);
      if (is_robot_variable("turn")) {
        seed_state->setVariablePosition("turn", extract_monitor_turn_);
      }
      seed_state->enforceBounds(joint_group_);
      seed_state->update();

      moveit::core::RobotState grasp_state(*seed_state);
      nlohmann::json grasp_extra;
      robot_motion::core::UpdownAwareIkResult direct_ik_result;
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
  std::string execution_action_name_ = "/dual_arm_trajectory_controller/follow_joint_trajectory";
  double execution_action_wait_timeout_s_ = 5.0;
  double execution_result_timeout_s_ = 0.0;
  bool execution_include_turn_ = true;
  bool execution_allow_hold_missing_target_joints_ = true;
  bool execution_reject_unmapped_planned_joints_ = true;
  bool reject_ik_collisions_ = false;
  bool check_goal_collision_ = false;
  bool prefer_commanded_state_ = true;
  bool include_top_suction_ = true;
  size_t ik_analytic_root_samples_ = 360;
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
  bool container_track_vehicle_drift_ = false;
  std::string vehicle_drift_global_frame_ = "map";
  double vehicle_drift_translation_threshold_m_ = 0.05;
  double vehicle_drift_rotation_threshold_rad_ = 0.02;
  double container_length_ = 4.0;
  double container_width_ = 1.8;
  double container_height_ = 2.4;
  double container_center_x_ = 0.8;
  double container_center_y_ = 0.0;
  bool container_pose_dynamic_ = false;
  double container_pose_map_x_ = 0.8;
  double container_pose_map_y_ = 0.0;
  double container_pose_map_yaw_ = 0.0;
  std::optional<ContainerRelativePose> container_pose_dynamic_cache_;
  double container_floor_z_ = 0.0;
  double container_wall_thickness_ = 0.02;
  bool enable_attached_box_collision_ = true;
  double carried_box_depth_ = 0.3;
  double carried_box_width_ = 0.5;
  double carried_box_height_ = 0.4;
  double carried_box_grasp_lateral_offset_ = 0.0;
  double attached_box_collision_padding_ = -0.002;
  bool enforce_loaded_plan_aabb_clearance_ = false;
  bool enforce_loaded_static_box_wall_aabb_clearance_ = true;
  bool enable_static_box_obstacles_ = true;
  double static_box_obstacle_inset_ = 0.002;
  int extract_demo_left_box_id_ = 1;
  int extract_demo_right_box_id_ = 3;
  std::vector<std::pair<int, int>> extract_demo_pair_sequence_{
    {1, 3}, {4, 6}, {7, 9}, {10, 12}, {13, 15}};
  bool extract_demo_all_rows_ = false;
  bool extract_monitor_top_suction_ = false;
  bool extract_monitor_left_top_suction_ = false;
  bool extract_monitor_right_top_suction_ = false;
  bool extract_monitor_use_explicit_targets_ = false;
  geometry_msgs::msg::Pose extract_monitor_left_target_;
  geometry_msgs::msg::Pose extract_monitor_right_target_;
  bool extract_monitor_require_left_detached_ = true;
  bool extract_monitor_require_right_detached_ = true;
  size_t extract_monitor_left_front_clearance_levels_ = 1;
  size_t extract_monitor_right_front_clearance_levels_ = 1;
  bool extract_monitor_top_height_mismatch_ = false;
  double extract_monitor_left_retreat_priority_ = 1.0;
  double extract_monitor_left_lift_priority_ = 1.0;
  double extract_monitor_left_pitch_priority_ = 1.0;
  double extract_monitor_right_retreat_priority_ = 1.0;
  double extract_monitor_right_lift_priority_ = 1.0;
  double extract_monitor_right_pitch_priority_ = 1.0;
  double extract_step_x_ = 0.03;
  double extract_max_x_ = 0.36;
  std::vector<double> extract_lift_candidates_;
  std::vector<double> extract_pitch_candidates_deg_;
  double extract_neighbor_margin_ = 0.02;
  bool extract_fail_fast_ = false;
  size_t extract_success_extra_steps_ = 3;
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
  double extract_monitor_turn_ = 0.0;
  bool extract_benchmark_all_legal_ik_ = false;
  bool extract_benchmark_dual_arm_ = false;
  bool extract_benchmark_dual_async_ = false;
  std::string extract_benchmark_csv_path_;
  bool extract_benchmark_record_rollouts_ = false;
  size_t extract_benchmark_candidate_limit_ = 0;
  size_t extract_benchmark_extract_workers_ = 1;
  size_t extract_benchmark_extract_success_quorum_ = 3;
  size_t extract_benchmark_extract_quality_success_quorum_ = 0;
  double extract_benchmark_extract_quality_loaded_distance_sum_ = 0.0;
  bool extract_ik_dedup_enabled_ = true;
  double extract_ik_dedup_joint_threshold_ = 1.0 * M_PI / 180.0;
  double extract_ik_dedup_h_threshold_ = 0.005;
  bool extract_ik_stratified_limit_enabled_ = false;
  double extract_ik_stratified_h_bucket_ = 0.05;
  size_t extract_ik_stratified_top_score_count_ = 12;
  size_t extract_ik_candidate_reserve_limit_ = 64;
  bool extract_ik_candidate_reserve_stratified_ = true;
  size_t extract_ik_candidate_reserve_interleave_stride_ = 4;
  double extract_ik_loaded_distance_order_weight_ = 0.0;
  bool extract_monitor_capture_raw_ik_ = false;
  bool extract_monitor_build_final_replay_ = true;
  bool extract_monitor_place_cycle_enabled_ = false;
  double extract_monitor_place_updown_ = 0.10;
  double extract_monitor_place_transition_updown_ = 0.10;
  std::vector<double> extract_monitor_place_left_arm_;
  std::vector<double> extract_monitor_place_right_arm_;
  std::string extract_rollout_mode_ = "greedy";
  double extract_top_updown_lift_distance_ = 0.40;
  bool extract_rrt_rollout_enabled_ = false;
  size_t extract_box_pose_rrt_max_iterations_ = 160;
  size_t extract_box_pose_rrt_paths_per_arm_ = 8;
  size_t extract_box_pose_rrt_path_pair_limit_ = 64;
  double extract_box_pose_rrt_max_retreat_ = 0.55;
  double extract_box_pose_rrt_max_lift_ = 0.55;
  double extract_box_pose_rrt_separation_margin_ = 0.03;
  size_t extract_box_pose_rrt_analytic_root_samples_ = 12;
  bool extract_box_pose_rrt_diagnostics_ = false;
  bool extract_box_pose_rrt_edge_scene_collision_ = true;
  size_t extract_box_pose_rrt_parent_candidates_ = 8;
  size_t extract_box_pose_rrt_parent_diverse_candidates_ = 0;
  double extract_box_pose_rrt_parent_endpoint_score_weight_ = 0.05;
  double extract_box_pose_rrt_parent_node_score_weight_ = 0.0;
  double extract_box_pose_rrt_parent_density_weight_ = 0.0;
  double extract_box_pose_rrt_max_lateral_ = 0.0;
  double extract_box_pose_rrt_step_lateral_ = 0.02;
  bool extract_box_pose_rrt_front_free_motion_ = true;
  bool extract_box_pose_rrt_front_goal_requires_max_pitch_ = false;
  bool extract_box_pose_rrt_best_first_fallback_ = true;
  bool extract_box_pose_rrt_best_first_first_ = false;
  bool extract_box_pose_rrt_top_best_first_first_ = false;
  double extract_box_pose_rrt_top_goal_min_pitch_deg_ = 5.0;
  size_t extract_box_pose_rrt_best_first_max_expansions_ = 800;
  double extract_box_pose_rrt_best_first_heuristic_weight_ = 1.0;
  std::string extract_rrt_planning_group_ = "dual_arm";
  double extract_rrt_planning_time_ = 0.35;
  int extract_rrt_planning_attempts_ = 1;
  size_t extract_rrt_endpoint_per_arm_limit_ = 8;
  size_t extract_rrt_goal_limit_ = 8;
  bool extract_benchmark_plan_loaded_after_success_ = false;
  std::string extract_loaded_planning_group_ = "dual_arm_with_base";
  std::string extract_loaded_planner_id_;
  double extract_loaded_planning_time_ = 1.0;
  int extract_loaded_planning_attempts_ = 8;
  bool extract_loaded_use_direct_pipeline_ = false;
  std::string extract_loaded_planning_mode_ = "rrt";
  size_t extract_loaded_parallel_workers_ = 1;
  size_t extract_loaded_candidate_limit_ = 0;
  bool extract_loaded_sort_by_pose_distance_ = false;
  bool extract_loaded_stop_on_first_success_ = false;
  double extract_loaded_target_updown_ = 0.3;
  bool extract_loaded_preserve_lower_updown_ = false;
  bool extract_loaded_lateral_shift_enabled_ = false;
  double extract_loaded_lateral_shift_distance_ = 0.4;
  double extract_loaded_lateral_shift_step_ = 0.0;
  int extract_loaded_lateral_shift_column_ = 3;
  int extract_loaded_pre_lower_left_box_id_ = 0;
  int extract_loaded_pre_lower_right_box_id_ = 0;
  double extract_loaded_pre_lower_updown_delta_ = 0.0;
  bool record_tip_error_ik_candidates_ = false;
  size_t record_tip_error_ik_candidate_limit_ = 80;
  std::string record_jsonl_path_;
  bool record_trajectories_ = true;
  std::string extract_monitor_snapshot_path_;
  ExtractMonitorSnapshotWriter extract_monitor_snapshot_writer_;
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
  robot_motion::core::UpdownAwareIkConfig ik_config_;
  LoadedPoseSelectorConfig loaded_pose_selector_config_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> loaded_move_group_;
  std::unique_ptr<LoadedPoseSelector> loaded_pose_selector_;
  std::unique_ptr<LoadedPosePlanner> loaded_pose_planner_;
  std::unique_ptr<ExtractMotionPlanner> extract_motion_planner_;
  std::unique_ptr<ExtractCandidateScorer> extract_candidate_scorer_;
  std::unique_ptr<ExtractCandidateSolver> extract_candidate_solver_;
  std::unique_ptr<ExtractCandidateSolver> box_pose_rrt_candidate_solver_;
  std::unique_ptr<ExtractCandidateSolver> pre_contact_candidate_solver_;
  std::unique_ptr<ExtractRolloutPlanner> extract_rollout_planner_;
  std::unique_ptr<BoxPoseRrtExtractPlanner> box_pose_rrt_extract_planner_;
  std::unique_ptr<IkCandidateSelector> ik_candidate_selector_;
  std::unique_ptr<MotionSceneAdapter> scene_adapter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr vehicle_drift_check_timer_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  mutable std::atomic<uint64_t> extract_collision_scene_epoch_{1};
  planning_pipeline::PlanningPipelinePtr loaded_planning_pipeline_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  const moveit::core::JointModelGroup* left_arm_group_ = nullptr;
  const moveit::core::JointModelGroup* right_arm_group_ = nullptr;
  moveit::core::RobotStatePtr last_commanded_state_;
  std::unique_ptr<OptimizedDualIkSolver> optimized_dual_ik_solver_;
  std::unique_ptr<MotionFlowRecorder> recorder_;

  ExtractMonitorController extract_monitor_controller_;
  ExtractMonitorState extract_monitor_state_;
  std::mutex extract_monitor_mutex_;
  double extract_monitor_last_stage_ms_ = 0.0;
  mutable std::atomic<uint64_t> extract_collision_check_count_{0};
  mutable std::atomic<uint64_t> extract_collision_check_total_ns_{0};
  mutable std::atomic<uint64_t> extract_collision_check_max_ns_{0};
  std::shared_ptr<BoxPoseRrtExtractPlannerConfig::Profile> box_pose_rrt_profile_ =
    std::make_shared<BoxPoseRrtExtractPlannerConfig::Profile>();
  std::shared_ptr<ExtractCandidateSolverConfig::Profile> box_pose_solver_profile_ =
    std::make_shared<ExtractCandidateSolverConfig::Profile>();

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr demo_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr box_stack_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_demo_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_monitor_next_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_monitor_full_selected_srv_;
  rclcpp::Service<alfa_robot_moveit_config::srv::ConfigureExtractMonitor>::SharedPtr extract_monitor_config_srv_;
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
