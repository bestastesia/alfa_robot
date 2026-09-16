#include "alfa_robot_moveit_config/demo_failure_markers.hpp"
#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>
#include <alfa_robot_moveit_config/planning_diagnostics.hpp>
#include <alfa_robot_moveit_config/natural_joint_motion.hpp>
#include <alfa_robot_moveit_config/comfort_height.hpp>
#include <alfa_robot_moveit_config/wall_sequence.hpp>
#include <alfa_robot_moveit_config/wall_trajectory_postprocessing.hpp>
#include <ompl/util/RandomNumbers.h>
#include <alfa_robot_moveit_config/srv/plan_wall_box_demo.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometric_shapes/shapes.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
using alfa_robot::analytic_ik::V3RedundantArmModel;
using alfa_robot::analytic_ik::V3RedundantIkRequest;
using alfa_robot::analytic_ik::V3RedundantIkSolution;
using WallRequest = alfa_robot_moveit_config::srv::PlanWallBoxDemo;
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
using Marker = visualization_msgs::msg::Marker;

constexpr double kPi = 3.14159265358979323846;
constexpr char kMarkerName[] = "extract_box";
constexpr char kCarriedBoxId[] = "carried_target_box";
constexpr char kCarriedBoxLeftId[] = "carried_target_box_left";
constexpr char kCarriedBoxRightId[] = "carried_target_box_right";

double degToRad(double value)
{
  return value * kPi / 180.0;
}

double radToDeg(double value)
{
  return value * 180.0 / kPi;
}

double normalizedAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double maximumJointDelta(
  const std::array<double, 7>& from,
  const std::array<double, 7>& to)
{
  double maximum = 0.0;
  for (size_t index = 0; index < from.size(); ++index) {
    maximum = std::max(maximum, std::abs(normalizedAngle(to[index] - from[index])));
  }
  return maximum;
}


std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA output;
  output.r = red;
  output.g = green;
  output.b = blue;
  output.a = alpha;
  return output;
}

InteractiveMarkerControl axisControl(
  const std::string& name,
  double x,
  double y,
  double z)
{
  InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.orientation.w = 1.0;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  return control;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

using ReplayFrame = alfa_robot::motion::TrajectoryFrame;

struct AnalyticCandidate
{
  moveit::core::RobotStatePtr state;
  V3RedundantIkSolution solution;
  double score = std::numeric_limits<double>::infinity();
};

struct RrtPlanResult
{
  bool success = false;
  double wall_ms = 0.0;
  double planner_ms = 0.0;
  std::string reason;
  std::vector<moveit::core::RobotStatePtr> states;
  moveit::core::RobotStatePtr rejected_state;
};

struct PlanningMetrics
{
  uint64_t ik_calls = 0;
  double ik_ms = 0.0;
  uint64_t collision_checks = 0;
  double collision_ms = 0.0;
  double analytic_path_ms = 0.0;
  double rrt_approach_ms = 0.0;
  double rrt_return_ms = 0.0;
  double shortcut_ms = 0.0;
  double chomp_ms = 0.0;
  double totg_ms = 0.0;
  double ruckig_ms = 0.0;

  void add(const PlanningMetrics& other)
  {
    ik_calls += other.ik_calls;
    ik_ms += other.ik_ms;
    collision_checks += other.collision_checks;
    collision_ms += other.collision_ms;
    analytic_path_ms += other.analytic_path_ms;
    rrt_approach_ms += other.rrt_approach_ms;
    rrt_return_ms += other.rrt_return_ms;
    shortcut_ms += other.shortcut_ms;
    chomp_ms += other.chomp_ms;
    totg_ms += other.totg_ms;
    ruckig_ms += other.ruckig_ms;
  }
};

struct TaskResult
{
  bool success = false;
  std::string failure_stage;
  std::string failure_reason;
  double total_ms = 0.0;
  PlanningMetrics metrics;
  std::vector<ReplayFrame> frames;
  std::vector<ReplayFrame> diagnostic_frames;
  moveit::core::RobotStatePtr rejected_state;
  nlohmann::json diagnostic = nlohmann::json::object();
  size_t complete_candidate_count = 0;
  size_t selected_candidate_rank = 0;
  double selection_score = std::numeric_limits<double>::infinity();
  double execution_duration_s = 0.0;
  bool timing_valid = false;
  std::string effective_trajectory_variant = "topk";
  std::string optimizer_status = "not_requested";
  std::string variant_fallback_reason;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_jerk = 0.0;
};

class V3SingleArmBoxExtractDemo : public rclcpp::Node
{
public:
  explicit V3SingleArmBoxExtractDemo(const rclcpp::NodeOptions& options)
  : Node("v3_single_arm_box_extract_demo", options)
  {}

  void init()
  {
    distance_demo_ = getParameter<bool>("distance_demo", false);
    post_extract_policy_ = getParameter<std::string>(
      "post_extract_policy", "rear_release");
    if (post_extract_policy_ != "rear_release" &&
        post_extract_policy_ != "loaded_home") {
      throw std::invalid_argument(
              "post_extract_policy must be rear_release or loaded_home");
    }
    initial_pose_ = getParameter<std::string>("initial_pose", "home");
    if (initial_pose_ != "home" && (initial_pose_ != "arms_down" || !distance_demo_))
      throw std::invalid_argument("initial_pose must be home, or arms_down for wall simulation");
    side_ = getParameter<std::string>("side", "left");
    world_frame_ = getParameter<std::string>("world_frame", "world");
    arm_base_link_ = getParameter<std::string>("arm_base_link", "arm_carriage");
    planning_group_name_ = getParameter<std::string>(
      "planning_group", side_ == "left" ? "left_arm" : "right_arm");
    tool_link_ = getParameter<std::string>(
      "tool_link", side_ == "left" ? "left_tool0" : "right_tool0");
    const auto initial_box = getParameter<std::vector<double>>(
      "initial_box_center", {0.88, -0.20, 0.55});
    if (initial_box.size() != 3) {
      throw std::invalid_argument("initial_box_center must contain x/y/z");
    }
    box_center_ = Eigen::Vector3d(initial_box[0], initial_box[1], initial_box[2]);
    wall_context_ = getParameter<std::string>("wall_context", "full");
    if (wall_context_ != "full" && wall_context_ != "sequence_prefix" &&
        wall_context_ != "target_only") {
      throw std::invalid_argument(
              "wall_context must be full, sequence_prefix, or target_only");
    }
    scene_layout_ = getParameter<std::string>("scene_layout", "cross");
    if (distance_demo_) scene_layout_ = "wall_5x5";
    wall_target_row_ = getParameter<int>("wall_target_row", 0);
    wall_target_column_ = getParameter<int>("wall_target_column", 2);
    wall_gap_ = getParameter<double>("wall_gap", 0.01);
    if (scene_layout_ != "cross" && scene_layout_ != "wall_5x5") {
      throw std::invalid_argument("scene_layout must be cross or wall_5x5");
    }
    if (wall_target_row_ < 0 || wall_target_row_ >= 5 ||
        wall_target_column_ < 0 || wall_target_column_ >= 5 ||
        !std::isfinite(wall_gap_) || wall_gap_ < 0.0) {
      throw std::invalid_argument("wall target indices must be 0..4 and wall_gap finite/nonnegative");
    }
    box_depth_ = getParameter<double>("box_depth", 0.30);
    box_width_ = getParameter<double>("box_width", 0.40);
    box_height_ = getParameter<double>("box_height", 0.40);
    if (scene_layout_ == "wall_5x5" && !distance_demo_) {
      const auto origin = getParameter<std::vector<double>>("wall_origin", {});
      if (origin.size() != 3) {
        throw std::invalid_argument("wall_origin must contain the bottom-row column-0 box center x/y/z");
      }
      box_center_ = Eigen::Vector3d(origin[0], origin[1], origin[2]) + Eigen::Vector3d(
        0.0, wall_target_column_ * (box_width_ + wall_gap_),
        wall_target_row_ * (box_height_ + wall_gap_));
    }
    initial_box_center_ = box_center_;
    control_handle_clearance_ = std::max(
      0.10, getParameter<double>("control_handle_clearance", 0.25));
    control_handle_lateral_offset_ = std::max(
      0.50, getParameter<double>("control_handle_lateral_offset", 0.90));
    approach_distance_ = getParameter<double>("approach_distance", 0.05);
    retreat_distance_ = getParameter<double>("retreat_distance", 0.35);
    cartesian_step_ = getParameter<double>("cartesian_step", 0.01);
    collision_inset_ = getParameter<double>(
      "collision_inset", scene_layout_ == "wall_5x5" ? 0.0 : 0.002);
    psi_step_ = degToRad(getParameter<double>("psi_step_deg", 5.0));
    maximum_cartesian_joint_step_ = degToRad(
      getParameter<double>("maximum_cartesian_joint_step_deg", 15.0));
    edge_joint_resolution_ = degToRad(
      getParameter<double>("edge_joint_resolution_deg", distance_demo_ ? 0.25 : 2.5));
    precontact_candidate_limit_ = static_cast<size_t>(std::max(
      1, getParameter<int>("precontact_candidate_limit", 8)));
    rrt_planning_time_ = std::max(0.05, getParameter<double>("rrt_planning_time", 1.0));
    rrt_planning_attempts_ = std::max(1, getParameter<int>("rrt_planning_attempts", 1));
    auto_run_once_ = getParameter<bool>("auto_run_once", false);
    sequence_mode_ = getParameter<bool>("sequence_mode", false);
    if (sequence_mode_ && !distance_demo_) throw std::invalid_argument("sequence_mode requires distance_demo");

    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be left or right");
    }
    if (planning_group_name_ != side_ + "_arm" || tool_link_ != side_ + "_tool0" ||
        arm_base_link_ != "arm_carriage") {
      throw std::invalid_argument("planning_group/tool_link/arm_base_link must match the V3.1.1 side");
    }
    for (const double value : {box_depth_, box_width_, box_height_, approach_distance_,
         retreat_distance_, cartesian_step_, psi_step_, maximum_cartesian_joint_step_,
         edge_joint_resolution_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("box dimensions, Cartesian distances and angular steps must be finite/positive");
      }
    }
    if (!box_center_.allFinite() || !std::isfinite(collision_inset_) ||
        collision_inset_ < 0.0 ||
        2.0 * collision_inset_ >= std::min({box_depth_, box_width_, box_height_})) {
      throw std::invalid_argument("box center must be finite and collision_inset must preserve positive box dimensions");
    }

    planning_seed_ = getParameter<int>("planning_seed", 0);
    if (planning_seed_ < 0) throw std::invalid_argument("planning_seed must be nonnegative");
    trajectory_variant_ = getParameter<std::string>("trajectory_variant", "topk");
    if (trajectory_variant_ != "topk" && trajectory_variant_ != "shortcut_ruckig" &&
        trajectory_variant_ != "chomp_ruckig")
      throw std::invalid_argument("trajectory_variant must be topk, shortcut_ruckig, or chomp_ruckig");
    top_k_complete_ = getParameter<int>("top_k_complete", 3);
    if (top_k_complete_ < 1 || top_k_complete_ > 8)
      throw std::invalid_argument("top_k_complete must be in [1, 8]");
    trajectory_sample_period_ = getParameter<double>("trajectory_sample_period", 0.05);
    if (!std::isfinite(trajectory_sample_period_) || trajectory_sample_period_ <= 0.0)
      throw std::invalid_argument("trajectory_sample_period must be finite and positive");
    empty_velocity_scaling_ = getParameter<double>("empty_velocity_scaling", 0.50);
    empty_acceleration_scaling_ = getParameter<double>("empty_acceleration_scaling", 0.50);
    loaded_velocity_scaling_ = getParameter<double>("loaded_velocity_scaling", 0.25);
    loaded_acceleration_scaling_ = getParameter<double>("loaded_acceleration_scaling", 0.25);
    arm_max_jerk_ = getParameter<double>("arm_max_jerk", 2.0);
    head_max_jerk_ = getParameter<double>("head_max_jerk", 2.0);
    updown_max_jerk_ = getParameter<double>("updown_max_jerk", 0.30);
    for (const double value : {empty_velocity_scaling_, empty_acceleration_scaling_,
         loaded_velocity_scaling_, loaded_acceleration_scaling_})
      if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
        throw std::invalid_argument("trajectory velocity/acceleration scaling must be in (0, 1]");
    for (const double value : {arm_max_jerk_, head_max_jerk_, updown_max_jerk_})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("trajectory jerk limits must be finite and positive");
    if (planning_seed_ > 0) ompl::RNG::setSeed(static_cast<unsigned int>(planning_seed_));
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("failed to load robot model");
    }
    if (world_frame_ != robot_model_->getModelFrame()) {
      throw std::invalid_argument("world_frame must match the robot model frame");
    }
    planning_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    if (!planning_group_ || planning_group_->getVariableCount() != 7U) {
      throw std::runtime_error("planning group must be a seven-axis arm: " + planning_group_name_);
    }
    if (!robot_model_->hasLinkModel(tool_link_) || !robot_model_->hasLinkModel(arm_base_link_)) {
      throw std::runtime_error("missing tool or arm base link");
    }

    all_joint_names_.reserve(16);
    for (const std::string arm_side : {std::string("left"), std::string("right")}) {
      for (int index = 1; index <= 7; ++index) {
        all_joint_names_.push_back(arm_side + "_joint" + std::to_string(index));
      }
    }

    // Keep the original 14 arm entries in order; publish the shared axes for complete TF.
    all_joint_names_.push_back("updown");
    all_joint_names_.push_back("head_joint");

    initial_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    initial_state_->setToDefaultValues();
    for (const auto& name : all_joint_names_) {
      if (robot_model_->hasJointModel(name)) {
        initial_state_->setVariablePosition(name, 0.0);
      }
    }
    if (distance_demo_ && !initial_state_->setToDefaultValues(
        robot_model_->getJointModelGroup("whole_body"), "home")) {
      throw std::runtime_error("distance demo requires SRDF whole_body/home");
    }
    initial_state_->update(true);
    home_state_ = std::make_shared<moveit::core::RobotState>(*initial_state_);
    resetInitialState();
    display_state_ = std::make_shared<moveit::core::RobotState>(*initial_state_);
    if (distance_demo_) {
      // Keep rounded STL faces strictly behind the box plane without relaxing collisions.
      // This is a simulation numerical gap, not calibrated suction compliance.
      contact_numerical_gap_ = getParameter<double>("contact_numerical_gap", 1e-6);
      if (!std::isfinite(contact_numerical_gap_) || contact_numerical_gap_ < 0.0 ||
          contact_numerical_gap_ > 1e-4) {
        throw std::invalid_argument("contact_numerical_gap must be finite and in [0, 0.0001] metres");
      }
      align_height_ = getParameter<bool>("align_height", true);
      height_strategy_ = getParameter<std::string>("height_strategy", "fixed_offset");
      top_shoulder_above_wrist_ = getParameter<double>("top_shoulder_above_wrist", 0.10);
      if (!std::isfinite(top_shoulder_above_wrist_) || top_shoulder_above_wrist_ < 0.0)
        throw std::invalid_argument("top_shoulder_above_wrist must be finite and nonnegative metres");
      comfort_branch_ = getParameter<std::string>("comfort_branch", "auto");
      comfort_min_ = getParameter<double>("comfort_ratio_min", 0.8);
      comfort_preferred_ = getParameter<double>("comfort_ratio_preferred", 0.8);
      comfort_max_ = getParameter<double>("comfort_ratio_max", 0.8);
      if (height_strategy_ != "fixed_offset" && height_strategy_ != "comfort_radius")
        throw std::invalid_argument("height_strategy must be fixed_offset or comfort_radius");
      // Validate even when disabled; do not silently accept a broken experiment config.
      alfa_robot::motion::chooseComfortHeight(1, 0, 0, 1, 0, -1, 0,
        comfort_min_, comfort_preferred_, comfort_max_, comfort_branch_);
      shoulder_box_offset_ = getParameter<double>("shoulder_box_offset", 0.25);
      if (!std::isfinite(shoulder_box_offset_) || shoulder_box_offset_ < 0.0) {
        throw std::invalid_argument("shoulder_box_offset must be finite and nonnegative metres");
      }
      const Eigen::Vector3d shoulder_midpoint = 0.5 * (
        V3RedundantArmAnalyticIk(V3RedundantArmModel::V311Left).modelShoulderCenterInArmBase() +
        V3RedundantArmAnalyticIk(V3RedundantArmModel::V311Right).modelShoulderCenterInArmBase());
      initial_shoulder_z_ = (initial_state_->getGlobalLinkTransform(arm_base_link_) *
        shoulder_midpoint).z();
      chassis_front_x_ = getParameter<double>("chassis_front_x", modelChassisFrontX());
      chassis_rear_x_ = modelChassisFrontX(true);
      rear_clearance_ = getParameter<double>("rear_clearance", 0.02);
      if (!std::isfinite(rear_clearance_) || rear_clearance_ < 0.01)
        throw std::invalid_argument("rear_clearance must be finite and at least 0.01m");
      wall_center_y_ = getParameter<double>("wall_center_y", 0.0);
      wall_bottom_z_ = getParameter<double>("wall_bottom_z", 0.0);
      if (!std::isfinite(chassis_front_x_) || !std::isfinite(wall_center_y_) ||
          !std::isfinite(wall_bottom_z_) || collision_inset_ != 0.0) {
        throw std::invalid_argument("distance demo requires finite placement and zero collision_inset");
      }
      updateWallTarget(getParameter<double>("x", -1.0), getParameter<int>("box_id", 0));
      initial_box_center_ = box_center_;
      requested_suction_mode_ = getParameter<std::string>("suction_mode", "auto");
      if (requested_suction_mode_ != "auto" && requested_suction_mode_ != "top")
        throw std::invalid_argument("suction_mode must be auto or top");
      top_suction_ = requested_suction_mode_ == "top" || alfa_robot::motion::isBottomBox(
        box_center_.z(), box_height_, wall_bottom_z_);
      requested_arm_ = getParameter<std::string>("arm", "auto");
      if (requested_arm_ != "auto" && requested_arm_ != "left" && requested_arm_ != "right")
        throw std::invalid_argument("arm must be left/right/auto");
    }

    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side_ == "left" ? V3RedundantArmModel::V311Left : V3RedundantArmModel::V311Right);

    std::vector<std::string> request_adapters = {
      "default_planner_request_adapters/AddTimeOptimalParameterization",
      "default_planner_request_adapters/ResolveConstraintFrames",
      "default_planner_request_adapters/FixWorkspaceBounds",
      "default_planner_request_adapters/FixStartStateBounds",
      "default_planner_request_adapters/FixStartStateCollision",
      "default_planner_request_adapters/FixStartStatePathConstraints",
    };
    // This standalone demo returns geometric paths, not time-parameterized commands.
    // Avoid TOTG resampling/deforming the path; validate every returned edge below.
    if (distance_demo_) request_adapters.erase(request_adapters.begin());
    planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      robot_model_, shared_from_this(), "ompl", "ompl_interface/OMPLPlanner", request_adapters);
    planning_pipeline_->displayComputedMotionPlans(false);
    planning_pipeline_->publishReceivedRequests(false);
    planning_pipeline_->checkSolutionPaths(true);

    task_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/task_json", rclcpp::QoS(1).reliable().transient_local());
    // At most 25 checked box segments per task. The separate depth-1 topic remains
    // the authoritative full snapshot for late subscribers/reconnects.
    segment_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/task_json_segments", rclcpp::QoS(32).reliable().transient_local());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    scene_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/scene_markers", rclcpp::QoS(1).reliable().transient_local());
    status_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/status_markers", rclcpp::QoS(1).reliable().transient_local());
    run_service_ = create_service<std_srvs::srv::Trigger>(
      "~/run_current_box",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = requestPlanning();
        response->message = response->success ?
          "planning request accepted" : "planner is already running";
      });

    if (distance_demo_) {
      sequence_service_ = create_service<std_srvs::srv::Trigger>("~/plan_wall_sequence",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (busy()) {
            response->message = "planner or sequence playback is busy";
            return;
          }
          sequence_requested_ = true;
          response->success = requestPlanning();
          response->message = response->success ? "sequence accepted; result on ~/task_json" : "busy";
        });
      wall_service_ = create_service<WallRequest>("~/plan_wall_box",
        [this](const std::shared_ptr<WallRequest::Request> request,
               std::shared_ptr<WallRequest::Response> response) {
          if (!std::isfinite(request->x) || request->x <= 0.0 ||
              request->box_id < 0 || request->box_id >= 25 ||
              (request->arm != "left" && request->arm != "right" && request->arm != "auto")) {
            response->failure_stage = "invalid_request";
            response->failure_reason = "x must be finite/positive, box_id 0..24, arm left/right/auto";
            return;
          }
          if (busy()) {
            response->failure_stage = "busy";
            response->failure_reason = "a planning request is pending";
            return;
          }
          try {
            updateWallTarget(request->x, request->box_id);
          } catch (const std::exception& error) {
            response->failure_stage = "invalid_request";
            response->failure_reason = error.what();
            return;
          }
          requested_arm_ = request->arm;
          sequence_requested_ = false;
          requestPlanning();
          onWorkerTimer();
          response->success = last_result_.at("success").get<bool>();
          response->generation = generation_;
          response->selected_arm = response->success ? side_ : "";
          response->failure_stage = last_result_.at("failure_stage").get<std::string>();
          response->failure_reason = last_result_.at("failure_reason").get<std::string>();
          response->result_json = last_result_.dump();
        });
    }

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "v3_single_arm_box_extract_demo_marker",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());
    if (!distance_demo_) createBoxMarker();
    confirm_menu_entry_ = menu_handler_.insert(
      "确认并计算当前箱位",
      [this](const Feedback::ConstSharedPtr&) {requestPlanning();});
    reset_menu_entry_ = menu_handler_.insert(
      "恢复默认箱位",
      [this](const Feedback::ConstSharedPtr&) {resetBoxPose();});
    (void)confirm_menu_entry_;
    (void)reset_menu_entry_;
    if (!distance_demo_) menu_handler_.apply(*marker_server_, kMarkerName);
    marker_server_->applyChanges();

    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds(25), [this]() {onWorkerTimer();});
    display_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {publishDisplayState();});
    if (distance_demo_) selectArm(requested_arm_ == "auto" ? "left" : requested_arm_);
    publishPreview(distance_demo_ ? "用 plan_wall_box 服务选择距离和箱号" :
      "拖动箱体XYZ；右键箱体并选择“确认并计算当前箱位”");
    publishSceneMarkers();
    publishStatus("READY", true);

    if (auto_run_once_) {
      auto_run_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() {
          if (auto_run_timer_) {
            auto_run_timer_->cancel();
          }
          sequence_requested_ = sequence_mode_;
          requestPlanning();
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "V3 single-arm box extract demo ready: side=%s box=(%.2f,%.2f,%.2f) "
      "approach=%.2fm retreat=%.2fm psi_step=%.1fdeg RRT=%.2fs",
      side_.c_str(), box_depth_, box_width_, box_height_, approach_distance_,
      retreat_distance_, radToDeg(psi_step_), rrt_planning_time_);
  }

private:
  template<typename T>
  T getParameter(const std::string& name, const T& default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void loadEnvironment()
  {
    environment_objects_.clear();
    environment_json_ = {{"enabled", getParameter<bool>("check_environment", true)},
      {"frame_id", world_frame_}, {"boxes", nlohmann::json::array()}};
    if (!environment_json_.at("enabled").get<bool>()) {
      RCLCPP_WARN(get_logger(), "ENVIRONMENT COLLISIONS DISABLED: regression-only wall scene");
      return;
    }
    try {
      const auto config = nlohmann::json::parse(getParameter<std::string>("environment_json", ""));
      if (config.at("frame_id").get<std::string>() != world_frame_)
        throw std::invalid_argument("frame_id must match planning world frame");
      const auto anchor = config.value("anchor", std::string("world"));
      if (anchor != "world" && anchor != "box_wall_back")
        throw std::invalid_argument("anchor must be world or box_wall_back");
      const Eigen::Vector3d offset = anchor == "box_wall_back" ?
        // Same micrometre tolerance as tool contact: avoid false penetration on attachment.
        Eigen::Vector3d(chassis_front_x_ + wall_distance_ + box_depth_ + contact_numerical_gap_,
                        wall_center_y_, 0.0) :
        Eigen::Vector3d::Zero();
      environment_json_["anchor"] = anchor;
      const auto& boxes = config.at("boxes");
      if (!boxes.is_array() || boxes.empty() || boxes.size() > 128)
        throw std::invalid_argument("boxes must contain 1..128 axis-aligned boxes, including ground");
      std::set<std::string> ids;
      for (const auto& box : boxes) {
        const auto id = box.at("id").get<std::string>();
        if (id.empty() || id.find_first_not_of(
              "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos ||
            !ids.insert(id).second)
          throw std::invalid_argument("box ids must be unique nonempty letters/digits/_/-");
        // Reject unsupported rotation rather than silently drawing/checking a different obstacle.
        for (auto it = box.begin(); it != box.end(); ++it)
          if (it.key() != "id" && it.key() != "center" && it.key() != "size")
            throw std::invalid_argument("box supports only id, center, size (world-axis aligned)");
        for (const auto* key : {"center", "size"}) {
          const auto& values = box.at(key);
          if (!values.is_array() || values.size() != 3)
            throw std::invalid_argument("center/size must be 3 numbers in metres");
          for (const auto& value : values) {
            if (!value.is_number() || !std::isfinite(value.get<double>()) ||
                std::abs(value.get<double>()) > 10000.0 ||
                (std::string(key) == "size" && value.get<double>() <= 0.0))
              throw std::invalid_argument("finite center/positive size required, magnitude <=10000m");
          }
        }
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = world_frame_;
        object.id = "environment_" + id;
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {box.at("size")[0].get<double>(),
          box.at("size")[1].get<double>(), box.at("size")[2].get<double>()};
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.at("center")[0].get<double>() + offset.x();
        pose.position.y = box.at("center")[1].get<double>() + offset.y();
        pose.position.z = box.at("center")[2].get<double>();
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        environment_objects_.push_back(object);
        environment_json_["boxes"].push_back({{"id", object.id},
          {"center", {pose.position.x, pose.position.y, pose.position.z}}, {"size", box.at("size")}});
      }
      if (!ids.count("ground")) throw std::invalid_argument("a box named ground is required");
      environment_json_["description"] = config.at("description").get<std::string>();
    } catch (const std::exception& error) {
      throw std::invalid_argument(std::string("invalid environment configuration: ") + error.what());
    }
    RCLCPP_WARN(get_logger(), "Environment collision checking ON: %zu objects; %s. "
      "Configured geometry only, not sensed/calibrated surroundings.", environment_objects_.size(),
      environment_json_.at("description").get<std::string>().c_str());
  }

  double modelChassisFrontX(bool rear = false) const
  {
    constexpr std::array<const char*, 11> chassis_links = {
      "model_base", "chassis_base", "active_suspension_carriage",
      "caster01", "caster02", "caster03", "caster04",
      "wheel01", "wheel02", "wheel03", "wheel04"};
    double front = rear ? std::numeric_limits<double>::infinity() :
      -std::numeric_limits<double>::infinity();
    for (const char* link_name : chassis_links) {
      const auto* link = robot_model_->getLinkModel(link_name);
      if (!link) throw std::runtime_error(std::string(link_name) + " missing from V3 chassis model");
      const auto& shapes = link->getShapes();
      const auto& origins = link->getCollisionOriginTransforms();
      for (size_t i = 0; i < shapes.size(); ++i) {
        if (shapes[i]->type != shapes::MESH) {
          throw std::runtime_error(std::string(link_name) + " chassis collision must be a mesh");
        }
        const auto* mesh = static_cast<const shapes::Mesh*>(shapes[i].get());
        const Eigen::Isometry3d transform = initial_state_->getGlobalLinkTransform(link) * origins[i];
        for (unsigned int v = 0; v < mesh->vertex_count; ++v) {
          const Eigen::Vector3d point(
            mesh->vertices[3*v], mesh->vertices[3*v+1], mesh->vertices[3*v+2]);
          front = rear ? std::min(front, (transform * point).x()) :
            std::max(front, (transform * point).x());
        }
      }
    }
    if (!std::isfinite(front)) throw std::runtime_error("no V3 chassis collision vertices");
    return front;
  }

  Eigen::Vector3d wallBoxCenter(double x, int box_id) const
  {
    return {chassis_front_x_ + x + box_depth_ / 2.0,
      wall_center_y_ + (box_id % 5 - 2) * (box_width_ + wall_gap_),
      wall_bottom_z_ + box_height_ / 2.0 + (box_id / 5) * (box_height_ + wall_gap_)};
  }

  void updateWallTarget(double x, int box_id)
  {
    if (!std::isfinite(x) || x <= 0.0 || box_id < 0 || box_id >= 25) {
      throw std::invalid_argument("x must be finite/positive and box_id must be 0..24");
    }
    const Eigen::Vector3d center = wallBoxCenter(x, box_id);
    if (!center.allFinite()) throw std::invalid_argument("wall coordinates overflow");
    wall_distance_ = x;
    wall_target_row_ = box_id / 5;
    wall_target_column_ = box_id % 5;
    box_center_ = center;
    loadEnvironment();
  }

  void selectArm(const std::string& side)
  {
    height_clearance_ = {{"checked", false}};
    side_ = side;
    tool_link_ = side + "_tool0";
    planning_group_name_ = side + "_arm";
    planning_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side == "left" ? V3RedundantArmModel::V311Left : V3RedundantArmModel::V311Right);
  }

  void createBoxMarker()
  {
    const Eigen::Vector3d handle_position = controlHandlePosition(box_center_);
    InteractiveMarker marker;
    marker.header.frame_id = world_frame_;
    marker.name = kMarkerName;
    marker.description = "箱堆外侧XYZ控制球：拖动后右键确认计算";
    marker.scale = 0.65;
    marker.pose.position.x = handle_position.x();
    marker.pose.position.y = handle_position.y();
    marker.pose.position.z = handle_position.z();
    marker.pose.orientation.w = 1.0;

    InteractiveMarkerControl body;
    body.always_visible = true;
    body.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    Marker handle;
    handle.type = Marker::SPHERE;
    handle.scale.x = handle.scale.y = handle.scale.z = 0.09;
    handle.color = color(0.10F, 0.85F, 1.0F, 0.95F);
    body.markers.push_back(handle);
    marker.controls.push_back(body);
    marker.controls.push_back(axisControl("move_x", 1.0, 0.0, 0.0));
    marker.controls.push_back(axisControl("move_y", 0.0, 1.0, 0.0));
    marker.controls.push_back(axisControl("move_z", 0.0, 0.0, 1.0));

    marker_server_->insert(
      marker,
      [this](const Feedback::ConstSharedPtr& feedback) {handleMarkerFeedback(feedback);});
  }

  Eigen::Vector3d controlHandlePosition(const Eigen::Vector3d& center) const
  {
    const double lateral_sign = side_ == "left" ? -1.0 : 1.0;
    return center + Eigen::Vector3d(
      -(box_depth_ * 0.5 + control_handle_clearance_),
      lateral_sign * control_handle_lateral_offset_,
      0.0);
  }

  void handleMarkerFeedback(const Feedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type != Feedback::POSE_UPDATE &&
        feedback->event_type != Feedback::MOUSE_UP) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      box_center_ = Eigen::Vector3d(
        feedback->pose.position.x + box_depth_ * 0.5 + control_handle_clearance_,
        feedback->pose.position.y +
          (side_ == "left" ? control_handle_lateral_offset_ : -control_handle_lateral_offset_),
        feedback->pose.position.z);
    }
    publishPreview("箱位已更新，右键确认后才开始计算");
    publishSceneMarkers();
  }

  void resetBoxPose()
  {
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      box_center_ = initial_box_center_;
    }
    const Eigen::Vector3d handle_position = controlHandlePosition(box_center_);
    geometry_msgs::msg::Pose pose;
    pose.position.x = handle_position.x();
    pose.position.y = handle_position.y();
    pose.position.z = handle_position.z();
    pose.orientation.w = 1.0;
    marker_server_->setPose(kMarkerName, pose);
    marker_server_->applyChanges();
    publishPreview("已恢复默认箱位");
    publishSceneMarkers();
  }

  bool busy() const
  {
    return planning_active_.load() || planning_requested_.load() || sequence_playback_;
  }

  TaskResult planWithFallback(const Eigen::Vector3d& center, bool allow_opposite_arm = false)
  {
    attempts_ = nlohmann::json::array();
    if (!distance_demo_) return planTask(center);
    const auto started = std::chrono::steady_clock::now();
    struct Choice { TaskResult result; std::string side; bool top; };
    std::vector<Choice> complete;
    TaskResult deepest_failure;
    std::string failure_side;
    bool failure_top = false;
    nlohmann::json failure_height;
    PlanningMetrics metrics;
    for (const auto& [top, side] : alfa_robot::motion::wallGraspAttempts(
        alfa_robot::motion::isBottomBox(center.z(), box_height_, wall_bottom_z_), requested_arm_,
        requested_suction_mode_ == "top", allow_opposite_arm)) {
      top_suction_ = top;
      selectArm(side);
      std::vector<TaskResult> candidates;
      TaskResult result = planTask(center, &candidates);
      metrics.add(result.metrics);
      attempts_.push_back({{"arm", side}, {"suction_mode", top ? "top" : "front"},
        {"success", result.success}, {"failure_stage", result.failure_stage},
        {"failure_reason", result.failure_reason}, {"total_ms", result.total_ms},
        {"complete_candidate_count", candidates.size()},
        {"height_alignment", heightAlignment(center)}, {"height_selections", 1}});
      RCLCPP_INFO(get_logger(), "box=%d arm=%s suction=%s %s candidates=%zu stage=%s reason=%s",
        wall_target_row_ * 5 + wall_target_column_, side.c_str(), top ? "top" : "front",
        result.success ? "SUCCESS" : "FAILED", candidates.size(), result.failure_stage.c_str(),
        result.failure_reason.c_str());
      for (auto& candidate : candidates) {
        complete.push_back({std::move(candidate), side, top});
        if (complete.size() >= static_cast<size_t>(top_k_complete_)) break;
      }
      if (complete.size() >= static_cast<size_t>(top_k_complete_)) break;
      if (!result.success && result.diagnostic_frames.size() >= deepest_failure.diagnostic_frames.size()) {
        deepest_failure = std::move(result);
        failure_side = side;
        failure_top = top;
        failure_height = height_clearance_;
      }
    }
    if (!complete.empty()) {
      std::stable_sort(complete.begin(), complete.end(), [](const Choice& left, const Choice& right) {
        return left.result.selection_score < right.result.selection_score;
      });
      std::string postprocess_reason;
      for (size_t rank = 0; rank < complete.size(); ++rank) {
        selectArm(complete[rank].side);
        top_suction_ = complete[rank].top;
        const auto scene = makeScene(center);
        const auto loaded = loadedScene(scene);
        const auto edge_validator = [&](const ReplayFrame& from, const ReplayFrame& to, std::string* reason) {
          if (from.box_attached != to.box_attached) return from.joints == to.joints;
          moveit::core::RobotState from_state(*initial_state_), to_state(*initial_state_);
          for (size_t i = 0; i < all_joint_names_.size(); ++i) {
            from_state.setVariablePosition(all_joint_names_[i], from.joints[i]);
            to_state.setVariablePosition(all_joint_names_[i], to.joints[i]);
          }
          if (from.box_attached) { attachCarriedBox(from_state); attachCarriedBox(to_state); }
          from_state.update(true); to_state.update(true);
          return edgeClear(from.box_attached ? loaded : scene, from_state, to_state,
            from.box_attached, &metrics, reason);
        };
        const auto path_validator = [&](const std::vector<ReplayFrame>& frames, std::string* reason) {
          return validateSingleArmFrames(frames, scene, loaded, &metrics, reason);
        };
        TaskResult result = complete[rank].result;
        if (!postprocessCandidate(&result, scene, loaded, planning_group_name_, edge_validator, path_validator,
            &postprocess_reason)) {
          metrics.shortcut_ms += result.metrics.shortcut_ms;
          metrics.chomp_ms += result.metrics.chomp_ms;
          metrics.totg_ms += result.metrics.totg_ms;
          metrics.ruckig_ms += result.metrics.ruckig_ms;
          continue;
        }
        result.metrics.add(metrics);
        result.total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        result.complete_candidate_count = complete.size();
        result.selected_candidate_rank = rank + 1;
        return result;
      }
      deepest_failure.failure_stage = "trajectory_postprocess";
      deepest_failure.failure_reason = "all ranked candidates failed: " + postprocess_reason;
    }
    TaskResult result = std::move(deepest_failure);
    if (!failure_side.empty()) {
      top_suction_ = failure_top;
      selectArm(failure_side);
      height_clearance_ = std::move(failure_height);
    }
    result.metrics = metrics;
    result.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    result.frames.clear();
    return result;
  }

  static int transferPhase(const std::string& stage)
  {
    if (stage == "attach_box") return 3;
    if (stage == "cartesian_approach") return 2;
    if (stage == "cartesian_retreat" || stage == "cartesian_lift") return 4;
    if (stage == "rrt_return") return 5;
    if (stage == "updown_return" || stage == "loaded_home") return 6;
    if (stage == "rear_placement") return 6;
    if (stage == "release_box") return 7;
    if (stage == "rrt_approach") return 1;
    return 0;
  }

  nlohmann::json carriedBoxJson(
    int box_id, const Eigen::Vector3d& center, const std::string& side, bool top,
    bool attached, bool visible) const
  {
    const auto transform = toolToBox(side, top);
    nlohmann::json rotation = nlohmann::json::array();
    for (int row = 0; row < 3; ++row)
      rotation.push_back({transform.linear()(row, 0), transform.linear()(row, 1),
        transform.linear()(row, 2)});
    return {{"box_id", box_id}, {"side", side}, {"tool_link", side + "_tool0"},
      {"box_center", {center.x(), center.y(), center.z()}}, {"attached", attached},
      {"visible", visible},
      {"tool_to_box_center", {transform.translation().x(), transform.translation().y(),
        transform.translation().z()}}, {"tool_to_box_rotation", rotation}};
  }

  double foldedElbowPosition(const std::string& side) const
  {
    const std::string joint = side + "_joint4";
    const auto& bounds = robot_model_->getVariableBounds(joint);
    return home_state_->getVariablePosition(joint) >= 0.0 ?
      bounds.max_position_ - degToRad(5) : bounds.min_position_ + degToRad(5);
  }

  std::array<double, 7> sideJoints(
    const moveit::core::RobotState& state, const std::string& side) const
  {
    std::vector<double> values;
    state.copyJointGroupPositions(robot_model_->getJointModelGroup(side + "_arm"), values);
    std::array<double, 7> output{};
    if (values.size() != output.size()) throw std::runtime_error("unexpected dual arm joint count");
    std::copy(values.begin(), values.end(), output.begin());
    return output;
  }

  planning_scene::PlanningScenePtr makeDualScene(int left_id, int right_id) const
  {
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    scene->setCurrentState(*initial_state_);
    for (const auto& object : environment_objects_)
      if (!scene->processCollisionObjectMsg(object))
        throw std::runtime_error("failed to add " + object.id + " to dual planning scene");
    for (int id = 0; id < 25; ++id) {
      if (removed_boxes_.count(id)) continue;
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = world_frame_;
      object.id = id == left_id ? kCarriedBoxLeftId :
        (id == right_id ? kCarriedBoxRightId : "wall_box_" + std::to_string(id));
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {box_depth_, box_width_, box_height_};
      const auto center = wallBoxCenter(wall_distance_, id);
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = center.x(); pose.position.y = center.y(); pose.position.z = center.z();
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(pose);
      if (!scene->processCollisionObjectMsg(object))
        throw std::runtime_error("failed to add " + object.id + " to dual planning scene");
    }
    return scene;
  }

  bool validateDualFrames(
    const std::vector<ReplayFrame>& frames, int left_id, int right_id, bool top,
    PlanningMetrics* metrics, std::string* reason) const
  {
    const auto scene = makeDualScene(left_id, right_id);
    // Each independent approach already validates its own final suction contact.
    // Preserve only those pairs while recombining; cross-arm and environment
    // collisions remain checked.
    scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxLeftId, "left_tool0", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxLeftId, "left_joint7", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxRightId, "right_tool0", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxRightId, "right_joint7", true);
    auto loaded = planning_scene::PlanningScene::clone(scene);
    loaded->getWorldNonConst()->removeObject(kCarriedBoxLeftId);
    loaded->getWorldNonConst()->removeObject(kCarriedBoxRightId);
    moveit::core::RobotStatePtr previous;
    bool previous_attached = false;
    for (const auto& frame : frames) {
      moveit::core::RobotState state(*initial_state_);
      for (size_t i = 0; i < all_joint_names_.size(); ++i)
        state.setVariablePosition(all_joint_names_[i], frame.joints.at(i));
      const bool attached = std::any_of(frame.carried_boxes.begin(), frame.carried_boxes.end(),
        [](const nlohmann::json& box) {return box.value("attached", false);});
      if (attached) {
        attachCarriedBox(state, kCarriedBoxLeftId, "left", top);
        attachCarriedBox(state, kCarriedBoxRightId, "right", top);
      }
      state.update(true);
      if (!state.satisfiesBounds()) { if (reason) *reason = "dual_joint_bounds"; return false; }
      const std::string collision = collisionReason(attached ? loaded : scene, state, metrics);
      if (!collision.empty()) {
        if (reason) *reason = "dual_" + frame.stage + "_" + collision;
        return false;
      }
      if (previous) {
        if (!alfa_robot::motion::sameShoulderElbowBranch(sideJoints(*previous, "left"), sideJoints(state, "left")) ||
            !alfa_robot::motion::sameShoulderElbowBranch(sideJoints(*previous, "right"), sideJoints(state, "right"))) {
          if (reason) *reason = "dual_shoulder_elbow_branch_flip";
          return false;
        }
        if (attached == previous_attached) {
          double maximum_delta = 0.0;
          for (const auto& name : all_joint_names_)
            maximum_delta = std::max(maximum_delta, std::abs(
              state.getVariablePosition(name) - previous->getVariablePosition(name)));
          const size_t steps = std::max<size_t>(1, std::ceil(maximum_delta / edge_joint_resolution_));
          for (size_t step = 1; step < steps; ++step) {
            moveit::core::RobotState probe(*previous);
            const double ratio = static_cast<double>(step) / steps;
            for (const auto& name : all_joint_names_)
              probe.setVariablePosition(name, previous->getVariablePosition(name) +
                (state.getVariablePosition(name) - previous->getVariablePosition(name)) * ratio);
            if (attached) {
              attachCarriedBox(probe, kCarriedBoxLeftId, "left", top);
              attachCarriedBox(probe, kCarriedBoxRightId, "right", top);
            }
            probe.update(true);
            const std::string edge_collision = collisionReason(attached ? loaded : scene, probe, metrics);
            if (!probe.satisfiesBounds() || !edge_collision.empty()) {
              if (reason) *reason = "dual_edge_to_" + frame.stage + "_" +
                (edge_collision.empty() ? std::string("bounds") : edge_collision);
              return false;
            }
          }
        }
      }
      previous = std::make_shared<moveit::core::RobotState>(state);
      previous_attached = attached;
    }
    return true;
  }

  TaskResult mergeDualCandidate(
    const TaskResult& left, const TaskResult& right, int left_id, int right_id, bool top,
    const Eigen::Vector3d& left_center, const Eigen::Vector3d& right_center,
    const std::vector<double>& common_joints, PlanningMetrics* metrics)
  {
    TaskResult result;

    std::vector<double> left_hold = common_joints, right_hold = common_joints;
    bool left_attached = false, right_attached = false, left_visible = true, right_visible = true;
    for (int phase = 0; phase <= 7; ++phase) {
      std::vector<const ReplayFrame*> left_phase, right_phase;
      for (const auto& frame : left.frames) if (transferPhase(frame.stage) == phase) left_phase.push_back(&frame);
      for (const auto& frame : right.frames) if (transferPhase(frame.stage) == phase) right_phase.push_back(&frame);
      const size_t count = std::max(left_phase.size(), right_phase.size());
      if (!count) continue;
      for (size_t i = 0; i < count; ++i) {
        auto sample = [i, count](const std::vector<const ReplayFrame*>& phase_frames) -> const ReplayFrame* {
          if (phase_frames.empty()) return nullptr;
          const size_t index = count == 1 ? phase_frames.size() - 1 :
            std::min(phase_frames.size() - 1, i * phase_frames.size() / count);
          return phase_frames[index];
        };
        if (const auto* frame = sample(left_phase)) {
          left_hold = frame->joints; left_attached = frame->box_attached; left_visible = frame->box_visible;
        }
        if (const auto* frame = sample(right_phase)) {
          right_hold = frame->joints; right_attached = frame->box_attached; right_visible = frame->box_visible;
        }
        if (left_attached != right_attached || left_visible != right_visible) {
          result.failure_stage = "dual_stage_sync";
          result.failure_reason = "independent plans changed attachment at different phase boundaries";
          return result;
        }
        ReplayFrame merged;
        merged.stage = "dual_" + std::to_string(phase);
        merged.joints = common_joints;
        for (size_t j = 0; j < all_joint_names_.size(); ++j) {
          const auto& name = all_joint_names_[j];
          if (name.rfind("left_", 0) == 0) merged.joints[j] = left_hold[j];
          else if (name.rfind("right_", 0) == 0) merged.joints[j] = right_hold[j];
          else if (name == "updown") {
            if (phase > 0 && std::abs(left_hold[j] - right_hold[j]) > 1e-4) {
              result.failure_stage = "dual_height_sync";
              result.failure_reason = "left/right updown goals differ after alignment";
              return result;
            }
            merged.joints[j] = 0.5 * (left_hold[j] + right_hold[j]);
          } else merged.joints[j] = left_hold[j];
        }
        merged.box_attached = left_attached;
        merged.box_visible = left_visible;
        merged.carried_boxes.push_back(carriedBoxJson(
          left_id, left_center, "left", top, left_attached, left_visible));
        merged.carried_boxes.push_back(carriedBoxJson(
          right_id, right_center, "right", top, right_attached, right_visible));
        result.frames.push_back(std::move(merged));
      }
    }
    std::string validation_reason;
    if (!validateDualFrames(result.frames, left_id, right_id, top, metrics, &validation_reason)) {
      result.failure_stage = "dual_combined_validation";
      result.failure_reason = validation_reason;
      result.frames.clear();
      return result;
    }
    result.selection_score = naturalPathLength(result.frames);
    result.success = true;
    return result;
  }

  TaskResult planDualPair(int left_id, int right_id, bool top, double x)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto common_joints = allJoints(*initial_state_);

    updateWallTarget(x, left_id);
    const auto left_center = box_center_;
    selectArm("left");
    top_suction_ = top;
    height_clearance_ = {{"checked", false}};
    const double left_updown =
      heightAlignment(left_center).at("target_updown").get<double>();

    updateWallTarget(x, right_id);
    const auto right_center = box_center_;
    selectArm("right");
    top_suction_ = top;
    height_clearance_ = {{"checked", false}};
    const double right_updown =
      heightAlignment(right_center).at("target_updown").get<double>();

    const auto& updown_bounds = robot_model_->getVariableBounds("updown");
    synchronized_updown_ = alfa_robot::motion::chooseSharedUpdown(
      left_updown, right_updown,
      updown_bounds.min_position_, updown_bounds.max_position_);
    RCLCPP_INFO(
      get_logger(),
      "dual shared height: boxes=%d,%d left=%.6fm right=%.6fm selected=%.6fm",
      left_id, right_id, left_updown, right_updown, *synchronized_updown_);

    updateWallTarget(x, left_id);
    selectArm("left");
    top_suction_ = top;
    std::vector<TaskResult> left_candidates;
    TaskResult left = planTask(left_center, &left_candidates);

    updateWallTarget(x, right_id);
    selectArm("right");
    top_suction_ = top;
    std::vector<TaskResult> right_candidates;
    TaskResult right = planTask(right_center, &right_candidates);

    synchronized_updown_.reset();
    PlanningMetrics metrics;
    metrics.add(left.metrics); metrics.add(right.metrics);
    if (!left.success || !right.success) {
      TaskResult result;
      result.metrics = metrics;
      result.failure_stage = "dual_independent_plan";
      result.failure_reason = "left=" + (left.success ? std::string("ok") : left.failure_stage + ":" + left.failure_reason) +
        " right=" + (right.success ? std::string("ok") : right.failure_stage + ":" + right.failure_reason);
      result.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      return result;
    }

    struct Pair { size_t left; size_t right; double score; };
    std::vector<Pair> pairs;
    for (size_t l = 0; l < left_candidates.size(); ++l)
      for (size_t r = 0; r < right_candidates.size(); ++r)
        pairs.push_back({l, r, left_candidates[l].selection_score + right_candidates[r].selection_score});
    std::stable_sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.score < b.score; });

    std::vector<TaskResult> complete;
    TaskResult last_failure;
    for (const auto& pair : pairs) {
      TaskResult candidate = mergeDualCandidate(left_candidates[pair.left], right_candidates[pair.right],
        left_id, right_id, top, left_center, right_center, common_joints, &metrics);
      if (candidate.success) {
        complete.push_back(std::move(candidate));
        if (complete.size() >= static_cast<size_t>(top_k_complete_)) break;
      } else last_failure = std::move(candidate);
    }
    if (!complete.empty()) {
      std::stable_sort(complete.begin(), complete.end(), [](const TaskResult& left, const TaskResult& right) {
        return left.selection_score < right.selection_score;
      });
      const auto edge_validator = [&](const ReplayFrame& from, const ReplayFrame& to, std::string* edge_reason) {
        std::vector<ReplayFrame> edge{from, to};
        return validateDualFrames(edge, left_id, right_id, top, &metrics, edge_reason);
      };
      const auto dual_scene = makeDualScene(left_id, right_id);
      const auto dual_loaded = planning_scene::PlanningScene::clone(dual_scene);
      const auto path_validator = [&](const std::vector<ReplayFrame>& frames, std::string* path_reason) {
        return validateDualFrames(frames, left_id, right_id, top, &metrics, path_reason);
      };
      for (size_t rank = 0; rank < complete.size(); ++rank) {
        TaskResult candidate = complete[rank];
        std::string postprocess_reason;
        if (!postprocessCandidate(&candidate, dual_scene, dual_loaded, "whole_body", edge_validator, path_validator,
            &postprocess_reason)) {
          metrics.shortcut_ms += candidate.metrics.shortcut_ms;
          metrics.chomp_ms += candidate.metrics.chomp_ms;
          metrics.totg_ms += candidate.metrics.totg_ms;
          metrics.ruckig_ms += candidate.metrics.ruckig_ms;
          last_failure.failure_stage = "trajectory_postprocess";
          last_failure.failure_reason = "candidate rank " + std::to_string(rank + 1) + " " + postprocess_reason;
          continue;
        }
        candidate.metrics.add(metrics);
        candidate.total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        candidate.complete_candidate_count = complete.size();
        candidate.selected_candidate_rank = rank + 1;
        return candidate;
      }
    }
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    last_failure.metrics = metrics;
    last_failure.total_ms = elapsed;
    if (last_failure.failure_stage.empty()) {
      last_failure.failure_stage = "dual_candidate_search";
      last_failure.failure_reason = "no whole-body candidate combination passed validation";
    }
    return last_failure;
  }

  void ensureFailurePlayback(TaskResult& result, const Eigen::Vector3d& target) const
  {
    if (result.success || !result.diagnostic_frames.empty()) return;
    result.diagnostic_frames = result.frames;
    if (result.diagnostic_frames.empty())
      result.diagnostic_frames.push_back({"FAILED_HOLD: " + result.failure_stage,
        allJoints(*initial_state_), false});
    result.diagnostic = {{"diagnostic_only", true}, {"freeze_at_end", true},
      {"stage", result.failure_stage}, {"reason", result.failure_reason},
      {"snapshot", "last_available_state_no_rejected_configuration"},
      {"target", {target.x(), target.y(), target.z()}}};
  }

  void resetInitialState()
  {
    *initial_state_ = *home_state_;
    // Explicit simulation initial condition, not a motion from a colliding home.
    // The default SRDF home and all hardware/model startup states stay unchanged.
    if (initial_pose_ == "arms_down")
      for (const std::string side : {"left", "right"})
        initial_state_->setVariablePosition(side + "_joint4", 0.0);
    initial_state_->update(true);
  }

  void runSequence()
  {
    const auto started = std::chrono::steady_clock::now();
    const uint64_t generation = ++generation_;
    removed_boxes_.clear();
    resetInitialState();
    display_diagnostic_ = nlohmann::json::object();
    display_failure_frozen_ = false;
    playback_frames_.clear();
    playback_scenes_.clear();
    sequence_playback_ = false;
    const double x = get_parameter("x").as_double();
    requested_arm_ = get_parameter("arm").as_string();
    updateWallTarget(x, 20);
    selectArm("left");
    const auto first_scene = sceneJson(box_center_);
    publishPlanningStarted(generation, box_center_);

    TaskResult total;
    total.success = true;
    nlohmann::json boxes = nlohmann::json::array();
    nlohmann::json rounds_json = nlohmann::json::array();
    int failed_box = -1;
    size_t dual_success_count = 0;
    size_t fallback_count = 0;

    auto append_result = [&](TaskResult& result, const std::vector<int>& ids,
                             bool dual, const std::string& fallback_reason) {
      ensureFailurePlayback(result, box_center_);
      total.metrics.add(result.metrics);
      if (result.success) {
        const bool first_success = total.frames.empty();
        total.execution_duration_s += result.execution_duration_s;
        total.timing_valid = first_success ? result.timing_valid : total.timing_valid && result.timing_valid;
        total.max_velocity = std::max(total.max_velocity, result.max_velocity);
        total.max_acceleration = std::max(total.max_acceleration, result.max_acceleration);
        total.max_jerk = std::max(total.max_jerk, result.max_jerk);
        if (first_success) {
          total.effective_trajectory_variant = result.effective_trajectory_variant;
          total.optimizer_status = result.optimizer_status;
        } else {
          if (total.effective_trajectory_variant != result.effective_trajectory_variant)
            total.effective_trajectory_variant = "mixed";
          if (total.optimizer_status != result.optimizer_status) total.optimizer_status = "mixed";
        }
        if (!result.variant_fallback_reason.empty()) {
          if (!total.variant_fallback_reason.empty()) total.variant_fallback_reason += "; ";
          total.variant_fallback_reason += result.variant_fallback_reason;
        }
      }
      const size_t scene_index = playback_scenes_.size();
      auto context = sceneJson(box_center_);
      context["side"] = dual ? "dual" : side_;
      context["box_ids"] = ids;
      context["dual"] = dual;
      playback_scenes_.push_back(context);
      for (auto& frame : result.frames) frame.scene_index = scene_index;
      for (auto& frame : result.diagnostic_frames) frame.scene_index = scene_index;
      publishTaskResult(generation, box_center_, result, false);
      last_result_["side"] = dual ? "dual" : side_;
      last_result_["box_ids"] = ids;
      last_result_["dual"] = dual;
      if (!fallback_reason.empty()) last_result_["fallback_reason"] = fallback_reason;
      auto segment = last_result_;
      segment["kind"] = "segment";
      segment["sequence"] = true;
      segment["segment_index"] = scene_index;
      segment["frame_begin"] = total.frames.size();
      segment["frame_end"] = total.frames.size() +
        (result.success ? result.frames.size() : result.diagnostic_frames.size());
      segment["scenes"] = playback_scenes_;
      segment["planning_elapsed_ms"] = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      publishJson(segment, true);
      auto record = last_result_;
      record.erase("frames"); record.erase("diagnostic_frames");
      record["frame_begin"] = total.frames.size();
      if (result.success) {
        total.frames.insert(total.frames.end(), result.frames.begin(), result.frames.end());
        const auto& final = result.frames.back();
        for (size_t j = 0; j < all_joint_names_.size(); ++j)
          initial_state_->setVariablePosition(all_joint_names_[j], final.joints[j]);
        initial_state_->clearAttachedBodies();
        initial_state_->update(true);
        removed_boxes_.insert(ids.begin(), ids.end());
      } else {
        total.success = false;
        total.failure_stage = result.failure_stage;
        total.failure_reason = result.failure_reason;
        failed_box = ids.empty() ? -1 : ids.front();
        total.diagnostic_frames = total.frames;
        total.diagnostic_frames.insert(total.diagnostic_frames.end(),
          result.diagnostic_frames.begin(), result.diagnostic_frames.end());
        total.diagnostic = result.diagnostic;
      }
      record["frame_end"] = total.frames.size();
      boxes.push_back(record);
      RCLCPP_INFO(get_logger(), "SEQUENCE_SEGMENT_READY generation=%llu segment=%zu boxes=%s dual=%d success=%d",
        static_cast<unsigned long long>(generation), scene_index,
        nlohmann::json(ids).dump().c_str(), dual, result.success);
      return result.success;
    };

    for (const auto& round : alfa_robot::motion::wallTransferRounds()) {
      nlohmann::json round_record = {{"left_box", round.left_box}, {"right_box", round.right_box},
        {"dual_attempted", round.dual()}, {"dual_success", false}, {"fallback", false}};
      if (round.dual()) {
        TaskResult dual_result;
        std::string dual_failure;
        const bool bottom = round.left_box / 5 == 0;
        for (const bool top : bottom ? std::vector<bool>{true} : std::vector<bool>{false, true}) {
          dual_result = planDualPair(round.left_box, round.right_box, top, x);
          round_record["attempts"].push_back({{"suction_mode", top ? "top" : "front"},
            {"success", dual_result.success}, {"failure_stage", dual_result.failure_stage},
            {"failure_reason", dual_result.failure_reason}});
          if (dual_result.success) break;
          total.metrics.add(dual_result.metrics);
          dual_failure = dual_result.failure_stage + ": " + dual_result.failure_reason;
          RCLCPP_WARN(get_logger(), "DUAL_ATTEMPT boxes=%d,%d suction=%s failed: %s",
            round.left_box, round.right_box, top ? "top" : "front", dual_failure.c_str());
        }
        if (dual_result.success) {
          ++dual_success_count;
          round_record["dual_success"] = true;
          if (!append_result(dual_result, {round.left_box, round.right_box}, true, "")) break;
        } else {
          ++fallback_count;
          round_record["fallback"] = true;
          round_record["fallback_reason"] = dual_failure;
          for (const int id : {round.left_box, round.right_box}) {
            updateWallTarget(x, id);
            requested_arm_ = id == round.left_box ? "left" : "right";
            TaskResult single = planWithFallback(box_center_);
            if (!append_result(single, {id}, false, dual_failure)) break;
          }
        }
      } else {
        const int id = round.left_box >= 0 ? round.left_box : round.right_box;
        updateWallTarget(x, id);
        requested_arm_ = round.left_box >= 0 ? "left" : "right";
        TaskResult single = planWithFallback(box_center_, true);
        if (!append_result(single, {id}, false, "")) {
          rounds_json.push_back(round_record);
          break;
        }
      }
      rounds_json.push_back(round_record);
      if (!total.success) break;
    }

    if (total.frames.empty()) {
      playback_scenes_.push_back(first_scene);
      total.frames.push_back(ReplayFrame{"sequence_stopped", allJoints(*initial_state_), false});
    }
    total.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    publishTaskResult(generation, box_center_, total, false);
    for (auto it = first_scene.begin(); it != first_scene.end(); ++it) last_result_[it.key()] = it.value();
    last_result_["sequence"] = true;
    last_result_["segment_count"] = boxes.size();
    last_result_["frame_begin"] = 0;
    last_result_["frame_end"] = total.success ? total.frames.size() : total.diagnostic_frames.size();
    last_result_["sequence_order"] = alfa_robot::motion::wallSequenceOrder();
    last_result_["transfer_rounds"] = rounds_json;
    last_result_["dual_success_count"] = dual_success_count;
    last_result_["dual_target_count"] = 10;
    last_result_["fallback_count"] = fallback_count;
    last_result_["full_dual_pass"] = total.success && fallback_count == 0 && dual_success_count == 10;
    last_result_["completed_count"] = removed_boxes_.size();
    last_result_["remaining_count"] = 25 - removed_boxes_.size();
    last_result_["final_removed_box_ids"] = removed_boxes_;
    last_result_["failed_box_id"] = failed_box;
    last_result_["boxes"] = boxes;
    last_result_["scenes"] = playback_scenes_;
    last_result_["attempts"] = nlohmann::json::array();
    last_result_["scope"] = "simulation-only geometric dual-arm transfer; no suction dynamics validation";
    last_result_["final_joints"] = allJoints(*initial_state_);
    publishJson(last_result_);
    display_diagnostic_ = total.success ? nlohmann::json::object() : total.diagnostic;
    playback_frames_ = total.success || total.diagnostic_frames.empty() ?
      std::move(total.frames) : std::move(total.diagnostic_frames);
    playback_index_ = 0;
    sequence_playback_ = true;
    display_scene_ = playback_scenes_.front();
    publishStatus("SEQUENCE " + std::string(total.success ? "SUCCESS" : "STOPPED") +
      " completed=" + std::to_string(removed_boxes_.size()) + "/25 dual=" +
      std::to_string(dual_success_count) + "/10 fallback=" + std::to_string(fallback_count), total.success);
  }

  bool requestPlanning()
  {
    if (busy()) return false;
    if (planning_requested_.exchange(true)) {
      return false;
    }
    if (distance_demo_) selectArm(requested_arm_ == "auto" ? "left" : requested_arm_);
    publishStatus("PLANNING REQUESTED", true);
    return true;
  }

  void onWorkerTimer()
  {
    if (!planning_requested_.exchange(false)) {
      return;
    }
    if (planning_active_.exchange(true)) {
      return;
    }
    if (sequence_requested_) {
      sequence_requested_ = false;
      sequence_running_ = true;
      try { runSequence(); }
      catch (const std::exception& error) {
        TaskResult failure;
        failure.failure_stage = "exception";
        failure.failure_reason = error.what();
        publishTaskResult(generation_, box_center_, failure, false);
        last_result_["sequence"] = true;
        last_result_["completed_count"] = removed_boxes_.size();
        last_result_["remaining_count"] = 25 - removed_boxes_.size();
        last_result_["failed_box_id"] = wall_target_row_ * 5 + wall_target_column_;
        publishJson(last_result_);
        publishStatus(std::string("SEQUENCE ERROR: ") + error.what(), false);
        RCLCPP_ERROR(get_logger(), "sequence exception: %s", error.what());
      }
      sequence_running_ = false;
      planning_active_.store(false);
      return;
    }
    removed_boxes_.clear();
    if (wall_context_ == "sequence_prefix") {
      for (int id : alfa_robot::motion::wallSequenceOrder()) {
        if (id == wall_target_row_ * 5 + wall_target_column_) break;
        removed_boxes_.insert(id);
      }
    }
    resetInitialState();
    top_suction_ = distance_demo_ && (requested_suction_mode_ == "top" ||
      alfa_robot::motion::isBottomBox(box_center_.z(), box_height_, wall_bottom_z_));
    playback_scenes_.clear();
    display_scene_ = nlohmann::json();
    display_box_visible_ = true;
    Eigen::Vector3d box_center;
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      box_center = box_center_;
    }
    const uint64_t generation = ++generation_;
    publishPlanningStarted(generation, box_center);
    publishStatus("CALCULATING", true);
    RCLCPP_INFO(
      get_logger(),
      "[%llu] calculation started: box_center=[%.3f, %.3f, %.3f]",
      static_cast<unsigned long long>(generation),
      box_center.x(), box_center.y(), box_center.z());
    if (distance_demo_) {
      const auto alignment = heightAlignment(box_center);
      RCLCPP_INFO(get_logger(),
        "height proposal (clearance unchecked): strategy=%s enabled=%s descent=%.6fm target_updown=%.6fm",
        height_strategy_.c_str(), align_height_ ? "true" : "false",
        alignment.at("descent").get<double>(), alignment.at("target_updown").get<double>());
    }

    {
      std::lock_guard<std::mutex> lock(display_mutex_);
      display_diagnostic_ = nlohmann::json::object();
      display_failure_frozen_ = false;
      playback_frames_.clear();
      display_box_attached_ = false;
      *display_state_ = *initial_state_;
    }
    publishSceneMarkers();
    TaskResult result;
    const auto request_started = std::chrono::steady_clock::now();
    try {
      result = planWithFallback(box_center);
    } catch (const std::exception& error) {
      result.success = false;
      result.failure_stage = "exception";
      result.failure_reason = error.what();
    }
    if (distance_demo_) result.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - request_started).count();
    if (distance_demo_ && !result.success)
      result.frames = {ReplayFrame{"planning_failed", allJoints(*initial_state_), false}};
    ensureFailurePlayback(result, box_center);
    display_scene_ = sceneJson(box_center);
    publishTaskResult(generation, box_center, result);
    if (!result.frames.empty() || !result.diagnostic_frames.empty()) {
      std::lock_guard<std::mutex> lock(display_mutex_);
      playback_frames_ = result.success || result.diagnostic_frames.empty() ? result.frames : result.diagnostic_frames;
      display_diagnostic_ = result.success ? nlohmann::json::object() : result.diagnostic;
      playback_index_ = 0;
    }
    if (result.success) {
      std::ostringstream status;
      status << "SUCCESS total=" << std::fixed << std::setprecision(1)
             << result.total_ms << "ms";
      if (distance_demo_) {
        status << " arm=" << side_ << " lift=" << std::setprecision(3)
               << heightAlignment(box_center).at("target_updown").get<double>() << "m";
      }
      publishStatus(status.str(), true);
      RCLCPP_INFO(
        get_logger(),
        "[%llu] calculation completed: SUCCESS total=%.3fms analytic=%.3fms "
        "rrt_to=%.3fms rrt_return=%.3fms ik_calls=%llu collision_checks=%llu",
        static_cast<unsigned long long>(generation), result.total_ms,
        result.metrics.analytic_path_ms, result.metrics.rrt_approach_ms,
        result.metrics.rrt_return_ms,
        static_cast<unsigned long long>(result.metrics.ik_calls),
        static_cast<unsigned long long>(result.metrics.collision_checks));
    } else {
      publishStatus(
        "FAILED " + result.failure_stage + ": " + result.failure_reason, false);
      RCLCPP_ERROR(
        get_logger(),
        "[%llu] calculation completed: FAILED total=%.3fms stage=%s reason=%s",
        static_cast<unsigned long long>(generation), result.total_ms,
        result.failure_stage.c_str(), result.failure_reason.c_str());
    }
    planning_active_.store(false);
  }

  Eigen::Vector3d boxSize() const { return {box_depth_, box_width_, box_height_}; }

  Eigen::Isometry3d contactPose(
    const Eigen::Vector3d& box_center, const std::string& side, bool top) const
  {
    auto pose = alfa_robot::motion::wallContactPose(box_center, boxSize(), contact_numerical_gap_, top);
    if (top)
      pose.linear() = Eigen::AngleAxisd(side == "left" ? kPi / 2.0 : -kPi / 2.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix() * pose.linear();
    return pose;
  }

  Eigen::Isometry3d contactPose(const Eigen::Vector3d& box_center) const
  {
    return contactPose(box_center, side_, top_suction_);
  }

  Eigen::Isometry3d toolToBox(const std::string& side, bool top) const
  {
    return contactPose(Eigen::Vector3d::Zero(), side, top).inverse();
  }

  Eigen::Isometry3d toolToBox() const { return toolToBox(side_, top_suction_); }
  Eigen::Vector3d toolToBoxCenter() const { return toolToBox().translation(); }

  Eigen::Isometry3d precontactPose(const Eigen::Vector3d& box_center) const
  {
    Eigen::Isometry3d pose = contactPose(box_center);
    pose.translation() -= pose.linear().col(2) * approach_distance_;
    return pose;
  }

  Eigen::Isometry3d retreatPose(const Eigen::Vector3d& box_center) const
  {
    Eigen::Isometry3d pose = contactPose(box_center);
    pose.translation().x() -= retreat_distance_;
    return pose;
  }

  std::vector<Eigen::Vector3d> neighborCenters(const Eigen::Vector3d& center) const
  {
    if (scene_layout_ == "wall_5x5") {
      if (wall_context_ == "target_only") {
        return {};
      }
      std::vector<Eigen::Vector3d> neighbors;
      neighbors.reserve(24);
      for (int row = 0; row < 5; ++row) {
        for (int column = 0; column < 5; ++column) {
          if ((row == wall_target_row_ && column == wall_target_column_) ||
              removed_boxes_.count(row * 5 + column)) continue;
          neighbors.push_back(center + Eigen::Vector3d(
            0.0, (column - wall_target_column_) * (box_width_ + wall_gap_),
            (row - wall_target_row_) * (box_height_ + wall_gap_)));
        }
      }
      return neighbors;
    }
    return {
      center + Eigen::Vector3d(0.0, box_width_, 0.0),
      center - Eigen::Vector3d(0.0, box_width_, 0.0),
      center + Eigen::Vector3d(0.0, 0.0, box_height_),
      center - Eigen::Vector3d(0.0, 0.0, box_height_),
    };
  }

  planning_scene::PlanningScenePtr makeScene(const Eigen::Vector3d& box_center) const
  {
    // Legacy preview omits the target; wall mode keeps it until attachment.
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    scene->setCurrentState(*initial_state_);
    // Shared by initial state, lift, IK, approach, attachment, retreat and loaded RRT.
    // No robot/ground or carried-box/environment ACM exemptions.
    for (const auto& object : environment_objects_) {
      if (!scene->processCollisionObjectMsg(object))
        throw std::runtime_error("failed to add " + object.id + " to planning scene");
    }
    auto neighbors = neighborCenters(box_center);
    if (distance_demo_) neighbors.push_back(box_center);
    const double depth = std::max(0.001, box_depth_ - 2.0 * collision_inset_);
    const double width = std::max(0.001, box_width_ - 2.0 * collision_inset_);
    const double height = std::max(0.001, box_height_ - 2.0 * collision_inset_);
    for (size_t index = 0; index < neighbors.size(); ++index) {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = world_frame_;
      object.id = distance_demo_ && index == neighbors.size() - 1 ?
        kCarriedBoxId : "neighbor_box_" + std::to_string(index);
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {depth, width, height};
      geometry_msgs::msg::Pose pose;
      pose.position.x = neighbors[index].x();
      pose.position.y = neighbors[index].y();
      pose.position.z = neighbors[index].z();
      pose.orientation.w = 1.0;
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(pose);
      if (!scene->processCollisionObjectMsg(object)) {
        throw std::runtime_error("failed to add " + object.id + " to planning scene");
      }
    }
    return scene;
  }

  planning_scene::PlanningScenePtr loadedScene(
    const planning_scene::PlanningSceneConstPtr& scene) const
  {
    auto loaded = planning_scene::PlanningScene::clone(scene);
    if (distance_demo_) loaded->getWorldNonConst()->removeObject(kCarriedBoxId);
    return loaded;
  }

  void attachCarriedBox(
    moveit::core::RobotState& state, const std::string& object_id,
    const std::string& side, bool top) const
  {
    if (state.hasAttachedBody(object_id)) return;
    const Eigen::Vector3d size = boxSize().array() - 2.0 * collision_inset_;
    std::vector<shapes::ShapeConstPtr> shapes{
      std::make_shared<shapes::Box>(size.x(), size.y(), size.z())};
    EigenSTL::vector_Isometry3d shape_poses{toolToBox(side, top)};
    const std::string tool = side + "_tool0";
    const std::string prefix = side + "_";
    state.attachBody(object_id, Eigen::Isometry3d::Identity(), shapes, shape_poses,
      distance_demo_ ? std::vector<std::string>{tool, prefix + "joint7"} :
        std::vector<std::string>{tool, prefix + "joint7", prefix + "joint6"}, tool);
    state.update(true);
  }

  void attachCarriedBox(moveit::core::RobotState& state) const
  {
    attachCarriedBox(state, kCarriedBoxId, side_, top_suction_);
  }

  std::array<double, 7> armJoints(const moveit::core::RobotState& state) const
  {
    std::vector<double> values;
    state.copyJointGroupPositions(planning_group_, values);
    if (values.size() != 7U) {
      throw std::runtime_error("unexpected arm joint count");
    }
    std::array<double, 7> output{};
    std::copy(values.begin(), values.end(), output.begin());
    return output;
  }

  std::vector<double> allJoints(const moveit::core::RobotState& state) const
  {
    std::vector<double> output;
    output.reserve(all_joint_names_.size());
    for (const auto& name : all_joint_names_) {
      output.push_back(state.getVariablePosition(name));
    }
    return output;
  }

  bool validateSingleArmFrames(
    const std::vector<ReplayFrame>& frames,
    const planning_scene::PlanningSceneConstPtr& empty_scene,
    const planning_scene::PlanningSceneConstPtr& loaded_scene,
    PlanningMetrics* metrics,
    std::string* reason) const
  {
    const auto fullCollisionReason = [&](const planning_scene::PlanningSceneConstPtr& scene,
                                         const moveit::core::RobotState& state) {
      const auto started = std::chrono::steady_clock::now();
      const std::string collision = alfa_robot::motion::scene_collision_reason(scene, state, nullptr);
      if (metrics) {
        ++metrics->collision_checks;
        metrics->collision_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      }
      return collision;
    };

    moveit::core::RobotStatePtr previous;
    bool previous_attached = false;
    std::vector<double> previous_joints;
    for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
      const auto& frame = frames[frame_index];
      if (frame.joints.size() != all_joint_names_.size()) {
        if (reason) *reason = "whole_body_joint_count at frame " + std::to_string(frame_index);
        return false;
      }
      moveit::core::RobotState state(*initial_state_);
      for (size_t joint = 0; joint < all_joint_names_.size(); ++joint)
        state.setVariablePosition(all_joint_names_[joint], frame.joints[joint]);
      if (frame.box_attached) {
        attachCarriedBox(state);
      } else if (state.hasAttachedBody(kCarriedBoxId)) {
        state.clearAttachedBody(kCarriedBoxId);
      }
      state.update(true);
      if (!state.satisfiesBounds()) {
        if (reason) *reason = "whole_body_joint_bounds at frame " + std::to_string(frame_index);
        return false;
      }
      const std::string collision = fullCollisionReason(
        frame.box_attached ? loaded_scene : empty_scene, state);
      if (!collision.empty()) {
        if (reason) *reason = "whole_body_" + collision + " at frame " + std::to_string(frame_index);
        return false;
      }
      if (previous) {
        if (!alfa_robot::motion::sameShoulderElbowBranch(
              sideJoints(*previous, "left"), sideJoints(state, "left")) ||
            !alfa_robot::motion::sameShoulderElbowBranch(
              sideJoints(*previous, "right"), sideJoints(state, "right"))) {
          if (reason) *reason = "whole_body_shoulder_elbow_branch_flip at frame " +
            std::to_string(frame_index - 1) + " -> " + std::to_string(frame_index);
          return false;
        }
        if (frame.box_attached != previous_attached) {
          if (frame.joints != previous_joints) {
            if (reason) *reason = "attachment_transition_changes_joints at frame " +
              std::to_string(frame_index - 1) + " -> " + std::to_string(frame_index);
            return false;
          }
        } else {
          double maximum_delta = 0.0;
          for (const auto& name : all_joint_names_)
            maximum_delta = std::max(maximum_delta, std::abs(
              state.getVariablePosition(name) - previous->getVariablePosition(name)));
          const size_t steps = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_resolution_)));
          for (size_t step = 1; step < steps; ++step) {
            moveit::core::RobotState probe(*previous);
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            for (const auto& name : all_joint_names_)
              probe.setVariablePosition(name, previous->getVariablePosition(name) +
                (state.getVariablePosition(name) - previous->getVariablePosition(name)) * ratio);
            probe.update(true);
            if (!probe.satisfiesBounds()) {
              if (reason) *reason = "whole_body_edge_joint_bounds at frame " +
                std::to_string(frame_index - 1) + " -> " + std::to_string(frame_index);
              return false;
            }
            const std::string edge_collision = fullCollisionReason(
              frame.box_attached ? loaded_scene : empty_scene, probe);
            if (!edge_collision.empty()) {
              if (reason) *reason = "whole_body_edge_" + edge_collision + " at frame " +
                std::to_string(frame_index - 1) + " -> " +
                std::to_string(frame_index);
              return false;
            }
          }
        }
      }
      previous = std::make_shared<moveit::core::RobotState>(state);
      previous_attached = frame.box_attached;
      previous_joints = frame.joints;
    }
    return true;
  }

  double naturalPathLength(const std::vector<ReplayFrame>& frames) const
  {
    constexpr std::array<double, 7> weights{1.0, 1.0, 1.0, 1.0, 2.0, 3.0, 5.0};
    double length = 0.0;
    for (size_t i = 1; i < frames.size(); ++i) {
      if (frames[i - 1].joints.size() != all_joint_names_.size() ||
          frames[i].joints.size() != all_joint_names_.size()) continue;
      double squared = 0.0;
      for (size_t arm = 0; arm < 2; ++arm)
        for (size_t joint = 0; joint < weights.size(); ++joint) {
          const size_t index = arm * weights.size() + joint;
          const double delta = alfa_robot::motion::shortestAngleDelta(
            frames[i - 1].joints[index], frames[i].joints[index]);
          squared += weights[joint] * delta * delta;
        }
      for (size_t index = 14; index < frames[i].joints.size(); ++index) {
        const double delta = frames[i].joints[index] - frames[i - 1].joints[index];
        squared += delta * delta;
      }
      length += std::sqrt(squared);
    }
    return length;
  }

  TaskResult selectBestCandidate(
    std::vector<TaskResult> candidates, const PlanningMetrics& metrics, double total_ms = 0.0) const
  {
    if (candidates.empty()) return {};
    std::stable_sort(candidates.begin(), candidates.end(), [](const TaskResult& left, const TaskResult& right) {
      return left.selection_score < right.selection_score;
    });
    TaskResult selected = std::move(candidates.front());
    selected.metrics.add(metrics);
    selected.total_ms = total_ms;
    selected.complete_candidate_count = candidates.size();
    selected.selected_candidate_rank = 1;
    return selected;
  }

  bool postprocessCandidate(
    TaskResult* candidate,
    const planning_scene::PlanningSceneConstPtr& empty_scene,
    const planning_scene::PlanningSceneConstPtr& loaded_scene,
    const std::string& chomp_group,
    const alfa_robot::motion::EdgeValidator& edge_validator,
    const alfa_robot::motion::PathValidator& path_validator,
    std::string* reason)
  {
    alfa_robot::motion::TrajectoryPostprocessOptions options;
    options.variant = trajectory_variant_;
    options.sample_period = trajectory_sample_period_;
    options.empty_velocity_scaling = empty_velocity_scaling_;
    options.empty_acceleration_scaling = empty_acceleration_scaling_;
    options.loaded_velocity_scaling = loaded_velocity_scaling_;
    options.loaded_acceleration_scaling = loaded_acceleration_scaling_;
    options.arm_max_jerk = arm_max_jerk_;
    options.head_max_jerk = head_max_jerk_;
    options.updown_max_jerk = updown_max_jerk_;
    alfa_robot::motion::TrajectoryPostprocessMetrics post;
    const auto chomp = [&](std::vector<ReplayFrame>* frames, double* wall_ms, std::string* chomp_reason) {
      return alfa_robot::motion::optimizeChompFreeSpace(
        frames, robot_model_, empty_scene, loaded_scene, all_joint_names_, chomp_group,
        wall_ms, chomp_reason);
    };
    const bool success = alfa_robot::motion::postprocessWallTrajectory(
      &candidate->frames, robot_model_, all_joint_names_, options, edge_validator, path_validator, chomp,
      &post, reason);
    candidate->metrics.shortcut_ms += post.shortcut_ms;
    candidate->metrics.chomp_ms += post.chomp_ms;
    candidate->metrics.totg_ms += post.totg_ms;
    candidate->metrics.ruckig_ms += post.ruckig_ms;
    if (!success) return false;
    candidate->execution_duration_s = post.execution_duration_s;
    candidate->timing_valid = post.timing_valid;
    candidate->effective_trajectory_variant = post.effective_variant;
    candidate->optimizer_status = post.optimizer_status;
    candidate->variant_fallback_reason = post.fallback_reason;
    candidate->max_velocity = post.max_velocity;
    candidate->max_acceleration = post.max_acceleration;
    candidate->max_jerk = post.max_jerk;
    return true;
  }

  std::string collisionReason(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& state,
    PlanningMetrics* metrics) const
  {
    const auto started = std::chrono::steady_clock::now();
    const std::string reason = alfa_robot::motion::scene_collision_reason(
      scene, state, distance_demo_ ? nullptr : planning_group_);
    if (metrics) {
      ++metrics->collision_checks;
      metrics->collision_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    }
    return reason;
  }

  bool edgeClear(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to,
    bool attached,
    PlanningMetrics* metrics,
    std::string* reason,
    moveit::core::RobotStatePtr* rejected = nullptr) const
  {
    const auto from_joints = armJoints(from);
    const auto to_joints = armJoints(to);
    double maximum_delta = maximumJointDelta(from_joints, to_joints);
    if (distance_demo_) {
      maximum_delta = 0.0;
      for (size_t i = 0; i < from_joints.size(); ++i)
        maximum_delta = std::max(maximum_delta, std::abs(to_joints[i] - from_joints[i]));
    }
    const size_t steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_resolution_)));
    for (size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      std::array<double, 7> interpolated{};
      for (size_t index = 0; index < interpolated.size(); ++index) {
        interpolated[index] = from_joints[index] +
          (distance_demo_ ? to_joints[index] - from_joints[index] :
           normalizedAngle(to_joints[index] - from_joints[index])) * ratio;
      }
      moveit::core::RobotState probe(from);
      probe.setJointGroupPositions(planning_group_, interpolated.data());
      if (attached && !probe.hasAttachedBody(kCarriedBoxId)) {
        attachCarriedBox(probe);
      } else if (!attached && probe.hasAttachedBody(kCarriedBoxId)) {
        probe.clearAttachedBody(kCarriedBoxId);
      }
      probe.update(true);
      if (!probe.satisfiesBounds(planning_group_)) {
        if (reason) *reason = "joint_bounds";
        if (rejected) *rejected = std::make_shared<moveit::core::RobotState>(probe);
        return false;
      }
      const std::string collision = collisionReason(scene, probe, metrics);
      if (!collision.empty()) {
        if (reason) *reason = collision;
        if (rejected) *rejected = std::make_shared<moveit::core::RobotState>(probe);
        return false;
      }
    }
    return true;
  }

  RrtPlanResult moveUpdown(
    const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::RobotState& start,
    double target,
    bool attached,
    PlanningMetrics* metrics) const
  {
    const auto started = std::chrono::steady_clock::now();
    RrtPlanResult result;
    result.states.push_back(std::make_shared<moveit::core::RobotState>(start));
    const double initial = start.getVariablePosition("updown");
    const size_t steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(std::abs(target - initial) / 0.005)));
    for (size_t index = 1; index <= steps; ++index) {
      auto state = std::make_shared<moveit::core::RobotState>(start);
      state->setVariablePosition(
        "updown",
        initial + (target - initial) * static_cast<double>(index) /
        static_cast<double>(steps));
      if (attached && !state->hasAttachedBody(kCarriedBoxId)) {
        attachCarriedBox(*state);
      }
      state->update(true);
      if (!state->satisfiesBounds()) {
        result.reason = "joint_bounds";
        result.rejected_state = state;
        break;
      }
      const std::string collision = collisionReason(scene, *state, metrics);
      if (!collision.empty()) {
        result.reason = collision;
        result.rejected_state = state;
        break;
      }
      result.states.push_back(std::move(state));
    }
    result.success = result.states.size() == steps + 1U;
    result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    return result;
  }

  std::vector<moveit::core::RobotStatePtr> directArmPath(
    const moveit::core::RobotState& from, const moveit::core::RobotState& to) const
  {
    const auto from_joints = armJoints(from);
    const auto to_joints = armJoints(to);
    std::array<double, 7> deltas{};
    double maximum_delta = 0.0;
    for (size_t i = 0; i < deltas.size(); ++i) {
      deltas[i] = distance_demo_ ? to_joints[i] - from_joints[i] :
        normalizedAngle(to_joints[i] - from_joints[i]);
      maximum_delta = std::max(maximum_delta, std::abs(deltas[i]));
    }
    const size_t steps = std::max<size_t>(1, std::ceil(maximum_delta / edge_joint_resolution_));
    std::vector<moveit::core::RobotStatePtr> states;
    states.reserve(steps + 1);
    for (size_t step = 0; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      std::array<double, 7> joints{};
      for (size_t i = 0; i < joints.size(); ++i)
        joints[i] = from_joints[i] + deltas[i] * ratio;
      auto state = std::make_shared<moveit::core::RobotState>(from);
      state->setJointGroupPositions(planning_group_, joints.data());
      state->update(true);
      states.push_back(std::move(state));
    }
    return states;
  }

  std::vector<AnalyticCandidate> solvePoseCandidates(
    const Eigen::Isometry3d& target_world,
    const moveit::core::RobotState& seed_state,
    bool attached,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    bool enforce_step,
    std::string* rejection_summary,
    moveit::core::RobotStatePtr* rejected = nullptr) const
  {
    const Eigen::Isometry3d world_to_arm_base =
      seed_state.getGlobalLinkTransform(arm_base_link_).inverse();
    const Eigen::Isometry3d target_in_arm_base = world_to_arm_base * target_world;
    const auto seed_joints = armJoints(seed_state);
    std::vector<AnalyticCandidate> candidates;
    size_t bounds_rejects = 0;
    size_t jump_rejects = 0;
    size_t collision_rejects = 0;
    std::string last_collision;
    moveit::core::RobotStatePtr last_rejected;

    const int intervals = std::max(1, static_cast<int>(std::ceil(2.0 * kPi / psi_step_)));
    for (int index = 0; index < intervals; ++index) {
      V3RedundantIkRequest request;
      request.target_in_arm_base = target_in_arm_base;
      request.swivel_angle = -kPi + static_cast<double>(index) * 2.0 * kPi / intervals;
      request.seed = seed_joints;
      const auto started = std::chrono::steady_clock::now();
      const auto solutions = solver_->solveInArmBase(request);
      if (metrics) {
        ++metrics->ik_calls;
        metrics->ik_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      }
      for (const auto& solution : solutions) {
        if (enforce_step &&
            (maximumJointDelta(seed_joints, solution.joints) > maximum_cartesian_joint_step_ ||
             !alfa_robot::motion::sameShoulderElbowBranch(seed_joints, solution.joints))) {
          ++jump_rejects;
          continue;
        }
        moveit::core::RobotState candidate(seed_state);
        candidate.setJointGroupPositions(planning_group_, solution.joints.data());
        if (attached) {
          attachCarriedBox(candidate);
        } else if (candidate.hasAttachedBody(kCarriedBoxId)) {
          candidate.clearAttachedBody(kCarriedBoxId);
        }
        candidate.update(true);
        if (!candidate.satisfiesBounds(planning_group_)) {
          ++bounds_rejects;
          continue;
        }
        const std::string collision = collisionReason(scene, candidate, metrics);
        if (!collision.empty()) {
          ++collision_rejects;
          last_collision = collision;
          last_rejected = std::make_shared<moveit::core::RobotState>(candidate);
          continue;
        }
        const bool duplicate = std::any_of(
          candidates.begin(), candidates.end(),
          [&solution](const AnalyticCandidate& existing) {
            return maximumJointDelta(existing.solution.joints, solution.joints) < 1e-5;
          });
        if (duplicate) {
          continue;
        }
        AnalyticCandidate output;
        output.state = std::make_shared<moveit::core::RobotState>(candidate);
        output.solution = solution;
        const double margin_penalty = 0.02 /
          std::max(0.01, solution.minimum_joint_limit_margin);
        output.score = alfa_robot::motion::naturalJointDistanceSquared(seed_joints, solution.joints) + margin_penalty;
        candidates.push_back(std::move(output));
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const AnalyticCandidate& lhs, const AnalyticCandidate& rhs) {
        return lhs.score < rhs.score;
      });
    if (rejection_summary) {
      std::ostringstream summary;
      summary << "candidates=" << candidates.size()
              << " bounds=" << bounds_rejects
              << " jump=" << jump_rejects
              << " collision=" << collision_rejects
             ;
      if (!last_collision.empty()) {
        summary << " last=" << last_collision;
      }
      *rejection_summary = summary.str();
    }
    if (rejected) *rejected = candidates.empty() ? last_rejected : nullptr;
    return candidates;
  }

  std::optional<AnalyticCandidate> solveNextPose(
    const Eigen::Isometry3d& target_world,
    const moveit::core::RobotState& seed_state,
    bool attached,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    std::string* reason,
    moveit::core::RobotStatePtr* rejected = nullptr) const
  {
    auto candidates = solvePoseCandidates(
      target_world, seed_state, attached, scene, metrics, true, reason, rejected);
    // Only the first usable Cartesian candidate is consumed. Check edges in
    // score order instead of densely checking every unused swivel solution.
    for (const auto& candidate : candidates) {
      std::string edge_reason;
      if (edgeClear(scene, seed_state, *candidate.state, attached, metrics, &edge_reason, rejected)) {
        if (rejected) rejected->reset();
        return candidate;
      }
      if (reason) *reason = "cartesian candidate edge rejected: " + edge_reason;
    }
    return std::nullopt;
  }

  bool traceCartesianPath(
    const Eigen::Vector3d& box_center,
    const moveit::core::RobotState& precontact_state,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    std::vector<moveit::core::RobotStatePtr>* approach_states,
    std::vector<moveit::core::RobotStatePtr>* retreat_states,
    std::string* failure_stage,
    std::string* failure_reason,
    TaskResult* diagnostic_result) const
  {
    if (!approach_states || !retreat_states) {
      return false;
    }
    approach_states->clear();
    retreat_states->clear();
    approach_states->push_back(std::make_shared<moveit::core::RobotState>(precontact_state));
    moveit::core::RobotStatePtr current = approach_states->back();
    const Eigen::Isometry3d contact = contactPose(box_center);
    auto contact_scene = planning_scene::PlanningScene::clone(scene);
    if (distance_demo_) {
      contact_scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxId, tool_link_, true);
      contact_scene->getAllowedCollisionMatrixNonConst().setEntry(kCarriedBoxId, side_ + "_joint7", true);
    }
    const auto loaded = loadedScene(scene);
    const size_t approach_steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(approach_distance_ / cartesian_step_)));
    for (size_t step = 1; step <= approach_steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(approach_steps);
      Eigen::Isometry3d target = contact;
      target.translation() -= contact.linear().col(2) * approach_distance_ * (1.0 - ratio);
      std::string reason;
      auto next = solveNextPose(target, *current, false,
        distance_demo_ && step == approach_steps ? contact_scene : scene, metrics, &reason,
        &diagnostic_result->rejected_state);
      if (!next) {
        diagnostic_result->diagnostic["target"] = {target.translation().x(), target.translation().y(), target.translation().z()};
        if (failure_stage) *failure_stage = "cartesian_approach";
        if (failure_reason) {
          *failure_reason = "step " + std::to_string(step) + "/" +
            std::to_string(approach_steps) + " " + reason;
        }
        return false;
      }
      current = next->state;
      approach_states->push_back(current);
    }

    moveit::core::RobotState contact_attached(*current);
    attachCarriedBox(contact_attached);
    const std::string attach_collision = collisionReason(loaded, contact_attached, metrics);
    if (!attach_collision.empty()) {
      diagnostic_result->rejected_state = std::make_shared<moveit::core::RobotState>(contact_attached);
      if (failure_stage) *failure_stage = "attach_box";
      if (failure_reason) *failure_reason = attach_collision;
      return false;
    }
    current = std::make_shared<moveit::core::RobotState>(contact_attached);
    retreat_states->push_back(current);

    const size_t horizontal_steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(retreat_distance_ / cartesian_step_)));
    const size_t lift_steps = top_suction_ ? std::max<size_t>(
      1, static_cast<size_t>(std::ceil(approach_distance_ / cartesian_step_))) : 0;
    const size_t retreat_steps = lift_steps + horizontal_steps;
    for (size_t step = 1; step <= retreat_steps; ++step) {
      Eigen::Isometry3d target = contact;
      // Lift off the support while backing away from the flush rear wall.
      // A purely vertical joint-interpolated segment can tip a box corner into
      // that wall; preserve both obstacles and create clearance through motion.
      const double backoff = top_suction_ ? std::min(0.02, retreat_distance_) : 0.0;
      if (lift_steps) {
        const double lifted = static_cast<double>(std::min(step, lift_steps)) / lift_steps;
        target.translation().z() += approach_distance_ * lifted;
        target.translation().x() -= backoff * lifted;
      }
      if (step > lift_steps) target.translation().x() -= (retreat_distance_ - backoff) *
        static_cast<double>(step - lift_steps) / horizontal_steps;
      std::string reason;
      auto next = solveNextPose(target, *current, true, loaded, metrics, &reason,
        &diagnostic_result->rejected_state);
      if (!next) {
        diagnostic_result->diagnostic["target"] = {target.translation().x(), target.translation().y(), target.translation().z()};
        if (failure_stage) *failure_stage = step <= lift_steps ? "cartesian_lift" : "cartesian_retreat";
        if (failure_reason) {
          *failure_reason = "step " + std::to_string(step) + "/" +
            std::to_string(retreat_steps) + " " + reason;
        }
        return false;
      }
      current = next->state;
      retreat_states->push_back(current);
    }
    return true;
  }

  RrtPlanResult planRrt(
    const planning_scene::PlanningSceneConstPtr& base_scene,
    const moveit::core::RobotState& start_state,
    const moveit::core::RobotState& goal_state) const
  {
    RrtPlanResult result;
    const auto wall_started = std::chrono::steady_clock::now();
    auto scene = planning_scene::PlanningScene::clone(base_scene);
    scene->setCurrentState(start_state);
    const std::string start_collision = alfa_robot::motion::scene_collision_reason(
      scene, start_state, distance_demo_ ? nullptr : planning_group_);
    if (!start_collision.empty()) {
      result.rejected_state = std::make_shared<moveit::core::RobotState>(start_state);
      result.reason = "start_" + start_collision;
      return result;
    }
    const std::string goal_collision = alfa_robot::motion::scene_collision_reason(
      scene, goal_state, distance_demo_ ? nullptr : planning_group_);
    if (!goal_collision.empty()) {
      result.rejected_state = std::make_shared<moveit::core::RobotState>(goal_state);
      result.reason = "goal_" + goal_collision;
      return result;
    }

    planning_interface::MotionPlanRequest request;
    request.group_name = planning_group_name_;
    request.planner_id = "RRTConnectkConfigDefault";
    request.allowed_planning_time = rrt_planning_time_;
    request.num_planning_attempts = rrt_planning_attempts_;
    request.max_velocity_scaling_factor = 1.0;
    request.max_acceleration_scaling_factor = 1.0;
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state, true);
    request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal_state, planning_group_, 1e-3));

    planning_interface::MotionPlanResponse response;
    const bool generated = planning_pipeline_->generatePlan(scene, request, response);
    result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_started).count();
    result.planner_ms = response.planning_time_ * 1000.0;
    if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        !response.trajectory_) {
      result.reason = "RRTConnect code=" + std::to_string(response.error_code_.val) + " " +
        alfa_robot::motion::direct_pipeline_failure_diagnostic(
          scene, start_state, goal_state, planning_group_);
      return result;
    }
    for (size_t index = 0; index < response.trajectory_->getWayPointCount(); ++index) {
      result.states.push_back(std::make_shared<moveit::core::RobotState>(
        response.trajectory_->getWayPoint(index)));
    }
    if (result.states.empty()) {
      result.reason = "RRTConnect returned empty trajectory";
      return result;
    }
    if (distance_demo_) {
      const bool attached = start_state.hasAttachedBody(kCarriedBoxId);
      for (const auto& state : result.states) {
        if (!state->satisfiesBounds() || state->hasAttachedBody(kCarriedBoxId) != attached) {
          result.rejected_state = state;
          result.reason = "rrt_invalid_bounds_or_attachment attached=" +
            std::to_string(state->hasAttachedBody(kCarriedBoxId)) + " expected=" + std::to_string(attached);
          for (const auto& name : all_joint_names_) {
            const auto& bounds = robot_model_->getVariableBounds(name);
            const double value = state->getVariablePosition(name);
            if (bounds.position_bounded_ && (value < bounds.min_position_ || value > bounds.max_position_))
              result.reason += " " + name + "=" + std::to_string(value);
          }
          return result;
        }
        for (const auto& name : all_joint_names_) {
          if (std::find(planning_group_->getVariableNames().begin(),
              planning_group_->getVariableNames().end(), name) == planning_group_->getVariableNames().end() &&
              std::abs(state->getVariablePosition(name) - start_state.getVariablePosition(name)) > 1e-8) {
            result.rejected_state = state;
            result.reason = "rrt_changed_fixed_joint";
            return result;
          }
        }
      }
      // Check and replay exact boundary bridges, not an adapter-repaired teleport.
      result.states.insert(result.states.begin(), std::make_shared<moveit::core::RobotState>(start_state));
      result.states.push_back(std::make_shared<moveit::core::RobotState>(goal_state));
      for (size_t i = 1; i < result.states.size(); ++i) {
        if (!edgeClear(scene, *result.states[i-1], *result.states[i],
                       start_state.hasAttachedBody(kCarriedBoxId), nullptr, &result.reason, &result.rejected_state)) {
          result.reason = "rrt_edge_" + result.reason;
          return result;
        }
      }
    }
    std::vector<std::array<double, 7>> natural_path;
    natural_path.reserve(result.states.size());
    for (const auto& state : result.states) natural_path.push_back(armJoints(*state));
    if (!alfa_robot::motion::naturalJointPath(natural_path, 8.0, 3.0)) {
      result.reason = "rrt_unnatural_branch_flip_or_detour";
      result.success = false;
      return result;
    }
    result.success = true;
    return result;
  }

  void appendStates(
    const std::vector<moveit::core::RobotStatePtr>& states,
    const std::string& stage,
    bool attached,
    bool skip_first,
    std::vector<ReplayFrame>* frames) const
  {
    if (!frames) return;
    for (size_t index = skip_first && !states.empty() ? 1U : 0U; index < states.size(); ++index) {
      ReplayFrame frame;
      frame.stage = stage;
      frame.joints = allJoints(*states[index]);
      frame.box_attached = attached;
      frames->push_back(std::move(frame));
    }
  }

  nlohmann::json heightAlignment(const Eigen::Vector3d& box_center) const
  {
    const double current_shoulder_z = distance_demo_ ?
      (initial_state_->getGlobalLinkTransform(arm_base_link_) * solver_->modelShoulderCenterInArmBase()).z() : initial_shoulder_z_;
    const double difference = current_shoulder_z - (box_center.z() + shoulder_box_offset_);
    const double descent = align_height_ ? std::max(0.0, difference) : 0.0;
    const double initial = initial_state_->getVariablePosition("updown");
    const auto& bounds = robot_model_->getVariableBounds("updown");
    if (distance_demo_ && top_suction_) {
      // The 23.8cm wrist-to-TCP extension points DOWN for top suction. A TCP-based
      // radius suitable for front suction incorrectly lowers the shoulder below the wrist.
      const auto shoulder = (initial_state_->getGlobalLinkTransform(arm_base_link_) *
        solver_->modelShoulderCenterInArmBase()).eval();
      const auto seed = armJoints(*initial_state_);
      const auto tcp = solver_->forwardInArmBase(seed);
      const Eigen::Vector3d tcp_to_wrist = tcp.linear().transpose() *
        (solver_->wristCenterInArmBase(seed) - tcp.translation());
      const Eigen::Vector3d wrist = contactPose(box_center) * tcp_to_wrist;
      const double lower = height_clearance_.value("lower", bounds.min_position_);
      const double upper = height_clearance_.value("upper", bounds.max_position_);
      const double wrist_aligned = initial + wrist.z() - shoulder.z();
      const double ideal = wrist_aligned + top_shoulder_above_wrist_;
      const double selected = !align_height_ ? initial :
        synchronized_updown_.value_or(std::clamp(ideal, lower, upper));
      const double xy = (wrist - shoulder).head<2>().norm();
      return {{"enabled", align_height_}, {"strategy", "top_wrist_alignment"}, {"arm", side_},
        {"reference", "independent top-suction shoulder-above-wrist height; vertical tool offset included"},
        {"shoulder_above_wrist", top_shoulder_above_wrist_},
        {"return_policy", "checked rear placement and release; retain final posture for next box"},
        {"target_world", {wrist.x(), wrist.y(), wrist.z()}},
        {"initial_updown", initial}, {"target_updown", selected}, {"ideal_updown", ideal},
        {"dual_shared_updown", synchronized_updown_.has_value()},
        {"descent", initial - selected}, {"lower_limit", bounds.min_position_},
        {"upper_limit", bounds.max_position_}, {"reachable_lift", height_clearance_},
        {"collision_sample_step_m", 0.005}, {"xy", xy}, {"arm_length", solver_->modelArmLength()},
        {"actual_ratio", std::hypot(xy, selected - wrist_aligned) / solver_->modelArmLength()},
        {"inside_band", std::hypot(xy, selected - wrist_aligned) <= solver_->modelArmLength()},
        {"outside_reason", xy > solver_->modelArmLength() ? "top_wrist_xy_beyond_arm_reach" :
          (std::hypot(xy, selected - wrist_aligned) > solver_->modelArmLength() ? "top_wrist_lift_clearance_limited" : "")}};
    }
    if (distance_demo_ && height_strategy_ == "comfort_radius") {
      const auto shoulder = (initial_state_->getGlobalLinkTransform(arm_base_link_) *
        solver_->modelShoulderCenterInArmBase()).eval();
      const Eigen::Vector3d target = contactPose(box_center).translation();
      const Eigen::Vector3d delta = target - shoulder;
      const double xy = delta.head<2>().norm();
      const double length = solver_->modelArmLength();
      auto choice = alfa_robot::motion::chooseComfortHeight(shoulder.z(), target.z(), xy,
        length, initial, height_clearance_.value("lower", bounds.min_position_),
        height_clearance_.value("upper", bounds.max_position_),
        comfort_min_, comfort_preferred_, comfort_max_, comfort_branch_);
      if (!align_height_) {
        choice.position = choice.ideal_position = initial;
        choice.ratio = delta.norm() / length;
        choice.projected = false;
      } else if (synchronized_updown_) {
        choice.position = *synchronized_updown_;
        const double dz = shoulder.z() + choice.position - initial - target.z();
        choice.ratio = std::hypot(xy, dz) / length;
        choice.projected = std::abs(choice.position - choice.ideal_position) > 1e-9;
      }
      const double final_dz = shoulder.z() + choice.position - initial - target.z();
      const bool inside = choice.ratio >= comfort_min_ - 1e-9 && choice.ratio <= comfort_max_ + 1e-9;
      return {{"enabled", align_height_}, {"strategy", height_strategy_}, {"arm", side_},
        {"reference", "selected arm shoulder common-axis center to contact TCP in world"},
        {"shoulder_world", {shoulder.x(), shoulder.y(), shoulder.z()}},
        {"target_world", {target.x(), target.y(), target.z()}},
        {"delta_world", {delta.x(), delta.y(), delta.z()}}, {"xy", xy},
        {"arm_length", length}, {"initial_ratio", delta.norm() / length},
        {"ratio_min", comfort_min_}, {"ratio_preferred", comfort_preferred_},
        {"ratio_max", comfort_max_}, {"actual_ratio", choice.ratio}, {"inside_band", inside},
        {"branch_policy", comfort_branch_}, {"branch", final_dz >= -1e-9 ? "above" : "below"},
        {"ideal_updown", choice.ideal_position}, {"projected", choice.projected},
        {"dual_shared_updown", synchronized_updown_.has_value()},
        {"outside_reason", inside ? "" : (!align_height_ ? "alignment_disabled" :
          (xy > comfort_max_ * length ? "xy_exceeds_band" :
            (height_clearance_.value("checked", false) ? "reachable_lift_limits" : "lift_limits")))},
        {"initial_shoulder_z", shoulder.z()}, {"box_center_z", box_center.z()},
        {"descent", initial - choice.position}, {"initial_updown", initial},
        {"target_updown", choice.position}, {"lower_limit", bounds.min_position_},
        {"upper_limit", bounds.max_position_}, {"reachable_lift", height_clearance_},
        {"collision_sample_step_m", 0.005},
        {"return_policy", post_extract_policy_}};
    }
    return {{"enabled", align_height_}, {"strategy", "fixed_offset"},
      {"reference", "midpoint of left/right shoulder common-axis centers in world Z"},
      {"initial_shoulder_z", current_shoulder_z}, {"box_center_z", box_center.z()},
      {"shoulder_box_offset", shoulder_box_offset_}, {"height_difference", difference},
      {"descent", descent}, {"initial_updown", initial}, {"target_updown", initial - descent},
      {"lower_limit", bounds.min_position_}, {"upper_limit", bounds.max_position_},
      {"collision_sample_step_m", 0.005}, {"return_policy", post_extract_policy_}};
  }

  void checkHeightClearance(
    const planning_scene::PlanningScenePtr& scene, const moveit::core::RobotState& lift_start,
    PlanningMetrics& metrics)
  {
    if (!distance_demo_ || !align_height_ || (!top_suction_ && height_strategy_ != "comfort_radius")) return;
    const double initial = initial_state_->getVariablePosition("updown");
    const auto& bounds = robot_model_->getVariableBounds("updown");
    height_clearance_ = {{"checked", true}, {"lower", initial}, {"upper", initial},
      {"blocked", nlohmann::json::array()}};
    // Conservatively sample the component reachable from home. Never skip an
    // obstacle to select a lower safe island. Same <=5mm sampling as replay validation.
    for (const auto& endpoint : {std::make_pair("lower", bounds.min_position_),
                                 std::make_pair("upper", bounds.max_position_)}) {
      moveit::core::RobotState state(lift_start);
      const size_t steps = static_cast<size_t>(std::ceil(std::abs(endpoint.second - initial) / 0.005));
      for (size_t step = 1; step <= steps; ++step) {
        const double q = initial + (endpoint.second - initial) * step / steps;
        state.setVariablePosition("updown", q);
        state.update(true);
        const auto collision = collisionReason(scene, state, &metrics);
        if (!state.satisfiesBounds() || !collision.empty()) {
          height_clearance_["blocked"].push_back({{"direction", endpoint.first},
            {"updown", q}, {"reason", collision.empty() ? "joint bounds violated" : collision}});
          break;
        }
        height_clearance_[endpoint.first] = q;
      }
    }
  }

  bool alignHeight(
    const Eigen::Vector3d& box_center, const planning_scene::PlanningScenePtr& scene,
    moveit::core::RobotState& grasp_start, TaskResult& result) const
  {
    if (!distance_demo_ || !align_height_) return true;
    const auto alignment = heightAlignment(box_center);
    const double target = alignment.at("target_updown").get<double>();
    const auto& bounds = robot_model_->getVariableBounds("updown");
    if (height_clearance_.value("checked", false) &&
        (target < height_clearance_.at("lower").get<double>() - 1e-9 ||
         target > height_clearance_.at("upper").get<double>() + 1e-9)) {
      result.failure_stage = "height_alignment_clearance";
      result.failure_reason = "shared updown=" + std::to_string(target) +
        " outside collision-free interval [" +
        std::to_string(height_clearance_.at("lower").get<double>()) + ", " +
        std::to_string(height_clearance_.at("upper").get<double>()) + "] metres";
      return false;
    }
    if (target < bounds.min_position_ || target > bounds.max_position_) {
      result.failure_stage = "height_alignment_limits";
      result.failure_reason = "required updown=" + std::to_string(target) +
        " outside [" + std::to_string(bounds.min_position_) + ", " +
        std::to_string(bounds.max_position_) + "] metres; not clamped";
      return false;
    }
    const double initial = grasp_start.getVariablePosition("updown");
    if (target == initial) return true;
    const std::string stage = height_strategy_ == "comfort_radius" ?
      "move_to_grasp_height" : "lower_to_box_height";
    std::vector<ReplayFrame> prefix{ReplayFrame{stage, allJoints(grasp_start), false}};
    const size_t steps = static_cast<size_t>(std::ceil(std::abs(target - initial) / 0.005));
    for (size_t step = 1; step <= steps; ++step) {
      grasp_start.setVariablePosition("updown", initial + (target - initial) * step / steps);
      grasp_start.update(true);
      const std::string collision = collisionReason(scene, grasp_start, &result.metrics);
      if (!grasp_start.satisfiesBounds() || !collision.empty()) {
        result.failure_stage = "height_alignment_collision";
        result.failure_reason = "updown=" +
          std::to_string(grasp_start.getVariablePosition("updown")) + ": " +
          (collision.empty() ? "joint bounds violated" : collision);
        result.diagnostic_frames = prefix;
        result.rejected_state = std::make_shared<moveit::core::RobotState>(grasp_start);
        result.diagnostic["snapshot"] = "first_rejected_lift_sample";
        // Execution still rejects the complete descent; diagnostics are display-only.
        return false;
      }
      prefix.push_back(ReplayFrame{stage, allJoints(grasp_start), false});
    }
    result.frames = std::move(prefix);
    return true;
  }

  TaskResult planTask(
    const Eigen::Vector3d& box_center, std::vector<TaskResult>* complete_candidates_out = nullptr)
  {
    height_clearance_ = {{"checked", false}};
    TaskResult result;
    std::vector<TaskResult> complete_candidates;
    const auto total_started = std::chrono::steady_clock::now();
    const auto scene = makeScene(box_center);
    const auto loaded = loadedScene(scene);
    auto finish = [&]() {
      if (!result.success) {
        if (result.diagnostic_frames.empty()) result.diagnostic_frames = result.frames;
        if (result.diagnostic_frames.empty())
          result.diagnostic_frames.push_back({"initial_state", allJoints(*initial_state_), false});
        result.diagnostic["diagnostic_only"] = true;
        result.diagnostic["stage"] = result.failure_stage;
        result.diagnostic["reason"] = result.failure_reason;
        result.diagnostic["freeze_at_end"] = true;
        result.diagnostic["contacts"] = nlohmann::json::array();
        if (result.rejected_state) {
          const moveit::core::RobotState state(*result.rejected_state);
          // A search/goal snapshot is not a continuation of the robot motion. In particular,
          // never attach a box at an unvisited home pose just to show a rejected return goal.
          const bool connected = result.failure_stage == "initial_state" ||
            result.failure_stage == "height_alignment_collision" ||
            result.failure_stage == "cartesian_approach" || result.failure_stage == "cartesian_retreat" ||
            result.failure_stage == "cartesian_lift" ||
            result.failure_stage == "attach_box";
          result.diagnostic["rejected_joints"] = allJoints(state);
          result.diagnostic["rejected_box_attached"] = state.hasAttachedBody(kCarriedBoxId);
          if (connected) {
            moveit::core::RobotState previous(*initial_state_);
            const auto& frame = result.diagnostic_frames.back();
            for (size_t i = 0; i < all_joint_names_.size(); ++i)
              previous.setVariablePosition(all_joint_names_[i], frame.joints[i]);
            previous.update(true);
            moveit::core::RobotStatePtr first_rejected;
            std::string ignored;
            if (result.failure_stage != "height_alignment_collision") edgeClear(state.hasAttachedBody(kCarriedBoxId) ? loaded : scene, previous, state,
              state.hasAttachedBody(kCarriedBoxId), nullptr, &ignored, &first_rejected);
            if (first_rejected) result.rejected_state = first_rejected;
            result.diagnostic_frames.push_back({"FAILED_SAMPLE: " + result.failure_stage,
              allJoints(*result.rejected_state), result.rejected_state->hasAttachedBody(kCarriedBoxId)});
            result.diagnostic["snapshot"] = result.failure_stage == "height_alignment_collision" ?
              "first_rejected_lift_sample" : "first_rejected_path_sample";
          } else {
            result.diagnostic_frames.back().stage = "FAILED_HOLD: " + result.failure_stage;
            result.diagnostic["snapshot"] = "unconnected_rejected_state_not_replayed";
            const auto target = state.getGlobalLinkTransform(tool_link_).translation().eval();
            if (!result.diagnostic.contains("target"))
              result.diagnostic["target"] = {target.x(), target.y(), target.z()};
          }
          collision_detection::CollisionRequest request;
          collision_detection::CollisionResult contacts;
          request.contacts = true;
          request.max_contacts = 20;
          request.max_contacts_per_pair = 1;
          if (!distance_demo_) request.group_name = planning_group_name_;
          (result.rejected_state->hasAttachedBody(kCarriedBoxId) ? loaded : scene)->checkCollision(request, contacts, *result.rejected_state);
          for (const auto& pair : contacts.contacts)
            for (const auto& contact : pair.second)
              result.diagnostic[connected ? "contacts" : "rejected_contacts"].push_back({{"bodies", {pair.first.first, pair.first.second}},
                {"position", {contact.pos.x(), contact.pos.y(), contact.pos.z()}}});
        } else {
          result.diagnostic_frames.back().stage = "FAILED_HOLD: " + result.failure_stage;
          result.diagnostic["snapshot"] = "last_available_state_no_rejected_configuration";
        }
      }
      result.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - total_started).count();
      return result;
    };
    const std::string initial_collision = collisionReason(scene, *initial_state_, &result.metrics);
    if (!initial_state_->satisfiesBounds() || !initial_collision.empty()) {
      result.rejected_state = std::make_shared<moveit::core::RobotState>(*initial_state_);
      result.failure_stage = "initial_state";
      result.failure_reason = initial_collision.empty() ? "joint bounds violated" : initial_collision;
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
      return finish();
    }

    moveit::core::RobotState grasp_start(*initial_state_);
    std::vector<ReplayFrame> preparation;
    if (distance_demo_ && top_suction_) {
      const auto geometry = heightAlignment(box_center);
      if (geometry.at("xy").get<double>() > geometry.at("arm_length").get<double>()) {
        result.failure_stage = "precontact_ik";
        result.failure_reason = "top-suction wrist horizontal distance=" +
          std::to_string(geometry.at("xy").get<double>()) + "m exceeds arm length=" +
          std::to_string(geometry.at("arm_length").get<double>()) + "m; changing lift cannot fix XY reach";
        const auto target = precontactPose(box_center).translation().eval();
        result.diagnostic["target"] = {target.x(), target.y(), target.z()};
        return finish();
      }
      checkHeightClearance(scene, grasp_start, result.metrics);
      const auto height = heightAlignment(box_center);
      const auto& lift_bounds = robot_model_->getVariableBounds("updown");
      const double ideal = std::clamp(height.at("ideal_updown").get<double>(),
        lift_bounds.min_position_, lift_bounds.max_position_);
      // Keep an already retracted rear-release posture when it can reach the
      // working height. Do not force it back through a now-obstructed home fold.
      if (std::abs(height.at("target_updown").get<double>() - ideal) > 1e-6) {
        // Retract BOTH empty arms before lowering. Descending in forward-facing home
        // otherwise hits the lower rows with the idle suction plate and blocks valid IK heights.
        const std::string selected_side = side_;
        preparation.push_back({"initial_state", allJoints(grasp_start), false});
        for (const std::string side : {"left", "right"}) {
          selectArm(side);
          moveit::core::RobotState folded(grasp_start);
          folded.setJointGroupPositions(planning_group_, armJoints(*home_state_).data());
          folded.setVariablePosition(side + "_joint4", foldedElbowPosition(side));
          folded.update(true);
          const auto path = planRrt(scene, grasp_start, folded);
          result.metrics.rrt_approach_ms += path.wall_ms;
          if (!path.success) {
            result.failure_stage = "prepare_top_suction";
            result.failure_reason = side + ": " + path.reason;
            result.rejected_state = path.rejected_state;
            result.frames = preparation;
            selectArm(selected_side);
            return finish();
          }
          appendStates(path.states, "fold_arms_before_lift", false, true, &preparation);
          grasp_start = folded;
        }
        selectArm(selected_side);
      }
    }
    checkHeightClearance(scene, grasp_start, result.metrics);
    if (height_clearance_.value("checked", false)) {
      const auto alignment = heightAlignment(box_center);
      RCLCPP_INFO(get_logger(),
        "height selected: arm=%s updown=%.6fm reachable=[%.6f, %.6f]m ratio=%.6f inside_band=%s",
        side_.c_str(), alignment.at("target_updown").get<double>(),
        height_clearance_.at("lower").get<double>(), height_clearance_.at("upper").get<double>(),
        alignment.at("actual_ratio").get<double>(), alignment.at("inside_band").get<bool>() ? "true" : "false");
    }
    if (!alignHeight(box_center, scene, grasp_start, result)) {
      result.diagnostic_frames.insert(result.diagnostic_frames.begin(), preparation.begin(), preparation.end());
      result.frames = {ReplayFrame{"initial_state", allJoints(*initial_state_), false}};
      return finish();
    }
    result.frames.insert(result.frames.begin(), preparation.begin(), preparation.end());
    const auto lift_prefix = result.frames;
    std::string precontact_rejections;
    auto precontact_candidates = solvePoseCandidates(
      precontactPose(box_center), grasp_start, false, scene, &result.metrics,
      false, &precontact_rejections, &result.rejected_state);
    if (precontact_candidates.empty()) {
      const auto target = precontactPose(box_center).translation().eval();
      result.diagnostic["target"] = {target.x(), target.y(), target.z()};
      result.failure_stage = "precontact_ik";
      result.failure_reason = precontact_rejections;
      if (top_suction_) {
        const auto geometry = heightAlignment(box_center);
        if (geometry.at("xy").get<double>() > geometry.at("arm_length").get<double>())
          result.failure_reason += " top-suction wrist horizontal distance=" +
            std::to_string(geometry.at("xy").get<double>()) + "m exceeds arm length=" +
            std::to_string(geometry.at("arm_length").get<double>()) + "m; changing lift cannot fix XY reach";
      }
      if (result.frames.empty())
        result.frames.push_back(ReplayFrame{"initial_state", allJoints(grasp_start), false});
      return finish();
    }
    if (precontact_candidates.size() > precontact_candidate_limit_) {
      precontact_candidates.resize(precontact_candidate_limit_);
    }

    std::string last_failure_stage = "candidate_search";
    std::string last_failure_reason = "no candidate attempted";
    std::vector<ReplayFrame> best_partial = lift_prefix;
    for (size_t candidate_index = 0; candidate_index < precontact_candidates.size(); ++candidate_index) {
      result.rejected_state.reset();
      result.diagnostic = nlohmann::json::object();
      result.diagnostic_frames.clear();
      best_partial = lift_prefix;
      const auto approach_rrt = planRrt(
        scene, grasp_start, *precontact_candidates[candidate_index].state);
      result.metrics.rrt_approach_ms += approach_rrt.wall_ms;
      if (!approach_rrt.success) {
        result.rejected_state = approach_rrt.rejected_state;
        const auto target = precontactPose(box_center).translation().eval();
        result.diagnostic["target"] = {target.x(), target.y(), target.z()};
        last_failure_stage = "rrt_to_precontact";
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          approach_rrt.reason;
        continue;
      }

      std::vector<ReplayFrame> executable_prefix = lift_prefix;
      appendStates(approach_rrt.states, "rrt_to_precontact", false, false, &executable_prefix);
      const auto analytic_started = std::chrono::steady_clock::now();
      std::vector<moveit::core::RobotStatePtr> approach_states;
      std::vector<moveit::core::RobotStatePtr> retreat_states;
      std::string analytic_failure_stage;
      std::string analytic_failure_reason;
      const bool analytic_ok = traceCartesianPath(
        box_center, *precontact_candidates[candidate_index].state, scene,
        &result.metrics, &approach_states, &retreat_states,
        &analytic_failure_stage, &analytic_failure_reason, &result);
      result.metrics.analytic_path_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - analytic_started).count();
      appendStates(approach_states, "cartesian_approach", false, true, &executable_prefix);
      if (!retreat_states.empty()) {
        executable_prefix.push_back(
          ReplayFrame{"attach_box", allJoints(*retreat_states.front()), true});
      }
      const size_t retreat_begin = executable_prefix.size();
      appendStates(retreat_states, "cartesian_retreat", true, true, &executable_prefix);
      if (top_suction_) {
        const size_t lift_frames = std::max<size_t>(1, std::ceil(approach_distance_ / cartesian_step_));
        for (size_t i = retreat_begin; i < std::min(executable_prefix.size(), retreat_begin + lift_frames); ++i)
          executable_prefix[i].stage = "cartesian_lift";
      }
      best_partial = executable_prefix;
      if (!analytic_ok) {
        last_failure_stage = analytic_failure_stage;
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " + analytic_failure_reason;
        continue;
      }

      last_failure_stage.clear();
      moveit::core::RobotState return_goal(grasp_start);
      attachCarriedBox(return_goal);
      if (distance_demo_ && post_extract_policy_ == "rear_release") {
        // Position the entire rotated payload behind the chassis, not just the TCP.
        // A folded top-suction payload can extend forward of its TCP.
        moveit::core::RobotState rear_reference(*home_state_);
        rear_reference.setVariablePosition("updown", grasp_start.getVariablePosition("updown"));
        if (top_suction_)
          rear_reference.setVariablePosition(side_ + "_joint4", foldedElbowPosition(side_));
        rear_reference.update(true);
        const auto rear = alfa_robot::motion::wallRearPlacementPose(
          rear_reference.getGlobalLinkTransform(tool_link_), toolToBox(), boxSize(),
          chassis_rear_x_, rear_clearance_, wall_bottom_z_ + contact_numerical_gap_);
        std::string reason;
        auto candidates = solvePoseCandidates(rear, grasp_start, true, loaded, &result.metrics, false, &reason, &result.rejected_state);
        result.diagnostic["target"] = {rear.translation().x(), rear.translation().y(), rear.translation().z()};
        bool found = false;
        for (size_t i = 0; i < std::min(candidates.size(), precontact_candidate_limit_); ++i) {
          const auto& goal = *candidates[i].state;
          if (!alfa_robot::motion::boxBehindChassis(
              goal.getGlobalLinkTransform(tool_link_) * toolToBox(), boxSize(), chassis_rear_x_)) {
            reason = "rear target does not put the entire box at least 1cm behind chassis";
            continue;
          }
          return_goal = goal;
          found = true;
          break;
        }
        if (!found) {
          last_failure_stage = "rear_placement";
          last_failure_reason = reason;
          continue;
        }
      } else {
        const auto reason = collisionReason(loaded, return_goal, &result.metrics);
        if (!reason.empty()) {
          result.rejected_state = std::make_shared<moveit::core::RobotState>(return_goal);
          last_failure_stage = "return_goal";
          last_failure_reason = reason;
          continue;
        }
      }

      RrtPlanResult return_plan;
      std::string direct_reason;
      if (edgeClear(loaded, *retreat_states.back(), return_goal, true, &result.metrics,
          &direct_reason, &return_plan.rejected_state) &&
          alfa_robot::motion::sameShoulderElbowBranch(
            armJoints(*retreat_states.back()), armJoints(return_goal))) {
        return_plan.success = true;
        return_plan.states = directArmPath(*retreat_states.back(), return_goal);
      } else {
        return_plan = planRrt(loaded, *retreat_states.back(), return_goal);
        result.metrics.rrt_return_ms += return_plan.wall_ms;
      }
      if (!return_plan.success) {
        result.rejected_state = return_plan.rejected_state;
        const auto target = return_goal.getGlobalLinkTransform(tool_link_).translation().eval();
        result.diagnostic["target"] = {target.x(), target.y(), target.z()};
        last_failure_stage = "rrt_return";
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          (return_plan.reason.empty() ? direct_reason : return_plan.reason);
        continue;
      }

      TaskResult candidate;
      candidate.frames = std::move(executable_prefix);
      appendStates(return_plan.states, "rrt_return", true, true, &candidate.frames);
      if (post_extract_policy_ == "loaded_home") {
        const auto updown_return = moveUpdown(
          loaded, *return_plan.states.back(),
          home_state_->getVariablePosition("updown"), true, &result.metrics);
        if (!updown_return.success) {
          result.rejected_state = updown_return.rejected_state;
          last_failure_stage = "updown_return";
          last_failure_reason = updown_return.reason;
          continue;
        }
        appendStates(
          updown_return.states, "updown_return", true, true, &candidate.frames);
        candidate.frames.push_back(
          ReplayFrame{"loaded_home", allJoints(*updown_return.states.back()), true});
      } else if (distance_demo_) {
        // The loaded path is checked through this last visible pose. Removal is
        // a separate frame with identical joints, never an obstacle workaround.
        candidate.frames.push_back(ReplayFrame{"rear_placement", allJoints(return_goal), true});
        candidate.frames.push_back(ReplayFrame{"release_box", allJoints(return_goal), false, false});
      }
      candidate.selection_score = naturalPathLength(candidate.frames);
      candidate.success = true;
      complete_candidates.push_back(std::move(candidate));
      if (complete_candidates.size() >= static_cast<size_t>(top_k_complete_)) break;
    }

    if (!complete_candidates.empty()) {
      std::stable_sort(complete_candidates.begin(), complete_candidates.end(),
        [](const TaskResult& left, const TaskResult& right) {
          return left.selection_score < right.selection_score;
        });
      if (complete_candidates_out) {
        *complete_candidates_out = complete_candidates;
        result = selectBestCandidate(std::move(complete_candidates), result.metrics,
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - total_started).count());
        return result;
      }
      const auto edge_validator = [&](const ReplayFrame& from, const ReplayFrame& to, std::string* reason) {
        if (from.box_attached != to.box_attached) return from.joints == to.joints;
        moveit::core::RobotState from_state(*initial_state_), to_state(*initial_state_);
        for (size_t i = 0; i < all_joint_names_.size(); ++i) {
          from_state.setVariablePosition(all_joint_names_[i], from.joints[i]);
          to_state.setVariablePosition(all_joint_names_[i], to.joints[i]);
        }
        if (from.box_attached) { attachCarriedBox(from_state); attachCarriedBox(to_state); }
        from_state.update(true); to_state.update(true);
        return edgeClear(from.box_attached ? loaded : scene, from_state, to_state,
          from.box_attached, &result.metrics, reason);
      };
      const auto path_validator = [&](const std::vector<ReplayFrame>& frames, std::string* reason) {
        return validateSingleArmFrames(frames, scene, loaded, &result.metrics, reason);
      };
      for (size_t rank = 0; rank < complete_candidates.size(); ++rank) {
        TaskResult candidate = complete_candidates[rank];
        std::string postprocess_reason;
        if (!postprocessCandidate(&candidate, scene, loaded, planning_group_name_, edge_validator, path_validator,
            &postprocess_reason)) {
          result.metrics.shortcut_ms += candidate.metrics.shortcut_ms;
          result.metrics.chomp_ms += candidate.metrics.chomp_ms;
          result.metrics.totg_ms += candidate.metrics.totg_ms;
          result.metrics.ruckig_ms += candidate.metrics.ruckig_ms;
          last_failure_stage = "trajectory_postprocess";
          last_failure_reason = "candidate rank " + std::to_string(rank + 1) + " " + postprocess_reason;
          continue;
        }
        candidate.metrics.add(result.metrics);
        candidate.total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - total_started).count();
        candidate.complete_candidate_count = complete_candidates.size();
        candidate.selected_candidate_rank = rank + 1;
        return candidate;
      }
    }

    result.failure_stage = last_failure_stage;
    result.failure_reason = last_failure_reason;
    result.frames = std::move(best_partial);
    if (result.frames.empty()) {
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(grasp_start), false});
    }
    return finish();
  }

  nlohmann::json sceneJson(const Eigen::Vector3d& box_center) const
  {
    const auto neighbors = neighborCenters(box_center);
    nlohmann::json output;
    output["box_center"] = {box_center.x(), box_center.y(), box_center.z()};
    output["box_size"] = {box_depth_, box_width_, box_height_};
    output["collision_inset"] = collision_inset_;
    output["world_frame"] = world_frame_;
    output["joint_names"] = all_joint_names_;
    output["initial_joints"] = allJoints(*initial_state_);
    if (distance_demo_) {
      output["distance_demo"] = true;
      output["post_extract_policy"] = post_extract_policy_;
      output["release_after_transfer"] = post_extract_policy_ == "rear_release";
      output["initial_pose"] = initial_pose_;
      output["planning_seed"] = planning_seed_;
      output["requested_trajectory_variant"] = trajectory_variant_;
      output["top_k_complete"] = top_k_complete_;
      output["trajectory_sample_period"] = trajectory_sample_period_;
      output["empty_velocity_scaling"] = empty_velocity_scaling_;
      output["empty_acceleration_scaling"] = empty_acceleration_scaling_;
      output["loaded_velocity_scaling"] = loaded_velocity_scaling_;
      output["loaded_acceleration_scaling"] = loaded_acceleration_scaling_;
      output["arm_max_jerk"] = arm_max_jerk_;
      output["head_max_jerk"] = head_max_jerk_;
      output["updown_max_jerk"] = updown_max_jerk_;
      output["edge_collision_resolution_deg"] = edge_joint_resolution_ * 180.0 / kPi;
      output["lift_after_attach_m"] = top_suction_ ? approach_distance_ : 0.0;
      output["lift_backoff_m"] = top_suction_ ? std::min(0.02, retreat_distance_) : 0.0;
      output["environment"] = environment_json_;
      output["height_alignment"] = heightAlignment(box_center);
      output["contact_numerical_gap"] = contact_numerical_gap_;
      output["requested_arm"] = requested_arm_;
      output["requested_suction_mode"] = requested_suction_mode_;
      output["wall_center_y"] = wall_center_y_;
      output["wall_bottom_z"] = wall_bottom_z_;
      output["scope"] = "simulation geometric extraction and loaded return; not hardware execution";
      output["x"] = wall_distance_;
      output["chassis_front_x"] = chassis_front_x_;
      output["box_id"] = wall_target_row_ * 5 + wall_target_column_;
      output["distance_reference"] = "world +X chassis front plane to wall near face";
    }
    output["tool_link"] = tool_link_;
    output["scene_layout"] = scene_layout_;
    output["wall_context"] = sequence_running_ ? "live_sequence" : wall_context_;
    if (scene_layout_ == "wall_5x5") {
      output["wall"] = {{"rows", 5}, {"columns", 5}, {"gap", wall_gap_},
        {"target_row", wall_target_row_}, {"target_column", wall_target_column_}};
    }
    output["neighbor_centers"] = nlohmann::json::array();
    for (const auto& center : neighbors) {
      output["neighbor_centers"].push_back({center.x(), center.y(), center.z()});
    }
    const auto precontact = precontactPose(box_center);
    const auto contact = contactPose(box_center);
    const auto retreat = retreatPose(box_center);
    output["precontact"] = {
      precontact.translation().x(), precontact.translation().y(), precontact.translation().z()};
    output["contact"] = {
      contact.translation().x(), contact.translation().y(), contact.translation().z()};
    output["retreat"] = {
      retreat.translation().x(), retreat.translation().y(), retreat.translation().z()};
    const auto offset = toolToBoxCenter();
    output["tool_to_box_center"] = {offset.x(), offset.y(), offset.z()};
    output["tool_to_box_rotation"] = nlohmann::json::array();
    for (int row = 0; row < 3; ++row)
      output["tool_to_box_rotation"].push_back({toolToBox().linear()(row, 0),
        toolToBox().linear()(row, 1), toolToBox().linear()(row, 2)});
    output["suction_mode"] = top_suction_ ? "top" : "front";
    output["removed_box_ids"] = removed_boxes_;
    output["chassis_rear_x"] = chassis_rear_x_;
    output["rear_clearance"] = rear_clearance_;
    return output;
  }

  void publishJson(nlohmann::json& payload, bool segment = false)
  {
    payload["publisher_id"] = publisher_id_;
    if (payload.contains("generation"))
      payload["task_id"] = publisher_id_ + ":" + std::to_string(payload["generation"].get<uint64_t>());
    std_msgs::msg::String message;
    message.data = payload.dump();
    (segment ? segment_publisher_ : task_publisher_)->publish(message);
  }

  void publishPreview(const std::string& status)
  {
    Eigen::Vector3d center;
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      center = box_center_;
    }
    nlohmann::json payload = sceneJson(center);
    payload["kind"] = "preview";
    payload["side"] = side_;
    payload["status"] = status;
    publishJson(payload);
  }

  void publishPlanningStarted(uint64_t generation, const Eigen::Vector3d& box_center)
  {
    nlohmann::json payload = sceneJson(box_center);
    payload["kind"] = "planning";
    payload["generation"] = generation;
    payload["side"] = side_;
    payload["status"] = "计算开始";
    publishJson(payload);
  }

  void publishTaskResult(
    uint64_t generation,
    const Eigen::Vector3d& box_center,
    const TaskResult& result, bool publish = true)
  {
    last_result_ = sceneJson(box_center);
    auto& payload = last_result_;
    payload["kind"] = "result";
    payload["generation"] = generation;
    payload["side"] = side_;
    payload["tool_link"] = tool_link_;
    payload["success"] = result.success;
    payload["failure_stage"] = result.failure_stage;
    payload["failure_reason"] = result.failure_reason;
    payload["total_ms"] = result.total_ms;
    payload["metrics"] = {
      {"ik_calls", result.metrics.ik_calls},
      {"ik_ms", result.metrics.ik_ms},
      {"collision_checks", result.metrics.collision_checks},
      {"collision_ms", result.metrics.collision_ms},
      {"analytic_path_ms", result.metrics.analytic_path_ms},
      {"rrt_approach_ms", result.metrics.rrt_approach_ms},
      {"rrt_return_ms", result.metrics.rrt_return_ms},
      {"shortcut_ms", result.metrics.shortcut_ms},
      {"chomp_ms", result.metrics.chomp_ms},
      {"totg_ms", result.metrics.totg_ms},
      {"ruckig_ms", result.metrics.ruckig_ms},
    };
    payload["requested_trajectory_variant"] = trajectory_variant_;
    payload["effective_trajectory_variant"] = result.effective_trajectory_variant;
    payload["top_k_complete"] = top_k_complete_;
    payload["complete_candidate_count"] = result.complete_candidate_count;
    payload["selected_candidate_rank"] = result.selected_candidate_rank;
    payload["selection_score"] = std::isfinite(result.selection_score) ?
      nlohmann::json(result.selection_score) : nlohmann::json(nullptr);
    payload["execution_duration_s"] = result.execution_duration_s;
    payload["timing_valid"] = result.timing_valid;
    payload["optimizer_status"] = result.optimizer_status;
    payload["variant_fallback_reason"] = result.variant_fallback_reason;
    payload["max_velocity"] = result.max_velocity;
    payload["max_acceleration"] = result.max_acceleration;
    payload["max_jerk"] = result.max_jerk;
    payload["joint_names"] = all_joint_names_;
    payload["attempts"] = attempts_;
    payload["verdict"] = result.success ? "path_found" : "no_path_found";
    payload["frames"] = nlohmann::json::array();
    payload["frames"].get_ref<nlohmann::json::array_t&>().reserve(result.frames.size());
    for (const auto& frame : result.frames) {
      payload["frames"].push_back({
        {"stage", frame.stage},
        {"joints", frame.joints},
        {"box_attached", frame.box_attached},
        {"box_visible", frame.box_visible},
        {"scene_index", frame.scene_index},
        {"carried_boxes", frame.carried_boxes},
        {"time_from_start_s", frame.time_from_start_s},
      });
    }
    payload["diagnostic"] = result.diagnostic;
    payload["diagnostic_frames"] = nlohmann::json::array();
    payload["diagnostic_frames"].get_ref<nlohmann::json::array_t&>().reserve(
      result.diagnostic_frames.size());
    for (const auto& frame : result.diagnostic_frames)
      payload["diagnostic_frames"].push_back({{"stage", frame.stage}, {"joints", frame.joints},
        {"box_attached", frame.box_attached}, {"box_visible", frame.box_visible},
        {"scene_index", frame.scene_index}, {"carried_boxes", frame.carried_boxes},
        {"time_from_start_s", frame.time_from_start_s}, {"diagnostic_only", true}});
    if (publish) publishJson(payload);
  }

  void publishStatus(const std::string& status, bool good)
  {
    visualization_msgs::msg::MarkerArray markers;
    Marker text;
    text.header.frame_id = world_frame_;
    text.header.stamp = now();
    text.ns = "v3_single_arm_box_extract_status";
    text.id = 0;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      text.pose.position.x = box_center_.x();
      text.pose.position.y = box_center_.y();
      text.pose.position.z = box_center_.z() + box_height_ * 0.75;
    }
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.035;
    text.color = good ? color(0.15F, 1.0F, 0.25F) : color(1.0F, 0.12F, 0.12F);
    text.text = status;
    if (distance_demo_) text.text = "box=" + std::to_string(wall_target_row_ * 5 + wall_target_column_) +
      " x=" + std::to_string(wall_distance_) + "m arm=" + side_ + "\n" + status;
    if (distance_demo_ && height_strategy_ == "comfort_radius") {
      Eigen::Vector3d center;
      { std::lock_guard<std::mutex> lock(box_mutex_); center = box_center_; }
      const auto h = heightAlignment(center);
      std::ostringstream geometry;
      geometry << std::fixed << std::setprecision(3) << "\nxy=" << h.at("xy").get<double>()
        << " rho=" << h.at("actual_ratio").get<double>() << " band=[" << comfort_min_
        << "," << comfort_max_ << "] lift=" << h.at("target_updown").get<double>()
        << " " << h.value("branch", "wrist_aligned") << " " << h.at("outside_reason").get<std::string>()
        << (height_clearance_.value("checked", false) ? " lift_checked" : " proposal_unchecked");
      text.text += geometry.str();
    }
    markers.markers.push_back(text);
    status_marker_publisher_->publish(markers);
  }

  void publishSceneMarkers()
  {
    Eigen::Vector3d center;
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      center = box_center_;
    }
    const auto snapshot = display_scene_.is_object() ? display_scene_ : sceneJson(center);
    if (distance_demo_) {
      const auto& p = snapshot.at("box_center");
      center = Eigen::Vector3d(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    }
    visualization_msgs::msg::MarkerArray markers;
    if (distance_demo_) {
      // A restarted planner may publish fewer obstacles; do not retain old RViz markers.
      Marker clear;
      clear.action = Marker::DELETEALL;
      markers.markers.push_back(clear);
    }
    for (size_t index = 0; index < environment_objects_.size(); ++index) {
      const auto& object = environment_objects_[index];
      Marker obstacle;
      obstacle.header.frame_id = world_frame_;
      obstacle.header.stamp = now();
      obstacle.ns = "environment";
      obstacle.id = static_cast<int>(index);
      obstacle.type = Marker::CUBE;
      obstacle.action = Marker::ADD;
      obstacle.pose = object.primitive_poses.front();
      const auto& size = object.primitives.front().dimensions;
      obstacle.scale.x = size[0];
      obstacle.scale.y = size[1];
      obstacle.scale.z = size[2];
      obstacle.color = color(0.45F, 0.55F, 0.65F, 0.20F);
      markers.markers.push_back(obstacle);
    }
    Marker target;
    target.header.frame_id = world_frame_;
    target.header.stamp = now();
    target.ns = "target_box";
    target.id = 0;
    target.type = Marker::CUBE;
    target.action = Marker::ADD;
    target.pose.position.x = center.x();
    target.pose.position.y = center.y();
    target.pose.position.z = center.z();
    target.pose.orientation.w = 1.0;
    target.scale.x = box_depth_;
    target.scale.y = box_width_;
    target.scale.z = box_height_;
    target.color = color(0.20F, 0.85F, 0.25F, 0.72F);
    if (display_box_attached_) {
      const auto& tool = display_state_->getGlobalLinkTransform(snapshot.at("tool_link").get<std::string>());
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      for (int row = 0; row < 3; ++row) {
        offset.translation()[row] = snapshot.at("tool_to_box_center")[row].get<double>();
        for (int col = 0; col < 3; ++col)
          offset.linear()(row, col) = snapshot.at("tool_to_box_rotation")[row][col].get<double>();
      }
      target.pose = eigenToPose(tool * offset);
    }
    if (display_box_visible_ || !distance_demo_) markers.markers.push_back(target);

    Marker control_link;
    control_link.header.frame_id = world_frame_;
    control_link.header.stamp = now();
    control_link.ns = "box_control_handle";
    control_link.id = 0;
    control_link.type = Marker::LINE_STRIP;
    control_link.action = Marker::ADD;
    control_link.pose.orientation.w = 1.0;
    control_link.scale.x = 0.008;
    control_link.color = color(0.10F, 0.85F, 1.0F, 0.85F);
    geometry_msgs::msg::Point handle_point;
    const Eigen::Vector3d handle_position = controlHandlePosition(center);
    handle_point.x = handle_position.x();
    handle_point.y = handle_position.y();
    handle_point.z = handle_position.z();
    control_link.points.push_back(handle_point);
    geometry_msgs::msg::Point center_point;
    center_point.x = center.x();
    center_point.y = center.y();
    center_point.z = center.z();
    control_link.points.push_back(center_point);
    if (!distance_demo_) markers.markers.push_back(control_link);

    std::vector<Eigen::Vector3d> neighbors;
    for (const auto& p : snapshot.at("neighbor_centers"))
      neighbors.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    for (size_t index = 0; index < neighbors.size(); ++index) {
      Marker box;
      box.header.frame_id = world_frame_;
      box.header.stamp = now();
      box.ns = "neighbor_boxes";
      box.id = static_cast<int>(index);
      box.type = Marker::CUBE;
      box.action = Marker::ADD;
      box.pose.position.x = neighbors[index].x();
      box.pose.position.y = neighbors[index].y();
      box.pose.position.z = neighbors[index].z();
      box.pose.orientation.w = 1.0;
      box.scale.x = box_depth_;
      box.scale.y = box_width_;
      box.scale.z = box_height_;
      box.color = color(1.0F, 0.48F, 0.08F, 0.45F);
      markers.markers.push_back(box);
    }

    const std::array<std::pair<Eigen::Vector3d, std::string>, 3> points = {{
      {precontactPose(center).translation(), "precontact"},
      {contactPose(center).translation(), "contact"},
      {retreatPose(center).translation(), "retreat"},
    }};
    for (size_t index = 0; index < points.size(); ++index) {
      Marker point;
      point.header.frame_id = world_frame_;
      point.header.stamp = now();
      point.ns = "task_points";
      point.id = static_cast<int>(index);
      point.type = Marker::SPHERE;
      point.action = Marker::ADD;
      const auto& position = snapshot.at(points[index].second);
      point.pose.position.x = position[0].get<double>();
      point.pose.position.y = position[1].get<double>();
      point.pose.position.z = position[2].get<double>();
      point.pose.orientation.w = 1.0;
      point.scale.x = point.scale.y = point.scale.z = 0.035;
      point.color = index == 0 ?
        color(0.15F, 0.55F, 1.0F, 0.9F) :
        (index == 1 ? color(0.2F, 1.0F, 0.2F, 0.9F) : color(0.95F, 0.2F, 0.95F, 0.9F));
      markers.markers.push_back(point);
    }
    if (distance_demo_) {
      const auto removed = snapshot.at("removed_box_ids").get<std::set<int>>();
      for (int id = 0; id < 25; ++id) {
        if (removed.count(id) || (id == snapshot.at("box_id").get<int>() &&
            (!display_box_visible_ || display_box_attached_))) continue;
        Marker label;
        label.header = target.header;
        label.ns = "wall_box_ids";
        label.id = id;
        label.type = Marker::TEXT_VIEW_FACING;
        label.action = Marker::ADD;
        label.pose.orientation.w = 1.0;
        label.pose.position.x = chassis_front_x_ + wall_distance_ - 0.02;
        label.pose.position.y = wall_center_y_ + (id % 5 - 2) * (box_width_ + wall_gap_);
        label.pose.position.z = wall_bottom_z_ + box_height_ / 2.0 + (id / 5) * (box_height_ + wall_gap_);
        label.scale.z = 0.07;
        label.color = color(1.0F, 1.0F, 1.0F);
        label.text = std::to_string(id);
        markers.markers.push_back(label);
      }
    }
    alfa_robot::motion::appendDemoFailureMarkers(
      markers, display_diagnostic_, display_failure_frozen_, world_frame_, now());
    scene_marker_publisher_->publish(markers);
  }

  void publishDisplayState()
  {
    std::lock_guard<std::mutex> lock(display_mutex_);
    if (!playback_frames_.empty()) {
      display_failure_frozen_ = !display_diagnostic_.empty() && playback_index_ + 1U == playback_frames_.size();
      const auto& frame = playback_frames_[playback_index_];
      for (size_t index = 0; index < all_joint_names_.size() && index < frame.joints.size(); ++index) {
        display_state_->setVariablePosition(all_joint_names_[index], frame.joints[index]);
      }
      display_state_->update(true);
      display_box_attached_ = frame.box_attached;
      display_box_visible_ = frame.box_visible;
      if (!playback_scenes_.empty()) display_scene_ = playback_scenes_.at(frame.scene_index);
      if (playback_index_ + 1U == playback_frames_.size()) sequence_playback_ = false;
      if (display_failure_frozen_)
        publishStatus("DIAGNOSTIC ONLY - FROZEN (not executable)\n" +
          display_diagnostic_.value("stage", "") + "\n" + display_diagnostic_.value("reason", ""), false);
      playback_index_ = (distance_demo_ || !display_diagnostic_.empty()) ? std::min(playback_index_ + 1U, playback_frames_.size() - 1U) :
        (playback_index_ + 1U) % playback_frames_.size();
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = all_joint_names_;
    message.position = allJoints(*display_state_);
    joint_state_publisher_->publish(message);
    publishSceneMarkers();
  }

  bool display_failure_frozen_ = false;
  nlohmann::json display_diagnostic_ = nlohmann::json::object();
  std::string wall_context_ = "full";
  bool top_suction_ = false;
  bool sequence_mode_ = false;
  std::string initial_pose_ = "home";
  double top_shoulder_above_wrist_ = 0.10;  // Offline-calibrated top policy, independent of front comfort ratio.
  bool sequence_running_ = false;
  bool sequence_requested_ = false;
  bool sequence_playback_ = false;
  bool display_box_visible_ = true;
  double chassis_rear_x_ = 0.0;
  double rear_clearance_ = 0.02;
  std::set<int> removed_boxes_;
  moveit::core::RobotStatePtr home_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sequence_service_;
  std::vector<nlohmann::json> playback_scenes_;
  nlohmann::json display_scene_;
  bool display_box_attached_ = false;
  std::vector<moveit_msgs::msg::CollisionObject> environment_objects_;
  nlohmann::json environment_json_;
  bool distance_demo_ = false;
  std::string post_extract_policy_ = "rear_release";
  bool align_height_ = false;
  std::string height_strategy_ = "fixed_offset";
  nlohmann::json height_clearance_ = {{"checked", false}};
  std::optional<double> synchronized_updown_;
  std::string comfort_branch_ = "auto";
  double comfort_min_ = 0.8, comfort_preferred_ = 0.8, comfort_max_ = 0.8;
  int planning_seed_ = 0;
  std::string trajectory_variant_ = "topk";
  int top_k_complete_ = 3;
  double trajectory_sample_period_ = 0.05;
  double empty_velocity_scaling_ = 0.50;
  double empty_acceleration_scaling_ = 0.50;
  double loaded_velocity_scaling_ = 0.25;
  double loaded_acceleration_scaling_ = 0.25;
  double arm_max_jerk_ = 2.0;
  double head_max_jerk_ = 2.0;
  double updown_max_jerk_ = 0.30;
  double shoulder_box_offset_ = 0.25;
  double initial_shoulder_z_ = 0.0;
  double chassis_front_x_ = 0.0;
  double wall_center_y_ = 0.0;
  double wall_bottom_z_ = 0.0;
  double wall_distance_ = 0.0;
  std::string requested_arm_ = "auto";
  std::string requested_suction_mode_ = "auto";
  nlohmann::json last_result_;
  nlohmann::json attempts_;
  rclcpp::Service<WallRequest>::SharedPtr wall_service_;
  std::string side_;
  std::string world_frame_;
  std::string arm_base_link_;
  double contact_numerical_gap_ = 0.0;
  std::string planning_group_name_;
  std::string tool_link_;
  Eigen::Vector3d box_center_{0.88, -0.20, 0.55};
  Eigen::Vector3d initial_box_center_{0.88, -0.20, 0.55};
  std::string scene_layout_ = "cross";
  int wall_target_row_ = 0;
  int wall_target_column_ = 2;
  double wall_gap_ = 0.01;
  double box_depth_ = 0.30;
  double box_width_ = 0.40;
  double box_height_ = 0.40;
  double control_handle_clearance_ = 0.25;
  double control_handle_lateral_offset_ = 0.90;
  double approach_distance_ = 0.05;
  double retreat_distance_ = 0.35;
  double cartesian_step_ = 0.01;
  double collision_inset_ = 0.002;
  double psi_step_ = degToRad(5.0);
  double maximum_cartesian_joint_step_ = degToRad(15.0);
  double edge_joint_resolution_ = degToRad(2.5);
  size_t precontact_candidate_limit_ = 8;
  double rrt_planning_time_ = 1.0;
  int rrt_planning_attempts_ = 1;
  bool auto_run_once_ = false;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* planning_group_ = nullptr;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  moveit::core::RobotStatePtr initial_state_;
  moveit::core::RobotStatePtr display_state_;
  std::unique_ptr<V3RedundantArmAnalyticIk> solver_;
  std::vector<std::string> all_joint_names_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle confirm_menu_entry_ = 0;
  interactive_markers::MenuHandler::EntryHandle reset_menu_entry_ = 0;
  const std::string publisher_id_ = std::to_string(
    std::chrono::system_clock::now().time_since_epoch().count());
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr segment_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_marker_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::TimerBase::SharedPtr auto_run_timer_;

  std::mutex box_mutex_;
  std::mutex display_mutex_;
  std::vector<ReplayFrame> playback_frames_;
  size_t playback_index_ = 0;
  std::atomic<bool> planning_requested_{false};
  std::atomic<bool> planning_active_{false};
  uint64_t generation_ = 0;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<V3SingleArmBoxExtractDemo>(options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  try {
    node->init();
    spin_thread.join();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "single-arm extract demo init failed: %s", error.what());
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
