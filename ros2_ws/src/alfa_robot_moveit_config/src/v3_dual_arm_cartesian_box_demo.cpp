#include "alfa_robot_moveit_config/demo_failure_markers.hpp"
#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>
#include <alfa_robot_moveit_config/planning_diagnostics.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shapes.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
using alfa_robot::analytic_ik::V3RedundantArmModel;
using alfa_robot::analytic_ik::V3RedundantIkRequest;
using alfa_robot::analytic_ik::V3RedundantIkSolution;
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
using Marker = visualization_msgs::msg::Marker;

constexpr double kPi = 3.14159265358979323846;
constexpr char kMarkerName[] = "dual_cartesian_box_target";
constexpr char kHeldBoxId[] = "dual_held_box";

double degToRad(double value)
{
  return value * kPi / 180.0;
}

double normalizedAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double rawMaximumJointDelta(
  const std::array<double, 7>& from,
  const std::array<double, 7>& to)
{
  double maximum = 0.0;
  for (size_t index = 0; index < from.size(); ++index) {
    maximum = std::max(maximum, std::abs(to[index] - from[index]));
  }
  return maximum;
}

double squaredJointDistance(
  const std::array<double, 7>& from,
  const std::array<double, 7>& to)
{
  double distance = 0.0;
  for (size_t index = 0; index < from.size(); ++index) {
    const double delta = to[index] - from[index];
    distance += delta * delta;
  }
  return distance;
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

InteractiveMarkerControl rotationControl(
  const std::string& name,
  double x,
  double y,
  double z)
{
  InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1.0;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  return control;
}

struct ArmCandidate
{
  std::array<double, 7> joints{};
  V3RedundantIkSolution solution;
  double score = std::numeric_limits<double>::infinity();
};

struct DualCandidate
{
  moveit::core::RobotStatePtr state;
  Eigen::Matrix3d left_orientation = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d right_orientation = Eigen::Matrix3d::Identity();
  double score = std::numeric_limits<double>::infinity();
};

struct ReplayFrame
{
  std::string stage;
  std::vector<double> joints;
  Eigen::Vector3d box_center = Eigen::Vector3d::Zero();
  double box_roll = 0.0;
};

struct PlanningMetrics
{
  uint64_t ik_calls = 0;
  double ik_ms = 0.0;
  uint64_t pair_checks = 0;
  uint64_t collision_checks = 0;
  double collision_ms = 0.0;
  double initialization_ms = 0.0;
  double cartesian_ms = 0.0;
};

struct TaskResult
{
  bool success = false;
  std::string failure_stage;
  std::string failure_reason;
  double total_ms = 0.0;
  PlanningMetrics metrics;
  std::vector<ReplayFrame> frames;
  moveit::core::RobotStatePtr final_state;
  std::vector<ReplayFrame> diagnostic_frames;
  nlohmann::json diagnostic = nlohmann::json::object();
};

class V3DualArmCartesianBoxDemo : public rclcpp::Node
{
public:
  explicit V3DualArmCartesianBoxDemo(const rclcpp::NodeOptions& options)
  : Node("v3_dual_arm_cartesian_box_demo", options)
  {}

  void init()
  {
    world_frame_ = getParameter<std::string>("world_frame", "world");
    arm_base_link_ = getParameter<std::string>("arm_base_link", "arm_carriage");
    left_group_name_ = getParameter<std::string>("left_group", "left_arm");
    right_group_name_ = getParameter<std::string>("right_group", "right_arm");
    dual_group_name_ = getParameter<std::string>("dual_group", "dual_arm");
    left_tool_link_ = getParameter<std::string>("left_tool_link", "left_tool0");
    right_tool_link_ = getParameter<std::string>("right_tool_link", "right_tool0");
    motion_mode_ = getParameter<std::string>("motion_mode", "translate");
    if (motion_mode_ != "translate" && motion_mode_ != "roll") {
      throw std::invalid_argument("motion_mode must be translate or roll");
    }
    grasp_pattern_ = getParameter<std::string>("grasp_pattern", "inward");
    if (grasp_pattern_ != "inward" &&
        grasp_pattern_ != "left_side_right_bottom" &&
        grasp_pattern_ != "right_side_left_bottom") {
      throw std::invalid_argument(
              "grasp_pattern must be inward, left_side_right_bottom or "
              "right_side_left_bottom");
    }
    const auto initial_center = getParameter<std::vector<double>>(
      "initial_box_center", {0.73, 0.0, 0.55});
    const auto initial_target_offset = getParameter<std::vector<double>>(
      "initial_target_offset", {-0.12, 0.0, 0.0});
    if (initial_center.size() != 3U || initial_target_offset.size() != 3U) {
      throw std::invalid_argument("initial_box_center and initial_target_offset require xyz");
    }
    initial_box_center_ = Eigen::Vector3d(
      getParameter<double>("initial_box_x", initial_center[0]),
      getParameter<double>("initial_box_y", initial_center[1]),
      getParameter<double>("initial_box_z", initial_center[2]));
    current_box_center_ = initial_box_center_;
    target_box_center_ = initial_box_center_ + Eigen::Vector3d(
      getParameter<double>("initial_target_offset_x", initial_target_offset[0]),
      getParameter<double>("initial_target_offset_y", initial_target_offset[1]),
      getParameter<double>("initial_target_offset_z", initial_target_offset[2]));
    current_box_roll_ = 0.0;
    target_box_roll_ = motion_mode_ == "roll" ?
      degToRad(getParameter<double>("initial_target_roll_deg", 45.0)) : 0.0;
    box_size_ = getParameter<double>("box_size", 0.40);
    upward_contact_lateral_offset_ = std::clamp(
      getParameter<double>("upward_contact_lateral_offset", 0.10),
      0.0, box_size_ * 0.5);
    cartesian_step_ = getParameter<double>("cartesian_step", 0.01);
    angular_step_ = degToRad(getParameter<double>("angular_step_deg", 2.0));
    psi_step_ = degToRad(getParameter<double>("psi_step_deg", 5.0));
    maximum_joint_step_ = degToRad(getParameter<double>("maximum_joint_step_deg", 12.0));
    edge_joint_resolution_ = degToRad(
      getParameter<double>("edge_joint_resolution_deg", 2.5));
    per_arm_candidate_limit_ = static_cast<size_t>(std::max(
      4, getParameter<int>("per_arm_candidate_limit", 20)));
    collision_inset_ = std::max(
      0.0, getParameter<double>("collision_inset", 0.002));
    control_handle_forward_offset_ = std::max(
      0.35, getParameter<double>("control_handle_forward_offset", 0.45));
    control_handle_lateral_offset_ = std::max(
      0.55, getParameter<double>("control_handle_lateral_offset", 0.85));
    playback_rate_hz_ = std::max(
      1.0, getParameter<double>("playback_rate_hz", 20.0));
    auto_run_once_ = getParameter<bool>("auto_run_once", false);
    if (box_size_ <= 0.0 || cartesian_step_ <= 0.0 || angular_step_ <= 0.0 ||
        psi_step_ <= 0.0) {
      throw std::invalid_argument(
              "box_size, cartesian_step, angular_step and psi_step must be positive");
    }

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("failed to load robot model");
    }
    left_group_ = robot_model_->getJointModelGroup(left_group_name_);
    right_group_ = robot_model_->getJointModelGroup(right_group_name_);
    dual_group_ = robot_model_->getJointModelGroup(dual_group_name_);
    if (!left_group_ || !right_group_ || !dual_group_ ||
        left_group_->getVariableCount() != 7U || right_group_->getVariableCount() != 7U ||
        dual_group_->getVariableCount() != 14U) {
      throw std::runtime_error("expected left/right seven-axis and dual fourteen-axis groups");
    }
    for (const auto& link : {arm_base_link_, left_tool_link_, right_tool_link_}) {
      if (!robot_model_->hasLinkModel(link)) {
        throw std::runtime_error("missing link: " + link);
      }
    }

    all_joint_names_.reserve(14);
    for (const std::string side : {std::string("left"), std::string("right")}) {
      for (int index = 1; index <= 7; ++index) {
        all_joint_names_.push_back(side + "_joint" + std::to_string(index));
      }
    }
    left_solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      V3RedundantArmModel::V311Left);
    right_solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      V3RedundantArmModel::V311Right);
    scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    PlanningMetrics initialization_metrics;
    const auto initialization_started = std::chrono::steady_clock::now();
    const auto initial = solveInitialGrasp(initial_box_center_, &initialization_metrics);
    initialization_metrics.initialization_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - initialization_started).count();
    initial_grasp_valid_ = initial.has_value();
    current_state_ = initial ? initial->state : std::make_shared<moveit::core::RobotState>(robot_model_);
    if (!initial) {
      current_state_->setToDefaultValues();
      current_state_->setToDefaultValues(dual_group_, "home");
      current_state_->update(true);
    }
    display_state_ = std::make_shared<moveit::core::RobotState>(*current_state_);
    display_box_center_ = initial_box_center_;
    display_box_roll_ = current_box_roll_;
    if (initial) {
      left_orientation_ = initial->left_orientation;
      right_orientation_ = initial->right_orientation;
      setHeldBoxTransform(initial_box_center_, current_box_roll_);
      attachHeldBox(*current_state_);
      attachHeldBox(*display_state_);
    }
    scene_->setCurrentState(*current_state_);
    initialization_metrics_ = initialization_metrics;
    initial_state_ = std::make_shared<moveit::core::RobotState>(*current_state_);

    task_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/task_json", rclcpp::QoS(1).reliable().transient_local());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    scene_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/scene_markers", rclcpp::QoS(1).reliable().transient_local());
    status_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/status_markers", rclcpp::QoS(1).reliable().transient_local());
    run_service_ = create_service<std_srvs::srv::Trigger>(
      "~/run_current_target",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = requestPlanning();
        response->message = response->success ?
          "planning request accepted" : (initial_grasp_valid_ ? "planner is already running" :
            "initial grasp failed; inspect diagnostic and restart with a reachable initial box pose");
      });

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "v3_dual_arm_cartesian_box_demo_marker",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());
    createTargetMarker();
    menu_handler_.insert(
      motion_mode_ == "roll" ? "确认并计算同步绕X旋转" : "确认并计算同步直线",
      [this](const Feedback::ConstSharedPtr&) {requestPlanning();});
    menu_handler_.insert(
      "目标恢复到当前箱位",
      [this](const Feedback::ConstSharedPtr&) {resetTargetToCurrent();});
    menu_handler_.insert(
      "机器人恢复初始握持位",
      [this](const Feedback::ConstSharedPtr&) {restoreInitialState();});
    menu_handler_.apply(*marker_server_, kMarkerName);
    marker_server_->applyChanges();

    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds(25), [this]() {onWorkerTimer();});
    const auto display_period = std::chrono::duration<double>(1.0 / playback_rate_hz_);
    display_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(display_period),
      [this]() {publishDisplayState();});
    publishPreview(
      motion_mode_ == "roll" ?
      "拖动蓝色旋转环；右键确认后计算双臂同步解析绕箱体中心旋转" :
      "拖动青色控制球；右键确认后计算双臂同步解析直线");
    if (!initial) {
      TaskResult failure;
      failure.failure_stage = "initial_grasp";
      failure.failure_reason = "no collision-free dual-arm initial grasp; " + initial_grasp_reason_;
      failure.frames.push_back({"initial_state", allJoints(*current_state_), initial_box_center_, 0.0});
      makeFailureDiagnostic(failure, initial_rejected_state_, initial_box_center_, target_box_roll_);
      display_diagnostic_ = failure.diagnostic;
      playback_frames_ = failure.diagnostic_frames;
      playback_index_ = 0;
      publishTaskResult(++generation_, initial_box_center_, target_box_center_, failure);
      publishStatus("INITIAL GRASP FAILED - DIAGNOSTIC ONLY", false);
    } else {
      publishStatus("READY", true);
    }

    if (auto_run_once_ && initial_grasp_valid_) {
      auto_run_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() {
          auto_run_timer_->cancel();
          requestPlanning();
        });
    }
    RCLCPP_INFO(
      get_logger(),
      "V3 dual-arm rigid box demo ready: mode=%s current=[%.3f, %.3f, %.3f]/%.1fdeg "
      "target=[%.3f, %.3f, %.3f]/%.1fdeg box=%.2fm init=%.3fms",
      motion_mode_.c_str(),
      current_box_center_.x(), current_box_center_.y(), current_box_center_.z(),
      current_box_roll_ * 180.0 / kPi,
      target_box_center_.x(), target_box_center_.y(), target_box_center_.z(),
      target_box_roll_ * 180.0 / kPi,
      box_size_, initialization_metrics_.initialization_ms);
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

  std::array<double, 7> armJoints(
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group) const
  {
    std::vector<double> values;
    state.copyJointGroupPositions(group, values);
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

  bool upwardGrasp(bool left) const
  {
    return
      (grasp_pattern_ == "left_side_right_bottom" && !left) ||
      (grasp_pattern_ == "right_side_left_bottom" && left);
  }

  Eigen::Matrix3d inwardOrientation(bool left, double roll) const
  {
    const bool upward = upwardGrasp(left);
    const Eigen::Vector3d tool_z = upward ?
      Eigen::Vector3d::UnitZ() :
      (left ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(0.0, -1.0, 0.0));
    const Eigen::Vector3d tool_x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d tool_y = tool_z.cross(tool_x);
    Eigen::Matrix3d base;
    base.col(0) = tool_x;
    base.col(1) = tool_y;
    base.col(2) = tool_z;
    return base * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  Eigen::Vector3d contactOffset(bool left) const
  {
    const bool upward = upwardGrasp(left);
    if (upward) {
      return Eigen::Vector3d(
        0.0, left ? -upward_contact_lateral_offset_ : upward_contact_lateral_offset_,
        -box_size_ * 0.5);
    }
    return Eigen::Vector3d(
      0.0, left ? -box_size_ * 0.5 : box_size_ * 0.5, 0.0);
  }

  Eigen::Isometry3d toolPose(
    const Eigen::Vector3d& box_center,
    bool left,
    const Eigen::Matrix3d& orientation,
    double box_roll = 0.0) const
  {
    const Eigen::Matrix3d box_rotation = Eigen::AngleAxisd(
      box_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = box_center + box_rotation * contactOffset(left);
    pose.linear() = box_rotation * orientation;
    return pose;
  }

  std::vector<ArmCandidate> solveArmCandidates(
    const Eigen::Isometry3d& target_world,
    const moveit::core::RobotState& seed_state,
    const moveit::core::JointModelGroup* group,
    const V3RedundantArmAnalyticIk& solver,
    bool enforce_step,
    PlanningMetrics* metrics) const
  {
    const Eigen::Isometry3d world_to_arm_base =
      seed_state.getGlobalLinkTransform(arm_base_link_).inverse();
    const auto seed = armJoints(seed_state, group);
    const int intervals = std::max(
      1, static_cast<int>(std::ceil(2.0 * kPi / psi_step_)));
    std::vector<double> swivel_samples;
    swivel_samples.reserve(static_cast<size_t>(intervals) + 5U);
    if (enforce_step) {
      const double seed_swivel = solver.swivelAngle(seed);
      for (const double offset : {
          0.0, -0.5 * psi_step_, 0.5 * psi_step_, -psi_step_, psi_step_}) {
        swivel_samples.push_back(normalizedAngle(seed_swivel + offset));
      }
    }
    for (int index = 0; index < intervals; ++index) {
      const double sample = -kPi + static_cast<double>(index) * 2.0 * kPi / intervals;
      const bool duplicate = std::any_of(
        swivel_samples.begin(), swivel_samples.end(),
        [sample](double existing) {
          return std::abs(normalizedAngle(existing - sample)) < 1e-9;
        });
      if (!duplicate) {
        swivel_samples.push_back(sample);
      }
    }
    std::vector<ArmCandidate> candidates;
    for (double swivel : swivel_samples) {
      V3RedundantIkRequest request;
      request.target_in_arm_base = world_to_arm_base * target_world;
      request.swivel_angle = swivel;
      request.seed = seed;
      const auto started = std::chrono::steady_clock::now();
      const auto solutions = solver.solveInArmBase(request);
      if (metrics) {
        ++metrics->ik_calls;
        metrics->ik_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      }
      for (const auto& solution : solutions) {
        if (enforce_step && rawMaximumJointDelta(seed, solution.joints) > maximum_joint_step_) {
          continue;
        }
        const bool duplicate = std::any_of(
          candidates.begin(), candidates.end(),
          [&solution](const ArmCandidate& existing) {
            return rawMaximumJointDelta(existing.joints, solution.joints) < 1e-6;
          });
        if (duplicate) {
          continue;
        }
        ArmCandidate candidate;
        candidate.joints = solution.joints;
        candidate.solution = solution;
        candidate.score = squaredJointDistance(seed, solution.joints) +
          0.02 / std::max(0.01, solution.minimum_joint_limit_margin);
        candidates.push_back(std::move(candidate));
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const ArmCandidate& lhs, const ArmCandidate& rhs) {
        return lhs.score < rhs.score;
      });
    if (candidates.size() > per_arm_candidate_limit_) {
      candidates.resize(per_arm_candidate_limit_);
    }
    return candidates;
  }

  std::string collisionReason(
    const moveit::core::RobotState& state,
    PlanningMetrics* metrics) const
  {
    const auto started = std::chrono::steady_clock::now();
    const std::string reason = alfa_robot::motion::scene_collision_reason(
      scene_, state, dual_group_);
    if (metrics) {
      ++metrics->collision_checks;
      metrics->collision_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    }
    return reason;
  }

  void setHeldBoxTransform(const Eigen::Vector3d& box_center, double box_roll)
  {
    const Eigen::Isometry3d left_pose = toolPose(
      box_center, true, left_orientation_, box_roll);
    Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
    box_pose.translation() = box_center;
    box_pose.linear() = Eigen::AngleAxisd(
      box_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    left_tool_to_box_ = left_pose.inverse() * box_pose;
  }

  void attachHeldBox(moveit::core::RobotState& state) const
  {
    if (state.hasAttachedBody(kHeldBoxId)) {
      return;
    }
    const double collision_size = std::max(0.001, box_size_ - 2.0 * collision_inset_);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(std::make_shared<shapes::Box>(
      collision_size, collision_size, collision_size));
    EigenSTL::vector_Isometry3d shape_poses;
    shape_poses.push_back(left_tool_to_box_);
    const std::vector<std::string> touch_links = {
      left_tool_link_, "left_joint7", "left_joint6",
      right_tool_link_, "right_joint7", "right_joint6",
    };
    state.attachBody(
      kHeldBoxId,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
      touch_links,
      left_tool_link_);
    state.update(true);
  }

  bool synchronizedEdgeClear(
    const moveit::core::RobotState& from,
    const moveit::core::RobotState& to,
    PlanningMetrics* metrics,
    std::string* reason,
    moveit::core::RobotStatePtr* rejected = nullptr) const
  {
    const auto left_from = armJoints(from, left_group_);
    const auto left_to = armJoints(to, left_group_);
    const auto right_from = armJoints(from, right_group_);
    const auto right_to = armJoints(to, right_group_);
    const double maximum_delta = std::max(
      rawMaximumJointDelta(left_from, left_to),
      rawMaximumJointDelta(right_from, right_to));
    const size_t steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_resolution_)));
    for (size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      std::array<double, 7> left{};
      std::array<double, 7> right{};
      for (size_t index = 0; index < 7U; ++index) {
        left[index] = left_from[index] + (left_to[index] - left_from[index]) * ratio;
        right[index] = right_from[index] + (right_to[index] - right_from[index]) * ratio;
      }
      moveit::core::RobotState probe(from);
      probe.setJointGroupPositions(left_group_, left.data());
      probe.setJointGroupPositions(right_group_, right.data());
      probe.update(true);
      if (!probe.satisfiesBounds(left_group_) || !probe.satisfiesBounds(right_group_)) {
        if (rejected) *rejected = std::make_shared<moveit::core::RobotState>(probe);
        if (reason) *reason = "joint_bounds";
        return false;
      }
      const std::string collision = collisionReason(probe, metrics);
      if (!collision.empty()) {
        if (rejected) *rejected = std::make_shared<moveit::core::RobotState>(probe);
        if (reason) *reason = collision;
        return false;
      }
    }
    return true;
  }

  std::optional<moveit::core::RobotStatePtr> solveSynchronizedStep(
    const Eigen::Vector3d& box_center,
    double box_roll,
    const moveit::core::RobotState& seed_state,
    PlanningMetrics* metrics,
    std::string* rejection,
    moveit::core::RobotStatePtr* rejected = nullptr) const
  {
    const auto left_candidates = solveArmCandidates(
      toolPose(box_center, true, left_orientation_, box_roll), seed_state,
      left_group_, *left_solver_, true, metrics);
    const auto right_candidates = solveArmCandidates(
      toolPose(box_center, false, right_orientation_, box_roll), seed_state,
      right_group_, *right_solver_, true, metrics);
    if (left_candidates.empty() || right_candidates.empty()) {
      if (rejection) {
        *rejection = "analytic candidates left=" + std::to_string(left_candidates.size()) +
          " right=" + std::to_string(right_candidates.size());
      }
      return std::nullopt;
    }
    struct PairIndex
    {
      size_t left = 0;
      size_t right = 0;
      double score = 0.0;
    };
    std::vector<PairIndex> pairs;
    pairs.reserve(left_candidates.size() * right_candidates.size());
    for (size_t left = 0; left < left_candidates.size(); ++left) {
      for (size_t right = 0; right < right_candidates.size(); ++right) {
        pairs.push_back(PairIndex{
          left, right, left_candidates[left].score + right_candidates[right].score});
      }
    }
    std::sort(
      pairs.begin(), pairs.end(),
      [](const PairIndex& lhs, const PairIndex& rhs) {return lhs.score < rhs.score;});
    std::string last_reason = "no pair checked";
    for (const auto& pair : pairs) {
      if (metrics) ++metrics->pair_checks;
      auto candidate = std::make_shared<moveit::core::RobotState>(seed_state);
      candidate->setJointGroupPositions(
        left_group_, left_candidates[pair.left].joints.data());
      candidate->setJointGroupPositions(
        right_group_, right_candidates[pair.right].joints.data());
      candidate->update(true);
      if (!candidate->satisfiesBounds(left_group_) ||
          !candidate->satisfiesBounds(right_group_)) {
        if (rejected) *rejected = candidate;
        last_reason = "joint_bounds";
        continue;
      }
      const std::string collision = collisionReason(*candidate, metrics);
      if (!collision.empty()) {
        if (rejected) *rejected = candidate;
        last_reason = collision;
        continue;
      }
      std::string edge_reason;
      if (!synchronizedEdgeClear(seed_state, *candidate, metrics, &edge_reason, rejected)) {
        last_reason = edge_reason;
        continue;
      }
      if (rejected) rejected->reset();
      return candidate;
    }
    if (rejection) {
      *rejection = "pairs=" + std::to_string(pairs.size()) + " last=" + last_reason;
    }
    return std::nullopt;
  }

  std::optional<DualCandidate> solveInitialGrasp(
    const Eigen::Vector3d& box_center,
    PlanningMetrics* metrics)
  {
    moveit::core::RobotState seed(robot_model_);
    seed.setToDefaultValues();
    seed.update(true);
    std::vector<double> left_rolls = {0.0, kPi / 2.0, -kPi / 2.0, kPi};
    std::vector<double> right_rolls = left_rolls;
    if (grasp_pattern_ != "inward") {
      left_rolls = {0.0};
      right_rolls = {0.0};
    }
    std::optional<DualCandidate> best;
    Eigen::Matrix3d previous_left_orientation = left_orientation_;
    Eigen::Matrix3d previous_right_orientation = right_orientation_;
    for (double left_roll : left_rolls) {
      for (double right_roll : right_rolls) {
        left_orientation_ = inwardOrientation(true, left_roll);
        right_orientation_ = inwardOrientation(false, right_roll);
        setHeldBoxTransform(box_center, 0.0);
        const auto left_candidates = solveArmCandidates(
          toolPose(box_center, true, left_orientation_), seed,
          left_group_, *left_solver_, false, metrics);
        const auto right_candidates = solveArmCandidates(
          toolPose(box_center, false, right_orientation_), seed,
          right_group_, *right_solver_, false, metrics);
        if (motion_mode_ == "roll") {
          const auto target_left_candidates = solveArmCandidates(
            toolPose(box_center, true, left_orientation_, target_box_roll_), seed,
            left_group_, *left_solver_, false, nullptr);
          const auto target_right_candidates = solveArmCandidates(
            toolPose(box_center, false, right_orientation_, target_box_roll_), seed,
            right_group_, *right_solver_, false, nullptr);
          RCLCPP_DEBUG(
            get_logger(),
            "roll grasp endpoint check wrist_roll=[%.0f, %.0f]deg target_candidates=[%zu, %zu]",
            left_roll * 180.0 / kPi, right_roll * 180.0 / kPi,
            target_left_candidates.size(), target_right_candidates.size());
          if (target_left_candidates.empty() || target_right_candidates.empty()) {
            initial_grasp_reason_ = "no analytic configuration at requested roll endpoint";
            continue;
          }
        }
        size_t collision_rejects = 0;
        std::string last_collision;
        for (const auto& left : left_candidates) {
          for (const auto& right : right_candidates) {
            if (metrics) ++metrics->pair_checks;
            auto candidate = std::make_shared<moveit::core::RobotState>(seed);
            candidate->setJointGroupPositions(left_group_, left.joints.data());
            candidate->setJointGroupPositions(right_group_, right.joints.data());
            candidate->update(true);
            attachHeldBox(*candidate);
            const std::string collision = collisionReason(*candidate, metrics);
            if (!collision.empty()) {
              ++collision_rejects;
              initial_rejected_state_ = std::make_shared<moveit::core::RobotState>(*candidate);
              initial_grasp_reason_ = collision;
              last_collision = collision;
              continue;
            }
            const double roll_penalty = 0.02 * (std::abs(left_roll) + std::abs(right_roll));
            const double score = left.score + right.score + roll_penalty;
            if (!best || score < best->score) {
              best = DualCandidate{
                candidate, left_orientation_, right_orientation_, score};
            }
          }
        }
        RCLCPP_DEBUG(
          get_logger(),
          "initial grasp rolls=[%.0f, %.0f]deg candidates=[%zu, %zu] "
          "collision_rejects=%zu last=%s",
          left_roll * 180.0 / kPi, right_roll * 180.0 / kPi,
          left_candidates.size(), right_candidates.size(), collision_rejects,
          last_collision.empty() ? "-" : last_collision.c_str());
      }
    }
    if (best) {
      left_orientation_ = best->left_orientation;
      right_orientation_ = best->right_orientation;
      setHeldBoxTransform(box_center, 0.0);
      best->state->clearAttachedBody(kHeldBoxId);
      attachHeldBox(*best->state);
    } else {
      left_orientation_ = previous_left_orientation;
      right_orientation_ = previous_right_orientation;
    }
    return best;
  }

  void makeFailureDiagnostic(TaskResult& result, const moveit::core::RobotStatePtr& rejected,
    const Eigen::Vector3d& box_center, double box_roll) const
  {
    result.diagnostic_frames = result.frames;
    result.diagnostic = {{"diagnostic_only", true}, {"freeze_at_end", true},
      {"stage", result.failure_stage}, {"reason", result.failure_reason},
      {"target", {box_center.x(), box_center.y(), box_center.z()}},
      {"target_box_roll", box_roll}, {"contacts", nlohmann::json::array()},
      {"snapshot", rejected ? "rejected_candidate_not_a_planned_path" : "last_available_state_no_rejected_configuration"}};
    if (rejected) {
      // Derive the displayed box from the same attached transform used by collision checking.
      const auto* body = rejected->getAttachedBody(kHeldBoxId);
      const auto pose = body->getGlobalCollisionBodyTransforms().front();
      const double roll = std::atan2(pose.linear()(2, 1), pose.linear()(1, 1));
      result.diagnostic_frames.push_back({"REJECTED_SNAPSHOT: " + result.failure_stage,
        allJoints(*rejected), pose.translation(), roll});
      collision_detection::CollisionRequest request;
      collision_detection::CollisionResult contacts;
      request.contacts = true;
      request.max_contacts = 20;
      request.max_contacts_per_pair = 1;
      request.group_name = dual_group_name_;
      scene_->checkCollision(request, contacts, *rejected);
      for (const auto& pair : contacts.contacts)
        for (const auto& contact : pair.second)
          result.diagnostic["contacts"].push_back({{"bodies", {pair.first.first, pair.first.second}},
            {"position", {contact.pos.x(), contact.pos.y(), contact.pos.z()}}});
    } else {
      result.diagnostic_frames.back().stage = "FAILED_HOLD: " + result.failure_stage;
    }
  }

  TaskResult planTask(
    const Eigen::Vector3d& start_center,
    const Eigen::Vector3d& target_center,
    double start_roll,
    double target_roll,
    const moveit::core::RobotState& start_state)
  {
    TaskResult result;
    const auto total_started = std::chrono::steady_clock::now();
    result.metrics.initialization_ms = initialization_metrics_.initialization_ms;
    result.frames.push_back(ReplayFrame{
      "start", allJoints(start_state), start_center, start_roll});
    const Eigen::Vector3d delta = target_center - start_center;
    const double distance = delta.norm();
    const double roll_delta = normalizedAngle(target_roll - start_roll);
    const size_t steps = std::max<size_t>(
      1, motion_mode_ == "roll" ?
      static_cast<size_t>(std::ceil(std::abs(roll_delta) / angular_step_)) :
      static_cast<size_t>(std::ceil(distance / cartesian_step_)));
    auto current = std::make_shared<moveit::core::RobotState>(start_state);
    const auto cartesian_started = std::chrono::steady_clock::now();
    for (size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const Eigen::Vector3d box_center = motion_mode_ == "roll" ?
        start_center : start_center + delta * ratio;
      const double box_roll = motion_mode_ == "roll" ?
        normalizedAngle(start_roll + roll_delta * ratio) : start_roll;
      std::string rejection;
      moveit::core::RobotStatePtr rejected;
      const auto next = solveSynchronizedStep(
        box_center, box_roll, *current, &result.metrics, &rejection, &rejected);
      if (!next) {
        result.failure_stage = motion_mode_ == "roll" ?
          "synchronized_box_roll" : "synchronized_cartesian";
        result.failure_reason = "step " + std::to_string(step) + "/" +
          std::to_string(steps) + " " + rejection;
        makeFailureDiagnostic(result, rejected, box_center, box_roll);
        result.metrics.cartesian_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - cartesian_started).count();
        result.total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - total_started).count();
        return result;
      }
      current = *next;
      result.frames.push_back(ReplayFrame{
        motion_mode_ == "roll" ? "synchronized_box_roll" : "synchronized_cartesian",
        allJoints(*current), box_center, box_roll});
    }
    result.metrics.cartesian_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cartesian_started).count();
    result.success = true;
    result.final_state = current;
    result.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - total_started).count();
    return result;
  }

  Eigen::Vector3d controlHandlePosition(const Eigen::Vector3d& target) const
  {
    return target + Eigen::Vector3d(
      -control_handle_forward_offset_, -control_handle_lateral_offset_, 0.0);
  }

  void createTargetMarker()
  {
    const Eigen::Vector3d handle = motion_mode_ == "roll" ?
      target_box_center_ : controlHandlePosition(target_box_center_);
    InteractiveMarker marker;
    marker.header.frame_id = world_frame_;
    marker.name = kMarkerName;
    marker.description = motion_mode_ == "roll" ?
      "绕箱体中心X轴旋转：右键确认同步旋转" :
      "目标箱中心XYZ控制球：右键确认同步直线";
    marker.scale = 0.65;
    marker.pose.position.x = handle.x();
    marker.pose.position.y = handle.y();
    marker.pose.position.z = handle.z();
    const Eigen::Quaterniond target_orientation(Eigen::AngleAxisd(
      target_box_roll_, Eigen::Vector3d::UnitX()));
    marker.pose.orientation.w = target_orientation.w();
    marker.pose.orientation.x = target_orientation.x();
    marker.pose.orientation.y = target_orientation.y();
    marker.pose.orientation.z = target_orientation.z();
    InteractiveMarkerControl body;
    body.always_visible = true;
    body.interaction_mode = motion_mode_ == "roll" ?
      InteractiveMarkerControl::BUTTON : InteractiveMarkerControl::MOVE_3D;
    Marker sphere;
    sphere.type = Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.09;
    sphere.color = color(0.10F, 0.85F, 1.0F, 0.95F);
    body.markers.push_back(sphere);
    marker.controls.push_back(body);
    if (motion_mode_ == "roll") {
      marker.controls.push_back(rotationControl("rotate_x", 1.0, 0.0, 0.0));
    } else {
      marker.controls.push_back(axisControl("move_x", 1.0, 0.0, 0.0));
      marker.controls.push_back(axisControl("move_y", 0.0, 1.0, 0.0));
      marker.controls.push_back(axisControl("move_z", 0.0, 0.0, 1.0));
    }
    marker_server_->insert(
      marker,
      [this](const Feedback::ConstSharedPtr& feedback) {handleTargetFeedback(feedback);});
  }

  void handleTargetFeedback(const Feedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type != Feedback::POSE_UPDATE &&
        feedback->event_type != Feedback::MOUSE_UP) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (motion_mode_ == "roll") {
        const auto& orientation = feedback->pose.orientation;
        target_box_center_ = current_box_center_;
        target_box_roll_ = normalizedAngle(std::atan2(
          2.0 * (orientation.w * orientation.x +
          orientation.y * orientation.z),
          1.0 - 2.0 * (orientation.x * orientation.x +
          orientation.y * orientation.y)));
      } else {
        target_box_center_ = Eigen::Vector3d(
          feedback->pose.position.x + control_handle_forward_offset_,
          feedback->pose.position.y + control_handle_lateral_offset_,
          feedback->pose.position.z);
      }
    }
    publishPreview(
      motion_mode_ == "roll" ?
      "目标旋转角已更新，右键旋转环确认后计算" :
      "目标已更新，右键控制球确认后计算");
  }

  void setMarkerToTarget()
  {
    const Eigen::Vector3d handle = motion_mode_ == "roll" ?
      target_box_center_ : controlHandlePosition(target_box_center_);
    geometry_msgs::msg::Pose pose;
    pose.position.x = handle.x();
    pose.position.y = handle.y();
    pose.position.z = handle.z();
    const Eigen::Quaterniond target_orientation(Eigen::AngleAxisd(
      target_box_roll_, Eigen::Vector3d::UnitX()));
    pose.orientation.w = target_orientation.w();
    pose.orientation.x = target_orientation.x();
    pose.orientation.y = target_orientation.y();
    pose.orientation.z = target_orientation.z();
    marker_server_->setPose(kMarkerName, pose);
    marker_server_->applyChanges();
  }

  void resetTargetToCurrent()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      target_box_center_ = current_box_center_;
      target_box_roll_ = current_box_roll_;
      setMarkerToTarget();
    }
    publishPreview("目标已恢复到当前箱位");
  }

  void restoreInitialState()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = std::make_shared<moveit::core::RobotState>(*initial_state_);
      current_box_center_ = initial_box_center_;
      target_box_center_ = initial_box_center_;
      current_box_roll_ = 0.0;
      target_box_roll_ = 0.0;
      display_state_ = std::make_shared<moveit::core::RobotState>(*initial_state_);
      display_box_center_ = initial_box_center_;
      display_box_roll_ = 0.0;
      display_diagnostic_ = nlohmann::json::object();
      display_failure_frozen_ = false;
      playback_frames_.clear();
      playback_index_ = 0;
      setMarkerToTarget();
    }
    publishPreview("机器人和箱体已恢复初始握持位");
  }

  bool requestPlanning()
  {
    if (!initial_grasp_valid_) return false;
    if (planning_active_.load() || planning_requested_.exchange(true)) {
      return false;
    }
    publishStatus("PLANNING REQUESTED", true);
    return true;
  }

  void onWorkerTimer()
  {
    if (!planning_requested_.exchange(false) || planning_active_.exchange(true)) {
      return;
    }
    Eigen::Vector3d start_center;
    Eigen::Vector3d target_center;
    double start_roll = 0.0;
    double target_roll = 0.0;
    moveit::core::RobotStatePtr start_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start_center = current_box_center_;
      target_center = target_box_center_;
      start_roll = current_box_roll_;
      target_roll = target_box_roll_;
      start_state = std::make_shared<moveit::core::RobotState>(*current_state_);
    }
    {
      std::lock_guard<std::mutex> lock(display_mutex_);
      playback_frames_.clear();
      display_diagnostic_ = nlohmann::json::object();
      display_failure_frozen_ = false;
    }
    const uint64_t generation = ++generation_;
    publishPlanningStarted(generation, start_center, target_center);
    publishStatus("CALCULATING", true);
    RCLCPP_INFO(
      get_logger(),
      "[%llu] calculation started: mode=%s start=[%.3f, %.3f, %.3f]/%.1fdeg "
      "target=[%.3f, %.3f, %.3f]/%.1fdeg",
      static_cast<unsigned long long>(generation),
      motion_mode_.c_str(),
      start_center.x(), start_center.y(), start_center.z(),
      start_roll * 180.0 / kPi,
      target_center.x(), target_center.y(), target_center.z(),
      target_roll * 180.0 / kPi);
    TaskResult result;
    try {
      result = planTask(
        start_center, target_center, start_roll, target_roll, *start_state);
    } catch (const std::exception& error) {
      result.failure_stage = "exception";
      result.failure_reason = error.what();
      result.frames.push_back({"FAILED_HOLD: exception", allJoints(*start_state), start_center, start_roll});
      makeFailureDiagnostic(result, nullptr, target_center, target_roll);
    }
    if (!result.frames.empty()) {
      std::lock_guard<std::mutex> lock(display_mutex_);
      playback_frames_ = result.success || result.diagnostic_frames.empty() ? result.frames : result.diagnostic_frames;
      display_diagnostic_ = result.success ? nlohmann::json::object() : result.diagnostic;
      playback_index_ = 0;
    }
    if (result.success && result.final_state) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = result.final_state;
      current_box_center_ = target_center;
      current_box_roll_ = target_roll;
      scene_->setCurrentState(*current_state_);
    }
    publishTaskResult(generation, start_center, target_center, result);
    if (result.success) {
      publishStatus(
        "SUCCESS total=" + std::to_string(result.total_ms) + "ms", true);
      RCLCPP_INFO(
        get_logger(),
        "[%llu] calculation completed: SUCCESS total=%.3fms cartesian=%.3fms "
        "ik=%llu/%.3fms pairs=%llu collision=%llu/%.3fms frames=%zu",
        static_cast<unsigned long long>(generation), result.total_ms,
        result.metrics.cartesian_ms,
        static_cast<unsigned long long>(result.metrics.ik_calls), result.metrics.ik_ms,
        static_cast<unsigned long long>(result.metrics.pair_checks),
        static_cast<unsigned long long>(result.metrics.collision_checks),
        result.metrics.collision_ms, result.frames.size());
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

  nlohmann::json baseJson(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& target,
    double current_roll,
    double target_roll) const
  {
    return {
      {"motion_mode", motion_mode_},
      {"grasp_pattern", grasp_pattern_},
      {"current_box_center", {current.x(), current.y(), current.z()}},
      {"target_box_center", {target.x(), target.y(), target.z()}},
      {"current_box_roll", current_roll},
      {"target_box_roll", target_roll},
      {"box_size", box_size_},
      {"left_tool_link", left_tool_link_},
      {"right_tool_link", right_tool_link_},
      {"grasp_spacing", (contactOffset(true) - contactOffset(false)).norm()},
    };
  }

  void publishJson(const nlohmann::json& payload)
  {
    std_msgs::msg::String message;
    message.data = payload.dump();
    task_publisher_->publish(message);
  }

  void publishPreview(const std::string& status)
  {
    Eigen::Vector3d current;
    Eigen::Vector3d target;
    double current_roll = 0.0;
    double target_roll = 0.0;
    std::vector<double> current_joints;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_box_center_;
      target = target_box_center_;
      current_roll = current_box_roll_;
      target_roll = target_box_roll_;
      current_joints = allJoints(*current_state_);
    }
    auto payload = baseJson(current, target, current_roll, target_roll);
    payload["kind"] = "preview";
    payload["status"] = status;
    payload["joint_names"] = all_joint_names_;
    payload["current_joints"] = current_joints;
    publishJson(payload);
    publishSceneMarkers(
      current, target, current, current_roll, target_roll, current_roll);
  }

  void publishPlanningStarted(
    uint64_t generation,
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& target)
  {
    double current_roll = 0.0;
    double target_roll = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_roll = current_box_roll_;
      target_roll = target_box_roll_;
    }
    auto payload = baseJson(current, target, current_roll, target_roll);
    payload["kind"] = "planning";
    payload["generation"] = generation;
    payload["status"] = "计算开始";
    publishJson(payload);
  }

  void publishTaskResult(
    uint64_t generation,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& target,
    const TaskResult& result)
  {
    double start_roll = 0.0;
    double target_roll = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start_roll = result.frames.empty() ? current_box_roll_ : result.frames.front().box_roll;
      target_roll = target_box_roll_;
    }
    auto payload = baseJson(start, target, start_roll, target_roll);
    payload["kind"] = "result";
    payload["generation"] = generation;
    payload["success"] = result.success;
    payload["failure_stage"] = result.failure_stage;
    payload["failure_reason"] = result.failure_reason;
    payload["total_ms"] = result.total_ms;
    payload["metrics"] = {
      {"initialization_ms", result.metrics.initialization_ms},
      {"cartesian_ms", result.metrics.cartesian_ms},
      {"ik_calls", result.metrics.ik_calls},
      {"ik_ms", result.metrics.ik_ms},
      {"pair_checks", result.metrics.pair_checks},
      {"collision_checks", result.metrics.collision_checks},
      {"collision_ms", result.metrics.collision_ms},
    };
    payload["joint_names"] = all_joint_names_;
    payload["frames"] = nlohmann::json::array();
    for (const auto& frame : result.frames) {
      payload["frames"].push_back({
        {"stage", frame.stage},
        {"joints", frame.joints},
        {"box_center", {
          frame.box_center.x(), frame.box_center.y(), frame.box_center.z()}},
        {"box_roll", frame.box_roll},
      });
    }
    payload["diagnostic"] = result.diagnostic;
    payload["diagnostic_frames"] = nlohmann::json::array();
    for (const auto& frame : result.diagnostic_frames)
      payload["diagnostic_frames"].push_back({{"stage", frame.stage}, {"joints", frame.joints},
        {"box_center", {frame.box_center.x(), frame.box_center.y(), frame.box_center.z()}},
        {"box_roll", frame.box_roll}, {"diagnostic_only", true}});
    publishJson(payload);
  }

  void publishStatus(const std::string& status, bool good)
  {
    Eigen::Vector3d target;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      target = target_box_center_;
    }
    visualization_msgs::msg::MarkerArray markers;
    Marker text;
    text.header.frame_id = world_frame_;
    text.header.stamp = now();
    text.ns = "v3_dual_arm_cartesian_box_status";
    text.id = 0;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.position.x = target.x();
    text.pose.position.y = target.y();
    text.pose.position.z = target.z() + box_size_ * 0.7;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.04;
    text.color = good ? color(0.15F, 1.0F, 0.25F) : color(1.0F, 0.12F, 0.12F);
    text.text = status;
    markers.markers.push_back(text);
    status_marker_publisher_->publish(markers);
  }

  void publishSceneMarkers(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& displayed,
    double current_roll,
    double target_roll,
    double displayed_roll)
  {
    visualization_msgs::msg::MarkerArray markers;
    Marker current_box;
    current_box.header.frame_id = world_frame_;
    current_box.header.stamp = now();
    current_box.ns = "current_box";
    current_box.id = 0;
    current_box.type = Marker::CUBE;
    current_box.action = Marker::ADD;
    current_box.pose.position.x = displayed.x();
    current_box.pose.position.y = displayed.y();
    current_box.pose.position.z = displayed.z();
    const Eigen::Quaterniond displayed_orientation(Eigen::AngleAxisd(
      displayed_roll, Eigen::Vector3d::UnitX()));
    current_box.pose.orientation.w = displayed_orientation.w();
    current_box.pose.orientation.x = displayed_orientation.x();
    current_box.pose.orientation.y = displayed_orientation.y();
    current_box.pose.orientation.z = displayed_orientation.z();
    current_box.scale.x = current_box.scale.y = current_box.scale.z = box_size_;
    current_box.color = color(0.12F, 0.45F, 1.0F, 0.75F);
    markers.markers.push_back(current_box);

    Marker target_box = current_box;
    target_box.ns = "target_box";
    target_box.pose.position.x = target.x();
    target_box.pose.position.y = target.y();
    target_box.pose.position.z = target.z();
    const Eigen::Quaterniond target_orientation(Eigen::AngleAxisd(
      target_roll, Eigen::Vector3d::UnitX()));
    target_box.pose.orientation.w = target_orientation.w();
    target_box.pose.orientation.x = target_orientation.x();
    target_box.pose.orientation.y = target_orientation.y();
    target_box.pose.orientation.z = target_orientation.z();
    target_box.color = color(0.20F, 1.0F, 0.25F, 0.28F);
    markers.markers.push_back(target_box);

    Marker line;
    line.header.frame_id = world_frame_;
    line.header.stamp = now();
    line.ns = motion_mode_ == "roll" ? "rotation_axis" : "cartesian_line";
    line.id = 0;
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.012;
    line.color = color(0.95F, 0.2F, 0.95F, 0.95F);
    const std::array<Eigen::Vector3d, 2> line_points = motion_mode_ == "roll" ?
      std::array<Eigen::Vector3d, 2>{
      current - Eigen::Vector3d(0.25, 0.0, 0.0),
      current + Eigen::Vector3d(0.25, 0.0, 0.0)} :
      std::array<Eigen::Vector3d, 2>{current, target};
    for (const auto& point : line_points) {
      geometry_msgs::msg::Point message;
      message.x = point.x();
      message.y = point.y();
      message.z = point.z();
      line.points.push_back(message);
    }
    markers.markers.push_back(line);

    Marker handle_line = line;
    handle_line.ns = "control_handle_line";
    handle_line.scale.x = 0.007;
    handle_line.color = color(0.10F, 0.85F, 1.0F, 0.85F);
    handle_line.points.clear();
    const Eigen::Vector3d handle = motion_mode_ == "roll" ?
      target : controlHandlePosition(target);
    for (const auto& point : {handle, target}) {
      geometry_msgs::msg::Point message;
      message.x = point.x();
      message.y = point.y();
      message.z = point.z();
      handle_line.points.push_back(message);
    }
    markers.markers.push_back(handle_line);
    alfa_robot::motion::appendDemoFailureMarkers(
      markers, display_diagnostic_, display_failure_frozen_, world_frame_, now());
    scene_marker_publisher_->publish(markers);
  }

  void publishDisplayState()
  {
    Eigen::Vector3d current;
    Eigen::Vector3d target;
    Eigen::Vector3d displayed;
    double current_roll = 0.0;
    double target_roll = 0.0;
    double displayed_roll = 0.0;
    sensor_msgs::msg::JointState message;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      current = current_box_center_;
      target = target_box_center_;
      current_roll = current_box_roll_;
      target_roll = target_box_roll_;
    }
    {
      std::lock_guard<std::mutex> display_lock(display_mutex_);
      if (!playback_frames_.empty() && playback_index_ < playback_frames_.size()) {
        display_failure_frozen_ = !display_diagnostic_.empty() && playback_index_ + 1U == playback_frames_.size();
        const auto& frame = playback_frames_[playback_index_];
        for (size_t index = 0;
             index < all_joint_names_.size() && index < frame.joints.size(); ++index) {
          display_state_->setVariablePosition(all_joint_names_[index], frame.joints[index]);
        }
        display_state_->update(true);
        display_box_center_ = frame.box_center;
        display_box_roll_ = frame.box_roll;
        if (playback_index_ + 1U < playback_frames_.size()) {
          ++playback_index_;
        }
      }
      displayed = display_box_center_;
      displayed_roll = display_box_roll_;
      message.header.stamp = now();
      message.name = all_joint_names_;
      message.position = allJoints(*display_state_);
    }
    if (display_failure_frozen_)
      publishStatus("DIAGNOSTIC ONLY - FROZEN (not executable)\n" +
        display_diagnostic_.value("stage", "") + "\n" + display_diagnostic_.value("reason", ""), false);
    joint_state_publisher_->publish(message);
    publishSceneMarkers(
      current, target, displayed, current_roll, target_roll, displayed_roll);
  }

  bool initial_grasp_valid_ = false;
  moveit::core::RobotStatePtr initial_rejected_state_;
  std::string initial_grasp_reason_ = "no analytic configuration available";
  bool display_failure_frozen_ = false;
  nlohmann::json display_diagnostic_ = nlohmann::json::object();
  std::string world_frame_;
  std::string arm_base_link_;
  std::string left_group_name_;
  std::string right_group_name_;
  std::string dual_group_name_;
  std::string left_tool_link_;
  std::string right_tool_link_;
  std::string motion_mode_ = "translate";
  std::string grasp_pattern_ = "inward";
  Eigen::Vector3d initial_box_center_{0.73, 0.0, 0.55};
  Eigen::Vector3d current_box_center_{0.73, 0.0, 0.55};
  Eigen::Vector3d target_box_center_{0.61, 0.0, 0.55};
  Eigen::Vector3d display_box_center_{0.73, 0.0, 0.55};
  double current_box_roll_ = 0.0;
  double target_box_roll_ = 0.0;
  double display_box_roll_ = 0.0;
  Eigen::Matrix3d left_orientation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d right_orientation_ = Eigen::Matrix3d::Identity();
  Eigen::Isometry3d left_tool_to_box_ = Eigen::Isometry3d::Identity();
  double box_size_ = 0.40;
  double upward_contact_lateral_offset_ = 0.10;
  double cartesian_step_ = 0.01;
  double angular_step_ = degToRad(2.0);
  double psi_step_ = degToRad(5.0);
  double maximum_joint_step_ = degToRad(12.0);
  double edge_joint_resolution_ = degToRad(2.5);
  size_t per_arm_candidate_limit_ = 20;
  double collision_inset_ = 0.002;
  double control_handle_forward_offset_ = 0.45;
  double control_handle_lateral_offset_ = 0.85;
  double playback_rate_hz_ = 20.0;
  bool auto_run_once_ = false;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* left_group_ = nullptr;
  const moveit::core::JointModelGroup* right_group_ = nullptr;
  const moveit::core::JointModelGroup* dual_group_ = nullptr;
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<V3RedundantArmAnalyticIk> left_solver_;
  std::unique_ptr<V3RedundantArmAnalyticIk> right_solver_;
  moveit::core::RobotStatePtr initial_state_;
  moveit::core::RobotStatePtr current_state_;
  moveit::core::RobotStatePtr display_state_;
  std::vector<std::string> all_joint_names_;
  PlanningMetrics initialization_metrics_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_marker_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::TimerBase::SharedPtr auto_run_timer_;

  std::mutex state_mutex_;
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
  auto node = std::make_shared<V3DualArmCartesianBoxDemo>(rclcpp::NodeOptions());
  node->init();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
