#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>
#include <alfa_robot_moveit_config/planning_diagnostics.hpp>
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

double squaredJointDistance(
  const std::array<double, 7>& from,
  const std::array<double, 7>& to)
{
  double distance = 0.0;
  for (size_t index = 0; index < from.size(); ++index) {
    const double delta = normalizedAngle(to[index] - from[index]);
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

struct ReplayFrame
{
  std::string stage;
  std::vector<double> joints;
  bool box_attached = false;
};

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
};

struct TaskResult
{
  bool success = false;
  std::string failure_stage;
  std::string failure_reason;
  double total_ms = 0.0;
  PlanningMetrics metrics;
  std::vector<ReplayFrame> frames;
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
      getParameter<double>("edge_joint_resolution_deg", 2.5));
    precontact_candidate_limit_ = static_cast<size_t>(std::max(
      1, getParameter<int>("precontact_candidate_limit", 8)));
    rrt_planning_time_ = std::max(0.05, getParameter<double>("rrt_planning_time", 1.0));
    rrt_planning_attempts_ = std::max(1, getParameter<int>("rrt_planning_attempts", 1));
    auto_run_once_ = getParameter<bool>("auto_run_once", false);

    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be left or right");
    }
    if (planning_group_name_ != side_ + "_arm" || tool_link_ != side_ + "_tool0" ||
        arm_base_link_ != "arm_carriage") {
      throw std::invalid_argument("planning_group/tool_link/arm_base_link must match the V3.0.9 side");
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
    initial_state_->update(true);
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
      shoulder_box_offset_ = getParameter<double>("shoulder_box_offset", 0.25);
      if (!std::isfinite(shoulder_box_offset_) || shoulder_box_offset_ < 0.0) {
        throw std::invalid_argument("shoulder_box_offset must be finite and nonnegative metres");
      }
      const Eigen::Vector3d shoulder_midpoint = 0.5 * (
        V3RedundantArmAnalyticIk(V3RedundantArmModel::V309Left).modelShoulderCenterInArmBase() +
        V3RedundantArmAnalyticIk(V3RedundantArmModel::V309Right).modelShoulderCenterInArmBase());
      initial_shoulder_z_ = (initial_state_->getGlobalLinkTransform(arm_base_link_) *
        shoulder_midpoint).z();
      chassis_front_x_ = getParameter<double>("chassis_front_x", modelChassisFrontX());
      wall_center_y_ = getParameter<double>("wall_center_y", 0.0);
      wall_bottom_z_ = getParameter<double>("wall_bottom_z", 0.0);
      if (!std::isfinite(chassis_front_x_) || !std::isfinite(wall_center_y_) ||
          !std::isfinite(wall_bottom_z_) || wall_bottom_z_ < 0.0 || collision_inset_ != 0.0) {
        throw std::invalid_argument("distance demo requires finite placement, nonnegative bottom and zero collision_inset");
      }
      updateWallTarget(getParameter<double>("x", -1.0), getParameter<int>("box_id", 0));
      initial_box_center_ = box_center_;
      requested_arm_ = getParameter<std::string>("arm", "auto");
      if (requested_arm_ != "auto" && requested_arm_ != "left" && requested_arm_ != "right")
        throw std::invalid_argument("arm must be left/right/auto");
    }

    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side_ == "left" ? V3RedundantArmModel::V309Left : V3RedundantArmModel::V309Right);

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
          if (planning_active_.load() || planning_requested_.load()) {
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

  double modelChassisFrontX() const
  {
    const auto* base = robot_model_->getLinkModel("model_base");
    if (!base) throw std::runtime_error("model_base missing: configure a supported chassis model");
    double front = -std::numeric_limits<double>::infinity();
    const auto& shapes = base->getShapes();
    const auto& origins = base->getCollisionOriginTransforms();
    for (size_t i = 0; i < shapes.size(); ++i) {
      if (shapes[i]->type != shapes::MESH) {
        throw std::runtime_error("chassis front extraction expects model_base collision meshes");
      }
      const auto* mesh = static_cast<const shapes::Mesh*>(shapes[i].get());
      const Eigen::Isometry3d transform = initial_state_->getGlobalLinkTransform(base) * origins[i];
      for (unsigned int v = 0; v < mesh->vertex_count; ++v) {
        const Eigen::Vector3d point(mesh->vertices[3*v], mesh->vertices[3*v+1], mesh->vertices[3*v+2]);
        front = std::max(front, (transform * point).x());
      }
    }
    if (!std::isfinite(front)) throw std::runtime_error("no chassis collision vertices");
    return front;
  }

  void updateWallTarget(double x, int box_id)
  {
    if (!std::isfinite(x) || x <= 0.0 || box_id < 0 || box_id >= 25) {
      throw std::invalid_argument("x must be finite/positive and box_id must be 0..24");
    }
    const Eigen::Vector3d center(chassis_front_x_ + x + box_depth_ / 2.0,
      wall_center_y_ + (box_id % 5 - 2) * (box_width_ + wall_gap_),
      wall_bottom_z_ + box_height_ / 2.0 + (box_id / 5) * (box_height_ + wall_gap_));
    if (!center.allFinite()) throw std::invalid_argument("wall coordinates overflow");
    wall_distance_ = x;
    wall_target_row_ = box_id / 5;
    wall_target_column_ = box_id % 5;
    box_center_ = center;
  }

  void selectArm(const std::string& side)
  {
    side_ = side;
    tool_link_ = side + "_tool0";
    planning_group_name_ = side + "_arm";
    planning_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side == "left" ? V3RedundantArmModel::V309Left : V3RedundantArmModel::V309Right);
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

  bool requestPlanning()
  {
    if (planning_active_.load() || planning_requested_.exchange(true)) {
      return false;
    }
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
    Eigen::Vector3d box_center;
    {
      std::lock_guard<std::mutex> lock(box_mutex_);
      box_center = box_center_;
    }
    const uint64_t generation = ++generation_;
    if (distance_demo_) selectArm(requested_arm_ == "auto" ? "left" : requested_arm_);
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
        "height alignment: enabled=%s shoulder_z=%.6fm offset=%.3fm descent=%.6fm target_updown=%.6fm",
        align_height_ ? "true" : "false", initial_shoulder_z_, shoulder_box_offset_,
        alignment.at("descent").get<double>(), alignment.at("target_updown").get<double>());
    }

    {
      std::lock_guard<std::mutex> lock(display_mutex_);
      playback_frames_.clear();
      display_box_attached_ = false;
      *display_state_ = *initial_state_;
    }
    publishSceneMarkers();
    TaskResult result;
    const auto request_started = std::chrono::steady_clock::now();
    nlohmann::json attempts = nlohmann::json::array();
    try {
      if (distance_demo_) {
        const std::vector<std::string> sides = requested_arm_ == "auto" ?
          std::vector<std::string>{"left", "right"} : std::vector<std::string>{requested_arm_};
        for (const auto& side : sides) {
          selectArm(side);
          result = planTask(box_center);
          attempts.push_back({{"arm", side}, {"success", result.success},
            {"failure_stage", result.failure_stage}, {"failure_reason", result.failure_reason}});
          RCLCPP_INFO(get_logger(), "[%llu] arm=%s %s stage=%s reason=%s",
            static_cast<unsigned long long>(generation), side.c_str(),
            result.success ? "SUCCESS" : "FAILED", result.failure_stage.c_str(),
            result.failure_reason.c_str());
          if (result.success) break;
        }
      } else {
        result = planTask(box_center);
      }
    } catch (const std::exception& error) {
      result.success = false;
      result.failure_stage = "exception";
      result.failure_reason = error.what();
    }
    if (distance_demo_) result.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - request_started).count();
    attempts_ = std::move(attempts);
    publishTaskResult(generation, box_center, result);
    if (!result.frames.empty()) {
      std::lock_guard<std::mutex> lock(display_mutex_);
      playback_frames_ = result.frames;
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

  Eigen::Vector3d toolToBoxCenter() const
  {
    return Eigen::Vector3d(0.0, 0.0, box_depth_ * 0.5 + contact_numerical_gap_);
  }

  Eigen::Isometry3d contactPose(const Eigen::Vector3d& box_center) const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = box_center - Eigen::Vector3d(toolToBoxCenter().z(), 0.0, 0.0);
    pose.linear() = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return pose;
  }

  Eigen::Isometry3d precontactPose(const Eigen::Vector3d& box_center) const
  {
    Eigen::Isometry3d pose = contactPose(box_center);
    pose.translation().x() -= approach_distance_;
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
      std::vector<Eigen::Vector3d> neighbors;
      neighbors.reserve(24);
      for (int row = 0; row < 5; ++row) {
        for (int column = 0; column < 5; ++column) {
          if (row == wall_target_row_ && column == wall_target_column_) continue;
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
    // ponytail: only legacy mode omits the target before contact. The distance demo
    // includes it until attachment; neither mode implements placement/release.
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    scene->setCurrentState(*initial_state_);
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

  void attachCarriedBox(moveit::core::RobotState& state) const
  {
    if (state.hasAttachedBody(kCarriedBoxId)) {
      return;
    }
    const double local_x = std::max(0.001, box_height_ - 2.0 * collision_inset_);
    const double local_y = std::max(0.001, box_width_ - 2.0 * collision_inset_);
    const double local_z = std::max(0.001, box_depth_ - 2.0 * collision_inset_);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(std::make_shared<shapes::Box>(local_x, local_y, local_z));
    EigenSTL::vector_Isometry3d shape_poses;
    Eigen::Isometry3d shape_pose = Eigen::Isometry3d::Identity();
    shape_pose.translation() = toolToBoxCenter();
    shape_poses.push_back(shape_pose);
    const std::string prefix = side_ + "_";
    state.attachBody(
      kCarriedBoxId,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
      distance_demo_ ? std::vector<std::string>{tool_link_, prefix + "joint7"} :
        std::vector<std::string>{tool_link_, prefix + "joint7", prefix + "joint6"},
      tool_link_);
    state.update(true);
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
    std::string* reason) const
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
      if (attached) {
        attachCarriedBox(probe);
      } else if (probe.hasAttachedBody(kCarriedBoxId)) {
        probe.clearAttachedBody(kCarriedBoxId);
      }
      probe.update(true);
      if (!probe.satisfiesBounds(planning_group_)) {
        if (reason) *reason = "joint_bounds";
        return false;
      }
      const std::string collision = collisionReason(scene, probe, metrics);
      if (!collision.empty()) {
        if (reason) *reason = collision;
        return false;
      }
    }
    return true;
  }

  std::vector<AnalyticCandidate> solvePoseCandidates(
    const Eigen::Isometry3d& target_world,
    const moveit::core::RobotState& seed_state,
    bool attached,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    bool enforce_step,
    std::string* rejection_summary) const
  {
    const Eigen::Isometry3d world_to_arm_base =
      seed_state.getGlobalLinkTransform(arm_base_link_).inverse();
    const Eigen::Isometry3d target_in_arm_base = world_to_arm_base * target_world;
    const auto seed_joints = armJoints(seed_state);
    std::vector<AnalyticCandidate> candidates;
    size_t bounds_rejects = 0;
    size_t jump_rejects = 0;
    size_t collision_rejects = 0;
    size_t edge_rejects = 0;
    std::string last_collision;

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
            maximumJointDelta(seed_joints, solution.joints) > maximum_cartesian_joint_step_) {
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
          continue;
        }
        if (enforce_step) {
          std::string edge_reason;
          if (!edgeClear(scene, seed_state, candidate, attached, metrics, &edge_reason)) {
            ++edge_rejects;
            last_collision = edge_reason;
            continue;
          }
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
        output.score = squaredJointDistance(seed_joints, solution.joints) + margin_penalty;
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
              << " edge=" << edge_rejects;
      if (!last_collision.empty()) {
        summary << " last=" << last_collision;
      }
      *rejection_summary = summary.str();
    }
    return candidates;
  }

  std::optional<AnalyticCandidate> solveNextPose(
    const Eigen::Isometry3d& target_world,
    const moveit::core::RobotState& seed_state,
    bool attached,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    std::string* reason) const
  {
    auto candidates = solvePoseCandidates(
      target_world, seed_state, attached, scene, metrics, true, reason);
    if (candidates.empty()) {
      return std::nullopt;
    }
    return candidates.front();
  }

  bool traceCartesianPath(
    const Eigen::Vector3d& box_center,
    const moveit::core::RobotState& precontact_state,
    const planning_scene::PlanningSceneConstPtr& scene,
    PlanningMetrics* metrics,
    std::vector<moveit::core::RobotStatePtr>* approach_states,
    std::vector<moveit::core::RobotStatePtr>* retreat_states,
    std::string* failure_stage,
    std::string* failure_reason) const
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
      target.translation().x() -= approach_distance_ * (1.0 - ratio);
      std::string reason;
      auto next = solveNextPose(target, *current, false,
        distance_demo_ && step == approach_steps ? contact_scene : scene, metrics, &reason);
      if (!next) {
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
      if (failure_stage) *failure_stage = "attach_box";
      if (failure_reason) *failure_reason = attach_collision;
      return false;
    }
    current = std::make_shared<moveit::core::RobotState>(contact_attached);
    retreat_states->push_back(current);

    const size_t retreat_steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(retreat_distance_ / cartesian_step_)));
    for (size_t step = 1; step <= retreat_steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(retreat_steps);
      Eigen::Isometry3d target = contact;
      target.translation().x() -= retreat_distance_ * ratio;
      std::string reason;
      auto next = solveNextPose(target, *current, true, loaded, metrics, &reason);
      if (!next) {
        if (failure_stage) *failure_stage = "cartesian_retreat";
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
      result.reason = "start_" + start_collision;
      return result;
    }
    const std::string goal_collision = alfa_robot::motion::scene_collision_reason(
      scene, goal_state, distance_demo_ ? nullptr : planning_group_);
    if (!goal_collision.empty()) {
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
                       start_state.hasAttachedBody(kCarriedBoxId), nullptr, &result.reason)) {
          result.reason = "rrt_edge_" + result.reason;
          return result;
        }
      }
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
    const double difference = initial_shoulder_z_ - (box_center.z() + shoulder_box_offset_);
    const double descent = align_height_ ? std::max(0.0, difference) : 0.0;
    const double initial = initial_state_->getVariablePosition("updown");
    const auto& bounds = robot_model_->getVariableBounds("updown");
    return {{"enabled", align_height_},
      {"reference", "midpoint of left/right shoulder common-axis centers in world Z"},
      {"initial_shoulder_z", initial_shoulder_z_}, {"box_center_z", box_center.z()},
      {"shoulder_box_offset", shoulder_box_offset_}, {"height_difference", difference},
      {"descent", descent}, {"initial_updown", initial}, {"target_updown", initial - descent},
      {"lower_limit", bounds.min_position_}, {"upper_limit", bounds.max_position_},
      {"collision_sample_step_m", 0.005}, {"return_policy", "keep aligned lift; arms return to zero"}};
  }

  bool alignHeight(
    const Eigen::Vector3d& box_center, const planning_scene::PlanningScenePtr& scene,
    moveit::core::RobotState& grasp_start, TaskResult& result) const
  {
    if (!distance_demo_ || !align_height_) return true;
    const auto alignment = heightAlignment(box_center);
    const double target = alignment.at("target_updown").get<double>();
    const auto& bounds = robot_model_->getVariableBounds("updown");
    if (target < bounds.min_position_ || target > bounds.max_position_) {
      result.failure_stage = "height_alignment_limits";
      result.failure_reason = "required updown=" + std::to_string(target) +
        " outside [" + std::to_string(bounds.min_position_) + ", " +
        std::to_string(bounds.max_position_) + "] metres; not clamped";
      return false;
    }
    const double initial = grasp_start.getVariablePosition("updown");
    if (target == initial) return true;
    std::vector<ReplayFrame> prefix{
      ReplayFrame{"lower_to_box_height", allJoints(grasp_start), false}};
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
        // Reject the complete descent, not a replay that stops just before collision.
        return false;
      }
      prefix.push_back(ReplayFrame{"lower_to_box_height", allJoints(grasp_start), false});
    }
    result.frames = std::move(prefix);
    return true;
  }

  TaskResult planTask(const Eigen::Vector3d& box_center)
  {
    TaskResult result;
    const auto total_started = std::chrono::steady_clock::now();
    auto finish = [&]() {
      result.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - total_started).count();
      return result;
    };
    const auto scene = makeScene(box_center);
    const auto loaded = loadedScene(scene);

    const std::string initial_collision = collisionReason(scene, *initial_state_, &result.metrics);
    if (!initial_collision.empty()) {
      result.failure_stage = "initial_state";
      result.failure_reason = initial_collision;
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
      return finish();
    }

    moveit::core::RobotState grasp_start(*initial_state_);
    if (!alignHeight(box_center, scene, grasp_start, result)) {
      result.frames = {ReplayFrame{"initial_state", allJoints(*initial_state_), false}};
      return finish();
    }
    const auto lift_prefix = result.frames;
    moveit::core::RobotState return_goal(grasp_start);
    attachCarriedBox(return_goal);
    const std::string return_goal_collision = collisionReason(loaded, return_goal, &result.metrics);
    if (!return_goal_collision.empty()) {
      result.failure_stage = "return_goal";
      result.failure_reason = "return pose cannot carry box: " + return_goal_collision;
      if (result.frames.empty())
        result.frames.push_back(ReplayFrame{"initial_state", allJoints(grasp_start), false});
      return finish();
    }

    std::string precontact_rejections;
    auto precontact_candidates = solvePoseCandidates(
      precontactPose(box_center), grasp_start, false, scene, &result.metrics,
      false, &precontact_rejections);
    if (precontact_candidates.empty()) {
      result.failure_stage = "precontact_ik";
      result.failure_reason = precontact_rejections;
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
      const auto analytic_started = std::chrono::steady_clock::now();
      std::vector<moveit::core::RobotStatePtr> approach_states;
      std::vector<moveit::core::RobotStatePtr> retreat_states;
      std::string analytic_failure_stage;
      std::string analytic_failure_reason;
      const bool analytic_ok = traceCartesianPath(
        box_center, *precontact_candidates[candidate_index].state, scene,
        &result.metrics, &approach_states, &retreat_states,
        &analytic_failure_stage, &analytic_failure_reason);
      result.metrics.analytic_path_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - analytic_started).count();
      if (!analytic_ok) {
        last_failure_stage = analytic_failure_stage;
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          analytic_failure_reason;
        continue;
      }

      const auto approach_rrt = planRrt(
        scene, grasp_start, *precontact_candidates[candidate_index].state);
      result.metrics.rrt_approach_ms += approach_rrt.wall_ms;
      if (!approach_rrt.success) {
        last_failure_stage = "rrt_to_precontact";
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          approach_rrt.reason;
        continue;
      }

      std::vector<ReplayFrame> executable_prefix = lift_prefix;
      appendStates(approach_rrt.states, "rrt_to_precontact", false, false, &executable_prefix);
      appendStates(approach_states, "cartesian_approach", false, true, &executable_prefix);
      if (!retreat_states.empty()) {
        executable_prefix.push_back(
          ReplayFrame{"attach_box", allJoints(*retreat_states.front()), true});
      }
      appendStates(retreat_states, "cartesian_retreat", true, true, &executable_prefix);
      if (executable_prefix.size() > best_partial.size()) {
        best_partial = executable_prefix;
      }

      const auto return_rrt = planRrt(loaded, *retreat_states.back(), return_goal);
      result.metrics.rrt_return_ms += return_rrt.wall_ms;
      if (!return_rrt.success) {
        last_failure_stage = "rrt_return";
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          return_rrt.reason;
        continue;
      }

      result.frames = std::move(executable_prefix);
      appendStates(return_rrt.states, "rrt_return", true, true, &result.frames);
      result.success = true;
      return finish();
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
    if (distance_demo_) {
      output["distance_demo"] = true;
      output["height_alignment"] = heightAlignment(box_center);
      output["contact_numerical_gap"] = contact_numerical_gap_;
      output["requested_arm"] = requested_arm_;
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
    output["tool_to_box_center"] = {0.0, 0.0, toolToBoxCenter().z()};
    return output;
  }

  void publishJson(const nlohmann::json& payload)
  {
    std_msgs::msg::String message;
    message.data = payload.dump();
    task_publisher_->publish(message);
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
    const TaskResult& result)
  {
    nlohmann::json payload = sceneJson(box_center);
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
    };
    payload["joint_names"] = all_joint_names_;
    payload["attempts"] = attempts_;
    payload["verdict"] = result.success ? "path_found" : "no_path_found";
    payload["frames"] = nlohmann::json::array();
    for (const auto& frame : result.frames) {
      payload["frames"].push_back({
        {"stage", frame.stage},
        {"joints", frame.joints},
        {"box_attached", frame.box_attached},
      });
    }
    last_result_ = payload;
    publishJson(payload);
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
    visualization_msgs::msg::MarkerArray markers;
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
    if (distance_demo_ && display_box_attached_) {
      const auto& tool = display_state_->getGlobalLinkTransform(tool_link_);
      const Eigen::Vector3d position = tool * toolToBoxCenter();
      target.pose.position.x = position.x();
      target.pose.position.y = position.y();
      target.pose.position.z = position.z();
      const Eigen::Quaterniond q(tool.rotation());
      target.pose.orientation.x = q.x();
      target.pose.orientation.y = q.y();
      target.pose.orientation.z = q.z();
      target.pose.orientation.w = q.w();
      target.scale.x = box_height_;
      target.scale.z = box_depth_;
    }
    markers.markers.push_back(target);

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

    const auto neighbors = neighborCenters(center);
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
      point.pose.position.x = points[index].first.x();
      point.pose.position.y = points[index].first.y();
      point.pose.position.z = points[index].first.z();
      point.pose.orientation.w = 1.0;
      point.scale.x = point.scale.y = point.scale.z = 0.035;
      point.color = index == 0 ?
        color(0.15F, 0.55F, 1.0F, 0.9F) :
        (index == 1 ? color(0.2F, 1.0F, 0.2F, 0.9F) : color(0.95F, 0.2F, 0.95F, 0.9F));
      markers.markers.push_back(point);
    }
    if (distance_demo_) {
      for (int id = 0; id < 25; ++id) {
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
    scene_marker_publisher_->publish(markers);
  }

  void publishDisplayState()
  {
    std::lock_guard<std::mutex> lock(display_mutex_);
    if (!playback_frames_.empty()) {
      const auto& frame = playback_frames_[playback_index_];
      for (size_t index = 0; index < all_joint_names_.size() && index < frame.joints.size(); ++index) {
        display_state_->setVariablePosition(all_joint_names_[index], frame.joints[index]);
      }
      display_state_->update(true);
      display_box_attached_ = frame.box_attached;
      playback_index_ = distance_demo_ ? std::min(playback_index_ + 1U, playback_frames_.size() - 1U) :
        (playback_index_ + 1U) % playback_frames_.size();
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = all_joint_names_;
    message.position = allJoints(*display_state_);
    joint_state_publisher_->publish(message);
    if (distance_demo_) publishSceneMarkers();
  }

  bool display_box_attached_ = false;
  bool distance_demo_ = false;
  bool align_height_ = false;
  double shoulder_box_offset_ = 0.25;
  double initial_shoulder_z_ = 0.0;
  double chassis_front_x_ = 0.0;
  double wall_center_y_ = 0.0;
  double wall_bottom_z_ = 0.0;
  double wall_distance_ = 0.0;
  std::string requested_arm_ = "auto";
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
