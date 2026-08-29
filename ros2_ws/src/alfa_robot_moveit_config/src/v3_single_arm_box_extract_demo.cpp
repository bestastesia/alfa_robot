#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>
#include <alfa_robot_moveit_config/planning_diagnostics.hpp>

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
    box_depth_ = getParameter<double>("box_depth", 0.30);
    box_width_ = getParameter<double>("box_width", 0.40);
    box_height_ = getParameter<double>("box_height", 0.40);
    control_handle_clearance_ = std::max(
      0.10, getParameter<double>("control_handle_clearance", 0.25));
    control_handle_lateral_offset_ = std::max(
      0.50, getParameter<double>("control_handle_lateral_offset", 0.90));
    approach_distance_ = getParameter<double>("approach_distance", 0.05);
    retreat_distance_ = getParameter<double>("retreat_distance", 0.35);
    cartesian_step_ = getParameter<double>("cartesian_step", 0.01);
    collision_inset_ = getParameter<double>("collision_inset", 0.002);
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
    if (box_depth_ <= 0.0 || box_width_ <= 0.0 || box_height_ <= 0.0 ||
        approach_distance_ <= 0.0 || retreat_distance_ <= 0.0 || cartesian_step_ <= 0.0) {
      throw std::invalid_argument("box dimensions and Cartesian distances must be positive");
    }

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("failed to load robot model");
    }
    planning_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    if (!planning_group_ || planning_group_->getVariableCount() != 7U) {
      throw std::runtime_error("planning group must be a seven-axis arm: " + planning_group_name_);
    }
    if (!robot_model_->hasLinkModel(tool_link_) || !robot_model_->hasLinkModel(arm_base_link_)) {
      throw std::runtime_error("missing tool or arm base link");
    }

    all_joint_names_.reserve(14);
    for (const std::string arm_side : {std::string("left"), std::string("right")}) {
      for (int index = 1; index <= 7; ++index) {
        all_joint_names_.push_back(arm_side + "_joint" + std::to_string(index));
      }
    }

    initial_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    initial_state_->setToDefaultValues();
    for (const auto& name : all_joint_names_) {
      if (robot_model_->hasJointModel(name)) {
        initial_state_->setVariablePosition(name, 0.0);
      }
    }
    initial_state_->update(true);
    display_state_ = std::make_shared<moveit::core::RobotState>(*initial_state_);

    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side_ == "left" ? V3RedundantArmModel::V309Left : V3RedundantArmModel::V309Right);

    const std::vector<std::string> request_adapters = {
      "default_planner_request_adapters/AddTimeOptimalParameterization",
      "default_planner_request_adapters/ResolveConstraintFrames",
      "default_planner_request_adapters/FixWorkspaceBounds",
      "default_planner_request_adapters/FixStartStateBounds",
      "default_planner_request_adapters/FixStartStateCollision",
      "default_planner_request_adapters/FixStartStatePathConstraints",
    };
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

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "v3_single_arm_box_extract_demo_marker",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());
    createBoxMarker();
    confirm_menu_entry_ = menu_handler_.insert(
      "确认并计算当前箱位",
      [this](const Feedback::ConstSharedPtr&) {requestPlanning();});
    reset_menu_entry_ = menu_handler_.insert(
      "恢复默认箱位",
      [this](const Feedback::ConstSharedPtr&) {resetBoxPose();});
    (void)confirm_menu_entry_;
    (void)reset_menu_entry_;
    menu_handler_.apply(*marker_server_, kMarkerName);
    marker_server_->applyChanges();

    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds(25), [this]() {onWorkerTimer();});
    display_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {publishDisplayState();});
    publishPreview("拖动箱体XYZ；右键箱体并选择“确认并计算当前箱位”");
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
      box_center_ = Eigen::Vector3d(0.88, -0.20, 0.55);
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
    publishPlanningStarted(generation, box_center);
    publishStatus("CALCULATING", true);
    RCLCPP_INFO(
      get_logger(),
      "[%llu] calculation started: box_center=[%.3f, %.3f, %.3f]",
      static_cast<unsigned long long>(generation),
      box_center.x(), box_center.y(), box_center.z());

    TaskResult result;
    try {
      result = planTask(box_center);
    } catch (const std::exception& error) {
      result.success = false;
      result.failure_stage = "exception";
      result.failure_reason = error.what();
    }
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

  Eigen::Isometry3d contactPose(const Eigen::Vector3d& box_center) const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = box_center - Eigen::Vector3d(box_depth_ * 0.5, 0.0, 0.0);
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

  std::array<Eigen::Vector3d, 4> neighborCenters(const Eigen::Vector3d& center) const
  {
    return {
      center + Eigen::Vector3d(0.0, box_width_, 0.0),
      center - Eigen::Vector3d(0.0, box_width_, 0.0),
      center + Eigen::Vector3d(0.0, 0.0, box_height_),
      center - Eigen::Vector3d(0.0, 0.0, box_height_),
    };
  }

  planning_scene::PlanningScenePtr makeScene(const Eigen::Vector3d& box_center) const
  {
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    scene->setCurrentState(*initial_state_);
    const auto neighbors = neighborCenters(box_center);
    const double depth = std::max(0.001, box_depth_ - 2.0 * collision_inset_);
    const double width = std::max(0.001, box_width_ - 2.0 * collision_inset_);
    const double height = std::max(0.001, box_height_ - 2.0 * collision_inset_);
    for (size_t index = 0; index < neighbors.size(); ++index) {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = world_frame_;
      object.id = "neighbor_box_" + std::to_string(index);
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
    shape_pose.translation().z() = box_depth_ * 0.5;
    shape_poses.push_back(shape_pose);
    const std::string prefix = side_ + "_";
    state.attachBody(
      kCarriedBoxId,
      Eigen::Isometry3d::Identity(),
      shapes,
      shape_poses,
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
      scene, state, planning_group_);
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
    const double maximum_delta = maximumJointDelta(from_joints, to_joints);
    const size_t steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(maximum_delta / edge_joint_resolution_)));
    for (size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      std::array<double, 7> interpolated{};
      for (size_t index = 0; index < interpolated.size(); ++index) {
        interpolated[index] = from_joints[index] +
          normalizedAngle(to_joints[index] - from_joints[index]) * ratio;
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
    const size_t approach_steps = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(approach_distance_ / cartesian_step_)));
    for (size_t step = 1; step <= approach_steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(approach_steps);
      Eigen::Isometry3d target = contact;
      target.translation().x() -= approach_distance_ * (1.0 - ratio);
      std::string reason;
      auto next = solveNextPose(target, *current, false, scene, metrics, &reason);
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
    const std::string attach_collision = collisionReason(scene, contact_attached, metrics);
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
      auto next = solveNextPose(target, *current, true, scene, metrics, &reason);
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
      scene, start_state, planning_group_);
    if (!start_collision.empty()) {
      result.reason = "start_" + start_collision;
      return result;
    }
    const std::string goal_collision = alfa_robot::motion::scene_collision_reason(
      scene, goal_state, planning_group_);
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

    const std::string initial_collision = collisionReason(scene, *initial_state_, &result.metrics);
    if (!initial_collision.empty()) {
      result.failure_stage = "initial_state";
      result.failure_reason = initial_collision;
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
      return finish();
    }

    moveit::core::RobotState return_goal(*initial_state_);
    attachCarriedBox(return_goal);
    const std::string return_goal_collision = collisionReason(scene, return_goal, &result.metrics);
    if (!return_goal_collision.empty()) {
      result.failure_stage = "return_goal";
      result.failure_reason = "initial pose cannot carry box: " + return_goal_collision;
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
      return finish();
    }

    std::string precontact_rejections;
    auto precontact_candidates = solvePoseCandidates(
      precontactPose(box_center), *initial_state_, false, scene, &result.metrics,
      false, &precontact_rejections);
    if (precontact_candidates.empty()) {
      result.failure_stage = "precontact_ik";
      result.failure_reason = precontact_rejections;
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
      return finish();
    }
    if (precontact_candidates.size() > precontact_candidate_limit_) {
      precontact_candidates.resize(precontact_candidate_limit_);
    }

    std::string last_failure_stage = "candidate_search";
    std::string last_failure_reason = "no candidate attempted";
    std::vector<ReplayFrame> best_partial;
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
        scene, *initial_state_, *precontact_candidates[candidate_index].state);
      result.metrics.rrt_approach_ms += approach_rrt.wall_ms;
      if (!approach_rrt.success) {
        last_failure_stage = "rrt_to_precontact";
        last_failure_reason = "candidate " + std::to_string(candidate_index) + " " +
          approach_rrt.reason;
        continue;
      }

      std::vector<ReplayFrame> executable_prefix;
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

      const auto return_rrt = planRrt(scene, *retreat_states.back(), return_goal);
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
      result.frames.push_back(ReplayFrame{"initial_state", allJoints(*initial_state_), false});
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
    output["tool_to_box_center"] = {0.0, 0.0, box_depth_ * 0.5};
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
    payload["frames"] = nlohmann::json::array();
    for (const auto& frame : result.frames) {
      payload["frames"].push_back({
        {"stage", frame.stage},
        {"joints", frame.joints},
        {"box_attached", frame.box_attached},
      });
    }
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
    markers.markers.push_back(control_link);

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
      playback_index_ = (playback_index_ + 1U) % playback_frames_.size();
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = all_joint_names_;
    message.position = allJoints(*display_state_);
    joint_state_publisher_->publish(message);
  }

  std::string side_;
  std::string world_frame_;
  std::string arm_base_link_;
  std::string planning_group_name_;
  std::string tool_link_;
  Eigen::Vector3d box_center_{0.88, -0.20, 0.55};
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
