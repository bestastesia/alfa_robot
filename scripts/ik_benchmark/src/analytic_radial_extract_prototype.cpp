#include "alfa_robot_analytic_ik/analytic_ik.hpp"
#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"
#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <Eigen/Geometry>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using alfa_robot::analytic_ik::ArmAnalyticIkSolution;
using alfa_robot::analytic_ik::ArmSide;
using alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk;
using Json = nlohmann::json;

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kWorldToBaseZ = 0.202094;
constexpr double kTurnCenterX = 0.07134;
constexpr double kJoint2CenterZAboveUpdownValueInBase = 0.65;
constexpr double kAnalyticToolReach = 0.0825 + 0.243;
constexpr double kAnalyticParallelPlaneConstraint = 0.1265 + 0.058;
constexpr double kLoadedUpdown = 0.1;
constexpr std::array<double, 6> kLoadedArmDegrees{0.0, -90.0, 120.0, -75.0, 0.0, 0.0};

struct Options
{
  std::filesystem::path snapshot;
  std::filesystem::path output;
  int frames = 50;
  double target_radius = 0.79;
  double orientation_only_step_deg = 2.0;
  double shortcut_start_radial_rotation_deg = 30.0;
  double top_retreat_step_x = 0.01;
  int top_shortcut_start_step = 30;
  int candidate_index = 0;
  std::string path_mode = "radial";
  bool resume_radial_after_orientation = false;
  bool interleave_loaded_shortcuts = false;
};

double degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double lerp(double start, double goal, double ratio)
{
  return start + (goal - start) * ratio;
}

Eigen::Matrix3d rotY(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Isometry3d armFkInBase(
  const ThreeParallelArmAnalyticIk& solver,
  ArmSide side,
  double updown,
  const std::array<double, 6>& joints)
{
  return ThreeParallelArmAnalyticIk::baseLinkToArmBase(side, updown) *
         solver.forwardInArmBase(side, joints);
}

std::array<double, 6> jointsFromRecord(const Json& record, const std::string& side)
{
  const auto& joint_map = record.at("state").at("joint_map");
  std::array<double, 6> joints{};
  for (size_t index = 0; index < joints.size(); ++index) {
    joints[index] = joint_map.at(side + "_joint" + std::to_string(index + 1)).get<double>();
  }
  return joints;
}

double branchDistance(
  const std::array<double, 6>& lhs,
  const std::array<double, 6>& rhs)
{
  double sum = 0.0;
  for (size_t index = 0; index < lhs.size(); ++index) {
    const double delta = alfa_robot::analytic_ik::normalizeAngle(lhs[index] - rhs[index]);
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

std::optional<ArmAnalyticIkSolution> nearestSolution(
  const std::vector<ArmAnalyticIkSolution>& solutions,
  const std::array<double, 6>& previous)
{
  if (solutions.empty()) return std::nullopt;
  return *std::min_element(
    solutions.begin(), solutions.end(),
    [&](const auto& lhs, const auto& rhs) {
      return branchDistance(lhs.joints, previous) < branchDistance(rhs.joints, previous);
    });
}

geometry_msgs::msg::Pose identityPose(const std::array<double, 3>& center)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];
  pose.orientation.w = 1.0;
  return pose;
}

moveit_msgs::msg::CollisionObject collisionObjectFromJson(const Json& item)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = item.at("id").get<std::string>();
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  for (double dimension : item.at("size").get<std::vector<double>>()) {
    primitive.dimensions.push_back(dimension);
  }
  object.primitives.push_back(primitive);

  const auto center_values = item.at("center").get<std::vector<double>>();
  if (center_values.size() != 3) throw std::runtime_error("collision object center must have 3 values");
  std::array<double, 3> center{center_values[0], center_values[1], center_values[2]};
  auto pose = identityPose(center);
  const double yaw = item.value("yaw", 0.0);
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  object.primitive_poses.push_back(pose);
  return object;
}

moveit_msgs::msg::AttachedCollisionObject attachedObjectFromJson(const Json& item)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = item.at("link_name").get<std::string>();
  attached.touch_links = {attached.link_name};
  const std::string side = attached.link_name.rfind("left_", 0) == 0 ? "left" : "right";
  for (int joint = 3; joint <= 6; ++joint) {
    attached.touch_links.push_back(side + "_joint" + std::to_string(joint));
  }
  attached.object.header.frame_id = attached.link_name;
  attached.object.id = item.at("id").get<std::string>();
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  for (double dimension : item.at("size").get<std::vector<double>>()) {
    primitive.dimensions.push_back(dimension);
  }
  attached.object.primitives.push_back(primitive);

  const auto center_values = item.at("center_in_link").get<std::vector<double>>();
  if (center_values.size() != 3) throw std::runtime_error("attached box center must have 3 values");
  attached.object.primitive_poses.push_back(identityPose(
    {center_values[0], center_values[1], center_values[2]}));
  return attached;
}

std::vector<std::string> collisionContacts(
  const planning_scene::PlanningSceneConstPtr& scene,
  const moveit::core::RobotState& state)
{
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.group_name = "dual_arm_with_base";
  request.contacts = true;
  request.max_contacts = 100;
  request.max_contacts_per_pair = 4;
  scene->checkCollision(request, result, state, scene->getAllowedCollisionMatrix());
  std::vector<std::string> contacts;
  for (const auto& [pair, values] : result.contacts) {
    (void)values;
    contacts.push_back(pair.first + " <-> " + pair.second);
  }
  std::sort(contacts.begin(), contacts.end());
  contacts.erase(std::unique(contacts.begin(), contacts.end()), contacts.end());
  return contacts;
}

Json vectorJson(const Eigen::Vector3d& value)
{
  return Json::array({value.x(), value.y(), value.z()});
}

Json rotationJson(const Eigen::Matrix3d& rotation)
{
  const Eigen::Quaterniond quaternion(rotation);
  return Json::array({quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()});
}

Json jointsJson(const std::array<double, 6>& joints)
{
  return Json::array({joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]});
}

double variableDelta(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& name,
  double from,
  double to)
{
  return alfa_robot::motion::extract_transition_variable_delta(robot_model, name, from, to);
}

moveit::planning_interface::MoveGroupInterface::Plan makeInterpolatedPlan(
  const moveit::core::RobotState& start,
  const moveit::core::RobotState& goal,
  const std::vector<std::string>& joint_names,
  double duration_s)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto& trajectory = plan.trajectory_.joint_trajectory;
  trajectory.joint_names = joint_names;
  double step_count = 1.0;
  for (const auto& name : joint_names) {
    const double step = name == "updown" ? 0.01 : 5.0 * kPi / 180.0;
    step_count = std::max(
      step_count,
      std::abs(variableDelta(
        start.getRobotModel(), name,
        start.getVariablePosition(name), goal.getVariablePosition(name))) / step);
  }
  const size_t points = std::max<size_t>(2, static_cast<size_t>(std::ceil(step_count)) + 1);
  trajectory.points.reserve(points);
  for (size_t index = 0; index < points; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(points - 1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s * ratio);
    point.positions.reserve(joint_names.size());
    for (const auto& name : joint_names) {
      point.positions.push_back(
        start.getVariablePosition(name) +
        variableDelta(
          start.getRobotModel(), name,
          start.getVariablePosition(name), goal.getVariablePosition(name)) * ratio);
    }
    trajectory.points.push_back(std::move(point));
  }
  moveit::core::robotStateToRobotStateMsg(start, plan.start_state_, true);
  return plan;
}

moveit::core::RobotState stateFromTrajectoryPoint(
  const moveit::core::RobotState& start,
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  size_t point_index)
{
  moveit::core::RobotState state(start);
  const auto& point = trajectory.points.at(point_index);
  for (size_t index = 0;
       index < trajectory.joint_names.size() && index < point.positions.size(); ++index) {
    state.setVariablePosition(trajectory.joint_names[index], point.positions[index]);
  }
  state.update(true);
  return state;
}

void declareOmplParameters(const rclcpp::Node::SharedPtr& node)
{
  const auto declare_string = [&node](const std::string& name, const std::string& value) {
    if (!node->has_parameter(name)) node->declare_parameter<std::string>(name, value);
  };
  const auto declare_double = [&node](const std::string& name, double value) {
    if (!node->has_parameter(name)) node->declare_parameter<double>(name, value);
  };
  const auto declare_strings = [&node](
    const std::string& name, const std::vector<std::string>& value) {
    if (!node->has_parameter(name)) node->declare_parameter<std::vector<std::string>>(name, value);
  };
  declare_string("ompl.planner_configs.RRTConnectkConfigDefault.type", "geometric::RRTConnect");
  declare_double("ompl.planner_configs.RRTConnectkConfigDefault.range", 0.0);
  declare_string("ompl.dual_arm_with_base.default_planner_config", "RRTConnectkConfigDefault");
  declare_strings(
    "ompl.dual_arm_with_base.planner_configs",
    {"RRTConnectkConfigDefault"});
}

Json stateJson(const moveit::core::RobotState& state)
{
  Json joints = Json::object();
  joints["updown"] = state.getVariablePosition("updown");
  for (const auto& side : {std::string("left"), std::string("right")}) {
    for (size_t index = 1; index <= 6; ++index) {
      const std::string name = side + "_joint" + std::to_string(index);
      joints[name] = state.getVariablePosition(name);
    }
  }
  return joints;
}

Json trajectoryFrameJson(
  const moveit::core::RobotState& state,
  const ThreeParallelArmAnalyticIk& solver,
  const std::vector<std::string>& contacts,
  size_t index)
{
  std::array<double, 6> left{};
  std::array<double, 6> right{};
  for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
    left[joint_index] = state.getVariablePosition("left_joint" + std::to_string(joint_index + 1));
    right[joint_index] = state.getVariablePosition("right_joint" + std::to_string(joint_index + 1));
  }
  const double updown = state.getVariablePosition("updown");
  Eigen::Isometry3d left_world = armFkInBase(solver, ArmSide::Left, updown, left);
  Eigen::Isometry3d right_world = armFkInBase(solver, ArmSide::Right, updown, right);
  left_world.translation().z() += kWorldToBaseZ;
  right_world.translation().z() += kWorldToBaseZ;
  return Json{
    {"phase", "loaded_transition"},
    {"index", index},
    {"solved", true},
    {"collision_free", contacts.empty()},
    {"contacts", contacts},
    {"left_joints", jointsJson(left)},
    {"right_joints", jointsJson(right)},
    {"compensated_updown", updown},
    {"updown_compensation", 0.0},
    {"left_target_position_world", vectorJson(left_world.translation())},
    {"right_target_position_world", vectorJson(right_world.translation())},
    {"left_target_orientation_xyzw", rotationJson(left_world.linear())},
    {"right_target_orientation_xyzw", rotationJson(right_world.linear())},
  };
}

double q1FeasibilityMargin(
  ArmSide side,
  double updown,
  const Eigen::Isometry3d& target_world)
{
  Eigen::Isometry3d target_base = target_world;
  target_base.translation().z() -= kWorldToBaseZ;
  const Eigen::Isometry3d target_arm =
    ThreeParallelArmAnalyticIk::baseLinkToArmBase(side, updown).inverse() * target_base;
  const Eigen::Vector3d& position = target_arm.translation();
  const Eigen::Vector3d normal = target_arm.linear().col(2);
  const double coefficient_sin = -position.x() + kAnalyticToolReach * normal.x();
  const double coefficient_cos = position.y() - kAnalyticToolReach * normal.y();
  return std::hypot(coefficient_sin, coefficient_cos) - kAnalyticParallelPlaneConstraint;
}

Options parseOptions(int argc, char** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--snapshot" && index + 1 < argc) {
      options.snapshot = argv[++index];
    } else if (argument == "--output" && index + 1 < argc) {
      options.output = argv[++index];
    } else if (argument == "--frames" && index + 1 < argc) {
      options.frames = std::stoi(argv[++index]);
    } else if (argument == "--target-radius" && index + 1 < argc) {
      options.target_radius = std::stod(argv[++index]);
    } else if (argument == "--orientation-only-step-deg" && index + 1 < argc) {
      options.orientation_only_step_deg = std::stod(argv[++index]);
    } else if (argument == "--shortcut-start-radial-rotation-deg" && index + 1 < argc) {
      options.shortcut_start_radial_rotation_deg = std::stod(argv[++index]);
    } else if (argument == "--top-retreat-step-x" && index + 1 < argc) {
      options.top_retreat_step_x = std::stod(argv[++index]);
    } else if (argument == "--top-shortcut-start-step" && index + 1 < argc) {
      options.top_shortcut_start_step = std::stoi(argv[++index]);
    } else if (argument == "--candidate-index" && index + 1 < argc) {
      options.candidate_index = std::stoi(argv[++index]);
    } else if (argument == "--path-mode" && index + 1 < argc) {
      options.path_mode = argv[++index];
    } else if (argument == "--resume-radial-after-orientation" && index + 1 < argc) {
      options.resume_radial_after_orientation = std::string(argv[++index]) == "true";
    } else if (argument == "--interleave-loaded-shortcuts" && index + 1 < argc) {
      options.interleave_loaded_shortcuts = std::string(argv[++index]) == "true";
    }
  }
  if (options.snapshot.empty()) throw std::runtime_error("--snapshot is required");
  if (options.output.empty()) throw std::runtime_error("--output is required");
  if (options.frames < 2) throw std::runtime_error("--frames must be >= 2");
  if (!(options.target_radius > 0.0)) throw std::runtime_error("--target-radius must be positive");
  if (!(options.orientation_only_step_deg > 0.0)) {
    throw std::runtime_error("--orientation-only-step-deg must be positive");
  }
  if (options.shortcut_start_radial_rotation_deg < 0.0) {
    throw std::runtime_error("--shortcut-start-radial-rotation-deg must be non-negative");
  }
  if (!(options.top_retreat_step_x > 0.0)) {
    throw std::runtime_error("--top-retreat-step-x must be positive");
  }
  if (options.top_shortcut_start_step < 0) {
    throw std::runtime_error("--top-shortcut-start-step must be non-negative");
  }
  if (options.candidate_index < 0) {
    throw std::runtime_error("--candidate-index must be non-negative");
  }
  if (options.path_mode != "radial" && options.path_mode != "top_horizontal_retract") {
    throw std::runtime_error("--path-mode must be radial or top_horizontal_retract");
  }
  return options;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
      "analytic_radial_extract_prototype",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    std::ifstream input(options.snapshot);
    if (!input) throw std::runtime_error("failed to open snapshot: " + options.snapshot.string());
    Json snapshot;
    input >> snapshot;
    if (!snapshot.contains("records") || snapshot.at("records").empty()) {
      throw std::runtime_error("snapshot contains no IK records");
    }
    if (options.candidate_index >= static_cast<int>(snapshot.at("records").size())) {
      throw std::runtime_error("candidate index exceeds snapshot records");
    }
    const Json& selected = snapshot.at("records").at(options.candidate_index);
    const double updown = selected.at("h").get<double>();
    std::array<double, 6> left_previous = jointsFromRecord(selected, "left");
    std::array<double, 6> right_previous = jointsFromRecord(selected, "right");

    robot_model_loader::RobotModelLoader loader(node, "robot_description");
    const auto robot_model = loader.getModel();
    if (!robot_model) throw std::runtime_error("failed to load robot model");
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    for (const auto& panel : snapshot.value("container_panels", Json::array())) {
      scene->processCollisionObjectMsg(collisionObjectFromJson(panel));
    }
    const Json static_boxes = snapshot.value("static_box_obstacles", Json::object()).value("boxes", Json::array());
    for (const auto& obstacle : static_boxes) {
      scene->processCollisionObjectMsg(collisionObjectFromJson(obstacle));
    }
    auto pregrasp_scene = planning_scene::PlanningScene::clone(scene);
    for (const auto& box : snapshot.value("attached_boxes", Json::array())) {
      scene->processAttachedCollisionObjectMsg(attachedObjectFromJson(box));
    }
    std::vector<const moveit::core::AttachedBody*> moveit_attached_bodies;
    scene->getCurrentState().getAttachedBodies(moveit_attached_bodies);

    ThreeParallelArmAnalyticIk solver;
    const Eigen::Isometry3d left_start_base = armFkInBase(solver, ArmSide::Left, updown, left_previous);
    const Eigen::Isometry3d right_start_base = armFkInBase(solver, ArmSide::Right, updown, right_previous);
    Eigen::Isometry3d left_start_world = left_start_base;
    Eigen::Isometry3d right_start_world = right_start_base;
    left_start_world.translation().z() += kWorldToBaseZ;
    right_start_world.translation().z() += kWorldToBaseZ;

    const double center_z = kWorldToBaseZ + kJoint2CenterZAboveUpdownValueInBase + updown;
    const Eigen::Vector3d left_center(kTurnCenterX, left_start_world.translation().y(), center_z);
    const Eigen::Vector3d right_center(kTurnCenterX, right_start_world.translation().y(), center_z);

    const auto make_geometry = [&](const Eigen::Isometry3d& start, const Eigen::Vector3d& center) {
      const Eigen::Vector3d radial = start.translation() - center;
      const double radius = std::hypot(radial.x(), radial.z());
      const double radial_angle = std::atan2(radial.x(), radial.z());
      const Eigen::Vector3d normal = start.linear().col(2);
      const double tool_angle = std::atan2(normal.x(), normal.z());
      return std::array<double, 3>{radius, radial_angle, tool_angle};
    };
    const auto left_geometry = make_geometry(left_start_world, left_center);
    const auto right_geometry = make_geometry(right_start_world, right_center);
    const bool top_horizontal_retract = options.path_mode == "top_horizontal_retract";

    Eigen::Isometry3d left_pregrasp_world = left_start_world;
    Eigen::Isometry3d right_pregrasp_world = right_start_world;
    if (top_horizontal_retract) {
      left_pregrasp_world.translation().z() += 0.05;
      right_pregrasp_world.translation().z() += 0.05;
    } else {
      left_pregrasp_world.translation().x() -= 0.05;
      right_pregrasp_world.translation().x() -= 0.05;
    }
    Json pregrasp_transition{
      {"valid", true},
      {"failure_reason", ""},
      {"offset_m", 0.05},
      {"frames", Json::array()},
    };
    std::array<double, 6> pregrasp_left_previous = left_previous;
    std::array<double, 6> pregrasp_right_previous = right_previous;
    moveit::core::RobotState pregrasp_state(pregrasp_scene->getCurrentState());
    constexpr int kPregraspSteps = 5;
    for (int frame_index = 0; frame_index <= kPregraspSteps; ++frame_index) {
      const double ratio = static_cast<double>(frame_index) /
        static_cast<double>(kPregraspSteps);
      Eigen::Isometry3d left_target_world = left_pregrasp_world;
      Eigen::Isometry3d right_target_world = right_pregrasp_world;
      left_target_world.translation() =
        (1.0 - ratio) * left_pregrasp_world.translation() +
        ratio * left_start_world.translation();
      right_target_world.translation() =
        (1.0 - ratio) * right_pregrasp_world.translation() +
        ratio * right_start_world.translation();

      const auto solve_pregrasp_side = [&](ArmSide side,
                                           const Eigen::Isometry3d& target_world,
                                           const std::array<double, 6>& previous,
                                           std::array<double, 6>* joints) {
        Eigen::Isometry3d target_base = target_world;
        target_base.translation().z() -= kWorldToBaseZ;
        const auto solutions = solver.solveInBaseLink(
          side, target_base, updown, previous, 1e-5, 1e-5);
        const auto nearest = nearestSolution(solutions, previous);
        if (!nearest) return false;
        *joints = nearest->joints;
        return true;
      };

      std::array<double, 6> left_joints = pregrasp_left_previous;
      std::array<double, 6> right_joints = pregrasp_right_previous;
      std::string failure_reason;
      if (!solve_pregrasp_side(
            ArmSide::Left, left_target_world, pregrasp_left_previous, &left_joints)) {
        failure_reason = "left_no_analytic_solution";
      }
      if (!solve_pregrasp_side(
            ArmSide::Right, right_target_world, pregrasp_right_previous, &right_joints)) {
        if (!failure_reason.empty()) failure_reason += ";";
        failure_reason += "right_no_analytic_solution";
      }

      Json frame{
        {"phase", "pregrasp_to_ik"},
        {"index", frame_index},
        {"ratio", ratio},
        {"compensated_updown", updown},
        {"left_target_position_world", vectorJson(left_target_world.translation())},
        {"right_target_position_world", vectorJson(right_target_world.translation())},
        {"left_target_orientation_xyzw", rotationJson(left_target_world.linear())},
        {"right_target_orientation_xyzw", rotationJson(right_target_world.linear())},
        {"solved", failure_reason.empty()},
        {"solve_failure", failure_reason},
      };
      if (!failure_reason.empty()) {
        frame["collision_free"] = false;
        frame["bounds_ok"] = false;
        frame["contacts"] = Json::array();
        pregrasp_transition["valid"] = false;
        pregrasp_transition["failure_reason"] = failure_reason;
        pregrasp_transition["frames"].push_back(std::move(frame));
        break;
      }

      pregrasp_state.setVariablePosition("pitch", 0.0);
      pregrasp_state.setVariablePosition("turn", 0.0);
      pregrasp_state.setVariablePosition("updown", updown);
      for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
        pregrasp_state.setVariablePosition(
          "left_joint" + std::to_string(joint_index + 1), left_joints[joint_index]);
        pregrasp_state.setVariablePosition(
          "right_joint" + std::to_string(joint_index + 1), right_joints[joint_index]);
      }
      pregrasp_state.update(true);
      const bool bounds_ok = pregrasp_state.satisfiesBounds(
        robot_model->getJointModelGroup("dual_arm_with_base"));
      const auto contacts = collisionContacts(pregrasp_scene, pregrasp_state);
      frame["left_joints"] = jointsJson(left_joints);
      frame["right_joints"] = jointsJson(right_joints);
      frame["bounds_ok"] = bounds_ok;
      frame["collision_free"] = contacts.empty();
      frame["contacts"] = contacts;
      pregrasp_transition["frames"].push_back(std::move(frame));
      if (!bounds_ok || !contacts.empty()) {
        pregrasp_transition["valid"] = false;
        pregrasp_transition["failure_reason"] =
          !bounds_ok ? "joint_bounds" : contacts.front();
        break;
      }
      pregrasp_left_previous = left_joints;
      pregrasp_right_previous = right_joints;
    }

    const int path_frame_count = top_horizontal_retract
      ? static_cast<int>(std::ceil(std::max(
          std::abs(left_start_world.translation().x() - left_center.x()),
          std::abs(right_start_world.translation().x() - right_center.x())) /
          options.top_retreat_step_x)) + 1
      : options.frames;

    Json output;
    output["type"] = "analytic_radial_extract_prototype";
    output["snapshot"] = options.snapshot.string();
    output["frames_requested"] = path_frame_count;
    output["path_mode"] = options.path_mode;
    output["top_retreat_step_x"] = options.top_retreat_step_x;
    output["top_shortcut_start_step"] = options.top_shortcut_start_step;
    output["candidate_index"] = options.candidate_index;
    output["candidate_score"] = selected.value("score", 0.0);
    output["updown"] = updown;
    output["target_radius"] = options.target_radius;
    output["orientation_only_step_deg"] = options.orientation_only_step_deg;
    output["shortcut_start_radial_rotation_deg"] =
      options.shortcut_start_radial_rotation_deg;
    output["resume_radial_after_orientation"] = options.resume_radial_after_orientation;
    output["interleave_loaded_shortcuts"] = options.interleave_loaded_shortcuts;
    output["world_to_base_z"] = kWorldToBaseZ;
    output["turn_center_x"] = kTurnCenterX;
    output["joint2_center_z_above_updown_value_in_base"] = kJoint2CenterZAboveUpdownValueInBase;
    output["left_center_world"] = vectorJson(left_center);
    output["right_center_world"] = vectorJson(right_center);
    output["left_start_radius"] = left_geometry[0];
    output["right_start_radius"] = right_geometry[0];
    output["left_start_radial_angle_deg"] = degrees(left_geometry[1]);
    output["right_start_radial_angle_deg"] = degrees(right_geometry[1]);
    output["left_start_tool_angle_deg"] = degrees(left_geometry[2]);
    output["right_start_tool_angle_deg"] = degrees(right_geometry[2]);
    output["left_start_relative_angle_deg"] = degrees(left_geometry[2] - left_geometry[1]);
    output["right_start_relative_angle_deg"] = degrees(right_geometry[2] - right_geometry[1]);
    output["pregrasp_transition"] = std::move(pregrasp_transition);
    output["container_panels"] = snapshot.value("container_panels", Json::array());
    output["static_box_obstacles"] = snapshot.value("static_box_obstacles", Json::object());
    output["attached_boxes"] = snapshot.value("attached_boxes", Json::array());
    output["moveit_attached_body_count"] = moveit_attached_bodies.size();
    output["frames"] = Json::array();
    output["updown_compensation_rule"] =
      "after fixed-updown dual-arm IK, shift updown so the lower tool stays at the initial lower tool height, clamped to updown bounds";
    output["collision_check_order"] =
      "for every solved frame: merge both arm states, apply updown compensation, then immediately check bounds and the full planning scene";

    moveit::core::RobotState state(scene->getCurrentState());
    state.setVariablePosition("pitch", 0.0);
    state.setVariablePosition("turn", 0.0);
    bool path_valid = true;
    int first_failure = -1;
    std::string first_failure_reason;
    double max_joint_step = 0.0;
    double max_updown_compensation = 0.0;
    std::optional<moveit::core::RobotState> last_valid_state;
    int last_valid_frame = -1;
    std::vector<std::pair<int, moveit::core::RobotState>> radial_shortcut_states;
    const double initial_min_tool_z = std::min(
      left_start_world.translation().z(), right_start_world.translation().z());
    const auto strategy_start = std::chrono::steady_clock::now();
    auto solve_start = std::chrono::steady_clock::now();

    for (int frame_index = 0; frame_index < path_frame_count; ++frame_index) {
      const double ratio = path_frame_count > 1
        ? static_cast<double>(frame_index) / static_cast<double>(path_frame_count - 1)
        : 0.0;
      Json frame;
      frame["index"] = frame_index;
      frame["phase"] = "radial";
      frame["ratio"] = ratio;
      bool frame_solved = true;
      std::string solve_failure;

      std::array<double, 6> left_joints = left_previous;
      std::array<double, 6> right_joints = right_previous;
      Eigen::Isometry3d left_target_world = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d right_target_world = Eigen::Isometry3d::Identity();

      const auto solve_side = [&](ArmSide side,
                                  const Eigen::Isometry3d& start_world,
                                  const Eigen::Vector3d& center,
                                  const std::array<double, 3>& geometry,
                                  const std::array<double, 6>& previous,
                                  Eigen::Isometry3d* target_world,
                                  std::array<double, 6>* joints) -> bool {
        *target_world = start_world;
        if (top_horizontal_retract) {
          const double initial_x_offset = start_world.translation().x() - center.x();
          const double initial_x_distance = std::abs(initial_x_offset);
          const double retreat_distance = std::min(
            initial_x_distance,
            options.top_retreat_step_x * static_cast<double>(frame_index));
          const double upright_progress = initial_x_distance > 1e-12
            ? retreat_distance / initial_x_distance
            : 1.0;
          const double radial_angle = geometry[1] * (1.0 - upright_progress);
          const double remaining_x_offset = initial_x_offset * (1.0 - upright_progress);
          const double radial_radius = std::abs(std::sin(radial_angle)) > 1e-12
            ? std::abs(remaining_x_offset / std::sin(radial_angle))
            : (std::abs(geometry[1]) > 1e-12
                ? std::abs(initial_x_offset / geometry[1])
                : geometry[0]);
          target_world->translation() = center + Eigen::Vector3d(
            radial_radius * std::sin(radial_angle),
            0.0,
            radial_radius * std::cos(radial_angle));
          target_world->linear() = start_world.linear();
        } else {
          const double radius = lerp(geometry[0], options.target_radius, ratio);
          const double radial_angle = lerp(geometry[1], 0.0, ratio);
          const double relative_angle = lerp(geometry[2] - geometry[1], 0.0, ratio);
          const double tool_angle = radial_angle + relative_angle;
          target_world->translation() = center + Eigen::Vector3d(
            radius * std::sin(radial_angle),
            0.0,
            radius * std::cos(radial_angle));
          target_world->linear() = rotY(tool_angle - geometry[2]) * start_world.linear();
        }

        Eigen::Isometry3d target_base = *target_world;
        target_base.translation().z() -= kWorldToBaseZ;
        const auto solutions = solver.solveInBaseLink(
          side, target_base, updown, previous, 1e-5, 1e-5);
        const auto nearest = nearestSolution(solutions, previous);
        if (!nearest) return false;
        *joints = nearest->joints;
        return true;
      };

      if (!solve_side(
            ArmSide::Left, left_start_world, left_center, left_geometry,
            left_previous, &left_target_world, &left_joints)) {
        frame_solved = false;
        solve_failure = "left_no_analytic_solution";
      }
      if (!solve_side(
            ArmSide::Right, right_start_world, right_center, right_geometry,
            right_previous, &right_target_world, &right_joints)) {
        frame_solved = false;
        if (!solve_failure.empty()) solve_failure += ";";
        solve_failure += "right_no_analytic_solution";
      }

      frame["left_target_position_world"] = vectorJson(left_target_world.translation());
      frame["right_target_position_world"] = vectorJson(right_target_world.translation());
      frame["left_target_orientation_xyzw"] = rotationJson(left_target_world.linear());
      frame["right_target_orientation_xyzw"] = rotationJson(right_target_world.linear());
      const auto left_target_geometry = make_geometry(left_target_world, left_center);
      const auto right_target_geometry = make_geometry(right_target_world, right_center);
      frame["left_radius"] = left_target_geometry[0];
      frame["right_radius"] = right_target_geometry[0];
      frame["left_radial_angle_deg"] = degrees(left_target_geometry[1]);
      frame["right_radial_angle_deg"] = degrees(right_target_geometry[1]);
      const double left_radial_rotation_deg = std::abs(degrees(
        left_target_geometry[1] - left_geometry[1]));
      const double right_radial_rotation_deg = std::abs(degrees(
        right_target_geometry[1] - right_geometry[1]));
      const double synchronized_radial_rotation_deg = std::min(
        left_radial_rotation_deg, right_radial_rotation_deg);
      const bool shortcut_eligible = top_horizontal_retract
        ? frame_index >= options.top_shortcut_start_step
        : synchronized_radial_rotation_deg + 1e-9 >=
            options.shortcut_start_radial_rotation_deg;
      frame["left_radial_rotation_deg"] = left_radial_rotation_deg;
      frame["right_radial_rotation_deg"] = right_radial_rotation_deg;
      frame["synchronized_radial_rotation_deg"] = synchronized_radial_rotation_deg;
      frame["loaded_shortcut_eligible"] = shortcut_eligible;
      frame["left_relative_angle_deg"] = degrees(
        left_target_geometry[2] - left_target_geometry[1]);
      frame["right_relative_angle_deg"] = degrees(
        right_target_geometry[2] - right_target_geometry[1]);
      frame["left_q1_feasibility_margin_m"] =
        q1FeasibilityMargin(ArmSide::Left, updown, left_target_world);
      frame["right_q1_feasibility_margin_m"] =
        q1FeasibilityMargin(ArmSide::Right, updown, right_target_world);
      frame["solved"] = frame_solved;
      frame["solve_failure"] = solve_failure;
      frame["compensated_updown"] = updown;
      frame["updown_compensation"] = 0.0;

      if (frame_solved) {
        const double left_step = branchDistance(left_joints, left_previous);
        const double right_step = branchDistance(right_joints, right_previous);
        const double frame_step = std::max(left_step, right_step);
        max_joint_step = std::max(max_joint_step, frame_step);
        frame["left_joints"] = jointsJson(left_joints);
        frame["right_joints"] = jointsJson(right_joints);
        frame["max_joint_step_rad"] = frame_step;
        frame["max_joint_step_deg"] = degrees(frame_step);

        const Eigen::Isometry3d left_fixed_base =
          armFkInBase(solver, ArmSide::Left, updown, left_joints);
        const Eigen::Isometry3d right_fixed_base =
          armFkInBase(solver, ArmSide::Right, updown, right_joints);
        const double fixed_min_tool_z = kWorldToBaseZ + std::min(
          left_fixed_base.translation().z(), right_fixed_base.translation().z());
        const double required_compensation = initial_min_tool_z - fixed_min_tool_z;
        const double compensated_updown = std::clamp(updown + required_compensation, 0.0, 0.7);
        const double applied_compensation = compensated_updown - updown;
        max_updown_compensation = std::max(max_updown_compensation, std::abs(applied_compensation));
        frame["fixed_updown_min_tool_z_world"] = fixed_min_tool_z;
        frame["initial_min_tool_z_world"] = initial_min_tool_z;
        frame["requested_updown_compensation"] = required_compensation;
        frame["updown_compensation"] = applied_compensation;
        frame["compensated_updown"] = compensated_updown;
        frame["compensation_clamped"] = std::abs(applied_compensation - required_compensation) > 1e-12;
        frame["collision_check_uses_compensated_updown"] = true;

        state.setVariablePosition("pitch", 0.0);
        state.setVariablePosition("turn", 0.0);
        state.setVariablePosition("updown", compensated_updown);
        for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
          state.setVariablePosition("left_joint" + std::to_string(joint_index + 1), left_joints[joint_index]);
          state.setVariablePosition("right_joint" + std::to_string(joint_index + 1), right_joints[joint_index]);
        }
        state.update(true);
        const auto contacts = collisionContacts(scene, state);
        const bool bounds_ok = state.satisfiesBounds(robot_model->getJointModelGroup("dual_arm_with_base"));
        frame["bounds_ok"] = bounds_ok;
        frame["collision_free"] = contacts.empty();
        frame["contacts"] = contacts;
        if (!bounds_ok || !contacts.empty()) {
          path_valid = false;
          if (first_failure < 0) {
            first_failure = frame_index;
            first_failure_reason = !bounds_ok ? "joint_bounds" : contacts.front();
          }
        }
        if (path_valid && bounds_ok && contacts.empty()) {
          last_valid_state = state;
          last_valid_frame = frame_index;
          if (shortcut_eligible) {
            radial_shortcut_states.emplace_back(frame_index, state);
          }
        }
        left_previous = left_joints;
        right_previous = right_joints;
      } else {
        path_valid = false;
        if (first_failure < 0) {
          first_failure = frame_index;
          first_failure_reason = solve_failure;
        }
      }
      output["frames"].push_back(std::move(frame));
      if (top_horizontal_retract && first_failure >= 0) {
        break;
      }
    }

    const auto solve_end = std::chrono::steady_clock::now();
    output["solve_and_collision_ms"] =
      std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
    output["path_valid"] = path_valid;
    output["first_failure_frame"] = first_failure;
    output["first_failure_reason"] = first_failure_reason;
    output["max_joint_step_rad"] = max_joint_step;
    output["max_joint_step_deg"] = degrees(max_joint_step);
    output["max_updown_compensation"] = max_updown_compensation;

    Json orientation_only_transition{
      {"attempted", false},
      {"complete", false},
      {"progressed", false},
      {"failure_reason", top_horizontal_retract ? "path_only_mode" : "no_last_valid_radial_state"},
      {"start_radial_frame", last_valid_frame},
      {"step_deg", options.orientation_only_step_deg},
      {"steps_requested", 0},
      {"valid_steps", 0},
      {"frames", Json::array()},
    };
    std::optional<moveit::core::RobotState> orientation_end_state;
    std::vector<moveit::core::RobotState> orientation_valid_states;
    if (last_valid_state && !top_horizontal_retract) {
      orientation_only_transition["attempted"] = true;
      orientation_only_transition["failure_reason"] = "";
      orientation_only_transition["start_state"] = stateJson(*last_valid_state);
      const double orientation_updown = last_valid_state->getVariablePosition("updown");
      std::array<double, 6> orientation_left_previous{};
      std::array<double, 6> orientation_right_previous{};
      for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
        orientation_left_previous[joint_index] = last_valid_state->getVariablePosition(
          "left_joint" + std::to_string(joint_index + 1));
        orientation_right_previous[joint_index] = last_valid_state->getVariablePosition(
          "right_joint" + std::to_string(joint_index + 1));
      }

      Eigen::Isometry3d orientation_left_start = armFkInBase(
        solver, ArmSide::Left, orientation_updown, orientation_left_previous);
      Eigen::Isometry3d orientation_right_start = armFkInBase(
        solver, ArmSide::Right, orientation_updown, orientation_right_previous);
      orientation_left_start.translation().z() += kWorldToBaseZ;
      orientation_right_start.translation().z() += kWorldToBaseZ;
      const double orientation_center_z =
        kWorldToBaseZ + kJoint2CenterZAboveUpdownValueInBase + orientation_updown;
      const Eigen::Vector3d orientation_left_center(
        kTurnCenterX, orientation_left_start.translation().y(), orientation_center_z);
      const Eigen::Vector3d orientation_right_center(
        kTurnCenterX, orientation_right_start.translation().y(), orientation_center_z);
      const auto orientation_geometry = [&](const Eigen::Isometry3d& pose,
                                            const Eigen::Vector3d& center) {
        const Eigen::Vector3d radial = pose.translation() - center;
        const double radius = std::hypot(radial.x(), radial.z());
        const double radial_angle = std::atan2(radial.x(), radial.z());
        const Eigen::Vector3d normal = pose.linear().col(2);
        const double tool_angle = std::atan2(normal.x(), normal.z());
        const double relative_angle = alfa_robot::analytic_ik::normalizeAngle(
          tool_angle - radial_angle);
        return std::array<double, 3>{radius, radial_angle, relative_angle};
      };
      const auto orientation_left_geometry = orientation_geometry(
        orientation_left_start, orientation_left_center);
      const auto orientation_right_geometry = orientation_geometry(
        orientation_right_start, orientation_right_center);
      const double maximum_relative_angle = std::max(
        std::abs(orientation_left_geometry[2]), std::abs(orientation_right_geometry[2]));
      const int orientation_steps = std::max(
        0, static_cast<int>(std::ceil(
          maximum_relative_angle / (options.orientation_only_step_deg * kPi / 180.0))));
      orientation_only_transition["steps_requested"] = orientation_steps;
      orientation_only_transition["left_frozen_radius"] = orientation_left_geometry[0];
      orientation_only_transition["right_frozen_radius"] = orientation_right_geometry[0];
      orientation_only_transition["left_frozen_radial_angle_deg"] =
        degrees(orientation_left_geometry[1]);
      orientation_only_transition["right_frozen_radial_angle_deg"] =
        degrees(orientation_right_geometry[1]);
      orientation_only_transition["left_start_relative_angle_deg"] =
        degrees(orientation_left_geometry[2]);
      orientation_only_transition["right_start_relative_angle_deg"] =
        degrees(orientation_right_geometry[2]);

      moveit::core::RobotState orientation_state(*last_valid_state);
      orientation_end_state = *last_valid_state;
      int valid_orientation_steps = 0;
      for (int step_index = 1; step_index <= orientation_steps; ++step_index) {
        const double ratio = static_cast<double>(step_index) /
          static_cast<double>(orientation_steps);
        Json frame;
        frame["phase"] = "orientation_only";
        frame["index"] = step_index - 1;
        frame["ratio"] = ratio;
        frame["compensated_updown"] = orientation_updown;
        frame["updown_compensation"] = 0.0;
        frame["left_radius"] = orientation_left_geometry[0];
        frame["right_radius"] = orientation_right_geometry[0];
        frame["left_center_world"] = vectorJson(orientation_left_center);
        frame["right_center_world"] = vectorJson(orientation_right_center);
        frame["left_radial_angle_deg"] = degrees(orientation_left_geometry[1]);
        frame["right_radial_angle_deg"] = degrees(orientation_right_geometry[1]);
        frame["left_relative_angle_deg"] = degrees(
          orientation_left_geometry[2] * (1.0 - ratio));
        frame["right_relative_angle_deg"] = degrees(
          orientation_right_geometry[2] * (1.0 - ratio));

        Eigen::Isometry3d left_target_world = orientation_left_start;
        Eigen::Isometry3d right_target_world = orientation_right_start;
        left_target_world.linear() = rotY(-orientation_left_geometry[2] * ratio) *
          orientation_left_start.linear();
        right_target_world.linear() = rotY(-orientation_right_geometry[2] * ratio) *
          orientation_right_start.linear();
        frame["left_target_position_world"] = vectorJson(left_target_world.translation());
        frame["right_target_position_world"] = vectorJson(right_target_world.translation());
        frame["left_target_orientation_xyzw"] = rotationJson(left_target_world.linear());
        frame["right_target_orientation_xyzw"] = rotationJson(right_target_world.linear());
        frame["left_q1_feasibility_margin_m"] =
          q1FeasibilityMargin(ArmSide::Left, orientation_updown, left_target_world);
        frame["right_q1_feasibility_margin_m"] =
          q1FeasibilityMargin(ArmSide::Right, orientation_updown, right_target_world);

        const auto solve_orientation_side = [&](ArmSide side,
                                                const Eigen::Isometry3d& target_world,
                                                const std::array<double, 6>& previous,
                                                std::array<double, 6>* joints) {
          Eigen::Isometry3d target_base = target_world;
          target_base.translation().z() -= kWorldToBaseZ;
          const auto solutions = solver.solveInBaseLink(
            side, target_base, orientation_updown, previous, 1e-5, 1e-5);
          const auto nearest = nearestSolution(solutions, previous);
          if (!nearest) return false;
          *joints = nearest->joints;
          return true;
        };
        std::array<double, 6> left_joints = orientation_left_previous;
        std::array<double, 6> right_joints = orientation_right_previous;
        std::string failure_reason;
        if (!solve_orientation_side(
              ArmSide::Left, left_target_world, orientation_left_previous, &left_joints)) {
          failure_reason = "left_no_analytic_solution";
        }
        if (!solve_orientation_side(
              ArmSide::Right, right_target_world, orientation_right_previous, &right_joints)) {
          if (!failure_reason.empty()) failure_reason += ";";
          failure_reason += "right_no_analytic_solution";
        }
        frame["solved"] = failure_reason.empty();
        frame["solve_failure"] = failure_reason;
        if (!failure_reason.empty()) {
          frame["bounds_ok"] = false;
          frame["collision_free"] = false;
          frame["contacts"] = Json::array();
          orientation_only_transition["failure_reason"] = failure_reason;
          orientation_only_transition["frames"].push_back(std::move(frame));
          break;
        }

        frame["left_joints"] = jointsJson(left_joints);
        frame["right_joints"] = jointsJson(right_joints);
        const double frame_step = std::max(
          branchDistance(left_joints, orientation_left_previous),
          branchDistance(right_joints, orientation_right_previous));
        frame["max_joint_step_rad"] = frame_step;
        frame["max_joint_step_deg"] = degrees(frame_step);
        orientation_state.setVariablePosition("pitch", 0.0);
        orientation_state.setVariablePosition("turn", 0.0);
        orientation_state.setVariablePosition("updown", orientation_updown);
        for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
          orientation_state.setVariablePosition(
            "left_joint" + std::to_string(joint_index + 1), left_joints[joint_index]);
          orientation_state.setVariablePosition(
            "right_joint" + std::to_string(joint_index + 1), right_joints[joint_index]);
        }
        orientation_state.update(true);
        const bool bounds_ok = orientation_state.satisfiesBounds(
          robot_model->getJointModelGroup("dual_arm_with_base"));
        const auto contacts = collisionContacts(scene, orientation_state);
        frame["bounds_ok"] = bounds_ok;
        frame["collision_free"] = contacts.empty();
        frame["contacts"] = contacts;
        orientation_only_transition["frames"].push_back(std::move(frame));
        if (!bounds_ok || !contacts.empty()) {
          orientation_only_transition["failure_reason"] =
            !bounds_ok ? "joint_bounds" : contacts.front();
          break;
        }
        ++valid_orientation_steps;
        orientation_left_previous = left_joints;
        orientation_right_previous = right_joints;
        orientation_end_state = orientation_state;
        orientation_valid_states.push_back(orientation_state);
      }
      orientation_only_transition["valid_steps"] = valid_orientation_steps;
      orientation_only_transition["progressed"] = valid_orientation_steps > 0;
      orientation_only_transition["complete"] = valid_orientation_steps == orientation_steps;
      if (valid_orientation_steps == orientation_steps) {
        orientation_only_transition["failure_reason"] = "";
      }
      if (orientation_end_state) {
        orientation_only_transition["end_state"] = stateJson(*orientation_end_state);
      }
    }
    output["orientation_only_transition"] = orientation_only_transition;

    Json radial_resume_transition{
      {"attempted", false},
      {"complete", false},
      {"progressed", false},
      {"failure_reason", "disabled"},
      {"start_radial_frame", last_valid_frame},
      {"steps_requested", 0},
      {"valid_steps", 0},
      {"frames", Json::array()},
    };
    std::optional<moveit::core::RobotState> radial_resume_end_state;
    std::vector<moveit::core::RobotState> radial_resume_valid_states;
    if (options.resume_radial_after_orientation) {
      radial_resume_transition["failure_reason"] = "orientation_not_complete";
    }
    if (options.resume_radial_after_orientation &&
        orientation_only_transition.value("complete", false) && orientation_end_state) {
      radial_resume_transition["attempted"] = true;
      radial_resume_transition["failure_reason"] = "";
      radial_resume_transition["start_state"] = stateJson(*orientation_end_state);
      const double resume_reference_updown =
        orientation_end_state->getVariablePosition("updown");
      std::array<double, 6> resume_left_previous{};
      std::array<double, 6> resume_right_previous{};
      for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
        resume_left_previous[joint_index] = orientation_end_state->getVariablePosition(
          "left_joint" + std::to_string(joint_index + 1));
        resume_right_previous[joint_index] = orientation_end_state->getVariablePosition(
          "right_joint" + std::to_string(joint_index + 1));
      }

      Eigen::Isometry3d resume_left_start = armFkInBase(
        solver, ArmSide::Left, resume_reference_updown, resume_left_previous);
      Eigen::Isometry3d resume_right_start = armFkInBase(
        solver, ArmSide::Right, resume_reference_updown, resume_right_previous);
      resume_left_start.translation().z() += kWorldToBaseZ;
      resume_right_start.translation().z() += kWorldToBaseZ;
      const double resume_center_z =
        kWorldToBaseZ + kJoint2CenterZAboveUpdownValueInBase + resume_reference_updown;
      const Eigen::Vector3d resume_left_center(
        kTurnCenterX, resume_left_start.translation().y(), resume_center_z);
      const Eigen::Vector3d resume_right_center(
        kTurnCenterX, resume_right_start.translation().y(), resume_center_z);
      const auto resume_geometry = [](const Eigen::Isometry3d& pose,
                                      const Eigen::Vector3d& center) {
        const Eigen::Vector3d radial = pose.translation() - center;
        return std::array<double, 2>{
          std::hypot(radial.x(), radial.z()), std::atan2(radial.x(), radial.z())};
      };
      const auto resume_left_geometry = resume_geometry(resume_left_start, resume_left_center);
      const auto resume_right_geometry = resume_geometry(resume_right_start, resume_right_center);
      const int resume_steps = std::max(0, options.frames - 1 - last_valid_frame);
      radial_resume_transition["steps_requested"] = resume_steps;
      radial_resume_transition["reference_updown"] = resume_reference_updown;
      radial_resume_transition["target_radius"] = options.target_radius;
      radial_resume_transition["target_radial_angle_deg"] = 0.0;
      radial_resume_transition["left_start_radius"] = resume_left_geometry[0];
      radial_resume_transition["right_start_radius"] = resume_right_geometry[0];
      radial_resume_transition["left_start_radial_angle_deg"] =
        degrees(resume_left_geometry[1]);
      radial_resume_transition["right_start_radial_angle_deg"] =
        degrees(resume_right_geometry[1]);

      moveit::core::RobotState resume_state(*orientation_end_state);
      radial_resume_end_state = *orientation_end_state;
      int valid_resume_steps = 0;
      for (int step_index = 1; step_index <= resume_steps; ++step_index) {
        const double ratio = static_cast<double>(step_index) /
          static_cast<double>(resume_steps);
        const double left_radius = lerp(
          resume_left_geometry[0], options.target_radius, ratio);
        const double right_radius = lerp(
          resume_right_geometry[0], options.target_radius, ratio);
        const double left_radial_angle = lerp(resume_left_geometry[1], 0.0, ratio);
        const double right_radial_angle = lerp(resume_right_geometry[1], 0.0, ratio);
        Eigen::Isometry3d left_target_world = resume_left_start;
        Eigen::Isometry3d right_target_world = resume_right_start;
        left_target_world.translation() = resume_left_center + Eigen::Vector3d(
          left_radius * std::sin(left_radial_angle),
          0.0,
          left_radius * std::cos(left_radial_angle));
        right_target_world.translation() = resume_right_center + Eigen::Vector3d(
          right_radius * std::sin(right_radial_angle),
          0.0,
          right_radius * std::cos(right_radial_angle));
        left_target_world.linear() =
          rotY(left_radial_angle - resume_left_geometry[1]) * resume_left_start.linear();
        right_target_world.linear() =
          rotY(right_radial_angle - resume_right_geometry[1]) * resume_right_start.linear();

        Json frame;
        frame["phase"] = "radial_resume";
        frame["index"] = step_index - 1;
        frame["ratio"] = ratio;
        frame["left_radius"] = left_radius;
        frame["right_radius"] = right_radius;
        frame["left_center_world"] = vectorJson(resume_left_center);
        frame["right_center_world"] = vectorJson(resume_right_center);
        frame["left_radial_angle_deg"] = degrees(left_radial_angle);
        frame["right_radial_angle_deg"] = degrees(right_radial_angle);
        frame["left_relative_angle_deg"] = 0.0;
        frame["right_relative_angle_deg"] = 0.0;
        frame["left_target_position_world"] = vectorJson(left_target_world.translation());
        frame["right_target_position_world"] = vectorJson(right_target_world.translation());
        frame["left_target_orientation_xyzw"] = rotationJson(left_target_world.linear());
        frame["right_target_orientation_xyzw"] = rotationJson(right_target_world.linear());
        frame["left_q1_feasibility_margin_m"] =
          q1FeasibilityMargin(ArmSide::Left, resume_reference_updown, left_target_world);
        frame["right_q1_feasibility_margin_m"] =
          q1FeasibilityMargin(ArmSide::Right, resume_reference_updown, right_target_world);
        frame["compensated_updown"] = resume_reference_updown;
        frame["updown_compensation"] = 0.0;

        const auto solve_resume_side = [&](ArmSide side,
                                           const Eigen::Isometry3d& target_world,
                                           const std::array<double, 6>& previous,
                                           std::array<double, 6>* joints) {
          Eigen::Isometry3d target_base = target_world;
          target_base.translation().z() -= kWorldToBaseZ;
          const auto solutions = solver.solveInBaseLink(
            side, target_base, resume_reference_updown, previous, 1e-5, 1e-5);
          const auto nearest = nearestSolution(solutions, previous);
          if (!nearest) return false;
          *joints = nearest->joints;
          return true;
        };
        std::array<double, 6> left_joints = resume_left_previous;
        std::array<double, 6> right_joints = resume_right_previous;
        std::string failure_reason;
        if (!solve_resume_side(
              ArmSide::Left, left_target_world, resume_left_previous, &left_joints)) {
          failure_reason = "left_no_analytic_solution";
        }
        if (!solve_resume_side(
              ArmSide::Right, right_target_world, resume_right_previous, &right_joints)) {
          if (!failure_reason.empty()) failure_reason += ";";
          failure_reason += "right_no_analytic_solution";
        }
        frame["solved"] = failure_reason.empty();
        frame["solve_failure"] = failure_reason;
        if (!failure_reason.empty()) {
          frame["bounds_ok"] = false;
          frame["collision_free"] = false;
          frame["contacts"] = Json::array();
          radial_resume_transition["failure_reason"] = failure_reason;
          radial_resume_transition["frames"].push_back(std::move(frame));
          break;
        }

        frame["left_joints"] = jointsJson(left_joints);
        frame["right_joints"] = jointsJson(right_joints);
        const double frame_step = std::max(
          branchDistance(left_joints, resume_left_previous),
          branchDistance(right_joints, resume_right_previous));
        frame["max_joint_step_rad"] = frame_step;
        frame["max_joint_step_deg"] = degrees(frame_step);
        const Eigen::Isometry3d left_fixed_base = armFkInBase(
          solver, ArmSide::Left, resume_reference_updown, left_joints);
        const Eigen::Isometry3d right_fixed_base = armFkInBase(
          solver, ArmSide::Right, resume_reference_updown, right_joints);
        const double fixed_min_tool_z = kWorldToBaseZ + std::min(
          left_fixed_base.translation().z(), right_fixed_base.translation().z());
        const double required_compensation = initial_min_tool_z - fixed_min_tool_z;
        const double compensated_updown = std::clamp(
          resume_reference_updown + required_compensation, 0.0, 0.7);
        const double applied_compensation = compensated_updown - resume_reference_updown;
        frame["fixed_updown_min_tool_z_world"] = fixed_min_tool_z;
        frame["initial_min_tool_z_world"] = initial_min_tool_z;
        frame["requested_updown_compensation"] = required_compensation;
        frame["updown_compensation"] = applied_compensation;
        frame["compensated_updown"] = compensated_updown;
        frame["compensation_clamped"] =
          std::abs(applied_compensation - required_compensation) > 1e-12;
        frame["collision_check_uses_compensated_updown"] = true;

        resume_state.setVariablePosition("pitch", 0.0);
        resume_state.setVariablePosition("turn", 0.0);
        resume_state.setVariablePosition("updown", compensated_updown);
        for (size_t joint_index = 0; joint_index < 6; ++joint_index) {
          resume_state.setVariablePosition(
            "left_joint" + std::to_string(joint_index + 1), left_joints[joint_index]);
          resume_state.setVariablePosition(
            "right_joint" + std::to_string(joint_index + 1), right_joints[joint_index]);
        }
        resume_state.update(true);
        const bool bounds_ok = resume_state.satisfiesBounds(
          robot_model->getJointModelGroup("dual_arm_with_base"));
        const auto contacts = collisionContacts(scene, resume_state);
        frame["bounds_ok"] = bounds_ok;
        frame["collision_free"] = contacts.empty();
        frame["contacts"] = contacts;
        radial_resume_transition["frames"].push_back(std::move(frame));
        if (!bounds_ok || !contacts.empty()) {
          radial_resume_transition["failure_reason"] =
            !bounds_ok ? "joint_bounds" : contacts.front();
          break;
        }
        ++valid_resume_steps;
        resume_left_previous = left_joints;
        resume_right_previous = right_joints;
        radial_resume_end_state = resume_state;
        radial_resume_valid_states.push_back(resume_state);
      }
      radial_resume_transition["valid_steps"] = valid_resume_steps;
      radial_resume_transition["progressed"] = valid_resume_steps > 0;
      radial_resume_transition["complete"] = valid_resume_steps == resume_steps;
      if (valid_resume_steps == resume_steps) {
        radial_resume_transition["failure_reason"] = "";
      }
      if (radial_resume_end_state) {
        radial_resume_transition["end_state"] = stateJson(*radial_resume_end_state);
      }
    }
    output["radial_resume_transition"] = radial_resume_transition;

    Json loaded_transition{
      {"attempted", false},
      {"valid", false},
      {"method", ""},
      {"failure_reason", top_horizontal_retract ? "path_only_mode" : "no_last_valid_radial_state"},
      {"start_radial_frame", last_valid_frame},
      {"goal_updown", kLoadedUpdown},
      {"goal_arm_degrees", kLoadedArmDegrees},
      {"frames", Json::array()},
    };
    if (last_valid_state && (!top_horizontal_retract || options.interleave_loaded_shortcuts)) {
      loaded_transition["attempted"] = true;
      loaded_transition["start_state"] = stateJson(*last_valid_state);
      std::vector<const moveit::core::AttachedBody*> start_attached_bodies;
      last_valid_state->getAttachedBodies(start_attached_bodies);
      loaded_transition["start_attached_body_count"] = start_attached_bodies.size();
      const auto* loaded_group = robot_model->getJointModelGroup("dual_arm_with_base");
      if (!loaded_group) throw std::runtime_error("missing dual_arm_with_base group");
      moveit::core::RobotState goal_state(*last_valid_state);
      goal_state.setVariablePosition("updown", kLoadedUpdown);
      for (const auto& side : {std::string("left"), std::string("right")}) {
        for (size_t index = 0; index < kLoadedArmDegrees.size(); ++index) {
          goal_state.setVariablePosition(
            side + "_joint" + std::to_string(index + 1),
          kLoadedArmDegrees[index] * kPi / 180.0);
        }
      }
      goal_state.enforceBounds(loaded_group);
      goal_state.update(true);
      loaded_transition["goal_state"] = stateJson(goal_state);

      const std::vector<std::string> joint_names = loaded_group->getVariableNames();
      declareOmplParameters(node);
      const std::vector<std::string> request_adapters = {
        "default_planner_request_adapters/AddTimeOptimalParameterization",
        "default_planner_request_adapters/ResolveConstraintFrames",
        "default_planner_request_adapters/FixWorkspaceBounds",
        "default_planner_request_adapters/FixStartStateBounds",
        "default_planner_request_adapters/FixStartStateCollision",
        "default_planner_request_adapters/FixStartStatePathConstraints",
      };
      auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
        robot_model, node, "ompl", "ompl_interface/OMPLPlanner", request_adapters);
      planning_pipeline->displayComputedMotionPlans(false);
      planning_pipeline->publishReceivedRequests(false);
      planning_pipeline->checkSolutionPaths(true);

      alfa_robot::motion::ExtractMonitorTransitionPlanner transition_planner;
      transition_planner.make_interpolated_plan =
        [&joint_names](const auto& start, const auto& goal, double duration_s) {
          return makeInterpolatedPlan(start, goal, joint_names, duration_s);
        };
      transition_planner.densify_plan = [](const auto& plan) { return plan; };
      transition_planner.validate_plan =
        [&scene, &loaded_group](const auto& plan, const auto& start, std::string* reason) {
          const auto& trajectory = plan.trajectory_.joint_trajectory;
          for (size_t index = 0; index < trajectory.points.size(); ++index) {
            const auto current = stateFromTrajectoryPoint(start, trajectory, index);
            if (!current.satisfiesBounds(loaded_group)) {
              if (reason) *reason = "trajectory_point_" + std::to_string(index) + ":joint_bounds";
              return false;
            }
            const auto contacts = collisionContacts(scene, current);
            if (!contacts.empty()) {
              if (reason) {
                *reason = "trajectory_point_" + std::to_string(index) + ":" + contacts.front();
              }
              return false;
            }
          }
          if (reason) reason->clear();
          return true;
        };
      transition_planner.direct_plan =
        [&scene, &planning_pipeline, &loaded_group](
          const auto& start, const auto& goal, auto* plan, std::string* reason) {
          auto planning_scene = planning_scene::PlanningScene::clone(scene);
          planning_scene->setCurrentState(start);
          planning_interface::MotionPlanRequest request;
          request.group_name = "dual_arm_with_base";
          request.allowed_planning_time = 1.0;
          request.num_planning_attempts = 8;
          request.max_velocity_scaling_factor = 1.0;
          request.max_acceleration_scaling_factor = 1.0;
          moveit::core::robotStateToRobotStateMsg(start, request.start_state, true);
          request.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(goal, loaded_group, 1e-4));
          planning_interface::MotionPlanResponse response;
          const bool generated = planning_pipeline->generatePlan(planning_scene, request, response);
          if (!generated || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
              !response.trajectory_) {
            if (reason) {
              *reason = "moveit_rrtconnect_failed_code_" +
                std::to_string(response.error_code_.val);
            }
            return false;
          }
          moveit::core::robotStateToRobotStateMsg(start, plan->start_state_, true);
          response.trajectory_->getRobotTrajectoryMsg(plan->trajectory_);
          plan->planning_time_ = response.planning_time_;
          return !plan->trajectory_.joint_trajectory.points.empty();
        };

      loaded_transition["attempts"] = Json::array();
      loaded_transition["shortcut_attempts"] = Json::array();
      std::vector<std::pair<std::string, moveit::core::RobotState>> start_candidates;
      bool orientation_shortcut_precheck_valid = false;
      if (!options.resume_radial_after_orientation &&
          orientation_only_transition.value("progressed", false) && orientation_end_state) {
        const auto orientation_shortcut = alfa_robot::motion::retime_plan_by_max_joint_speed(
          makeInterpolatedPlan(*orientation_end_state, goal_state, joint_names, 1.0),
          robot_model,
          20.0 * kPi / 180.0);
        std::string orientation_shortcut_reason;
        orientation_shortcut_precheck_valid = transition_planner.validate_plan(
          orientation_shortcut, *orientation_end_state, &orientation_shortcut_reason);
        orientation_only_transition["loaded_shortcut_precheck_valid"] =
          orientation_shortcut_precheck_valid;
        orientation_only_transition["loaded_shortcut_precheck_reason"] =
          orientation_shortcut_reason;
      }
      if (options.interleave_loaded_shortcuts) {
        struct ShortcutCheckpoint
        {
          std::string phase;
          int phase_step;
          moveit::core::RobotState state;
        };
        std::vector<ShortcutCheckpoint> checkpoints;
        for (const auto& [frame_index, radial_state] : radial_shortcut_states) {
          checkpoints.push_back(ShortcutCheckpoint{
            top_horizontal_retract ? "top_upright_checkpoint" : "radial_checkpoint",
            top_horizontal_retract ? frame_index : frame_index + 1,
            radial_state});
        }
        if (!top_horizontal_retract && (radial_shortcut_states.empty() ||
            radial_shortcut_states.back().first != last_valid_frame)) {
          checkpoints.push_back(ShortcutCheckpoint{
            "radial_checkpoint", last_valid_frame + 1, *last_valid_state});
        }
        for (size_t index = 0; index < orientation_valid_states.size(); ++index) {
          checkpoints.push_back(ShortcutCheckpoint{
            "orientation_only", static_cast<int>(index + 1), orientation_valid_states[index]});
        }
        for (size_t index = 0; index < radial_resume_valid_states.size(); ++index) {
          checkpoints.push_back(ShortcutCheckpoint{
            "radial_resume", static_cast<int>(index + 1), radial_resume_valid_states[index]});
        }

        bool shortcut_selected = false;
        double shortcut_check_ms = 0.0;
        for (size_t checkpoint_index = 0;
             checkpoint_index < checkpoints.size(); ++checkpoint_index) {
          const auto& checkpoint = checkpoints[checkpoint_index];
          const auto shortcut = alfa_robot::motion::retime_plan_by_max_joint_speed(
            makeInterpolatedPlan(checkpoint.state, goal_state, joint_names, 1.0),
            robot_model,
            20.0 * kPi / 180.0);
          std::string shortcut_reason;
          const auto check_start = std::chrono::steady_clock::now();
          const bool shortcut_valid = transition_planner.validate_plan(
            shortcut, checkpoint.state, &shortcut_reason);
          const auto check_end = std::chrono::steady_clock::now();
          const double check_ms = std::chrono::duration<double, std::milli>(
            check_end - check_start).count();
          shortcut_check_ms += check_ms;
          loaded_transition["shortcut_attempts"].push_back(Json{
            {"checkpoint_index", checkpoint_index},
            {"phase", checkpoint.phase},
            {"phase_step", checkpoint.phase_step},
            {"valid", shortcut_valid},
            {"failure_reason", shortcut_reason},
            {"check_ms", check_ms},
            {"point_count", shortcut.trajectory_.joint_trajectory.points.size()},
          });
          if (!shortcut_valid) continue;

          shortcut_selected = true;
          loaded_transition["start_phase"] = checkpoint.phase;
          loaded_transition["start_phase_step"] = checkpoint.phase_step;
          loaded_transition["start_state"] = stateJson(checkpoint.state);
          loaded_transition["valid"] = true;
          loaded_transition["method"] = "joint_interpolation";
          loaded_transition["failure_reason"] = "";
          loaded_transition["point_count"] = shortcut.trajectory_.joint_trajectory.points.size();
          loaded_transition["frames"] = Json::array();
          const auto& trajectory = shortcut.trajectory_.joint_trajectory;
          for (size_t index = 0; index < trajectory.points.size(); ++index) {
            const auto current = stateFromTrajectoryPoint(checkpoint.state, trajectory, index);
            loaded_transition["frames"].push_back(
              trajectoryFrameJson(current, solver, collisionContacts(scene, current), index));
          }
          break;
        }
        loaded_transition["shortcut_check_ms"] = shortcut_check_ms;
        if (!shortcut_selected) {
          if (top_horizontal_retract) {
            loaded_transition["failure_reason"] =
              checkpoints.empty() ? "path_failed_before_top_shortcut_start_step" :
              "no_valid_loaded_shortcut_before_path_failure";
          } else {
            const auto& fallback = checkpoints.back();
            start_candidates.emplace_back(fallback.phase, fallback.state);
            loaded_transition["fallback_after_shortcut_sweep"] = true;
            loaded_transition["fallback_phase_step"] = fallback.phase_step;
          }
        }
      } else if (options.resume_radial_after_orientation) {
        if (radial_resume_end_state) {
          start_candidates.emplace_back("radial_resume", *radial_resume_end_state);
        }
      } else {
        if (orientation_shortcut_precheck_valid && orientation_end_state) {
          start_candidates.emplace_back("orientation_only", *orientation_end_state);
        } else {
          start_candidates.emplace_back("radial_fallback", *last_valid_state);
          if (orientation_only_transition.value("progressed", false) && orientation_end_state) {
            start_candidates.emplace_back("orientation_only", *orientation_end_state);
          }
        }
      }
      bool selected_transition = loaded_transition.value("valid", false);
      double total_planning_ms = loaded_transition.value("shortcut_check_ms", 0.0);
      if (!selected_transition && start_candidates.empty() && !top_horizontal_retract) {
        loaded_transition["failure_reason"] = "no_radial_resume_state";
      }
      for (const auto& [start_phase, candidate_start] : start_candidates) {
        const auto planning_start = std::chrono::steady_clock::now();
        const auto transition = transition_planner.plan(candidate_start, goal_state);
        const auto planning_end = std::chrono::steady_clock::now();
        const double planning_ms = std::chrono::duration<double, std::milli>(
          planning_end - planning_start).count();
        total_planning_ms += planning_ms;
        loaded_transition["attempts"].push_back(Json{
          {"start_phase", start_phase},
          {"valid", transition.valid},
          {"method", transition.method},
          {"failure_reason", transition.failure_reason},
          {"planning_ms", planning_ms},
          {"point_count", transition.plan.trajectory_.joint_trajectory.points.size()},
        });
        if (selected_transition || (!transition.valid && start_phase != start_candidates.back().first)) {
          continue;
        }
        selected_transition = transition.valid;
        loaded_transition["start_phase"] = start_phase;
        if (options.interleave_loaded_shortcuts) {
          loaded_transition["start_phase_step"] =
            loaded_transition.value("fallback_phase_step", 0);
        }
        loaded_transition["start_state"] = stateJson(candidate_start);
        loaded_transition["valid"] = transition.valid;
        loaded_transition["method"] = transition.method;
        loaded_transition["failure_reason"] = transition.failure_reason;
        loaded_transition["point_count"] =
          transition.plan.trajectory_.joint_trajectory.points.size();
        loaded_transition["frames"] = Json::array();
        const auto& trajectory = transition.plan.trajectory_.joint_trajectory;
        for (size_t index = 0; index < trajectory.points.size(); ++index) {
          const auto current = stateFromTrajectoryPoint(candidate_start, trajectory, index);
          loaded_transition["frames"].push_back(
            trajectoryFrameJson(current, solver, collisionContacts(scene, current), index));
        }
        if (transition.valid) break;
      }
      loaded_transition["planning_ms"] = total_planning_ms;
      const std::string loaded_start_phase = loaded_transition.value("start_phase", "");
      const int loaded_start_phase_step = loaded_transition.value("start_phase_step", 0);
      orientation_only_transition["used_for_loaded_transition"] =
        loaded_start_phase == "orientation_only" || loaded_start_phase == "radial_resume";
      orientation_only_transition["used_frame_count"] =
        loaded_start_phase == "orientation_only" ? loaded_start_phase_step :
        (loaded_start_phase == "radial_resume" ? orientation_valid_states.size() : 0);
      radial_resume_transition["used_for_loaded_transition"] =
        loaded_start_phase == "radial_resume";
      radial_resume_transition["used_frame_count"] =
        loaded_start_phase == "radial_resume" ? loaded_start_phase_step : 0;
      output["orientation_only_transition"] = orientation_only_transition;
      output["radial_resume_transition"] = radial_resume_transition;
    }
    output["loaded_transition"] = std::move(loaded_transition);
    output["strategy_total_ms"] = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - strategy_start).count();

    std::filesystem::create_directories(options.output.parent_path());
    std::ofstream output_stream(options.output);
    output_stream << std::setw(2) << output << '\n';
    std::cout << "output=" << options.output << '\n'
              << "path_valid=" << std::boolalpha << path_valid << '\n'
              << "first_failure_frame=" << first_failure << '\n'
              << "first_failure_reason=" << first_failure_reason << '\n'
              << "max_joint_step_deg=" << degrees(max_joint_step) << '\n'
              << "max_updown_compensation=" << max_updown_compensation << '\n'
              << "solve_and_collision_ms=" << output["solve_and_collision_ms"] << '\n'
              << "loaded_transition_valid=" << output["loaded_transition"]["valid"] << '\n'
              << "loaded_transition_method=" << output["loaded_transition"]["method"] << '\n'
              << "loaded_transition_reason=" << output["loaded_transition"]["failure_reason"] << '\n';
    rclcpp::shutdown();
    return path_valid ? 0 : 3;
  } catch (const std::exception& error) {
    std::cerr << "analytic_radial_extract_prototype failed: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
