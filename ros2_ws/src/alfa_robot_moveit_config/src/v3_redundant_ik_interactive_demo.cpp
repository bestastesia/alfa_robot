#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
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
using BranchKey = std::tuple<int, int, int>;

constexpr double kPi = 3.14159265358979323846;
constexpr char kTargetMarkerName[] = "target_pose";

double degToRad(double value)
{
  return value * kPi / 180.0;
}

double radToDeg(double value)
{
  return value * 180.0 / kPi;
}

double normalizeAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double maximumJointDelta(
  const std::array<double, 7>& from,
  const std::array<double, 7>& to)
{
  double maximum_delta = 0.0;
  for (size_t index = 0; index < from.size(); ++index) {
    maximum_delta = std::max(
      maximum_delta, std::abs(normalizeAngle(to[index] - from[index])));
  }
  return maximum_delta;
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  rotation = rotation.norm() < 1e-12 ?
    Eigen::Quaterniond::Identity() : rotation.normalized();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = rotation.toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Quaternion quaternionFromRpy(
  double roll, double pitch, double yaw)
{
  const Eigen::Quaterniond rotation(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  geometry_msgs::msg::Quaternion message;
  message.x = rotation.x();
  message.y = rotation.y();
  message.z = rotation.z();
  message.w = rotation.w();
  return message;
}

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA message;
  message.r = red;
  message.g = green;
  message.b = blue;
  message.a = alpha;
  return message;
}

InteractiveMarkerControl axisControl(
  const std::string& name,
  uint8_t interaction_mode,
  double x,
  double y,
  double z)
{
  InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = interaction_mode;
  control.orientation.w = 1.0;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  return control;
}

Marker targetBodyMarker()
{
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = 0.08;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color = color(0.15F, 0.55F, 1.0F, 0.78F);
  return marker;
}

struct FamilySample
{
  double swivel_angle = 0.0;
  V3RedundantIkSolution solution;
};

class V3RedundantIkInteractiveDemo : public rclcpp::Node
{
public:
  V3RedundantIkInteractiveDemo()
  : Node("v3_redundant_ik_interactive_demo")
  {
    side_ = declare_parameter<std::string>("side", "left");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    arm_base_frame_ = declare_parameter<std::string>("arm_base_frame", "arm_carriage");
    const auto initial_target = declare_parameter<std::vector<double>>(
      "initial_target_xyzrpy", {0.73, -0.20, 0.55, 0.0, kPi / 2.0, 0.0});
    psi_step_ = degToRad(std::clamp(
      declare_parameter<double>("psi_step_deg", 2.0), 0.1, 30.0));
    maximum_segment_joint_jump_ = degToRad(std::clamp(
      declare_parameter<double>("maximum_segment_joint_jump_deg", 20.0), 1.0, 180.0));
    debounce_duration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(
        0.0, declare_parameter<double>("target_debounce_s", 0.15))));

    if (side_ != "left" && side_ != "right") {
      throw std::invalid_argument("side must be 'left' or 'right'");
    }
    if (initial_target.size() != 6) {
      throw std::invalid_argument("initial_target_xyzrpy must have 6 values");
    }

    solver_ = std::make_unique<V3RedundantArmAnalyticIk>(
      side_ == "left" ? V3RedundantArmModel::V311Left : V3RedundantArmModel::V311Right);
    target_pose_.position.x = initial_target[0];
    target_pose_.position.y = initial_target[1];
    target_pose_.position.z = initial_target[2];
    target_pose_.orientation = quaternionFromRpy(
      initial_target[3], initial_target[4], initial_target[5]);

    for (const std::string arm_side : {std::string("left"), std::string("right")}) {
      for (int index = 1; index <= 7; ++index) {
        all_joint_names_.push_back(arm_side + "_joint" + std::to_string(index));
      }
    }
    active_joint_offset_ = side_ == "left" ? 0U : 7U;
    all_joint_positions_.assign(14, 0.0);

    family_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/solution_family_json", rclcpp::QoS(1).transient_local());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/status_markers", rclcpp::QoS(1).transient_local());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "v3_redundant_ik_demo",
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_topics_interface(),
      get_node_services_interface());

    createTargetMarker();
    marker_server_->applyChanges();
    pending_generation_ = true;
    pending_since_ = std::chrono::steady_clock::now() - debounce_duration_;
    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "V3 redundant solution-family demo ready: side=%s psi_step=%.2fdeg",
      side_.c_str(), radToDeg(psi_step_));
  }

private:
  void createTargetMarker()
  {
    InteractiveMarker target;
    target.header.frame_id = world_frame_;
    target.name = kTargetMarkerName;
    target.description = "拖动末端目标，Rerun自动播放全部冗余解族";
    target.scale = 0.25;
    target.pose = target_pose_;

    InteractiveMarkerControl body;
    body.always_visible = true;
    body.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    body.markers.push_back(targetBodyMarker());
    target.controls.push_back(body);
    target.controls.push_back(axisControl(
      "rotate_x", InteractiveMarkerControl::ROTATE_AXIS, 1.0, 0.0, 0.0));
    target.controls.push_back(axisControl(
      "move_x", InteractiveMarkerControl::MOVE_AXIS, 1.0, 0.0, 0.0));
    target.controls.push_back(axisControl(
      "rotate_y", InteractiveMarkerControl::ROTATE_AXIS, 0.0, 1.0, 0.0));
    target.controls.push_back(axisControl(
      "move_y", InteractiveMarkerControl::MOVE_AXIS, 0.0, 1.0, 0.0));
    target.controls.push_back(axisControl(
      "rotate_z", InteractiveMarkerControl::ROTATE_AXIS, 0.0, 0.0, 1.0));
    target.controls.push_back(axisControl(
      "move_z", InteractiveMarkerControl::MOVE_AXIS, 0.0, 0.0, 1.0));
    marker_server_->insert(
      target,
      [this](const Feedback::ConstSharedPtr& feedback) {handleTargetFeedback(feedback);});
  }

  void handleTargetFeedback(const Feedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type != Feedback::POSE_UPDATE &&
        feedback->event_type != Feedback::MOUSE_UP) {
      return;
    }
    target_pose_ = feedback->pose;
    marker_server_->setPose(kTargetMarkerName, target_pose_);
    marker_server_->applyChanges();
    pending_generation_ = true;
    pending_since_ = feedback->event_type == Feedback::MOUSE_UP ?
      std::chrono::steady_clock::now() - debounce_duration_ :
      std::chrono::steady_clock::now();
  }

  bool updateArmBaseTransform()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        arm_base_frame_, world_frame_, tf2::TimePointZero);
      world_to_arm_base_ = tf2::transformToEigen(transform.transform);
      transform_ready_ = true;
      return true;
    } catch (const tf2::TransformException&) {
      transform_ready_ = false;
      return false;
    }
  }

  void onTimer()
  {
    if (!transform_ready_ && !updateArmBaseTransform()) {
      publishStatus("WAITING TF", false);
      return;
    }
    publishJointState();
    if (!pending_generation_) {
      return;
    }
    if (std::chrono::steady_clock::now() - pending_since_ < debounce_duration_) {
      return;
    }
    pending_generation_ = false;
    generateAndPublishFamily();
  }

  void generateAndPublishFamily()
  {
    const auto started = std::chrono::steady_clock::now();
    const Eigen::Isometry3d target_in_arm_base = world_to_arm_base_ * poseToEigen(target_pose_);
    std::map<BranchKey, std::vector<FamilySample>> samples_by_branch;
    std::optional<V3RedundantIkSolution> representative;
    size_t valid_solution_count = 0;
    size_t sampled_psi_count = 0;

    const int interval_count = static_cast<int>(std::ceil(2.0 * kPi / psi_step_));
    for (int index = 0; index <= interval_count; ++index) {
      const double swivel_angle = index == interval_count ?
        kPi : -kPi + static_cast<double>(index) * 2.0 * kPi / interval_count;
      V3RedundantIkRequest request;
      request.target_in_arm_base = target_in_arm_base;
      request.swivel_angle = swivel_angle;
      request.seed = representative_joints_;
      const auto solutions = solver_->solveInArmBase(request);
      ++sampled_psi_count;
      std::vector<std::array<double, 7>> unique_at_psi;
      for (const auto& solution : solutions) {
        const bool duplicate = std::any_of(
          unique_at_psi.begin(), unique_at_psi.end(),
          [&solution](const std::array<double, 7>& joints) {
            return maximumJointDelta(joints, solution.joints) < 1e-7;
          });
        if (duplicate) {
          continue;
        }
        unique_at_psi.push_back(solution.joints);
        const BranchKey branch{
          solution.shoulder_branch,
          solution.elbow_branch,
          solution.wrist_branch};
        samples_by_branch[branch].push_back(FamilySample{swivel_angle, solution});
        ++valid_solution_count;
        if (!representative || solution.seed_distance < representative->seed_distance) {
          representative = solution;
        }
      }
    }

    nlohmann::json payload;
    payload["generation"] = ++generation_;
    payload["side"] = side_;
    payload["world_frame"] = world_frame_;
    payload["target_pose"] = {
      {"position", {
        target_pose_.position.x,
        target_pose_.position.y,
        target_pose_.position.z}},
      {"orientation_xyzw", {
        target_pose_.orientation.x,
        target_pose_.orientation.y,
        target_pose_.orientation.z,
        target_pose_.orientation.w}},
    };
    payload["psi_step_deg"] = radToDeg(2.0 * kPi / interval_count);
    payload["sampled_psi_count"] = sampled_psi_count;
    payload["valid_solution_count"] = valid_solution_count;
    payload["segments"] = nlohmann::json::array();

    size_t segment_id = 0;
    for (const auto& [branch, branch_samples] : samples_by_branch) {
      std::vector<FamilySample> current_segment;
      const auto flush_segment = [&]() {
          if (current_segment.empty()) {
            return;
          }
          nlohmann::json segment;
          segment["segment_id"] = segment_id++;
          segment["branch"] = {
            {"shoulder", std::get<0>(branch)},
            {"elbow", std::get<1>(branch)},
            {"wrist", std::get<2>(branch)},
          };
          segment["samples"] = nlohmann::json::array();
          for (const auto& sample : current_segment) {
            segment["samples"].push_back({
              {"psi_deg", radToDeg(sample.swivel_angle)},
              {"joints", sample.solution.joints},
              {"minimum_joint_limit_margin_deg",
                radToDeg(sample.solution.minimum_joint_limit_margin)},
            });
          }
          payload["segments"].push_back(std::move(segment));
          current_segment.clear();
        };

      for (const auto& sample : branch_samples) {
        if (!current_segment.empty()) {
          const auto& previous = current_segment.back();
          const double psi_gap = sample.swivel_angle - previous.swivel_angle;
          const double joint_jump = maximumJointDelta(
            previous.solution.joints, sample.solution.joints);
          if (psi_gap > 1.5 * psi_step_ || joint_jump > maximum_segment_joint_jump_) {
            flush_segment();
          }
        }
        current_segment.push_back(sample);
      }
      flush_segment();
    }

    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    payload["segment_count"] = segment_id;
    payload["solve_ms"] = elapsed;
    payload["collision_checked"] = false;
    payload["diagnostic"] = representative ? nlohmann::json::object() : nlohmann::json{
      {"diagnostic_only", true}, {"freeze_at_end", true}, {"stage", "analytic_ik"},
      {"reason", "no legal IK family; holding last state (IK-only, no collision checking)"},
      {"snapshot", "last_available_state_no_rejected_configuration"},
      {"target", {target_pose_.position.x, target_pose_.position.y, target_pose_.position.z}}};
    std_msgs::msg::String message;
    message.data = payload.dump();
    family_publisher_->publish(message);

    if (representative) {
      representative_joints_ = representative->joints;
      for (size_t index = 0; index < representative_joints_.size(); ++index) {
        all_joint_positions_[active_joint_offset_ + index] = representative_joints_[index];
      }
      publishJointState();
      std::ostringstream status;
      status << "solutions=" << valid_solution_count
             << " segments=" << segment_id
             << " solve=" << std::fixed << std::setprecision(1) << elapsed << "ms";
      publishStatus(status.str(), true);
    } else {
      publishStatus("NO LEGAL IK - FROZEN AT LAST STATE\nRED TARGET: NO SOLUTION (IK ONLY, NOT COLLISION CHECKED)", false);
    }
  }

  void publishJointState()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = all_joint_names_;
    message.position = all_joint_positions_;
    joint_state_publisher_->publish(message);
  }

  void publishStatus(const std::string& message, bool success)
  {
    visualization_msgs::msg::MarkerArray array;
    Marker text;
    text.header.frame_id = world_frame_;
    text.header.stamp = now();
    text.ns = "v3_redundant_ik_family_demo";
    text.id = 0;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.position = target_pose_.position;
    text.pose.position.z += 0.14;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.03;
    text.color = success ?
      color(0.15F, 1.0F, 0.25F, 1.0F) : color(1.0F, 0.15F, 0.15F, 1.0F);
    text.text = message;
    array.markers.push_back(text);
    Marker target = text;
    target.id = 1;
    target.type = Marker::SPHERE;
    target.action = success ? Marker::DELETE : Marker::ADD;
    target.pose.position = target_pose_.position;
    target.scale.x = target.scale.y = target.scale.z = 0.045;
    array.markers.push_back(target);
    status_publisher_->publish(array);
  }

  std::string side_;
  std::string world_frame_;
  std::string arm_base_frame_;
  std::unique_ptr<V3RedundantArmAnalyticIk> solver_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr family_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  geometry_msgs::msg::Pose target_pose_;
  Eigen::Isometry3d world_to_arm_base_ = Eigen::Isometry3d::Identity();
  std::vector<std::string> all_joint_names_;
  std::vector<double> all_joint_positions_;
  std::array<double, 7> representative_joints_{};
  size_t active_joint_offset_ = 0;
  double psi_step_ = degToRad(2.0);
  double maximum_segment_joint_jump_ = degToRad(20.0);
  std::chrono::steady_clock::duration debounce_duration_{};
  std::chrono::steady_clock::time_point pending_since_{};
  bool transform_ready_ = false;
  bool pending_generation_ = false;
  uint64_t generation_ = 0;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<V3RedundantIkInteractiveDemo>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("v3_redundant_ik_interactive_demo"),
      "Interactive IK family demo failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
