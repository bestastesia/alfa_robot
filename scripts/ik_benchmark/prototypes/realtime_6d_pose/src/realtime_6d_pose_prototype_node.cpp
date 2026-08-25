#include "realtime_6d_pose_pipeline.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using ik_benchmark::prototype::Realtime6dPoseConfig;
using ik_benchmark::prototype::Realtime6dPosePipeline;
using ik_benchmark::prototype::Realtime6dPoseResult;
using SteadyClock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

const std::vector<std::string> kDefaultRtControlJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

std::string read_file(const std::string& path)
{
  std::ifstream file(path);
  if (!file.good()) {
    throw std::runtime_error("cannot read: " + path);
  }
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::string run_xacro(const std::string& path)
{
  const std::string command = "xacro '" + path + "'";
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("failed to start xacro for: " + path);
  }
  std::string output;
  std::array<char, 8192> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    output += buffer.data();
  }
  if (output.empty()) {
    throw std::runtime_error("xacro returned empty URDF for: " + path);
  }
  return output;
}

moveit::core::RobotModelPtr load_current_robot_model()
{
  const std::string description_share =
    ament_index_cpp::get_package_share_directory("alfa_robot_description");
  const std::string moveit_share =
    ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");
  const std::string installed_urdf =
    description_share + "/urdf/alfa_robot/alfa_robot.urdf";
  std::ifstream installed_file(installed_urdf);
  const std::string urdf_xml = installed_file.good() ?
    read_file(installed_urdf) : run_xacro(description_share + "/urdf/alfa_robot.urdf.xacro");
  const std::string srdf_xml = read_file(moveit_share + "/config/alfa_robot.srdf");

  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initString(urdf_xml)) {
    throw std::runtime_error("failed to parse current URDF");
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, srdf_xml)) {
    throw std::runtime_error("failed to parse current SRDF");
  }
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

std::array<double, 6> to_array6(const std::vector<double>& values, const std::string& label)
{
  if (values.size() != 6) {
    throw std::invalid_argument(label + " must contain exactly 6 values");
  }
  std::array<double, 6> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  if (quaternion.norm() < 1e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose pose_from_eigen(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond quaternion(transform.linear());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double percentile(std::vector<double> values, double ratio)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    std::round(std::clamp(ratio, 0.0, 1.0) * static_cast<double>(values.size() - 1)));
  return values[index];
}

}  // namespace

class Realtime6dPosePrototypeNode : public rclcpp::Node
{
public:
  Realtime6dPosePrototypeNode()
  : Node("realtime_6d_pose_prototype"), robot_model_(load_current_robot_model())
  {
    const std::string arm = declare_parameter<std::string>("arm", "left");
    benchmark_mode_ = declare_parameter<bool>("benchmark_mode", false);
    wait_for_feedback_init_ = declare_parameter<bool>("wait_for_feedback_init", true);
    feedback_sync_enabled_ = declare_parameter<bool>("feedback_sync_enabled", true);
    pose_source_ = declare_parameter<std::string>("pose_source", "synthetic");
    target_rate_hz_ = declare_parameter<double>("target_rate_hz", 100.0);
    path_period_s_ = declare_parameter<double>("path_period_s", 5.0);
    benchmark_samples_ = declare_parameter<int>("benchmark_samples", 5000);
    benchmark_cycle_samples_ = declare_parameter<int>("benchmark_cycle_samples", 1000);
    publish_every_n_ = declare_parameter<int>("publish_every_n", 1);
    rt_control_joint_names_ = declare_parameter<std::vector<std::string>>(
      "rt_control_joint_names", kDefaultRtControlJointNames);
    const double fixed_updown = declare_parameter<double>("fixed_updown", 0.3);
    const double jump_threshold_deg = declare_parameter<double>("jump_threshold_deg", 10.0);
    const double collision_edge_step_deg =
      declare_parameter<double>("collision_edge_step_deg", 2.0);
    position_amplitudes_ = to_array6(
      declare_parameter<std::vector<double>>(
        "position_amplitudes_m", std::vector<double>{0.012, 0.010, 0.008, 0.0, 0.0, 0.0}),
      "position_amplitudes_m");
    orientation_amplitudes_ = to_array6(
      declare_parameter<std::vector<double>>(
        "orientation_amplitudes_deg", std::vector<double>{2.0, 2.0, 3.0, 0.0, 0.0, 0.0}),
      "orientation_amplitudes_deg");
    const auto initial_arm = to_array6(
      declare_parameter<std::vector<double>>(
        "initial_arm_joints_deg", std::vector<double>{0.0, -45.0, 120.0, -75.0, 0.0, 0.0}),
      "initial_arm_joints_deg");
    const auto initial_other = to_array6(
      declare_parameter<std::vector<double>>(
        "initial_other_arm_joints_deg", std::vector<double>{0.0, -45.0, 120.0, -75.0, 0.0, 0.0}),
      "initial_other_arm_joints_deg");

    const bool left = arm == "left";
    if (!left && arm != "right") {
      throw std::invalid_argument("arm must be 'left' or 'right'");
    }
    Realtime6dPoseConfig config;
    config.side = left ? alfa_robot::analytic_ik::ArmSide::Left :
      alfa_robot::analytic_ik::ArmSide::Right;
    config.group_name = left ? "left_arm" : "right_arm";
    config.tool_link = left ? "left_tool0" : "right_tool0";
    config.fixed_updown = fixed_updown;
    config.jump_threshold_rad = jump_threshold_deg * kPi / 180.0;
    config.collision_edge_step_rad = collision_edge_step_deg * kPi / 180.0;

    auto radians = [](std::array<double, 6> values) {
      for (double& value : values) {
        value *= kPi / 180.0;
      }
      return values;
    };
    pipeline_ = std::make_unique<Realtime6dPosePipeline>(
      robot_model_, config, radians(initial_arm), radians(initial_other));
    left_arm_ = left;
    if (rt_control_joint_names_.empty()) {
      throw std::invalid_argument("rt_control_joint_names must not be empty");
    }
    const auto& model_variable_names = robot_model_->getVariableNames();
    for (const auto& joint_name : rt_control_joint_names_) {
      if (std::find(model_variable_names.begin(), model_variable_names.end(), joint_name) ==
        model_variable_names.end())
      {
        throw std::invalid_argument(
                "rt_control_joint_names contains unknown model variable: " + joint_name);
      }
    }
    baseline_pose_ = pipeline_->currentToolPoseInBase();
    latest_topic_target_ = baseline_pose_;
    feedback_initialized_ = benchmark_mode_ || !wait_for_feedback_init_;

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/realtime_6d_pose/target_pose", rclcpp::SensorDataQoS());
    initial_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/realtime_6d_pose/initial_target_pose", rclcpp::QoS(1).transient_local().reliable());
    commanded_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/realtime_6d_pose/commanded_pose", rclcpp::SensorDataQoS());
    commanded_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/realtime_6d_pose/commanded_joint_states", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/realtime_6d_pose/status", rclcpp::SensorDataQoS());

    if (feedback_initialized_) {
      publishInitialTarget();
    }

    if (feedback_sync_enabled_) {
      feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/realtime_6d_pose/joint_states",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (!feedback_initialized_ && synchronizeFromJointState(*msg)) {
            baseline_pose_ = pipeline_->currentToolPoseInBase();
            latest_topic_target_ = baseline_pose_;
            feedback_initialized_ = true;
            publishInitialTarget();
            RCLCPP_INFO(get_logger(), "initialized Motion IK state from first complete feedback");
          }
        });
      accepted_reference_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/realtime_6d_pose/accepted_reference_joint_states",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (feedback_initialized_) {
            synchronizeFromJointState(*msg);
          }
        });
    }

    if (pose_source_ == "topic") {
      target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/realtime_6d_pose/target_pose_cmd",
        rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "ignore target frame '%s'; prototype expects base_link", msg->header.frame_id.c_str());
            return;
          }
          latest_topic_target_ = pose_to_eigen(msg->pose);
          topic_target_received_ = true;
          topic_target_dirty_ = true;
        });
    } else if (pose_source_ != "synthetic") {
      throw std::invalid_argument("pose_source must be 'synthetic' or 'topic'");
    }

    start_time_ = SteadyClock::now();
    report_window_start_ = start_time_;
    if (!benchmark_mode_) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, target_rate_hz_)),
        [this]() { on_timer(); });
    }

    RCLCPP_INFO(
      get_logger(),
      "PROTOTYPE ready: arm=%s source=%s fixed_updown=%.3fm rate=%.1fHz jump=%.1fdeg edge=%.1fdeg benchmark=%s wait_feedback=%s feedback_sync=%s",
      arm.c_str(), pose_source_.c_str(), fixed_updown, target_rate_hz_, jump_threshold_deg,
      collision_edge_step_deg, benchmark_mode_ ? "true" : "false",
      wait_for_feedback_init_ ? "true" : "false",
      feedback_sync_enabled_ ? "true" : "false");
  }

  bool benchmarkMode() const { return benchmark_mode_; }

  void runBenchmark()
  {
    std::vector<double> total_ms;
    std::vector<double> ik_ms;
    std::vector<double> collision_ms;
    total_ms.reserve(static_cast<std::size_t>(std::max(1, benchmark_samples_)));
    ik_ms.reserve(total_ms.capacity());
    collision_ms.reserve(total_ms.capacity());
    std::size_t accepted = 0;
    std::size_t rejected = 0;
    std::size_t total_collision_checks = 0;
    std::map<std::string, std::size_t> status_counts;
    double max_position_error_m = 0.0;
    double max_orientation_error_rad = 0.0;
    const auto wall_start = SteadyClock::now();
    for (int i = 0; i < std::max(1, benchmark_samples_); ++i) {
      const double phase = 2.0 * kPi * static_cast<double>(i % std::max(1, benchmark_cycle_samples_)) /
        static_cast<double>(std::max(1, benchmark_cycle_samples_));
      const auto result = pipeline_->process(synthetic_target(phase));
      total_ms.push_back(result.timing.total_ms);
      ik_ms.push_back(result.timing.ik_ms);
      collision_ms.push_back(result.timing.collision_ms);
      total_collision_checks += result.collision_state_checks;
      ++status_counts[result.status];
      max_position_error_m = std::max(max_position_error_m, result.position_error_m);
      max_orientation_error_rad = std::max(
        max_orientation_error_rad, result.orientation_error_rad);
      result.accepted ? ++accepted : ++rejected;
    }
    const double wall_s = std::chrono::duration<double>(SteadyClock::now() - wall_start).count();
    const double average_total = std::accumulate(total_ms.begin(), total_ms.end(), 0.0) /
      static_cast<double>(total_ms.size());
    nlohmann::json summary = {
      {"prototype", "realtime_6d_pose"},
      {"samples", total_ms.size()},
      {"accepted", accepted},
      {"rejected", rejected},
      {"acceptance_rate", static_cast<double>(accepted) / static_cast<double>(total_ms.size())},
      {"wall_seconds", wall_s},
      {"throughput_hz", static_cast<double>(total_ms.size()) / std::max(1e-9, wall_s)},
      {"total_ms", {{"mean", average_total}, {"p50", percentile(total_ms, 0.50)},
        {"p95", percentile(total_ms, 0.95)}, {"max", *std::max_element(total_ms.begin(), total_ms.end())}}},
      {"ik_ms", {{"p50", percentile(ik_ms, 0.50)}, {"p95", percentile(ik_ms, 0.95)}}},
      {"collision_ms", {{"p50", percentile(collision_ms, 0.50)},
        {"p95", percentile(collision_ms, 0.95)}}},
      {"collision_state_checks", total_collision_checks},
      {"status_counts", status_counts},
      {"max_position_error_m", max_position_error_m},
      {"max_orientation_error_rad", max_orientation_error_rad},
      {"fixed_updown", pipeline_->config().fixed_updown},
      {"jump_threshold_deg", pipeline_->config().jump_threshold_rad * 180.0 / kPi},
      {"collision_edge_step_deg", pipeline_->config().collision_edge_step_rad * 180.0 / kPi},
    };
    std::cout << summary.dump(2) << std::endl;
  }

private:
  void publishInitialTarget()
  {
    geometry_msgs::msg::PoseStamped initial_target;
    initial_target.header.stamp = now();
    initial_target.header.frame_id = "base_link";
    initial_target.pose = pose_from_eigen(baseline_pose_);
    initial_target_pub_->publish(initial_target);
  }

  bool synchronizeFromJointState(const sensor_msgs::msg::JointState& message)
  {
    if (message.name.size() != message.position.size()) {
      return false;
    }
    std::map<std::string, double> values;
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      if (std::isfinite(message.position[i])) {
        values[message.name[i]] = message.position[i];
      }
    }
    for (const auto& name : rt_control_joint_names_) {
      if (values.find(name) == values.end()) {
        return false;
      }
    }
    std::array<double, 6> right{};
    std::array<double, 6> left{};
    for (std::size_t i = 0; i < 6; ++i) {
      right[i] = values.at("right_joint" + std::to_string(i + 1));
      left[i] = values.at("left_joint" + std::to_string(i + 1));
    }
    pipeline_->synchronizeState(
      left_arm_ ? left : right,
      left_arm_ ? right : left,
      values.at("turn"),
      values.at("updown"));
    return true;
  }

  Eigen::Isometry3d synthetic_target(double phase) const
  {
    Eigen::Isometry3d target = baseline_pose_;
    target.translation().x() += position_amplitudes_[0] * std::sin(phase);
    target.translation().y() += position_amplitudes_[1] * std::sin(2.0 * phase + 0.4);
    target.translation().z() += position_amplitudes_[2] * std::sin(3.0 * phase + 0.8);
    const double roll = orientation_amplitudes_[0] * kPi / 180.0 * std::sin(phase + 0.2);
    const double pitch = orientation_amplitudes_[1] * kPi / 180.0 * std::sin(2.0 * phase + 0.7);
    const double yaw = orientation_amplitudes_[2] * kPi / 180.0 * std::sin(3.0 * phase + 1.1);
    target.linear() = baseline_pose_.linear() *
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return target;
  }

  void on_timer()
  {
    if (!feedback_initialized_) {
      return;
    }
    Eigen::Isometry3d target;
    Realtime6dPoseResult result;
    if (pose_source_ == "topic") {
      // Do not turn the internally generated baseline pose into a command.
      // Real target tracking is armed only by an external Marker/Pose message.
      if (!topic_target_received_ || !latest_topic_target_) {
        return;
      }
      target = *latest_topic_target_;
      if (topic_target_dirty_ || !cached_topic_result_) {
        cached_topic_result_ = pipeline_->process(target);
        topic_target_dirty_ = false;
      }
      result = *cached_topic_result_;
    } else {
      const double elapsed = std::chrono::duration<double>(SteadyClock::now() - start_time_).count();
      target = synthetic_target(2.0 * kPi * elapsed / std::max(0.1, path_period_s_));
      result = pipeline_->process(target);
    }
    ++sequence_;
    ++window_count_;
    result.accepted ? ++window_accepted_ : ++window_rejected_;
    if (publish_every_n_ <= 1 || sequence_ % static_cast<std::size_t>(publish_every_n_) == 0) {
      publish(target, result);
    }
    const double report_s = std::chrono::duration<double>(SteadyClock::now() - report_window_start_).count();
    if (report_s >= 1.0) {
      RCLCPP_INFO(
        get_logger(),
        "loop=%.1fHz accepted=%zu rejected=%zu last=%s total=%.3fms ik=%.3fms collision=%.3fms checks=%zu",
        static_cast<double>(window_count_) / report_s,
        window_accepted_, window_rejected_, result.status.c_str(), result.timing.total_ms,
        result.timing.ik_ms, result.timing.collision_ms, result.collision_state_checks);
      report_window_start_ = SteadyClock::now();
      window_count_ = 0;
      window_accepted_ = 0;
      window_rejected_ = 0;
    }
  }

  void publish(const Eigen::Isometry3d& target, const Realtime6dPoseResult& result)
  {
    const auto stamp = now();
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = stamp;
    target_msg.header.frame_id = "base_link";
    target_msg.pose = pose_from_eigen(target);
    target_pub_->publish(target_msg);

    if (result.accepted) {
      geometry_msgs::msg::PoseStamped commanded_pose_msg;
      commanded_pose_msg.header = target_msg.header;
      commanded_pose_msg.pose = pose_from_eigen(result.actual_pose_in_base);
      commanded_pose_pub_->publish(commanded_pose_msg);

      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = stamp;
      joint_msg.name = robot_model_->getVariableNames();
      joint_msg.position.assign(
        pipeline_->currentState().getVariablePositions(),
        pipeline_->currentState().getVariablePositions() + joint_msg.name.size());
      // A cached Cartesian solution remains the active-arm goal while the
      // accepted-reference callback keeps the other eight axes authoritative.
      const std::string active_prefix = left_arm_ ? "left_joint" : "right_joint";
      for (std::size_t axis = 0; axis < result.joints.size(); ++axis) {
        const auto found = std::find(
          joint_msg.name.begin(), joint_msg.name.end(),
          active_prefix + std::to_string(axis + 1));
        if (found != joint_msg.name.end()) {
          joint_msg.position[static_cast<std::size_t>(found - joint_msg.name.begin())] =
            result.joints[axis];
        }
      }
      commanded_joint_pub_->publish(joint_msg);

    }

    const double window_s = std::chrono::duration<double>(SteadyClock::now() - report_window_start_).count();
    nlohmann::json status = {
      {"sequence", sequence_},
      {"accepted", result.accepted},
      {"status", result.status},
      {"loop_hz", window_s > 1e-6 ? static_cast<double>(window_count_) / window_s : 0.0},
      {"ik_ms", result.timing.ik_ms},
      {"jump_ms", result.timing.jump_ms},
      {"collision_ms", result.timing.collision_ms},
      {"total_ms", result.timing.total_ms},
      {"solutions", result.analytic_solution_count},
      {"max_joint_delta_deg", result.max_joint_delta_rad * 180.0 / kPi},
      {"position_error_m", result.position_error_m},
      {"orientation_error_rad", result.orientation_error_rad},
      {"jump_rejections", result.jump_rejections},
      {"collision_rejections", result.collision_rejections},
      {"collision_state_checks", result.collision_state_checks},
      {"fixed_updown", pipeline_->config().fixed_updown},
      {"feedback_initialized", feedback_initialized_},
    };
    std_msgs::msg::String status_msg;
    status_msg.data = status.dump();
    status_pub_->publish(status_msg);
  }

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<Realtime6dPosePipeline> pipeline_;
  Eigen::Isometry3d baseline_pose_ = Eigen::Isometry3d::Identity();
  std::array<double, 6> position_amplitudes_{};
  std::array<double, 6> orientation_amplitudes_{};
  bool benchmark_mode_ = false;
  bool wait_for_feedback_init_ = true;
  bool feedback_sync_enabled_ = true;
  bool feedback_initialized_ = false;
  bool left_arm_ = true;
  std::string pose_source_ = "synthetic";
  double target_rate_hz_ = 100.0;
  double path_period_s_ = 5.0;
  int benchmark_samples_ = 5000;
  int benchmark_cycle_samples_ = 1000;
  int publish_every_n_ = 1;
  std::vector<std::string> rt_control_joint_names_;
  std::size_t sequence_ = 0;
  std::size_t window_count_ = 0;
  std::size_t window_accepted_ = 0;
  std::size_t window_rejected_ = 0;
  SteadyClock::time_point start_time_;
  SteadyClock::time_point report_window_start_;
  std::optional<Eigen::Isometry3d> latest_topic_target_;
  std::optional<Realtime6dPoseResult> cached_topic_result_;
  bool topic_target_received_ = false;
  bool topic_target_dirty_ = false;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr commanded_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commanded_joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr accepted_reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Realtime6dPosePrototypeNode>();
    if (node->benchmarkMode()) {
      node->runBenchmark();
    } else {
      rclcpp::spin(node);
    }
  } catch (const std::exception& error) {
    std::cerr << "realtime_6d_pose_prototype failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
