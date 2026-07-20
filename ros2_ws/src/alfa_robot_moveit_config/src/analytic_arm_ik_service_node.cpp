#include "alfa_robot_analytic_ik/analytic_ik.hpp"
#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"
#include "alfa_robot_moveit_config/runtime_status_publisher.hpp"
#include "robot_motion_interfaces/srv/solve_arm_ik.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace
{

using SolveArmIk = robot_motion_interfaces::srv::SolveArmIk;
using alfa_robot::analytic_ik::ArmAnalyticIkRequest;
using alfa_robot::analytic_ik::ArmSide;
using alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk;

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::string strip_leading_slash(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  return value;
}

std::string side_name(ArmSide side)
{
  return side == ArmSide::Left ? "left" : "right";
}

bool parse_side(const std::string& value, ArmSide& side)
{
  const std::string normalized = lower_copy(value);
  if (normalized == "left" || normalized == "l") {
    side = ArmSide::Left;
    return true;
  }
  if (normalized == "right" || normalized == "r") {
    side = ArmSide::Right;
    return true;
  }
  return false;
}

Eigen::Quaterniond quaternion_from_isometry(const Eigen::Isometry3d& transform)
{
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  return q;
}

geometry_msgs::msg::Pose pose_from_isometry(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const auto q = quaternion_from_isometry(transform);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

std::vector<std::string> joint_name_aliases(const std::string& side, size_t index)
{
  const std::string suffix = std::to_string(index + 1);
  return {
    side + "_joint" + suffix,
    side + "_joint" + suffix,
    side + "_v5_joint" + suffix,
  };
}

std::array<double, 6> seed_for_side(
  const sensor_msgs::msg::JointState& joint_state,
  ArmSide side)
{
  std::array<double, 6> seed{};
  const std::string prefix = side_name(side);
  for (size_t joint_index = 0; joint_index < seed.size(); ++joint_index) {
    for (const auto& alias : joint_name_aliases(prefix, joint_index)) {
      const auto found = std::find(joint_state.name.begin(), joint_state.name.end(), alias);
      if (found == joint_state.name.end()) {
        continue;
      }
      const auto source_index = static_cast<size_t>(std::distance(joint_state.name.begin(), found));
      if (source_index < joint_state.position.size()) {
        seed[joint_index] = joint_state.position[source_index];
      }
      break;
    }
  }
  return seed;
}

sensor_msgs::msg::JointState solution_to_joint_state(
  const rclcpp::Time& stamp,
  ArmSide side,
  double fixed_updown,
  const alfa_robot::analytic_ik::ArmAnalyticIkSolution& solution)
{
  sensor_msgs::msg::JointState out;
  out.header.stamp = stamp;
  out.name.reserve(7);
  out.position.reserve(7);
  out.name.push_back("updown");
  out.position.push_back(fixed_updown);

  const std::string prefix = side_name(side);
  for (size_t i = 0; i < solution.joints.size(); ++i) {
    out.name.push_back(prefix + "_joint" + std::to_string(i + 1));
    out.position.push_back(solution.joints[i]);
  }
  return out;
}

}  // namespace

class AnalyticArmIkServiceNode : public rclcpp::Node
{
public:
  AnalyticArmIkServiceNode()
  : Node("analytic_arm_ik_service")
  {
    service_name_ = declare_parameter<std::string>("service_name", "/robot_motion/solve_arm_ik");
    default_position_tolerance_ = declare_parameter<double>("default_position_tolerance", 1e-4);
    default_orientation_tolerance_ = declare_parameter<double>("default_orientation_tolerance", 1e-4);
    top_suction_orientation_tolerance_ =
      declare_parameter<double>("top_suction_orientation_tolerance", 5e-2);
    root_samples_ = static_cast<size_t>(
      std::max<long>(32, declare_parameter<int>("root_samples", 720)));
    default_max_solutions_ =
      static_cast<uint32_t>(
        std::max<long>(1, declare_parameter<int>("default_max_solutions", 8)));

    service_ = create_service<SolveArmIk>(
      service_name_,
      std::bind(&AnalyticArmIkServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    status_ = std::make_shared<alfa_robot::motion::RuntimeStatusPublisher>(
      *this,
      service_name_,
      "analytic_ik");

    if (declare_parameter<bool>("self_test", false)) {
      run_self_test();
    }

    RCLCPP_INFO(
      get_logger(),
      "Analytic arm IK service ready: service=%s root_samples=%zu default_max_solutions=%u",
      service_name_.c_str(),
      root_samples_,
      default_max_solutions_);
    status_->mark_ready("root_samples=" + std::to_string(root_samples_));
  }

private:
  void run_self_test()
  {
    SolveArmIk::Request request;
    request.side = "left";
    request.fixed_updown = 0.3;
    request.target.header.frame_id = "base_link";
    const std::array<double, 6> seed{};
    const Eigen::Isometry3d target_in_base =
      ThreeParallelArmAnalyticIk::baseLinkToArmBase(ArmSide::Left, request.fixed_updown) *
      solver_.forwardInArmBase(ArmSide::Left, seed);
    request.target.pose = pose_from_isometry(target_in_base);
    request.seed_state.name = {
      "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6"};
    request.seed_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    request.max_solutions = 8;

    auto request_ptr = std::make_shared<SolveArmIk::Request>(request);
    auto response = std::make_shared<SolveArmIk::Response>();
    handle_request(request_ptr, response);
    if (!response->success || response->solutions.empty()) {
      throw std::runtime_error("analytic_arm_ik_service self_test failed: " + response->message);
    }
    RCLCPP_INFO(
      get_logger(),
      "Analytic arm IK self-test passed: %s",
      response->message.c_str());
  }

  void handle_request(
    const std::shared_ptr<SolveArmIk::Request> request,
    std::shared_ptr<SolveArmIk::Response> response)
  {
    ArmSide side = ArmSide::Left;
    if (!parse_side(request->side, side)) {
      response->success = false;
      response->message = "unsupported_side:" + request->side;
      status_->mark_done(false, response->message);
      return;
    }

    const std::string frame_id = strip_leading_slash(request->target.header.frame_id);
    const std::string side_prefix = side_name(side);
    const Eigen::Isometry3d target = alfa_robot::motion::pose_to_eigen(request->target.pose);
    const auto seed = seed_for_side(request->seed_state, side);
    const double position_tolerance =
      request->position_tolerance > 0.0 ? request->position_tolerance : default_position_tolerance_;
    const double orientation_tolerance = request->orientation_tolerance > 0.0
      ? request->orientation_tolerance
      : (request->top_suction ? top_suction_orientation_tolerance_ : default_orientation_tolerance_);
    const uint32_t max_solutions =
      request->max_solutions > 0 ? request->max_solutions : default_max_solutions_;
    status_->mark_running(
      "side=" + side_prefix +
      " frame=" + (frame_id.empty() ? "base_link" : frame_id) +
      " h=" + std::to_string(request->fixed_updown));

    std::vector<alfa_robot::analytic_ik::ArmAnalyticIkSolution> solutions;
    if (frame_id.empty() || frame_id == "base_link") {
      solutions = solver_.solveInBaseLink(
        side,
        target,
        request->fixed_updown,
        seed,
        position_tolerance,
        orientation_tolerance,
        root_samples_);
    } else if (
      frame_id == side_prefix + "_arm_base" ||
      frame_id == side_prefix + "_base" ||
      frame_id == "arm_base") {
      ArmAnalyticIkRequest ik_request;
      ik_request.side = side;
      ik_request.target_in_arm_base = target;
      ik_request.seed = seed;
      ik_request.position_tolerance = position_tolerance;
      ik_request.orientation_tolerance = orientation_tolerance;
      ik_request.root_samples = root_samples_;
      solutions = solver_.solveInArmBase(ik_request);
    } else {
      response->success = false;
      response->message = "unsupported_frame:" + frame_id + " expected base_link or " +
        side_prefix + "_arm_base";
      status_->mark_done(false, response->message);
      return;
    }

    if (solutions.empty()) {
      std::ostringstream oss;
      oss << "no_solution side=" << side_prefix
          << " frame=" << (frame_id.empty() ? "base_link" : frame_id)
          << " fixed_updown=" << request->fixed_updown
          << " pos_tol=" << position_tolerance
          << " ori_tol=" << orientation_tolerance;
      response->success = false;
      response->message = oss.str();
      status_->mark_done(false, response->message);
      return;
    }

    const auto stamp = now();
    const size_t output_count = std::min<size_t>(solutions.size(), max_solutions);
    response->solutions.reserve(output_count);
    response->position_errors.reserve(output_count);
    response->orientation_errors.reserve(output_count);
    for (size_t i = 0; i < output_count; ++i) {
      response->solutions.push_back(
        solution_to_joint_state(stamp, side, request->fixed_updown, solutions[i]));
      response->position_errors.push_back(solutions[i].position_error);
      response->orientation_errors.push_back(solutions[i].orientation_error);
    }

    std::ostringstream oss;
    oss << "ok side=" << side_prefix
        << " frame=" << (frame_id.empty() ? "base_link" : frame_id)
        << " returned=" << output_count
        << " total=" << solutions.size();
    response->success = true;
    response->message = oss.str();
    status_->mark_done(true, response->message);
  }

  std::string service_name_;
  double default_position_tolerance_ = 1e-4;
  double default_orientation_tolerance_ = 1e-4;
  double top_suction_orientation_tolerance_ = 5e-2;
  size_t root_samples_ = 720;
  uint32_t default_max_solutions_ = 8;
  ThreeParallelArmAnalyticIk solver_;
  rclcpp::Service<SolveArmIk>::SharedPtr service_;
  std::shared_ptr<alfa_robot::motion::RuntimeStatusPublisher> status_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalyticArmIkServiceNode>());
  rclcpp::shutdown();
  return 0;
}
