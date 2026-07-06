#include "alfa_robot_moveit_config/motion_core/pose_math.hpp"

#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace alfa_robot::motion
{

std::vector<double> deg_to_rad(const std::vector<double>& degrees)
{
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (double degree : degrees) {
    radians.push_back(degree * M_PI / 180.0);
  }
  return radians;
}

std::vector<double> parse_degrees_list(std::string value)
{
  value = trim_copy(value);
  if (!value.empty() && value.front() == '[') value.erase(value.begin());
  if (!value.empty() && value.back() == ']') value.pop_back();
  std::vector<double> result;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = trim_copy(token);
    if (!token.empty()) {
      result.push_back(std::stod(token));
    }
  }
  return result;
}

std::vector<std::vector<double>> parse_pose_family_degrees(const std::string& value)
{
  std::vector<std::vector<double>> family;
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, ';')) {
    auto pose_deg = parse_degrees_list(segment);
    if (pose_deg.size() != 6) {
      continue;
    }
    family.push_back(deg_to_rad(pose_deg));
  }
  return family;
}

nlohmann::json pose_family_degrees_json(const std::vector<std::vector<double>>& family)
{
  nlohmann::json out = nlohmann::json::array();
  for (const auto& pose : family) {
    nlohmann::json pose_json = nlohmann::json::array();
    for (double value : pose) {
      pose_json.push_back(value * 180.0 / M_PI);
    }
    out.push_back(pose_json);
  }
  return out;
}

nlohmann::json pose_degrees_json(const std::vector<double>& pose)
{
  nlohmann::json out = nlohmann::json::array();
  for (double value : pose) {
    out.push_back(value * 180.0 / M_PI);
  }
  return out;
}

double shortest_angular_distance(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

std::string format_degrees(
  const std::vector<std::string>& names,
  const std::vector<double>& values)
{
  std::ostringstream oss;
  const size_t count = std::min(names.size(), values.size());
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) oss << ", ";
    oss << names[i] << "=" << values[i] * 180.0 / M_PI;
  }
  return oss.str();
}

Eigen::Quaterniond forward_x_orientation()
{
  return Eigen::Quaterniond(0.70710678, 0.0, 0.70710678, 0.0);
}

Eigen::Quaterniond top_suction_orientation()
{
  return Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);
}

Eigen::Quaterniond pitch_up_orientation(double pitch_up_rad)
{
  Eigen::Quaterniond q = Eigen::AngleAxisd(-pitch_up_rad, Eigen::Vector3d::UnitY()) * forward_x_orientation();
  q.normalize();
  return q;
}

geometry_msgs::msg::Pose make_pose(
  double x,
  double y,
  double z,
  const Eigen::Quaterniond& quat)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  Eigen::Quaterniond q = quat;
  q.normalize();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

geometry_msgs::msg::Pose make_identity_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Pose make_front_grasp_pose(
  const BoxSpec& box,
  double world_to_base_z)
{
  return make_pose(box.x, box.y, box.z - world_to_base_z, forward_x_orientation());
}

geometry_msgs::msg::Pose make_top_suction_pose(
  const BoxSpec& box,
  double world_to_base_z,
  double x_offset,
  double z_offset)
{
  return make_pose(
    box.x + x_offset,
    box.y,
    box.z + z_offset - world_to_base_z,
    top_suction_orientation());
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  tf.linear() = q.toRotationMatrix();
  return tf;
}

nlohmann::json pose_json(const geometry_msgs::msg::Pose& pose)
{
  return {
    {"position", {pose.position.x, pose.position.y, pose.position.z}},
    {"orientation_xyzw", {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}},
  };
}

nlohmann::json vector_json(const std::vector<double>& values)
{
  nlohmann::json out = nlohmann::json::array();
  for (double value : values) out.push_back(value);
  return out;
}

nlohmann::json names_values_json(
  const std::vector<std::string>& names,
  const std::vector<double>& values)
{
  nlohmann::json out = nlohmann::json::object();
  const size_t count = std::min(names.size(), values.size());
  for (size_t i = 0; i < count; ++i) out[names[i]] = values[i];
  return out;
}

double pose_position_error(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual)
{
  return (target.translation() - actual.translation()).norm();
}

double pose_orientation_error(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual)
{
  Eigen::AngleAxisd aa(target.linear().transpose() * actual.linear());
  return aa.angle();
}

}  // namespace alfa_robot::motion
