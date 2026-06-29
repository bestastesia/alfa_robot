#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <string>
#include <vector>

namespace alfa_robot::motion
{

std::vector<double> deg_to_rad(const std::vector<double>& degrees);

std::vector<double> parse_degrees_list(std::string value);

std::vector<std::vector<double>> parse_pose_family_degrees(const std::string& value);

nlohmann::json pose_family_degrees_json(const std::vector<std::vector<double>>& family);

nlohmann::json pose_degrees_json(const std::vector<double>& pose);

double shortest_angular_distance(double a, double b);

std::string format_degrees(
  const std::vector<std::string>& names,
  const std::vector<double>& values);

Eigen::Quaterniond forward_x_orientation();

Eigen::Quaterniond top_suction_orientation();

Eigen::Quaterniond pitch_up_orientation(double pitch_up_rad);

geometry_msgs::msg::Pose make_pose(
  double x,
  double y,
  double z,
  const Eigen::Quaterniond& quat);

geometry_msgs::msg::Pose make_identity_pose(double x, double y, double z);

geometry_msgs::msg::Pose make_front_grasp_pose(
  const BoxSpec& box,
  double world_to_base_z);

geometry_msgs::msg::Pose make_top_suction_pose(
  const BoxSpec& box,
  double world_to_base_z,
  double x_offset,
  double z_offset);

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose);

nlohmann::json pose_json(const geometry_msgs::msg::Pose& pose);

nlohmann::json vector_json(const std::vector<double>& values);

nlohmann::json names_values_json(
  const std::vector<std::string>& names,
  const std::vector<double>& values);

double pose_position_error(
  const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& actual);

double pose_orientation_error(
  const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& actual);

}  // namespace alfa_robot::motion
