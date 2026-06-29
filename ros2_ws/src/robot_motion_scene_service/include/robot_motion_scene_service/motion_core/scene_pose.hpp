#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <array>

namespace alfa_robot::motion
{

geometry_msgs::msg::Pose make_identity_pose(double x, double y, double z);

}  // namespace alfa_robot::motion
