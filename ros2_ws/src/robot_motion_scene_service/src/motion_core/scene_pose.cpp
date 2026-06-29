#include "robot_motion_scene_service/motion_core/scene_pose.hpp"

namespace alfa_robot::motion
{

geometry_msgs::msg::Pose make_identity_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace alfa_robot::motion
