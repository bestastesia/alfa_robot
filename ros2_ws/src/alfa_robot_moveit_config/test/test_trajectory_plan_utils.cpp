#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/duration.hpp>
#include <srdfdom/model.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>

#include <cassert>
#include <cmath>
#include <memory>

namespace
{

moveit::core::RobotModelPtr one_joint_model()
{
  const std::string urdf_xml =
    R"(<robot name="one_joint_robot">
      <link name="world"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="world"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14159265" upper="3.14159265" effort="1" velocity="1"/>
      </joint>
    </robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  const bool urdf_ok = urdf_model->initString(urdf_xml);
  assert(urdf_ok);
  auto srdf_model = std::make_shared<srdf::Model>();
  const bool srdf_ok = srdf_model->initString(*urdf_model, R"(<robot name="one_joint_robot"/>)");
  assert(srdf_ok);
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit::core::RobotModelPtr revolute_and_updown_model()
{
  const std::string urdf_xml =
    R"(<robot name="updown_robot">
      <link name="world"/>
      <link name="updown_link"/>
      <link name="link1"/>
      <joint name="updown" type="prismatic">
        <parent link="world"/>
        <child link="updown_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="joint1" type="revolute">
        <parent link="updown_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
    </robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  const bool urdf_ok = urdf_model->initString(urdf_xml);
  assert(urdf_ok);
  auto srdf_model = std::make_shared<srdf::Model>();
  const bool srdf_ok = srdf_model->initString(*urdf_model, R"(<robot name="updown_robot"/>)");
  assert(srdf_ok);
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::nearest_equivalent_joint_position;
  using alfa_robot::motion::single_state_plan;
  using alfa_robot::motion::tip_floor_updown_target;
  using alfa_robot::motion::retime_plan_by_max_joint_speed;

  const auto lowered_tip_floor = tip_floor_updown_target(0.3, 1.5, 1.3, 0.0, 0.7);
  assert(lowered_tip_floor.feasible);
  assert(!lowered_tip_floor.clamped);
  assert(std::abs(lowered_tip_floor.position - 0.1) < 1e-9);
  assert(std::abs(lowered_tip_floor.resulting_min_tip_z - 1.3) < 1e-9);

  const auto lower_limit_tip_floor = tip_floor_updown_target(0.1, 1.5, 1.3, 0.0, 0.7);
  assert(lower_limit_tip_floor.feasible);
  assert(lower_limit_tip_floor.clamped);
  assert(std::abs(lower_limit_tip_floor.position) < 1e-9);
  assert(lower_limit_tip_floor.resulting_min_tip_z > 1.3);

  const auto unreachable_tip_floor = tip_floor_updown_target(0.6, 1.0, 1.3, 0.0, 0.7);
  assert(!unreachable_tip_floor.feasible);
  assert(unreachable_tip_floor.clamped);
  assert(std::abs(unreachable_tip_floor.position - 0.7) < 1e-9);

  moveit::core::RobotState state(one_joint_model());
  state.setToDefaultValues();
  state.setVariablePosition("joint1", 0.42);

  const auto plan = single_state_plan(state, {"joint1", "missing_joint"}, 0.6);
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  assert(trajectory.joint_names.size() == 2);
  assert(trajectory.joint_names[0] == "joint1");
  assert(trajectory.joint_names[1] == "missing_joint");
  assert(trajectory.points.size() == 1);
  assert(trajectory.points[0].positions.size() == 2);
  assert(trajectory.points[0].positions[0] == 0.42);
  assert(trajectory.points[0].positions[1] == 0.0);
  assert(trajectory.points[0].time_from_start.sec == 0);
  assert(trajectory.points[0].time_from_start.nanosec == 600000000u);

  moveit::planning_interface::MoveGroupInterface::Plan slow_plan;
  auto& slow_trajectory = slow_plan.trajectory_.joint_trajectory;
  slow_trajectory.joint_names = {"joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  trajectory_msgs::msg::JointTrajectoryPoint mid_point;
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  start_point.positions = {0.0};
  mid_point.positions = {10.0 * M_PI / 180.0};
  end_point.positions = {30.0 * M_PI / 180.0};
  start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  mid_point.time_from_start = rclcpp::Duration::from_seconds(10.0);
  end_point.time_from_start = rclcpp::Duration::from_seconds(20.0);
  mid_point.velocities = {123.0};
  slow_trajectory.points = {start_point, mid_point, end_point};

  const auto retimed = retime_plan_by_max_joint_speed(
    slow_plan,
    state.getRobotModel(),
    20.0 * M_PI / 180.0);
  const auto& retimed_points = retimed.trajectory_.joint_trajectory.points;
  assert(retimed_points.size() == 16);
  assert(std::abs(rclcpp::Duration(retimed_points[0].time_from_start).seconds() - 0.0) < 1e-9);
  assert(std::abs(rclcpp::Duration(retimed_points[5].time_from_start).seconds() - 0.5) < 1e-9);
  assert(std::abs(rclcpp::Duration(retimed_points.back().time_from_start).seconds() - 1.5) < 1e-9);
  assert(std::abs(retimed_points[5].positions[0] - 10.0 * M_PI / 180.0) < 1e-9);
  assert(std::abs(retimed_points.back().positions[0] - 30.0 * M_PI / 180.0) < 1e-9);
  assert(retimed_points[1].velocities.empty());

  moveit::planning_interface::MoveGroupInterface::Plan updown_plan;
  auto& updown_trajectory = updown_plan.trajectory_.joint_trajectory;
  updown_trajectory.joint_names = {"updown", "joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint updown_start;
  trajectory_msgs::msg::JointTrajectoryPoint updown_end;
  updown_start.positions = {0.0, 0.0};
  updown_end.positions = {0.1, 0.0};
  updown_start.time_from_start = rclcpp::Duration::from_seconds(0.0);
  updown_end.time_from_start = rclcpp::Duration::from_seconds(0.1);
  updown_trajectory.points = {updown_start, updown_end};

  const auto updown_retimed = retime_plan_by_max_joint_speed(
    updown_plan,
    revolute_and_updown_model(),
    20.0 * M_PI / 180.0,
    10.0,
    0.05);
  const auto& updown_points = updown_retimed.trajectory_.joint_trajectory.points;
  assert(std::abs(rclcpp::Duration(updown_points.back().time_from_start).seconds() - 2.0) < 1e-9);
  assert(updown_points.size() == 21);

  const auto boundary_model = one_joint_model();
  const double lower = boundary_model->getVariableBounds("joint1").min_position_;
  const double upper = boundary_model->getVariableBounds("joint1").max_position_;
  const double continuous_boundary = nearest_equivalent_joint_position(
    boundary_model, "joint1", upper, lower);
  assert(std::abs(continuous_boundary - upper) < 1e-9);
  const double non_equivalent_wrap = nearest_equivalent_joint_position(
    boundary_model, "joint1", 3.0, -3.0);
  assert(std::abs(non_equivalent_wrap + 3.0) < 1e-9);

  return 0;
}
