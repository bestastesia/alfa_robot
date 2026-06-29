#include "alfa_robot_moveit_config/trajectory_plan_utils.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
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
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
    </robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="one_joint_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::single_state_plan;

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

  return 0;
}
