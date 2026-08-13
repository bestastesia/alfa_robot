#include "alfa_robot_moveit_config/planning_diagnostics.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <cmath>
#include <memory>

namespace
{

moveit::core::RobotModelPtr updown_model()
{
  const std::string urdf_xml =
    R"(<robot name="updown_robot">
      <link name="world"/>
      <link name="updown_link"/>
      <joint name="updown" type="prismatic">
        <parent link="world"/>
        <child link="updown_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="0.7" effort="1" velocity="1"/>
      </joint>
    </robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="updown_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::clamp_variable_to_bounds_if_near;

  moveit::core::RobotState state(updown_model());
  state.setToDefaultValues();

  state.setVariablePosition("updown", -3.4e-5);
  assert(clamp_variable_to_bounds_if_near(&state, "updown", 1.0e-4));
  assert(std::abs(state.getVariablePosition("updown")) < 1.0e-12);

  state.setVariablePosition("updown", 0.700034);
  assert(clamp_variable_to_bounds_if_near(&state, "updown", 1.0e-4));
  assert(std::abs(state.getVariablePosition("updown") - 0.7) < 1.0e-12);

  state.setVariablePosition("updown", -1.1e-4);
  assert(!clamp_variable_to_bounds_if_near(&state, "updown", 1.0e-4));
  assert(std::abs(state.getVariablePosition("updown") + 1.1e-4) < 1.0e-12);

  state.setVariablePosition("updown", 0.25);
  assert(!clamp_variable_to_bounds_if_near(&state, "updown", 1.0e-4));
  assert(std::abs(state.getVariablePosition("updown") - 0.25) < 1.0e-12);

  return 0;
}
