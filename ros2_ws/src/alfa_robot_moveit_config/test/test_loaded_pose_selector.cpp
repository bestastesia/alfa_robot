#include "alfa_robot_moveit_config/loaded_pose_planning.hpp"
#include "alfa_robot_moveit_config/extract_planning_pipeline.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <string>

namespace
{

moveit::core::RobotModelPtr loaded_pose_test_model()
{
  std::string urdf_xml = R"(
<robot name="loaded_pose_robot">
  <link name="base_link"/>
  <link name="updown_link"/>
  <joint name="updown" type="prismatic">
    <parent link="base_link"/>
    <child link="updown_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
)";
  for (const auto side : {"left", "right"}) {
    std::string parent = "updown_link";
    for (int i = 1; i <= 6; ++i) {
      const std::string link = std::string(side) + "_v5_link" + std::to_string(i);
      urdf_xml +=
        "  <link name=\"" + link + "\"/>\n"
        "  <joint name=\"" + std::string(side) + "_v5_joint" + std::to_string(i) + "\" type=\"revolute\">\n"
        "    <parent link=\"" + parent + "\"/>\n"
        "    <child link=\"" + link + "\"/>\n"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
        "    <axis xyz=\"0 0 1\"/>\n"
        "    <limit lower=\"-3.14159\" upper=\"3.14159\" effort=\"1\" velocity=\"1\"/>\n"
        "  </joint>\n";
      parent = link;
    }
  }
  urdf_xml += "</robot>";

  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="loaded_pose_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::ExtractRolloutTiming;
  using alfa_robot::motion::LoadedPoseSelector;
  using alfa_robot::motion::LoadedPoseSelectorConfig;

  LoadedPoseSelectorConfig config;
  config.left_pose_family = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
  };
  config.right_pose_family = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
  };
  config.target_updown = 0.3;
  LoadedPoseSelector selector(config);

  const auto model = loaded_pose_test_model();
  auto state = std::make_shared<moveit::core::RobotState>(model);
  state->setToDefaultValues();
  state->setVariablePosition("updown", 0.8);
  for (int i = 1; i <= 6; ++i) {
    state->setVariablePosition("left_v5_joint" + std::to_string(i), 0.9);
    state->setVariablePosition("right_v5_joint" + std::to_string(i), -0.8);
  }
  state->update();

  ExtractRolloutTiming timing;
  selector.fillTimingDistanceMetrics(timing);
  assert(timing.loaded_pose_distance_sum == 0.0);
  assert(timing.loaded_pose_distance_l2 == 0.0);
  assert(timing.loaded_pose_max_joint_delta == 0.0);

  timing.final_state = state;
  selector.fillTimingDistanceMetrics(timing);
  assert(timing.selected_left_loaded_pose_index == 1);
  assert(timing.selected_right_loaded_pose_index == 1);
  assert(timing.selected_left_loaded_pose_distance > 0.0);
  assert(timing.selected_right_loaded_pose_distance > 0.0);
  assert(timing.loaded_pose_distance_sum > 0.0);
  assert(timing.loaded_pose_distance_l2 > 0.0);
  assert(std::abs(timing.loaded_pose_max_joint_delta - 0.2) < 1e-9);

  alfa_robot::motion::LoadedPoseSelection selection;
  const auto goal = selector.makeGoalState(*state, &selection);
  assert(selection.left_index == timing.selected_left_loaded_pose_index);
  assert(selection.right_index == timing.selected_right_loaded_pose_index);
  assert(std::abs(goal.getVariablePosition("updown") - 0.3) < 1e-9);
  assert(std::abs(goal.getVariablePosition("left_v5_joint1") - 1.0) < 1e-9);
  assert(std::abs(goal.getVariablePosition("right_v5_joint1") + 1.0) < 1e-9);

  return 0;
}
