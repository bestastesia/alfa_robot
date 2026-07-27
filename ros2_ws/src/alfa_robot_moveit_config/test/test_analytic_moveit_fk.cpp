#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <Eigen/Geometry>

#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::string make_test_urdf()
{
  std::ostringstream urdf;
  urdf << R"(
<robot name="analytic_fk_robot">
  <link name="base_link"/>
  <link name="pitch"/>
  <joint name="pitch" type="revolute">
    <parent link="base_link"/>
    <child link="pitch"/>
    <origin xyz="0.25918 -0.0545 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.26" effort="1" velocity="1"/>
  </joint>
  <link name="turn"/>
  <joint name="turn" type="continuous">
    <parent link="pitch"/>
    <child link="turn"/>
    <origin xyz="-0.18784 0.0545 0.135" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="updown"/>
  <joint name="updown" type="prismatic">
    <parent link="turn"/>
    <child link="updown"/>
    <origin xyz="-0.081156 0 0.207" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.7" effort="1" velocity="1"/>
  </joint>
)";

  for (const std::string side : {"left", "right"}) {
    const double sign = side == "left" ? 1.0 : -1.0;
    urdf << "<link name=\"" << side << "_arm_base\"/>"
         << "<joint name=\"" << side << "_arm_mount\" type=\"fixed\">"
         << "<parent link=\"updown\"/>"
         << "<child link=\"" << side << "_arm_base\"/>"
         << "<origin xyz=\"0.095 " << sign * 0.26 << " 0.2\" rpy=\"0 0 0\"/>"
         << "</joint>\n";

    const std::vector<std::string> origins = {
      "0 0 0",
      std::string("0 ") + std::to_string(sign * 0.0825) + " 0.058",
      std::string("0 -0.4 ") + std::to_string(sign * 0.008),
      std::string("0 -0.34 ") + std::to_string(sign * 0.036),
      std::string("0 0.076 ") + std::to_string(sign * 0.058),
      "0.0825 0 -0.058",
    };
    const std::vector<std::string> rpys = {
      "0 0 0",
      "-1.5708 0 0",
      "0 0 0",
      "0 0 3.14159265",
      "-1.5708 0 3.14159265",
      "0 1.5708 0",
    };

    std::string parent = side + "_arm_base";
    for (int i = 1; i <= 6; ++i) {
      const std::string link = side + "_joint" + std::to_string(i);
      urdf << "<link name=\"" << link << "\"/>"
           << "<joint name=\"" << link << "\" type=\"revolute\">"
           << "<parent link=\"" << parent << "\"/>"
           << "<child link=\"" << link << "\"/>"
           << "<origin xyz=\"" << origins.at(i - 1) << "\" rpy=\"" << rpys.at(i - 1) << "\"/>"
           << "<axis xyz=\"0 0 1\"/>"
           << "<limit lower=\"-3.14159265\" upper=\"3.14159265\" effort=\"1\" velocity=\"1\"/>"
           << "</joint>\n";
      parent = link;
    }

    urdf << "<link name=\"" << side << "_tool0\"/>"
         << "<joint name=\"" << side << "_tool0_fixed\" type=\"fixed\">"
         << "<parent link=\"" << parent << "\"/>"
         << "<child link=\"" << side << "_tool0\"/>"
         << "<origin xyz=\"0 0 0.259\" rpy=\"0 0 0\"/>"
         << "</joint>\n";
  }

  urdf << "</robot>";
  return urdf.str();
}

moveit::core::RobotModelPtr make_robot_model()
{
  auto urdf_model = std::make_shared<urdf::Model>();
  const bool urdf_ok = urdf_model->initString(make_test_urdf());
  assert(urdf_ok);
  auto srdf_model = std::make_shared<srdf::Model>();
  const bool srdf_ok = srdf_model->initString(*urdf_model, R"(<robot name="analytic_fk_robot"/>)");
  assert(srdf_ok);
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

void assert_fk_matches(
  const moveit::core::RobotModelPtr& model,
  alfa_robot::analytic_ik::ArmSide side,
  const std::array<double, 6>& joints,
  double updown)
{
  const std::string side_name = alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk::sideName(side);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePosition("updown", updown);
  for (size_t i = 0; i < joints.size(); ++i) {
    state.setVariablePosition(side_name + "_joint" + std::to_string(i + 1), joints[i]);
  }
  state.update();

  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk solver;
  const Eigen::Isometry3d analytic =
    alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk::baseLinkToArmBase(side, updown) *
    solver.forwardInArmBase(side, joints);
  const Eigen::Isometry3d moveit =
    state.getGlobalLinkTransform("base_link").inverse() *
    state.getGlobalLinkTransform(side_name + "_tool0");

  assert((analytic.translation() - moveit.translation()).norm() < 1e-4);
  assert((analytic.linear() - moveit.linear()).norm() < 1e-4);
}

}  // namespace

int main()
{
  const auto model = make_robot_model();
  const std::vector<std::array<double, 6>> samples = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.1, 1.1, 1.0, -2.0, 0.1, 0.0},
    {0.75, -0.4, 1.5, 0.6, -1.2, 2.1},
    {-1.2, 1.8, -0.9, 0.3, 0.8, -1.7},
  };
  for (const auto& joints : samples) {
    assert_fk_matches(model, alfa_robot::analytic_ik::ArmSide::Left, joints, 0.3);
    assert_fk_matches(model, alfa_robot::analytic_ik::ArmSide::Right, joints, 0.3);
  }
  return 0;
}
