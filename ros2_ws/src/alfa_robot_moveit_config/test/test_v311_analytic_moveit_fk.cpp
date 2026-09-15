#include <alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace
{

using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
using alfa_robot::analytic_ik::V3RedundantArmModel;
using alfa_robot::analytic_ik::V3RedundantIkRequest;

void require(bool condition, const std::string& message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::string renderInstalledUrdf()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("alfa_robot_description") +
    "/urdf/alfa_robot.urdf.xacro";
  const std::string command = "xacro '" + path + "' end_effector:=suction";
  std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
  require(pipe != nullptr, "failed to launch xacro");
  std::string output;
  std::array<char, 8192> buffer{};
  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    output += buffer.data();
  }
  require(!output.empty(), "xacro returned an empty URDF");
  return output;
}

moveit::core::RobotModelPtr makeRobotModel()
{
  auto urdf_model = std::make_shared<urdf::Model>();
  require(urdf_model->initString(renderInstalledUrdf()), "failed to parse installed V3.1.1 URDF");
  auto srdf_model = std::make_shared<srdf::Model>();
  require(
    srdf_model->initString(*urdf_model, "<robot name='alfa_robot'/>"),
    "failed to initialize empty SRDF");
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

double orientationError(const Eigen::Matrix3d& expected, const Eigen::Matrix3d& actual)
{
  return std::abs(Eigen::AngleAxisd(expected.transpose() * actual).angle());
}

void setArm(
  moveit::core::RobotState& state,
  const std::string& side,
  const std::array<double, 7>& joints)
{
  for (size_t index = 0; index < joints.size(); ++index) {
    state.setVariablePosition(
      side + "_joint" + std::to_string(index + 1U), joints[index]);
  }
  state.update(true);
}

Eigen::Isometry3d moveitToolInCarriage(
  const moveit::core::RobotState& state,
  const std::string& side)
{
  return state.getGlobalLinkTransform("arm_carriage").inverse() *
         state.getGlobalLinkTransform(side + "_tool0");
}

void checkSide(
  const moveit::core::RobotModelPtr& robot_model,
  const std::string& side,
  V3RedundantArmModel model_kind,
  std::mt19937& generator)
{
  V3RedundantArmAnalyticIk solver(model_kind);
  const auto lower = solver.jointLowerLimits();
  const auto upper = solver.jointUpperLimits();
  moveit::core::RobotState state(robot_model);
  std::array<std::uniform_real_distribution<double>, 7> distributions = {
    std::uniform_real_distribution<double>(0.75 * lower[0], 0.75 * upper[0]),
    std::uniform_real_distribution<double>(0.75 * lower[1], 0.75 * upper[1]),
    std::uniform_real_distribution<double>(0.75 * lower[2], 0.75 * upper[2]),
    std::uniform_real_distribution<double>(0.75 * lower[3], 0.75 * upper[3]),
    std::uniform_real_distribution<double>(0.75 * lower[4], 0.75 * upper[4]),
    std::uniform_real_distribution<double>(0.75 * lower[5], 0.75 * upper[5]),
    std::uniform_real_distribution<double>(0.75 * lower[6], 0.75 * upper[6]),
  };

  double maximum_fk_position_error = 0.0;
  double maximum_fk_orientation_error = 0.0;
  double maximum_ik_position_error = 0.0;
  double maximum_ik_orientation_error = 0.0;
  double solve_microseconds = 0.0;
  constexpr size_t kSampleCount = 256;
  const Eigen::Vector3d shoulder = solver.modelShoulderCenterInArmBase();

  for (size_t sample = 0; sample < kSampleCount; ++sample) {
    std::array<double, 7> joints{};
    for (size_t index = 0; index < joints.size(); ++index) {
      joints[index] = distributions[index](generator);
    }
    state.setToDefaultValues();
    state.setVariablePosition("updown", -0.35);
    setArm(state, side, joints);

    const Eigen::Isometry3d expected = moveitToolInCarriage(state, side);
    const Eigen::Isometry3d analytic = solver.forwardInArmBase(joints);
    const double fk_position_error =
      (expected.translation() - analytic.translation()).norm();
    const double fk_orientation_error =
      orientationError(expected.linear(), analytic.linear());
    maximum_fk_position_error = std::max(maximum_fk_position_error, fk_position_error);
    maximum_fk_orientation_error = std::max(maximum_fk_orientation_error, fk_orientation_error);
    require(fk_position_error < 2e-7, side + " V3.1.1 analytic FK position mismatch");
    require(fk_orientation_error < 2e-7, side + " V3.1.1 analytic FK orientation mismatch");

    V3RedundantIkRequest request;
    request.target_in_arm_base = expected;
    request.swivel_angle = solver.swivelAngle(joints);
    request.seed = joints;
    request.position_tolerance = 2e-7;
    request.orientation_tolerance = 2e-7;
    const auto start = std::chrono::steady_clock::now();
    const auto solutions = solver.solveInArmBase(request);
    solve_microseconds += std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - start).count();
    require(!solutions.empty(), side + " V3.1.1 MoveIt FK target returned no analytic IK");

    setArm(state, side, solutions.front().joints);
    const Eigen::Isometry3d solved = moveitToolInCarriage(state, side);
    const double ik_position_error =
      (expected.translation() - solved.translation()).norm();
    const double ik_orientation_error =
      orientationError(expected.linear(), solved.linear());
    maximum_ik_position_error = std::max(maximum_ik_position_error, ik_position_error);
    maximum_ik_orientation_error = std::max(maximum_ik_orientation_error, ik_orientation_error);
    require(ik_position_error < 2e-7, side + " V3.1.1 IK MoveIt position residual");
    require(ik_orientation_error < 2e-7, side + " V3.1.1 IK MoveIt orientation residual");
  }

  std::cout << std::setprecision(12) << side
            << " samples=" << kSampleCount
            << " shoulder_m=[" << shoulder.x() << ',' << shoulder.y() << ',' << shoulder.z() << ']'
            << " arm_length_m=" << solver.modelArmLength()
            << " max_fk_position_m=" << maximum_fk_position_error
            << " max_fk_orientation_rad=" << maximum_fk_orientation_error
            << " max_ik_position_m=" << maximum_ik_position_error
            << " max_ik_orientation_rad=" << maximum_ik_orientation_error
            << " average_ik_us=" << solve_microseconds / kSampleCount
            << '\n';
}

}  // namespace

int main()
{
  const auto model = makeRobotModel();
  std::mt19937 generator(20260915);
  checkSide(model, "left", V3RedundantArmModel::V311Left, generator);
  checkSide(model, "right", V3RedundantArmModel::V311Right, generator);
  return 0;
}
