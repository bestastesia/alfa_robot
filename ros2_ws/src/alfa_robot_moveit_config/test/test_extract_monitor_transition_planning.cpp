#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <memory>
#include <string>

namespace
{

moveit::core::RobotModelPtr empty_model()
{
  const std::string urdf_xml =
    R"(<robot name="empty_robot"><link name="world"/></robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="empty_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit::planning_interface::MoveGroupInterface::Plan tagged_plan(double planning_time)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.planning_time_ = planning_time;
  return plan;
}

}  // namespace

int main()
{
  using alfa_robot::motion::ExtractMonitorTransitionPlanner;

  const auto model = empty_model();
  moveit::core::RobotState start(model);
  moveit::core::RobotState goal(model);

  int make_calls = 0;
  int densify_calls = 0;
  int direct_calls = 0;
  int shortcut_calls = 0;

  ExtractMonitorTransitionPlanner planner;
  planner.make_interpolated_plan = [&](const auto&, const auto&, double duration_s) {
    ++make_calls;
    return tagged_plan(duration_s);
  };
  planner.densify_plan = [&](const auto& plan) {
    ++densify_calls;
    auto out = plan;
    out.planning_time_ += 10.0;
    return out;
  };
  planner.validate_plan = [](const auto&, const auto&, std::string* reason) {
    if (reason) *reason = "";
    return true;
  };
  planner.direct_plan = [&](const auto&, const auto&, auto*, std::string*) {
    ++direct_calls;
    return false;
  };
  planner.shortcut_plan = [&](const auto& plan, const auto&, std::string*) {
    ++shortcut_calls;
    return plan;
  };

  const auto interpolation = planner.plan(start, goal);
  assert(interpolation.valid);
  assert(interpolation.method == "joint_interpolation");
  assert(make_calls == 1);
  assert(densify_calls == 1);
  assert(direct_calls == 0);
  assert(shortcut_calls == 0);

  bool first_validation = true;
  planner.validate_plan = [&](const auto&, const auto&, std::string* reason) {
    if (first_validation) {
      first_validation = false;
      if (reason) *reason = "interpolation_collision";
      return false;
    }
    if (reason) *reason = "";
    return true;
  };
  planner.direct_plan = [&](const auto&, const auto&, auto* plan, std::string*) {
    ++direct_calls;
    if (plan) *plan = tagged_plan(20.0);
    return true;
  };
  planner.shortcut_plan = [&](const auto& plan, const auto&, std::string* reason) {
    ++shortcut_calls;
    if (reason) *reason = "shortcut 5 -> 2 points";
    auto out = plan;
    out.planning_time_ += 1.0;
    return out;
  };

  make_calls = 0;
  densify_calls = 0;
  direct_calls = 0;
  shortcut_calls = 0;
  const auto fallback = planner.plan(start, goal);
  assert(fallback.valid);
  assert(fallback.method == "rrt");
  assert(fallback.failure_reason == "shortcut 5 -> 2 points");
  assert(make_calls == 1);
  assert(densify_calls == 2);
  assert(direct_calls == 1);
  assert(shortcut_calls == 1);

  first_validation = true;
  planner.direct_plan = [&](const auto&, const auto&, auto*, std::string* reason) {
    if (reason) *reason = "direct_failed";
    return false;
  };
  const auto failed = planner.plan(start, goal);
  assert(!failed.valid);
  assert(failed.method == "rrt");
  assert(failed.failure_reason == "direct_failed");

  return 0;
}
