#include "alfa_robot_moveit_config/curobo_segment_client.hpp"
#include "alfa_robot_moveit_config/extract_monitor_transition_planning.hpp"
#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

using alfa_robot::motion::AttachedBoxSpec;
using alfa_robot::motion::CuroboSegmentClient;
using alfa_robot::motion::CuroboSegmentClientConfig;
using alfa_robot::motion::ExtractMonitorLocalPlanMetrics;
using alfa_robot::motion::ExtractMonitorTransitionPlanner;

using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

moveit::core::RobotModelPtr model_13dof()
{
  const auto names = alfa_robot::motion::dual_arm_with_updown_joint_names();
  std::ostringstream urdf;
  urdf << "<robot name='segment_client_test'><link name='link0'/>";
  for (size_t i = 0; i < names.size(); ++i) {
    urdf << "<link name='link" << (i + 1) << "'/>"
         << "<joint name='" << names[i] << "' type='prismatic'>"
         << "<parent link='link" << i << "'/><child link='link" << (i + 1) << "'/>"
         << "<axis xyz='1 0 0'/><limit lower='-2' upper='2' effort='1' velocity='1'/>"
         << "</joint>";
  }
  urdf << "</robot>";
  std::ostringstream srdf;
  srdf << "<robot name='segment_client_test'><group name='dual_arm_with_base'>";
  for (const auto& name : names) srdf << "<joint name='" << name << "'/>";
  srdf << "</group></robot>";

  auto urdf_model = std::make_shared<urdf::Model>();
  const bool urdf_ok = urdf_model->initString(urdf.str());
  if (!urdf_ok) throw std::runtime_error("failed to initialize test URDF");
  auto srdf_model = std::make_shared<srdf::Model>();
  const bool srdf_ok = srdf_model->initString(*urdf_model, srdf.str());
  if (!srdf_ok) throw std::runtime_error("failed to initialize test SRDF");
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit::core::RobotState make_state(
  const moveit::core::RobotModelPtr& model, double value)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const auto& name : alfa_robot::motion::dual_arm_with_updown_joint_names()) {
    state.setVariablePosition(name, value);
  }
  state.update(true);
  return state;
}

std::vector<AttachedBoxSpec> boxes()
{
  return {
    {"carried_left_box_1", "left_tool0", {0.0, 0.0, 0.15}, {0.4, 0.4, 0.3}},
    {"carried_right_box_3", "right_tool0", {0.0, 0.0, 0.15}, {0.4, 0.4, 0.3}},
  };
}

Plan joint_plan(
  const moveit::core::RobotState& start,
  const moveit::core::RobotState& goal,
  size_t point_count,
  bool add_detour)
{
  Plan plan;
  auto& trajectory = plan.trajectory_.joint_trajectory;
  trajectory.joint_names = alfa_robot::motion::dual_arm_with_updown_joint_names();
  for (size_t point_index = 0; point_index < point_count; ++point_index) {
    const double ratio = static_cast<double>(point_index) /
      static_cast<double>(point_count - 1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.02 * point_index);
    for (const auto& name : trajectory.joint_names) {
      double value = start.getVariablePosition(name) +
        ratio * (goal.getVariablePosition(name) - start.getVariablePosition(name));
      if (add_detour && point_index == point_count / 2 && name == "updown") {
        value += 0.3;
      }
      point.positions.push_back(value);
    }
    trajectory.points.push_back(std::move(point));
  }
  return plan;
}

bool crosses_mock_blocked_band(const Plan& plan)
{
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  const auto it = std::find(
    trajectory.joint_names.begin(), trajectory.joint_names.end(), "updown");
  if (it == trajectory.joint_names.end()) return false;
  const size_t index = static_cast<size_t>(
    std::distance(trajectory.joint_names.begin(), it));
  for (size_t point_index = 1; point_index < trajectory.points.size(); ++point_index) {
    const auto& previous = trajectory.points[point_index - 1].positions;
    const auto& current = trajectory.points[point_index].positions;
    if (index >= previous.size() || index >= current.size()) return true;
    if (std::min(previous[index], current[index]) <= 0.05 &&
        std::max(previous[index], current[index]) >= 0.05) {
      return true;
    }
  }
  return false;
}

ExtractMonitorTransitionPlanner integration_planner()
{
  ExtractMonitorTransitionPlanner planner;
  planner.make_interpolated_plan = [](const auto& start, const auto& goal, double) {
    return joint_plan(start, goal, 13, false);
  };
  planner.densify_plan = [](const auto& plan) { return plan; };
  planner.validate_plan = [](const auto& plan, const auto&, std::string* reason) {
    const auto& trajectory = plan.trajectory_.joint_trajectory;
    const auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), "updown");
    const size_t index = static_cast<size_t>(std::distance(trajectory.joint_names.begin(), it));
    const bool detoured = it != trajectory.joint_names.end() &&
      std::any_of(trajectory.points.begin(), trajectory.points.end(), [index](const auto& point) {
        return index < point.positions.size() && point.positions[index] > 0.2;
      });
    const bool valid = detoured || !crosses_mock_blocked_band(plan);
    if (reason) *reason = valid ? "" : "mock_fcl_blocked_shortcut";
    return valid;
  };
  planner.direct_plan = [](const auto& start, const auto& goal, auto* plan, std::string*) {
    *plan = joint_plan(start, goal, 3, true);
    return true;
  };
  planner.local_window_points = 8;
  planner.local_max_segments = 4;
  planner.local_max_calls = 8;
  planner.fallback_to_direct = true;
  return planner;
}

bool wait_ready(const CuroboSegmentClient& client)
{
  for (size_t i = 0; i < 100; ++i) {
    if (client.serviceReady()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

void fill_success_response(
  const CuroboSegmentClient::Service::Request& request,
  CuroboSegmentClient::Service::Response* response)
{
  response->success = true;
  response->message = "ok";
  response->planner_method = "mock_curobo";
  response->total_time_ms = 4.0;
  response->solve_time_ms = 2.0;
  response->queue_time_ms = 0.25;
  response->endpoint_check_time_ms = 0.5;
  response->graph_time_ms = 0.75;
  response->trajopt_time_ms = 1.0;
  response->interpolation_time_ms = 0.2;
  response->trajectory.joint_names = request.start_state.name;
  std::reverse(
    response->trajectory.joint_names.begin(), response->trajectory.joint_names.end());
  const auto value_for = [](const sensor_msgs::msg::JointState& state, const std::string& name) {
    const auto it = std::find(state.name.begin(), state.name.end(), name);
    assert(it != state.name.end());
    return state.position[static_cast<size_t>(std::distance(state.name.begin(), it))];
  };
  for (size_t point_index = 0; point_index < 3; ++point_index) {
    const double ratio = 0.5 * static_cast<double>(point_index);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.02 * point_index);
    for (const auto& name : response->trajectory.joint_names) {
      const double start = value_for(request.start_state, name);
      const double goal = value_for(request.goal_state, name);
      double value = start + (goal - start) * ratio;
      if (point_index == 1 && name == "updown") value += 0.3;
      point.positions.push_back(value);
    }
    response->trajectory.points.push_back(point);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto model = model_13dof();
  const auto start = make_state(model, 0.0);
  const auto goal = make_state(model, 0.1);

  auto client_node = std::make_shared<rclcpp::Node>("curobo_segment_client_test_client");
  auto server_node = std::make_shared<rclcpp::Node>("curobo_segment_client_test_server");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(client_node);
  executor.add_node(server_node);
  std::thread spin([&executor]() { executor.spin(); });

  auto success_service = server_node->create_service<CuroboSegmentClient::Service>(
    "/test/plan_joint_segment",
    [](const std::shared_ptr<CuroboSegmentClient::Service::Request> request,
       std::shared_ptr<CuroboSegmentClient::Service::Response> response) {
      assert(request->attached_boxes.size() == 2);
      fill_success_response(*request, response.get());
    });
  CuroboSegmentClientConfig config;
  config.service_name = "/test/plan_joint_segment";
  config.timeout_s = 0.2;
  CuroboSegmentClient client(*client_node, config);
  assert(wait_ready(client));

  CuroboSegmentClient::Plan plan;
  ExtractMonitorLocalPlanMetrics metrics;
  std::string reason;
  assert(client.plan("mock", start, goal, boxes(), &plan, &metrics, &reason));
  assert(reason.empty());
  assert(metrics.planner_method == "mock_curobo");
  assert(metrics.backend_solve_ms == 2.0);
  assert(metrics.backend_queue_ms == 0.25);
  assert(metrics.backend_endpoint_check_ms == 0.5);
  assert(metrics.backend_graph_ms == 0.75);
  assert(metrics.backend_trajopt_ms == 1.0);
  assert(metrics.backend_interpolation_ms == 0.2);
  assert(plan.trajectory_.joint_trajectory.joint_names ==
    alfa_robot::motion::dual_arm_with_updown_joint_names());
  assert(plan.trajectory_.joint_trajectory.points.size() == 3);

  // Full transition planner integration: FCL rejects the shortcut, the C++
  // adapter calls the ROS mock service, stitches the patch, and passes final FCL.
  auto transition = integration_planner();
  transition.preferred_local_plan =
    [&client](const auto& local_start, const auto& local_goal, auto* local_plan,
              auto* local_metrics, std::string* local_reason) {
      return client.plan(
        "integration", local_start, local_goal, boxes(),
        local_plan, local_metrics, local_reason);
    };
  const auto integrated = transition.plan(start, goal);
  assert(integrated.valid);
  assert(integrated.method == "shortcut_local_curobo");
  assert(integrated.local_preferred_call_count > 0);
  assert(integrated.local_preferred_segment_count > 0);
  assert(integrated.final_fcl_valid);
  const auto& integrated_trajectory = integrated.plan.trajectory_.joint_trajectory;
  assert(integrated_trajectory.points.size() >= 2);
  for (size_t point_index = 1; point_index < integrated_trajectory.points.size(); ++point_index) {
    const double previous_time = rclcpp::Duration(
      integrated_trajectory.points[point_index - 1].time_from_start).seconds();
    const double current_time = rclcpp::Duration(
      integrated_trajectory.points[point_index].time_from_start).seconds();
    assert(current_time > previous_time);
    assert(integrated_trajectory.points[point_index].positions !=
      integrated_trajectory.points[point_index - 1].positions);
  }
  for (size_t joint_index = 0;
       joint_index < integrated_trajectory.joint_names.size(); ++joint_index) {
    const auto& name = integrated_trajectory.joint_names[joint_index];
    assert(std::abs(integrated_trajectory.points.front().positions[joint_index] -
      start.getVariablePosition(name)) < 1e-9);
    assert(std::abs(integrated_trajectory.points.back().positions[joint_index] -
      goal.getVariablePosition(name)) < 1e-9);
  }

  // A collision-free shortcut must never call the optional backend or fallback.
  auto clear_shortcut = integration_planner();
  int unexpected_curobo_calls = 0;
  int unexpected_rrt_calls = 0;
  clear_shortcut.validate_plan = [](const auto&, const auto&, std::string* validation_reason) {
    if (validation_reason) validation_reason->clear();
    return true;
  };
  clear_shortcut.preferred_local_plan = [&unexpected_curobo_calls](
    const auto&, const auto&, auto*, auto*, std::string*) {
    ++unexpected_curobo_calls;
    return false;
  };
  clear_shortcut.direct_plan = [&unexpected_rrt_calls](
    const auto&, const auto&, auto*, std::string*) {
    ++unexpected_rrt_calls;
    return false;
  };
  const auto clear_result = clear_shortcut.plan(start, goal);
  assert(clear_result.valid);
  assert(clear_result.method == "joint_interpolation");
  assert(unexpected_curobo_calls == 0);
  assert(unexpected_rrt_calls == 0);

  // A cuRobo response rejected by the production validator falls back on the
  // same local window to the retained local-RRT backend.
  auto fcl_rejected = integration_planner();
  fcl_rejected.preferred_local_plan = [](const auto& local_start, const auto& local_goal,
                                         auto* local_plan, auto*, std::string*) {
    *local_plan = joint_plan(local_start, local_goal, 3, false);
    return true;
  };
  const auto fcl_fallback = fcl_rejected.plan(start, goal);
  assert(fcl_fallback.valid);
  assert(fcl_fallback.method == "shortcut_local_rrt");
  assert(fcl_fallback.local_fallback_used);
  assert(fcl_fallback.local_preferred_segment_count == 0);
  assert(fcl_fallback.local_fallback_segment_count > 0);
  assert(fcl_fallback.failure_reason.find("local_curobo_fcl_rejected") !=
    std::string::npos);

  // Backend call limits are hard bounds and do not loop indefinitely.
  auto call_limited = integration_planner();
  call_limited.local_max_calls = 1;
  call_limited.fallback_to_direct = false;
  int limited_calls = 0;
  call_limited.preferred_local_plan = [&limited_calls](
    const auto&, const auto&, auto*, auto*, std::string* local_reason) {
    ++limited_calls;
    if (local_reason) {
      *local_reason =
        "local_curobo_service_failed:curobo_start_state_in_collision_or_bounds";
    }
    return false;
  };
  const auto limited_result = call_limited.plan(start, goal);
  assert(!limited_result.valid);
  assert(limited_calls == 1);
  assert(limited_result.local_preferred_call_count == 1);

  // A conservative cuRobo sphere model may reject the FCL-safe patch boundary.
  // The planner backs up within the configured bound before using local-RRT.
  auto boundary_retry = integration_planner();
  boundary_retry.local_boundary_backoff_points = 2;
  int boundary_calls = 0;
  boundary_retry.preferred_local_plan =
    [&client, &boundary_calls](const auto& local_start, const auto& local_goal,
                              auto* local_plan, auto* local_metrics,
                              std::string* local_reason) {
      ++boundary_calls;
      if (boundary_calls == 1) {
        if (local_reason) {
          *local_reason =
            "local_curobo_service_failed:curobo_start_state_in_collision_or_bounds";
        }
        return false;
      }
      return client.plan(
        "boundary_retry", local_start, local_goal, boxes(),
        local_plan, local_metrics, local_reason);
    };
  const auto boundary_repaired = boundary_retry.plan(start, goal);
  assert(boundary_repaired.valid);
  assert(boundary_repaired.method == "shortcut_local_curobo");
  assert(boundary_calls >= 2);
  assert(boundary_repaired.local_fallback_segment_count == 0);

  // Backing up a rejected start may reveal that the original patch goal is
  // also conservative-model invalid.  Continue from the backed-up start and
  // advance the patch goal instead of falling back prematurely.
  auto combined_boundary_retry = integration_planner();
  combined_boundary_retry.local_window_points = 1;
  combined_boundary_retry.local_boundary_backoff_points = 2;
  int combined_boundary_calls = 0;
  combined_boundary_retry.preferred_local_plan =
    [&client, &combined_boundary_calls](const auto& local_start, const auto& local_goal,
                                        auto* local_plan, auto* local_metrics,
                                        std::string* local_reason) {
      ++combined_boundary_calls;
      if (combined_boundary_calls <= 2) {
        if (local_reason) {
          *local_reason = combined_boundary_calls == 1 ?
            "local_curobo_service_failed:curobo_start_state_in_collision_or_bounds" :
            "local_curobo_service_failed:curobo_goal_state_in_collision_or_bounds";
        }
        return false;
      }
      return client.plan(
        "combined_boundary_retry", local_start, local_goal, boxes(),
        local_plan, local_metrics, local_reason);
    };
  const auto combined_boundary_repaired = combined_boundary_retry.plan(start, goal);
  assert(combined_boundary_repaired.valid);
  assert(combined_boundary_repaired.method == "shortcut_local_curobo");
  assert(combined_boundary_calls >= 3);
  assert(combined_boundary_repaired.local_fallback_segment_count == 0);

  // Missing service returns immediately and never enters a nested executor spin.
  CuroboSegmentClientConfig missing_config = config;
  missing_config.service_name = "/test/missing_joint_segment";
  CuroboSegmentClient missing(*client_node, missing_config);
  const auto missing_started = std::chrono::steady_clock::now();
  assert(!missing.plan("missing", start, goal, boxes(), &plan, &metrics, &reason));
  assert(reason.find("service_unavailable") != std::string::npos);
  assert(std::chrono::duration<double>(std::chrono::steady_clock::now() - missing_started).count() < 0.1);

  auto missing_transition = integration_planner();
  missing_transition.preferred_local_plan =
    [&missing](const auto& local_start, const auto& local_goal, auto* local_plan,
               auto* local_metrics, std::string* local_reason) {
      return missing.plan(
        "missing_integration", local_start, local_goal, boxes(),
        local_plan, local_metrics, local_reason);
    };
  const auto missing_fallback = missing_transition.plan(start, goal);
  assert(missing_fallback.valid);
  assert(missing_fallback.method == "shortcut_local_rrt");
  assert(missing_fallback.local_fallback_used);

  auto timeout_service = server_node->create_service<CuroboSegmentClient::Service>(
    "/test/timeout_joint_segment",
    [](const std::shared_ptr<CuroboSegmentClient::Service::Request> request,
       std::shared_ptr<CuroboSegmentClient::Service::Response> response) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      fill_success_response(*request, response.get());
    });
  CuroboSegmentClientConfig timeout_config = config;
  timeout_config.service_name = "/test/timeout_joint_segment";
  timeout_config.timeout_s = 0.02;
  CuroboSegmentClient timeout_client(*client_node, timeout_config);
  assert(wait_ready(timeout_client));
  assert(!timeout_client.plan("timeout", start, goal, boxes(), &plan, &metrics, &reason));
  assert(reason.find("local_curobo_timeout") != std::string::npos);
  assert(metrics.roundtrip_ms >= 15.0 && metrics.roundtrip_ms < 100.0);

  auto timeout_transition = integration_planner();
  timeout_transition.preferred_local_plan =
    [&timeout_client](const auto& local_start, const auto& local_goal, auto* local_plan,
                      auto* local_metrics, std::string* local_reason) {
      return timeout_client.plan(
        "timeout_integration", local_start, local_goal, boxes(),
        local_plan, local_metrics, local_reason);
    };
  const auto timeout_fallback = timeout_transition.plan(start, goal);
  assert(timeout_fallback.valid);
  assert(timeout_fallback.method == "shortcut_local_rrt");
  assert(timeout_fallback.local_fallback_used);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return 0;
}
