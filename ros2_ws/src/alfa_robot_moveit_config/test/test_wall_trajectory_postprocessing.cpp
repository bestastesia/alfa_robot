#include <alfa_robot_moveit_config/wall_trajectory_postprocessing.hpp>

#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using alfa_robot::motion::TrajectoryFrame;
using alfa_robot::motion::TrajectoryPostprocessMetrics;
using alfa_robot::motion::TrajectoryPostprocessOptions;

moveit::core::RobotModelPtr testModel()
{
  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(R"(
    <robot name="wall_postprocessing_test">
      <link name="world"/>
      <link name="carriage"/>
      <joint name="updown" type="prismatic">
        <parent link="world"/>
        <child link="carriage"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>)"));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(
    <robot name="wall_postprocessing_test">
      <group name="whole_body"><joint name="updown"/></group>
    </robot>)"));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  auto* joint = model->getJointModel("updown");
  auto bounds = joint->getVariableBounds("updown");
  bounds.acceleration_bounded_ = true;
  bounds.min_acceleration_ = -1.0;
  bounds.max_acceleration_ = 1.0;
  joint->setVariableBounds("updown", bounds);
  return model;
}

TrajectoryFrame frame(
  double joint,
  std::string stage = "move",
  bool attached = false,
  bool visible = true)
{
  TrajectoryFrame result;
  result.stage = std::move(stage);
  result.joints = {joint};
  result.box_attached = attached;
  result.box_visible = visible;
  return result;
}

bool process(
  std::vector<TrajectoryFrame>* frames,
  const TrajectoryPostprocessOptions& options,
  const alfa_robot::motion::EdgeValidator& validator,
  TrajectoryPostprocessMetrics* metrics,
  std::string* reason)
{
  return alfa_robot::motion::postprocessWallTrajectory(
    frames, testModel(), {"updown"}, options, validator, {}, {}, metrics, reason);
}

bool close(double left, double right)
{
  return std::abs(left - right) < 1e-8;
}
}  // namespace

int main()
{
  {
    std::vector<TrajectoryFrame> frames{frame(0.0), frame(0.2), frame(0.4)};
    TrajectoryPostprocessOptions options;
    options.variant = "topk";
    options.sample_period = 0.1;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    assert(alfa_robot::motion::postprocessWallTrajectory(
      &frames, {}, {}, options, {}, {}, {}, &metrics, &reason));
    assert(reason.empty());
    assert(close(frames[0].time_from_start_s, 0.0));
    assert(close(frames[1].time_from_start_s, 0.1));
    assert(close(frames[2].time_from_start_s, 0.2));
    assert(!metrics.timing_valid);
    assert(metrics.effective_variant == "topk");
  }

  {
    const std::vector<TrajectoryFrame> original{
      frame(0.0), frame(0.2), frame(0.4), frame(0.6)};
    std::vector<std::pair<double, double>> attempts;
    const auto validator = [&attempts](const TrajectoryFrame& from, const TrajectoryFrame& to,
                           std::string* reason) {
      attempts.emplace_back(from.joints[0], to.joints[0]);
      if (std::abs(to.joints[0] - from.joints[0]) <= 0.41) return true;
      if (reason) *reason = "too_far";
      return false;
    };
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.sample_period = 0.05;
    options.updown_max_jerk = 1.0;
    TrajectoryPostprocessMetrics first_metrics;
    std::string reason;
    auto first = original;
    assert(process(&first, options, validator, &first_metrics, &reason));
    assert(attempts.size() >= 3);
    assert(close(attempts[0].first, 0.0) && close(attempts[0].second, 0.6));
    assert(close(attempts[1].first, 0.0) && close(attempts[1].second, 0.4));
    assert(first_metrics.timing_valid);
    assert(first_metrics.effective_variant == "shortcut_ruckig");
    assert(first_metrics.execution_duration_s > 0.0);
    assert(close(first.front().joints[0], 0.0));
    assert(close(first.back().joints[0], 0.6));

    attempts.clear();
    TrajectoryPostprocessMetrics second_metrics;
    auto second = original;
    assert(process(&second, options, validator, &second_metrics, &reason));
    assert(first.size() == second.size());
    for (size_t index = 0; index < first.size(); ++index) {
      assert(close(first[index].joints[0], second[index].joints[0]));
      assert(close(first[index].time_from_start_s, second[index].time_from_start_s));
    }
  }

  {
    std::vector<TrajectoryFrame> frames{frame(0.0), frame(0.2), frame(0.4)};
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.sample_period = 0.05;
    options.updown_max_jerk = 1.0;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_edges = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    const auto rejects_timed_replay = [](const std::vector<TrajectoryFrame>& timed,
                                         std::string* path_reason) {
      if (timed.size() > 3 && timed.back().time_from_start_s > 0.0) {
        if (path_reason) *path_reason = "timed replay collision";
        return false;
      }
      return true;
    };
    assert(!alfa_robot::motion::postprocessWallTrajectory(
      &frames, testModel(), {"updown"}, options, accepts_edges, rejects_timed_replay, {},
      &metrics, &reason));
    assert(reason.find("timed replay collision") != std::string::npos);
  }

  {
    std::vector<TrajectoryFrame> frames{frame(0.0), frame(0.2), frame(0.4)};
    TrajectoryPostprocessOptions options;
    options.variant = "chomp_ruckig";
    options.updown_max_jerk = 1.0;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_everything = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    const auto failing_chomp = [](std::vector<TrajectoryFrame>*, double* wall_ms, std::string* why) {
      *wall_ms = 1.5;
      *why = "test failure";
      return false;
    };
    assert(alfa_robot::motion::postprocessWallTrajectory(
      &frames, testModel(), {"updown"}, options, accepts_everything, {}, failing_chomp,
      &metrics, &reason));
    assert(reason.empty());
    assert(metrics.timing_valid);
    assert(metrics.effective_variant == "shortcut_ruckig");
    assert(metrics.optimizer_status == "failed_fallback");
    assert(metrics.fallback_reason == "CHOMP failed: test failure");
    assert(close(metrics.chomp_ms, 1.5));
    assert(close(frames.front().joints[0], 0.0));
    assert(close(frames.back().joints[0], 0.4));
  }

  {
    std::vector<TrajectoryFrame> frames{
      frame(0.0, "approach"),
      frame(0.2, "approach"),
      frame(0.2, "attach", true),
      frame(0.4, "carry", true),
    };
    const auto validator = [](const TrajectoryFrame& from, const TrajectoryFrame& to,
                           std::string* reason) {
      if (from.box_attached != to.box_attached && from.joints != to.joints) {
        if (reason) *reason = "attachment_motion";
        return false;
      }
      return true;
    };
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.sample_period = 0.05;
    options.updown_max_jerk = 1.0;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    assert(process(&frames, options, validator, &metrics, &reason));
    bool preserved_boundary = false;
    for (size_t index = 1; index < frames.size(); ++index) {
      if (!frames[index - 1].box_attached && frames[index].box_attached) {
        preserved_boundary = close(frames[index - 1].joints[0], frames[index].joints[0]) &&
          close(frames[index - 1].time_from_start_s, frames[index].time_from_start_s);
      }
    }
    assert(preserved_boundary);
  }

  {
    std::vector<TrajectoryFrame> frames{
      frame(0.0, "dual_6", true, false),
      frame(5e-7, "dual_6", true, false),
    };
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.updown_max_jerk = 1.0;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_everything = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    assert(process(&frames, options, accepts_everything, &metrics, &reason));
    assert(reason.empty());
    assert(metrics.timing_valid);
    assert(metrics.execution_duration_s == 0.0);
    assert(frames.size() == 2);
    assert(frames.front().stage == "dual_6" && frames.back().stage == "dual_6");
    assert(frames.front().box_attached && frames.back().box_attached);
    assert(!frames.front().box_visible && !frames.back().box_visible);
    assert(frames.front().joints[0] == 0.0 && frames.back().joints[0] == 5e-7);
    assert(frames.front().time_from_start_s == 0.0 && frames.back().time_from_start_s == 0.0);
  }

  {
    std::vector<TrajectoryFrame> frames{
      frame(0.0, "approach", false),
      frame(0.001, "approach", false),
      frame(0.001, "attach", true),
      frame(0.0, "return", true),
    };
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.sample_period = 0.05;
    options.updown_max_jerk = 0.01;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_everything = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    assert(process(&frames, options, accepts_everything, &metrics, &reason));
    assert(metrics.max_velocity <= 1.0 + 1e-9);
    assert(metrics.max_acceleration <= 1.0 + 1e-9);
    assert(metrics.max_jerk <= options.updown_max_jerk + 1e-9);
  }

  {
    std::vector<TrajectoryFrame> frames{frame(0.0), frame(0.001)};
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    options.sample_period = 0.05;
    options.updown_max_jerk = 1e-6;
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_everything = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    assert(process(&frames, options, accepts_everything, &metrics, &reason));
    assert(reason.empty());
    assert(metrics.timing_valid);
    assert(metrics.execution_duration_s > 1.0);
    assert(close(frames.front().joints[0], 0.0));
    assert(close(frames.back().joints[0], 0.001));
  }

  {
    std::vector<TrajectoryFrame> frames{
      frame(0.0, "approach", false),
      frame(0.2, "carry", true),
    };
    TrajectoryPostprocessOptions options;
    options.variant = "shortcut_ruckig";
    TrajectoryPostprocessMetrics metrics;
    std::string reason;
    const auto accepts_everything = [](const TrajectoryFrame&, const TrajectoryFrame&, std::string*) {
      return true;
    };
    assert(!process(&frames, options, accepts_everything, &metrics, &reason));
    assert(reason.find("lifecycle transition changes joints") != std::string::npos);
  }
}
