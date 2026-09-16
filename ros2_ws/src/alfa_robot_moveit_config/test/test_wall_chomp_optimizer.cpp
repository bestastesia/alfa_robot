#include <alfa_robot_moveit_config/wall_trajectory_postprocessing.hpp>

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using alfa_robot::motion::TrajectoryFrame;
using alfa_robot::motion::optimizeChompFreeSpace;

namespace
{
TrajectoryFrame frame(const std::string& stage, bool attached = false, bool visible = true)
{
  TrajectoryFrame result;
  result.stage = stage;
  result.joints = {0.0};
  result.box_attached = attached;
  result.box_visible = visible;
  return result;
}

bool sameFrames(const std::vector<TrajectoryFrame>& left, const std::vector<TrajectoryFrame>& right)
{
  if (left.size() != right.size()) return false;
  for (size_t index = 0; index < left.size(); ++index) {
    if (left[index].stage != right[index].stage || left[index].joints != right[index].joints ||
        left[index].box_attached != right[index].box_attached ||
        left[index].box_visible != right[index].box_visible ||
        left[index].scene_index != right[index].scene_index ||
        left[index].carried_boxes != right[index].carried_boxes ||
        left[index].time_from_start_s != right[index].time_from_start_s)
      return false;
  }
  return true;
}
}  // namespace

std::string readFile(const char* path)
{
  std::ifstream stream(path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

int main(int argc, char** argv)
{
  std::vector<TrajectoryFrame> split_by_stage{
    frame("rrt_a"), frame("rrt_a"), frame("rrt_b"), frame("rrt_b")};
  const auto original_stage_split = split_by_stage;
  double wall_ms = -1.0;
  std::string reason = "stale";
  assert(optimizeChompFreeSpace(
    &split_by_stage, {}, {}, {}, {}, "", &wall_ms, &reason));
  assert(sameFrames(split_by_stage, original_stage_split));
  assert(wall_ms == 0.0);
  assert(reason.empty());

  std::vector<TrajectoryFrame> split_by_lifecycle{
    frame("rrt_a"), frame("rrt_a"), frame("rrt_a", true),
    frame("rrt_a", true), frame("rrt_a", true, false), frame("rrt_a", true, false)};
  const auto original_lifecycle_split = split_by_lifecycle;
  assert(optimizeChompFreeSpace(
    &split_by_lifecycle, {}, {}, {}, {}, "", &wall_ms, &reason));
  assert(sameFrames(split_by_lifecycle, original_lifecycle_split));

  std::vector<TrajectoryFrame> optimizable{
    frame("rrt_a"), frame("rrt_a"), frame("rrt_a")};
  const auto original_failure = optimizable;
  assert(!optimizeChompFreeSpace(
    &optimizable, {}, {}, {}, {}, "", &wall_ms, &reason));
  assert(sameFrames(optimizable, original_failure));
  assert(reason == "CHOMP robot model is null");

  if (argc == 3) {
    auto urdf = urdf::parseURDF(readFile(argv[1]));
    auto srdf = std::make_shared<srdf::Model>();
    assert(urdf && srdf->initString(*urdf, readFile(argv[2])));
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    moveit::core::RobotState home(model);
    home.setToDefaultValues();
    assert(home.setToDefaultValues(model->getJointModelGroup("whole_body"), "home"));
    scene->setCurrentState(home);

    const std::vector<std::string> names{
      "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5",
      "left_joint6", "left_joint7", "right_joint1", "right_joint2", "right_joint3",
      "right_joint4", "right_joint5", "right_joint6", "right_joint7", "updown", "head_joint"};
    std::vector<TrajectoryFrame> path;
    for (size_t index = 0; index < 7; ++index) {
      TrajectoryFrame point = frame("rrt_smoke");
      point.joints.clear();
      for (const auto& name : names) point.joints.push_back(home.getVariablePosition(name));
      point.joints.front() += 0.05 * std::sin(3.141592653589793 * index / 6.0);
      path.push_back(std::move(point));
    }
    const auto metadata = path;
    assert(optimizeChompFreeSpace(
      &path, model, scene, scene, names, "whole_body", &wall_ms, &reason));
    assert(path.size() == metadata.size());
    assert(path.front().joints == metadata.front().joints);
    assert(path.back().joints == metadata.back().joints);
    for (size_t index = 0; index < path.size(); ++index) {
      assert(path[index].stage == metadata[index].stage);
      assert(path[index].box_attached == metadata[index].box_attached);
      assert(path[index].box_visible == metadata[index].box_visible);
      assert(path[index].scene_index == metadata[index].scene_index);
      assert(path[index].carried_boxes == metadata[index].carried_boxes);
    }
    assert(wall_ms >= 0.0);
    assert(reason.empty());

    auto left_arm_path = metadata;
    assert(optimizeChompFreeSpace(
      &left_arm_path, model, scene, scene, names, "left_arm", &wall_ms, &reason));
    assert(left_arm_path.front().joints == metadata.front().joints);
    assert(left_arm_path.back().joints == metadata.back().joints);
    for (size_t index = 0; index < left_arm_path.size(); ++index)
      for (size_t joint = 7; joint < names.size(); ++joint)
        assert(left_arm_path[index].joints[joint] == metadata[index].joints[joint]);

    std::vector<TrajectoryFrame> attached_path;
    TrajectoryFrame attachment = frame("attach_box", true);
    attachment.joints.clear();
    for (const auto& name : names) attachment.joints.push_back(home.getVariablePosition(name));
    attached_path.push_back(attachment);
    for (size_t index = 0; index < 3; ++index) {
      TrajectoryFrame point = attachment;
      point.stage = "rrt_return";
      attached_path.push_back(std::move(point));
    }
    const auto attached_original = attached_path;
    assert(!optimizeChompFreeSpace(
      &attached_path, model, scene, scene, names, "left_arm", &wall_ms, &reason));
    assert(sameFrames(attached_path, attached_original));
    assert(reason == "CHOMP cannot reconstruct payload carried_target_box on left_tool0");
    assert(wall_ms >= 0.0);
  }
}
