#include "robot_motion_scene_service/motion_core/task_geometry.hpp"

#include <sstream>

namespace alfa_robot::motion
{

std::string trim_copy(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::map<int, BoxSpec> make_boxes(double front_x, double y_shift)
{
  const std::vector<std::vector<std::pair<int, double>>> rows_top_to_bottom = {
    {{1, 0.8}, {2, 0.4}, {3, 0.0}, {4, -0.4}, {5, -0.8}},
    {{6, 0.8}, {7, 0.4}, {8, 0.0}, {9, -0.4}, {10, -0.8}},
    {{11, 0.8}, {12, 0.4}, {13, 0.0}, {14, -0.4}, {15, -0.8}},
    {{16, 0.8}, {17, 0.4}, {18, 0.0}, {19, -0.4}, {20, -0.8}},
    {{21, 0.8}, {22, 0.4}, {23, 0.0}, {24, -0.4}, {25, -0.8}},
  };

  std::map<int, BoxSpec> boxes;
  for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
    const double z = 0.2 + 0.4 * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
    for (const auto& [id, y] : rows_top_to_bottom[row]) {
      boxes[id] = BoxSpec{id, front_x, y + y_shift, z};
    }
  }
  return boxes;
}

std::vector<std::pair<int, int>> parse_box_pair_list(const std::string& value)
{
  std::vector<std::pair<int, int>> pairs;
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, ';')) {
    segment = trim_copy(segment);
    if (segment.empty()) continue;
    const auto comma = segment.find(',');
    const auto slash = segment.find('/');
    const auto sep = comma == std::string::npos ? slash : comma;
    if (sep == std::string::npos) continue;
    const std::string left = trim_copy(segment.substr(0, sep));
    const std::string right = trim_copy(segment.substr(sep + 1));
    if (left.empty() || right.empty()) continue;
    pairs.push_back({std::stoi(left), std::stoi(right)});
  }
  return pairs;
}

std::vector<PickPair> make_pick_pairs(
  bool include_top_suction,
  const std::vector<std::pair<int, int>>& front_pairs)
{
  std::vector<PickPair> pairs;
  pairs.reserve(front_pairs.size() + 1);
  for (size_t index = 0; index < front_pairs.size(); ++index) {
    pairs.push_back({
      static_cast<int>(index + 1),
      front_pairs[index].first,
      front_pairs[index].second,
      false,
    });
  }
  if (include_top_suction) {
    const int round = static_cast<int>(pairs.size() + 1);
    pairs.push_back({round, 22, 24, true});
  }
  return pairs;
}

std::vector<std::string> dual_arm_with_updown_joint_names()
{
  return {
    "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
  };
}

}  // namespace alfa_robot::motion
