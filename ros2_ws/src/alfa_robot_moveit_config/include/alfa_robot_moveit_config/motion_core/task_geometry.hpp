#pragma once

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{

struct BoxSpec
{
  int id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PickPair
{
  int round = 0;
  int left_box = 0;
  int right_box = 0;
  bool top_suction = false;
};

struct ContainerPanel
{
  std::string id;
  std::array<double, 3> center;
  std::array<double, 3> size;
};

struct StaticBoxObstacle
{
  std::string id;
  std::array<double, 3> center;
  std::array<double, 3> size;
};

struct AttachedBoxSpec
{
  std::string id;
  std::string link_name;
  std::array<double, 3> center_in_link;
  std::array<double, 3> size;
};

struct AxisAlignedBox
{
  std::array<double, 3> center;
  std::array<double, 3> size;
};

std::string trim_copy(std::string value);

std::map<int, BoxSpec> make_boxes(double front_x);

std::vector<std::pair<int, int>> parse_box_pair_list(const std::string& value);

std::vector<PickPair> make_pick_pairs(
  bool include_top_suction,
  const std::vector<std::pair<int, int>>& front_pairs);

}  // namespace alfa_robot::motion
