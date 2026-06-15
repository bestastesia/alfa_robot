#pragma once

#include "alfa_robot_moveit_config/extract_planner_types.hpp"

#include <rclcpp/logger.hpp>

#include <string>
#include <vector>

namespace alfa_robot::motion
{

class ExtractBenchmarkCsvWriter
{
public:
  static bool write(
    const std::string& path,
    const std::vector<ExtractRolloutTiming>& timings,
    const rclcpp::Logger& logger);
};

}  // namespace alfa_robot::motion
