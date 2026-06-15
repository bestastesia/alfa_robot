#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace alfa_robot::motion
{

struct ExtractDemoCallbacks
{
  std::function<void()> clear_scene;
  std::function<void()> reset_commanded_state;
  std::function<bool(int, int)> run_pair;
  std::function<void(bool, const std::string&, const std::vector<std::pair<int, int>>&)> record_summary;
  std::function<std::string()> last_error;
  std::function<void(const std::string&)> set_last_error;
};

struct ExtractDemoConfig
{
  bool all_rows = false;
  int left_box_id = 2;
  int right_box_id = 4;
  std::vector<std::pair<int, int>> pair_sequence;
};

class ExtractDemoOrchestrator
{
public:
  ExtractDemoOrchestrator(ExtractDemoConfig config, ExtractDemoCallbacks callbacks);

  bool run();

private:
  ExtractDemoConfig config_;
  ExtractDemoCallbacks callbacks_;
};

}  // namespace alfa_robot::motion
