#include "alfa_robot_moveit_config/extract_demo_orchestrator.hpp"

#include <utility>

namespace alfa_robot::motion
{

ExtractDemoOrchestrator::ExtractDemoOrchestrator(
  ExtractDemoConfig config,
  ExtractDemoCallbacks callbacks)
: config_(std::move(config)), callbacks_(std::move(callbacks))
{}

bool ExtractDemoOrchestrator::run()
{
  if (callbacks_.clear_scene) callbacks_.clear_scene();
  if (callbacks_.set_last_error) callbacks_.set_last_error("");
  if (callbacks_.reset_commanded_state) callbacks_.reset_commanded_state();

  if (!config_.all_rows) {
    const bool ok = callbacks_.run_pair
      ? callbacks_.run_pair(config_.left_box_id, config_.right_box_id)
      : false;
    return ok;
  }

  bool all_ok = true;
  std::string first_error;
  for (const auto& [left_box_id, right_box_id] : config_.pair_sequence) {
    const bool ok = callbacks_.run_pair
      ? callbacks_.run_pair(left_box_id, right_box_id)
      : false;
    all_ok = all_ok && ok;
    if (!ok && first_error.empty() && callbacks_.last_error) {
      first_error = callbacks_.last_error();
    }
    if (callbacks_.clear_scene) callbacks_.clear_scene();
    if (callbacks_.reset_commanded_state) callbacks_.reset_commanded_state();
  }

  if (callbacks_.record_summary) {
    callbacks_.record_summary(all_ok, all_ok ? "" : first_error, config_.pair_sequence);
  }
  if (!all_ok && !first_error.empty() && callbacks_.set_last_error) {
    callbacks_.set_last_error(first_error);
  }
  return all_ok;
}

}  // namespace alfa_robot::motion
