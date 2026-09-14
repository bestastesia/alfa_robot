#include "alfa_robot_moveit_config/demo_failure_markers.hpp"
#include "alfa_robot_moveit_config/extract_demo_orchestrator.hpp"
#include <cassert>

int main()
{
  using namespace alfa_robot::motion;
  int clears = 0, resets = 0, calls = 0;
  ExtractDemoCallbacks callbacks;
  callbacks.clear_scene = [&]() { ++clears; };
  callbacks.reset_commanded_state = [&]() { ++resets; };
  callbacks.run_pair = [&](int, int) { return ++calls < 2; };
  callbacks.last_error = []() { return "collision"; };
  callbacks.record_summary = [&](bool ok, const auto& error, const auto&) {
    assert(!ok && error == "collision");
  };
  ExtractDemoConfig config;
  config.all_rows = true;
  config.pair_sequence = {{0, 1}, {2, 3}, {4, 5}};
  assert(!ExtractDemoOrchestrator(config, callbacks).run());
  assert(calls == 2 && clears == 2 && resets == 2);  // no reset/continue after failure

  visualization_msgs::msg::MarkerArray markers;
  nlohmann::json diagnostic = {{"target", {1., 2., 3.}}, {"contacts", {
    {{"position", {.1, .2, .3}}, {"bodies", {"robot", "box"}}}}}};
  appendDemoFailureMarkers(markers, diagnostic, false, "world", builtin_interfaces::msg::Time());
  for (const auto& marker : markers.markers) assert(marker.action == marker.DELETE);
  markers.markers.clear();
  appendDemoFailureMarkers(markers, diagnostic, true, "world", builtin_interfaces::msg::Time());
  assert(markers.markers.size() == 46);
  assert(markers.markers[42].action == visualization_msgs::msg::Marker::ADD);
  assert(markers.markers[42].pose.position.x == .1);
  assert(markers.markers[43].text == "robot <-> box");
  assert(markers.markers[45].text == "FAILED TARGET (diagnostic only)");
}
