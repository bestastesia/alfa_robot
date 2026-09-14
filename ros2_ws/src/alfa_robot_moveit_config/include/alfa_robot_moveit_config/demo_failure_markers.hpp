#pragma once

#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace alfa_robot::motion
{
// Observer-only markers. Never change a planning scene, ACM, or controller command.
inline void appendDemoFailureMarkers(
  visualization_msgs::msg::MarkerArray& markers, const nlohmann::json& diagnostic,
  bool frozen, const std::string& world, const builtin_interfaces::msg::Time& stamp)
{
  using Marker = visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = world;
  marker.header.stamp = stamp;
  marker.ns = "failure_diagnostic";
  // 20 contacts + one target, each with a label. Clear on new tasks too.
  for (int id = 0; id < 42; ++id) {
    marker.id = id;
    marker.action = Marker::DELETE;
    markers.markers.push_back(marker);
  }
  if (!frozen || diagnostic.empty()) return;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0F;
  marker.color.a = 1.0F;
  marker.id = 0;
  auto point = [&](const nlohmann::json& position, const std::string& label) {
    marker.type = Marker::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.045;
    marker.pose.position.x = position.at(0).get<double>();
    marker.pose.position.y = position.at(1).get<double>();
    marker.pose.position.z = position.at(2).get<double>();
    markers.markers.push_back(marker);
    ++marker.id;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.025;
    marker.pose.position.z += 0.05;
    marker.text = label;
    markers.markers.push_back(marker);
    ++marker.id;
  };
  for (const auto& contact : diagnostic.value("contacts", nlohmann::json::array()))
    point(contact.at("position"), contact.at("bodies").at(0).get<std::string>() +
      " <-> " + contact.at("bodies").at(1).get<std::string>());
  if (diagnostic.contains("target")) point(diagnostic.at("target"), "FAILED TARGET (diagnostic only)");
}
}  // namespace alfa_robot::motion
