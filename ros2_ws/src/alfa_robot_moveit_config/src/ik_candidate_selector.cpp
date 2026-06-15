#include "alfa_robot_moveit_config/ik_candidate_selector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace alfa_robot::motion
{
namespace
{

double wrapped_angle_delta(double lhs, double rhs)
{
  double delta = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
  if (delta < 0.0) {
    delta += 2.0 * M_PI;
  }
  return std::abs(delta - M_PI);
}

std::optional<double> candidate_joint_value(
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  const std::string& joint_name)
{
  for (size_t i = 0; i < candidate.full_joint_names.size() && i < candidate.full_joint_values.size(); ++i) {
    if (candidate.full_joint_names[i] == joint_name) {
      return candidate.full_joint_values[i];
    }
  }
  return std::nullopt;
}

}  // namespace

IkCandidateSelector::IkCandidateSelector(IkCandidateSelectorConfig config)
: config_(std::move(config))
{}

bool IkCandidateSelector::similar(
  const ik_benchmark::UpdownAwareIkCandidate& candidate,
  const ik_benchmark::UpdownAwareIkCandidate& kept) const
{
  if (config_.h_threshold >= 0.0 &&
      std::abs(candidate.h - kept.h) > config_.h_threshold) {
    return false;
  }
  if (config_.joint_threshold <= 0.0) {
    return false;
  }

  static const std::array<const char*, 12> arm_joints = {
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3",
    "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3",
    "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
  };

  size_t compared = 0;
  for (const auto* joint_name : arm_joints) {
    const auto lhs = candidate_joint_value(candidate, joint_name);
    const auto rhs = candidate_joint_value(kept, joint_name);
    if (!lhs || !rhs) {
      continue;
    }
    ++compared;
    if (wrapped_angle_delta(*lhs, *rhs) > config_.joint_threshold) {
      return false;
    }
  }
  return compared > 0;
}

std::vector<ik_benchmark::UpdownAwareIkCandidate> IkCandidateSelector::select(
  const std::vector<ik_benchmark::UpdownAwareIkCandidate>& sorted_legal_candidates,
  IkCandidateSelectionStats* stats) const
{
  IkCandidateSelectionStats local_stats;
  local_stats.enabled = config_.dedup_enabled && config_.joint_threshold > 0.0;
  local_stats.input_count = sorted_legal_candidates.size();

  std::vector<ik_benchmark::UpdownAwareIkCandidate> selected;
  if (local_stats.enabled) {
    const auto start = std::chrono::steady_clock::now();
    selected.reserve(sorted_legal_candidates.size());
    for (const auto& candidate : sorted_legal_candidates) {
      bool duplicate = false;
      for (const auto& kept : selected) {
        if (similar(candidate, kept)) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        selected.push_back(candidate);
      }
    }
    local_stats.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  } else {
    selected = sorted_legal_candidates;
  }

  local_stats.unique_count = selected.size();
  local_stats.removed_count = local_stats.input_count > local_stats.unique_count
    ? local_stats.input_count - local_stats.unique_count
    : 0;
  if (config_.candidate_limit > 0 && selected.size() > config_.candidate_limit) {
    selected.resize(config_.candidate_limit);
  }
  local_stats.selected_count = selected.size();
  if (stats) {
    *stats = local_stats;
  }
  return selected;
}

}  // namespace alfa_robot::motion
