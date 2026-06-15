#pragma once

#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <cstddef>
#include <vector>

namespace alfa_robot::motion
{

struct IkCandidateSelectorConfig
{
  bool dedup_enabled = false;
  double joint_threshold = 1.0 * 3.14159265358979323846 / 180.0;
  double h_threshold = 0.005;
  size_t candidate_limit = 0;
};

struct IkCandidateSelectionStats
{
  bool enabled = false;
  size_t input_count = 0;
  size_t unique_count = 0;
  size_t selected_count = 0;
  size_t removed_count = 0;
  double elapsed_ms = 0.0;
};

class IkCandidateSelector
{
public:
  explicit IkCandidateSelector(IkCandidateSelectorConfig config);

  const IkCandidateSelectorConfig& config() const { return config_; }

  std::vector<ik_benchmark::UpdownAwareIkCandidate> select(
    const std::vector<ik_benchmark::UpdownAwareIkCandidate>& sorted_legal_candidates,
    IkCandidateSelectionStats* stats = nullptr) const;

private:
  bool similar(
    const ik_benchmark::UpdownAwareIkCandidate& candidate,
    const ik_benchmark::UpdownAwareIkCandidate& kept) const;

  IkCandidateSelectorConfig config_;
};

}  // namespace alfa_robot::motion
