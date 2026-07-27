#include "robot_motion_core/ik_candidate_types.hpp"

#include <cassert>
#include <cmath>

int main()
{
  robot_motion::core::UpdownAwareIkConfig config;
  assert(config.h_candidate_count == 5);
  assert(config.h_lower == 0.0);
  assert(config.h_upper == 0.7);
  assert(config.full_h_range_scan);
  assert(config.h_step == 0.01);

  robot_motion::core::UpdownAwareIkRequest request;
  assert(request.left_target.isApprox(Eigen::Isometry3d::Identity()));
  assert(request.grasp_mode == robot_motion::core::UpdownAwareIkRequest::GraspMode::Front);

  robot_motion::core::UpdownAwareIkCandidate candidate;
  assert(!candidate.legal);
  assert(std::isinf(candidate.score));

  robot_motion::core::UpdownAwareIkResult result;
  result.candidates.push_back(candidate);
  assert(result.candidates.size() == 1);
  result.pre_score_candidates.push_back(candidate);
  assert(result.pre_score_candidates.size() == 1);
  assert(!config.cost_updown_enabled);
  assert(config.joint_limit_weights.size() == 6);
  assert(config.joint_limit_weights[1] > config.joint_limit_weights[4]);
  return 0;
}
