#include "alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp"
#include <algorithm>
#include <array>

using alfa_robot::analytic_ik::V3RedundantArmAnalyticIk;
using alfa_robot::analytic_ik::V3RedundantArmModel;
using alfa_robot::analytic_ik::V3RedundantIkRequest;

extern "C" double v3_swivel(int side, const double* joints)
{
  V3RedundantArmAnalyticIk solver(
    side == 0 ? V3RedundantArmModel::V322Left : V3RedundantArmModel::V322Right);
  std::array<double, 7> values;
  std::copy_n(joints, 7, values.begin());
  return solver.swivelAngle(values);
}

extern "C" int v3_solve(int side, const double* matrix, double swivel,
                         const double* seed, double* output, int capacity)
{
  V3RedundantArmAnalyticIk solver(
    side == 0 ? V3RedundantArmModel::V322Left : V3RedundantArmModel::V322Right);
  V3RedundantIkRequest request;
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      request.target_in_arm_base.matrix()(row, column) = matrix[row * 4 + column];
  request.swivel_angle = swivel;
  std::copy_n(seed, 7, request.seed.begin());
  request.position_tolerance = 1e-6;
  request.orientation_tolerance = 1e-6;
  const auto solutions = solver.solveInArmBase(request);
  const int count = std::min(capacity, static_cast<int>(solutions.size()));
  for (int index = 0; index < count; ++index)
    std::copy(solutions[index].joints.begin(), solutions[index].joints.end(), output + 7 * index);
  return count;
}

// Batched branch-fixed solve: for each target keep the solution whose signs of q2/q4/q6 match
// `branch` (+1/-1) and that is nearest to its seed; `found[i]` reports whether one exists.
extern "C" int v3_solve_batch(int side, int count, const double* matrices, const double* swivels,
                              const double* seeds, const int* branch, double* output, int* found)
{
  V3RedundantArmAnalyticIk solver(
    side == 0 ? V3RedundantArmModel::V322Left : V3RedundantArmModel::V322Right);
  int solved = 0;
  for (int item = 0; item < count; ++item) {
    V3RedundantIkRequest request;
    for (int row = 0; row < 4; ++row)
      for (int column = 0; column < 4; ++column)
        request.target_in_arm_base.matrix()(row, column) = matrices[16 * item + row * 4 + column];
    request.swivel_angle = swivels[item];
    const double* seed = seeds + 7 * item;
    std::copy_n(seed, 7, request.seed.begin());
    request.position_tolerance = 1e-6;
    request.orientation_tolerance = 1e-6;
    found[item] = 0;
    double best = 0.0;
    for (const auto& solution : solver.solveInArmBase(request)) {
      const auto& joints = solution.joints;
      if ((joints[1] > 0) != (branch[0] > 0) || (joints[3] > 0) != (branch[1] > 0) ||
          (joints[5] > 0) != (branch[2] > 0))
        continue;
      double distance = 0.0;
      for (int joint = 0; joint < 7; ++joint)
        distance += (joints[joint] - seed[joint]) * (joints[joint] - seed[joint]);
      if (!found[item] || distance < best) {
        std::copy(joints.begin(), joints.end(), output + 7 * item);
        best = distance;
        found[item] = 1;
      }
    }
    solved += found[item];
  }
  return solved;
}
