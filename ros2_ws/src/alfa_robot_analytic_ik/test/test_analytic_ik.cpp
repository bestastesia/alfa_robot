#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <cassert>
#include <cmath>
#include <random>

namespace
{

double random_between(std::mt19937& rng, double lower, double upper)
{
  std::uniform_real_distribution<double> dist(lower, upper);
  return dist(rng);
}

bool contains_close_solution(
  const std::vector<alfa_robot::analytic_ik::ArmAnalyticIkSolution>& solutions,
  const std::array<double, 6>& expected)
{
  for (const auto& solution : solutions) {
    double max_delta = 0.0;
    for (size_t i = 0; i < expected.size(); ++i) {
      max_delta = std::max(
        max_delta,
        alfa_robot::analytic_ik::shortestAngularDistance(solution.joints[i], expected[i]));
    }
    if (max_delta < 1e-3) {
      return true;
    }
  }
  return false;
}

void check_side(alfa_robot::analytic_ik::ArmSide side)
{
  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk solver;
  std::mt19937 rng(side == alfa_robot::analytic_ik::ArmSide::Left ? 42 : 84);
  for (size_t i = 0; i < 100; ++i) {
    std::array<double, 6> joints = {
      random_between(rng, -1.8, 1.8),
      random_between(rng, -2.4, 2.4),
      random_between(rng, -2.4, 2.4),
      random_between(rng, -2.4, 2.4),
      random_between(rng, -2.4, 2.4),
      random_between(rng, -2.4, 2.4),
    };
    const auto target = solver.forwardInArmBase(side, joints);
    alfa_robot::analytic_ik::ArmAnalyticIkRequest request;
    request.side = side;
    request.target_in_arm_base = target;
    request.seed = joints;
    request.position_tolerance = 1e-5;
    request.orientation_tolerance = 1e-5;
    const auto solutions = solver.solveInArmBase(request);
    assert(!solutions.empty());
    assert(contains_close_solution(solutions, joints));
  }
}

}  // namespace

int main()
{
  check_side(alfa_robot::analytic_ik::ArmSide::Left);
  check_side(alfa_robot::analytic_ik::ArmSide::Right);

  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk solver;
  const std::array<double, 6> seed{};
  const auto left_zero = solver.forwardInArmBase(alfa_robot::analytic_ik::ArmSide::Left, seed);
  const auto target_in_base =
    alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk::baseLinkToArmBase(
      alfa_robot::analytic_ik::ArmSide::Left, 0.3) *
    left_zero;
  const auto base_solutions = solver.solveInBaseLink(
    alfa_robot::analytic_ik::ArmSide::Left, target_in_base, 0.3, seed);
  assert(!base_solutions.empty());
  assert(contains_close_solution(base_solutions, seed));
  return 0;
}
