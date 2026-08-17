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

void assert_solutions_reach_target(
  const alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk& solver,
  alfa_robot::analytic_ik::ArmSide side,
  const Eigen::Isometry3d& target,
  const std::vector<alfa_robot::analytic_ik::ArmAnalyticIkSolution>& solutions)
{
  assert(!solutions.empty());
  for (const auto& solution : solutions) {
    const auto actual = solver.forwardInArmBase(side, solution.joints);
    assert((actual.translation() - target.translation()).norm() < 1e-5);
    assert(Eigen::AngleAxisd(target.linear().transpose() * actual.linear()).angle() < 1e-5);
  }
}

void check_side(alfa_robot::analytic_ik::ArmSide side)
{
  alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk solver;
  std::mt19937 rng(side == alfa_robot::analytic_ik::ArmSide::Left ? 42 : 84);
  const std::array<double, 6> limits = {
    1.57079632679,
    1.57079632679,
    2.44346095279,
    3.14159265359,
    2.18166156499,
    3.12413936107,
  };
  for (size_t i = 0; i < 5000; ++i) {
    std::array<double, 6> joints = {
      random_between(rng, -1.4, 1.4),
      random_between(rng, -1.4, 1.4),
      random_between(rng, -2.2, 2.2),
      random_between(rng, -2.8, 2.8),
      random_between(rng, -1.9, 1.9),
      random_between(rng, -2.8, 2.8),
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
    assert_solutions_reach_target(solver, side, target, solutions);
    for (const auto& solution : solutions) {
      for (size_t joint_index = 0; joint_index < solution.joints.size(); ++joint_index) {
        assert(std::abs(solution.joints[joint_index]) <= limits[joint_index] + 1e-7);
      }
      double raw_seed_distance_squared = 0.0;
      for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index) {
        const double delta = solution.joints[joint_index] - request.seed[joint_index];
        raw_seed_distance_squared += delta * delta;
      }
      assert(std::abs(solution.seed_distance - std::sqrt(raw_seed_distance_squared)) < 1e-12);
    }
  }


  for (double q5 : {-M_PI / 2.0, M_PI / 2.0}) {
    const std::array<double, 6> singular_joints = {0.4, -0.7, 1.1, -0.5, q5, 0.3};
    const auto target = solver.forwardInArmBase(side, singular_joints);
    alfa_robot::analytic_ik::ArmAnalyticIkRequest request;
    request.side = side;
    request.target_in_arm_base = target;
    request.seed = singular_joints;
    request.position_tolerance = 1e-5;
    request.orientation_tolerance = 1e-5;
    assert_solutions_reach_target(solver, side, target, solver.solveInArmBase(request));
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
