#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace alfa_robot::motion
{
struct ComfortHeight
{
  double position;
  double ratio;
  double ideal_position;
  bool projected;
};

// Geometric selection only: never probes IK/collisions or retries another height.
inline ComfortHeight chooseComfortHeight(
  double shoulder_z, double target_z, double xy, double length, double initial,
  double lower, double upper, double ratio_min, double preferred, double ratio_max,
  const std::string& branch = "auto")
{
  for (double v : {shoulder_z, target_z, xy, length, initial, lower, upper,
                   ratio_min, preferred, ratio_max}) {
    if (!std::isfinite(v)) throw std::invalid_argument("comfort geometry must be finite");
  }
  if (xy < 0 || length <= 0 || lower > upper || initial < lower || initial > upper ||
      ratio_min <= 0 || ratio_min > preferred || preferred > ratio_max ||
      (branch != "auto" && branch != "above" && branch != "below"))
    throw std::invalid_argument("invalid comfort interval, bounds or branch");
  const double equal = initial + target_z - shoulder_z;
  const double radius = preferred * length;
  if (!std::isfinite(radius)) throw std::invalid_argument("comfort radius overflow");
  std::vector<double> ideals;
  if (radius >= xy) {
    const double dz = radius * std::sqrt(std::max(0.0, 1.0 - (xy / radius) * (xy / radius)));
    if (branch != "below") ideals.push_back(equal + dz);
    if (branch != "above") ideals.push_back(equal - dz);
  } else {
    ideals.push_back(equal);
  }
  // For forced-branch experiments, use only its projected root. Never use these
  // oracle diagnostics to choose a successful branch after planning.
  if (branch == "auto") {
    ideals.push_back(equal);
    ideals.push_back(lower);
    ideals.push_back(upper);
  }
  auto evaluate = [&](double ideal) {
    const double q = std::clamp(ideal, lower, upper);
    return ComfortHeight{q, std::hypot(xy, q - equal) / length, ideal,
                         std::abs(q - ideal) > 1e-9};
  };
  auto better = [&](const ComfortHeight& a, const ComfortHeight& b) {
    const auto gap = [&](double r) { return std::max({ratio_min - r, r - ratio_max, 0.0}); };
    const double scores_a[] = {gap(a.ratio), std::abs(a.ratio - preferred),
      a.position < equal - 1e-9 ? 1.0 : 0.0, std::abs(a.position - initial), -a.position};
    const double scores_b[] = {gap(b.ratio), std::abs(b.ratio - preferred),
      b.position < equal - 1e-9 ? 1.0 : 0.0, std::abs(b.position - initial), -b.position};
    for (int i = 0; i < 5; ++i) {
      if (scores_a[i] < scores_b[i] - 1e-9) return true;
      if (scores_a[i] > scores_b[i] + 1e-9) return false;
    }
    return false;
  };
  auto best = evaluate(ideals.front());
  for (double ideal : ideals) {
    auto candidate = evaluate(ideal);
    if (better(candidate, best)) best = candidate;
  }
  return best;
}
}  // namespace alfa_robot::motion
