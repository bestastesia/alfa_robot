#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;

struct Config
{
  double min_x = -0.09;
  double max_x = 0.98;
  double min_y = -0.8;
  double max_y = 0.0;
  double min_z = 1.15;
  double max_z = 2.2;
  double step_x = 0.05;
  double step_y = 0.05;
  double step_z = 0.05;
  double fixed_updown = 0.45;
  double world_to_base_z = 0.202094;
  double min_angle_deg = 0.0;
  double max_angle_deg = 90.0;
  double angle_step_deg = 5.0;
  std::string output_csv = "analytic_orientation_rgb_reachability.csv";
};

double parse_double(const std::string& text, const std::string& name)
{
  size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("invalid value for " + name + ": " + text);
  }
  return value;
}

Config parse_arguments(int argc, char** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index) {
    const std::string name = argv[index];
    if (name == "--help" || name == "-h") {
      std::cout
        << "Pure analytic right-arm orientation reachability scan\n"
        << "  --min-x/--max-x/--min-y/--max-y/--min-z/--max-z METERS\n"
        << "  --step-x/--step-y/--step-z METERS\n"
        << "  --fixed-updown METERS --world-to-base-z METERS\n"
        << "  --min-angle-deg/--max-angle-deg/--angle-step-deg DEGREES\n"
        << "  --output-csv PATH\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + name);
    }
    const std::string value = argv[++index];
    if (name == "--min-x") config.min_x = parse_double(value, name);
    else if (name == "--max-x") config.max_x = parse_double(value, name);
    else if (name == "--min-y") config.min_y = parse_double(value, name);
    else if (name == "--max-y") config.max_y = parse_double(value, name);
    else if (name == "--min-z") config.min_z = parse_double(value, name);
    else if (name == "--max-z") config.max_z = parse_double(value, name);
    else if (name == "--step-x") config.step_x = parse_double(value, name);
    else if (name == "--step-y") config.step_y = parse_double(value, name);
    else if (name == "--step-z") config.step_z = parse_double(value, name);
    else if (name == "--fixed-updown") config.fixed_updown = parse_double(value, name);
    else if (name == "--world-to-base-z") config.world_to_base_z = parse_double(value, name);
    else if (name == "--min-angle-deg") config.min_angle_deg = parse_double(value, name);
    else if (name == "--max-angle-deg") config.max_angle_deg = parse_double(value, name);
    else if (name == "--angle-step-deg") config.angle_step_deg = parse_double(value, name);
    else if (name == "--output-csv") config.output_csv = value;
    else throw std::invalid_argument("unknown argument: " + name);
  }
  for (const auto& [name, value] : std::array<std::pair<const char*, double>, 4>{
      std::pair{"step_x", config.step_x},
      std::pair{"step_y", config.step_y},
      std::pair{"step_z", config.step_z},
      std::pair{"angle_step_deg", config.angle_step_deg}}) {
    if (value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be > 0");
    }
  }
  if (config.fixed_updown < 0.0 || config.fixed_updown > 0.7) {
    throw std::invalid_argument("fixed_updown must be within [0, 0.7]");
  }
  if (std::abs(config.max_angle_deg - config.min_angle_deg) < 1e-12) {
    throw std::invalid_argument("angle range must have non-zero width");
  }
  return config;
}

std::vector<double> inclusive_range(double lower, double upper, double step)
{
  if (lower > upper) std::swap(lower, upper);
  std::vector<double> values;
  const size_t regular_count = static_cast<size_t>(std::floor((upper - lower) / step + 1e-10));
  values.reserve(regular_count + 2);
  for (size_t index = 0; index <= regular_count; ++index) {
    values.push_back(lower + static_cast<double>(index) * step);
  }
  if (values.empty() || upper - values.back() > 1e-9) {
    values.push_back(upper);
  } else {
    values.back() = upper;
  }
  return values;
}

Eigen::Quaterniond forward_x_orientation()
{
  Eigen::Quaterniond orientation(0.0, 0.7071067811865476, 0.0, 0.7071067811865476);
  orientation.normalize();
  return orientation;
}

Eigen::Quaterniond orientation_for_angle(double angle_deg)
{
  Eigen::Quaterniond orientation =
    Eigen::AngleAxisd(-angle_deg * kPi / 180.0, Eigen::Vector3d::UnitY()) *
    forward_x_orientation();
  orientation.normalize();
  return orientation;
}

void validate_orientation_convention()
{
  const Eigen::Vector3d down_normal = orientation_for_angle(-90.0) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d forward_normal = orientation_for_angle(0.0) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d up_normal = orientation_for_angle(90.0) * Eigen::Vector3d::UnitZ();
  if (!down_normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-9) ||
      !forward_normal.isApprox(Eigen::Vector3d::UnitX(), 1e-9) ||
      !up_normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-9)) {
    throw std::logic_error("orientation convention must map -90=-Z, 0=+X, 90=+Z");
  }
}

int band_index(double angle_deg, double min_angle_deg, double max_angle_deg)
{
  const double lower = std::min(min_angle_deg, max_angle_deg);
  const double upper = std::max(min_angle_deg, max_angle_deg);
  const double span = upper - lower;
  if (angle_deg <= lower + span / 3.0 + 1e-9) return 0;
  if (angle_deg <= lower + 2.0 * span / 3.0 + 1e-9) return 1;
  return 2;
}

std::string join_angles(const std::vector<double>& angles)
{
  std::ostringstream stream;
  for (size_t index = 0; index < angles.size(); ++index) {
    if (index > 0) stream << ';';
    stream << std::fixed << std::setprecision(0) << angles[index];
  }
  return stream.str();
}

int channel_value(size_t successes, size_t total)
{
  if (total == 0) return 0;
  return static_cast<int>(std::lround(255.0 * static_cast<double>(successes) /
    static_cast<double>(total)));
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    const Config config = parse_arguments(argc, argv);
    validate_orientation_convention();
    const auto xs = inclusive_range(config.min_x, config.max_x, config.step_x);
    const auto ys = inclusive_range(config.min_y, config.max_y, config.step_y);
    const auto zs = inclusive_range(config.min_z, config.max_z, config.step_z);
    const auto angles = inclusive_range(
      config.min_angle_deg, config.max_angle_deg, config.angle_step_deg);
    const double min_angle_deg = angles.front();
    const double max_angle_deg = angles.back();
    const size_t point_count = xs.size() * ys.size() * zs.size();
    const size_t test_count = point_count * angles.size();

    const std::filesystem::path output_path(config.output_csv);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
      throw std::runtime_error("cannot open output CSV: " + output_path.string());
    }
    output
      << "x,y,z,r_success,r_total,g_success,g_total,b_success,b_total,"
      << "r,g,b,total_success,total_orientations,raw_reachable_orientations,"
      << "joint_sign_rejected_orientations,min_angle_deg,max_angle_deg,angle_step_deg,"
      << "reachable_angles_deg,angle_mask\n";

    alfa_robot::analytic_ik::ThreeParallelArmAnalyticIk solver;
    const std::array<double, 6> seed{};
    std::array<size_t, 3> global_band_success{};
    std::array<size_t, 3> global_band_total{};
    size_t any_reachable_points = 0;
    size_t all_reachable_points = 0;
    size_t raw_reachable_calls = 0;
    size_t joint_sign_rejected_calls = 0;
    size_t completed_points = 0;
    const auto start = std::chrono::steady_clock::now();

    std::cout
      << "Starting pure analytic orientation scan: points=" << point_count
      << " orientations=" << angles.size()
      << " IK_calls=" << test_count << '\n'
      << "  right arm, updown=" << config.fixed_updown
      << "m, world_to_base_z=" << config.world_to_base_z << "m\n"
      << "  angle convention: -90deg=-Z/down, 0deg=+X, +90deg=+Z/up\n"
      << "  scan range=" << min_angle_deg << ".." << max_angle_deg
      << "deg, RGB=three equal angle bands\n"
      << "  legal branch constraint: joint3 > 0rad and joint4 < 0rad\n";

    for (double x : xs) {
      for (double y : ys) {
        for (double z_world : zs) {
          std::array<size_t, 3> band_success{};
          std::array<size_t, 3> band_total{};
          std::vector<double> reachable_angles;
          size_t raw_reachable_orientations = 0;
          size_t joint_sign_rejected_orientations = 0;
          std::string angle_mask;
          angle_mask.reserve(angles.size());

          for (double angle_deg : angles) {
            const int band = band_index(angle_deg, min_angle_deg, max_angle_deg);
            ++band_total[band];
            ++global_band_total[band];

            Eigen::Isometry3d target_in_base = Eigen::Isometry3d::Identity();
            target_in_base.translation() = Eigen::Vector3d(x, y, z_world - config.world_to_base_z);
            target_in_base.linear() = orientation_for_angle(angle_deg).toRotationMatrix();
            const auto solutions = solver.solveInBaseLink(
              alfa_robot::analytic_ik::ArmSide::Right,
              target_in_base,
              config.fixed_updown,
              seed,
              1e-4,
              1e-4);
            const bool raw_reachable = !solutions.empty();
            const bool reachable = std::any_of(
              solutions.begin(), solutions.end(),
              [](const alfa_robot::analytic_ik::ArmAnalyticIkSolution& solution) {
                return solution.joints[2] > 0.0 && solution.joints[3] < 0.0;
              });
            raw_reachable_orientations += static_cast<size_t>(raw_reachable);
            raw_reachable_calls += static_cast<size_t>(raw_reachable);
            if (raw_reachable && !reachable) {
              ++joint_sign_rejected_orientations;
              ++joint_sign_rejected_calls;
            }
            angle_mask.push_back(reachable ? '1' : '0');
            if (reachable) {
              ++band_success[band];
              ++global_band_success[band];
              reachable_angles.push_back(angle_deg);
            }
          }

          const size_t total_success =
            band_success[0] + band_success[1] + band_success[2];
          any_reachable_points += static_cast<size_t>(total_success > 0);
          all_reachable_points += static_cast<size_t>(total_success == angles.size());
          output << std::fixed << std::setprecision(6)
                 << x << ',' << y << ',' << z_world << ','
                 << band_success[0] << ',' << band_total[0] << ','
                 << band_success[1] << ',' << band_total[1] << ','
                 << band_success[2] << ',' << band_total[2] << ','
                 << channel_value(band_success[0], band_total[0]) << ','
                 << channel_value(band_success[1], band_total[1]) << ','
                 << channel_value(band_success[2], band_total[2]) << ','
                 << total_success << ',' << angles.size() << ','
                 << raw_reachable_orientations << ','
                 << joint_sign_rejected_orientations << ','
                 << min_angle_deg << ',' << max_angle_deg << ',' << config.angle_step_deg << ','
                 << '"' << join_angles(reachable_angles) << '"' << ','
                 << angle_mask << '\n';

          ++completed_points;
          if (completed_points % 500 == 0 || completed_points == point_count) {
            const double progress = 100.0 * static_cast<double>(completed_points) /
              static_cast<double>(point_count);
            std::cout << "progress " << completed_points << '/' << point_count
                      << " (" << std::fixed << std::setprecision(1) << progress << "%)\n";
          }
        }
      }
    }

    output.close();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    std::cout
      << "Completed: elapsed_ms=" << std::fixed << std::setprecision(3) << elapsed_ms
      << " avg_ik_us=" << elapsed_ms * 1000.0 / static_cast<double>(test_count)
      << " any_reachable_points=" << any_reachable_points << '/' << point_count
      << " all_orientations_reachable_points=" << all_reachable_points << '/' << point_count << '\n'
      << "Raw reachable IK calls=" << raw_reachable_calls << '/' << test_count
      << " joint-sign rejected calls=" << joint_sign_rejected_calls << '\n'
      << "Band success: R=" << global_band_success[0] << '/' << global_band_total[0]
      << " G=" << global_band_success[1] << '/' << global_band_total[1]
      << " B=" << global_band_success[2] << '/' << global_band_total[2] << '\n'
      << "CSV: " << output_path << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "analytic_orientation_rgb_reachability failed: " << error.what() << '\n';
    return 1;
  }
}
