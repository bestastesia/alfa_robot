#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using ik_benchmark::IkResult;
using ik_benchmark::IkSolver;
using ik_benchmark::IkSolverOptions;

namespace {

struct Range {
    double min = 0.0;
    double max = 0.0;
    double step = 0.05;
};

struct Config {
    std::string group = "left_arm_with_base";
    std::string solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    std::string urdf;
    std::string srdf;
    std::string base_frame = "base_link";
    std::string tip_link = "leftjoint6";
    std::string output = "/tmp/alfa_ik_range_grid.csv";
    Range x{-0.3, 2.0, 0.05};
    Range y{-0.3, 0.8, 0.05};
    Range z{0.5, 2.5, 0.05};
    double timeout = 0.02;
    std::string forward_axis = "y";
    std::string target_axis = "y";
    int spin_samples = 12;
    double spin_min = 0.0;
    double spin_max = 2.0 * M_PI;
    bool include_failed = true;
    bool free_joint6 = true;
};

std::string solverShortcut(const std::string& value)
{
    if (value == "kdl") return "kdl_kinematics_plugin/KDLKinematicsPlugin";
    if (value == "bio_ik") return "bio_ik/BioIKKinematicsPlugin";
    return value;
}

Eigen::Vector3d axisVector(const std::string& name)
{
    double sign = 1.0;
    std::string axis = name;
    if (!axis.empty() && axis[0] == '-') {
        sign = -1.0;
        axis = axis.substr(1);
    }
    if (axis == "x") return sign * Eigen::Vector3d::UnitX();
    if (axis == "y") return sign * Eigen::Vector3d::UnitY();
    if (axis == "z") return sign * Eigen::Vector3d::UnitZ();
    throw std::runtime_error("axis must be x/y/z or -x/-y/-z: " + name);
}

std::vector<double> valuesForRange(const Range& range)
{
    std::vector<double> values;
    if (range.step <= 0.0) {
        throw std::runtime_error("range step must be positive");
    }
    for (double value = range.min; value <= range.max + range.step * 0.5; value += range.step) {
        values.push_back(value);
    }
    return values;
}

std::vector<double> spinValues(const Config& config)
{
    std::vector<double> values;
    if (config.spin_samples <= 1) {
        values.push_back(config.spin_min);
        return values;
    }
    const double denom = (std::abs(config.spin_max - config.spin_min - 2.0 * M_PI) < 1e-9)
        ? static_cast<double>(config.spin_samples)
        : static_cast<double>(config.spin_samples - 1);
    for (int i = 0; i < config.spin_samples; ++i) {
        const double t = static_cast<double>(i) / denom;
        values.push_back(config.spin_min + (config.spin_max - config.spin_min) * t);
    }
    return values;
}

Eigen::Matrix3d makeAxisAlignedRotation(const std::string& ee_axis_name,
                                        const std::string& target_axis_name,
                                        double spin)
{
    const Eigen::Vector3d local_axis = axisVector(ee_axis_name).normalized();
    const Eigen::Vector3d target_axis = axisVector(target_axis_name).normalized();
    Eigen::Quaterniond align = Eigen::Quaterniond::FromTwoVectors(local_axis, target_axis);
    Eigen::Matrix3d base_rotation = align.toRotationMatrix();
    Eigen::Matrix3d spin_rotation = Eigen::AngleAxisd(spin, target_axis).toRotationMatrix();
    return spin_rotation * base_rotation;
}

std::string csvBool(bool value)
{
    return value ? "true" : "false";
}

void printHelp()
{
    std::cout
        << "Usage: ik_range_grid [options]\n"
        << "  --group <name>           MoveIt group, default left_arm_with_base\n"
        << "  --solver <name/plugin>   kdl/bio_ik or full plugin\n"
        << "  --urdf <path>            Optional standalone URDF path\n"
        << "  --srdf <path>            Optional SRDF path\n"
        << "  --base-frame <link>      Base frame, default base_link\n"
        << "  --tip-link <link>        IK tip link, default leftjoint6\n"
        << "  --x min max step         X range in base frame\n"
        << "  --y min max step         Y range in base frame\n"
        << "  --z min max step         Z range in base frame\n"
        << "  --timeout <sec>          IK timeout per roll sample\n"
        << "  --forward-axis <axis>    Local EE axis that must face target direction\n"
        << "  --target-axis <axis>     Fixed target direction in base frame\n"
        << "  --spin-samples <n>       Samples around target direction, default 12\n"
        << "  --spin-min <rad>         Start spin angle\n"
        << "  --spin-max <rad>         End spin angle\n"
        << "  --output <csv>           CSV output path\n"
        << "  --success-only           Save only successful positions\n"
        << "  --fix-joint6             Keep legacy random/home joint6 behavior\n";
}

bool parseRange(int& index, int argc, char** argv, Range& range)
{
    if (index + 3 >= argc) return false;
    range.min = std::stod(argv[++index]);
    range.max = std::stod(argv[++index]);
    range.step = std::stod(argv[++index]);
    return true;
}

Config parseArgs(int argc, char** argv)
{
    Config config;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--help") {
            printHelp();
            std::exit(0);
        } else if (arg == "--group" && i + 1 < argc) {
            config.group = argv[++i];
        } else if (arg == "--solver" && i + 1 < argc) {
            config.solver = solverShortcut(argv[++i]);
        } else if (arg == "--urdf" && i + 1 < argc) {
            config.urdf = argv[++i];
        } else if (arg == "--srdf" && i + 1 < argc) {
            config.srdf = argv[++i];
        } else if (arg == "--base-frame" && i + 1 < argc) {
            config.base_frame = argv[++i];
        } else if (arg == "--tip-link" && i + 1 < argc) {
            config.tip_link = argv[++i];
        } else if (arg == "--x") {
            if (!parseRange(i, argc, argv, config.x)) throw std::runtime_error("--x expects min max step");
        } else if (arg == "--y") {
            if (!parseRange(i, argc, argv, config.y)) throw std::runtime_error("--y expects min max step");
        } else if (arg == "--z") {
            if (!parseRange(i, argc, argv, config.z)) throw std::runtime_error("--z expects min max step");
        } else if (arg == "--timeout" && i + 1 < argc) {
            config.timeout = std::stod(argv[++i]);
        } else if (arg == "--forward-axis" && i + 1 < argc) {
            config.forward_axis = argv[++i];
        } else if (arg == "--target-axis" && i + 1 < argc) {
            config.target_axis = argv[++i];
        } else if (arg == "--spin-samples" && i + 1 < argc) {
            config.spin_samples = std::stoi(argv[++i]);
        } else if (arg == "--spin-min" && i + 1 < argc) {
            config.spin_min = std::stod(argv[++i]);
        } else if (arg == "--spin-max" && i + 1 < argc) {
            config.spin_max = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            config.output = argv[++i];
        } else if (arg == "--success-only") {
            config.include_failed = false;
        } else if (arg == "--fix-joint6") {
            config.free_joint6 = false;
        } else {
            throw std::runtime_error("unknown or incomplete argument: " + arg);
        }
    }
    return config;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        Config config = parseArgs(argc, argv);
        IkSolverOptions options;
        options.urdf_path = config.urdf;
        options.srdf_path = config.srdf;
        options.base_frame = config.base_frame;
        options.tip_link = config.tip_link;

        IkSolver ik(config.group, config.solver, config.timeout, config.free_joint6, options);
        const std::vector<double> seed = ik.getHomeSeed();
        const std::vector<double> xs = valuesForRange(config.x);
        const std::vector<double> ys = valuesForRange(config.y);
        const std::vector<double> zs = valuesForRange(config.z);
        const std::vector<double> spins = spinValues(config);
        const std::size_t total = xs.size() * ys.size() * zs.size();

        std::filesystem::path output_path(config.output);
        if (!output_path.parent_path().empty()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::error_code remove_error;
        std::filesystem::remove(output_path, remove_error);
        std::ofstream csv(config.output, std::ios::out | std::ios::trunc);
        if (!csv.good()) {
            throw std::runtime_error("cannot write output CSV: " + config.output);
        }

        csv << "x,y,z,is_success,time_ms,error_code,spin_rad,pos_error,ori_error";
        for (const auto& name : ik.ikJointNames()) {
            csv << "," << name;
        }
        csv << "\n";

        std::cout << "=== IK Range Grid ===\n"
                  << "  group: " << config.group << "\n"
                  << "  solver: " << config.solver << "\n"
                  << "  urdf: " << (config.urdf.empty() ? "<installed/current>" : config.urdf) << "\n"
                  << "  srdf: " << (config.srdf.empty() ? "<installed/current>" : config.srdf) << "\n"
                  << "  base/tip: " << config.base_frame << " -> " << config.tip_link << "\n"
                  << "  x/y/z counts: " << xs.size() << "/" << ys.size() << "/" << zs.size() << " = " << total << "\n"
                  << "  forward: EE " << config.forward_axis << " -> target " << config.target_axis << "\n"
                  << "  spin samples: " << spins.size() << "\n"
                  << "  timeout: " << config.timeout << " s per spin\n"
                  << "  output: " << config.output << "\n";

        std::size_t done = 0;
        std::size_t success_count = 0;
        double total_success_ms = 0.0;
        const auto start = std::chrono::steady_clock::now();

        for (double x : xs) {
            for (double y : ys) {
                for (double z : zs) {
                    ++done;
                    bool success = false;
                    int error_code = 0;
                    double best_ms = 0.0;
                    double best_spin = std::numeric_limits<double>::quiet_NaN();
                    double best_pos_error = 0.0;
                    double best_ori_error = 0.0;
                    std::vector<double> best_joints;

                    for (double spin : spins) {
                        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
                        target.translation() = Eigen::Vector3d(x, y, z);
                        target.linear() = makeAxisAlignedRotation(config.forward_axis, config.target_axis, spin);

                        IkResult result = ik.solve(target, seed, config.timeout);
                        best_ms += result.solve_ms;
                        if (result.success) {
                            success = true;
                            error_code = 1;
                            best_spin = spin;
                            best_pos_error = result.pos_error;
                            best_ori_error = result.ori_error;
                            best_joints = result.joint_values;
                            break;
                        }
                        error_code = -1;
                    }

                    if (success) {
                        ++success_count;
                        total_success_ms += best_ms;
                    }

                    if (success || config.include_failed) {
                        csv << std::fixed << std::setprecision(6)
                            << x << "," << y << "," << z << ","
                            << csvBool(success) << "," << best_ms << "," << error_code << ","
                            << best_spin << "," << best_pos_error << "," << best_ori_error;
                        for (double value : best_joints) {
                            csv << "," << value;
                        }
                        csv << "\n";
                    }

                    if (done % 100 == 0 || done == total) {
                        const auto now = std::chrono::steady_clock::now();
                        const double elapsed = std::chrono::duration<double>(now - start).count();
                        const double rate = 100.0 * static_cast<double>(success_count) / static_cast<double>(done);
                        const double eta = done > 0 ? elapsed / static_cast<double>(done) * static_cast<double>(total - done) : 0.0;
                        std::cout << "  progress " << done << "/" << total
                                  << " success " << success_count << " (" << std::setprecision(1) << std::fixed << rate << "%)"
                                  << " elapsed " << std::setprecision(1) << elapsed << "s"
                                  << " eta " << eta << "s\n";
                    }
                }
            }
        }

        const auto end = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(end - start).count();
        std::cout << "=== Summary ===\n"
                  << "  total points: " << total << "\n"
                  << "  success: " << success_count << " (" << std::setprecision(2) << std::fixed
                  << (100.0 * static_cast<double>(success_count) / static_cast<double>(total)) << "%)\n"
                  << "  elapsed: " << elapsed << " s\n";
        if (success_count > 0) {
            std::cout << "  avg success query time: " << (total_success_ms / static_cast<double>(success_count)) << " ms\n";
        }
        std::cout << "  saved: " << config.output << "\n";
    } catch (const std::exception& exc) {
        std::cerr << "ik_range_grid error: " << exc.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
