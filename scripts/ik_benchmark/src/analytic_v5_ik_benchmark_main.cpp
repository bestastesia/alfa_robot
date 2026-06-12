#include "ik_benchmark/analytic_v5_ik_solver.h"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

namespace {

std::vector<double> degSeed(const std::vector<double>& degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (double degree : degrees) radians.push_back(degree * M_PI / 180.0);
    return radians;
}

void printHelp()
{
    std::cout << "analytic_v5_ik_benchmark\n"
              << "  --side left|right       Arm side, default left\n"
              << "  --samples <n>           FK->IK random samples, default 20\n"
              << "  --output <path>         JSONL output\n"
              << "  --position-tol <m>      FK verification position tolerance, default 0.002\n"
              << "  --orientation-tol <rad> FK verification orientation tolerance, default 0.01\n";
}

} // namespace

int main(int argc, char** argv)
{
    ik_benchmark::AnalyticV5IkConfig config;
    size_t samples = 20;
    std::string output = "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/analytic_v5_ik/analytic_v5_ik_benchmark.jsonl";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--side" && i + 1 < argc) {
            const std::string side = argv[++i];
            config.side = side == "right" ? ik_benchmark::V5ArmSide::Right : ik_benchmark::V5ArmSide::Left;
        } else if (arg == "--samples" && i + 1 < argc) {
            samples = static_cast<size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) {
            output = argv[++i];
        } else if (arg == "--position-tol" && i + 1 < argc) {
            config.position_tolerance = std::stod(argv[++i]);
        } else if (arg == "--orientation-tol" && i + 1 < argc) {
            config.orientation_tolerance = std::stod(argv[++i]);
        } else if (arg == "--q1-step" && i + 1 < argc) {
            config.q1_scan_step = std::stod(argv[++i]);
        } else if (arg == "--debug") {
            config.debug = true;
        } else if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        }
    }

    ik_benchmark::AnalyticV5IkSolver solver(config);

    std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) std::filesystem::create_directories(output_path.parent_path());
    std::ofstream ofs(output);
    if (!ofs) {
        std::cerr << "failed to open output: " << output << "\n";
        return 2;
    }

    nlohmann::json header = {
        {"type", "header"},
        {"schema", "analytic_v5_ik_benchmark_v0"},
        {"side", config.side == ik_benchmark::V5ArmSide::Left ? "left" : "right"},
        {"samples", samples},
        {"position_tolerance", config.position_tolerance},
        {"orientation_tolerance", config.orientation_tolerance},
        {"note", "Analytic V5 benchmark: q1 grid, closed-form wrist orientation, true-URDF 2R position closure, FK verification. KDL/BioIK benchmarks are unchanged."}
    };
    ofs << header.dump() << "\n";

    std::mt19937 rng(42);
    std::vector<std::vector<double>> test_joints = {
        degSeed({0, -90, 135, 45, 0, 0}),
        degSeed({0, -60, 120, 90, 0, 0}),
        degSeed({0, 15, 135, 0, 60, 0}),
        degSeed({20, -70, 125, 80, 5, 20}),
        degSeed({-20, -85, 135, 40, 0, -20}),
        degSeed({35, 10, 120, 15, 45, 35}),
    };
    while (test_joints.size() < samples) {
        const auto base = test_joints[test_joints.size() % 6];
        std::normal_distribution<double> noise(0.0, 0.08);
        std::vector<double> perturbed = base;
        for (double& value : perturbed) value += noise(rng);
        test_joints.push_back(std::move(perturbed));
    }

    size_t success = 0;
    double total_ms = 0.0;
    std::cout << "=== Analytic V5 IK Benchmark ===\n"
              << "  side=" << (config.side == ik_benchmark::V5ArmSide::Left ? "left" : "right")
              << " samples=" << samples << " output=" << output << "\n";

    for (size_t i = 0; i < samples; ++i) {
        const auto target = solver.fk(test_joints[i]);
        if (config.debug && i == 0) {
            std::cerr << "debug target finite=" << target.matrix().allFinite()
                      << " p=" << target.translation().transpose() << "\n"
                      << "R=\n" << target.linear() << "\n";
        }
        const auto t0 = std::chrono::steady_clock::now();
        const auto solutions = solver.solve(target);
        const auto t1 = std::chrono::steady_clock::now();
        const double wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        total_ms += wall_ms;
        const bool ok = !solutions.empty();
        success += static_cast<size_t>(ok);

        nlohmann::json rec = {
            {"type", "sample"},
            {"sample", i},
            {"success", ok},
            {"candidate_count", solutions.size()},
            {"wall_ms", wall_ms},
            {"seed_joint_values", test_joints[i]},
        };
        if (ok) {
            rec["selected_joint_values"] = solutions.front().joint_values;
            rec["selected_branch"] = solutions.front().branch;
            rec["position_error"] = solutions.front().position_error;
            rec["orientation_error"] = solutions.front().orientation_error;
            rec["score"] = solutions.front().score;
        }
        ofs << rec.dump() << "\n";
        std::cout << "sample " << std::setw(2) << i << " " << (ok ? "OK" : "FAIL")
                  << " candidates=" << solutions.size()
                  << " wall=" << std::fixed << std::setprecision(2) << wall_ms << "ms";
        if (ok) {
            std::cout << " pos_err=" << solutions.front().position_error
                      << " ori_err=" << solutions.front().orientation_error;
        }
        std::cout << "\n";
    }

    nlohmann::json summary = {
        {"type", "summary"},
        {"samples", samples},
        {"success", success},
        {"success_rate", samples == 0 ? 0.0 : static_cast<double>(success) / static_cast<double>(samples)},
        {"total_wall_ms", total_ms},
        {"avg_wall_ms", samples == 0 ? 0.0 : total_ms / static_cast<double>(samples)}
    };
    ofs << summary.dump() << "\n";
    std::cout << "Success: " << success << "/" << samples << " avg_wall="
              << (samples == 0 ? 0.0 : total_ms / static_cast<double>(samples)) << "ms\n";
    return success == samples ? 0 : 2;
}
