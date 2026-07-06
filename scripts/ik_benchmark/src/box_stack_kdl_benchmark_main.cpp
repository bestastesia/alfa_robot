#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

using ik_benchmark::IkResult;
using ik_benchmark::IkSolver;
using ik_benchmark::IkSolverOptions;

namespace {

struct BoxSpec {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct PickPair {
    int round = 0;
    int left_box = 0;
    int right_box = 0;
};

Eigen::Isometry3d poseForwardX(double x, double y, double z)
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(x, y, z);
    Eigen::Quaterniond q(0.7071, 0.0, 0.7071, 0.0); // xyzw=(0,0.7071,0,0.7071), tool +Z -> world +X
    q.normalize();
    tf.linear() = q.toRotationMatrix();
    return tf;
}

std::vector<double> degSeed(const std::vector<double>& degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (double degree : degrees) radians.push_back(degree * M_PI / 180.0);
    return radians;
}

std::map<int, BoxSpec> makeBoxes(double x_offset)
{
    const double base_x = 0.375 + x_offset;
    const std::vector<std::vector<std::pair<int, double>>> rows_top_to_bottom = {
        {{1, 0.4}, {2, 0.0}, {4, -0.4}},
        {{3, 0.4}, {5, 0.0}, {6, -0.4}},
        {{7, 0.0}, {8, -0.4}, {10, -0.8}},
        {{9, 0.4}, {11, 0.0}, {12, -0.4}},
        {{13, 0.4}, {15, 0.0}, {14, -0.4}},
    };
    std::map<int, BoxSpec> boxes;
    for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
        const double z = 0.2 + 0.4 * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
        for (const auto& [id, y] : rows_top_to_bottom[row]) {
            boxes[id] = BoxSpec{id, base_x, y, z};
        }
    }
    return boxes;
}

std::vector<PickPair> makePickPairs()
{
    return {
        {1, 1, 2},
        {2, 3, 4},
        {3, 5, 6},
        {4, 7, 8},
        {5, 9, 10},
        {6, 11, 12},
    };
}

std::vector<std::string> fullJointNames()
{
    return {
        "updown",
        "leftjoint1", "leftjoint2", "leftjoint3", "leftjoint4", "leftjoint5", "leftjoint6",
        "rightjoint1", "rightjoint2", "rightjoint3", "rightjoint4", "rightjoint5", "rightjoint6",
    };
}

std::vector<double> fullValues(double updown, const std::vector<double>& left, const std::vector<double>& right)
{
    std::vector<double> values;
    values.reserve(13);
    values.push_back(updown);
    values.insert(values.end(), left.begin(), left.end());
    values.insert(values.end(), right.begin(), right.end());
    return values;
}

void printHelp()
{
    std::cout << "box_stack_kdl_benchmark\n"
              << "  --x-offset <m>          box x = 0.375 + x_offset, default 0.0\n"
              << "  --grasp-updown <m>      updown for pregrasp/grasp IK, default 0.45\n"
              << "  --load-place-updown <m> updown for loaded grasp/place fixed states, default 0.45\n"
              << "  --timeout <sec>         KDL timeout, default 0.01\n"
              << "  --output <path>         JSONL output path\n"
              << "  --allow-collisions      Do not reject combined full-state collision\n";
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    double x_offset = 0.0;
    double grasp_updown = 0.45;
    double load_place_updown = 0.45;
    double timeout = 0.01;
    bool reject_collisions = true;
    std::string output = "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/box_stack_kdl/box_stack_kdl_benchmark.jsonl";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--x-offset" && i + 1 < argc) x_offset = std::stod(argv[++i]);
        else if (arg == "--grasp-updown" && i + 1 < argc) grasp_updown = std::stod(argv[++i]);
        else if (arg == "--load-place-updown" && i + 1 < argc) load_place_updown = std::stod(argv[++i]);
        else if (arg == "--timeout" && i + 1 < argc) timeout = std::stod(argv[++i]);
        else if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--allow-collisions") reject_collisions = false;
        else if (arg == "--help" || arg == "-h") { printHelp(); rclcpp::shutdown(); return 0; }
    }

    IkSolverOptions left_options;
    left_options.base_frame = "left_arm_base";
    left_options.tip_link = "left_tool0";
    left_options.reject_collisions = false;
    IkSolverOptions right_options;
    right_options.base_frame = "right_arm_base";
    right_options.tip_link = "right_tool0";
    right_options.reject_collisions = false;
    IkSolverOptions scene_options;
    scene_options.reject_collisions = false;
    scene_options.enforce_arm_base_collisions = true;

    IkSolver left_ik("left_arm", "kdl_kinematics_plugin/KDLKinematicsPlugin", timeout, false, left_options);
    IkSolver right_ik("right_arm", "kdl_kinematics_plugin/KDLKinematicsPlugin", timeout, false, right_options);
    IkSolver scene_checker("dual_arm_with_base", "bio_ik/BioIKKinematicsPlugin", timeout, false, scene_options);

    const auto boxes = makeBoxes(x_offset);
    const auto pairs = makePickPairs();
    const auto all_joint_names = fullJointNames();
    const auto pregrasp_seed = degSeed({0, 15, 135, 0, 60, 0});
    const auto loaded_seed = degSeed({0, 5, 145, 0, 120, 0});
    const auto place_seed = degSeed({0, -90, -90, 0, -90, 180});

    std::filesystem::path output_path(output);
    if (!output_path.parent_path().empty()) std::filesystem::create_directories(output_path.parent_path());
    std::ofstream ofs(output);
    if (!ofs) {
        std::cerr << "Failed to open output: " << output << "\n";
        rclcpp::shutdown();
        return 2;
    }

    nlohmann::json header = {
        {"type", "header"},
        {"schema", "box_stack_kdl_benchmark_v1"},
        {"box_x", 0.375 + x_offset},
        {"x_offset", x_offset},
        {"grasp_updown", grasp_updown},
        {"load_place_updown", load_place_updown},
        {"timeout", timeout},
        {"reject_collisions", reject_collisions},
        {"orientation", "tool +Z toward world +X"},
        {"rounds", pairs.size()}
    };
    ofs << header.dump() << "\n";

    size_t success_rounds = 0;
    double total_left_ms = 0.0;
    double total_right_ms = 0.0;

    std::cout << "=== Box Stack KDL Benchmark ===\n"
              << "  box_x=" << (0.375 + x_offset) << " grasp_updown=" << grasp_updown
              << " load/place_updown=" << load_place_updown << " timeout=" << timeout << "s\n"
              << "  output=" << output << "\n\n";

    for (const auto& pair : pairs) {
        const BoxSpec& left_box = boxes.at(pair.left_box);
        const BoxSpec& right_box = boxes.at(pair.right_box);
        auto t0 = std::chrono::steady_clock::now();
        IkResult left = left_ik.solve(poseForwardX(left_box.x, left_box.y, left_box.z), pregrasp_seed, timeout);
        IkResult right = right_ik.solve(poseForwardX(right_box.x, right_box.y, right_box.z), pregrasp_seed, timeout);
        auto t1 = std::chrono::steady_clock::now();

        total_left_ms += left.solve_ms;
        total_right_ms += right.solve_ms;
        bool collision_free = false;
        std::vector<std::string> collision_pairs;
        if (left.success && right.success) {
            collision_free = scene_checker.isNamedStateCollisionFree(all_joint_names,
                fullValues(grasp_updown, left.joint_values, right.joint_values), &collision_pairs);
        }
        const bool round_success = left.success && right.success && (!reject_collisions || collision_free);
        success_rounds += static_cast<size_t>(round_success);
        const double wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        nlohmann::json record = {
            {"type", "round"},
            {"round", pair.round},
            {"left_box", pair.left_box},
            {"right_box", pair.right_box},
            {"left_target", {left_box.x, left_box.y, left_box.z}},
            {"right_target", {right_box.x, right_box.y, right_box.z}},
            {"pregrasp_updown", grasp_updown},
            {"loaded_updown", load_place_updown},
            {"place_updown", load_place_updown},
            {"left_success", left.success},
            {"right_success", right.success},
            {"collision_free", collision_free},
            {"success", round_success},
            {"left_solve_ms", left.solve_ms},
            {"right_solve_ms", right.solve_ms},
            {"wall_ms", wall_ms},
            {"left_pos_error", left.pos_error},
            {"right_pos_error", right.pos_error},
            {"left_ori_error", left.ori_error},
            {"right_ori_error", right.ori_error},
            {"collision_pairs", collision_pairs},
            {"left_joint_values", left.joint_values},
            {"right_joint_values", right.joint_values},
            {"pregrasp_left_degrees", {0, 15, 135, 0, 60, 0}},
            {"loaded_left_degrees", {0, 5, 145, 0, 120, 0}},
            {"place_left_degrees", {0, -90, -90, 0, -90, 180}}
        };
        ofs << record.dump() << "\n";

        std::cout << "round " << pair.round << " boxes " << pair.left_box << "+" << pair.right_box
                  << " left=" << (left.success ? "ok" : "fail")
                  << " right=" << (right.success ? "ok" : "fail")
                  << " collision=" << (collision_free ? "free" : "blocked")
                  << " success=" << (round_success ? "yes" : "no")
                  << " wall=" << std::fixed << std::setprecision(2) << wall_ms << "ms\n";
    }

    nlohmann::json summary = {
        {"type", "summary"},
        {"rounds", pairs.size()},
        {"success_rounds", success_rounds},
        {"failed_rounds", pairs.size() - success_rounds},
        {"total_left_solve_ms", total_left_ms},
        {"total_right_solve_ms", total_right_ms}
    };
    ofs << summary.dump() << "\n";
    std::cout << "\nSuccess: " << success_rounds << "/" << pairs.size() << "\n";
    std::cout << "Results saved to: " << output << "\n";

    rclcpp::shutdown();
    return success_rounds == pairs.size() ? 0 : 2;
}
