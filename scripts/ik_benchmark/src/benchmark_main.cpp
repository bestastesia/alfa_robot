#include "ik_benchmark/ik_solver.h"

#include <nlohmann/json.hpp>
#include <random>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace ik_benchmark;

// 末端位姿空间加扰动：在位置上加高斯噪声，在旋转上加小角度轴角扰动
static Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose,
                                      double pos_noise, double ori_noise,
                                      std::mt19937& rng)
{
    Eigen::Isometry3d result = pose;

    // 位置扰动
    std::normal_distribution<double> pos_dist(0.0, pos_noise);
    result.translation().x() += pos_dist(rng);
    result.translation().y() += pos_dist(rng);
    result.translation().z() += pos_dist(rng);

    // 旋转扰动：小角度轴角
    if (ori_noise > 0) {
        std::normal_distribution<double> ori_dist(0.0, ori_noise);
        Eigen::Vector3d axis(ori_dist(rng), ori_dist(rng), ori_dist(rng));
        double angle = axis.norm();
        if (angle > 1e-8) {
            axis.normalize();
            Eigen::AngleAxisd delta(angle, axis);
            result.linear() = result.linear() * delta.toRotationMatrix();
        }
    }

    return result;
}

// Eigen::Isometry3d → JSON
static nlohmann::json poseToJson(const Eigen::Isometry3d& pose)
{
    Eigen::Quaterniond q(pose.linear());
    return {
        {"position", {pose.translation().x(), pose.translation().y(), pose.translation().z()}},
        {"orientation", {q.x(), q.y(), q.z(), q.w()}}
    };
}

// IkResult → JSON
static nlohmann::json resultToJson(const IkResult& r)
{
    return {
        {"success", r.success},
        {"solve_ms", r.solve_ms},
        {"pos_error", r.pos_error},
        {"ori_error", r.ori_error},
        {"joint_values", r.joint_values}
    };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 参数
    std::string group  = "left_arm";
    std::string solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    int samples        = 20;
    double timeout     = 2.0;
    double pos_noise   = 0.01;   // 末端位置扰动标准差 (m)
    double ori_noise   = 0.05;   // 末端旋转扰动标准差 (rad)
    bool free_joint6   = false;
    std::string output = "/tmp/ik_benchmark.jsonl";

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--group" && i+1 < argc)        group       = argv[++i];
        else if (arg == "--solver" && i+1 < argc)  solver      = argv[++i];
        else if (arg == "--samples" && i+1 < argc) samples     = std::stoi(argv[++i]);
        else if (arg == "--timeout" && i+1 < argc) timeout     = std::stod(argv[++i]);
        else if (arg == "--pos-noise" && i+1 < argc) pos_noise = std::stod(argv[++i]);
        else if (arg == "--ori-noise" && i+1 < argc) ori_noise = std::stod(argv[++i]);
        else if (arg == "--free-joint6")            free_joint6 = true;
        else if (arg == "--output" && i+1 < argc)   output      = argv[++i];
        else if (arg == "--help") {
            std::cout << "Usage: ik_benchmark [options]\n"
                      << "  --group <name>       Joint group (left_arm/right_arm/dual_arm_with_base)\n"
                      << "  --solver <plugin>    IK solver plugin\n"
                      << "  --samples <n>        Number of test samples\n"
                      << "  --timeout <s>        IK timeout per solve\n"
                      << "  --pos-noise <m>      Position perturbation std (m)\n"
                      << "  --ori-noise <rad>    Orientation perturbation std (rad)\n"
                      << "  --free-joint6        Don't fix joint6 to 0\n"
                      << "  --output <path>      JSONL output path\n";
            return 0;
        }
    }

    std::cout << "=== IK Benchmark ===\n"
              << "  Group:   " << group << "\n"
              << "  Solver:  " << solver << "\n"
              << "  Samples: " << samples << "\n"
              << "  Timeout: " << timeout << "s\n"
              << "  Pos noise: " << pos_noise << "m\n"
              << "  Ori noise: " << ori_noise << "rad\n"
              << "  Free joint6: " << (free_joint6 ? "yes" : "no") << "\n"
              << "  Output:  " << output << "\n\n";

    IkSolver ik(group, solver, timeout, free_joint6);
    std::mt19937 rng(42);

    int success = 0;
    double total_pos_err = 0, total_ori_err = 0, total_ms = 0;

    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write to " << output << "\n";
        return 1;
    }

    for (int i = 0; i < samples; ++i) {
        // 1. 随机关节 → FK 得到末端位姿
        auto seed_joints = ik.getRandomSeed();
        auto fk_poses = ik.fk(seed_joints);

        // 2. 在末端位姿空间加扰动
        Eigen::Isometry3d target, target2;
        target = perturbPose(fk_poses[0], pos_noise, ori_noise, rng);

        IkResult result;
        nlohmann::json record;
        record["sample"]      = i;
        record["group"]       = group;
        record["solver"]      = solver;
        record["seed_joints"] = seed_joints;
        record["fk_pose"]     = poseToJson(fk_poses[0]);
        record["target_pose"] = poseToJson(target);

        if (ik.isDualArm()) {
            target2 = perturbPose(fk_poses[1], pos_noise, ori_noise, rng);
            result = ik.solveDual(target, target2, ik.getHomeSeed(), timeout);
            record["fk_pose2"]    = poseToJson(fk_poses[1]);
            record["target_pose2"] = poseToJson(target2);
        } else {
            result = ik.solve(target, ik.getHomeSeed(), timeout);
        }

        record["result"] = resultToJson(result);
        if (result.success) {
            // 记录 IK 求解后的实际末端位姿
            auto actual_poses = ik.fk(result.joint_values);
            record["actual_pose"] = poseToJson(actual_poses[0]);
            if (ik.isDualArm()) {
                record["actual_pose2"] = poseToJson(actual_poses[1]);
            }
        }

        ofs << record.dump() << "\n";

        if (result.success) {
            success++;
            total_pos_err += result.pos_error;
            total_ori_err += result.ori_error;
            total_ms += result.solve_ms;
            std::cout << "[" << i << "] OK  " << result.solve_ms << "ms  "
                      << "pos_err=" << result.pos_error << "  ori_err=" << result.ori_error << "\n";
        } else {
            std::cout << "[" << i << "] FAIL\n";
        }
    }

    ofs.close();

    std::cout << "\n=== Summary ===\n"
              << "  Success: " << success << "/" << samples
              << " (" << (100.0 * success / samples) << "%)\n";
    if (success > 0) {
        std::cout << "  Avg pos error: " << (total_pos_err / success) << "m\n"
                  << "  Avg ori error: " << (total_ori_err / success) << "rad\n"
                  << "  Avg solve time: " << (total_ms / success) << "ms\n";
    }
    std::cout << "  Results saved to: " << output << "\n";

    rclcpp::shutdown();
    return 0;
}
