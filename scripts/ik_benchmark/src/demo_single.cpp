#include "ik_benchmark/ik_solver.h"

#include <nlohmann/json.hpp>
#include <random>
#include <iostream>
#include <fstream>

using namespace ik_benchmark;

// 末端位姿空间加扰动
static Eigen::Isometry3d perturbPose(const Eigen::Isometry3d& pose,
                                      double pos_noise, double ori_noise,
                                      std::mt19937& rng)
{
    Eigen::Isometry3d result = pose;
    std::normal_distribution<double> pos_dist(0.0, pos_noise);
    result.translation().x() += pos_dist(rng);
    result.translation().y() += pos_dist(rng);
    result.translation().z() += pos_dist(rng);
    if (ori_noise > 0) {
        std::normal_distribution<double> ori_dist(0.0, ori_noise);
        Eigen::Vector3d axis(ori_dist(rng), ori_dist(rng), ori_dist(rng));
        double angle = axis.norm();
        if (angle > 1e-8) {
            axis.normalize();
            result.linear() = result.linear() * Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }
    }
    return result;
}

static nlohmann::json poseToJson(const Eigen::Isometry3d& pose)
{
    Eigen::Quaterniond q(pose.linear());
    return {
        {"position", {pose.translation().x(), pose.translation().y(), pose.translation().z()}},
        {"orientation", {q.x(), q.y(), q.z(), q.w()}}
    };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string group  = "left_arm";
    std::string solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    double timeout     = 2.0;
    double pos_noise   = 0.01;
    double ori_noise   = 0.05;
    bool free_joint6   = false;
    bool no_perturb    = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--group" && i+1 < argc)       group       = argv[++i];
        else if (arg == "--solver" && i+1 < argc) solver      = argv[++i];
        else if (arg == "--timeout" && i+1 < argc) timeout     = std::stod(argv[++i]);
        else if (arg == "--pos-noise" && i+1 < argc) pos_noise = std::stod(argv[++i]);
        else if (arg == "--ori-noise" && i+1 < argc) ori_noise = std::stod(argv[++i]);
        else if (arg == "--free-joint6")           free_joint6 = true;
        else if (arg == "--no-perturb")            no_perturb  = true;
    }

    std::cout << "=== IK Demo ===\n"
              << "  Group: " << group << "  Solver: " << solver << "\n"
              << "  Timeout: " << timeout << "s\n"
              << "  Pos noise: " << pos_noise << "m  Ori noise: " << ori_noise << "rad\n"
              << "  Free joint6: " << (free_joint6 ? "yes" : "no") << "\n"
              << "  No perturb: " << (no_perturb ? "yes" : "no") << "\n\n";

    IkSolver ik(group, solver, timeout, free_joint6);
    std::mt19937 rng(42);

    // 随机关节 → FK → 扰动 → IK → 输出 JSON
    auto seed_joints = ik.getRandomSeed();
    auto fk_poses = ik.fk(seed_joints);

    Eigen::Isometry3d target, target2;
    if (no_perturb) {
        target = fk_poses[0];
        if (ik.isDualArm()) target2 = fk_poses[1];
    } else {
        target = perturbPose(fk_poses[0], pos_noise, ori_noise, rng);
        if (ik.isDualArm()) target2 = perturbPose(fk_poses[1], pos_noise, ori_noise, rng);
    }

    IkResult result;
    if (ik.isDualArm()) {
        result = ik.solveDual(target, target2, ik.getHomeSeed(), timeout);
    } else {
        result = ik.solve(target, ik.getHomeSeed(), timeout);
    }

    // JSON 输出
    nlohmann::json out;
    out["group"]       = group;
    out["solver"]      = solver;
    out["seed_joints"] = seed_joints;
    out["fk_pose"]     = poseToJson(fk_poses[0]);
    out["target_pose"] = poseToJson(target);
    if (ik.isDualArm()) {
        out["fk_pose2"]    = poseToJson(fk_poses[1]);
        out["target_pose2"] = poseToJson(target2);
    }

    out["success"]    = result.success;
    out["solve_ms"]   = result.solve_ms;
    out["pos_error"]  = result.pos_error;
    out["ori_error"]  = result.ori_error;
    out["joint_values"] = result.joint_values;

    if (result.success) {
        auto actual_poses = ik.fk(result.joint_values);
        out["actual_pose"] = poseToJson(actual_poses[0]);
        if (ik.isDualArm()) {
            out["actual_pose2"] = poseToJson(actual_poses[1]);
        }
    }

    std::cout << out.dump(2) << "\n";

    rclcpp::shutdown();
    return 0;
}