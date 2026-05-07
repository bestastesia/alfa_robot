/**
 * ik_demo — 单次 IK 求解 demo
 *
 * 流程: 随机关节 → FK → 加噪 → IK → 对比
 * 输出 JSON 结果到 stdout，方便 Python 解析。
 *
 * 用法:
 *   ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver trac_ik
 *   ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver pick_ik
 *   ros2 run alfa_robot_benchmarks ik_demo --group right_arm --solver kdl --perturb-pos 0.05
 */

#include "ik_benchmark/ik_solver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <rclcpp/rclcpp.hpp>

// ── CLI ────────────────────────────────────────────────────────────────────

struct Args {
    std::string group = "left_arm";
    std::string solver = "kdl";
    double perturb_pos = 0.05;
    double perturb_ori = 0.25;
    double timeout = 2.0;
    int seed = 42;
    bool json = true;
};

static Args parse(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--group"       && i+1 < argc) a.group       = argv[++i];
        if (s == "--solver"      && i+1 < argc) a.solver      = argv[++i];
        if (s == "--perturb-pos" && i+1 < argc) a.perturb_pos = std::stod(argv[++i]);
        if (s == "--perturb-ori" && i+1 < argc) a.perturb_ori = std::stod(argv[++i]);
        if (s == "--timeout"     && i+1 < argc) a.timeout     = std::stod(argv[++i]);
        if (s == "--seed"        && i+1 < argc) a.seed        = std::stoi(argv[++i]);
        if (s == "--no-json")                     a.json       = false;
    }
    // solver 快捷名
    if (a.solver == "kdl")     a.solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    if (a.solver == "trac_ik") a.solver = "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin";
    if (a.solver == "pick_ik") a.solver = "pick_ik/PickIkPlugin";
    if (a.solver == "bio_ik")  a.solver = "bio_ik/BioIKKinematicsPlugin";
    return a;
}

// ── 扰动 ──────────────────────────────────────────────────────────────────

static Eigen::Isometry3d perturb(const Eigen::Isometry3d& fk,
                                  double pos_amp, double ori_amp,
                                  std::mt19937& rng)
{
    Eigen::Isometry3d result = fk;
    std::uniform_real_distribution<double> d(-1.0, 1.0);
    result.translation().x() += d(rng) * pos_amp;
    result.translation().y() += d(rng) * pos_amp;
    result.translation().z() += d(rng) * pos_amp;
    if (ori_amp > 0) {
        double angle = d(rng) * ori_amp;
        Eigen::Vector3d axis(d(rng), d(rng), d(rng));
        if (axis.norm() < 1e-6) axis = Eigen::Vector3d::UnitZ();
        axis.normalize();
        result.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix() * result.linear();
    }
    return result;
}

// ── JSON 辅助 ─────────────────────────────────────────────────────────────

static std::string json_pose(const Eigen::Isometry3d& tf)
{
    Eigen::Quaterniond q(tf.linear());
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "{\"pos\":[" << tf.translation().x()
       << "," << tf.translation().y()
       << "," << tf.translation().z() << "]"
       << ",\"quat\":[" << q.x() << "," << q.y()
       << "," << q.z() << "," << q.w() << "]}";
    return os.str();
}

static std::string json_array(const std::vector<double>& v)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ", ";
        os << std::fixed << std::setprecision(8) << v[i];
    }
    os << "]";
    return os.str();
}

static std::string json_str_array(const std::vector<std::string>& v)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ", ";
        os << "\"" << v[i] << "\"";
    }
    os << "]";
    return os.str();
}

// ── 主 ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto args = parse(argc, argv);

    try {
        // 自动定位 URDF / SRDF
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("alfa_robot_description");
        std::string urdf_path = pkg_dir + "/urdf/alfa_robot.urdf";
        std::string moveit_dir = ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");
        std::string srdf_path = moveit_dir + "/config/alfa_robot.srdf";

        ik_benchmark::IkSolver solver(urdf_path, srdf_path, args.group, args.solver, args.timeout);

        std::mt19937 rng(args.seed);
        auto random_joints = solver.getRandomSeed();

        // FK
        auto fk_poses = solver.fk(random_joints);

        // 加噪
        Eigen::Isometry3d target_left = perturb(fk_poses[0], args.perturb_pos, args.perturb_ori, rng);
        Eigen::Isometry3d target_right = Eigen::Isometry3d::Identity();
        if (solver.isDualArm() && fk_poses.size() > 1) {
            target_right = perturb(fk_poses[1], args.perturb_pos, args.perturb_ori, rng);
        }

        // IK
        ik_benchmark::IkResult result;
        if (solver.isDualArm()) {
            result = solver.solveDual(target_left, target_right);
        } else {
            result = solver.solve(target_left);
        }

        // JSON 输出
        if (args.json) {
            std::cout << "{"
                << "\"group\":\"" << args.group << "\""
                << ",\"solver\":\"" << args.solver << "\""
                << ",\"success\":" << (result.success ? "true" : "false")
                << ",\"solve_ms\":" << std::fixed << std::setprecision(2) << result.solve_ms
                << ",\"pos_error\":" << std::setprecision(8) << result.pos_error
                << ",\"ori_error\":" << result.ori_error
                << ",\"joint_names\":" << json_str_array(result.joint_names)
                << ",\"joint_values\":" << (result.success ? json_array(result.joint_values) : "[]")
                << ",\"fk_joints\":" << json_array(random_joints)
                << ",\"target_left\":" << json_pose(target_left)
                << ",\"target_right\":" << (solver.isDualArm() ? json_pose(target_right) : "null")
                << ",\"fk_left\":" << json_pose(fk_poses[0])
                << ",\"fk_right\":" << (solver.isDualArm() && fk_poses.size() > 1 ? json_pose(fk_poses[1]) : "null")
                << "}\n";
        } else {
            std::cout << "\n=== IK Demo ===\n"
                      << "Group:  " << args.group << "\n"
                      << "Solver: " << args.solver << "\n"
                      << "Result: " << (result.success ? "SUCCESS" : "FAILED") << "\n";
            if (result.success) {
                std::cout << "Time:   " << result.solve_ms << " ms\n"
                          << "Pos err: " << result.pos_error << " m\n"
                          << "Ori err: " << result.ori_error << " rad\n"
                          << "Joints: ";
                for (size_t i = 0; i < result.joint_names.size(); ++i)
                    std::cout << result.joint_names[i] << "=" << result.joint_values[i] << " ";
                std::cout << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
