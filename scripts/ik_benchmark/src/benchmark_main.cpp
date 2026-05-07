/**
 * ik_benchmark — 批量 IK 求解器性能测试
 *
 * 用法:
 *   ros2 run alfa_robot_benchmarks ik_benchmark --group dual_arm_with_base --solver pick_ik --samples 20
 *   ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver trac_ik --samples 50
 */

#include "ik_benchmark/ik_solver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <rclcpp/rclcpp.hpp>

struct Args {
    std::string group = "dual_arm_with_base";
    std::string solver = "pick_ik";
    int    samples = 200;
    double timeout = 2.0;
    double pos_thresh = 0.005;
    double ori_thresh = 0.01;
    double perturb_pos = 0.05;
    double perturb_ori = 0.25;
    bool   verbose = false;
    std::string jsonl_path;
};

static Args parse(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--group"       && i+1 < argc) a.group       = argv[++i];
        if (s == "--solver"      && i+1 < argc) a.solver      = argv[++i];
        if (s == "--samples"     && i+1 < argc) a.samples     = std::stoi(argv[++i]);
        if (s == "--timeout"     && i+1 < argc) a.timeout     = std::stod(argv[++i]);
        if (s == "--pos-thresh"  && i+1 < argc) a.pos_thresh  = std::stod(argv[++i]);
        if (s == "--ori-thresh"  && i+1 < argc) a.ori_thresh  = std::stod(argv[++i]);
        if (s == "--perturb-pos" && i+1 < argc) a.perturb_pos = std::stod(argv[++i]);
        if (s == "--perturb-ori" && i+1 < argc) a.perturb_ori = std::stod(argv[++i]);
        if (s == "--jsonl"       && i+1 < argc) a.jsonl_path  = argv[++i];
        if (s == "--verbose")                     a.verbose     = true;
    }
    if (a.solver == "kdl")     a.solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    if (a.solver == "trac_ik") a.solver = "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin";
    if (a.solver == "pick_ik") a.solver = "pick_ik/PickIkPlugin";
    if (a.solver == "bio_ik")  a.solver = "bio_ik/BioIKKinematicsPlugin";
    return a;
}

enum class Status { kSuccess, kLargeError, kFailed };

struct SampleResult {
    int index;
    Status status;
    double solve_ms;
    double pos_error;
    double ori_error;
};

struct Summary {
    std::string name;
    int total = 0, success = 0, large_error = 0, failed = 0;
    double sum_ms = 0, max_ms = 0;
    double sum_pos = 0, max_pos = 0;
    double sum_ori = 0, max_ori = 0;

    void add(const SampleResult& r) {
        ++total;
        sum_ms += r.solve_ms;
        max_ms = std::max(max_ms, r.solve_ms);
        if (r.status == Status::kFailed) { ++failed; }
        else {
            sum_pos += r.pos_error; max_pos = std::max(max_pos, r.pos_error);
            sum_ori += r.ori_error; max_ori = std::max(max_ori, r.ori_error);
            if (r.status == Status::kSuccess) ++success;
            else ++large_error;
        }
    }

    void print() const {
        int solved = success + large_error;
        double avg_ms  = total  > 0 ? sum_ms  / total  : 0.0;
        double avg_pos = solved > 0 ? sum_pos / solved  : 0.0;
        double avg_ori = solved > 0 ? sum_ori / solved  : 0.0;
        double pct_ok  = total  > 0 ? 100.0 * success / total : 0.0;
        std::cout
            << "\n=== " << name << " ===\n"
            << "  总样本数      : " << total << "\n"
            << "  成功（达标）  : " << success
            << "  (" << std::fixed << std::setprecision(1) << pct_ok << "%)\n"
            << "  成功（误差大）: " << large_error << "\n"
            << "  失败          : " << failed << "\n"
            << "  平均耗时      : " << std::fixed << std::setprecision(2) << avg_ms << " ms\n"
            << "  最大耗时      : " << max_ms << " ms\n"
            << "  平均位置误差  : " << std::scientific << std::setprecision(3) << avg_pos << " m\n"
            << "  最大位置误差  : " << max_pos << " m\n"
            << "  平均姿态误差  : " << avg_ori << " rad"
            << "  (" << std::fixed << std::setprecision(2) << avg_ori * 180.0 / M_PI << " deg)\n";
    }
};

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

static std::string json_array(const std::vector<double>& v) {
    std::ostringstream os; os << "[";
    for (size_t i = 0; i < v.size(); ++i) { if (i) os << ", "; os << std::setprecision(8) << v[i]; }
    os << "]"; return os.str();
}

static std::string json_pose(const Eigen::Isometry3d& tf) {
    Eigen::Quaterniond q(tf.linear());
    std::ostringstream os;
    os << std::setprecision(8) << "{\"pos\":[" << tf.translation().x()
       << "," << tf.translation().y() << "," << tf.translation().z() << "]"
       << ",\"quat\":[" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "]}";
    return os.str();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto args = parse(argc, argv);

    std::cout
        << "╔══════════════════════════════════════════════════╗\n"
        << "║      ALFA Robot IK Solver Benchmark              ║\n"
        << "╚══════════════════════════════════════════════════╝\n\n"
        << "规划组       : " << args.group    << "\n"
        << "求解器       : " << args.solver   << "\n"
        << "样本数       : " << args.samples  << "\n"
        << "超时         : " << args.timeout  << " s\n"
        << "位置阈值     : " << args.pos_thresh << " m\n"
        << "姿态阈值     : " << args.ori_thresh << " rad\n"
        << "末端位置扰动 : ±" << args.perturb_pos << " m\n"
        << "末端姿态扰动 : ±" << args.perturb_ori << " rad\n\n";

    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("alfa_robot_description");
        std::string urdf_path = pkg_dir + "/urdf/alfa_robot.urdf";
        std::string moveit_dir = ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");
        std::string srdf_path = moveit_dir + "/config/alfa_robot.srdf";

        ik_benchmark::IkSolver solver(urdf_path, srdf_path, args.group, args.solver, args.timeout);

        std::mt19937 rng(42);
        Summary summary;
        summary.name = args.solver;

        std::ofstream jsonl_file;
        if (!args.jsonl_path.empty()) {
            jsonl_file.open(args.jsonl_path, std::ios::trunc);
            jsonl_file << "{\"header\":true,\"group\":\"" << args.group
                       << "\",\"solver\":\"" << args.solver
                       << "\",\"samples\":" << args.samples << "}\n";
        }

        for (int i = 0; i < args.samples; ++i) {
            auto random_joints = solver.getRandomSeed();
            auto fk_poses = solver.fk(random_joints);

            Eigen::Isometry3d target_left = perturb(fk_poses[0], args.perturb_pos, args.perturb_ori, rng);
            Eigen::Isometry3d target_right = Eigen::Isometry3d::Identity();
            if (solver.isDualArm() && fk_poses.size() > 1) {
                target_right = perturb(fk_poses[1], args.perturb_pos, args.perturb_ori, rng);
            }

            ik_benchmark::IkResult ik_result;
            if (solver.isDualArm()) {
                ik_result = solver.solveDual(target_left, target_right);
            } else {
                ik_result = solver.solve(target_left);
            }

            SampleResult r;
            r.index = i;
            r.solve_ms = ik_result.solve_ms;
            r.pos_error = ik_result.pos_error;
            r.ori_error = ik_result.ori_error;

            if (!ik_result.success) r.status = Status::kFailed;
            else r.status = (r.pos_error <= args.pos_thresh && r.ori_error <= args.ori_thresh)
                            ? Status::kSuccess : Status::kLargeError;

            summary.add(r);

            if (jsonl_file.is_open()) {
                const char* st = (r.status == Status::kSuccess) ? "success" :
                                 (r.status == Status::kLargeError) ? "large_error" : "failed";
                jsonl_file << "{"
                    << "\"index\":" << i
                    << ",\"status\":\"" << st << "\""
                    << ",\"solve_ms\":" << std::setprecision(4) << r.solve_ms
                    << ",\"pos_error\":" << std::setprecision(8) << r.pos_error
                    << ",\"target_joints\":" << json_array(random_joints)
                    << ",\"solved_joints\":" << (ik_result.success ? json_array(ik_result.joint_values) : "[]")
                    << ",\"fk_left\":" << json_pose(fk_poses[0])
                    << ",\"target_left\":" << json_pose(target_left)
                    << "}\n";
            }

            if (args.verbose) {
                const char* tag = (r.status == Status::kSuccess) ? "OK  " :
                                  (r.status == Status::kLargeError) ? "ERR " : "FAIL";
                std::cout << "  [" << std::setw(4) << i << "] " << tag
                          << "  " << std::fixed << std::setprecision(1) << r.solve_ms << " ms";
                if (ik_result.success) std::cout << "  pos=" << std::scientific << std::setprecision(2) << r.pos_error;
                std::cout << "\n";
            }
        }

        summary.print();

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "\n基准测试完成。\n";
    rclcpp::shutdown();
    return 0;
}
