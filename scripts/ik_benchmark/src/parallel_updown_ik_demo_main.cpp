#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <Eigen/Geometry>
#include <iostream>
#include <string>

namespace {

Eigen::Isometry3d makePose(double x, double y, double z)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, z);
    Eigen::Quaterniond q(0.7071, 0.0, 0.7071, 0.0);
    q.normalize();
    pose.linear() = q.toRotationMatrix();
    return pose;
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    ik_benchmark::UpdownAwareIkConfig config;
    config.workers = 4;
    config.timeout = 0.5;
    config.h_candidate_count = 5;
    config.seed_count = 4;
    config.h_step = 0.1;
    config.h_search_margin = 0.1;
    config.check_collision = false;
    config.fallback_enabled = true;
    config.fallback_timeout = 2.0;
    config.fallback_seed_count = 8;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--workers" && i + 1 < argc) config.workers = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--timeout" && i + 1 < argc) config.timeout = std::stod(argv[++i]);
        else if (arg == "--h-mode" && i + 1 < argc) {
            const std::string mode = argv[++i];
            config.h_search_mode = mode == "continuous_range"
                ? ik_benchmark::UpdownAwareIkConfig::HSearchMode::ContinuousRange
                : ik_benchmark::UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
        }
        else if (arg == "--check-collision") config.check_collision = true;
        else if (arg == "--no-fallback") config.fallback_enabled = false;
    }

    ik_benchmark::ParallelUpdownAwareIkSolver solver(config);
    ik_benchmark::UpdownAwareIkRequest request;
    request.left_target = makePose(0.6, 0.5, 1.5);
    request.right_target = makePose(0.6, -0.5, 1.5);
    request.current_h = 0.0;
    request.current_arm_joints.assign(solver.fixedVariableNames().size(), 0.0);
    request.current_full_joints.assign(solver.freeVariableNames().size(), 0.0);

    const auto result = solver.solve(request);
    std::cout << "=== ParallelUpdownAwareIkSolver demo ===\n";
    std::cout << "success: " << (result.success ? "yes" : "no") << "\n";
    std::cout << "solver_path: " << result.solver_path << "\n";
    std::cout << "fallback_used: " << (result.fallback_used ? "yes" : "no") << "\n";
    std::cout << "range_reachable: " << (result.range_reachable ? "yes" : "no") << "\n";
    std::cout << "h_interval: [" << result.h_interval_lower << ", " << result.h_interval_upper << "]\n";
    std::cout << "h_center: " << result.h_center << "\n";
    std::cout << "h_candidates:";
    for (double h : result.h_candidates) std::cout << " " << h;
    std::cout << "\n";
    std::cout << "trial_count: " << result.trial_count << "\n";
    std::cout << "legal_count: " << result.legal_count << "\n";
    std::cout << "timeout_like_count: " << result.timeout_like_count << "\n";
    std::cout << "wall_ms: " << result.wall_ms << "\n";
    if (result.success) {
        std::cout << "selected_h: " << result.selected.h << "\n";
        std::cout << "selected_score: " << result.selected.score << "\n";
        std::cout << "selected_target_order: " << result.selected.target_order << "\n";
        std::cout << "direct_pos_error: " << result.selected.direct_pos_error << "\n";
        std::cout << "direct_ori_error: " << result.selected.direct_ori_error << "\n";
        std::cout << "collision_free: " << (result.selected.collision_free ? "yes" : "no") << "\n";
    } else {
        std::cout << "failure_reason: " << result.failure_reason << "\n";
    }

    rclcpp::shutdown();
    return result.success ? 0 : 2;
}
