#include "ik_benchmark/parallel_updown_aware_ik_solver.h"
#include "alfa_robot_benchmarks/srv/solve_dual_ik.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

namespace {

using SolveDualIk = alfa_robot_benchmarks::srv::SolveDualIk;
using ik_benchmark::ParallelUpdownAwareIkSolver;
using ik_benchmark::UpdownAwareIkConfig;
using ik_benchmark::UpdownAwareIkRequest;

constexpr double kWorldToBaseZ = 0.202094;

std::vector<double> degSeed(const std::vector<double>& degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (double degree : degrees) radians.push_back(degree * M_PI / 180.0);
    return radians;
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

Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (q.norm() < 1e-12) q = Eigen::Quaterniond::Identity();
    q.normalize();
    out.linear() = q.toRotationMatrix();
    return out;
}

std::vector<double> valuesForNames(
    const sensor_msgs::msg::JointState& joint_state,
    const std::vector<std::string>& names,
    const std::vector<double>& fallback)
{
    std::unordered_map<std::string, double> by_name;
    for (size_t i = 0; i < joint_state.name.size() && i < joint_state.position.size(); ++i) {
        by_name[joint_state.name[i]] = joint_state.position[i];
    }
    std::vector<double> values;
    values.reserve(names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        const auto found = by_name.find(names[i]);
        if (found != by_name.end()) {
            values.push_back(found->second);
        } else if (i < fallback.size()) {
            values.push_back(fallback[i]);
        } else {
            values.push_back(0.0);
        }
    }
    return values;
}

UpdownAwareIkConfig makeConfig(rclcpp::Node& node)
{
    UpdownAwareIkConfig config;
    const double fixed_updown = node.has_parameter("fixed_updown")
        ? node.get_parameter("fixed_updown").as_double()
        : node.declare_parameter<double>("fixed_updown", 0.18);
    const bool lock_updown = node.declare_parameter<bool>("lock_updown", true);
    config.fixed_group = node.declare_parameter<std::string>("fixed_group", "dual_v5_arm");
    config.free_group = node.declare_parameter<std::string>("free_group", "dual_v5_arm_with_base");
    config.solver_plugin = node.declare_parameter<std::string>("solver_plugin", "bio_ik/BioIKKinematicsPlugin");
    config.base_frame = node.declare_parameter<std::string>("base_frame", "base_link");
    config.left_tip = node.declare_parameter<std::string>("left_tip", "left_v5_tool0");
    config.right_tip = node.declare_parameter<std::string>("right_tip", "right_v5_tool0");
    config.tool0_offset = node.declare_parameter<double>("tool0_offset", 0.0);

    config.gripper_z_reach_lower = node.declare_parameter<double>("front_z_window_lower", 0.45 - kWorldToBaseZ);
    config.gripper_z_reach_upper = node.declare_parameter<double>("front_z_window_upper", 1.25 - kWorldToBaseZ);
    config.top_suction_z_reach_lower = node.declare_parameter<double>("top_z_window_lower", 0.3 - kWorldToBaseZ);
    config.top_suction_z_reach_upper = node.declare_parameter<double>("top_z_window_upper", 0.45 - kWorldToBaseZ);
    config.h_lower = node.declare_parameter<double>("h_lower", 0.0);
    config.h_upper = node.declare_parameter<double>("h_upper", 0.99);
    if (lock_updown) {
        config.h_lower = fixed_updown;
        config.h_upper = fixed_updown;
    }

    const std::string h_mode = node.declare_parameter<std::string>("h_mode", "fixed_discrete");
    config.h_search_mode = h_mode == "continuous_range"
        ? UpdownAwareIkConfig::HSearchMode::ContinuousRange
        : UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
    config.h_search_margin = node.declare_parameter<double>("h_search_margin", 0.2);
    config.h_step = node.declare_parameter<double>("h_step", 0.1);
    config.h_candidate_count = static_cast<size_t>(std::max<int64_t>(1, node.declare_parameter<int64_t>("h_candidate_count", 16)));
    config.seed_count = static_cast<size_t>(std::max<int64_t>(1, node.declare_parameter<int64_t>("seed_count", 32)));
    config.workers = static_cast<size_t>(std::max<int64_t>(1, node.declare_parameter<int64_t>("workers", 16)));
    config.timeout = node.declare_parameter<double>("timeout", 0.01);
    config.seed_noise = node.declare_parameter<double>("seed_noise", 0.35);
    config.try_target_orders = node.declare_parameter<bool>("try_target_orders", false);
    config.use_reversed_target_order = node.declare_parameter<bool>("use_reversed_target_order", true);

    config.check_tip_error = node.declare_parameter<bool>("check_tip_error", true);
    config.position_tolerance = node.declare_parameter<double>("position_tolerance", 0.02);
    config.top_suction_position_tolerance = node.declare_parameter<double>("top_suction_position_tolerance", 0.04);
    config.orientation_tolerance = node.declare_parameter<double>("orientation_tolerance", 0.05);
    config.top_suction_orientation_tolerance = node.declare_parameter<double>("top_suction_orientation_tolerance", 5.0 * M_PI / 180.0);
    config.check_collision = node.declare_parameter<bool>("check_collision", true);
    config.enforce_arm_base_collisions = node.declare_parameter<bool>("enforce_arm_base_collisions", true);
    config.reject_swapped_tips = node.declare_parameter<bool>("reject_swapped_tips", true);

    config.fallback_enabled = node.declare_parameter<bool>("fallback_enabled", false);
    config.fallback_timeout = node.declare_parameter<double>("fallback_timeout", 0.02);
    config.fallback_seed_count = static_cast<size_t>(std::max<int64_t>(1, node.declare_parameter<int64_t>("fallback_seed_count", 64)));
    config.fallback_rounds = static_cast<size_t>(std::max<int64_t>(1, node.declare_parameter<int64_t>("fallback_rounds", 1)));

    config.cost_updown_static_bonus = node.declare_parameter<double>("cost_updown_static_bonus", 1.0);
    config.cost_updown_within_0p1_bonus = node.declare_parameter<double>("cost_updown_within_0p1_bonus", 0.3);
    config.cost_updown_over_0p1_distance = node.declare_parameter<double>("cost_updown_over_0p1_distance", 2.0);
    config.cost_joint2_torque = node.declare_parameter<double>("cost_joint2_torque", 1.0);
    config.cost_joint3_torque = node.declare_parameter<double>("cost_joint3_torque", 0.5);

    config.solver_options.reject_collisions = false;
    config.solver_options.enforce_arm_base_collisions = config.enforce_arm_base_collisions;
    return config;
}

class FixedPlatformDualIkService : public rclcpp::Node {
public:
    FixedPlatformDualIkService()
        : Node("fixed_platform_dual_ik_service"),
          fixed_updown_(declare_parameter<double>("fixed_updown", 0.18)),
          home_arm_seed_(degSeed({0, 5, 145, 0, 120, 0})),
          config_(makeConfig(*this)),
          solver_(std::make_unique<ParallelUpdownAwareIkSolver>(config_))
    {
        home_full_seed_ = fullValues(fixed_updown_, home_arm_seed_, home_arm_seed_);
        service_ = create_service<SolveDualIk>(
            declare_parameter<std::string>("service_name", "/alfa_dual_ik/solve"),
            std::bind(&FixedPlatformDualIkService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Fixed platform dual IK service ready: fixed_updown=%.3f workers=%zu trials=%zu timeout=%.3fs",
                    fixed_updown_, config_.workers, config_.h_candidate_count * config_.seed_count, config_.timeout);
    }

private:
    void handleRequest(const std::shared_ptr<SolveDualIk::Request> request,
                       std::shared_ptr<SolveDualIk::Response> response)
    {
        const double current_h = request->current_updown >= 0.0 ? request->current_updown : fixed_updown_;

        UpdownAwareIkRequest ik_request;
        ik_request.left_target = poseMsgToEigen(request->left_target);
        ik_request.right_target = poseMsgToEigen(request->right_target);
        ik_request.current_h = current_h;
        ik_request.grasp_mode = request->grasp_mode == "top_suction"
            ? UpdownAwareIkRequest::GraspMode::TopSuction
            : UpdownAwareIkRequest::GraspMode::Front;
        ik_request.current_arm_joints = valuesForNames(request->current_joint_state, solver_->fixedVariableNames(), fullValues(0.0, home_arm_seed_, home_arm_seed_));
        ik_request.current_full_joints = valuesForNames(request->current_joint_state, solver_->freeVariableNames(), home_full_seed_);
        for (size_t i = 0; i < solver_->freeVariableNames().size() && i < ik_request.current_full_joints.size(); ++i) {
            if (solver_->freeVariableNames()[i] == "updown") {
                ik_request.current_full_joints[i] = current_h;
            }
        }

        const auto result = solver_->solve(ik_request);

        std::map<std::string, size_t> rejection_counts;
        std::map<std::string, size_t> solver_path_counts;
        std::map<std::string, size_t> target_order_counts;
        std::set<size_t> unique_seed_indices;
        std::set<size_t> unique_h_indices;
        double best_direct_pos_error = std::numeric_limits<double>::infinity();
        double best_direct_ori_error = std::numeric_limits<double>::infinity();
        size_t best_error_seed_index = 0;
        std::string best_error_reason;
        const ik_benchmark::UpdownAwareIkCandidate* best_error_candidate = nullptr;
        for (const auto& candidate : result.candidates) {
            std::string reason = candidate.rejection_reason;
            if (reason.empty()) {
                reason = candidate.legal ? "legal" : "unknown_rejected";
            }
            ++rejection_counts[reason];
            ++solver_path_counts[candidate.solver_path.empty() ? "unknown" : candidate.solver_path];
            ++target_order_counts[candidate.target_order.empty() ? "unknown" : candidate.target_order];
            unique_seed_indices.insert(candidate.seed_index);
            unique_h_indices.insert(candidate.h_index);
            if (candidate.direct_pos_error < best_direct_pos_error) {
                best_direct_pos_error = candidate.direct_pos_error;
                best_direct_ori_error = candidate.direct_ori_error;
                best_error_seed_index = candidate.seed_index;
                best_error_reason = reason;
                best_error_candidate = &candidate;
            }
        }
        auto countsToJson = [](const std::map<std::string, size_t>& counts) {
            nlohmann::json out = nlohmann::json::object();
            for (const auto& item : counts) {
                out[item.first] = item.second;
            }
            return out;
        };
        auto candidateToJson = [](const ik_benchmark::UpdownAwareIkCandidate& candidate, size_t index) {
            return nlohmann::json{
                {"candidate_index", index},
                {"h", candidate.h},
                {"h_index", candidate.h_index},
                {"seed_index", candidate.seed_index},
                {"target_order", candidate.target_order},
                {"solver_path", candidate.solver_path},
                {"rejection_reason", candidate.rejection_reason},
                {"direct_pos_error", candidate.direct_pos_error},
                {"direct_ori_error", candidate.direct_ori_error},
                {"swapped_pos_error", candidate.swapped_pos_error},
                {"collision_free", candidate.collision_free},
                {"joint_names", candidate.joint_names},
                {"joint_values", candidate.joint_values},
                {"full_joint_names", candidate.full_joint_names},
                {"full_joint_values", candidate.full_joint_values},
                {"collision_pairs", candidate.collision_pairs}
            };
        };
        std::vector<std::pair<double, size_t>> candidate_order;
        candidate_order.reserve(result.candidates.size());
        for (size_t i = 0; i < result.candidates.size(); ++i) {
            if (!result.candidates[i].full_joint_values.empty()) {
                candidate_order.push_back({result.candidates[i].direct_pos_error, i});
            }
        }
        std::sort(candidate_order.begin(), candidate_order.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.first < rhs.first;
        });
        nlohmann::json debug_best_candidates = nlohmann::json::array();
        const size_t debug_limit = static_cast<size_t>(std::max<int64_t>(0, get_parameter_or("debug_candidate_limit", int64_t{10})));
        for (size_t rank = 0; rank < candidate_order.size() && rank < debug_limit; ++rank) {
            debug_best_candidates.push_back(candidateToJson(result.candidates[candidate_order[rank].second], candidate_order[rank].second));
        }

        response->success = result.success;
        response->failure_reason = result.failure_reason;
        response->solver_path = result.solver_path;
        response->fallback_used = result.fallback_used;
        response->selected_h = result.success ? result.selected.h : current_h;
        response->score = result.success ? result.selected.score : std::numeric_limits<double>::infinity();
        response->wall_ms = result.wall_ms;
        response->trial_count = static_cast<uint32_t>(result.trial_count);
        response->legal_count = static_cast<uint32_t>(result.legal_count);

        response->joint_target.header = request->header;
        if (result.success) {
            response->joint_target.name = result.selected.joint_names;
            response->joint_target.position = result.selected.joint_values;
        }

        nlohmann::json diagnostics = {
            {"range_reachable", result.range_reachable},
            {"h_interval", {result.h_interval_lower, result.h_interval_upper}},
            {"h_center", result.h_center},
            {"h_candidates", result.h_candidates},
            {"timeout_like_count", result.timeout_like_count},
            {"swapped_rejected_count", result.swapped_rejected_count},
            {"candidate_count", result.candidates.size()},
            {"unique_seed_index_count", unique_seed_indices.size()},
            {"unique_h_index_count", unique_h_indices.size()},
            {"rejection_counts", countsToJson(rejection_counts)},
            {"solver_path_counts", countsToJson(solver_path_counts)},
            {"target_order_counts", countsToJson(target_order_counts)},
            {"best_direct_pos_error", best_direct_pos_error},
            {"best_direct_ori_error", best_direct_ori_error},
            {"best_error_seed_index", best_error_seed_index},
            {"best_error_reason", best_error_reason},
            {"debug_best_candidates", debug_best_candidates},
            {"best_candidate", best_error_candidate ? nlohmann::json{
                {"h", best_error_candidate->h},
                {"h_index", best_error_candidate->h_index},
                {"seed_index", best_error_candidate->seed_index},
                {"target_order", best_error_candidate->target_order},
                {"solver_path", best_error_candidate->solver_path},
                {"rejection_reason", best_error_candidate->rejection_reason},
                {"direct_pos_error", best_error_candidate->direct_pos_error},
                {"direct_ori_error", best_error_candidate->direct_ori_error},
                {"swapped_pos_error", best_error_candidate->swapped_pos_error},
                {"collision_free", best_error_candidate->collision_free},
                {"joint_names", best_error_candidate->joint_names},
                {"joint_values", best_error_candidate->joint_values},
                {"full_joint_names", best_error_candidate->full_joint_names},
                {"full_joint_values", best_error_candidate->full_joint_values},
                {"collision_pairs", best_error_candidate->collision_pairs}
            } : nlohmann::json::object()},
            {"selected", result.success ? nlohmann::json{
                {"h", result.selected.h},
                {"h_index", result.selected.h_index},
                {"seed_index", result.selected.seed_index},
                {"score", result.selected.score},
                {"direct_pos_error", result.selected.direct_pos_error},
                {"direct_ori_error", result.selected.direct_ori_error},
                {"collision_free", result.selected.collision_free},
                {"target_order", result.selected.target_order},
                {"solver_path", result.selected.solver_path},
                {"joint_names", result.selected.joint_names},
                {"joint_values", result.selected.joint_values},
                {"full_joint_names", result.selected.full_joint_names},
                {"full_joint_values", result.selected.full_joint_values}
            } : nlohmann::json::object()}
        };
        response->diagnostics_json = diagnostics.dump();

        const std::string rejection_summary = countsToJson(rejection_counts).dump();
        RCLCPP_INFO(get_logger(), "IK %s mode=%s h=%.3f trials=%u legal=%u wall=%.1fms reason=%s unique_seeds=%zu unique_h=%zu best_pos_err=%.4f best_ori_err=%.4f best_seed=%zu best_reason=%s reject=%s",
                    response->success ? "success" : "failed",
                    request->grasp_mode.empty() ? "front" : request->grasp_mode.c_str(),
                    response->selected_h,
                    response->trial_count,
                    response->legal_count,
                    response->wall_ms,
                    response->failure_reason.c_str(),
                    unique_seed_indices.size(),
                    unique_h_indices.size(),
                    best_direct_pos_error,
                    best_direct_ori_error,
                    best_error_seed_index,
                    best_error_reason.c_str(),
                    rejection_summary.c_str());
    }

    double fixed_updown_;
    std::vector<double> home_arm_seed_;
    std::vector<double> home_full_seed_;
    UpdownAwareIkConfig config_;
    std::unique_ptr<ParallelUpdownAwareIkSolver> solver_;
    rclcpp::Service<SolveDualIk>::SharedPtr service_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedPlatformDualIkService>());
    rclcpp::shutdown();
    return 0;
}
