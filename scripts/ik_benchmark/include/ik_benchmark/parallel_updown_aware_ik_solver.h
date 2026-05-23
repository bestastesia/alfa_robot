#pragma once

#include "ik_benchmark/ik_solver.h"

#include <Eigen/Geometry>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ik_benchmark {

struct ReachSphereConfig {
    double cx = -0.065;
    double cy = 0.2;
    double cz = 1.025;
    double radius = 0.815;
};

struct UpdownAwareIkConfig {
    enum class HSearchMode {
        FixedDiscrete,
        ContinuousRange,
    };

    std::string fixed_group = "dual_v5_arm";
    std::string free_group = "dual_v5_arm_with_base";
    std::string solver_plugin = "bio_ik/BioIKKinematicsPlugin";
    std::string base_frame = "base_link";
    std::string left_tip = "left_v5_tool0";
    std::string right_tip = "right_v5_tool0";

    ReachSphereConfig left_reach_sphere;
    ReachSphereConfig right_reach_sphere = {-0.065, -0.2, 1.025, 0.815};
    double tool0_offset = 0.1;
    double sphere_margin = 0.0;
    double h_lower = 0.0;
    double h_upper = 0.99;

    HSearchMode h_search_mode = HSearchMode::FixedDiscrete;
    double h_search_margin = 0.1;
    double h_step = 0.1;
    size_t h_candidate_count = 5;
    double max_updown_delta = std::numeric_limits<double>::infinity();

    size_t seed_count = 4;
    double seed_noise = 0.35;
    bool try_target_orders = true;

    size_t workers = 4;
    double timeout = 0.5;

    bool check_tip_error = true;
    double position_tolerance = 0.02;
    double orientation_tolerance = 0.05;
    bool check_collision = false;
    bool reject_swapped_tips = true;

    bool fallback_enabled = true;
    double fallback_timeout = 2.0;
    size_t fallback_seed_count = 12;

    double cost_updown_delta = 1.0;
    double cost_joint_delta = 0.01;
    double cost_solve_ms = 0.001;
    double cost_h_center_delta = 0.0;

    IkSolverOptions solver_options;
};

struct UpdownAwareIkRequest {
    Eigen::Isometry3d left_target = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_target = Eigen::Isometry3d::Identity();
    double current_h = 0.0;
    std::vector<double> current_arm_joints;
    std::vector<double> current_full_joints;
};

struct UpdownAwareIkCandidate {
    bool legal = false;
    bool collision_free = true;
    bool swapped = false;
    bool timeout_like = false;
    std::string solver_path;
    std::string target_order;
    std::string rejection_reason;

    double h = 0.0;
    double h_center = 0.0;
    double score = std::numeric_limits<double>::infinity();
    double solve_ms = 0.0;
    double direct_pos_error = 0.0;
    double direct_ori_error = 0.0;
    double swapped_pos_error = 0.0;
    double updown_delta = 0.0;
    double joint_delta = 0.0;

    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    std::vector<std::string> full_joint_names;
    std::vector<double> full_joint_values;
    std::vector<std::string> collision_pairs;
};

struct UpdownAwareIkResult {
    bool success = false;
    bool fallback_used = false;
    bool range_reachable = false;
    std::string solver_path;
    std::string failure_reason;

    double h_interval_lower = 0.0;
    double h_interval_upper = 0.0;
    double h_center = 0.0;
    std::vector<double> h_candidates;

    size_t trial_count = 0;
    size_t legal_count = 0;
    size_t timeout_like_count = 0;
    size_t swapped_rejected_count = 0;
    double wall_ms = 0.0;
    double sum_solve_ms = 0.0;

    UpdownAwareIkCandidate selected;
    std::vector<UpdownAwareIkCandidate> candidates;
};

using UpdownAwareCostFn = std::function<double(const UpdownAwareIkCandidate&, const UpdownAwareIkRequest&)>;

class ParallelUpdownAwareIkSolver {
public:
    explicit ParallelUpdownAwareIkSolver(UpdownAwareIkConfig config,
                                         UpdownAwareCostFn custom_cost = {});

    UpdownAwareIkResult solve(const UpdownAwareIkRequest& request);

    const std::vector<std::string>& fixedVariableNames() const;
    const std::vector<std::string>& freeVariableNames() const;

private:
    struct HeightInterval {
        bool reachable = false;
        double lower = 0.0;
        double upper = 0.0;
    };

    struct HeightPlan {
        bool reachable = false;
        HeightInterval left;
        HeightInterval right;
        HeightInterval combined;
        double h_center = 0.0;
        std::vector<double> candidates;
    };

    struct TrialSpec {
        bool free_updown = false;
        double h = 0.0;
        double h_range_lower = 0.0;
        double h_range_upper = 0.0;
        size_t h_index = 0;
        size_t seed_index = 0;
        std::string solver_path;
        std::vector<double> seed;
    };

    HeightPlan planHeight(const UpdownAwareIkRequest& request) const;
    HeightInterval intervalForTarget(const Eigen::Isometry3d& target, const ReachSphereConfig& sphere) const;
    std::vector<double> makeFixedHCandidates(const HeightInterval& interval, double h_center) const;
    std::vector<TrialSpec> makeNormalTrials(const UpdownAwareIkRequest& request, const HeightPlan& plan) const;
    std::vector<TrialSpec> makeFallbackTrials(const UpdownAwareIkRequest& request, const HeightPlan& plan) const;
    std::vector<UpdownAwareIkCandidate> executeTrials(const std::vector<TrialSpec>& trials,
                                                      const UpdownAwareIkRequest& request,
                                                      const HeightPlan& plan,
                                                      bool fallback);
    UpdownAwareIkCandidate solveTrial(IkSolver& solver,
                                      const TrialSpec& trial,
                                      const UpdownAwareIkRequest& request,
                                      const HeightPlan& plan,
                                      bool swapped_order,
                                      bool fallback) const;

    Eigen::Isometry3d compensateTool0(const Eigen::Isometry3d& target) const;
    Eigen::Isometry3d fixedTarget(const Eigen::Isometry3d& target, double h) const;
    std::vector<double> makePerturbedSeed(const std::vector<std::string>& variable_names,
                                          const std::vector<double>& base_seed,
                                          size_t attempt_index,
                                          double revolute_noise,
                                          double prismatic_noise) const;
    std::vector<double> makeFullSeedFromArmSeed(double h, const std::vector<double>& arm_seed) const;
    double extractUpdown(const std::vector<std::string>& names, const std::vector<double>& values, double fallback) const;
    std::vector<std::string> fullJointNamesForFixedGroup() const;
    std::vector<double> fullJointValuesForFixedGroup(double h, const std::vector<double>& arm_values) const;
    double positionError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const;
    double orientationError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const;
    double jointDelta(const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const;
    double scoreCandidate(const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const;
    void sortAndSelect(UpdownAwareIkResult& result, const UpdownAwareIkRequest& request) const;
    bool shouldUseFallback(const HeightPlan& plan, const std::vector<UpdownAwareIkCandidate>& candidates) const;

    UpdownAwareIkConfig config_;
    UpdownAwareCostFn custom_cost_;
    std::vector<std::unique_ptr<IkSolver>> fixed_solvers_;
    std::vector<std::unique_ptr<IkSolver>> free_solvers_;
};

} // namespace ik_benchmark
