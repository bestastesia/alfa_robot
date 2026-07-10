#pragma once

#include "ik_benchmark/ik_solver.h"
#include "robot_motion_core/ik_candidate_types.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ik_benchmark {

using ReachSphereConfig = robot_motion::core::ReachSphereConfig;
using UpdownAwareIkConfig = robot_motion::core::UpdownAwareIkConfig;
using UpdownAwareIkRequest = robot_motion::core::UpdownAwareIkRequest;
using UpdownAwareIkCandidate = robot_motion::core::UpdownAwareIkCandidate;
using UpdownAwareIkResult = robot_motion::core::UpdownAwareIkResult;
using UpdownAwareCostFn = robot_motion::core::UpdownAwareCostFn;

class ParallelUpdownAwareIkSolver {
public:
    explicit ParallelUpdownAwareIkSolver(UpdownAwareIkConfig config,
                                         UpdownAwareCostFn custom_cost = {});

    UpdownAwareIkResult solve(const UpdownAwareIkRequest& request);

    const std::vector<std::string>& fixedVariableNames() const;
    std::vector<std::string> fixedFullVariableNames() const;
    const std::vector<std::string>& freeVariableNames() const;
    double jointLeverProxy(const UpdownAwareIkCandidate& candidate, const std::string& prefix, int joint_index) const;

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
    HeightInterval intervalForTarget(const Eigen::Isometry3d& target, UpdownAwareIkRequest::GraspMode grasp_mode) const;
    std::vector<double> makeFixedHCandidates(const HeightInterval& interval, double h_center) const;
    std::vector<TrialSpec> makeNormalTrials(const UpdownAwareIkRequest& request, const HeightPlan& plan) const;
    std::vector<TrialSpec> makeFallbackTrials(const UpdownAwareIkRequest& request, const HeightPlan& plan, size_t round_index) const;
    void ensureFreeSolvers() const;
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
    Eigen::Isometry3d updownTransformInBase(double h) const;
    Eigen::Isometry3d fixedTarget(const Eigen::Isometry3d& target, double h) const;
    std::vector<double> makePerturbedSeed(const std::vector<std::string>& variable_names,
                                          const std::vector<double>& base_seed,
                                          size_t attempt_index,
                                          double revolute_noise,
                                          double prismatic_noise) const;
    std::vector<std::vector<double>> makeFallbackSeedFamilies(const UpdownAwareIkRequest& request, size_t round_index) const;
    std::vector<double> makeFullSeedFromArmSeed(double h, const std::vector<double>& arm_seed) const;
    double extractUpdown(const std::vector<std::string>& names, const std::vector<double>& values, double fallback) const;
    std::vector<std::string> fullJointNamesForFixedGroup() const;
    std::vector<double> fullJointValuesForFixedGroup(double h, const std::vector<double>& arm_values) const;
    double positionError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const;
    double orientationError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const;
    double toolAxisError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual) const;
    double jointDelta(const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const;
    double jointValue(const UpdownAwareIkCandidate& candidate, const std::string& name, double fallback = 0.0) const;
    double armTorqueProxy(const UpdownAwareIkCandidate& candidate, const std::string& prefix) const;
    double loadedPoseDistance(const UpdownAwareIkCandidate& candidate,
                              const std::string& prefix,
                              const std::vector<double>& pose) const;
    double loadedPoseFamilyMinDistance(const UpdownAwareIkCandidate& candidate,
                                       const std::string& prefix,
                                       const std::vector<std::vector<double>>& family) const;
    double loadedPosePreferredDistance(const UpdownAwareIkCandidate& candidate,
                                       const std::string& prefix,
                                       const std::vector<std::vector<double>>& family,
                                       size_t preferred_index) const;
    double scoreCandidate(const UpdownAwareIkCandidate& candidate, const UpdownAwareIkRequest& request) const;
    void sortAndSelect(UpdownAwareIkResult& result, const UpdownAwareIkRequest& request) const;
    bool shouldUseFallback(const HeightPlan& plan, const std::vector<UpdownAwareIkCandidate>& candidates) const;

    UpdownAwareIkConfig config_;
    UpdownAwareCostFn custom_cost_;
    std::vector<std::unique_ptr<IkSolver>> fixed_solvers_;
    mutable std::vector<std::unique_ptr<IkSolver>> free_solvers_;
    mutable std::mutex free_solvers_mutex_;
};

} // namespace ik_benchmark
