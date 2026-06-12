#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace ik_benchmark {

enum class V5ArmSide {
    Left,
    Right,
};

struct AnalyticV5IkSolution {
    bool valid = false;
    std::string branch;
    std::vector<double> joint_values;
    double position_error = 0.0;
    double orientation_error = 0.0;
    double score = 0.0;
};

struct AnalyticV5IkConfig {
    V5ArmSide side = V5ArmSide::Left;
    double position_tolerance = 0.002;
    double orientation_tolerance = 0.01;
    double q1_min = -2.35619449;
    double q1_max = 2.35619449;
    double q_min = -3.14159265;
    double q_max = 3.14159265;
    double q1_scan_step = 0.01;
    size_t max_solutions = 16;
    bool debug = false;
};

class AnalyticV5IkSolver {
public:
    explicit AnalyticV5IkSolver(AnalyticV5IkConfig config = {});

    const AnalyticV5IkConfig& config() const { return config_; }
    Eigen::Isometry3d fk(const std::vector<double>& joints) const;
    std::vector<AnalyticV5IkSolution> solve(const Eigen::Isometry3d& target) const;

private:
    struct JointTransform {
        Eigen::Vector3d xyz;
        Eigen::Matrix3d rpy;
        Eigen::Vector3d axis;
        bool revolute = true;
    };

    AnalyticV5IkConfig config_;
    std::vector<JointTransform> chain_;

    static Eigen::Matrix3d rpy(double roll, double pitch, double yaw);
    static Eigen::Matrix3d rotAxis(const Eigen::Vector3d& axis, double angle);
    static double wrapToPi(double value);
    static double orientationError(const Eigen::Matrix3d& desired, const Eigen::Matrix3d& actual);
    bool inLimits(const std::vector<double>& joints) const;
    void initChain();
};

} // namespace ik_benchmark
