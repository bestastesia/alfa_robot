#include "ik_benchmark/analytic_v5_ik_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace ik_benchmark {
namespace {
constexpr double kEps = 1e-9;

Eigen::Isometry3d makeTf(const Eigen::Vector3d& xyz, const Eigen::Matrix3d& rot)
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = xyz;
    tf.linear() = rot;
    return tf;
}

Eigen::Matrix3d rz(double angle)
{
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double orientationScore(const Eigen::Matrix3d& desired, const Eigen::Matrix3d& actual)
{
    if (!desired.allFinite() || !actual.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    return (desired - actual).norm();
}

} // namespace

AnalyticV5IkSolver::AnalyticV5IkSolver(AnalyticV5IkConfig config)
    : config_(std::move(config))
{
    initChain();
}

Eigen::Matrix3d AnalyticV5IkSolver::rpy(double roll, double pitch, double yaw)
{
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d AnalyticV5IkSolver::rotAxis(const Eigen::Vector3d& axis, double angle)
{
    return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

double AnalyticV5IkSolver::wrapToPi(double value)
{
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
}

double AnalyticV5IkSolver::orientationError(const Eigen::Matrix3d& desired, const Eigen::Matrix3d& actual)
{
    Eigen::AngleAxisd aa(desired.transpose() * actual);
    return std::abs(aa.angle());
}

void AnalyticV5IkSolver::initChain()
{
    chain_.clear();
    if (config_.side == V5ArmSide::Left) {
        chain_ = {
            {{0.0, 0.0, 0.0}, rpy(0, 0, 0), {0, 0, 1}, true},
            {{-0.0011221, 0.082492, 0.058}, rpy(-1.5708, 0, 0), {0, 0, 1}, true},
            {{-0.00054177, -0.4, 0.0079934}, rpy(0, 0, 0), {0, 0, 1}, true},
            {{0.0010609, -0.239, -0.077993}, rpy(1.5708, 0, 0), {0, 0, 1}, false},
            {{-0.0010337, 0.075993, 0.058}, rpy(-1.5708, 0, 0), {0, 0, 1}, true},
            {{-0.00078891, -0.083, 0.057995}, rpy(-1.57106636, -1.55719433, -3.14132260), {0, 0, -1}, true},
            {{-0.00078891, -0.083, 0.057995}, rpy(1.5708, -0.013602, 0), {0, 0, 1}, true},
            {{0.0, 0.0, 0.209}, rpy(0, 0, 0), {0, 0, 1}, false},
        };
    } else {
        chain_ = {
            {{0.0, 0.0, 0.0}, rpy(0, 0, 0), {0, 0, 1}, true},
            {{0.0011222, -0.082492, 0.058}, rpy(-1.5708, 0, 0), {0, 0, 1}, true},
            {{0.00054177, -0.4, -0.0079934}, rpy(0, 0, 0), {0, 0, 1}, true},
            {{-0.00226694, -0.23900051, 0.07798831}, rpy(1.5708, 0, 0), {0, 0, 1}, false},
            {{-0.00103370, -0.07599300, 0.05800000}, rpy(1.5708, 0, 0), {0, 0, 1}, true},
            {{-0.00078891, 0.08300000, 0.05799500}, rpy(1.57106636, -1.55719433, 3.14132260), {0, 0, -1}, true},
            {{-0.00078891, 0.08300000, 0.05799500}, rpy(-1.5708, -0.013602, 0), {0, 0, 1}, true},
            {{0.0, 0.0, 0.209}, rpy(0, 0, 0), {0, 0, 1}, false},
        };
    }
}

Eigen::Isometry3d AnalyticV5IkSolver::fk(const std::vector<double>& joints) const
{
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    size_t qi = 0;
    for (const auto& item : chain_) {
        tf = tf * makeTf(item.xyz, item.rpy);
        if (item.revolute) {
            const double q = qi < joints.size() ? joints[qi] : 0.0;
            tf.linear() = tf.linear() * rotAxis(item.axis, q);
            ++qi;
        }
    }
    return tf;
}

bool AnalyticV5IkSolver::inLimits(const std::vector<double>& joints) const
{
    if (joints.size() != 6) return false;
    if (joints[0] < config_.q1_min - 1e-9 || joints[0] > config_.q1_max + 1e-9) return false;
    for (size_t i = 1; i < joints.size(); ++i) {
        if (joints[i] < config_.q_min - 1e-9 || joints[i] > config_.q_max + 1e-9) return false;
    }
    return true;
}

std::vector<AnalyticV5IkSolution> AnalyticV5IkSolver::solve(const Eigen::Isometry3d& target) const
{
    // First production-facing version deliberately keeps KDL/BioIK untouched.
    // It uses the structural property verified from the current URDF:
    //   left wrist orientation variables  = q1, phi=(q2+q3+q4), q5, q6
    //   right wrist orientation variables = q1, phi=(q2+q3-q4), q5, q6
    // For each q1 grid candidate, solve phi/q5/q6 in closed form from the
    // target tool orientation, then close q2/q3 with the true URDF 2R link
    // vectors and derive q4 from phi.  Every output is FK-checked.
    std::vector<AnalyticV5IkSolution> out;

    const Eigen::Vector3d target_p = target.translation();
    const Eigen::Matrix3d target_R = target.linear();
    const double q4_sign = config_.side == V5ArmSide::Left ? 1.0 : -1.0;
    const Eigen::Vector3d joint2_origin = chain_[1].xyz;
    const Eigen::Vector3d v23 = chain_[2].xyz;
    const Eigen::Vector3d v34 = chain_[3].xyz + chain_[3].rpy * chain_[4].xyz;
    const double a_len = std::hypot(v23.x(), v23.y());
    const double b_len = std::hypot(v34.x(), v34.y());
    const double a_angle = std::atan2(v23.y(), v23.x());
    const double b_angle = std::atan2(v34.y(), v34.x());

    auto make_orientation = [&](double q1, double phi, double q5, double q6) {
        std::vector<double> joints = {q1, 0.0, 0.0, q4_sign * phi, q5, q6};
        const Eigen::Isometry3d pose = fk(joints);
        return Eigen::Matrix3d(pose.linear());
    };
    const Eigen::Matrix3d A = chain_[1].rpy;
    const Eigen::Matrix3d C = chain_[6].rpy;
    const Eigen::Matrix3d B = A.transpose() * make_orientation(0.0, 0.0, 0.0, 0.0) * C.transpose();
    const Eigen::Vector3d wrist_tool_z = C * Eigen::Vector3d::UnitZ();
    auto make_orientation_fast = [&](double q1, double phi, double q5, double q6) -> Eigen::Matrix3d {
        const Eigen::Matrix3d orientation = rz(q1) * A * rz(phi) * B * rz(-q5) * C * rz(q6);
        return orientation;
    };

    auto joint4AxisAtZeroTriangle = [&](double q1, double phi, double q5, double q6) -> Eigen::Vector3d {
        std::vector<double> wrist_pose_joints = {q1, 0.0, 0.0, q4_sign * phi, q5, q6};
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        size_t qi = 0;
        for (size_t idx = 0; idx < chain_.size(); ++idx) {
            const auto& item = chain_[idx];
            tf = tf * makeTf(item.xyz, item.rpy);
            if (idx == 4) return Eigen::Vector3d(tf.translation());
            if (item.revolute) {
                const double q = qi < wrist_pose_joints.size() ? wrist_pose_joints[qi] : 0.0;
                tf.linear() = tf.linear() * rotAxis(item.axis, q);
                ++qi;
            }
        }
        return Eigen::Vector3d(tf.translation());
    };

    struct OrientationCandidate {
        double q1 = 0.0;
        double phi = 0.0;
        double q5 = 0.0;
        double q6 = 0.0;
        double score = std::numeric_limits<double>::infinity();
    };

    auto add_orientation_candidate = [](std::vector<OrientationCandidate>& candidates,
                                        const OrientationCandidate& candidate) {
        for (const auto& existing : candidates) {
            if (std::abs(existing.q1 - candidate.q1) < 1e-3 &&
                std::abs(existing.phi - candidate.phi) < 1e-3 &&
                std::abs(existing.q5 - candidate.q5) < 1e-3 &&
                std::abs(existing.q6 - candidate.q6) < 1e-3) {
                return;
            }
        }
        candidates.push_back(candidate);
    };

    std::vector<OrientationCandidate> orientation_candidates;
    // The current v6 proxy wrist is not perfectly spherical, but its
    // orientation still factorises cleanly in the URDF:
    //     R = R(q1, phi, q5, q6), phi = q2 + q3 ± q4.
    // For a fixed q1, the tool z-axis is independent of q6.  We solve q5 and
    // phi from that z-axis by projecting into the exact URDF zero-q5 basis,
    // then recover q6 from the remaining rotation around the tool z-axis.
    auto add_orientation_candidates_for_q1 = [&](double q1) {
        if (q1 < config_.q1_min - kEps || q1 > config_.q1_max + kEps) return;

        const Eigen::Vector3d v = A.transpose() * rz(-q1) * target_R.col(2);
        const double alpha = B(2, 0) * wrist_tool_z.x() + B(2, 1) * wrist_tool_z.y();
        const double beta = B(2, 0) * wrist_tool_z.y() - B(2, 1) * wrist_tool_z.x();
        const double gamma = B(2, 2) * wrist_tool_z.z();
        const double radius = std::hypot(alpha, beta);
        const double rhs = v.z() - gamma;
        std::vector<double> q5_options;
        if (radius < 1e-9) {
            if (std::abs(rhs) > 1e-6) return;
            q5_options = {0.0, M_PI};
        } else {
            const double ratio = rhs / radius;
            if (ratio < -1.0 - 1e-7 || ratio > 1.0 + 1e-7) return;
            const double clipped = std::clamp(ratio, -1.0, 1.0);
            const double offset = std::atan2(beta, alpha);
            const double angle = std::acos(clipped);
            q5_options = {wrapToPi(offset + angle), wrapToPi(offset - angle)};
        }

        for (double q5 : q5_options) {
            if (q5 < config_.q_min - kEps || q5 > config_.q_max + kEps) continue;
            double phi = 0.0;
            const Eigen::Vector3d u = B * rz(-q5) * wrist_tool_z;
            if (std::hypot(u.x(), u.y()) > 1e-8 && std::hypot(v.x(), v.y()) > 1e-8) {
                phi = wrapToPi(std::atan2(v.y(), v.x()) - std::atan2(u.y(), u.x()));
            } else {
                for (double singular_phi : {0.0, M_PI_2, -M_PI_2, M_PI}) {
                    for (double q6_probe : {0.0}) {
                        OrientationCandidate cand;
                        cand.q1 = q1;
                        cand.phi = singular_phi;
                        cand.q5 = q5;
                        const Eigen::Matrix3d base_R = make_orientation_fast(q1, singular_phi, q5, 0.0);
                        const Eigen::Matrix3d delta_R = base_R.transpose() * target_R;
                        cand.q6 = wrapToPi(q6_probe + std::atan2(delta_R(1, 0), delta_R(0, 0)));
                        cand.score = orientationScore(target_R, make_orientation_fast(cand.q1, cand.phi, cand.q5, cand.q6));
                        if (cand.score <= std::sqrt(2.0) * config_.orientation_tolerance) {
                            add_orientation_candidate(orientation_candidates, cand);
                        }
                    }
                }
                continue;
            }

            for (double phi_branch : {wrapToPi(phi)}) {
                if (phi_branch < config_.q_min - kEps || phi_branch > config_.q_max + kEps) continue;
                const Eigen::Matrix3d base_R = make_orientation_fast(q1, phi_branch, q5, 0.0);
                const Eigen::Matrix3d delta_R = base_R.transpose() * target_R;
                const double q6 = wrapToPi(std::atan2(delta_R(1, 0), delta_R(0, 0)));
                if (q6 < config_.q_min - kEps || q6 > config_.q_max + kEps) continue;

                OrientationCandidate cand;
                cand.q1 = q1;
                cand.phi = phi_branch;
                cand.q5 = q5;
                cand.q6 = q6;
                cand.score = orientationScore(target_R, make_orientation_fast(cand.q1, cand.phi, cand.q5, cand.q6));
                if (config_.debug && std::abs(q1) < 0.02) {
                    std::cerr << "debug orient-probe q1=" << q1
                              << " q5=" << q5
                              << " phi=" << phi_branch
                              << " q6=" << q6
                              << " score=" << cand.score
                              << " ratio=" << (radius < 1e-9 ? 0.0 : rhs / radius)
                              << "\n";
                }
                if (cand.score <= std::sqrt(2.0) * config_.orientation_tolerance) {
                    add_orientation_candidate(orientation_candidates, cand);
                }
            }
        }
    };

    const double step = std::max(1e-5, std::abs(config_.q1_scan_step));
    const int count = static_cast<int>(std::ceil((config_.q1_max - config_.q1_min) / step));
    for (int i = 0; i <= count; ++i) {
        const double q1 = std::min(config_.q1_max, config_.q1_min + static_cast<double>(i) * step);
        add_orientation_candidates_for_q1(q1);
    }

    std::sort(orientation_candidates.begin(), orientation_candidates.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.score < rhs.score; });

    auto add_solution_from_orientation = [&](const OrientationCandidate& orientation) {
        const double q1 = orientation.q1;
        const double phi = orientation.phi;
        const double q5 = orientation.q5;
        const double q6 = orientation.q6;

        std::vector<double> wrist_pose_joints = {q1, 0.0, 0.0, q4_sign * phi, q5, q6};
        const Eigen::Isometry3d zero_pose = fk(wrist_pose_joints);
        const Eigen::Vector3d zero_tool = zero_pose.translation();
        const Eigen::Vector3d zero_joint4_axis = joint4AxisAtZeroTriangle(q1, phi, q5, q6);
        Eigen::Vector3d wrist_target = target_p - (zero_tool - zero_joint4_axis);

        const Eigen::Matrix3d joint2_frame = rz(q1) * chain_[1].rpy;
        const Eigen::Vector3d joint2_world = rz(q1) * joint2_origin;
        const Eigen::Vector3d p_local = joint2_frame.transpose() * (wrist_target - joint2_world);
        const double x = p_local.x();
        const double y = p_local.y();
        const double d2 = x * x + y * y;
        const double c3_delta = (d2 - a_len * a_len - b_len * b_len) / (2.0 * a_len * b_len);
        if (config_.debug) {
            std::cerr << "debug q1=" << q1 << " phi=" << phi
                      << " q5=" << q5 << " q6=" << q6
                      << " p_local=" << p_local.transpose()
                      << " c3_delta=" << c3_delta << "\n";
        }
        if (c3_delta < -1.0 || c3_delta > 1.0) return;
        const double delta_abs = std::acos(std::clamp(c3_delta, -1.0, 1.0));
        for (double delta : {delta_abs, -delta_abs}) {
            const double q3 = wrapToPi(a_angle - b_angle + delta);
            const Eigen::Vector2d inner(
                a_len * std::cos(a_angle) + b_len * std::cos(q3 + b_angle),
                a_len * std::sin(a_angle) + b_len * std::sin(q3 + b_angle));
            const double q2 = wrapToPi(std::atan2(y, x) - std::atan2(inner.y(), inner.x()));
            const double q4 = q4_sign * (phi - q2 - q3);
            std::vector<double> q = {
                q1,
                wrapToPi(q2),
                wrapToPi(q3),
                wrapToPi(q4),
                q5,
                q6,
            };
            if (!inLimits(q)) continue;

            const auto actual = fk(q);
            const double pe = (actual.translation() - target_p).norm();
            const double oe = orientationError(target_R, actual.linear());
            if (config_.debug) {
                std::cerr << "debug cand q=";
                for (double value : q) std::cerr << value << ",";
                std::cerr << " pe=" << pe << " oe=" << oe << "\n";
            }
            if (pe <= config_.position_tolerance && oe <= config_.orientation_tolerance) {
                AnalyticV5IkSolution sol;
                sol.valid = true;
                std::ostringstream branch;
                branch << "q1_orient_true_2r_" << (delta >= 0.0 ? "plus" : "minus");
                sol.branch = branch.str();
                sol.joint_values = q;
                sol.position_error = pe;
                sol.orientation_error = oe;
                sol.score = pe * 5.0 + oe;
                out.push_back(std::move(sol));
                std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.score < b.score; });
                if (out.size() > config_.max_solutions) out.resize(config_.max_solutions);
            }
        }
    };

    for (const auto& orientation : orientation_candidates) {
        add_solution_from_orientation(orientation);
        if (out.size() >= config_.max_solutions) break;
    }

    return out;
}

} // namespace ik_benchmark
