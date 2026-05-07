#include "ik_benchmark/ik_solver.h"
#include <moveit/robot_state/robot_state.h>
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace ik_benchmark {

IkSolver::IkSolver(const std::string& urdf_path,
                   const std::string& srdf_path,
                   const std::string& group_name,
                   const std::string& solver_plugin,
                   double timeout)
    : group_name_(group_name)
    , solver_plugin_(solver_plugin)
    , default_timeout_(timeout)
{
    is_dual_ = (group_name == "dual_arm_with_base" || group_name == "dual_arms");
    loadRobotModel(urdf_path, srdf_path);
    loadIkPlugin();
}

void IkSolver::loadRobotModel(const std::string& urdf_path, const std::string& srdf_path)
{
    // 解析 URDF
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
        throw std::runtime_error("Cannot open URDF: " + urdf_path);
    }
    std::string urdf_str((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
    auto urdf_model = urdf_parser::parseURDF(urdf_str);
    if (!urdf_model) {
        throw std::runtime_error("Failed to parse URDF");
    }

    // 解析 SRDF
    std::ifstream srdf_file(srdf_path);
    if (!srdf_file.is_open()) {
        throw std::runtime_error("Cannot open SRDF: " + srdf_path);
    }
    std::string srdf_str((std::istreambuf_iterator<char>(srdf_file)),
                          std::istreambuf_iterator<char>());
    srdf::Model srdf_model;
    if (!srdf_model.initString(urdf_model, srdf_str)) {
        throw std::runtime_error("Failed to parse SRDF");
    }

    // 构建 RobotModel
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

    jmg_ = robot_model_->getJointModelGroup(group_name_);
    if (!jmg_) {
        auto groups = robot_model_->getJointModelGroupNames();
        throw std::runtime_error("Group '" + group_name_ +
            "' not found. Available: " + [&]{
                std::string s; for (auto& g : groups) s += g + " "; return s;
            }());
    }

    joint_names_ = jmg_->getActiveJointModelNames();

    // 确定末端 link
    if (is_dual_) {
        tip_link_  = "leftjoint6_link";
        tip_link2_ = "rightjoint6_link";
    } else if (group_name_.find("left") != std::string::npos) {
        tip_link_ = "leftjoint6_link";
    } else {
        tip_link_ = "rightjoint6_link";
    }
}

void IkSolver::loadIkPlugin()
{
    loader_ = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
        "moveit_core", "kinematics::KinematicsBase");

    try {
        ik_solver_ = loader_->createSharedInstance(solver_plugin_);
    } catch (const pluginlib::PluginlibException& e) {
        throw std::runtime_error("Failed to load IK plugin '" +
            solver_plugin_ + "': " + e.what());
    }

    // 初始化求解器
    if (is_dual_) {
        std::vector<std::string> tips = {tip_link_, tip_link2_};
        ik_solver_->initialize(robot_model_, group_name_, "base_link", tips, 0.01);
    } else {
        ik_solver_->initialize(robot_model_, group_name_, "base_link", tip_link_, 0.01);
    }
}

IkResult IkSolver::solve(const Eigen::Isometry3d& target,
                          const std::vector<double>& seed,
                          double timeout)
{
    if (is_dual_) {
        throw std::runtime_error("Use solveDual() for dual-arm groups");
    }

    double t = (timeout > 0) ? timeout : default_timeout_;
    IkResult result;
    result.joint_names = joint_names_;

    // seed
    std::vector<double> seed_vals = seed.empty()
        ? std::vector<double>(joint_names_.size(), 0.0) : seed;

    // Eigen → geometry_msgs::Pose
    geometry_msgs::msg::Pose pose_msg;
    Eigen::Quaterniond q(target.linear());
    pose_msg.position.x  = target.translation().x();
    pose_msg.position.y  = target.translation().y();
    pose_msg.position.z  = target.translation().z();
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    moveit::core::RobotState state(robot_model_);
    state.setVariablePositions(joint_names_, seed_vals);

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = ik_solver_->searchPositionIK(pose_msg, state, t, solution, error_code);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.success = ok;
    result.joint_values = solution;

    if (ok) {
        state.setVariablePositions(joint_names_, solution);
        Eigen::Isometry3d actual = state.getGlobalLinkTransform(tip_link_);
        result.pos_error = (target.translation() - actual.translation()).norm();
        Eigen::AngleAxisd aa(target.linear().transpose() * actual.linear());
        result.ori_error = aa.angle();
    }

    return result;
}

IkResult IkSolver::solveDual(const Eigen::Isometry3d& left_target,
                              const Eigen::Isometry3d& right_target,
                              const std::vector<double>& seed,
                              double timeout)
{
    if (!is_dual_) {
        throw std::runtime_error("Use solve() for single-arm groups");
    }

    double t = (timeout > 0) ? timeout : default_timeout_;
    IkResult result;
    result.joint_names = joint_names_;

    std::vector<double> seed_vals = seed.empty()
        ? std::vector<double>(joint_names_.size(), 0.0) : seed;

    // 两个目标位姿
    auto to_msg = [](const Eigen::Isometry3d& tf) -> geometry_msgs::msg::Pose {
        geometry_msgs::msg::Pose msg;
        Eigen::Quaterniond q(tf.linear());
        msg.position.x  = tf.translation().x();
        msg.position.y  = tf.translation().y();
        msg.position.z  = tf.translation().z();
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        return msg;
    };

    std::vector<geometry_msgs::msg::Pose> targets = {
        to_msg(left_target), to_msg(right_target)
    };

    moveit::core::RobotState state(robot_model_);
    state.setVariablePositions(joint_names_, seed_vals);

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = ik_solver_->searchPositionIK(targets, state, t, solution, error_code);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.success = ok;
    result.joint_values = solution;

    if (ok) {
        state.setVariablePositions(joint_names_, solution);
        Eigen::Isometry3d actual_left  = state.getGlobalLinkTransform(tip_link_);
        Eigen::Isometry3d actual_right = state.getGlobalLinkTransform(tip_link2_);
        double pos_l = (left_target.translation() - actual_left.translation()).norm();
        double pos_r = (right_target.translation() - actual_right.translation()).norm();
        Eigen::AngleAxisd aa_l(left_target.linear().transpose() * actual_left.linear());
        Eigen::AngleAxisd aa_r(right_target.linear().transpose() * actual_right.linear());
        result.pos_error = (pos_l + pos_r) / 2.0;
        result.ori_error = (aa_l.angle() + aa_r.angle()) / 2.0;
    }

    return result;
}

std::vector<Eigen::Isometry3d> IkSolver::fk(const std::vector<double>& joint_values)
{
    moveit::core::RobotState state(robot_model_);
    state.setVariablePositions(joint_names_, joint_values);

    std::vector<Eigen::Isometry3d> poses;
    poses.push_back(state.getGlobalLinkTransform(tip_link_));
    if (is_dual_) {
        poses.push_back(state.getGlobalLinkTransform(tip_link2_));
    }
    return poses;
}

std::vector<double> IkSolver::getHomeSeed() const
{
    return std::vector<double>(joint_names_.size(), 0.0);
}

std::vector<double> IkSolver::getRandomSeed() const
{
    moveit::core::RobotState state(robot_model_);
    state.setToRandomPositions(jmg_);
    // clamp revolute joints to limits
    for (auto* jm : jmg_->getActiveJointModels()) {
        if (jm->getType() == moveit::core::JointModel::REVOLUTE) {
            double val = state.getJointPositions(jm)[0];
            const auto& bounds = jm->getVariableBounds()[0];
            val = std::max(bounds.min_position_, std::min(bounds.max_position_, val));
            state.setJointPositions(jm, &val);
        }
    }
    std::vector<double> vals;
    state.copyJointGroupPositions(jmg_, vals);
    return vals;
}

} // namespace ik_benchmark
