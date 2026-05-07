#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.hpp>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace ik_benchmark {

struct IkResult {
    bool success = false;
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    double solve_ms = 0.0;
    double pos_error = 0.0;
    double ori_error = 0.0;
};

class IkSolver {
public:
    /// 离线加载: 直接传入 URDF/SRDF 路径
    IkSolver(const std::string& urdf_path,
             const std::string& srdf_path,
             const std::string& group_name,
             const std::string& solver_plugin,
             double timeout = 2.0);

    /// 单臂 IK
    IkResult solve(const Eigen::Isometry3d& target,
                   const std::vector<double>& seed = {},
                   double timeout = 0.0);  // 0 = 用构造时的默认值

    /// 双臂 IK (left_target + right_target)
    IkResult solveDual(const Eigen::Isometry3d& left_target,
                       const Eigen::Isometry3d& right_target,
                       const std::vector<double>& seed = {},
                       double timeout = 0.0);

    /// 正运动学: 给定关节值，返回末端位姿 (base_link 坐标系)
    /// 单臂返回1个，双臂返回2个(left, right)
    std::vector<Eigen::Isometry3d> fk(const std::vector<double>& joint_values);

    /// 从 home 姿态出发的 seed (全零)
    std::vector<double> getHomeSeed() const;

    /// 随机 seed (clamped to joint limits)
    std::vector<double> getRandomSeed() const;

    /// 获取关节名列表
    const std::vector<std::string>& getJointNames() const { return joint_names_; }

    /// 获取规划组名
    const std::string& getGroupName() const { return group_name_; }

    /// 是否为双臂模式
    bool isDualArm() const { return is_dual_; }

private:
    void loadRobotModel(const std::string& urdf_path, const std::string& srdf_path);
    void loadIkPlugin();

    std::string group_name_;
    std::string solver_plugin_;
    double default_timeout_;
    bool is_dual_ = false;

    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* jmg_ = nullptr;
    std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
    kinematics::KinematicsBasePtr ik_solver_;

    std::vector<std::string> joint_names_;
    std::string tip_link_;          // 单臂末端 or 双臂左末端
    std::string tip_link2_;         // 双臂右臂末端
};

} // namespace ik_benchmark
