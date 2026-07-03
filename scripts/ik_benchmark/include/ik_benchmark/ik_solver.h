#pragma once

#include <Eigen/Geometry>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <string>
#include <vector>

namespace planning_scene {
class PlanningScene;
}

namespace ik_benchmark {

struct IkResult {
    bool success = false;
    bool collision_checked = false;
    bool collision_free = false;
    int collision_rejection_count = 0;
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    std::vector<std::string> collision_pairs;
    double solve_ms  = 0.0;
    double pos_error = 0.0;
    double ori_error = 0.0;
};

struct IkSolverOptions {
    std::string urdf_path;
    std::string srdf_path;
    std::string base_frame;
    std::string tip_link;
    std::string tip_link2;
    bool reject_collisions = true;
    bool enforce_arm_base_collisions = false;
};

class IkSolver {
public:
    IkSolver(const std::string& group_name,
             const std::string& solver_plugin,
             double timeout = 2.0,
             bool free_joint6 = false,
             const IkSolverOptions& options = {});

    /// 单臂 IK
    IkResult solve(const Eigen::Isometry3d& target,
                   const std::vector<double>& seed = {},
                   double timeout = 0.0);

    /// 双臂 IK
    IkResult solveDual(const Eigen::Isometry3d& left_target,
                       const Eigen::Isometry3d& right_target,
                       const std::vector<double>& seed = {},
                       double timeout = 0.0);

    IkResult solveDual(const Eigen::Isometry3d& left_target,
                       const Eigen::Isometry3d& right_target,
                       const std::vector<double>& seed,
                       double timeout,
                       double updown_lower,
                       double updown_upper);

    /// 正运动学 (输入维度 = ik_joint_names_.size())
    std::vector<Eigen::Isometry3d> fk(const std::vector<double>& joint_values);

    /// 正运动学 (输入为完整命名关节状态，未提供的关节使用默认值)
    std::vector<Eigen::Isometry3d> fkNamed(const std::vector<std::string>& joint_names,
                                           const std::vector<double>& joint_values);

    /// 返回指定 link 相对当前 solver base frame 的位姿。
    Eigen::Isometry3d linkTransformNamed(const std::string& link_name,
                                         const std::vector<std::string>& joint_names,
                                         const std::vector<double>& joint_values) const;
    Eigen::Isometry3d linkTransformNamedInFrame(const std::string& reference_link,
                                                const std::string& link_name,
                                                const std::vector<std::string>& joint_names,
                                                const std::vector<double>& joint_values) const;

    /// 碰撞检查 (输入为完整命名关节状态，未提供的关节使用默认值)
    bool isNamedStateCollisionFree(const std::vector<std::string>& joint_names,
                                   const std::vector<double>& joint_values,
                                   std::vector<std::string>* collision_pairs = nullptr) const;

    bool isDualArm() const { return is_dual_; }

    /// 返回 IK solver 维度的 home seed
    std::vector<double> getHomeSeed() const;

    /// 返回 IK solver 维度的随机 seed
    std::vector<double> getRandomSeed() const;

    const std::vector<std::string>& ikJointNames() const { return ik_joint_names_; }
    const std::vector<std::string>& variableNames() const { return variable_names_; }

private:
    void loadRobotModel();
    void loadIkPlugin();
    void declareSolverParams();
    void buildIkMapping();
    bool isSolutionCollisionFree(const std::vector<double>& solution,
                                 std::vector<std::string>* collision_pairs = nullptr) const;

    /// 从 JMG seed 构建 IK solver seed
    std::vector<double> makeIkSeed(const std::vector<double>& jmg_seed) const;

    // Robot model
    std::string group_name_;
    std::string solver_plugin_;
    double default_timeout_;
    bool free_joint6_;
    bool is_dual_;
    IkSolverOptions options_;

    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* jmg_ = nullptr;
    std::string tip_link_, tip_link2_;
    std::string base_frame_;

    std::vector<std::string> variable_names_;   // JMG 变量名

    // IK solver 关节映射
    std::vector<std::string> ik_joint_names_;            // IK solver 管理的关节名
    std::vector<size_t> ik_to_jmg_index_;                // ik_to_jmg_index_[ik_i] = JMG 索引, SIZE_MAX = 不在 JMG 中
    std::vector<std::pair<size_t,size_t>> solution_to_jmg_; // (ik_idx, jmg_idx) pairs

    // IK solver
    std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
    kinematics::KinematicsBasePtr ik_solver_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
};

} // namespace ik_benchmark
