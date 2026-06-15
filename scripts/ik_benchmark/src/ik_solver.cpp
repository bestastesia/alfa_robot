#include "ik_benchmark/ik_solver.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <srdfdom/model.h>
#include <urdf/urdf/model.h>

namespace ik_benchmark {
namespace {

bool isV5ArmLink(const std::string& link)
{
    return link.rfind("left_v5_", 0) == 0 || link.rfind("right_v5_", 0) == 0;
}

bool isBaseStructureLink(const std::string& link)
{
    return link == "base_link" || link == "pitch" || link == "turn" || link == "updown";
}

bool isIgnoredArmBasePair(const std::string& link1, const std::string& link2)
{
    return (link1 == "updown" && (link2 == "left_v5_link1" || link2 == "right_v5_link1")) ||
           (link2 == "updown" && (link1 == "left_v5_link1" || link1 == "right_v5_link1"));
}

bool isArmBaseCollisionPair(const std::string& link1, const std::string& link2)
{
    if (isIgnoredArmBasePair(link1, link2)) {
        return false;
    }
    return (isV5ArmLink(link1) && isBaseStructureLink(link2)) ||
           (isV5ArmLink(link2) && isBaseStructureLink(link1));
}

} // namespace

IkSolver::IkSolver(const std::string& group_name,
                   const std::string& solver_plugin,
                   double timeout,
                   bool free_joint6,
                   const IkSolverOptions& options)
    : group_name_(group_name)
    , solver_plugin_(solver_plugin)
    , default_timeout_(timeout)
    , free_joint6_(free_joint6)
    , options_(options)
{
    is_dual_ = (group_name == "dual_arm_with_base" ||
                group_name == "dual_arms" ||
                group_name == "dual_v5_arm_with_base" ||
                group_name == "dual_v5_arm");

    node_ = std::make_shared<rclcpp::Node>(
        "_ik_bench_node",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(false)
            .use_global_arguments(false));

    loadRobotModel();
    loadIkPlugin();
}

void IkSolver::loadRobotModel()
{
    std::string desc_share;
    std::string moveit_share;
    if (options_.urdf_path.empty()) {
        desc_share = ament_index_cpp::get_package_share_directory("alfa_robot_description");
    }
    if (options_.srdf_path.empty()) {
        moveit_share = ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");
    }

    std::string urdf_path = options_.urdf_path.empty()
        ? desc_share + "/urdf/alfa_robot/alfa_robot.urdf"
        : options_.urdf_path;
    if (options_.urdf_path.empty()) {
        std::ifstream test(urdf_path);
        if (!test.good()) {
            std::string xacro_src = desc_share + "/urdf/alfa_robot.urdf.xacro";
            int ret = std::system(("xacro " + xacro_src + " > /tmp/_alfa_bench.urdf 2>/dev/null").c_str());
            if (ret != 0) {
                throw std::runtime_error("xacro failed — source install/setup.bash?");
            }
            urdf_path = "/tmp/_alfa_bench.urdf";
        }
    }

    std::string srdf_path = options_.srdf_path.empty()
        ? moveit_share + "/config/alfa_robot.srdf"
        : options_.srdf_path;

    auto read_file = [](const std::string& path) -> std::string {
        std::ifstream f(path);
        if (!f.good()) throw std::runtime_error("Cannot read: " + path);
        return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    };

    std::string urdf_xml = read_file(urdf_path);
    std::string srdf_xml = read_file(srdf_path);

    auto urdf_model = std::make_shared<urdf::Model>();
    if (!urdf_model->initString(urdf_xml)) {
        throw std::runtime_error("Failed to parse URDF");
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, srdf_xml)) {
        throw std::runtime_error("Failed to parse SRDF");
    }

    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    auto& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    for (const auto& collision_pair : srdf_model->getDisabledCollisionPairs()) {
        acm.setEntry(collision_pair.link1_, collision_pair.link2_, true);
    }
    if (options_.enforce_arm_base_collisions) {
        for (const auto& link1 : robot_model_->getLinkModelNames()) {
            for (const auto& link2 : robot_model_->getLinkModelNames()) {
                if (isArmBaseCollisionPair(link1, link2)) {
                    acm.setEntry(link1, link2, false);
                }
            }
        }
    }

    jmg_ = robot_model_->getJointModelGroup(group_name_);
    if (!jmg_) {
        auto groups = robot_model_->getJointModelGroupNames();
        throw std::runtime_error("Group '" + group_name_ +
            "' not found. Available: " + [&]{
                std::string s; for (auto& g : groups) s += g + " "; return s;
            }());
    }

    variable_names_ = jmg_->getVariableNames();

    // 确定 base_frame: 对于含基座关节的组用 base_link，否则用第一个关节的 parent link
    if (!options_.base_frame.empty()) {
        base_frame_ = options_.base_frame;
    } else if (is_dual_ || group_name_.find("_with_base") != std::string::npos) {
        base_frame_ = "base_link";
    } else {
        // 找到组中第一个 active joint 的 parent link
        const auto& active_jmodels = jmg_->getActiveJointModels();
        if (!active_jmodels.empty()) {
            base_frame_ = active_jmodels[0]->getParentLinkModel()->getName();
        } else {
            base_frame_ = "base_link";
        }
    }

    if (is_dual_) {
        tip_link_  = options_.tip_link.empty() ? "leftjoint6" : options_.tip_link;
        tip_link2_ = options_.tip_link2.empty() ? "rightjoint6" : options_.tip_link2;
    } else if (group_name_.find("left") != std::string::npos) {
        tip_link_ = options_.tip_link.empty() ? "leftjoint6" : options_.tip_link;
    } else {
        tip_link_ = options_.tip_link.empty() ? "rightjoint6" : options_.tip_link;
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

    declareSolverParams();

    if (is_dual_) {
        std::vector<std::string> tips = {tip_link_, tip_link2_};
        ik_solver_->initialize(node_, *robot_model_, group_name_, base_frame_, tips, 0.005);
    } else {
        std::vector<std::string> tips = {tip_link_};
        ik_solver_->initialize(node_, *robot_model_, group_name_, base_frame_, tips, 0.005);
    }

    // IK 插件初始化后，构建关节映射
    buildIkMapping();
}

void IkSolver::declareSolverParams()
{
    const std::string ns = "robot_description_kinematics." + group_name_;

    node_->declare_parameter(ns + ".kinematics_solver",                   solver_plugin_);
    node_->declare_parameter(ns + ".kinematics_solver_search_resolution", 0.005);
    node_->declare_parameter(ns + ".kinematics_solver_timeout",           default_timeout_);
    node_->declare_parameter(ns + ".kinematics_solver_attempts",          10);

    if (solver_plugin_.find("pick_ik") != std::string::npos) {
        node_->declare_parameter(ns + ".mode",                        std::string("global"));
        node_->declare_parameter(ns + ".position_threshold",          0.001);
        node_->declare_parameter(ns + ".orientation_threshold",       0.01);
        node_->declare_parameter(ns + ".position_scale",              1.0);
        node_->declare_parameter(ns + ".rotation_scale",              0.5);
        node_->declare_parameter(ns + ".minimal_displacement_weight", 0.001);
        node_->declare_parameter(ns + ".memetic_num_threads",         4);
        node_->declare_parameter(ns + ".memetic_population_size",     32);
        node_->declare_parameter(ns + ".memetic_max_generations",     200);
        node_->declare_parameter(ns + ".memetic_elite_size",          8);
        node_->declare_parameter(ns + ".cost_threshold",              0.1);
        node_->declare_parameter(ns + ".fix_unspecified_end_effectors", true);
    }

    if (solver_plugin_.find("bio_ik") != std::string::npos) {
        node_->declare_parameter(ns + ".bio_ik_max_computation_time", default_timeout_);
    }
}

void IkSolver::buildIkMapping()
{
    ik_joint_names_ = ik_solver_->getJointNames();

    // IK solver 的关节可能超出 JMG 的变量范围 (如 TRAC-IK 包含 turn/updown)
    // 对于超出 JMG 范围的关节，标记为 base_joint，seed 中设为 0
    ik_to_jmg_index_.resize(ik_joint_names_.size(), SIZE_MAX);

    for (size_t ik_i = 0; ik_i < ik_joint_names_.size(); ++ik_i) {
        for (size_t jmg_i = 0; jmg_i < variable_names_.size(); ++jmg_i) {
            if (variable_names_[jmg_i] == ik_joint_names_[ik_i]) {
                ik_to_jmg_index_[ik_i] = jmg_i;
                break;
            }
        }
    }

    // IK solution → JMG variable 映射：哪些 IK joint 对应 JMG variable
    for (size_t ik_i = 0; ik_i < ik_joint_names_.size(); ++ik_i) {
        if (ik_to_jmg_index_[ik_i] != SIZE_MAX) {
            solution_to_jmg_.push_back({ik_i, ik_to_jmg_index_[ik_i]});
        }
    }
}

std::vector<double> IkSolver::makeIkSeed(const std::vector<double>& jmg_seed) const
{
    // 从 JMG seed 提取 IK solver 需要的关节值
    // 对于不在 JMG 中的关节 (turn/updown)，设为 0
    std::vector<double> ik_seed(ik_joint_names_.size(), 0.0);
    for (size_t ik_i = 0; ik_i < ik_joint_names_.size(); ++ik_i) {
        size_t jmg_i = ik_to_jmg_index_[ik_i];
        if (jmg_i != SIZE_MAX && jmg_i < jmg_seed.size()) {
            ik_seed[ik_i] = jmg_seed[jmg_i];
        }
    }
    return ik_seed;
}

bool IkSolver::isSolutionCollisionFree(const std::vector<double>& solution,
                                       std::vector<std::string>* collision_pairs) const
{
    if (!planning_scene_) {
        return true;
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    const auto& ik_jnames = ik_solver_->getJointNames();
    for (size_t k = 0; k < solution.size() && k < ik_jnames.size(); ++k) {
        state.setJointPositions(ik_jnames[k], {solution[k]});
    }
    state.update();
    state.updateCollisionBodyTransforms();

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = group_name_;
    req.contacts = (collision_pairs != nullptr);
    req.max_contacts = 20;
    req.max_contacts_per_pair = 1;
    req.verbose = false;

    planning_scene_->checkCollision(req, res, state, planning_scene_->getAllowedCollisionMatrix());

    if (collision_pairs) {
        collision_pairs->clear();
        for (const auto& entry : res.contacts) {
            collision_pairs->push_back(entry.first.first + " <-> " + entry.first.second);
        }
    }

    return !res.collision;
}

bool IkSolver::isNamedStateCollisionFree(const std::vector<std::string>& joint_names,
                                         const std::vector<double>& joint_values,
                                         std::vector<std::string>* collision_pairs) const
{
    if (!planning_scene_) {
        return true;
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (size_t k = 0; k < joint_names.size() && k < joint_values.size(); ++k) {
        state.setJointPositions(joint_names[k], {joint_values[k]});
    }
    state.update();
    state.updateCollisionBodyTransforms();

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = (collision_pairs != nullptr);
    req.max_contacts = 20;
    req.max_contacts_per_pair = 1;
    req.verbose = false;

    planning_scene_->checkCollision(req, res, state, planning_scene_->getAllowedCollisionMatrix());

    if (collision_pairs) {
        collision_pairs->clear();
        for (const auto& entry : res.contacts) {
            collision_pairs->push_back(entry.first.first + " <-> " + entry.first.second);
        }
    }

    return !res.collision;
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
    result.joint_names = ik_joint_names_;

    std::vector<double> jmg_seed = seed.empty() ? getHomeSeed() : seed;
    std::vector<double> ik_seed = makeIkSeed(jmg_seed);

    geometry_msgs::msg::Pose pose_msg;
    Eigen::Quaterniond q(target.linear());
    pose_msg.position.x  = target.translation().x();
    pose_msg.position.y  = target.translation().y();
    pose_msg.position.z  = target.translation().z();
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    int collision_rejections = 0;
    std::vector<std::string> first_collision_pairs;
    std::vector<double> first_collision_solution;
    kinematics::KinematicsBase::IKCallbackFn callback;
    if (options_.reject_collisions) {
        callback = [this, &collision_rejections, &first_collision_pairs, &first_collision_solution](
                       const geometry_msgs::msg::Pose&, const std::vector<double>& solution,
                       moveit_msgs::msg::MoveItErrorCodes& callback_error_code) {
            std::vector<std::string> candidate_pairs;
            if (isSolutionCollisionFree(solution, &candidate_pairs)) {
                callback_error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            } else {
                ++collision_rejections;
                if (first_collision_pairs.empty()) {
                    first_collision_pairs = candidate_pairs;
                    first_collision_solution = solution;
                }
                callback_error_code.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION;
            }
        };
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = ik_solver_->searchPositionIK(pose_msg, ik_seed, t, solution, callback, error_code, options);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.success = ok;
    result.joint_values = solution;
    result.collision_checked = true;
    result.collision_rejection_count = collision_rejections;

    if (ok) {
        result.collision_free = isSolutionCollisionFree(solution, &result.collision_pairs);
        if (options_.reject_collisions && !result.collision_free) {
            result.success = false;
            return result;
        }
    } else if (!first_collision_pairs.empty()) {
        result.collision_free = false;
        result.collision_pairs = first_collision_pairs;
        result.joint_values = first_collision_solution;
    }

    if (ok) {
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();
        const auto& ik_jnames = ik_solver_->getJointNames();
        for (size_t k = 0; k < solution.size(); ++k) {
            state.setJointPositions(ik_jnames[k], {solution[k]});
        }
        state.update();

        const Eigen::Isometry3d T_base_inv =
            state.getGlobalLinkTransform(base_frame_).inverse();
        Eigen::Isometry3d actual = T_base_inv * state.getGlobalLinkTransform(tip_link_);
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
    return solveDual(left_target, right_target, seed, timeout,
                     -std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity());
}

IkResult IkSolver::solveDual(const Eigen::Isometry3d& left_target,
                              const Eigen::Isometry3d& right_target,
                              const std::vector<double>& seed,
                              double timeout,
                              double updown_lower,
                              double updown_upper)
{
    if (!is_dual_) {
        throw std::runtime_error("Use solve() for single-arm groups");
    }

    double t = (timeout > 0) ? timeout : default_timeout_;
    IkResult result;
    result.joint_names = ik_joint_names_;

    std::vector<double> jmg_seed = seed.empty() ? getHomeSeed() : seed;
    std::vector<double> ik_seed = makeIkSeed(jmg_seed);

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

    // BioIK stores multi-tip goals in the plugin's internal tip order, which is
    // reversed from the explicit {left, right} order passed to initialize() for
    // the current dual_v5 groups. Keep IkSolver's public API as left/right and
    // compensate here so FK(actual[0]) still means left tip.
    std::vector<geometry_msgs::msg::Pose> targets = {
        to_msg(right_target), to_msg(left_target)
    };

    std::vector<double> consistency_limits;
    const bool restrict_updown = std::isfinite(updown_lower) && std::isfinite(updown_upper) && updown_lower <= updown_upper;
    moveit::core::VariableBounds original_updown_bounds;
    bool changed_updown_bounds = false;
    if (restrict_updown) {
        const auto& ik_jnames = ik_solver_->getJointNames();
        for (size_t k = 0; k < ik_jnames.size() && k < ik_seed.size(); ++k) {
            if (ik_jnames[k] == "updown") {
                ik_seed[k] = std::min(std::max(ik_seed[k], updown_lower), updown_upper);
                break;
            }
        }
        if (auto* updown_joint = robot_model_->getJointModel("updown")) {
            original_updown_bounds = updown_joint->getVariableBounds("updown");
            auto restricted = original_updown_bounds;
            restricted.min_position_ = std::max(restricted.min_position_, updown_lower);
            restricted.max_position_ = std::min(restricted.max_position_, updown_upper);
            if (restricted.min_position_ <= restricted.max_position_) {
                updown_joint->setVariableBounds("updown", restricted);
                changed_updown_bounds = true;
            }
        }
    }
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    int collision_rejections = 0;
    std::vector<std::string> first_collision_pairs;
    std::vector<double> first_collision_solution;
    kinematics::KinematicsBase::IKCallbackFn callback;
    if (options_.reject_collisions) {
        callback = [this, &collision_rejections, &first_collision_pairs, &first_collision_solution](
                       const geometry_msgs::msg::Pose&, const std::vector<double>& candidate_solution,
                       moveit_msgs::msg::MoveItErrorCodes& callback_error_code) {
            std::vector<std::string> candidate_pairs;
            if (isSolutionCollisionFree(candidate_solution, &candidate_pairs)) {
                callback_error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            } else {
                ++collision_rejections;
                if (first_collision_pairs.empty()) {
                    first_collision_pairs = candidate_pairs;
                    first_collision_solution = candidate_solution;
                }
                callback_error_code.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION;
            }
        };
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = ik_solver_->searchPositionIK(targets, ik_seed, t,
                                            consistency_limits, solution,
                                            callback, error_code, options);
    auto t1 = std::chrono::high_resolution_clock::now();
    if (changed_updown_bounds) {
        if (auto* updown_joint = robot_model_->getJointModel("updown")) {
            updown_joint->setVariableBounds("updown", original_updown_bounds);
        }
    }

    result.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.success = ok;
    result.joint_values = solution;
    result.collision_checked = true;
    result.collision_rejection_count = collision_rejections;

    if (ok) {
        result.collision_free = isSolutionCollisionFree(solution, &result.collision_pairs);
        if (options_.reject_collisions && !result.collision_free) {
            result.success = false;
            return result;
        }
    } else if (!first_collision_pairs.empty()) {
        result.collision_free = false;
        result.collision_pairs = first_collision_pairs;
        result.joint_values = first_collision_solution;
    }

    if (ok) {
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();
        const auto& ik_jnames = ik_solver_->getJointNames();
        for (size_t k = 0; k < solution.size(); ++k) {
            state.setJointPositions(ik_jnames[k], {solution[k]});
        }
        state.update();

        const Eigen::Isometry3d T_base_inv =
            state.getGlobalLinkTransform(base_frame_).inverse();
        Eigen::Isometry3d actual_left  = T_base_inv * state.getGlobalLinkTransform(tip_link_);
        Eigen::Isometry3d actual_right = T_base_inv * state.getGlobalLinkTransform(tip_link2_);
        double pos_l = (left_target.translation() - actual_left.translation()).norm();
        double pos_r = (right_target.translation() - actual_right.translation()).norm();
        Eigen::AngleAxisd aa_l(left_target.linear().transpose() * actual_left.linear());
        Eigen::AngleAxisd aa_r(right_target.linear().transpose() * actual_right.linear());
        result.pos_error = std::max(pos_l, pos_r);
        result.ori_error = std::max(aa_l.angle(), aa_r.angle());
    }

    return result;
}

std::vector<Eigen::Isometry3d> IkSolver::fk(const std::vector<double>& joint_values)
{
    // joint_values 维度 = ik_joint_names_.size()
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (size_t k = 0; k < joint_values.size() && k < ik_joint_names_.size(); ++k) {
        state.setJointPositions(ik_joint_names_[k], {joint_values[k]});
    }
    state.update();

    const Eigen::Isometry3d T_base_inv =
        state.getGlobalLinkTransform(base_frame_).inverse();

    std::vector<Eigen::Isometry3d> poses;
    poses.push_back(T_base_inv * state.getGlobalLinkTransform(tip_link_));
    if (is_dual_) {
        poses.push_back(T_base_inv * state.getGlobalLinkTransform(tip_link2_));
    }
    return poses;
}

std::vector<Eigen::Isometry3d> IkSolver::fkNamed(const std::vector<std::string>& joint_names,
                                                 const std::vector<double>& joint_values)
{
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (size_t k = 0; k < joint_names.size() && k < joint_values.size(); ++k) {
        state.setJointPositions(joint_names[k], {joint_values[k]});
    }
    state.update();

    const Eigen::Isometry3d T_base_inv =
        state.getGlobalLinkTransform(base_frame_).inverse();

    std::vector<Eigen::Isometry3d> poses;
    poses.push_back(T_base_inv * state.getGlobalLinkTransform(tip_link_));
    if (is_dual_) {
        poses.push_back(T_base_inv * state.getGlobalLinkTransform(tip_link2_));
    }
    return poses;
}

Eigen::Isometry3d IkSolver::linkTransformNamed(
    const std::string& link_name,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_values) const
{
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    for (size_t k = 0; k < joint_names.size() && k < joint_values.size(); ++k) {
        state.setJointPositions(joint_names[k], {joint_values[k]});
    }
    state.update();

    const Eigen::Isometry3d T_base_inv =
        state.getGlobalLinkTransform(base_frame_).inverse();
    return T_base_inv * state.getGlobalLinkTransform(link_name);
}

std::vector<double> IkSolver::getHomeSeed() const
{
    // 返回 IK solver 维度的 home seed (全零)
    return std::vector<double>(ik_joint_names_.size(), 0.0);
}

std::vector<double> IkSolver::getRandomSeed() const
{
    // 生成 IK solver 维度的随机 seed
    std::mt19937 rng(42);
    std::vector<double> seed(ik_joint_names_.size(), 0.0);
    for (size_t k = 0; k < ik_joint_names_.size(); ++k) {
        const moveit::core::JointModel* jm = robot_model_->getJointModel(ik_joint_names_[k]);
        if (jm) {
            const auto& bounds = jm->getVariableBounds();
            if (!bounds.empty() && bounds[0].position_bounded_) {
                std::uniform_real_distribution<double> dist(bounds[0].min_position_, bounds[0].max_position_);
                seed[k] = dist(rng);
            } else {
                // continuous 或 unbounded joint: 随机 [-π, π]
                std::uniform_real_distribution<double> dist(-M_PI, M_PI);
                seed[k] = dist(rng);
            }
        }
    }

    // 固定 joint6 = 0
    if (!free_joint6_) {
        for (size_t k = 0; k < ik_joint_names_.size(); ++k) {
            if (ik_joint_names_[k] == "leftjoint6" || ik_joint_names_[k] == "rightjoint6") {
                seed[k] = 0.0;
            }
        }
    }

    return seed;
}

} // namespace ik_benchmark
