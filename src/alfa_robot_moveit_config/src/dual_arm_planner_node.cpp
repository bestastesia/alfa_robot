/**
 * dual_arm_planner_node.cpp
 *
 * 双末端位姿规划节点。
 *
 * 核心思路：
 *   1. 手动构造 RobotState，调用 setFromIK(group, {left_pose, right_pose}, tips)
 *      → bio_ik 一次性看到两个末端目标，统一协调 updown 关节，输出完整关节角
 *   2. 把该 RobotState 通过 setJointValueTarget(robot_state) 传给规划器
 *      → 规划器只做路径规划，不再需要 IkConstraintSampler（彻底绕开多 tip 冲突）
 *
 * 对外接口：
 *   ROS 2 service  ~/plan_and_execute  (std_srvs/Trigger) — 触发 demo
 *   C++ API:
 *     plan_and_execute(left_pose, right_pose, execute)
 *     plan_and_execute_single(move_left, target_pose, execute)
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const std::string PLANNING_GROUP = "dual_arm_with_base";
static const std::string LEFT_TIP       = "leftjoint6_link";
static const std::string RIGHT_TIP      = "rightjoint4_link";
static const std::string BASE_FRAME     = "base_link";

class DualArmPlannerNode : public rclcpp::Node
{
public:
  explicit DualArmPlannerNode(const rclcpp::NodeOptions & options)
  : Node("dual_arm_planner", options)
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void init()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    move_group_->setPlanningTime(15.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    // 获取机器人模型和规划组，用于手动调用 setFromIK
    robot_model_  = move_group_->getRobotModel();
    joint_group_  = robot_model_->getJointModelGroup(PLANNING_GROUP);

    plan_exec_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        resp->success = demo_move();
        resp->message = resp->success ? "success" : "failed";
      });

    RCLCPP_INFO(get_logger(), "DualArmPlannerNode ready");
    RCLCPP_INFO(get_logger(), "  Group   : %s", PLANNING_GROUP.c_str());
    RCLCPP_INFO(get_logger(), "  Left tip: %s", LEFT_TIP.c_str());
    RCLCPP_INFO(get_logger(), "  Right tip:%s", RIGHT_TIP.c_str());
  }

  /**
   * 核心接口：双末端位姿 → bio_ik 求关节角 → 路径规划 → （可选）执行
   *
   * 步骤：
   *   a) 用当前真实关节角初始化 RobotState（作为 IK 种子）
   *   b) setFromIK(group, {left_pose, right_pose}, {LEFT_TIP, RIGHT_TIP}, timeout)
   *      bio_ik 统一优化 updown + 双臂关节
   *   c) setJointValueTarget(ik_state) → plan() → execute()
   */
  bool plan_and_execute(const geometry_msgs::msg::Pose & left_pose,
                        const geometry_msgs::msg::Pose & right_pose,
                        bool execute = true)
  {
    // --- a) 构造种子状态（当前真实关节值）---
    auto ik_state = std::make_shared<moveit::core::RobotState>(robot_model_);
    auto current = move_group_->getCurrentState(2.0);
    if (!current) {
      RCLCPP_ERROR(get_logger(), "无法获取当前机器人状态");
      return false;
    }
    *ik_state = *current;

    // --- b) 双末端 IK，bio_ik 统一求解 ---
    EigenSTL::vector_Isometry3d poses(2);
    tf2::fromMsg(left_pose,  poses[0]);
    tf2::fromMsg(right_pose, poses[1]);

    std::vector<std::string> tips = { LEFT_TIP, RIGHT_TIP };

    RCLCPP_INFO(get_logger(),
      "调用 bio_ik 双末端 IK:\n"
      "  左臂目标: (%.3f, %.3f, %.3f)\n"
      "  右臂目标: (%.3f, %.3f, %.3f)",
      left_pose.position.x,  left_pose.position.y,  left_pose.position.z,
      right_pose.position.x, right_pose.position.y, right_pose.position.z);

    bool ik_ok = ik_state->setFromIK(joint_group_, poses, tips, /*timeout=*/2.0);
    if (!ik_ok) {
      RCLCPP_ERROR(get_logger(), "bio_ik 双末端 IK 求解失败");
      return false;
    }

    // 打印 bio_ik 求出的关节角（便于调试）
    std::vector<double> jv;
    ik_state->copyJointGroupPositions(joint_group_, jv);
    auto & jnames = joint_group_->getVariableNames();
    RCLCPP_INFO(get_logger(), "bio_ik IK 解:");
    for (size_t i = 0; i < jnames.size(); ++i)
      RCLCPP_INFO(get_logger(), "  %s = %.4f", jnames[i].c_str(), jv[i]);

    // --- c) 用关节目标规划（绕开 IkConstraintSampler）---
    move_group_->setJointValueTarget(*ik_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "路径规划失败，错误码: %d", result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "路径规划成功，轨迹点数: %zu",
      plan.trajectory_.joint_trajectory.points.size());

    if (!execute) return true;

    auto exec = move_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "执行失败，错误码: %d", exec.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "执行成功");
    return true;
  }

  /**
   * 单臂模式：只动一侧，另一侧固定为当前 TF 位姿。
   * bio_ik 仍然同时看到两个约束，updown 协调不丢失。
   */
  bool plan_and_execute_single(bool move_left,
                               const geometry_msgs::msg::Pose & target,
                               bool execute = true)
  {
    geometry_msgs::msg::Pose fixed_pose;
    const std::string & fixed_tip = move_left ? RIGHT_TIP : LEFT_TIP;
    if (!get_current_pose(fixed_tip, fixed_pose)) {
      RCLCPP_ERROR(get_logger(), "无法获取 %s 当前位姿", fixed_tip.c_str());
      return false;
    }

    const auto & left_pose  = move_left ? target     : fixed_pose;
    const auto & right_pose = move_left ? fixed_pose : target;

    RCLCPP_INFO(get_logger(), "%s臂运动，%s臂位姿固定为当前值",
      move_left ? "左" : "右", move_left ? "右" : "左");

    return plan_and_execute(left_pose, right_pose, execute);
  }

private:
  bool get_current_pose(const std::string & link, geometry_msgs::msg::Pose & out)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
        BASE_FRAME, link, tf2::TimePointZero, tf2::durationFromSec(2.0));
      out.position.x    = tf.transform.translation.x;
      out.position.y    = tf.transform.translation.y;
      out.position.z    = tf.transform.translation.z;
      out.orientation   = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF %s->%s 失败: %s", BASE_FRAME.c_str(), link.c_str(), e.what());
      return false;
    }
  }

  // Demo：左臂在当前位姿基础上向上移动 10cm，右臂位姿保持不变
  bool demo_move()
  {
    geometry_msgs::msg::Pose left_current;
    if (!get_current_pose(LEFT_TIP, left_current)) return false;

    auto left_target = left_current;
    left_target.position.z += 0.10;

    RCLCPP_INFO(get_logger(), "Demo: 左臂上移 10cm，右臂固定");
    return plan_and_execute_single(/*move_left=*/true, left_target, /*execute=*/true);
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotModelConstPtr                                robot_model_;
  const moveit::core::JointModelGroup *                          joint_group_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer>                               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                    tf_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             plan_exec_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DualArmPlannerNode>(options);
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
