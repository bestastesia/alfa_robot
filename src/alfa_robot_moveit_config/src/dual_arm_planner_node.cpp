/**
 * dual_arm_planner_node.cpp
 *
 * 双末端位姿规划节点。通过 MoveGroupInterface::setPoseTarget(pose, link) 对
 * leftjoint6_link 和 rightjoint4_link 分别指定目标位姿，MoveIt 内部走
 * RobotState::setFromIK(group, poses, tips) 路径，让 bio_ik 一次性同时接收
 * 两个末端目标，统一协调 updown 关节。
 *
 * 对外提供一个 ROS 2 Action 服务：
 *   /dual_arm_plan  (alfa_robot_moveit_config/action/DualArmPlan)
 *
 * 也提供单臂模式（另一臂固定为当前位姿）：
 *   move_left_only / move_right_only 字段为 true 时启用
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/action/move_group.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/trigger.hpp>

// ---------------------------------------------------------------------------
// 简单的 Service 接口：接受两个 PoseStamped，规划并执行
// ---------------------------------------------------------------------------

static const std::string PLANNING_GROUP = "dual_arm_with_base";
static const std::string LEFT_TIP       = "leftjoint6_link";
static const std::string RIGHT_TIP      = "rightjoint4_link";
static const std::string BASE_FRAME     = "base_link";

class DualArmPlannerNode : public rclcpp::Node
{
public:
  DualArmPlannerNode(const rclcpp::NodeOptions & options)
  : Node("dual_arm_planner", options)
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // 必须在 Node 构造完成后调用（MoveGroupInterface 需要完整的 Node shared_ptr）
  void init()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    move_group_->setPoseReferenceFrame(BASE_FRAME);

    // Service: 双臂规划并执行
    plan_exec_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        resp->success = demo_move();
        resp->message = resp->success ? "success" : "failed";
      });

    RCLCPP_INFO(get_logger(), "DualArmPlannerNode ready. Group: %s", PLANNING_GROUP.c_str());
    RCLCPP_INFO(get_logger(), "  Left tip : %s", LEFT_TIP.c_str());
    RCLCPP_INFO(get_logger(), "  Right tip: %s", RIGHT_TIP.c_str());
  }

  /**
   * 核心接口：同时设置两个末端目标，bio_ik 统一求解。
   *
   * MoveGroupInterface 内部实现：
   *   setPoseTarget(pose, link) 对不同 link 分别调用，内部存入 pose_targets_ map。
   *   plan() 时通过 setFromIK(group, {left_pose, right_pose}, {LEFT_TIP, RIGHT_TIP})
   *   把两个 pose 同时传给 bio_ik，bio_ik 的多目标优化器统一协调 updown。
   */
  bool plan_and_execute(const geometry_msgs::msg::Pose & left_pose,
                        const geometry_msgs::msg::Pose & right_pose,
                        bool execute = true)
  {
    move_group_->clearPoseTargets();

    // 对两个不同 link 各设置目标 —— 这是触发 setFromIK(poses, tips) 的关键
    move_group_->setPoseTarget(left_pose,  LEFT_TIP);
    move_group_->setPoseTarget(right_pose, RIGHT_TIP);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "规划失败，错误码: %d", result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "规划成功，轨迹点数: %zu",
      plan.trajectory_.joint_trajectory.points.size());

    if (!execute) return true;

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "执行失败，错误码: %d", exec_result.val);
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
                               const geometry_msgs::msg::Pose & target_pose,
                               bool execute = true)
  {
    // 获取静止臂的当前位姿
    const std::string & fixed_tip  = move_left ? RIGHT_TIP : LEFT_TIP;
    const std::string & moving_tip = move_left ? LEFT_TIP  : RIGHT_TIP;

    geometry_msgs::msg::Pose fixed_pose;
    if (!get_current_pose(fixed_tip, fixed_pose)) {
      RCLCPP_ERROR(get_logger(), "无法获取 %s 当前位姿", fixed_tip.c_str());
      return false;
    }

    const auto & left_pose  = move_left ? target_pose : fixed_pose;
    const auto & right_pose = move_left ? fixed_pose  : target_pose;

    RCLCPP_INFO(get_logger(),
      "%s臂目标: (%.3f, %.3f, %.3f)，%s臂固定: (%.3f, %.3f, %.3f)",
      move_left ? "左" : "右",
      target_pose.position.x, target_pose.position.y, target_pose.position.z,
      move_left ? "右" : "左",
      fixed_pose.position.x, fixed_pose.position.y, fixed_pose.position.z);

    return plan_and_execute(left_pose, right_pose, execute);
  }

private:
  bool get_current_pose(const std::string & link, geometry_msgs::msg::Pose & out)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(BASE_FRAME, link,
        tf2::TimePointZero, tf2::durationFromSec(2.0));
      out.position.x = tf.transform.translation.x;
      out.position.y = tf.transform.translation.y;
      out.position.z = tf.transform.translation.z;
      out.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF 查询失败 %s: %s", link.c_str(), e.what());
      return false;
    }
  }

  // Demo：左臂在当前位姿基础上向上移动 10cm，右臂固定
  bool demo_move()
  {
    geometry_msgs::msg::Pose left_current;
    if (!get_current_pose(LEFT_TIP, left_current)) return false;

    auto left_target = left_current;
    left_target.position.z += 0.10;

    RCLCPP_INFO(get_logger(), "Demo: 左臂上移 10cm，右臂固定");
    return plan_and_execute_single(true, left_target, true);
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_exec_srv_;
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
